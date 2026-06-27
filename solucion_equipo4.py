"""REFUGIO warehouse policy submission — Equipo 4 (optimized).

The evaluator imports this single file and calls exactly two entry points:
`create_layout()` once per evaluation setup and `act(observation)` once per
robot per tick.

Improvements over baseline:
1. A* pathfinding with Manhattan heuristic (heapq) instead of greedy BFS
2. Persistent state between ticks — cached plans reused when still valid
3. Deadlock resolution with deterministic priority protocol (lower ID wins)
4. Smart pickup cell selection considering total trip distance (pickup + return)
5. Drop cell bypass — don't block drop cells when not dropping
6. Micro-optimizations: inlined hot paths, precomputed neighbor tables
"""

from __future__ import annotations

import heapq
from collections import deque

from warehouse_api import Action, CellType, Observation, Position


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ACTION_DELTAS: tuple[tuple[Action, Position], ...] = (
    (Action.UP, (0, -1)),
    (Action.RIGHT, (1, 0)),
    (Action.DOWN, (0, 1)),
    (Action.LEFT, (-1, 0)),
)
DELTA_BY_ACTION: dict[Action, Position] = dict(ACTION_DELTAS)
GRID_INDEX_CACHE_LIMIT = 8

# ---------------------------------------------------------------------------
# Persistent state between ticks (improvement #2)
# ---------------------------------------------------------------------------
# Each robot's cached plan: list of positions to follow.
_ROBOT_PLANS: dict[int, list[Position]] = {}
# How many consecutive ticks a robot has been stuck (waiting).
_STUCK_COUNTS: dict[int, int] = {}
# The goal each robot was pathfinding towards (to detect if target changed).
_ROBOT_GOALS: dict[int, Position] = {}


# ---------------------------------------------------------------------------
# Grid index — precomputed neighbors (same as baseline, kept for performance)
# ---------------------------------------------------------------------------
class GridIndex:
    """Static pathfinding structures cached per distinct grid layout."""

    __slots__ = ("grid", "passable", "neighbors")

    def __init__(self, grid: tuple[tuple[CellType, ...], ...]) -> None:
        self.grid = grid
        passable = frozenset(
            (x, y)
            for y, row in enumerate(grid)
            for x, cell in enumerate(row)
            if cell == CellType.EMPTY
        )
        self.passable = passable
        self.neighbors: dict[Position, tuple[tuple[Action, Position], ...]] = {
            position: tuple(
                (action, candidate)
                for action, (dx, dy) in ACTION_DELTAS
                if (candidate := (position[0] + dx, position[1] + dy)) in passable
            )
            for position in passable
        }


GRID_INDEXES: dict[int, GridIndex] = {}


def grid_index(grid: tuple[tuple[CellType, ...], ...]) -> GridIndex:
    key = id(grid)
    index = GRID_INDEXES.get(key)
    if index is None:
        index = next(
            (cached for cached in GRID_INDEXES.values() if cached.grid == grid),
            None,
        ) or GridIndex(grid)
        GRID_INDEXES[key] = index
        while len(GRID_INDEXES) > GRID_INDEX_CACHE_LIMIT:
            del GRID_INDEXES[next(iter(GRID_INDEXES))]
    return index


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
def create_layout() -> dict[str, object]:
    """Return the canonical rack-block layout from the starter kit."""
    shelves: list[list[int]] = []
    for x0 in range(3, 48, 4):
        for y0, y1 in ((3, 12), (15, 24), (27, 36), (39, 48)):
            for x in (x0, x0 + 1):
                for y in range(y0, y1 + 1):
                    shelves.append([x, y])
    return {"schema_version": 1, "shelves": shelves}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def are_adjacent(a: Position, b: Position) -> bool:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def manhattan(a: Position, b: Position) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def drop_cell_for_base(base: Position) -> Position:
    x, y = base
    if y == 0:
        return (x, 1)
    if y == 51:
        return (x, 50)
    if x == 0:
        return (1, y)
    return (50, y)


# ---------------------------------------------------------------------------
# Main policy entry point
# ---------------------------------------------------------------------------
def act(observation: Observation) -> Action:
    """Return an optimized action for the current observation."""
    rid = observation.robot_id
    pos = observation.position

    # --- Immediate actions: PICKUP / DROP ---
    if not observation.carrying_item and are_adjacent(
        pos, observation.target_item_position
    ):
        # We arrived at pickup position — grab the item.
        _ROBOT_PLANS.pop(rid, None)
        _STUCK_COUNTS[rid] = 0
        _ROBOT_GOALS.pop(rid, None)
        return Action.PICKUP

    drop_cell = drop_cell_for_base(observation.base_position)
    if observation.carrying_item and pos == drop_cell:
        # We're at the drop cell with an item — deliver it.
        _ROBOT_PLANS.pop(rid, None)
        _STUCK_COUNTS[rid] = 0
        _ROBOT_GOALS.pop(rid, None)
        return Action.DROP

    # --- Build context ---
    index = grid_index(observation.grid)
    blocked = {
        position
        for robot_id, position in observation.all_robot_positions.items()
        if robot_id != rid
    }

    # --- Improvement #5: Drop cell bypass ---
    # If we're standing on our own drop cell but don't need to drop,
    # move away immediately to not block other robots near this base.
    if not observation.carrying_item and pos == drop_cell:
        departure = _base_departure_action(observation, index, blocked)
        if departure is not None:
            return departure

    # --- Determine goal ---
    if observation.carrying_item:
        current_goal = drop_cell
        goal_positions = (drop_cell,)
    else:
        # Improvement #4: Smart pickup cell selection
        goal_positions = _smart_pickup_positions(observation, index, blocked)
        current_goal = goal_positions[0] if goal_positions else None

    if not goal_positions:
        _STUCK_COUNTS[rid] = _STUCK_COUNTS.get(rid, 0) + 1
        return Action.WAIT

    # --- Improvement #2: Reuse cached plan if still valid ---
    plan = _ROBOT_PLANS.get(rid)
    previous_goal = _ROBOT_GOALS.get(rid)

    plan_valid = (
        plan is not None
        and len(plan) > 0
        and previous_goal == current_goal
        # Next step in plan must not be blocked by another robot
        and plan[0] not in blocked
        # We must still be at a position consistent with the plan
        # (the plan starts from our next step, so just check it's adjacent)
    )

    if plan_valid:
        # Follow the cached plan
        next_pos = plan[0]
        action = _action_for_move(pos, next_pos)
        if action is not None:
            _ROBOT_PLANS[rid] = plan[1:]
            _STUCK_COUNTS[rid] = 0
            return action

    # --- Need to recompute path ---
    # Improvement #3: Deadlock resolution
    stuck_count = _STUCK_COUNTS.get(rid, 0)

    if stuck_count > 2:
        # We've been stuck for a while. Use yielding protocol:
        # If we're stuck and a higher-priority (lower ID) robot is adjacent,
        # try to find an alternative path or step aside.
        action = _try_yield_or_reroute(
            observation, index, blocked, goal_positions, stuck_count
        )
        if action is not None:
            return action

    # Improvement #1: A* pathfinding
    action, new_plan = _astar_step(pos, goal_positions, index, blocked)

    if action == Action.WAIT:
        _STUCK_COUNTS[rid] = stuck_count + 1
    else:
        _STUCK_COUNTS[rid] = 0
        # Cache the plan for future ticks
        _ROBOT_PLANS[rid] = new_plan
        _ROBOT_GOALS[rid] = current_goal

    return action


# ---------------------------------------------------------------------------
# Improvement #1: A* pathfinding
# ---------------------------------------------------------------------------
def _astar_step(
    start: Position,
    goals: tuple[Position, ...],
    index: GridIndex,
    blocked: set[Position],
) -> tuple[Action, list[Position]]:
    """A* search returning (first_action, remaining_plan).

    Uses Manhattan distance as admissible heuristic. Returns (WAIT, []) if
    no path exists.
    """
    if start in goals:
        return Action.WAIT, []

    goal_set = frozenset(goals)

    # Priority queue: (f_score, tie_y, tie_x, position, first_action, path)
    # f_score = g_score + h (Manhattan to nearest goal)
    # Ties broken by (y, x) for determinism matching original behavior.
    counter = 0  # Tie-breaker for heap stability
    g_scores: dict[Position, int] = {start: 0}

    # Compute initial heuristic
    def heuristic(p: Position) -> int:
        return min(manhattan(p, g) for g in goals)

    h_start = heuristic(start)
    # heap items: (f, counter, pos, first_action, path)
    heap: list[tuple[int, int, Position, Action | None, list[Position]]] = [
        (h_start, counter, start, None, [])
    ]

    neighbors = index.neighbors

    while heap:
        f, _cnt, current, first_action, path = heapq.heappop(heap)

        # Check if we can skip this (already found a better route here)
        g = f - heuristic(current) if first_action is not None else 0
        if g > g_scores.get(current, float("inf")):
            continue

        current_neighbors = neighbors.get(current)
        if current_neighbors is None:
            current_neighbors = tuple(
                (action, candidate)
                for action, (dx, dy) in ACTION_DELTAS
                if (candidate := (current[0] + dx, current[1] + dy)) in index.passable
            )

        for action, candidate in current_neighbors:
            if candidate in blocked:
                continue

            new_g = g + 1
            if new_g >= g_scores.get(candidate, float("inf")):
                continue

            g_scores[candidate] = new_g
            new_first = action if first_action is None else first_action
            new_path = path + [candidate]

            if candidate in goal_set:
                return new_first, new_path[1:]  # Remaining plan after first step

            h = heuristic(candidate)
            counter += 1
            heapq.heappush(heap, (new_g + h, counter, candidate, new_first, new_path))

    return Action.WAIT, []


# ---------------------------------------------------------------------------
# Improvement #3: Deadlock resolution with yielding
# ---------------------------------------------------------------------------
def _try_yield_or_reroute(
    observation: Observation,
    index: GridIndex,
    blocked: set[Position],
    goals: tuple[Position, ...],
    stuck_count: int,
) -> Action | None:
    """Try to resolve a deadlock by yielding or finding an alternate route."""
    rid = observation.robot_id
    pos = observation.position

    # Strategy 1: If stuck for a long time (>5 ticks), try to move to any
    # adjacent free cell to break the deadlock pattern.
    if stuck_count > 5:
        # Try stepping aside — pick a free adjacent cell that's not blocked
        current_neighbors = index.neighbors.get(pos)
        if current_neighbors:
            for action, candidate in current_neighbors:
                if candidate not in blocked:
                    # Move to this cell to break the deadlock
                    _ROBOT_PLANS[rid] = []
                    _ROBOT_GOALS.pop(rid, None)
                    _STUCK_COUNTS[rid] = 0
                    return action

    # Strategy 2: Try rerouting around the blocking robot(s).
    # Expand the blocked set to include cells that are causing the jam.
    # Find the blocking robot(s) that are directly in our path.
    expanded_blocked = set(blocked)

    # Also temporarily block the cells of robots that are directly adjacent
    # and have lower IDs (they have priority, we should go around them).
    for other_id, other_pos in observation.all_robot_positions.items():
        if other_id == rid:
            continue
        if manhattan(pos, other_pos) <= 2 and other_id < rid:
            # This robot has priority and is nearby — also block their
            # likely next positions to avoid re-collision.
            expanded_blocked.add(other_pos)
            for _, neighbor in index.neighbors.get(other_pos, ()):
                if neighbor != pos:
                    # Don't block our own position
                    pass  # Keep expanded_blocked focused

    # Try A* with expanded blocked set
    action, plan = _astar_step(pos, goals, index, expanded_blocked)
    if action != Action.WAIT:
        _ROBOT_PLANS[rid] = plan
        _ROBOT_GOALS[rid] = goals[0] if goals else None
        _STUCK_COUNTS[rid] = 0
        return action

    return None


# ---------------------------------------------------------------------------
# Improvement #4: Smart pickup cell selection
# ---------------------------------------------------------------------------
def _smart_pickup_positions(
    observation: Observation,
    index: GridIndex,
    blocked: set[Position],
) -> tuple[Position, ...]:
    """Select pickup positions considering total trip distance.

    Instead of just minimizing distance from robot to pickup cell, we consider
    the total cost: distance(robot → pickup) + distance(pickup → base drop cell).
    """
    px, py = observation.position
    tx, ty = observation.target_item_position
    base_drop = drop_cell_for_base(observation.base_position)

    candidates = [
        candidate
        for candidate in ((tx + 1, ty), (tx, ty + 1), (tx - 1, ty), (tx, ty - 1))
        if candidate in index.passable and candidate not in blocked
    ]

    if not candidates:
        # All pickup cells blocked by robots — still return them without
        # the blocked filter so A* can target them (robots may move).
        candidates = [
            candidate
            for candidate in ((tx + 1, ty), (tx, ty + 1), (tx - 1, ty), (tx, ty - 1))
            if candidate in index.passable
        ]

    # Sort by total trip distance: (robot→pickup) + (pickup→base_drop)
    # This means we prefer a pickup cell that leads to a shorter overall journey.
    candidates.sort(
        key=lambda c: (
            manhattan((px, py), c) + manhattan(c, base_drop),
            # Tie-break by just robot→pickup distance
            manhattan((px, py), c),
            # Final tie-break for determinism
            c[1],
            c[0],
        )
    )
    return tuple(candidates)


# ---------------------------------------------------------------------------
# Improvement #5: Base departure action
# ---------------------------------------------------------------------------
def _base_departure_action(
    observation: Observation,
    index: GridIndex,
    blocked: set[Position],
) -> Action | None:
    """When sitting on the drop cell without needing to drop, move away fast."""
    drop_cell = drop_cell_for_base(observation.base_position)
    if observation.position != drop_cell:
        return None

    bx, by = observation.base_position
    px, py = observation.position

    # Move away from the base (opposite direction to base)
    candidates = (
        (Action.DOWN, by < py),
        (Action.UP, by > py),
        (Action.RIGHT, bx < px),
        (Action.LEFT, bx > px),
    )
    for action, matches in candidates:
        if not matches:
            continue
        dx, dy = DELTA_BY_ACTION[action]
        candidate = (px + dx, py + dy)
        if candidate in index.passable and candidate not in blocked:
            return action
        return None
    return None


# ---------------------------------------------------------------------------
# Utility: convert position delta to action
# ---------------------------------------------------------------------------
def _action_for_move(current: Position, target: Position) -> Action | None:
    """Convert a (current, target) pair to a directional action."""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    if dx == 1 and dy == 0:
        return Action.RIGHT
    if dx == -1 and dy == 0:
        return Action.LEFT
    if dx == 0 and dy == 1:
        return Action.DOWN
    if dx == 0 and dy == -1:
        return Action.UP
    return None
