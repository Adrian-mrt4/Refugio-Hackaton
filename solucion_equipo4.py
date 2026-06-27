"""REFUGIO warehouse policy submission.

The evaluator imports this single file and calls exactly two entry points:
`create_layout()` once per evaluation setup and `act(observation)` once per
robot per tick.
"""

from __future__ import annotations

from collections import deque

from warehouse_api import Action, CellType, Observation, Position


ACTION_DELTAS: tuple[tuple[Action, Position], ...] = (
    (Action.UP, (0, -1)),
    (Action.RIGHT, (1, 0)),
    (Action.DOWN, (0, 1)),
    (Action.LEFT, (-1, 0)),
)
DELTA_BY_ACTION: dict[Action, Position] = dict(ACTION_DELTAS)
GRID_INDEX_CACHE_LIMIT = 8


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


def create_layout() -> dict[str, object]:
    """Return the canonical rack-block layout from the starter kit."""
    shelves: list[list[int]] = []
    for x0 in range(3, 48, 4):
        for y0, y1 in ((3, 12), (15, 24), (27, 36), (39, 48)):
            for x in (x0, x0 + 1):
                for y in range(y0, y1 + 1):
                    shelves.append([x, y])
    return {"schema_version": 1, "shelves": shelves}


def act(observation: Observation) -> Action:
    """Return a deterministic greedy BFS action for the current observation."""
    if not observation.carrying_item and are_adjacent(
        observation.position,
        observation.target_item_position,
    ):
        return Action.PICKUP

    drop_cell = drop_cell_for_base(observation.base_position)
    if observation.carrying_item and observation.position == drop_cell:
        return Action.DROP

    index = grid_index(observation.grid)
    blocked = {
        position
        for robot_id, position in observation.all_robot_positions.items()
        if robot_id != observation.robot_id
    }

    if not observation.carrying_item:
        departure_action = base_departure_action(observation, index, blocked)
        if departure_action is not None:
            return departure_action

    goals = (
        (drop_cell,)
        if observation.carrying_item
        else pickup_positions(observation, index, blocked)
    )
    if not goals:
        return Action.WAIT

    return shortest_path_step(observation.position, goals, index, blocked)


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


def are_adjacent(a: Position, b: Position) -> bool:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def drop_cell_for_base(base: Position) -> Position:
    x, y = base
    if y == 0:
        return (x, 1)
    if y == 51:
        return (x, 50)
    if x == 0:
        return (1, y)
    return (50, y)


def base_departure_action(
    observation: Observation,
    index: GridIndex,
    blocked: set[Position],
) -> Action | None:
    drop_cell = drop_cell_for_base(observation.base_position)
    if observation.position != drop_cell:
        return None

    bx, by = observation.base_position
    px, py = observation.position
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


def pickup_positions(
    observation: Observation,
    index: GridIndex,
    blocked: set[Position],
) -> tuple[Position, ...]:
    px, py = observation.position
    tx, ty = observation.target_item_position
    candidates = [
        candidate
        for candidate in ((tx + 1, ty), (tx, ty + 1), (tx - 1, ty), (tx, ty - 1))
        if candidate in index.passable and candidate not in blocked
    ]
    candidates.sort(
        key=lambda candidate: (
            abs(px - candidate[0]) + abs(py - candidate[1]),
            candidate[1],
            candidate[0],
        )
    )
    return tuple(candidates)


def shortest_path_step(
    start: Position,
    goals: tuple[Position, ...],
    index: GridIndex,
    blocked: set[Position],
) -> Action:
    """Find the first action of a shortest path with deterministic ties."""
    if start in goals:
        return Action.WAIT

    goal_set = frozenset(goals)
    visited = {start}
    queue: deque[tuple[Position, Action]] = deque()

    current_neighbors = index.neighbors.get(start)
    if current_neighbors is None:
        current_neighbors = tuple(
            (action, candidate)
            for action, (dx, dy) in ACTION_DELTAS
            if (candidate := (start[0] + dx, start[1] + dy)) in index.passable
        )

    current_first_action: Action | None = None
    expansions: list[tuple[int, int, int, Action, Position]] = []
    while True:
        expansions.clear()
        for action, candidate in current_neighbors:
            if candidate in visited or candidate in blocked:
                continue
            cx, cy = candidate
            best_distance = min(
                abs(cx - gx) + abs(cy - gy)
                for gx, gy in goals
            )
            expansions.append((best_distance, cy, cx, action, candidate))

        expansions.sort()
        for _distance, _cy, _cx, action, candidate in expansions:
            visited.add(candidate)
            first_action = action if current_first_action is None else current_first_action
            if candidate in goal_set:
                return first_action
            queue.append((candidate, first_action))

        if not queue:
            return Action.WAIT
        current, current_first_action = queue.popleft()
        current_neighbors = index.neighbors[current]
