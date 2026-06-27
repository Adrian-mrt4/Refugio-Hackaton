# Robot Policy Design for Lifelong MAPF — Reference Guide

## Policy Design Principles

The `act(observation)` function must return one action per robot per tick. Given the observation structure, the core policy challenge is:

1. **Path planning**: How to route a robot from its current position to its target
2. **Collision avoidance**: How to avoid other robots without centralized coordination
3. **Deadlock prevention**: How to handle situations where robots block each other
4. **Congestion management**: How to prevent traffic jams at popular shelf locations

---

## Baseline: Greedy A* + WAIT

The simplest baseline (as likely implemented in `examples/basic_greedy_submission.py`):
- Compute A* path from current position to target
- Execute next step; if blocked by another robot, WAIT

**Limitations**:
- Causes deadlocks in narrow corridors
- No global traffic awareness
- Ignores other robots' trajectories

---

## Level 1: PIBT-Inspired Policy

### Core Idea

PIBT (Priority Inheritance with Backtracking) is the gold-standard algorithm for per-tick lifelong MAPF. A simplified version can be implemented as the `act()` function:

**Priority Assignment**:
```python
# Robots carrying items (heading to base) get highest priority
# Robots heading to target get priority based on distance to target
# Robots that have been waiting longer get higher priority
priority = -distance_to_goal  # higher priority = more negative = shorter path
```

**Movement Logic** (per-tick, per-robot):
1. Compute preferred next cell (first step of A* toward goal)
2. Check if preferred cell is occupied by another robot
3. If occupied, check if the occupying robot can move away (priority inheritance)
4. If not, backtrack and try an alternative cell
5. If no alternative, WAIT

**Key PIBT properties to implement**:
- **Priority inheritance**: If robot A wants cell C occupied by robot B, B gets A's priority for this tick (allowing B to move out of the way)
- **Backtracking**: If movement leads to a dead configuration, undo and try alternatives
- **Guarantee**: In biconnected graphs, all robots reach their goal in finite time

### PIBT Pseudocode for `act()`

```python
def act(observation):
    """PIBT-inspired per-robot action."""
    pos = observation.position
    target = observation.target_item_position
    carrying = observation.carrying_item
    base = observation.base_position
    
    goal = base if carrying else target
    
    if pos == goal:
        return Action.PICKUP if not carrying else Action.DROP
    
    # Get preferred next cell (A* / BFS next step)
    next_cell = bfs_next_step(pos, goal, observation.grid)
    
    if next_cell is None:
        return Action.WAIT
    
    # Check occupancy
    occupied_by = get_robot_at(next_cell, observation.all_robot_positions)
    
    if occupied_by is None:
        return cell_to_action(pos, next_cell)
    
    # Cell occupied — WAIT (simplified; full PIBT would do priority inheritance)
    # Try alternative: move to any free adjacent cell that doesn't block goal path
    alt = find_alternative_move(pos, next_cell, goal, observation)
    if alt:
        return cell_to_action(pos, alt)
    
    return Action.WAIT
```

---

## Level 2: Guidance-Augmented Policy

### Concept

Pre-compute a **guidance graph** in `create_layout()` and use it in `act()` to bias movement choices:

```python
def create_layout():
    shelves = compute_optimal_layout()
    # Pre-compute guidance: preferred movement direction per cell
    guidance = compute_guidance_graph(shelves)
    # Store globally for use in act()
    global GUIDANCE
    GUIDANCE = guidance
    return {"schema_version": 1, "shelves": shelves}

def act(observation):
    pos = observation.position
    goal = get_goal(observation)
    
    # Combine A* heuristic with guidance preference
    preferred_next = guidance_aware_next_step(pos, goal, GUIDANCE)
    ...
```

**Guidance graph implementation**:
- Assign each cell a preferred outgoing direction (e.g., in a horizontal aisle, prefer RIGHT)
- This implements **virtual one-way traffic patterns** without hard constraints
- Robots following guidance naturally avoid head-on collisions in corridors

### Highway Pattern (Simple Guidance)

A practical approximation:
- **Even rows**: prefer RIGHT movement
- **Odd rows**: prefer LEFT movement  
- **Vertical connectors**: prefer DOWN or UP based on column parity
- This creates a **circulation pattern** that prevents most deadlocks

```python
def highway_guidance(x, y):
    """Returns preferred direction for cell (x,y)."""
    if y % 2 == 0:   # horizontal aisle, even row
        return Action.RIGHT
    else:             # horizontal aisle, odd row
        return Action.LEFT
    # Vertical connectors handled separately
```

---

## Level 3: Learning-Based Policy

### Imitation Learning from PIBT

Train a neural network to imitate PIBT decisions, then use it as a fast policy:
- PIBT generates ground-truth action labels for training states
- NN learns the mapping: (local observation window) → action
- At inference, NN runs in microseconds vs. PIBT's O(n) per tick

**Reference**: SILLM (Scalable Imitation Learning for LMAPF), arXiv:2410.21415  
- Introduces communication module for inter-robot coordination
- Handles up to 10,000 agents within <1 second
- Requires offline training (not viable for hackathon unless pre-trained model available)

### Reinforcement Learning Priority Assignment

**Reference**: RL-RH-PP (arXiv:2603.23838)
- RL assigns dynamic priorities to agents each planning step
- Attention-based network processes local agent neighborhoods
- Formulated as POMDP: state = agent positions + goals, action = priority ordering
- Trained with warehouse simulation — can be adapted to REFUGIO simulator

---

## Deadlock Detection and Resolution

### Common Deadlock Patterns

1. **Face-to-face**: Two robots moving toward each other in a 1-wide corridor
2. **Circular**: Robot A waits for B, B waits for C, C waits for A
3. **Bottleneck**: Many robots converging on one shelf/aisle

### Detection
```python
def detect_deadlock(positions, intended_moves):
    """Detect circular wait cycles."""
    wait_for = {}
    for robot_id, move in intended_moves.items():
        target_cell = get_cell_after_move(positions[robot_id], move)
        blocker = get_robot_at(target_cell, positions)
        if blocker:
            wait_for[robot_id] = blocker
    # Find cycles in wait_for graph using DFS
    return find_cycles(wait_for)
```

### Resolution Strategies
1. **Priority breaking**: Assign random tie-breaking to break symmetric deadlocks
2. **Yield rule**: Lower-priority robot moves to any free adjacent cell, even temporarily away from goal
3. **Temporal avoidance**: Use space-time A* to plan paths that avoid predicted robot positions

---

## Space-Time A* for Collision-Free Planning

For pre-computing conflict-free paths (useful if time budget allows):

```python
def space_time_astar(start, goal, grid, reservations, t_start=0, t_max=50):
    """
    A* in space-time: state = (x, y, t)
    reservations: set of (x, y, t) already occupied by other robots
    """
    open_set = [(0, t_start, start)]
    g_cost = {(start, t_start): 0}
    
    while open_set:
        f, t, pos = heappop(open_set)
        
        if pos == goal:
            return reconstruct_path(...)
        
        for next_pos in neighbors(pos, grid):
            next_t = t + 1
            if (next_pos, next_t) in reservations:
                continue  # Space-time collision
            new_g = g_cost[(pos, t)] + 1
            if (next_pos, next_t) not in g_cost or new_g < g_cost[(next_pos, next_t)]:
                g_cost[(next_pos, next_t)] = new_g
                f = new_g + manhattan(next_pos, goal)
                heappush(open_set, (f, next_t, next_pos))
    
    return None  # No path found within t_max
```

---

## Distance Precomputation (Critical Optimization)

Pre-compute BFS distance maps from every shelf and base in `create_layout()`:

```python
def bfs_distances(source, grid):
    """Returns dict: (x,y) -> distance from source."""
    dist = {source: 0}
    queue = deque([source])
    while queue:
        pos = queue.popleft()
        for nb in orthogonal_neighbors(pos, grid):
            if nb not in dist:
                dist[nb] = dist[pos] + 1
                queue.append(nb)
    return dist

# Pre-compute during setup (in create_layout or first act() call)
DIST_TO_BASE = {}
for robot_id, base_pos in enumerate(all_bases):
    DIST_TO_BASE[base_pos] = bfs_distances(base_pos, grid)

DIST_TO_SHELF = {}
for shelf_pos in all_shelves:
    DIST_TO_SHELF[shelf_pos] = bfs_distances(shelf_pos, grid)
```

This makes per-tick navigation lookups O(1) instead of O(n log n) for each A* call.

---

## Policy Budget Management

The 180-second budget is cumulative across all seeds and includes both setup and policy execution:

**Budget allocation strategy**:
- Reserve ~120s for `create_layout()` layout optimization (run once)
- Reserve ~60s for 300 ticks × 96 robots = 28,800 `act()` calls  
- Target: **< 2ms per `act()` call**

For 2ms/call: avoid full A* on every call; use pre-computed distance maps + PIBT heuristics.

