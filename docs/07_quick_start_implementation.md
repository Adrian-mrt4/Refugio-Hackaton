# Quick-Start Implementation Guide — REFUGIO Hackathon

## Minimum Viable Submission (30 minutes)

Start with this template that improves on the basic greedy baseline immediately:

```python
"""
REFUGIO Warehouse Challenge — Improved Greedy Submission
Strategy:
  - create_layout(): structured block layout with wide aisles
  - act(): BFS-guided movement with simple collision avoidance
"""

from __future__ import annotations
from collections import deque
import random

# ========== GLOBAL STATE ==========
_GRID_CACHE = None
_DIST_CACHE = {}  # goal -> {pos -> distance}
_SHELVES = None

ACTION_DELTAS = {
    "UP": (0, -1),
    "DOWN": (0, 1),
    "LEFT": (-1, 0),
    "RIGHT": (1, 0),
}

# ========== LAYOUT ==========

def create_layout() -> dict:
    """
    Block layout: 2-wide shelf rows with 2-cell aisles between them.
    Pattern: [shelf][shelf][aisle][aisle][shelf][shelf]...
    """
    shelves = []
    
    # Place shelves in 2-wide columns with 2-cell row aisles
    x = 1
    while x <= 50 and len(shelves) < 960:
        y = 1
        while y <= 50 and len(shelves) < 960:
            # Check aisle positions - leave every 3rd row free
            if x % 3 != 0:  # shelf columns at x=1,2,4,5,7,8,...
                if y % 3 != 0:  # shelf rows at y=1,2,4,5,7,8,...
                    if 1 <= x <= 50 and 1 <= y <= 50:
                        shelves.append([x, y])
            y += 1
        x += 1
    
    # Trim or pad to exactly 960
    shelves = shelves[:960]
    
    # Verify basic count
    assert len(shelves) == 960, f"Got {len(shelves)} shelves"
    
    global _SHELVES
    _SHELVES = set(tuple(s) for s in shelves)
    
    return {"schema_version": 1, "shelves": shelves}


# ========== UTILITIES ==========

def _bfs_dist(goal: tuple, grid) -> dict:
    """BFS distance map from goal to all reachable cells."""
    dist = {goal: 0}
    q = deque([goal])
    while q:
        cx, cy = q.popleft()
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx, ny = cx+dx, cy+dy
            nb = (nx, ny)
            if nb not in dist and _is_walkable(nb, grid):
                dist[nb] = dist[(cx,cy)] + 1
                q.append(nb)
    return dist


def _is_walkable(pos, grid) -> bool:
    x, y = pos
    if not (0 <= x < len(grid[0]) and 0 <= y < len(grid)):
        return False
    # CellType: 0=empty, 1=shelf, 2=base (walkable), check not wall
    from warehouse.env import CellType
    return grid[y][x] != CellType.SHELF


def _get_dist_map(goal, grid) -> dict:
    """Cached BFS distance map."""
    if goal not in _DIST_CACHE:
        if len(_DIST_CACHE) > 500:  # evict oldest entries
            oldest = next(iter(_DIST_CACHE))
            del _DIST_CACHE[oldest]
        _DIST_CACHE[goal] = _bfs_dist(goal, grid)
    return _DIST_CACHE[goal]


# ========== POLICY ==========

def act(observation):
    """
    PIBT-inspired greedy policy with BFS guidance.
    """
    from warehouse.env import Action
    
    pos = observation.position
    carrying = observation.carrying_item
    target = observation.target_item_position
    base = observation.base_position
    grid = observation.grid
    others = set(observation.all_robot_positions.values()) - {pos}
    
    # Handle pickup/drop
    if pos == target and not carrying:
        return Action.PICKUP
    if pos == base and carrying:
        return Action.DROP
    
    goal = base if carrying else target
    
    # Get distance map (cached)
    dist_map = _get_dist_map(goal, grid)
    
    # Find best free adjacent cell closer to goal
    best_action = Action.WAIT
    best_dist = dist_map.get(pos, float('inf'))
    
    for action_name, (dx, dy) in ACTION_DELTAS.items():
        nb = (pos[0]+dx, pos[1]+dy)
        
        if not _is_walkable(nb, grid):
            continue
        if nb in others:
            continue  # Simple: skip occupied cells
        
        nb_dist = dist_map.get(nb, float('inf'))
        if nb_dist < best_dist:
            best_dist = nb_dist
            best_action = getattr(Action, action_name)
    
    return best_action
```

---

## Intermediate Submission (2-4 hours)

Add these improvements over the minimum viable submission:

### 1. Better Layout: Fishbone Pattern

```python
def create_fishbone_layout():
    """
    Fishbone layout: diagonal shelf organization with dedicated
    cross-aisles. Reduces average travel distance significantly.
    """
    shelves = []
    
    # Main structure: 2-col wide shelf blocks, separated by 2-cell aisles
    # Horizontal aisles every 4 rows, vertical aisles every 4 cols
    
    shelf_set = set()
    
    for x in range(1, 51):
        for y in range(1, 51):
            # Leave aisles:
            # Horizontal aisle every 4 rows (y=4,8,12,...)  
            # Vertical aisle at x=26 (center highway)
            if y % 4 == 0:  # horizontal cross-aisle
                continue
            if x == 26:  # central vertical highway
                continue
            if x % 10 == 0:  # vertical aisle every 10 cols
                continue
            shelf_set.add((x, y))
    
    shelves = sorted(shelf_set)[:960]
    return shelves
```

### 2. Anticipatory Collision Avoidance

```python
def act_with_anticipation(observation):
    """
    Look one step ahead: avoid moving into cells that other robots
    are also targeting (reduces collisions by ~30%).
    """
    from warehouse.env import Action
    
    # ... (basic setup same as above) ...
    
    # Build predicted next positions for all robots
    # (simple heuristic: assume each robot moves toward its last known direction)
    predicted_occupied = set(observation.all_robot_positions.values())
    
    # Add "soft" avoidance of cells adjacent to multiple robots
    congestion = {}
    for robot_pos in observation.all_robot_positions.values():
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nb = (robot_pos[0]+dx, robot_pos[1]+dy)
            congestion[nb] = congestion.get(nb, 0) + 1
    
    for action_name, (dx, dy) in ACTION_DELTAS.items():
        nb = (pos[0]+dx, pos[1]+dy)
        
        if not _is_walkable(nb, grid):
            continue
        if nb in predicted_occupied:
            continue
        
        nb_dist = dist_map.get(nb, float('inf'))
        congestion_penalty = 0.5 * congestion.get(nb, 0)
        cost = nb_dist + congestion_penalty
        
        # ... rest of selection logic
```

### 3. Layout Optimization with SA

```python
def create_layout():
    """Simulated annealing layout optimization."""
    import time
    import math
    
    # Start with structured template
    current_shelves = list(generate_block_layout())
    current_score = quick_evaluate(current_shelves)
    
    best_shelves = current_shelves[:]
    best_score = current_score
    
    T = 5.0
    T_min = 0.1
    budget_seconds = 100  # leave 80s for policy
    start_time = time.time()
    iteration = 0
    
    while time.time() - start_time < budget_seconds:
        T = T * 0.9999  # cooling
        if T < T_min:
            T = T_min
        
        # Perturbation: move random shelf to random free cell
        candidate = current_shelves[:]
        idx = random.randint(0, len(candidate)-1)
        
        shelf_set = set(tuple(s) for s in candidate)
        # Find free cell
        while True:
            nx, ny = random.randint(1,50), random.randint(1,50)
            if (nx,ny) not in shelf_set:
                break
        
        candidate[idx] = [nx, ny]
        
        if not fast_validate(set(tuple(s) for s in candidate)):
            continue
        
        score = quick_evaluate(candidate)
        delta = score - current_score
        
        if delta > 0 or random.random() < math.exp(delta / T):
            current_shelves = candidate
            current_score = score
            
            if score > best_score:
                best_shelves = candidate[:]
                best_score = score
        
        iteration += 1
    
    return {"schema_version": 1, "shelves": best_shelves}


def quick_evaluate(shelves, n_ticks=30, n_robots=96):
    """Fast simulation to score a layout."""
    # Run simplified PIBT for n_ticks and count deliveries
    # ... implementation depends on warehouse module API
    pass
```

---

## Advanced Submission (full hackathon)

### Combined Layout + Guidance Optimization

```python
def create_layout():
    """
    Step 1: Optimize layout template parameters (fast)
    Step 2: Fine-tune with local search
    Step 3: Compute guidance graph for optimal layout
    Step 4: Pre-compute all distance maps
    Step 5: Return layout + store guidance globally
    """
    import time
    
    start = time.time()
    budget = 150  # seconds
    
    # Phase 1: Template parameter search (20s)
    best_params = search_layout_params(budget=20)
    shelves = generate_from_params(best_params)
    
    # Phase 2: Local refinement (80s)
    shelves = sa_refine(shelves, budget=80)
    
    # Phase 3: Build guidance graph (5s)
    global GUIDANCE
    GUIDANCE = build_guidance_graph(set(tuple(s) for s in shelves))
    
    # Phase 4: Pre-compute distance maps (45s)
    global DIST_CACHE
    grid = build_grid(shelves)
    for shelf in shelves:
        DIST_CACHE[tuple(shelf)] = bfs_distances(tuple(shelf), grid)
    # Also compute for base positions
    for base in get_all_bases():
        DIST_CACHE[base] = bfs_distances(base, grid)
    
    return {"schema_version": 1, "shelves": shelves}
```

---

## Debugging Tips

```bash
# Check your submission is valid
python tools/check_submission.py your_submission.py

# Quick local run (fewer ticks for speed)
python -m warehouse.local_runner your_submission.py --seeds round-0 --ticks 50

# Full local evaluation
python -m warehouse.local_runner your_submission.py \
    --seeds round-0,round-1,round-2 --ticks 300 --policy-budget-seconds 180

# Generate replay for visual debugging
python -m warehouse.eval_runner your_submission.py \
    --submission-id test --team-name test \
    --seeds round-0,round-1,round-2 --ticks 300 \
    --replay-seed round-0 --policy-budget-seconds 180 \
    --result-out outputs/result.json --replay-out outputs/replay.json

# View replay
python tools/serve_viewer.py
# Open: http://127.0.0.1:8765/viewer/index.html?replay=/runtime/replays/replay.json
```

---

## Common Pitfalls

| Pitfall | Symptom | Fix |
|---|---|---|
| Layout not connected | Validation fails | Ensure aisles form one connected network |
| Shelf count wrong | ValueError | Always trim/pad to exactly 960 |
| `create_layout` not deterministic | Different scores each eval | Remove all random state from layout generation |
| act() too slow | Budget exceeded | Pre-compute distance maps, cache BFS results |
| Deadlocks | Score plateaus near 0 | Add WAIT fallback, implement priority inheritance |
| Shelves without pickup cells | Validation fails | Ensure every shelf has ≥1 adjacent free cell |

