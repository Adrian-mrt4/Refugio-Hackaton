# Optimization Strategies for REFUGIO — Hackathon Playbook

## Two-Phase Optimization Framework

The REFUGIO problem naturally decomposes into:

```
Phase 1 (create_layout): OFFLINE OPTIMIZATION
  - Spend most of the setup budget optimizing shelf placement
  - Pre-compute all necessary data structures (distance maps, guidance)
  - Budget: up to ~120-150 seconds

Phase 2 (act): ONLINE POLICY EXECUTION
  - Must run in real-time: <2ms per act() call
  - Uses pre-computed data from Phase 1
  - Budget: remaining time from 180s total
```

---

## Layout Optimization Algorithms

### Option A: CMA-ES (Recommended for hackathon)

Covariance Matrix Adaptation Evolution Strategy — a black-box optimizer well-suited for REFUGIO:

```python
import cma

def evaluate_layout(shelf_positions, n_robots=96, n_ticks=100):
    """Simulate layout and return negative throughput (minimize)."""
    layout = shelves_to_layout(shelf_positions)
    if not is_valid_layout(layout):
        return 0  # penalty
    throughput = simulate_pibt(layout, n_robots, n_ticks)
    return -throughput  # CMA-ES minimizes

# CMA-ES setup
x0 = flatten(canonical_layout_positions)  # start from structured layout
sigma0 = 2.0  # initial step size (in grid cells)
opts = {'maxiter': 100, 'tolx': 0.5, 'bounds': [1, 50]}
es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

while not es.stop():
    solutions = es.ask()
    fitnesses = [evaluate_layout(s) for s in solutions]
    es.tell(solutions, fitnesses)

best_layout = reshape(es.result.xbest)
```

**Key consideration**: The layout space has 960×2 = 1,920 continuous dimensions — CMA-ES may struggle. Consider fixing a layout template and only optimizing a few parameters (aisle widths, block sizes).

### Option B: Parametric Layout Templates

Define layout families with a small number of parameters:

```python
def block_layout(n_rows, n_cols, aisle_width, shelf_height):
    """
    Generate a block-style layout parameterized by:
    - n_rows: number of shelf rows
    - n_cols: number of shelf columns  
    - aisle_width: cells between shelf rows
    - shelf_height: cells per shelf block
    """
    shelves = []
    y = 1
    while y <= 50 and len(shelves) < 960:
        x = 1
        while x <= 50 and len(shelves) < 960:
            # Place a shelf block
            for dy in range(shelf_height):
                for dx in range(1):  # 1 column wide
                    shelves.append([x + dx, y + dy])
            x += 1 + aisle_width
        y += shelf_height + aisle_width
    return shelves[:960]
```

Then optimize over (n_rows, n_cols, aisle_width, shelf_height) — just 4 parameters!

### Option C: Simulated Annealing Local Search

Start from a valid layout and perform local perturbations:

```python
import math, random

def sa_optimize_layout(initial_shelves, n_iters=5000, T_start=10, T_end=0.1):
    current = initial_shelves[:]
    current_score = evaluate_layout(current)
    best = current[:]
    best_score = current_score
    
    for i in range(n_iters):
        T = T_start * (T_end / T_start) ** (i / n_iters)
        
        # Perturbation: move one shelf to a random free cell
        candidate = current[:]
        idx = random.randint(0, len(candidate) - 1)
        new_pos = random_free_cell(candidate)
        candidate[idx] = new_pos
        
        if not is_valid(candidate):
            continue
        
        candidate_score = evaluate_layout(candidate)
        delta = candidate_score - current_score
        
        if delta > 0 or random.random() < math.exp(delta / T):
            current = candidate
            current_score = candidate_score
            if current_score > best_score:
                best = current[:]
                best_score = current_score
    
    return best
```

---

## Fast Layout Validator

Validation must be fast (called thousands of times during optimization):

```python
from collections import deque

def fast_validate(shelves_set, width=52, height=52):
    """Returns True if layout is valid. shelves_set: set of (x,y) tuples."""
    
    # Check count
    if len(shelves_set) != 960:
        return False
    
    # Check bounds
    for x, y in shelves_set:
        if not (1 <= x <= 50 and 1 <= y <= 50):
            return False
    
    # Check each shelf has at least one adjacent free cell
    for x, y in shelves_set:
        adj_free = any(
            (x+dx, y+dy) not in shelves_set
            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]
        )
        if not adj_free:
            return False
    
    # Check connectivity of walkable region
    walkable = set()
    for x in range(0, width):
        for y in range(0, height):
            if (x, y) not in shelves_set:
                walkable.add((x, y))
    
    if not walkable:
        return False
    
    start = next(iter(walkable))
    visited = {start}
    queue = deque([start])
    while queue:
        cx, cy = queue.popleft()
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nb = (cx+dx, cy+dy)
            if nb in walkable and nb not in visited:
                visited.add(nb)
                queue.append(nb)
    
    return len(visited) == len(walkable)
```

---

## Fast PIBT Simulator for Layout Evaluation

Build a minimal simulator to score layouts quickly:

```python
def simulate_pibt_throughput(shelves, n_robots=96, n_ticks=100, n_trials=3):
    """Returns average deliveries per tick across n_trials."""
    total_deliveries = 0
    
    for _ in range(n_trials):
        # Initialize robots at base positions
        robots = init_robots(n_robots)
        deliveries = 0
        
        for tick in range(n_ticks):
            # Assign priorities (higher = closer to goal)
            priorities = assign_priorities(robots)
            
            # PIBT step
            new_positions = pibt_step(robots, priorities, shelves)
            
            # Process pickups/drops
            for r in robots:
                if r.at_target and not r.carrying:
                    r.carrying = True
                elif r.at_base and r.carrying:
                    r.carrying = False
                    deliveries += 1
                    r.target = random.choice(shelves)
            
            robots = new_positions
        
        total_deliveries += deliveries
    
    return total_deliveries / (n_trials * n_ticks)  # throughput
```

---

## Guidance Graph Construction

Build a guidance graph to reduce congestion — can be computed once in `create_layout()`:

```python
def build_guidance_graph(shelves_set, width=52, height=52):
    """
    Build circulation-based guidance edge weights.
    Returns: dict (x,y) -> dict direction -> weight
    where lower weight = preferred direction
    """
    guidance = {}
    
    for x in range(1, 51):
        for y in range(1, 51):
            if (x, y) in shelves_set:
                continue
            
            guidance[(x, y)] = {}
            
            # Even rows: prefer RIGHT (→)
            # Odd rows: prefer LEFT (←)
            # Even cols: prefer DOWN (↓)  
            # Odd cols: prefer UP (↑)
            if y % 2 == 0:
                guidance[(x,y)][(1,0)] = 0.0   # RIGHT: low cost
                guidance[(x,y)][(-1,0)] = 2.0  # LEFT: high cost
            else:
                guidance[(x,y)][(-1,0)] = 0.0  # LEFT: low cost
                guidance[(x,y)][(1,0)] = 2.0   # RIGHT: high cost
            
            if x % 2 == 0:
                guidance[(x,y)][(0,1)] = 0.0   # DOWN: low cost
                guidance[(x,y)][(0,-1)] = 2.0  # UP: high cost
            else:
                guidance[(x,y)][(0,-1)] = 0.0  # UP: low cost
                guidance[(x,y)][(0,1)] = 2.0   # DOWN: high cost
    
    return guidance
```

---

## act() Performance Template

```python
# Global state (initialized in create_layout or first act() call)
SHELVES = None
GUIDANCE = None
DIST_MAPS = {}  # base_pos -> BFS distance map
SHELF_DIST = {}  # shelf_pos -> BFS distance map

def act(observation):
    global SHELVES, GUIDANCE, DIST_MAPS
    
    pos = observation.position
    carrying = observation.carrying_item
    target = observation.target_item_position
    base = observation.base_position
    goal = base if carrying else target
    
    # Handle pickup/drop
    if pos == target and not carrying:
        return Action.PICKUP
    if pos == base and carrying:
        return Action.DROP
    
    # Get BFS next step toward goal (O(1) lookup)
    if goal in DIST_MAPS:
        dist_map = DIST_MAPS[goal]
    else:
        # Fallback: compute on-the-fly (expensive, avoid)
        dist_map = bfs_distances(goal, observation.grid)
        DIST_MAPS[goal] = dist_map
    
    # Find best move: minimize dist_map[next_pos] + guidance_cost
    best_action = Action.WAIT
    best_cost = float('inf')
    
    for action, (dx, dy) in ACTION_DELTAS.items():
        next_pos = (pos[0]+dx, pos[1]+dy)
        
        if not is_walkable(next_pos, observation.grid):
            continue
        if next_pos in observation.all_robot_positions.values():
            continue  # Occupied — skip (PIBT would do inheritance here)
        
        dist = dist_map.get(next_pos, float('inf'))
        guidance_cost = GUIDANCE.get(pos, {}).get((dx,dy), 1.0)
        cost = dist + 0.3 * guidance_cost
        
        if cost < best_cost:
            best_cost = cost
            best_action = action
    
    return best_action
```

---

## Expected Throughput Benchmarks

Based on published MAPF warehouse results (50×50 map, ~38% shelf density):

| Policy Type | Expected Deliveries/Tick | Notes |
|---|---|---|
| Random walk | ~0.05 | Baseline |
| Greedy A* (no collision handling) | ~0.3-0.5 | Many deadlocks |
| Greedy A* + WAIT | ~0.8-1.2 | Deadlocks resolved by waiting |
| PIBT | ~2.0-3.5 | State-of-the-art rule-based |
| PIBT + good layout | ~3.5-5.0 | Layout optimization benefit |
| PIBT + GGO | ~4.0-6.0 | Guidance graph benefit |
| RHCR | ~4.5-7.0 | Best search-based (too slow?) |

*Note: These are rough estimates based on published papers for similar configurations. Actual REFUGIO values depend on the specific simulation implementation.*

