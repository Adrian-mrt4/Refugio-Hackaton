# Warehouse Layout Optimization for MAPF — Reference Guide

## Why Layout Matters

Even with state-of-the-art MAPF algorithms, **human-designed warehouse layouts** often create bottlenecks that limit throughput, especially at high robot densities. Research by Zhang et al. (IJCAI 2023) demonstrated that optimized layouts can:

- Reduce traffic congestion and improve throughput significantly
- **Double the number of robots** that can operate efficiently in some configurations
- Generate layouts tailored to specific diversity or connectivity requirements

The `create_layout()` function in REFUGIO is precisely this layout optimization problem.

---

## Key Papers

### 1. Multi-Robot Coordination and Layout Design for Automated Warehousing ⭐ Core Paper

**Authors**: Yulun Zhang, Matthew Fontaine, Varun Bhatt, Stefanos Nikolaidis, Jiaoyang Li  
**Venue**: IJCAI 2023  
**arXiv**: https://arxiv.org/abs/2305.06436  
**PDF**: https://arxiv.org/pdf/2305.06436.pdf

This is the **most directly relevant paper** to `create_layout()`. Key contributions:
- Frames warehouse layout as an **optimization problem over shelf placement**
- Uses **Quality-Diversity (QD) algorithms** (specifically CMA-ME, an extension of CMA-ES) to search the layout space
- Evaluates layouts by simulating MAPF throughput — exactly what REFUGIO does
- Shows that optimized layouts outperform standard "fishbone", "random", and "block" layouts

**Layout Design Rules in the Paper** (matching REFUGIO constraints):
- Shelves must have adjacent pickup cells (matches REFUGIO requirement)
- All walkable cells must be connected (matches REFUGIO requirement)
- Optimizes for throughput under a fixed MAPF algorithm

---

### 2. Guidance Graph Optimization for Lifelong MAPF ⭐ Core Paper

**Authors**: Yulun Zhang, He Jiang, Varun Bhatt, Stefanos Nikolaidis, Jiaoyang Li  
**Venue**: IJCAI 2024  
**arXiv**: https://arxiv.org/abs/2402.01446

Introduces the **Guidance Graph** — a directed weighted graph that biases agent movement to reduce congestion:
- Edge weights represent preferred movement directions (e.g., one-way aisles)
- Two GGO algorithms: (1) direct CMA-ES edge weight optimization, (2) a Parameterized Iterative Update (PIU) neural model
- A guidance graph trained on one map **transfers** to larger maps of similar structure
- PIBT + GGO **matches RHCR throughput** at a fraction of the compute cost

**Practical insight for REFUGIO**: The guidance graph can be pre-computed in `create_layout()` and used to bias the `act()` function's movement decisions, essentially implementing one-way aisle traffic patterns.

---

### 3. Online Guidance Graph Optimization for Lifelong MAPF

**Authors**: Hongzhi Zang, Yulun Zhang, He Jiang, Zhe Chen, Daniel Harabor, Peter Stuckey, Jiaoyang Li  
**Venue**: AAAI 2025  
**arXiv**: https://arxiv.org/abs/2411.16506

Extends GGO to **dynamic, online adaptation** based on real-time traffic:
- Guidance graph edge weights updated at runtime based on observed congestion
- Two pipelines for incorporating guidance into PIBT
- Outperforms static guidance and human-designed policies (e.g., highway patterns)

---

### 4. Scaling Lifelong MAPF to More Realistic Settings

**Authors**: He Jiang, Yulun Zhang, Rishi Veerapaneni, Jiaoyang Li  
**arXiv**: https://arxiv.org/abs/2404.16162

Research challenges paper identifying key gaps:
- Planning under tight time budgets (directly relevant to 180s REFUGIO constraint)
- Handling heterogeneous task distributions
- Bridging gap between benchmark and real warehouse performance

---

### 5. Novel Framework for Automated Warehouse Layout Generation

**Authors**: Various  
**Venue**: Frontiers in AI 2024  
**URL**: https://www.frontiersin.org/journals/artificial-intelligence/articles/10.3389/frai.2024.1465186/full  
**PMC**: https://pmc.ncbi.nlm.nih.gov/articles/PMC11518846/

AI-driven layout generation using **constrained beam search**:
- Enforces feasibility constraints (access points, connectivity)
- Optimizes across warehouse dimensions, door placements
- Produces multiple valid layout configurations for evaluation

---

## Layout Design Strategies

### Strategy A: Structured Aisle Patterns

Classic approaches from warehouse operations research:
- **Fishbone layout**: Diagonal aisles reduce average travel distance
- **Block layout**: Rectangular shelf blocks with cross-aisles
- **Random layout**: Baseline (surprisingly competitive in some MAPF studies)

### Strategy B: Topology-Aware Design

Design the grid topology to enable PIBT's biconnectivity guarantee:
- Ensure every cell is part of a simple cycle of length ≥ 3
- Avoid dead-ends (single-access shelf locations create deadlock risk)
- Use "highway" patterns: dedicate some aisles to one-directional flow

### Strategy C: Optimization-Based Search (Recommended)

Use evolutionary/quality-diversity optimization:

1. **CMA-ES** (Covariance Matrix Adaptation Evolution Strategy):
   - Black-box optimization over shelf coordinate configurations
   - Evaluates layouts by running a fast PIBT simulation
   - Naturally handles the 960-shelf placement as a high-dimensional optimization

2. **Quality-Diversity (QD / CMA-ME)**:
   - Maintains a *archive* of diverse high-quality layouts
   - Explores different layout "styles" (aisle density, shelf clustering, etc.)
   - Can generate multiple candidate layouts for ensemble evaluation

3. **Simulated Annealing / Local Search**:
   - Start from a structured layout (fishbone or block)
   - Perturb: move a shelf, swap two shelves, add/remove shelf from cluster
   - Accept improvements; occasionally accept worse solutions (SA)
   - Fast to implement; may suffice for hackathon time constraints

---

## Connectivity Validation

The REFUGIO README requires all non-shelf walkable cells to form one connected region. Validation algorithm:

```python
from collections import deque

def is_connected(grid, width=52, height=52):
    """BFS to check walkable connectivity."""
    walkable = {(x, y) for x in range(width) for y in range(height)
                if grid[y][x] != CellType.SHELF}
    if not walkable:
        return False
    start = next(iter(walkable))
    visited = {start}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nb = (x+dx, y+dy)
            if nb in walkable and nb not in visited:
                visited.add(nb)
                queue.append(nb)
    return len(visited) == len(walkable)
```

---

## Practical Layout Recommendations for REFUGIO

Given 50×50 grid, 960 shelves (~38.4% density), 96 robots:

1. **Start with a structured template**: Place shelves in rectangular blocks of 2×N or 3×N with aisles between them. This ensures connectivity by construction.
2. **Reserve highway corridors**: Leave 2-3 cell wide corridors along edges and center for fast robot routing.
3. **Avoid isolated shelf clusters**: All regions must be reachable.
4. **Minimize dead-end aisles**: Single-cell-wide dead ends near shelves cause PIBT backtracking overhead.
5. **Pre-compute A* distance maps**: In the setup phase, compute BFS/Dijkstra distances from every cell to every shelf — use these in `act()` for heuristic guidance without re-computing each tick.

