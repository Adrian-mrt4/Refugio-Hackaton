# REFUGIO Warehouse Challenge — Problem Overview & Key Concepts

## Problem Summary

The **REFUGIO Warehouse Challenge** is a **Lifelong Multi-Agent Pickup and Delivery (LMAPD)** problem on a discrete 50×50 grid. The scoring objective is to maximize the total number of item deliveries completed by 96 robots over 300 ticks across 3 evaluation seeds.

Two independent subproblems must be solved together:

1. **Layout Design** (`create_layout`): Place exactly 960 shelves on the grid such that robots can navigate efficiently without congestion.
2. **Robot Control Policy** (`act`): At each tick, decide the next action (UP, DOWN, LEFT, RIGHT, WAIT, PICKUP, DROP) for each robot independently, given only local observation.

---

## Grid and Constraints

| Property | Value |
|---|---|
| Grid size | 50×50 (walkable interior: x,y ∈ [1,50]) |
| Total shelf count | 960 (exactly) |
| Robots | 96 |
| Ticks per seed | 300 |
| Policy budget (total) | 180 seconds cumulative |
| Evaluation seeds | 3 (leaderboard uses hidden seeds) |

### Layout Constraints
- All 960 shelf coordinates must be unique
- Each shelf must have at least one orthogonally adjacent **empty pickup cell**
- All non-shelf walkable cells must form **one connected region** (connectivity requirement)
- Fixed base entry cells must remain open
- `create_layout()` must be **deterministic**

### Observation Available to `act()`
```python
Observation(
    tick: int,
    robot_id: int,
    position: tuple[int, int],
    base_position: tuple[int, int],
    target_item_position: tuple[int, int],
    carrying_item: bool,
    grid: tuple[tuple[CellType, ...], ...],
    all_robot_positions: Mapping[int, tuple[int, int]],
)
```

Each robot knows its own target and sees all other robot positions — but **not** other robots' targets. This partial observability is a key challenge.

---

## Problem Classification in Research Literature

This problem maps directly to **Lifelong Multi-Agent Path Finding (LMAPF)** with **pickup and delivery (PD)** tasks, also called **Multi-Agent Pickup and Delivery (MAPD)**:

- Each robot has a continuously assigned goal (target shelf → base)
- As soon as a delivery is completed, a new target is assigned
- Throughput (deliveries per tick) is the metric, not path length minimization

The combined layout + policy optimization is closely studied as **environment optimization for MAPF**, pioneered by Yulun Zhang and Jiaoyang Li at CMU's ARCS Lab.

---

## Scoring Formula

The raw score is the **sum of deliveries across all seeds**:

\[
\text{Score} = \sum_{s \in \text{seeds}} \text{Deliveries}(s, 300 \text{ ticks})
\]

Since the policy budget is shared across all seeds (180s total), per-tick computation must be extremely fast (< 0.2s per tick on average across all 96 robots).
