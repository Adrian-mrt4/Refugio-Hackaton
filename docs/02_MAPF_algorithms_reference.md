# Multi-Agent Path Finding (MAPF) Algorithms — Reference Guide

## What is MAPF?

Multi-Agent Path Finding (MAPF) is the problem of computing collision-free paths for a set of agents in a shared environment, each moving from a start location to a goal. In its **lifelong** variant (LMAPF), agents are continuously reassigned new goals upon delivery completion, making throughput — not path length — the primary metric.

---

## Algorithm Taxonomy

### 1. Priority Inheritance with Backtracking (PIBT) ⭐ Most Relevant

**Paper**: Okumura et al., IJCAI 2019 / AIJ 2022  
**arXiv**: https://arxiv.org/abs/1901.11282  
**Code**: https://kei18.github.io/pibt2/

PIBT is the **most relevant algorithm** for REFUGIO because:
- Operates in **near-linear time per timestep** w.r.t. number of agents
- Designed for **iterative/lifelong MAPF** (exactly one tick = one PIBT step)
- Works in decentralized fashion — each agent uses only local coordination
- **No pre-computed paths needed** — decisions are made per tick
- Completeness guaranteed in biconnected graphs (connected grids satisfy this)

**How it works:**
- Each agent gets a unique priority each tick
- High-priority agents move first; lower-priority agents inherit priorities and backtrack to avoid blocking
- Priority is updated to push agents toward their goals faster

**Key limitation**: Rule-based, can be outperformed when combined with guidance (see GGO below).

---

### 2. Rolling-Horizon Collision Resolution (RHCR)

**Paper**: Li et al., AAAI 2021  
**arXiv**: https://arxiv.org/abs/2005.07371

RHCR decomposes lifelong MAPF into a sequence of windowed MAPF instances:
- Plans collision-free paths within a **bounded time horizon** (window)
- Ignores collisions beyond the window
- Uses a MAPF solver (e.g., ECBS, LNS) within each window
- Demonstrated for up to **1,000 agents** in warehouse settings
- High quality but **computationally expensive** — may not fit 180s budget for 96 robots × 300 ticks

---

### 3. LaCAM / LaCAM* (Scalable Search-Based)

**Paper**: Okumura, IJCAI 2023 / AAMAS 2024  
**arXiv**: https://arxiv.org/abs/2305.03632  
**Code**: https://kei18.github.io/lacam2/

LaCAM uses **lazy successor generation** to solve MAPF for up to thousands of agents:
- Sub-optimal but highly scalable
- LaCAM* eventually converges to optimal solutions
- Solved 99% of MAPF benchmark instances with up to 1,000 agents within 10 seconds
- **Real-Time LaCAM** (arXiv:2504.06091) extends it with per-tick millisecond cutoffs and completeness guarantees — directly applicable to REFUGIO's tick-based model

---

### 4. MAPF-LNS2 (Large Neighborhood Search)

**Paper**: Li et al., AAAI 2022  
**arXiv**: https://arxiv.org/abs/2111.11357  
**Code**: https://github.com/Jiaoyang-Li/MAPF-LNS2

An **anytime algorithm** based on Large Neighborhood Search:
- Starts with a fast initial solution, then iteratively improves it
- Repairs collision-free paths by destroying and replanning subsets of agents
- State-of-the-art for **one-shot MAPF** quality
- Less suited for per-tick lifelong planning but useful for pre-computing paths

---

### 5. Priority-Based Search (PBS) and Greedy PBS

**Paper**: Ma et al. / GPBS: arXiv  
**Reference**: https://cris.biu.ac.il/en/publications/greedy-priority-based-search-for-suboptimal-multi-agent-path-find/

PBS finds paths by assigning priorities and replanning:
- Greedy PBS (GPBS) uses greedy strategies to minimize collisions
- Includes partial expansions, target reasoning, induced constraints, soft restarts
- Best sub-optimal algorithm for **small maps with dense obstacles** — relevant to REFUGIO's dense 50×50 grid

---

### 6. RL-guided Prioritized Planning (RL-RH-PP)

**Paper**: arXiv:2603.23838  
**URL**: https://arxiv.org/abs/2603.23838

Combines reinforcement learning with Rolling-Horizon Prioritized Planning:
- RL policy (attention-based neural network) assigns dynamic agent priorities
- Outperforms pure search-based methods in throughput for lifelong MAPF
- Generalizes across agent densities and warehouse layouts

---

## Algorithm Comparison Table

| Algorithm | Type | Per-tick Cost | Lifelong? | Throughput Quality | Complexity |
|---|---|---|---|---|---|
| **PIBT** | Rule-based | O(n) | ✅ Yes | Good | Low |
| **RHCR** | Search-based | High | ✅ Yes | Excellent | High |
| **LaCAM** | Search-based | Medium | Partial | Very Good | Medium |
| **Real-Time LaCAM** | Search-based | Milliseconds | ✅ Yes | Very Good | Medium |
| **MAPF-LNS2** | Anytime search | High | Partial | Excellent | High |
| **GPBS** | Priority-based | Medium | Partial | Good | Medium |
| **RL-RH-PP** | Learning-based | Low (inference) | ✅ Yes | Best | High (training) |

---

## Deadlock and Congestion Handling

Deadlocks are a critical failure mode in warehouse MAPF:
- **PIBT** prevents deadlocks through backtracking in biconnected graphs
- **Deadlock-Free MAPD** (arXiv:2205.12504) extends PIBT to handle tree-shaped paths (dead-ends near shelves)
- **Traffic Flow Optimization** (arXiv:2308.11234): guides agents via congestion-avoiding paths, showing large throughput improvements over PIBT, LNS, and LaCAM

---

## Practical Recommendations for REFUGIO

Given the **180s budget for 300 ticks × 96 robots**:

1. **Use PIBT or a PIBT variant** as the core per-tick policy — it runs in microseconds per agent
2. **Augment PIBT with guidance** (see GGO document) to improve throughput without cost overhead
3. **Pre-compute guidance** in `create_layout()` setup phase — spend most of the 180s budget there
4. **Avoid RHCR or full LNS** at inference time — too slow for real-time tick budget

