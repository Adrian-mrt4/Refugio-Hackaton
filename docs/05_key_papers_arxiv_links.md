# Key Papers & Resources — Annotated Bibliography

All papers are directly relevant to the REFUGIO Warehouse Challenge (Lifelong MAPF + Layout Optimization).

---

## 🏆 Tier 1: Must-Read (Most Directly Relevant)

### [1] Multi-Robot Coordination and Layout Design for Automated Warehousing
- **Authors**: Yulun Zhang, Matthew Fontaine, Varun Bhatt, Stefanos Nikolaidis, Jiaoyang Li
- **Venue**: IJCAI 2023
- **arXiv**: https://arxiv.org/abs/2305.06436
- **PDF**: https://arxiv.org/pdf/2305.06436.pdf
- **Why**: Directly solves `create_layout()` — optimizes shelf placement to maximize MAPF throughput using QD/CMA-ES. Shows that layout design rivals algorithm improvement for throughput gains.

### [2] Priority Inheritance with Backtracking for Iterative Multi-Agent Path Finding
- **Authors**: Keisuke Okumura, Manao Machida, Xavier Défago, Yasumasa Tamura
- **Venue**: IJCAI 2019 / Artificial Intelligence Journal 2022
- **arXiv**: https://arxiv.org/abs/1901.11282
- **Code**: https://kei18.github.io/pibt2/
- **Why**: PIBT is the state-of-the-art per-tick policy for lifelong MAPF — the foundation for implementing `act()`. Near-linear time, completeness guaranteed, warehouse-tested.

### [3] Guidance Graph Optimization for Lifelong Multi-Agent Path Finding
- **Authors**: Yulun Zhang, He Jiang, Varun Bhatt, Stefanos Nikolaidis, Jiaoyang Li
- **Venue**: IJCAI 2024
- **arXiv**: https://arxiv.org/abs/2402.01446
- **Why**: Shows how to pre-compute guidance (virtual traffic lanes) that dramatically improves PIBT throughput. Can be combined with layout optimization. PIBT+GGO ≈ RHCR performance.

### [4] Lifelong Multi-Agent Path Finding in Large-Scale Warehouses (RHCR)
- **Authors**: Jiaoyang Li, Andrew Tinka, Scott Kiesel, Joseph Durham, T.K.S. Kumar, Sven Koenig
- **Venue**: AAAI 2021
- **arXiv**: https://arxiv.org/abs/2005.07371
- **Why**: Defines the Lifelong MAPF problem setup used in REFUGIO. RHCR framework for comparison. Up to 1,000 agents in warehouse settings.

---

## 🥈 Tier 2: Important Background

### [5] Online Guidance Graph Optimization for Lifelong MAPF
- **Authors**: Hongzhi Zang, Yulun Zhang, He Jiang, Zhe Chen, Daniel Harabor, Peter Stuckey, Jiaoyang Li
- **Venue**: AAAI 2025
- **arXiv**: https://arxiv.org/abs/2411.16506
- **Why**: Dynamic guidance that adapts to real-time traffic — can run in REFUGIO's `act()` to update guidance between ticks.

### [6] Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding (LaCAM*)
- **Authors**: Keisuke Okumura
- **Venue**: IJCAI 2023
- **arXiv**: https://arxiv.org/abs/2305.03632
- **Code**: https://kei18.github.io/lacam2/
- **Why**: Scalable MAPF solver solving 99% of benchmark instances with up to 1,000 agents in 10s. Useful for pre-computing initial paths.

### [7] Real-Time LaCAM for Real-Time MAPF
- **Authors**: Various
- **arXiv**: https://arxiv.org/abs/2504.06091
- **Why**: First real-time MAPF method with completeness guarantees using millisecond cutoffs per tick. Directly applicable to REFUGIO's tick model.

### [8] Traffic Flow Optimisation for Lifelong Multi-Agent Path Finding
- **arXiv**: https://arxiv.org/html/2308.11234v4
- **Why**: Congestion-avoiding path guidance. Reports large throughput improvements over PIBT, LNS, and LaCAM. Key insight: beyond a peak agent density, adding more agents decreases throughput — relevant for 96-robot REFUGIO setup.

### [9] Learning-guided Prioritized Planning for Lifelong MAPF (RL-RH-PP)
- **arXiv**: https://arxiv.org/abs/2603.23838
- **Why**: RL assigns dynamic priorities. Achieves highest throughput among baselines. Shows RL+search hybrid approach for MAPF.

### [10] Novel Framework for Automated Warehouse Layout Generation
- **Venue**: Frontiers in Artificial Intelligence, 2024
- **URL**: https://www.frontiersin.org/journals/artificial-intelligence/articles/10.3389/frai.2024.1465186/full
- **Why**: Constrained beam search for layout generation with feasibility guarantees — alternative to evolutionary optimization.

---

## 🥉 Tier 3: Useful References

### [11] Scaling Lifelong MAPF to More Realistic Settings: Research Challenges
- **Authors**: He Jiang, Yulun Zhang, Rishi Veerapaneni, Jiaoyang Li
- **arXiv**: https://arxiv.org/abs/2404.16162
- **Why**: Research roadmap identifying key open challenges (tight time budgets, heterogeneous tasks).

### [12] MAPF-LNS2: Fast Repairing via Large Neighborhood Search
- **Authors**: Jiaoyang Li et al.
- **arXiv**: https://arxiv.org/abs/2111.11357
- **Code**: https://github.com/Jiaoyang-Li/MAPF-LNS2
- **Why**: State-of-the-art one-shot MAPF algorithm; useful as layout evaluator.

### [13] Scalable Imitation Learning for Lifelong MAPF (SILLM)
- **arXiv**: https://arxiv.org/abs/2410.21415
- **Why**: Neural network policy handling 10,000 agents. Imitation learning from PIBT provides training methodology.

### [14] Caching-Augmented Lifelong MAPF (CAL-MAPF)
- **arXiv**: https://arxiv.org/abs/2403.13421
- **Why**: Introduces cache grids for temporary item storage — relevant if REFUGIO has staging areas.

### [15] Deadlock-Free Method for MAPD Using PIBT with Temporary Priority
- **arXiv**: https://arxiv.org/abs/2205.12504
- **Why**: Extends PIBT to tree-shaped paths (dead-end aisles). Directly handles the case of shelves accessible from only one direction.

### [16] A Comprehensive Review on Leveraging ML for MAPF
- **Authors**: J.M. Alkazzi, Keisuke Okumura
- **Venue**: IEEE Access, 2024
- **Why**: Survey covering all ML-based MAPF methods — useful overview for choosing the learning-based approach.

### [17] Local Guidance for Configuration-Based Multi-Agent Pathfinding
- **arXiv**: https://arxiv.org/abs/2510.19072
- **Why**: Per-agent local spatiotemporal cues improve LaCAM performance significantly. New performance frontier for MAPF guidance (2025).

### [18] Lifelong LaCAM with Local Guidance (LLLG)
- **arXiv**: https://arxiv.org/abs/2605.16855
- **Why**: Lifelong version of LaCAM+local guidance. State-of-the-art lifelong MAPF as of May 2026. Receding-horizon windowed planning with warm-starts.

### [19] Advancing MAPF Toward the Real World: SMART Testbed
- **Authors**: Yulun Zhang, Zhe Chen et al.
- **arXiv**: https://arxiv.org/abs/2503.04798
- **Why**: Realistic MAPF evaluation platform from the same research group. Discusses gap between benchmark and real-world performance.

### [20] Mixed Guidance Graph Optimization (MGGO)
- **Authors**: Yulun Zhang et al.
- **URL**: https://yulunzhang.net/publication/zhang2026mggo/
- **Why**: Extends GGO to optimize both edge weights AND directions. Most recent (2026) work from the layout optimization group.

---

## Key Research Groups & Repositories

| Group | Focus | Links |
|---|---|---|
| ARCS Lab, CMU (Jiaoyang Li) | Lifelong MAPF, Layout Optimization | https://jiaoyangli.me |
| Yulun Zhang (CMU PhD) | Layout Optimization, GGO | https://yulunzhang.net |
| Keisuke Okumura (TiTech) | PIBT, LaCAM | https://kei18.github.io |
| mapf.info | MAPF benchmark & community | http://mapf.info |
| Jiaoyang Li GitHub | MAPF-LNS2 code | https://github.com/Jiaoyang-Li |

---

## Open-Source Code Repositories

| Repository | Algorithm | URL |
|---|---|---|
| pibt2 | PIBT, winPIBT | https://kei18.github.io/pibt2/ |
| lacam2 | LaCAM, LaCAM* | https://kei18.github.io/lacam2/ |
| MAPF-LNS2 | LNS-based MAPF | https://github.com/Jiaoyang-Li/MAPF-LNS2 |
| MAPF benchmark | Standard test maps | https://movingai.com/benchmarks/mapf/ |

