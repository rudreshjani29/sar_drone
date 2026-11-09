# Phase 6 — Multi-Drone Search Allocation (simulation, no PX4)

**Status: Built. Static plan; no live re-planning yet (that is Phase 8).**

## What it does
Splits the Phase 5A probability map between 1-5 drones, simulates each drone's
battery-limited flight (sorties, return to base, battery swap), and measures how
fast hidden people are found. Hidden people are sampled from an independent walker
(different settings and seed from the planning map, as in Phase 5B).

Strategies compared:
- **sweep** — snake-pattern coverage in order (uniform-search style)
- **greedy** — next cell = highest probability per second of travel + search
- **equal area** — drones get equal-size strips
- **POA sectors** — drones get pie slices around the base with equal probability mass

## Files
- `allocation.py` — coarse map, partitioning strategies, battery-aware flight simulator
- `run_allocation.py` — runs all strategies, prints a table, writes `allocation_comparison.png`
- Hardware numbers (speed, battery) come from `common/drone_config.py`.

## Run
```bash
source ~/sar-drone-env/bin/activate
cd phase6-multi-drone-allocation && python3 run_allocation.py
```

## Result (example scenario, 3.9 km², POD 0.8 assumed)
Time to reach 50% of hidden locations (minutes):
| drones | equal area + sweep | equal area + greedy | POA sectors + greedy |
|---|---|---|---|
| 1 | 239 | 192 (greedy) | — |
| 3 | 79 | 69 | 59 |
| 4 | 71 | 51 | 48 |
| 5 | 52 | 37 | 34 |

- Adding drones helps a lot, mostly because each drone has its own battery and more of the
  area is covered at once.
- Probability-ordered search beats sweep at every drone count.
- Probability-weighted sectors gave a smaller gain on top of greedy ordering (a few minutes).
- Cost: sectors finish the full area later (e.g. 192 vs 162 min at 4 drones), because balancing
  probability does not balance flight time. Phase 8 rebalances dynamically.

## Honest limitations
- Probability map and "truth" both come from assumed models (see Phase 5). Gains are
  simulation results, not real-world claims.
- 50 m coarse cells with one detection chance per visit; camera footprint, overlap and
  repeat passes are not modelled.
- Straight-line flight at constant speed; no wind, climb, turns, takeoff/landing, or
  obstacle avoidance. Battery swap time (5 min) and POD (0.8) are assumed.
- Common base at the PLS; no drone-to-drone collision avoidance.
- Static plan: when a cell is searched empty, other drones are not re-assigned. Phase 8.
