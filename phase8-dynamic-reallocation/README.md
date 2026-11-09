# Phase 8 (part 1) — Dynamic Reallocation (simulation, no PX4)

**Status: Built. Results are mixed; see below.**

## What it does
No fixed partition. Whenever a drone finishes a cell it picks the next one from a SHARED
belief map (highest belief per second of travel + search), skipping cells another drone has
claimed. Optionally, each search multiplies that cell's belief by (1 - POD) (Bayesian update),
so cells can be revisited. If a drone is lost, its claim is released and the others absorb its work.

Because drones learn only from misses, the plan before the first find is the same for every
hiding place, so one plan is simulated and scored against 3000 hidden locations (x10 detection draws).

## Files
- `dynamic.py` — event-driven shared-belief simulator
- `run_dynamic.py` — comparison, drone-failure test, matched-model check; writes `dynamic_comparison.png`

## Run
```bash
source ~/sar-drone-env/bin/activate
cd phase8-dynamic-reallocation && python3 run_dynamic.py
```
Needs the Phase 4, 5, 6 and `common/` folders alongside.

## Results (4 drones, example scenario)
Note: times include missed detections (POD 0.8), so they are larger than Phase 6's visit-time figures.

| Strategy | t50 (min) | found <60 min | found <240 min |
|---|---|---|---|
| static equal area + greedy | 72 | 46% | 79% |
| static probability sectors + greedy | 60 | 50% | 79% |
| dynamic, one look per cell | 60 | 50% | 79% |
| dynamic + Bayesian revisits | 74 | 46% | 93% |

Drone #2 lost at 20 min: found <240 min is 63% / 60% (static) vs 79% (dynamic) vs 85% (dynamic + revisits).

## What this shows
- **Resilience is the clear win.** When a drone is lost, static plans leave its sector unsearched;
  dynamic plans cover the gap (79% vs about 60% found by 240 min). Dynamic also covers the whole area
  sooner without a failure (149 min vs 163-192).
- **Without a failure, dynamic is about equal to the best static plan** early on. It does not
  beat probability sectors in time to 50%.
- **Bayesian revisits help late, hurt early.** They raise the long-run find rate (93% vs 79%) because
  second looks catch missed detections, but they slow the first hour. Matched-model check: when hidden
  people are drawn from the planning map itself, revisits cost nothing early (48.7 vs 49.2 min) and
  still win late (94% vs 79%). So the early penalty comes from the planning map being wrong about
  where the person is: revisiting "likely" cells wastes time. This is a real risk of trusting a model.
- Greedy rate-based choice is a heuristic, not an optimal planner.

## Honest limitations
- Same assumed models as Phases 5-6 (walker, POD 0.8, 50 m cells, swap time, quadcopter placeholders).
- No communication delays or loss between drones; shared belief is perfect and instant.
- No collision avoidance; claims only prevent two drones targeting one cell.
- The failure test is a single scenario (one drone, one time).
- Single run of each plan; no repeated random seeds or parameter sweeps yet.
