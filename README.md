# SAR Drone Swarm Simulation

A simulated autonomous search-and-rescue system: 3-5 drones coordinating to find a
missing person, using a probability-weighted search strategy (Lost Person Behavior
research and Bayesian search theory) instead of uniform-coverage waypoint flying.
Built with PX4 + Gazebo + MAVSDK-Python and plain Python simulation.

> **Simulation only.** All results use assumed parameters and simulated hiding places.
> They are not real rescue data. Every known gap is listed in
> [KNOWN_LIMITATIONS.md](./KNOWN_LIMITATIONS.md).

## The Core Idea

Real SAR planning uses statistical models of where a lost person is likely to be. This
project builds that pipeline end to end in simulation:

1. Define the search area, terrain and the Point Last Seen (PLS)
2. Build a probability-of-area (POA) map over the area
3. Split the map between drones (probability-weighted, battery-aware)
4. Fly all drones at the same time
5. Re-plan dynamically as cells are cleared or a drone is lost, to minimize time-to-find

Working approach: **replicate a baseline first, then improve**, and document the limits honestly.

## Project Status

| Phase | Focus | Status |
|---|---|---|
| [0](./phase0-toolchain-validation) | Toolchain validation (PX4 + Gazebo + MAVSDK) | ✅ Done |
| [1](./phase1-battery-aware-planning) | Battery-aware single-drone coverage planning | ✅ Done |
| [2](./phase2-gps-denied-navigation) | GPS-denied navigation | ⏸️ Blocked, `SIM_GPS_BLOCK` retry unverified |
| [3.1](./phase3.1-static-detection) | Detection confidence envelope (static images) | ✅ Done |
| 3.2 | Detection on live Gazebo camera feed | 🔜 Open |
| [4](./phase4-search-area-layout) | Search area layout (boundary, PLS, trails, water) | ✅ Done (hand-defined area) |
| [5](./phase5-probability-map) | POA map: 5A rings + trails, 5B agent-based walker | ✅ Done (illustrative parameters) |
| [6](./phase6-multi-drone-allocation) | Multi-drone task allocation (simulation) | ✅ Done |
| [7](./phase7-multi-drone-execution) | Multi-drone flight, several PX4 instances in Gazebo | 🔧 Runs; scaling to 3-5 drones in progress |
| [8](./phase8-dynamic-reallocation) | Dynamic reallocation (simulation) and detection integration | 🔧 Reallocation done; live integration open |
| 9 | End-to-end evaluation (time-to-find vs. uniform-search baseline) | Planned |
| — | Custom hexacopter airframe (Fusion 360, NACA-based body) → Gazebo model | 🔜 Parallel track |

## Highlights (simulation, assumed parameters)

- **More drones, much faster:** 4 drones reach half of the likely hiding places about 4x
  faster than 1 drone ([Phase 6](./phase6-multi-drone-allocation)).
- **Probability-ordered search beats a plain sweep** at every drone count.
- **Resilience:** when one drone is lost mid-search, dynamic re-planning found about 79% of
  hidden people versus about 60% for fixed plans ([Phase 8](./phase8-dynamic-reallocation)).
- **A wrong map costs time:** Bayesian revisits help in the long run but slow the first
  hour when the probability map is wrong (model mismatch).
- Against an independent walker model, the probability map's advantage over uniform search
  was about 13 points at 20% area covered, much smaller than the self-consistent test
  suggested ([Phase 5](./phase5-probability-map)).

## Repository Layout

```
common/                         shared hardware constants (drone_config.py)
phase0-... to phase8-...        one folder per phase, each with its own README
KNOWN_LIMITATIONS.md            honest list of gaps and simulator constraints
```

## Quick Start

Phases 4, 5, 6 and 8 are pure Python and need no simulator:

```bash
python3 -m venv ~/sar-drone-env && source ~/sar-drone-env/bin/activate
pip install numpy matplotlib "mavsdk<4"
cd phase4-search-area-layout && python3 plot_search_area.py
cd ../phase5-probability-map && python3 run_poa.py && python3 run_walker.py
cd ../phase6-multi-drone-allocation && python3 run_allocation.py
cd ../phase8-dynamic-reallocation && python3 run_dynamic.py
```

Phases 0-2 and 7 need PX4 SITL + Gazebo. See
[phase 0](./phase0-toolchain-validation) for the macOS setup and
[phase 7](./phase7-multi-drone-execution) for the multi-drone launch scripts.

**Note:** use MAVSDK-Python below v4 (`pip install "mavsdk<4"`). Version 4 changed the API.

## Stack

- **PX4 Autopilot** (SITL) and **Gazebo Harmonic**
- **MAVSDK-Python** for mission scripting and telemetry
- **NumPy / Matplotlib** for the probability map and planning simulations
- **YOLOv8** (ultralytics) for person detection tests
- **Autodesk Fusion 360 / XFLR5** for the custom airframe (in progress)

## Key Limitations

- Lost-person distance parameters are illustrative; the hiker-specific ISRID table was not used.
- Probability map and hidden-person model both come from assumed behaviour rules.
- Detection chance (0.8) is assumed until the live camera feed (Phase 3.2) is measured.
- Drones in the planning simulations share a perfect, instant belief map; no collision avoidance.

See [KNOWN_LIMITATIONS.md](./KNOWN_LIMITATIONS.md) for the full list.
