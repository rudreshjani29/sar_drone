# SAR Drone Swarm Simulation

A simulated autonomous search-and-rescue system: multiple drones (3-5)
coordinating to search for a missing person, using a probability-weighted
search strategy grounded in real Lost Person Behavior research — not just
uniform-coverage waypoint flying. Built on PX4 + Gazebo + MAVSDK.

## The Core Idea

Real SAR search planning doesn't just cover ground uniformly — it uses
statistical models of where a lost person is *likely* to be (based on
subject profile, terrain, and time elapsed) to prioritize search effort.
This project builds that pipeline in simulation:

1. Define the search area and the subject's Point Last Seen (PLS)
2. Generate a probability-of-area (POA) map — a probability index over
   the search area, grounded in real search-theory statistics
3. Partition that map across multiple drones, each getting a
   probability-weighted, battery-aware sub-region
4. Fly all drones simultaneously, each running real detection logic
5. Dynamically reallocate search effort as detections/battery status
   change, with the goal of minimizing total time-to-find — the metric
   that actually matters when a life is at stake

## Project Status

| Phase | Focus | Status |
|---|---|---|
| [0](./phase0-toolchain-validation) | Toolchain validation (PX4 + Gazebo + MAVSDK) | ✅ Done |
| [1](./phase1-battery-aware-planning) | Battery-aware single-drone coverage planning | ✅ Done (fixed: column-safe sortie splitting) |
| [2](./phase2-gps-denied-navigation) | GPS-denied navigation | ⏸️ Blocked — retry with `SIM_GPS_BLOCK` pending verification |
| [3.1](./phase3.1-static-detection) | Detection confidence envelope (static images) | ✅ Done |
| 3.2 | Detection confidence (live Gazebo camera feed) | 🔜 Open |
| 4 | Search area layout (boundary, Point Last Seen, terrain) | 🔧 In progress |
| — | Custom Fusion 360 airframe → Gazebo digital twin swap | 🔜 Open (parallel track, targeted before Phase 6) |
| 5 | Probability-of-Area (POA) map | Planned |
| 6 | Multi-drone task allocation | Planned |
| 7 | Multi-drone simultaneous execution | Planned |
| 8 | Detection integration & dynamic reallocation | Planned |
| 9 | End-to-end evaluation (time-to-find KPI vs. uniform-search baseline) | Planned |

See [KNOWN_LIMITATIONS.md](./KNOWN_LIMITATIONS.md) for honestly-documented
gaps and simulator constraints hit along the way.

## Stack

- **PX4 Autopilot** (SITL) — flight controller firmware, same code that
  would run on real hardware
- **Gazebo** — physics simulation
- **MAVSDK-Python** — mission scripting / telemetry
- **YOLOv8** (ultralytics) — person detection
- **Autodesk Fusion 360** — custom hexacopter airframe design (in progress)

## Setup

Each phase folder has its own README with exact run instructions. Phase
0's README covers the full environment setup (PX4 + Gazebo + MAVSDK
install) that every later phase depends on.
