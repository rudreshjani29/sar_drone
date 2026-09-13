# SAR Drone Simulation

A simulated autonomous search-and-rescue drone (PX4 + Gazebo + MAVSDK). Built to
address real bottlenecks identified in current SAR drone research — battery
endurance vs. coverage area, unreliable detection under degraded conditions,
and GPS-denial in canyons/mountains/dense canopy — rather than just flying
scripted waypoints.

## Project Status

| Phase | Focus | Status |
|---|---|---|
| [Phase 0](./phase0-toolchain-validation) | Toolchain validation (PX4 + Gazebo + MAVSDK) | ✅ Done |
| [Phase 1](./phase1-battery-aware-planning) | Battery-aware coverage path planning | ✅ Done |
| [Phase 2](./phase2-gps-denied-navigation) | GPS-denied navigation | ⏸️ Blocked (documented) |
| Phase 3 | Realistic detection (confidence, not binary) | 🔜 Next |
| Phase 4 | Integration into one mission manager | Planned |
| Phase 5 | Multi-drone coordination (stretch) | Planned |

See [KNOWN_LIMITATIONS.md](./KNOWN_LIMITATIONS.md) for honestly-documented gaps
and simulator constraints hit along the way.

## Stack

- **PX4 Autopilot** (SITL) — flight controller firmware, same code that would
  run on real hardware
- **Gazebo** — physics simulation
- **MAVSDK-Python** — mission scripting / telemetry

## Setup

Each phase folder has its own README with exact run instructions. Phase 0's
README covers the full environment setup (PX4 + Gazebo + MAVSDK install) that
every later phase depends on.
