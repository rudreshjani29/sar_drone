# Phase 2 — GPS-Denied Navigation

**Status: Blocked (documented) — revisit later**

## Goal

Real SAR drones lose GPS lock in canyons, dense forest, and mountainous
terrain — exactly the environments where search missions happen. The goal
was to deliberately trigger a GPS dropout mid-flight and measure how well
PX4's built-in EKF2 estimator holds a stable position estimate using other
sensors (IMU dead-reckoning) during the outage.

## What's Here

- `gps_denial_test.py` — flies a straight line, and partway through uses
  PX4's failure-injection system (`SYS_FAILURE_EN` param + `failure gps
  off` / `failure gps ok`, sent via MAVSDK's shell plugin) to actually
  turn GPS off, then back on. Logs position, altitude, and GPS-health
  status continuously to `gps_denial_log.csv`.
- `plot_gps_denial.py` — reads that CSV and plots the ground track and
  altitude over time, with the dropout window shaded red.

## Why This Is Blocked

PX4's own documentation states GPS failure injection **requires support in
the simulator, and is only supported in Gazebo Classic** — not the current
Gazebo (`gz_x500` target) used throughout this project. Running the test
confirmed this directly: `failure gps off` was accepted with no error, but
`is_global_position_ok` never actually went false, and the resulting plot
showed zero dropout — a continuous, unbroken GPS-healthy track throughout.

This is a real, external tooling gap, not a bug in the test logic itself.

## Two Ways Forward (deferred)

1. Run this specific test under Gazebo Classic instead of the current
   Gazebo.
2. Simulate GPS loss at a different layer entirely — e.g. intercepting
   and blocking/corrupting GPS MAVLink messages directly, rather than
   relying on PX4's built-in failure injection.

See the project-level `KNOWN_LIMITATIONS.md` for the full note.
