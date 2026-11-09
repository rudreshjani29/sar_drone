# Phase 7 — Multi-Drone Simultaneous Execution (PX4 SITL + Gazebo)

**Status: Written, NOT yet run against real PX4 by the author. Planning half is tested (`--dry-run`).**

## Goal
Fly 3-5 simulated drones at the same time, each following its own Phase 6 route, and check
(a) that several PX4 instances share one Gazebo world on your M1, (b) how real flight times
compare with the Phase 6 timing model, (c) how close the drones get to each other.

## Files
- `start_swarm.sh` / `stop_swarm.sh` — launch and stop N headless PX4 instances in one Gazebo world
- `swarm_mission.py` — plans (Phases 4-6), flies all drones concurrently, writes
  `swarm_legs.csv`, `swarm_tracks.csv`, `swarm_plan.png`

## Setup notes
- Use MAVSDK-Python **below v4**: `pip install "mavsdk<4"`. Version 4 renamed the package and changed the API
  (the scripts here use the old one, same as Phases 0-2).
- Commands in `start_swarm.sh` follow the PX4 multi-vehicle Gazebo docs: instance 1 starts the Gazebo
  server, later instances set `PX4_GZ_STANDALONE=1` with their own `-i` and `PX4_GZ_MODEL_POSE`.
  Check the docs for your PX4 version if anything differs.

## Run (three steps)
1. Test planning only, no simulator:
   ```bash
   source ~/sar-drone-env/bin/activate
   pip install "mavsdk<4" numpy matplotlib
   cd ~/Desktop/SaR_drone/phase7-multi-drone-execution
   python3 swarm_mission.py --n 3 --dry-run
   ```
2. Start with 2 drones first, then 3, then more (watch CPU and the logs):
   ```bash
   ./start_swarm.sh 2
   tail -n 20 ~/px4_swarm_logs/instance_2.log
   ```
3. Fly:
   ```bash
   python3 swarm_mission.py --n 2 --waypoints 8
   ./stop_swarm.sh
   ```

## Things that may need adjusting
- **UDP ports.** The script assumes drone i uses port 14540 + instance number (instance 1 -> 14541).
  If it hangs on "Connecting", look at each log for the MAVLink remote port and pass
  `--ports 14541,14542,...`, or `--first-instance 0`.
- **Lockstep and speed.** If Gazebo runs slower than real time on your M1, actual/planned times will be
  inflated by that factor, not by the drones. Watch for it: more instances means slower simulation.
- **First launch.** `start_swarm.sh` expects PX4 to be built already (`HEADLESS=1 make px4_sitl gz_x500` once).
- **Landing detection** can hang on your machine; the script times out after 90 s, as before.

## What it measures
- Median and range of actual/planned leg time (1.0 means the Phase 6 timing model is exact).
  Phase 6 ignores acceleration, turns and wind, so expect actual > planned.
- Minimum 3-D separation between any two drones (drones fly at 30, 35, 40, 45, 50 m AGL as a crude
  vertical separation).

## Limits
- Short test flights (default 8 waypoints per drone), not a full search mission.
- No collision avoidance beyond altitude layers.
- Static plan only; the Phase 8 dynamic reallocation is not wired to live flight yet.
- Detection is not part of this phase (Phase 3.2/8).
