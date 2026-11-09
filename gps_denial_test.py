"""
Phase 2 — GPS-Denied Navigation Test (FIXED)

Uses PX4's SIM_GPS_BLOCK parameter to actually block GPS data mid-flight,
then restore it, while continuously logging the drone's estimated
position (from PX4's own EKF2).

--- Why this version replaces the earlier "failure gps off" approach ---
The first version of this test used PX4's failure-injection system
(SYS_FAILURE_EN + "failure gps off"/"failure gps ok" shell commands).
That system requires simulator-level support that, per PX4's own docs,
is only implemented in Gazebo Classic — not the current Gazebo (gz_x500)
used throughout this project. Running it produced zero actual dropout:
is_global_position_ok never went false.

SIM_GPS_BLOCK operates at a different, lower level: PX4's core simulator
interface (simulator_mavlink.cpp), which blocks GPS data before it's
even passed up from the simulated sensor pipeline — independent of which
Gazebo backend is running. This should produce a real dropout regardless
of Gazebo version.

Requires PX4 SITL running (headless is fine):
    export GZ_IP=127.0.0.1
    HEADLESS=1 make px4_sitl gz_x500

Then, with sar-drone-env activated:
    python3 gps_denial_test.py
"""

import asyncio
import csv
import time
from mavsdk import System


HOME_LAT = 47.397606
HOME_LON = 8.543060
CRUISE_ALTITUDE_M = 508.0 + 30.0  # ~30m AGL

LOG_FILE = "gps_denial_log.csv"

BASELINE_DURATION_S = 20     # fly with normal GPS first (control data)
DROPOUT_DURATION_S = 20      # GPS blocked during this window
RECOVERY_DURATION_S = 20     # GPS back on, see how fast it recovers


async def wait_for_landing(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed")
            break


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for global position and home position estimates...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # --- Set up continuous logging (position + GPS health flag) ---
    log_rows = []
    health_state = {"gps_ok": True}
    stop_logging = asyncio.Event()
    start_time = time.monotonic()

    async def log_health():
        async for health in drone.telemetry.health():
            health_state["gps_ok"] = health.is_global_position_ok
            if stop_logging.is_set():
                break

    async def log_position():
        async for pos in drone.telemetry.position():
            t = round(time.monotonic() - start_time, 2)
            log_rows.append({
                "t_s": t,
                "lat": pos.latitude_deg,
                "lon": pos.longitude_deg,
                "alt_rel_m": round(pos.relative_altitude_m, 2),
                "gps_ok": health_state["gps_ok"],
            })
            if stop_logging.is_set():
                break

    health_task = asyncio.create_task(log_health())
    position_task = asyncio.create_task(log_position())

    # --- Fly the actual test profile ---
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print(f"-- Flying north, logging baseline ({BASELINE_DURATION_S}s, GPS healthy)")
    await drone.action.goto_location(HOME_LAT + 0.003, HOME_LON, CRUISE_ALTITUDE_M, 0)
    await asyncio.sleep(BASELINE_DURATION_S)

    print(f"-- BLOCKING GPS for {DROPOUT_DURATION_S}s: param set SIM_GPS_BLOCK 1")
    await drone.param.set_param_int("SIM_GPS_BLOCK", 1)
    await asyncio.sleep(DROPOUT_DURATION_S)

    print("-- RESTORING GPS: param set SIM_GPS_BLOCK 0")
    await drone.param.set_param_int("SIM_GPS_BLOCK", 0)
    await asyncio.sleep(RECOVERY_DURATION_S)

    print("-- Test flight complete, returning to launch")
    await drone.action.return_to_launch()

    try:
        await asyncio.wait_for(wait_for_landing(drone), timeout=60)
    except asyncio.TimeoutError:
        print("-- Landing detection timed out (drone likely landed, just didn't confirm)")

    # --- Stop logging and save results ---
    stop_logging.set()
    health_task.cancel()
    position_task.cancel()

    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["t_s", "lat", "lon", "alt_rel_m", "gps_ok"])
        writer.writeheader()
        writer.writerows(log_rows)

    print(f"\n-- Logged {len(log_rows)} samples to {LOG_FILE}")
    print("-- Run plot_gps_denial.py to visualize the dropout window.")


if __name__ == "__main__":
    asyncio.run(run())
