"""
Phase 2 — GPS-Denied Navigation Test

Uses PX4's built-in failure-injection system to actually turn GPS off
mid-flight, then back on, while continuously logging the drone's
estimated position (from PX4's own EKF2).

NOTE: as documented in this folder's README and the project-level
KNOWN_LIMITATIONS.md, GPS failure injection currently requires Gazebo
Classic and does NOT take effect under the newer Gazebo (gz_x500) target
used throughout this project. This script runs without error, but the
simulated GPS dropout will not actually occur until run under a
supported simulator backend.
"""

import asyncio
import csv
import time
from mavsdk import System


HOME_LAT = 47.397606
HOME_LON = 8.543060
CRUISE_ALTITUDE_M = 508.0 + 30.0

LOG_FILE = "gps_denial_log.csv"

BASELINE_DURATION_S = 20
DROPOUT_DURATION_S = 20
RECOVERY_DURATION_S = 20


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

    print("-- Enabling failure injection (SYS_FAILURE_EN)")
    await drone.param.set_param_int("SYS_FAILURE_EN", 1)
    await asyncio.sleep(1)

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

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print(f"-- Flying north, logging baseline ({BASELINE_DURATION_S}s, GPS healthy)")
    await drone.action.goto_location(HOME_LAT + 0.003, HOME_LON, CRUISE_ALTITUDE_M, 0)
    await asyncio.sleep(BASELINE_DURATION_S)

    print(f"-- INJECTING GPS FAILURE for {DROPOUT_DURATION_S}s: failure gps off")
    await drone.shell.send("failure gps off")
    await asyncio.sleep(DROPOUT_DURATION_S)

    print("-- RESTORING GPS: failure gps ok")
    await drone.shell.send("failure gps ok")
    await asyncio.sleep(RECOVERY_DURATION_S)

    print("-- Test flight complete, returning to launch")
    await drone.action.return_to_launch()

    try:
        await asyncio.wait_for(wait_for_landing(drone), timeout=60)
    except asyncio.TimeoutError:
        print("-- Landing detection timed out (drone likely landed, just didn't confirm)")

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
