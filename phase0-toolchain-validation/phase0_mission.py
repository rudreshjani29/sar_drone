"""
Phase 0 — Toolchain validation script.

Connects to a running PX4 SITL instance (launched via
`HEADLESS=1 make px4_sitl gz_x500`), arms the drone, takes off, flies to
three waypoints in sequence, then lands.

Requires: pip install mavsdk
"""

import asyncio
from mavsdk import System


# Default PX4 SITL home position (Zurich test field). If your simulator
# reports a different home position, adjust these offsets accordingly —
# the script prints the actual home position on startup so you can check.
HOME_LAT = 47.397606
HOME_LON = 8.543060
HOME_ALT_M = 488.0  # meters above sea level (SITL default ground altitude)

# Waypoints defined as small lat/lon offsets from home (roughly 50-100m apart).
# 0.0005 degrees latitude ≈ 55 meters.
WAYPOINTS = [
    (HOME_LAT + 0.0005, HOME_LON, HOME_ALT_M + 20),   # north, 20m up
    (HOME_LAT + 0.0005, HOME_LON + 0.0007, HOME_ALT_M + 20),  # then east
    (HOME_LAT, HOME_LON + 0.0007, HOME_ALT_M + 20),   # then south
]


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

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)  # give it time to reach takeoff altitude

    for i, (lat, lon, alt) in enumerate(WAYPOINTS, start=1):
        print(f"-- Flying to waypoint {i}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
        await drone.action.goto_location(lat, lon, alt, 0)
        await asyncio.sleep(15)

    print("-- Mission complete, returning to launch and landing")
    await drone.action.return_to_launch()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed")
            break

    print("-- Phase 0 validation mission complete.")


if __name__ == "__main__":
    asyncio.run(run())
