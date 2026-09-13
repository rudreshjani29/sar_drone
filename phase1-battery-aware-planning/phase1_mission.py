"""
Phase 1 — Battery-Aware Mission Execution

Uses coverage_planner.py to generate a lawnmower search pattern sized to
fit within one battery's real endurance, then flies it via MAVSDK against
a running PX4 SITL + Gazebo instance.
"""

import asyncio
from mavsdk import System

from coverage_planner import plan_mission, meters_to_lat_deg, meters_to_lon_deg


HOME_LAT = 47.397606
HOME_LON = 8.543060
CRUISE_ALTITUDE_M = 508.0 + 50.0  # SITL ground altitude (~508m) + 50m AGL


async def wait_for_landing(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed")
            break


async def fly_sortie(drone, waypoints, sortie_number):
    print(f"\n{'=' * 50}")
    print(f"STARTING SORTIE {sortie_number} ({len(waypoints)} waypoints)")
    print(f"{'=' * 50}")

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    for i, (lat, lon, _) in enumerate(waypoints, start=1):
        print(f"-- Sortie {sortie_number}, waypoint {i}/{len(waypoints)}: "
              f"lat={lat:.6f}, lon={lon:.6f}")
        await drone.action.goto_location(lat, lon, CRUISE_ALTITUDE_M, 0)
        await asyncio.sleep(8)

    print(f"-- Sortie {sortie_number} coverage complete, returning to launch")
    await drone.action.return_to_launch()

    try:
        await asyncio.wait_for(wait_for_landing(drone), timeout=60)
    except asyncio.TimeoutError:
        print("-- Landing detection timed out (drone likely landed, "
              "just didn't confirm)")


async def run():
    lat_span = meters_to_lat_deg(200)
    lon_span = meters_to_lon_deg(300, HOME_LAT)

    sorties = plan_mission(
        min_lat=HOME_LAT,
        min_lon=HOME_LON,
        max_lat=HOME_LAT + lat_span,
        max_lon=HOME_LON + lon_span,
        swath_width_m=40.0,
        altitude_m=50.0,
    )

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("\nWaiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for global position and home position estimates...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    for idx, sortie_waypoints in enumerate(sorties, start=1):
        await fly_sortie(drone, sortie_waypoints, idx)
        if idx < len(sorties):
            input(f"\n>> Sortie {idx} complete. Simulate a battery swap, "
                  f"then press Enter to continue to sortie {idx + 1}...")

    print("\n-- All sorties complete. Mission finished.")


if __name__ == "__main__":
    asyncio.run(run())
