"""
Phase 7 — Multi-drone simultaneous execution (PX4 SITL + Gazebo + MAVSDK-Python).

Plans with the Phase 4-6 modules (probability map -> probability sectors ->
greedy routes), then flies the first K waypoints of each drone's first sortie
on separate PX4 instances at the same time, and compares planned vs actual
leg times (a check of the Phase 6 timing model) and the minimum separation.

Requires MAVSDK-Python < 4 (pip install "mavsdk<4"); v4 changed the API.
Use --dry-run to test planning and plotting without any simulator.
"""
import argparse
import asyncio
import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for sub in ("common", "phase4-search-area-layout", "phase5-probability-map",
            "phase6-multi-drone-allocation"):
    sys.path.insert(0, os.path.join(HERE, "..", sub))
from drone_config import CRUISE_SPEED_MPS  # noqa: E402
from search_area import example_scenario, rasterize  # noqa: E402
from poa_model import build_prior  # noqa: E402
from allocation import CoarseMap, fly_drone, split_probability_sectors  # noqa: E402

SITL_HOME = (47.397606, 8.543060)     # PX4 default SITL home (instance 1 spawn)
ARRIVE_TOL_M = 10.0
ALT_BASE_AGL, ALT_STEP = 30.0, 5.0    # per-drone altitude layers for separation


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


# ---------------------------------------------------------------------------
# Planning (pure Python)
# ---------------------------------------------------------------------------
def plan_swarm(n, home_latlon, n_waypoints):
    """Return per-drone lists of (lat, lon, planned_leg_s) for sortie 1, first K waypoints."""
    grid = rasterize(example_scenario(), cell_m=25.0)
    cm = CoarseMap(grid, build_prior(grid))
    base = grid.frame.to_local(*home_latlon)
    groups = split_probability_sectors(cm, base, n)
    plans = []
    for g in groups:
        _, _, path = fly_drone(cm, g, "greedy", base)
        pts = [(x, y) for (_, x, y, sortie) in path if sortie == 1]
        prev, legs = pts[0], []
        for (x, y) in pts[1:n_waypoints + 1]:
            lat, lon = grid.frame.to_latlon(x, y)
            legs.append((lat, lon, math.hypot(x - prev[0], y - prev[1]) / CRUISE_SPEED_MPS))
            prev = (x, y)
        plans.append(legs)
    return plans, grid


def save_plan_plot(plans, grid, home_latlon, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 6))
    b = np.vstack([grid.boundary_xy, grid.boundary_xy[0]])
    ax.plot(b[:, 0], b[:, 1], "k-")
    hx, hy = grid.frame.to_local(*home_latlon)
    for i, legs in enumerate(plans):
        pts = [(hx, hy)] + [grid.frame.to_local(la, lo) for la, lo, _ in legs]
        p = np.array(pts)
        ax.plot(p[:, 0], p[:, 1], "-o", ms=4, label=f"drone {i + 1} (alt {ALT_BASE_AGL + i * ALT_STEP:.0f} m)")
    ax.plot(hx, hy, "r*", ms=15, mec="k", label="launch")
    ax.set_aspect("equal"); ax.legend(fontsize=8); ax.set_title("Planned first waypoints per drone")
    fig.tight_layout(); fig.savefig(path, dpi=110)


# ---------------------------------------------------------------------------
# Flight (needs PX4 SITL running)
# ---------------------------------------------------------------------------
async def connect_drone(idx, udp_port):
    from mavsdk import System
    drone = System(port=50051 + idx)             # one mavsdk_server per drone
    await drone.connect(system_address=f"udpin://0.0.0.0:{udp_port}")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            break
    return drone


async def first_position(drone):
    async for p in drone.telemetry.position():
        return p


async def wait_altitude_stable(drone, timeout=45):
    async def _run():
        last, t_last = None, time.time()
        async for p in drone.telemetry.position():
            if last is not None and abs(p.absolute_altitude_m - last) > 0.3:
                t_last = time.time()
            last = p.absolute_altitude_m
            if time.time() - t_last > 3 and time.time() - t0 > 6:
                return p.absolute_altitude_m
    t0 = time.time()
    return await asyncio.wait_for(_run(), timeout)


async def wait_arrival(drone, lat, lon, timeout):
    async def _run():
        async for p in drone.telemetry.position():
            if haversine_m(p.latitude_deg, p.longitude_deg, lat, lon) < ARRIVE_TOL_M:
                return
    await asyncio.wait_for(_run(), timeout)


async def track_positions(drone, idx, latest, tracks, stop):
    async for p in drone.telemetry.position():
        latest[idx] = (p.latitude_deg, p.longitude_deg, p.absolute_altitude_m)
        tracks.append((time.time(), idx, *latest[idx]))
        if stop.is_set():
            return


async def fly_one(drone, idx, legs, alt_agl, rows):
    name = f"drone {idx + 1}"
    await drone.action.set_takeoff_altitude(alt_agl)
    await drone.action.arm()
    await drone.action.takeoff()
    try:
        abs_alt = await wait_altitude_stable(drone)
    except asyncio.TimeoutError:
        abs_alt = (await first_position(drone)).absolute_altitude_m
    print(f"[{name}] at cruise altitude ({abs_alt:.1f} m AMSL), flying {len(legs)} waypoints")
    try:
        await drone.action.set_current_speed(CRUISE_SPEED_MPS)
    except Exception as e:                     # non-fatal; PX4 default speed is used
        print(f"[{name}] could not set speed: {e}")

    cur = await first_position(drone)
    for k, (lat, lon, planned) in enumerate(legs, start=1):
        dist = haversine_m(cur.latitude_deg, cur.longitude_deg, lat, lon)
        t0 = time.time()
        await drone.action.goto_location(lat, lon, abs_alt, 0)
        try:
            await wait_arrival(drone, lat, lon, timeout=max(30, 3 * dist / CRUISE_SPEED_MPS))
            ok = True
        except asyncio.TimeoutError:
            ok = False
        actual = time.time() - t0
        rows.append((idx + 1, k, round(dist, 1), round(planned, 1), round(actual, 1), int(ok)))
        print(f"[{name}] wp {k}/{len(legs)}: planned {planned:5.1f}s actual {actual:5.1f}s "
              f"{'' if ok else '(TIMEOUT)'}")
        cur = await first_position(drone)

    print(f"[{name}] returning to launch")
    await drone.action.return_to_launch()
    try:
        async def landed():
            async for a in drone.telemetry.in_air():
                if not a:
                    return
        await asyncio.wait_for(landed(), 90)
    except asyncio.TimeoutError:
        print(f"[{name}] landing not confirmed within 90 s (likely landed)")


async def run_flight(args, ports):
    n = args.n
    print(f"Connecting to {n} drones on UDP ports {ports} ...")
    drones = await asyncio.gather(*[connect_drone(i, p) for i, p in enumerate(ports)])
    home = await first_position(drones[0])
    home_ll = (home.latitude_deg, home.longitude_deg)
    plans, grid = plan_swarm(n, home_ll, args.waypoints)
    save_plan_plot(plans, grid, home_ll, os.path.join(HERE, "swarm_plan.png"))

    latest, tracks, stop = {}, [], asyncio.Event()
    trackers = [asyncio.create_task(track_positions(d, i, latest, tracks, stop))
                for i, d in enumerate(drones)]
    rows = []
    t_start = time.time()
    await asyncio.gather(*[
        fly_one(d, i, plans[i], ALT_BASE_AGL + i * ALT_STEP, rows) for i, d in enumerate(drones)
    ])
    stop.set()
    for t in trackers:
        t.cancel()
    print(f"\nMission wall time: {time.time() - t_start:.0f} s")
    summarize(rows, tracks, n)


def summarize(rows, tracks, n):
    with open(os.path.join(HERE, "swarm_legs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["drone", "waypoint", "dist_m", "planned_s", "actual_s", "arrived"])
        w.writerows(rows)
    with open(os.path.join(HERE, "swarm_tracks.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["unix_t", "drone", "lat", "lon", "alt_amsl_m"])
        w.writerows(tracks)

    ok = [r for r in rows if r[5] and r[3] > 5]
    if ok:
        ratio = np.array([r[4] / r[3] for r in ok])
        print(f"Actual / planned leg time: median {np.median(ratio):.2f}, "
              f"range {ratio.min():.2f}-{ratio.max():.2f}  (1.0 = Phase 6 timing model is exact)")
    # minimum separation using latest sample per drone at 1 s bins
    bins = {}
    for t, d, la, lo, al in tracks:
        bins.setdefault(int(t), {})[d] = (la, lo, al)
    min_sep = np.inf
    for pos in bins.values():
        ids = list(pos)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                pa, pb = pos[ids[a]], pos[ids[b]]
                h = haversine_m(pa[0], pa[1], pb[0], pb[1])
                min_sep = min(min_sep, math.hypot(h, pa[2] - pb[2]))
    if np.isfinite(min_sep):
        print(f"Minimum 3-D separation between any two drones: {min_sep:.1f} m")
    print("Wrote swarm_legs.csv, swarm_tracks.csv, swarm_plan.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3, help="number of drones (3-5)")
    ap.add_argument("--waypoints", type=int, default=8, help="waypoints per drone")
    ap.add_argument("--first-instance", type=int, default=1,
                    help="PX4 instance number of drone 1 (the docs start at -i 1)")
    ap.add_argument("--ports", type=str, default="",
                    help="comma-separated UDP ports, overrides 14540+instance")
    ap.add_argument("--dry-run", action="store_true", help="plan and plot only")
    args = ap.parse_args()

    if args.dry_run:
        plans, grid = plan_swarm(args.n, SITL_HOME, args.waypoints)
        for i, legs in enumerate(plans, 1):
            tot = sum(l[2] for l in legs)
            print(f"drone {i}: {len(legs)} waypoints, planned travel {tot:.0f} s")
        out = os.path.join(HERE, "swarm_plan.png")
        save_plan_plot(plans, grid, SITL_HOME, out)
        print("Saved", out)
        return

    ports = ([int(p) for p in args.ports.split(",")] if args.ports
             else [14540 + args.first_instance + i for i in range(args.n)])
    asyncio.run(run_flight(args, ports))


if __name__ == "__main__":
    main()
