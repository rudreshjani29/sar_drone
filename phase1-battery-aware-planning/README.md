# Phase 1 — Battery-Aware Coverage Path Planning

**Status: Done**

## Goal

Replace a fixed waypoint list with a real coverage planner: given a search
area and the drone's actual battery/endurance numbers (from the hardware
spec — 5200mAh battery, ~16A hover draw), generate a lawnmower search
pattern that fits within one battery charge, or automatically split into
multiple sorties (with a simulated battery-swap pause) if the area is too
large.

This directly targets the #1 bottleneck identified in real SAR drone
research: endurance vs. coverage area.

## What's Here

- `coverage_planner.py` — the planning logic. Generates a boustrophedon
  (back-and-forth) coverage path over a rectangular search area, computes
  real flight-time and energy-margin estimates, and splits into multiple
  sorties if needed. Runnable standalone to just see/print a plan without
  flying anything.
- `phase1_mission.py` — flies the generated plan against a running PX4 SITL
  instance via MAVSDK, sortie by sortie.

## Running the Planner Alone (no simulator needed)

```bash
python3 coverage_planner.py
```

Prints the plan (area, waypoint count, distance, estimated flight time,
energy margin, and sortie split if applicable).

## Running the Full Mission (simulator required)

With PX4 SITL running (see Phase 0's README for setup):

```bash
source ~/sar-drone-env/bin/activate
python3 phase1_mission.py
```

To see the multi-sortie splitting logic actually trigger, edit the area
size near the bottom of `phase1_mission.py` to something larger, e.g.:

```python
lat_span = meters_to_lat_deg(2000)
lon_span = meters_to_lon_deg(3000, HOME_LAT)
```

## Known Limitation

See the project-level `KNOWN_LIMITATIONS.md` — sortie splitting currently
chunks the flattened waypoint list evenly rather than splitting into clean
geographic sub-regions per sortie.
