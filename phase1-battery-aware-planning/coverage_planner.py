"""
Phase 1 — Battery-Aware Coverage Path Planner

Takes a rectangular search area (defined by lat/lon corners), a desired
camera swath width, and the drone's real battery/endurance numbers from
the hardware spec, and produces a lawnmower ("boustrophedon") search
pattern of waypoints.

Unlike a fixed waypoint list, this planner:
  1. Computes how much area a single battery charge can actually cover
  2. Warns (and splits into multiple sorties) if the requested area
     exceeds what one battery can cover
  3. Reports estimated flight time and remaining energy margin
"""

import math


# ---------------------------------------------------------------------------
# REAL HARDWARE NUMBERS (from the hardware spec — not placeholders)
# ---------------------------------------------------------------------------
BATTERY_CAPACITY_MAH = 5200
USABLE_FRACTION = 0.8          # don't drain below 20%, protects battery health
HOVER_CURRENT_DRAW_A = 16.0    # estimated system current draw at hover, from spec
CRUISE_CURRENT_DRAW_A = 12.0   # cruise (forward flight) typically draws less than hover
CRUISE_SPEED_MPS = 8.0         # forward flight speed during search legs, m/s

USABLE_CAPACITY_AH = (BATTERY_CAPACITY_MAH / 1000) * USABLE_FRACTION
MAX_FLIGHT_TIME_MIN = (USABLE_CAPACITY_AH / CRUISE_CURRENT_DRAW_A) * 60
SAFETY_MARGIN_FRACTION = 0.85
PLANNABLE_FLIGHT_TIME_MIN = MAX_FLIGHT_TIME_MIN * SAFETY_MARGIN_FRACTION


# ---------------------------------------------------------------------------
# GEOMETRY HELPERS
# ---------------------------------------------------------------------------
def meters_to_lat_deg(meters):
    return meters / 111320.0


def meters_to_lon_deg(meters, at_latitude_deg):
    return meters / (111320.0 * math.cos(math.radians(at_latitude_deg)))


def haversine_distance_m(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


# ---------------------------------------------------------------------------
# LAWNMOWER PATTERN GENERATOR
# ---------------------------------------------------------------------------
def generate_lawnmower_pattern(min_lat, min_lon, max_lat, max_lon,
                                swath_width_m, altitude_m):
    center_lat = (min_lat + max_lat) / 2
    area_height_m = haversine_distance_m(min_lat, min_lon, max_lat, min_lon)
    area_width_m = haversine_distance_m(min_lat, min_lon, min_lat, max_lon)

    num_passes = max(1, math.ceil(area_width_m / swath_width_m))
    lon_step_deg = meters_to_lon_deg(area_width_m / num_passes, center_lat)

    waypoints = []
    current_lon = min_lon
    going_north = True

    for i in range(num_passes + 1):
        if going_north:
            waypoints.append((max_lat, current_lon, altitude_m))
            waypoints.append((min_lat, current_lon, altitude_m))
        else:
            waypoints.append((min_lat, current_lon, altitude_m))
            waypoints.append((max_lat, current_lon, altitude_m))
        going_north = not going_north
        current_lon += lon_step_deg
        if current_lon > max_lon + 1e-9:
            break

    return waypoints, area_width_m, area_height_m


def total_path_distance_m(waypoints):
    dist = 0.0
    for i in range(len(waypoints) - 1):
        lat1, lon1, _ = waypoints[i]
        lat2, lon2, _ = waypoints[i + 1]
        dist += haversine_distance_m(lat1, lon1, lat2, lon2)
    return dist


# ---------------------------------------------------------------------------
# BATTERY-AWARE PLANNING LOGIC
# ---------------------------------------------------------------------------
def plan_mission(min_lat, min_lon, max_lat, max_lon,
                  swath_width_m=40.0, altitude_m=50.0):
    waypoints, area_width_m, area_height_m = generate_lawnmower_pattern(
        min_lat, min_lon, max_lat, max_lon, swath_width_m, altitude_m
    )

    distance_m = total_path_distance_m(waypoints)
    flight_time_min = (distance_m / CRUISE_SPEED_MPS) / 60

    area_km2 = (area_width_m * area_height_m) / 1_000_000

    print("=" * 60)
    print("MISSION PLAN")
    print("=" * 60)
    print(f"Search area:              {area_width_m:.0f}m x {area_height_m:.0f}m "
          f"({area_km2:.3f} km^2)")
    print(f"Swath width per pass:     {swath_width_m:.0f} m")
    print(f"Number of waypoints:      {len(waypoints)}")
    print(f"Total path distance:      {distance_m:.0f} m")
    print(f"Estimated flight time:    {flight_time_min:.1f} min")
    print(f"Plannable battery budget: {PLANNABLE_FLIGHT_TIME_MIN:.1f} min "
          f"(with safety margin)")

    if flight_time_min <= PLANNABLE_FLIGHT_TIME_MIN:
        margin_min = PLANNABLE_FLIGHT_TIME_MIN - flight_time_min
        print(f"\n>> FITS IN ONE SORTIE. Energy margin: {margin_min:.1f} min "
              f"({(margin_min / PLANNABLE_FLIGHT_TIME_MIN) * 100:.0f}% of budget)")
        return [waypoints]
    else:
        num_sorties = math.ceil(flight_time_min / PLANNABLE_FLIGHT_TIME_MIN)
        print(f"\n>> EXCEEDS ONE BATTERY'S BUDGET.")
        print(f">> Splitting into {num_sorties} sorties (requires a battery "
              f"swap between each).")
        # NOTE: naive split — see project KNOWN_LIMITATIONS.md
        chunk_size = math.ceil(len(waypoints) / num_sorties)
        sorties = [waypoints[i:i + chunk_size] for i in range(0, len(waypoints), chunk_size)]
        for idx, sortie in enumerate(sorties, start=1):
            sortie_dist = total_path_distance_m(sortie)
            sortie_time = (sortie_dist / CRUISE_SPEED_MPS) / 60
            print(f"   Sortie {idx}: {len(sortie)} waypoints, "
                  f"~{sortie_time:.1f} min flight time")
        return sorties


if __name__ == "__main__":
    HOME_LAT = 47.397606
    HOME_LON = 8.543060

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

    print("\nFirst sortie's waypoints:")
    for i, (lat, lon, alt) in enumerate(sorties[0], start=1):
        print(f"  {i}. lat={lat:.6f}, lon={lon:.6f}, alt={alt:.0f}m")
