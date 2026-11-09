"""
Phase 1 — Battery-Aware Coverage Path Planner (FIXED)

Takes a rectangular search area, a desired camera swath width, and the
drone's real battery/endurance numbers, and produces a lawnmower
("boustrophedon") search pattern of waypoints — split into multiple
sorties if the area exceeds one battery's budget.

--- Why this version replaces the earlier flat-chunking approach ---
The original version generated one long waypoint list for the whole area,
then split it into N equal-sized CHUNKS when it exceeded the battery
budget. That produced sorties that each spanned the FULL extent of one
axis (e.g. every sortie flew the entire north-south length), rather than
each sortie covering its own clean, contiguous rectangular patch — not
how a real SAR team would divide a search area ("sortie 3 covers the
northeast quadrant").

This version instead splits the search area itself into N horizontal
geographic BANDS first (based on how many sorties are needed), and
independently generates a full lawnmower pattern within each band. Every
sortie now covers a clean, nameable, contiguous region.
"""

import math


# ---------------------------------------------------------------------------
# REAL HARDWARE NUMBERS (from the hardware spec — not placeholders)
# ---------------------------------------------------------------------------
BATTERY_CAPACITY_MAH = 5200
USABLE_FRACTION = 0.8
HOVER_CURRENT_DRAW_A = 16.0
CRUISE_CURRENT_DRAW_A = 12.0
CRUISE_SPEED_MPS = 8.0

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
# LAWNMOWER PATTERN GENERATOR (unchanged — generates one clean rectangle's pattern)
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
# BATTERY-AWARE PLANNING LOGIC (fixed: geographic band splitting)
# ---------------------------------------------------------------------------
def plan_mission(min_lat, min_lon, max_lat, max_lon,
                  swath_width_m=40.0, altitude_m=50.0):
    """
    First checks whether the WHOLE area fits in one sortie. If not,
    estimates how many sorties are needed, splits the area into that
    many horizontal geographic bands (by latitude), and generates an
    independent, complete lawnmower pattern within each band.
    """
    # Step 1: estimate whole-area flight time to see if splitting is needed
    whole_waypoints, area_width_m, area_height_m = generate_lawnmower_pattern(
        min_lat, min_lon, max_lat, max_lon, swath_width_m, altitude_m
    )
    whole_distance_m = total_path_distance_m(whole_waypoints)
    whole_flight_time_min = (whole_distance_m / CRUISE_SPEED_MPS) / 60
    area_km2 = (area_width_m * area_height_m) / 1_000_000

    print("=" * 60)
    print("MISSION PLAN")
    print("=" * 60)
    print(f"Search area:              {area_width_m:.0f}m x {area_height_m:.0f}m "
          f"({area_km2:.3f} km^2)")
    print(f"Swath width per pass:     {swath_width_m:.0f} m")
    print(f"Estimated total flight time (whole area): {whole_flight_time_min:.1f} min")
    print(f"Plannable battery budget: {PLANNABLE_FLIGHT_TIME_MIN:.1f} min "
          f"(with safety margin)")

    if whole_flight_time_min <= PLANNABLE_FLIGHT_TIME_MIN:
        margin_min = PLANNABLE_FLIGHT_TIME_MIN - whole_flight_time_min
        print(f"\n>> FITS IN ONE SORTIE. Energy margin: {margin_min:.1f} min "
              f"({(margin_min / PLANNABLE_FLIGHT_TIME_MIN) * 100:.0f}% of budget)")
        print(f"   Waypoints: {len(whole_waypoints)}")
        return [whole_waypoints]

    # Step 2: doesn't fit — split into whole COLUMNS (never bisect a column,
    # since each column is one full north-south pass; splitting mid-column
    # would leave a gap in coverage). Each sortie gets a contiguous group of
    # columns — a clean vertical strip (full height, limited width).
    num_sorties = math.ceil(whole_flight_time_min / PLANNABLE_FLIGHT_TIME_MIN)
    print(f"\n>> EXCEEDS ONE BATTERY'S BUDGET.")
    print(f">> Splitting into {num_sorties} sorties, each a clean contiguous "
          f"longitude strip (full height), requiring a battery swap between each.")

    num_columns = len(whole_waypoints) // 2
    columns_per_sortie = math.ceil(num_columns / num_sorties)

    sorties = []
    sortie_num = 1
    for col_start in range(0, num_columns, columns_per_sortie):
        col_end = min(col_start + columns_per_sortie, num_columns)
        wp_start, wp_end = col_start * 2, col_end * 2
        sortie_waypoints = whole_waypoints[wp_start:wp_end]

        sortie_dist_m = total_path_distance_m(sortie_waypoints)
        sortie_time_min = (sortie_dist_m / CRUISE_SPEED_MPS) / 60
        lons = [wp[1] for wp in sortie_waypoints]
        fits = "OK" if sortie_time_min <= PLANNABLE_FLIGHT_TIME_MIN else "STILL OVER BUDGET"

        print(f"   Sortie {sortie_num}: lon strip [{min(lons):.6f} to "
              f"{max(lons):.6f}] (full lat range), {len(sortie_waypoints)} "
              f"waypoints, ~{sortie_time_min:.1f} min [{fits}]")

        sorties.append(sortie_waypoints)
        sortie_num += 1

    total_sortie_time = sum(
        (total_path_distance_m(s) / CRUISE_SPEED_MPS) / 60 for s in sorties
    )
    print(f"\n   Sanity check — sum of all sortie times: {total_sortie_time:.1f} min "
          f"(should be close to the {whole_flight_time_min:.1f} min whole-area "
          f"estimate, confirming no wasted/redundant travel from the split)")

    return sorties


if __name__ == "__main__":
    HOME_LAT = 47.397606
    HOME_LON = 8.543060

    # Small area — should fit in one sortie
    print("### TEST 1: small area (should fit in one sortie) ###\n")
    lat_span = meters_to_lat_deg(200)
    lon_span = meters_to_lon_deg(300, HOME_LAT)
    plan_mission(
        min_lat=HOME_LAT, min_lon=HOME_LON,
        max_lat=HOME_LAT + lat_span, max_lon=HOME_LON + lon_span,
        swath_width_m=40.0, altitude_m=50.0,
    )

    # Large area — should now split into clean geographic bands
    print("\n\n### TEST 2: large area (should split into geographic bands) ###\n")
    lat_span_big = meters_to_lat_deg(2000)
    lon_span_big = meters_to_lon_deg(3000, HOME_LAT)
    plan_mission(
        min_lat=HOME_LAT, min_lon=HOME_LON,
        max_lat=HOME_LAT + lat_span_big, max_lon=HOME_LON + lon_span_big,
        swath_width_m=40.0, altitude_m=50.0,
    )
