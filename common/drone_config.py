"""
drone_config.py — single source of truth for drone hardware constants.

CURRENT VALUES: QUADCOPTER PLACEHOLDERS (450 mm frame, 4S 5200 mAh, ~1.5 kg AUW).
These come from SAR_Drone_Hardware_Spec.md. The custom hexacopter airframe
(Fusion 360) is not finished yet. When it is, edit ONLY this file and every
phase picks up the new numbers.

Status: PLACEHOLDER — not the final airframe.
"""

DRONE_NAME = "quadcopter-placeholder"
NUM_MOTORS = 4

# ---- Energy ---------------------------------------------------------------
BATTERY_CAPACITY_MAH = 5200
USABLE_FRACTION = 0.8            # don't drain below 20 % (battery health)
HOVER_CURRENT_DRAW_A = 16.0      # estimated, from spec
CRUISE_CURRENT_DRAW_A = 12.0     # forward flight draws less than hover
SAFETY_MARGIN_FRACTION = 0.85    # plan only 85 % of theoretical endurance

# ---- Flight ---------------------------------------------------------------
CRUISE_SPEED_MPS = 8.0           # speed during search legs
SEARCH_ALTITUDE_M = 50.0         # AGL

# ---- Derived (do not edit) ------------------------------------------------
USABLE_CAPACITY_AH = (BATTERY_CAPACITY_MAH / 1000) * USABLE_FRACTION
MAX_FLIGHT_TIME_MIN = (USABLE_CAPACITY_AH / CRUISE_CURRENT_DRAW_A) * 60
PLANNABLE_FLIGHT_TIME_MIN = MAX_FLIGHT_TIME_MIN * SAFETY_MARGIN_FRACTION


if __name__ == "__main__":
    print(f"{DRONE_NAME}: {NUM_MOTORS} motors")
    print(f"Max cruise endurance: {MAX_FLIGHT_TIME_MIN:.1f} min, "
          f"plannable: {PLANNABLE_FLIGHT_TIME_MIN:.1f} min")
