# tests/goal_converter_inputs.py
#
# Raw input data for the goal converter.
# Only data is defined here — no math, no ROS, no planner.
# Converter output will be Pose2D (not the responsibility of this file).

# ─────────────────────────────────────────────
#  STOP GOALS
#  The vehicle will stop at these points.
#  yaw_deg: heading angle of the vehicle at the moment of stopping (in degrees)
# ─────────────────────────────────────────────

STOP_GOALS = [
    {"x": 5.0,  "y":  1.0, "yaw_deg":   0.0},   # straight ahead
    {"x": 8.0,  "y":  2.0, "yaw_deg":  15.0},   # slightly turned left
    {"x": 10.0, "y": -1.0, "yaw_deg": -10.0},   # slightly turned right
]

# ─────────────────────────────────────────────
#  PARK GOALS
#  The vehicle will park at these points.
#  yaw_deg: final heading angle at the parking position (in degrees)
# ─────────────────────────────────────────────

PARK_GOALS = [
    {"x": 2.0, "y": -1.0, "yaw_deg": -90.0},   # perpendicular park, exact 90°
    {"x": 3.5, "y": -1.2, "yaw_deg": -85.0},   # slightly angled park
    {"x": 4.0, "y": -0.8, "yaw_deg": -95.0},   # slightly angled the other way
]


# ─────────────────────────────────────────────
#  DEBUG — prints the lists when run directly
# ─────────────────────────────────────────────

if __name__ == "__main__":
    print("=== STOP GOALS ===")
    for i, goal in enumerate(STOP_GOALS, 1):
        print(f"  STOP_{i}: x={goal['x']}, y={goal['y']}, yaw={goal['yaw_deg']}°")

    print("\n=== PARK GOALS ===")
    for i, goal in enumerate(PARK_GOALS, 1):
        print(f"  PARK_{i}: x={goal['x']}, y={goal['y']}, yaw={goal['yaw_deg']}°")
