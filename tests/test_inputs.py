"""Test input scenarios for parking-stop planner."""

from parking_stop_planner.models import Pose2D

# ---------------------------------------------------------------------------
# Stop scenarios
# ---------------------------------------------------------------------------

STOP_CASE_1 = {
    "name": "basic_stop_case",
    "start": Pose2D(x=0.0, y=0.0, yaw=0.0),
    "goal":  Pose2D(x=5.0, y=0.0, yaw=0.0),
}

STOP_CASE_2 = {
    "name": "angled_stop_case",
    "start": Pose2D(x=0.0, y=0.0, yaw=0.0),
    "goal":  Pose2D(x=4.0, y=2.0, yaw=1.5708),   # 90 degrees
}

# ---------------------------------------------------------------------------
# Park scenarios
# ---------------------------------------------------------------------------

PARK_CASE_1 = {
    "name": "basic_park_case",
    "start": Pose2D(x=0.0,  y=0.0,  yaw=0.0),
    "goal":  Pose2D(x=6.0,  y=3.0,  yaw=1.5708),  # 90 degrees
}

PARK_CASE_2 = {
    "name": "reverse_park_case",
    "start": Pose2D(x=0.0,  y=0.0,  yaw=0.0),
    "goal":  Pose2D(x=-4.0, y=3.0,  yaw=3.1416),  # 180 degrees
}

ALL_CASES = [STOP_CASE_1, STOP_CASE_2, PARK_CASE_1, PARK_CASE_2]
