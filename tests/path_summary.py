"""Utility to summarise a planner path for quick inspection."""

from typing import List

from parking_stop_planner.models import Gear, Waypoint


def print_path_summary(path: List[Waypoint], label: str = "Path") -> None:
    """Print a concise summary of a planner output path.

    Args:
        path:  List of Waypoint objects returned by a planner.
        label: Optional title shown in the summary header.
    """
    print(f"\n=== {label} ===")

    if not path:
        print("  [EMPTY] Planner returned no waypoints.")
        return

    total    = len(path)
    forward  = sum(1 for wp in path if wp.gear == Gear.FORWARD)
    reverse  = sum(1 for wp in path if wp.gear == Gear.REVERSE)
    first_wp = path[0]
    last_wp  = path[-1]

    print(f"  Total waypoints : {total}")
    print(f"  First waypoint  : x={first_wp.x:.3f}  y={first_wp.y:.3f}  yaw={first_wp.yaw:.3f}  gear={first_wp.gear.value}")
    print(f"  Last waypoint   : x={last_wp.x:.3f}  y={last_wp.y:.3f}  yaw={last_wp.yaw:.3f}  gear={last_wp.gear.value}")
    print(f"  Forward         : {forward}")
    print(f"  Reverse         : {reverse}")
