"""Validation helpers for planner output paths."""

from math import hypot, pi
from typing import List, Tuple

from parking_stop_planner.models import Waypoint


def validate_path(
    path: List[Waypoint],
    max_position_jump_m: float = 1.0,
    max_yaw_jump_rad: float = pi / 4,
) -> Tuple[bool, List[str]]:
    """Validate a planner output path and return a list of issues found.

    Args:
        path:                List of Waypoint objects to validate.
        max_position_jump_m: Maximum allowed distance between consecutive waypoints (metres).
        max_yaw_jump_rad:    Maximum allowed yaw change between consecutive waypoints (radians).

    Returns:
        (is_valid, issues) where is_valid is True when no issues were found
        and issues is a list of human-readable problem descriptions.
    """
    issues: List[str] = []

    # Check 1: empty path
    if not path:
        issues.append("Path is empty — planner returned no waypoints.")
        return False, issues

    # Check 2: duplicate waypoints
    seen = set()
    for i, wp in enumerate(path):
        key = (round(wp.x, 6), round(wp.y, 6), round(wp.yaw, 6), wp.gear)
        if key in seen:
            issues.append(f"Duplicate waypoint at index {i}: x={wp.x:.3f} y={wp.y:.3f} yaw={wp.yaw:.3f}")
        seen.add(key)

    # Check 3: large position or yaw jumps between consecutive waypoints
    for i in range(len(path) - 1):
        current = path[i]
        nxt     = path[i + 1]

        dist = hypot(nxt.x - current.x, nxt.y - current.y)
        if dist > max_position_jump_m:
            issues.append(
                f"Large position jump between index {i} and {i+1}: {dist:.3f} m "
                f"(limit {max_position_jump_m} m)"
            )

        yaw_diff = abs(nxt.yaw - current.yaw)
        yaw_diff = min(yaw_diff, 2 * pi - yaw_diff)  # wrap to [0, pi]
        if yaw_diff > max_yaw_jump_rad:
            issues.append(
                f"Large yaw jump between index {i} and {i+1}: {yaw_diff:.3f} rad "
                f"(limit {max_yaw_jump_rad:.3f} rad)"
            )

    is_valid = len(issues) == 0
    return is_valid, issues


def print_validation_result(path: List[Waypoint], label: str = "Path") -> bool:
    """Print validation results to stdout and return True if path is valid.

    Args:
        path:  List of Waypoint objects to validate.
        label: Optional title shown in the output header.

    Returns:
        True if the path passed all checks, False otherwise.
    """
    is_valid, issues = validate_path(path)

    print(f"\n=== Validation: {label} ===")
    if is_valid:
        print("  [PASS] No issues found.")
    else:
        print(f"  [FAIL] {len(issues)} issue(s) found:")
        for issue in issues:
            print(f"    - {issue}")

    return is_valid
