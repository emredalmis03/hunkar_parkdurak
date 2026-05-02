"""Test report generator for parking-stop planner scenarios."""

from typing import Callable, List

from parking_stop_planner.models import Gear, Pose2D, Waypoint
from path_validation import validate_path


def generate_report(
    scenario: dict,
    planner_fn: Callable[[Pose2D, Pose2D], List[Waypoint]],
) -> None:
    """Run a single scenario through a planner and print a structured report.

    Args:
        scenario:   A scenario dict with keys 'name', 'start', and 'goal'.
        planner_fn: A callable that accepts (start: Pose2D, goal: Pose2D)
                    and returns a list of Waypoint objects.
    """
    name  = scenario["name"]
    start = scenario["start"]
    goal  = scenario["goal"]

    try:
        path = planner_fn(start, goal)
        is_valid, issues = validate_path(path)

        forward = sum(1 for wp in path if wp.gear == Gear.FORWARD)
        reverse = sum(1 for wp in path if wp.gear == Gear.REVERSE)
        result  = "SUCCESS" if (path and is_valid) else "FAIL"

        print(f"\nScenario      : {name}")
        print(f"Result        : {result}")
        print(f"Waypoint count: {len(path)}")
        print(f"Forward count : {forward}")
        print(f"Reverse count : {reverse}")

        if issues:
            print("Issues:")
            for issue in issues:
                print(f"  - {issue}")

    except Exception as exc:
        print(f"\nScenario      : {name}")
        print(f"Result        : ERROR")
        print(f"Error         : {exc}")


def run_all_reports(
    scenarios: List[dict],
    planner_fn: Callable[[Pose2D, Pose2D], List[Waypoint]],
    planner_name: str = "Planner",
) -> None:
    """Run all scenarios and print a full report for a given planner.

    Args:
        scenarios:    List of scenario dicts.
        planner_fn:   Planner callable (start, goal) -> List[Waypoint].
        planner_name: Display name used in the report header.
    """
    print(f"\n{'='*50}")
    print(f"  {planner_name} — Test Report")
    print(f"{'='*50}")

    for scenario in scenarios:
        generate_report(scenario, planner_fn)

    print(f"\n{'='*50}\n")
