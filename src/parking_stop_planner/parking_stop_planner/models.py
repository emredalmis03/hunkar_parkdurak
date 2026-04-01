"""Minimal prototype models for parking-stop planner."""

from dataclasses import dataclass
from enum import Enum
from math import pi


def normalize_angle(angle_rad: float) -> float:
	"""Normalize an angle to the range [-pi, pi)."""
	return (angle_rad + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class Pose2D:
	"""Start or goal pose in 2D."""

	x: float
	y: float
	yaw: float


class Gear(Enum):
	"""Longitudinal direction for a waypoint."""

	FORWARD = "FORWARD"
	REVERSE = "REVERSE"


@dataclass(frozen=True)
class Waypoint:
	"""Common planner output point for Dubins and Reeds-Shepp."""

	x: float
	y: float
	yaw: float
	gear: Gear


@dataclass(frozen=True)
class VehicleGeometry:
	"""Vehicle dimensions used by planners."""

	wheel_base: float
	width: float
	length: float
	front_overhang: float
	rear_overhang: float


@dataclass(frozen=True)
class PlannerParameters:
	"""Core planner settings."""

	sample_step_m: float = 0.2
	goal_position_tolerance_m: float = 0.5
	goal_yaw_tolerance_rad: float = 0.2


__all__ = [
	"Gear",
	"PlannerParameters",
	"Pose2D",
	"VehicleGeometry",
	"Waypoint",
	"normalize_angle",
]

