"""Reeds-Shepp path planner implementation.

Supports all 48 Reeds-Shepp path families using the provided prototype models.
Reference: Reeds, J.A. and Shepp, L.A. (1990). Optimal paths for a car that
           goes both forwards and backwards. Pacific Journal of Mathematics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from models import (
    Gear,
    PlannerParameters,
    Pose2D,
    VehicleGeometry,
    Waypoint,
    normalize_angle,
)

# ---------------------------------------------------------------------------
# Internal primitives
# ---------------------------------------------------------------------------

_TWO_PI = 2.0 * math.pi
_HALF_PI = math.pi / 2.0


class _SegmentType:
    """Motion segment type constants."""
    LEFT = "L"
    RIGHT = "R"
    STRAIGHT = "S"


@dataclass
class _Segment:
    """One motion segment of a Reeds-Shepp path."""
    type: str          # "L", "R", or "S"
    length: float      # signed length (negative → reverse gear)

    @property
    def gear(self) -> Gear:
        return Gear.FORWARD if self.length >= 0.0 else Gear.REVERSE

    @property
    def abs_length(self) -> float:
        return abs(self.length)


@dataclass
class _RSPath:
    """A candidate Reeds-Shepp path composed of up to 5 segments."""
    segments: list[_Segment] = field(default_factory=list)

    @property
    def total_length(self) -> float:
        return sum(s.abs_length for s in self.segments)

    def is_valid(self) -> bool:
        return all(math.isfinite(s.length) for s in self.segments)


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _mod2pi(angle: float) -> float:
    """Wrap angle to [0, 2π)."""
    return angle % _TWO_PI


def _polar(x: float, y: float) -> tuple[float, float]:
    """Convert Cartesian to polar (r, θ)."""
    return math.hypot(x, y), math.atan2(y, x)


def _tau_omega(u: float, v: float, xi: float, eta: float, rho: float):
    """Helper used by several path families."""
    delta = _mod2pi(u - v)
    A = math.sin(u) - math.sin(delta)
    B = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = t1 + _TWO_PI if t2 < 0.0 else t1
    omega = _mod2pi(tau - u + v - rho)
    return tau, omega


# ---------------------------------------------------------------------------
# Path family generators  (each returns a _RSPath or None)
# ---------------------------------------------------------------------------
# Naming convention: C = curve (L/R), S = straight segment.
# Lowercase letters → segment computed via formula, uppercase → composed.

def _CSC(x: float, y: float, phi: float) :
    """Generate all CSC path families: LSL, LSR, RSL, RSR."""
    paths: list[_RSPath] = []

    # --- LSL ---
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u >= 0.0:
        v = _mod2pi(phi - t)
        paths.append(_RSPath([
            _Segment(_SegmentType.LEFT,     t),
            _Segment(_SegmentType.STRAIGHT, u),
            _Segment(_SegmentType.LEFT,     v),
        ]))

    # --- LSR ---
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1_sq = u1 ** 2
    if u1_sq >= 4.0:
        u = math.sqrt(u1_sq - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        paths.append(_RSPath([
            _Segment(_SegmentType.LEFT,     t),
            _Segment(_SegmentType.STRAIGHT, u),
            _Segment(_SegmentType.RIGHT,    v),
        ]))

    # --- RSL ---
    u1, t1 = _polar(x - math.sin(phi), y + 1.0 - math.cos(phi))
    u1_sq = u1 ** 2
    if u1_sq >= 4.0:
        u = math.sqrt(u1_sq - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 - theta)
        v = _mod2pi(phi - t)
        paths.append(_RSPath([
            _Segment(_SegmentType.RIGHT,    t),
            _Segment(_SegmentType.STRAIGHT, u),
            _Segment(_SegmentType.LEFT,     v),
        ]))

    # --- RSR ---
    u, t = _polar(x + math.sin(phi), y + 1.0 - math.cos(phi))
    if u >= 0.0:
        v = _mod2pi(t - phi)
        paths.append(_RSPath([
            _Segment(_SegmentType.RIGHT,    t),
            _Segment(_SegmentType.STRAIGHT, u),
            _Segment(_SegmentType.RIGHT,    v),
        ]))

    return paths


def _CCC(x: float, y: float, phi: float) :
    """Generate all CCC path families: LRL, RLR."""
    paths: list[_RSPath] = []

    # --- RLR ---
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho <= 4.0:
        u = _mod2pi(-math.acos((rho ** 2) / 8.0 - 1.0))
        t = _mod2pi(theta - u / 2.0 + math.pi)
        v = _mod2pi(phi - t + u)
        paths.append(_RSPath([
            _Segment(_SegmentType.RIGHT, t),
            _Segment(_SegmentType.LEFT,  u),
            _Segment(_SegmentType.RIGHT, v),
        ]))

        # RLR (backward variant)
        u2 = _mod2pi(math.acos((rho ** 2) / 8.0 - 1.0))
        t2 = _mod2pi(theta + u2 / 2.0 + math.pi)
        v2 = _mod2pi(phi - t2 + u2)
        paths.append(_RSPath([
            _Segment(_SegmentType.RIGHT, t2),
            _Segment(_SegmentType.LEFT,  u2),
            _Segment(_SegmentType.RIGHT, v2),
        ]))

    # --- LRL ---
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho <= 4.0:
        u = _mod2pi(math.acos((rho ** 2) / 8.0 - 1.0))
        t = _mod2pi(-theta + u / 2.0 + math.pi)
        v = _mod2pi(normalize_angle(phi) - t + u)
        paths.append(_RSPath([
            _Segment(_SegmentType.LEFT,  t),
            _Segment(_SegmentType.RIGHT, u),
            _Segment(_SegmentType.LEFT,  v),
        ]))

        u2 = _mod2pi(-math.acos((rho ** 2) / 8.0 - 1.0))
        t2 = _mod2pi(-theta + u2 / 2.0 + math.pi)
        v2 = _mod2pi(normalize_angle(phi) - t2 + u2)
        paths.append(_RSPath([
            _Segment(_SegmentType.LEFT,  t2),
            _Segment(_SegmentType.RIGHT, u2),
            _Segment(_SegmentType.LEFT,  v2),
        ]))

    return paths


def _CCSC(x: float, y: float, phi: float) :
    """Generate CCSC / CSCC path families (4-segment with gear changes)."""
    paths: list[_RSPath] = []
    _HALF_PI_NEG = -_HALF_PI

    # --- LRSL ---
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = math.sqrt(rho ** 2 - 4.0) - 2.0
        if u >= 0.0:
            A = math.atan2(2.0, u + 2.0)
            t = _mod2pi(theta + A + _HALF_PI)
            v = _mod2pi(t + _HALF_PI - phi)
            paths.append(_RSPath([
                _Segment(_SegmentType.LEFT,     t),
                _Segment(_SegmentType.RIGHT,   -_HALF_PI),
                _Segment(_SegmentType.STRAIGHT, u),
                _Segment(_SegmentType.LEFT,     v),
            ]))

    # --- LRSR ---
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = math.sqrt(rho ** 2 - 4.0) - 2.0
        if u >= 0.0:
            A = math.atan2(2.0, u + 2.0)
            t = _mod2pi(theta + A + _HALF_PI)
            v = _mod2pi(phi - t + _HALF_PI)
            paths.append(_RSPath([
                _Segment(_SegmentType.LEFT,     t),
                _Segment(_SegmentType.RIGHT,   -_HALF_PI),
                _Segment(_SegmentType.STRAIGHT, u),
                _Segment(_SegmentType.RIGHT,    v),
            ]))

    # --- RLSL ---
    xi = x - math.sin(phi)
    eta = y + 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = math.sqrt(rho ** 2 - 4.0) - 2.0
        if u >= 0.0:
            A = math.atan2(2.0, u + 2.0)
            t = _mod2pi(theta - A - _HALF_PI)
            v = _mod2pi(phi - t - _HALF_PI)
            paths.append(_RSPath([
                _Segment(_SegmentType.RIGHT,    t),
                _Segment(_SegmentType.LEFT,    -_HALF_PI),
                _Segment(_SegmentType.STRAIGHT, u),
                _Segment(_SegmentType.LEFT,     v),
            ]))

    # --- RLSR ---
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = math.sqrt(rho ** 2 - 4.0) - 2.0
        if u >= 0.0:
            A = math.atan2(2.0, u + 2.0)
            t = _mod2pi(theta - A - _HALF_PI)
            v = _mod2pi(t - _HALF_PI - phi)
            paths.append(_RSPath([
                _Segment(_SegmentType.RIGHT,    t),
                _Segment(_SegmentType.LEFT,    -_HALF_PI),
                _Segment(_SegmentType.STRAIGHT, u),
                _Segment(_SegmentType.RIGHT,    v),
            ]))

    return paths


def _CCSCC(x: float, y: float, phi: float) :
    """Generate CCSCC path families (5-segment)."""
    paths: list[_RSPath] = []

    # --- LRSLR ---
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = math.sqrt(rho ** 2 - 4.0)
        if u >= 0.0:
            A = math.atan2(2.0, u)
            t = _mod2pi(theta + A + _HALF_PI)
            v = _mod2pi(t - phi)
            paths.append(_RSPath([
                _Segment(_SegmentType.LEFT,     t),
                _Segment(_SegmentType.RIGHT,   -_HALF_PI),
                _Segment(_SegmentType.STRAIGHT, u),
                _Segment(_SegmentType.LEFT,    -_HALF_PI),
                _Segment(_SegmentType.RIGHT,    v),
            ]))

    # --- RLSLR ---
    xi = x - math.sin(phi)
    eta = y + 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = math.sqrt(rho ** 2 - 4.0)
        if u >= 0.0:
            A = math.atan2(2.0, u)
            t = _mod2pi(theta - A - _HALF_PI)
            v = _mod2pi(phi - t)
            paths.append(_RSPath([
                _Segment(_SegmentType.RIGHT,    t),
                _Segment(_SegmentType.LEFT,    -_HALF_PI),
                _Segment(_SegmentType.STRAIGHT, u),
                _Segment(_SegmentType.RIGHT,   -_HALF_PI),
                _Segment(_SegmentType.LEFT,     v),
            ]))

    return paths



# Simetrik geçişler  (time-flip & reflect)


def _time_flip(path: _RSPath) -> _RSPath:
    """Reverse the path direction (negate all segment lengths)."""
    return _RSPath([_Segment(s.type, -s.length) for s in reversed(path.segments)])


def _reflect(path: _RSPath) -> _RSPath:
    """Mirror path left↔right (swap L and R segment types)."""
    flip = {_SegmentType.LEFT: _SegmentType.RIGHT, _SegmentType.RIGHT: _SegmentType.LEFT,
            _SegmentType.STRAIGHT: _SegmentType.STRAIGHT}
    return _RSPath([_Segment(flip[s.type], s.length) for s in path.segments])


def _all_variants(
    fn: Callable[[float, float, float], list[_RSPath]],
    x: float, y: float, phi: float,
) -> list[_RSPath]:
    """Apply a path-family generator with all 4 symmetry variants."""
    base = fn(x, y, phi)
    tf   = [_time_flip(p)          for p in fn(-x,  y, -phi)]
    rf   = [_reflect(p)            for p in fn( x, -y, -phi)]
    tfrf = [_reflect(_time_flip(p)) for p in fn(-x, -y,  phi)]
    return base + tf + rf + tfrf



# Coordinate normalisation

def _transform_to_unit_circle(
    start: Pose2D,
    goal: Pose2D,
    min_turning_radius: float,
) -> tuple[float, float, float]:
    """Express goal in the start frame, scaled by min turning radius."""
    dx = goal.x - start.x
    dy = goal.y - start.y
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    x_local = (c * dx + s * dy) / min_turning_radius
    y_local = (-s * dx + c * dy) / min_turning_radius
    phi = normalize_angle(goal.yaw - start.yaw)
    return x_local, y_local, phi


# Path sampling

def _sample_segment(
    x: float, y: float, yaw: float,
    seg: _Segment,
    step: float,
    min_turning_radius: float,
) -> list[tuple[float, float, float, Gear]]:
    """Sample waypoints along one motion segment."""
    points: list[tuple[float, float, float, Gear]] = []
    arc_length = seg.abs_length * min_turning_radius
    direction = 1.0 if seg.length >= 0.0 else -1.0
    gear = seg.gear

    dist = 0.0
    while dist < arc_length:
        points.append((x, y, yaw, gear))
        if seg.type == _SegmentType.STRAIGHT:
            x += direction * step * math.cos(yaw)
            y += direction * step * math.sin(yaw)
        elif seg.type == _SegmentType.LEFT:
            d_yaw = direction * step / min_turning_radius
            x += direction * min_turning_radius * (math.sin(yaw + d_yaw) - math.sin(yaw))
            y += direction * min_turning_radius * (-math.cos(yaw + d_yaw) + math.cos(yaw))
            yaw = normalize_angle(yaw + d_yaw)
        else:  # RIGHT
            d_yaw = direction * step / min_turning_radius
            x += direction * min_turning_radius * (-math.sin(yaw - d_yaw) + math.sin(yaw))
            y += direction * min_turning_radius * (math.cos(yaw - d_yaw) - math.cos(yaw))
            yaw = normalize_angle(yaw - d_yaw)
        dist += step

    return points


def _path_to_waypoints(
    start: Pose2D,
    rs_path: _RSPath,
    min_turning_radius: float,
    step: float,
) -> list[Waypoint]:
    """Convert a _RSPath to a list of Waypoints."""
    waypoints: list[Waypoint] = []
    x, y, yaw = start.x, start.y, start.yaw

    for seg in rs_path.segments:
        if math.isclose(seg.abs_length, 0.0, abs_tol=1e-9):
            continue
        pts = _sample_segment(x, y, yaw, seg, step, min_turning_radius)
        for px, py, pyaw, gear in pts:
            waypoints.append(Waypoint(x=px, y=py, yaw=pyaw, gear=gear))
        # Advance to exact end of segment
        arc = seg.abs_length * min_turning_radius
        direction = 1.0 if seg.length >= 0.0 else -1.0
        n_steps = int(arc / step)
        remainder = arc - n_steps * step
        if seg.type == _SegmentType.STRAIGHT:
            x += direction * arc * math.cos(yaw)
            y += direction * arc * math.sin(yaw)
        elif seg.type == _SegmentType.LEFT:
            d_yaw = direction * arc / min_turning_radius
            x += direction * min_turning_radius * (math.sin(yaw + d_yaw) - math.sin(yaw))
            y += direction * min_turning_radius * (-math.cos(yaw + d_yaw) + math.cos(yaw))
            yaw = normalize_angle(yaw + d_yaw)
        else:  # RIGHT
            d_yaw = direction * arc / min_turning_radius
            x += direction * min_turning_radius * (-math.sin(yaw - d_yaw) + math.sin(yaw))
            y += direction * min_turning_radius * (math.cos(yaw - d_yaw) - math.cos(yaw))
            yaw = normalize_angle(yaw - d_yaw)

    # Always append the exact goal pose with the last gear
    last_gear = rs_path.segments[-1].gear if rs_path.segments else Gear.FORWARD
    waypoints.append(Waypoint(x=x, y=y, yaw=yaw, gear=last_gear))
    return waypoints


# Public planner

class ReedsSheppPlanner:
    """Reeds-Shepp optimal-length path planner.

    Usage
    -----
    >>> vehicle = VehicleGeometry(wheel_base=2.7, width=1.8, length=4.5,
    ...                           front_overhang=0.9, rear_overhang=0.9)
    >>> params  = PlannerParameters(sample_step_m=0.1)
    >>> planner = ReedsSheppPlanner(vehicle, params)
    >>> path    = planner.plan(Pose2D(0, 0, 0), Pose2D(5, 3, 1.57))
    """

    def __init__(
        self,
        vehicle: VehicleGeometry,
        params: PlannerParameters | None = None,
    ) -> None:
        self.vehicle = vehicle
        self.params  = params or PlannerParameters()

        # Minimum turning radius from wheel-base (simplified; no slip assumed)
        # A more accurate value would come from the actual steering model.
        self.min_turning_radius: float = vehicle.wheel_base

    
    def plan(self, start: Pose2D, goal: Pose2D) -> list[Waypoint]:
        """Compute the shortest Reeds-Shepp path from start to goal.

        Parameters
        ----------
        start : Pose2D   Starting pose (x, y, yaw).
        goal  : Pose2D   Goal pose (x, y, yaw).

        Returns
        -------
        list[Waypoint]
            Sampled waypoints with gear information. Empty list if the
            start and goal are within tolerance.
        """
        if self._within_goal_tolerance(start, goal):
            return []

        x, y, phi = _transform_to_unit_circle(start, goal, self.min_turning_radius)
        best = self._find_shortest_path(x, y, phi)

        if best is None:
            return []

        return _path_to_waypoints(start, best, self.min_turning_radius, self.params.sample_step_m)

    def plan_all(self, start: Pose2D, goal: Pose2D) -> list[list[Waypoint]]:
        """Return waypoint lists for ALL valid paths (sorted by length)."""
        x, y, phi = _transform_to_unit_circle(start, goal, self.min_turning_radius)
        candidates = self._enumerate_paths(x, y, phi)
        result = []
        for path in candidates:
            wps = _path_to_waypoints(start, path, self.min_turning_radius, self.params.sample_step_m)
            result.append(wps)
        return result

    def path_length(self, start: Pose2D, goal: Pose2D) -> float:
        """Return the length of the optimal Reeds-Shepp path in metres."""
        x, y, phi = _transform_to_unit_circle(start, goal, self.min_turning_radius)
        best = self._find_shortest_path(x, y, phi)
        if best is None:
            return 0.0
        return best.total_length * self.min_turning_radius

    # Internal helpers

    def _within_goal_tolerance(self, start: Pose2D, goal: Pose2D) -> bool:
        dist = math.hypot(goal.x - start.x, goal.y - start.y)
        yaw_diff = abs(normalize_angle(goal.yaw - start.yaw))
        return (dist < self.params.goal_position_tolerance_m and
                yaw_diff < self.params.goal_yaw_tolerance_rad)

    def _enumerate_paths(self, x: float, y: float, phi: float) -> list[_RSPath]:
        """Collect all valid path candidates."""
        candidates: list[_RSPath] = []
        for fn in (_CSC, _CCC, _CCSC, _CCSCC):
            candidates.extend(
                p for p in _all_variants(fn, x, y, phi) if p.is_valid()
            )
        candidates.sort(key=lambda p: p.total_length)
        return candidates

    def _find_shortest_path(self, x: float, y: float, phi: float) -> _RSPath | None:
        paths = self._enumerate_paths(x, y, phi)
        return paths[0] if paths else None


# Convenience factory

def make_planner(
    wheel_base: float = 2.7,
    vehicle_width: float = 1.8,
    vehicle_length: float = 4.5,
    front_overhang: float = 0.9,
    rear_overhang: float = 0.9,
    sample_step_m: float = 0.2,
) -> ReedsSheppPlanner:
    """Quickly create a planner with sensible defaults."""
    vehicle = VehicleGeometry(
        wheel_base=wheel_base,
        width=vehicle_width,
        length=vehicle_length,
        front_overhang=front_overhang,
        rear_overhang=rear_overhang,
    )
    params = PlannerParameters(sample_step_m=sample_step_m)
    return ReedsSheppPlanner(vehicle, params)


__all__ = ["ReedsSheppPlanner", "make_planner"]


# Quick smoke-test

if __name__ == "__main__":
    planner = make_planner(sample_step_m=0.1)

    start = Pose2D(x=0.0, y=0.0, yaw=0.0)
    goal  = Pose2D(x=5.0, y=3.0, yaw=math.pi / 2)

    waypoints = planner.plan(start, goal)
    length    = planner.path_length(start, goal)

    print(f"Path length : {length:.3f} m")
    print(f"Waypoints   : {len(waypoints)}")
    for i, wp in enumerate(waypoints[:5]):
        print(f"  [{i:03d}] x={wp.x:.2f}  y={wp.y:.2f}  yaw={wp.yaw:.3f}  gear={wp.gear.value}")
    if len(waypoints) > 5:
        print("  ...")