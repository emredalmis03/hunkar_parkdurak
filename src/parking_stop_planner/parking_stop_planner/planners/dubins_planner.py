import math
from models import Pose2D, Waypoint, Gear, normalize_angle

#metre
MIN_TURNING_RADIUS = 5.0  
SAMPLE_STEP_M = 0.2


def _mod2pi(angle: float) -> float:
    return angle % (2 * math.pi)


def _calc_lsl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = 2 + d**2 - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))
    if p_sq < 0:
        return None
    
    p = math.sqrt(p_sq)
    tmp = math.atan2(math.cos(beta) - math.cos(alpha), d + math.sin(alpha) - math.sin(beta))
    t = _mod2pi(-alpha + tmp)
    q = _mod2pi(beta - tmp)
    return t, p, q


def _calc_rsr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    """Calculate lengths for Right-Straight-Right path."""
    p_sq = 2 + d**2 - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(beta) - math.sin(alpha))
    if p_sq < 0:
        return None
    
    p = math.sqrt(p_sq)
    tmp = math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
    t = _mod2pi(alpha - tmp)
    q = _mod2pi(-beta + tmp)
    return t, p, q


def _calc_rsl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = -2 + d**2 + 2 * math.cos(alpha - beta) - 2 * d * (math.sin(alpha) + math.sin(beta))
    if p_sq < 0:
        return None
    
    p = math.sqrt(p_sq)
    tmp = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta))
    t = _mod2pi(alpha - tmp + math.atan2(2, p))
    q = _mod2pi(beta - tmp + math.atan2(2, p))
    return t, p, q


def _calc_lsr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = -2 + d**2 + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) + math.sin(beta))
    if p_sq < 0:
        return None
    
    p = math.sqrt(p_sq)
    tmp = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta))
    t = _mod2pi(-alpha + tmp - math.atan2(-2, p))
    q = _mod2pi(-beta + tmp - math.atan2(-2, p))
    return t, p, q


def _calc_lrl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    tmp = (6.0 - d**2 + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(-alpha + math.atan2(math.cos(beta) - math.cos(alpha), d + math.sin(alpha) - math.sin(beta)) + p / 2)
    q = _mod2pi(beta - alpha - t + p)
    return t, p, q


def _calc_rlr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    tmp = (6.0 - d**2 + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(beta) - math.sin(alpha))) / 8.0
    if abs(tmp) > 1.0:
        return None
    
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(alpha - math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta)) + p / 2)
    q = _mod2pi(alpha - beta - t + p)
    return t, p, q


def _generate_trajectory(
    start_pose: Pose2D, 
    lengths: tuple[float, float, float], 
    types: str, 
    radius: float, 
    step_size: float
) -> list[Waypoint]:
    waypoints = []
    
    current_x = start_pose.x
    current_y = start_pose.y
    current_yaw = start_pose.yaw
    
    waypoints.append(Waypoint(current_x, current_y, current_yaw, Gear.FORWARD))

    for length, move_type in zip(lengths, types):
        segment_dist = length * radius 
        
        num_samples = int(math.floor(segment_dist / step_size))
        
        for _ in range(num_samples):
            if move_type == 'L':
                current_yaw += step_size / radius
            elif move_type == 'R':
                current_yaw -= step_size / radius
            
            current_x += step_size * math.cos(current_yaw)
            current_y += step_size * math.sin(current_yaw)
            
            waypoints.append(Waypoint(
                x=current_x, 
                y=current_y, 
                yaw=normalize_angle(current_yaw), 
                gear=Gear.FORWARD
            ))

    return waypoints


def plan(start_pose: Pose2D, goal_pose: Pose2D) -> list[Waypoint]:
    dx = goal_pose.x - start_pose.x
    dy = goal_pose.y - start_pose.y
    
    distance = math.hypot(dx, dy)
    d = distance / MIN_TURNING_RADIUS 
    theta = math.atan2(dy, dx)

    alpha = _mod2pi(start_pose.yaw - theta)
    beta = _mod2pi(goal_pose.yaw - theta)

    paths = {
        'LSL': _calc_lsl(alpha, beta, d),
        'RSR': _calc_rsr(alpha, beta, d),
        'RSL': _calc_rsl(alpha, beta, d),
        'LSR': _calc_lsr(alpha, beta, d),
        'LRL': _calc_lrl(alpha, beta, d),
        'RLR': _calc_rlr(alpha, beta, d)
    }

    best_type = None
    min_length = float('inf')
    best_lengths = None

    for path_type, path_lengths in paths.items():
        if path_lengths is not None:
            total_length = sum(path_lengths)
            if total_length < min_length:
                min_length = total_length
                best_lengths = path_lengths
                best_type = path_type

    print("best type:" + best_type)
    if best_type is None:
        return [
            Waypoint(start_pose.x, start_pose.y, start_pose.yaw, Gear.FORWARD),
            Waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, Gear.FORWARD)
        ]

    waypoints = _generate_trajectory(
        start_pose=start_pose,
        lengths=best_lengths,
        types=best_type,
        radius=MIN_TURNING_RADIUS,
        step_size=SAMPLE_STEP_M
    )

    waypoints.append(Waypoint(
        x=goal_pose.x,
        y=goal_pose.y,
        yaw=normalize_angle(goal_pose.yaw),
        gear=Gear.FORWARD
    ))

    return waypoints