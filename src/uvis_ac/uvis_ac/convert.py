"""Turn one Assetto Corsa sample into ROS body-frame values.

Assetto Corsa is Y-up. acc_g and local_angular_vel share one car frame:
x lateral (positive left, so a right turn reads negative), y up,
z longitudinal (positive forward, so braking reads negative). Gravity is
not included, so a parked car reads about zero.

Specific force has the same direction as that kinematic acceleration, plus
one g upward. Negating it makes a straight acceleration look like braking.
"""

import math

G = 9.81


def specific_force(acc_g):
    lateral, vertical, longitudinal = acc_g
    return (
        longitudinal * G,
        lateral * G,
        vertical * G + G,
    )


def body_velocity(local_velocity):
    """Car-frame velocity in m/s. Same axes as acc_g: x left, y up, z forward."""
    lateral, vertical, longitudinal = local_velocity
    return (longitudinal, lateral, vertical)


def specific_force_from_motion(velocity, previous, omega, dt):
    """Specific force from the car's own velocity.

    acc_g is not the derivative of speed. dv/dt plus the turning term is.
    """
    vx, vy, vz = velocity
    if previous is None or dt <= 1e-4:
        ax, ay, az = 0.0, 0.0, 0.0
    else:
        px, py, pz = previous
        ax = (vx - px) / dt
        ay = (vy - py) / dt
        az = (vz - pz) / dt
    wx, wy, wz = omega
    ax += wy * vz - wz * vy
    ay += wz * vx - wx * vz
    az += wx * vy - wy * vx
    return (ax, ay, az + G)


def angular_velocity(local_angular_vel):
    # AC car frame is x right, y up, z forward. ROS is x forward, y left, z up.
    right, up, forward = local_angular_vel
    return (forward, -right, up)


def _wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def body_rates(heading, previous_heading, dt):
    """Yaw rate from the game heading. Roll and pitch stay zero.

    The raw gyro axes tilt once the car is moving, and the body then falls
    behind and starts to circle. Yaw rate comes from the game heading.
    """
    if previous_heading is None or dt <= 1e-4:
        return (0.0, 0.0, 0.0)
    wz = _wrap(heading_to_yaw(heading) - heading_to_yaw(previous_heading)) / dt
    return (0.0, 0.0, wz)


def world_to_map(car_coordinates):
    # AC world is Y-up. At heading 0 the nose is +Z, and a positive heading
    # turns the nose toward -X (a right turn). ROS map X is that nose, and
    # ROS map Y is left, so AC X is kept rather than mirrored.
    ac_x, ac_y, ac_z = car_coordinates
    return (ac_z, ac_x, ac_y)


def heading_to_yaw(heading_rad):
    # Positive game heading is a right turn. Positive ROS yaw is a left turn.
    return -heading_rad


def pose_from_start(x, y, z, yaw, origin):
    """Subtract the first simulator position. Heading is left unchanged.

    Rotating the track so the car faces +X made the map slide around the
    lidar, because the view and the scan were no longer in the same frame.
    """
    ox, oy, oz = origin
    return (x - ox, y - oy, z - oz, yaw)
