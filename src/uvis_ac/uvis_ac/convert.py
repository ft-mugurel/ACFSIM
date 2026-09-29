"""Turn one Assetto Corsa sample into ROS body-frame values.

Assetto Corsa is Y-up. acc_g is the felt acceleration in g:
x lateral (negative while turning right), y up, z longitudinal
(negative while braking). The ROS IMU wants specific force in
x-forward, y-left, z-up.
"""

import math

G = 9.81


def specific_force(acc_g):
    lateral, vertical, longitudinal = acc_g
    return (
        -longitudinal * G,
        -lateral * G,
        vertical * G,
    )


def angular_velocity(local_angular_vel):
    # AC x is right, y is up, z is forward.
    right, up, forward = local_angular_vel
    return (forward, -right, up)


def world_to_map(car_coordinates):
    # AC (x right, y up, z forward) -> ROS map (x forward, y left, z up).
    right, up, forward = car_coordinates
    return (forward, -right, up)


def heading_to_yaw(heading_rad):
    return heading_rad


def pose_from_start(x, y, z, yaw, origin):
    """Subtract the first simulator position. Heading is left unchanged.

    Rotating the track so the car faces +X made the map slide around the
    lidar, because the view and the scan were no longer in the same frame.
    """
    ox, oy, oz = origin
    return (x - ox, y - oy, z - oz, yaw)
