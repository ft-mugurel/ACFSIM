"""Map a pure-pursuit command onto Assetto Corsa pedals and steering.

The MFT C3 lock is 110° at the wheel and the steering ratio is 2.74, so the
road-wheel limit is about 40°. Positive ROS steering is a left turn. The
game's positive steer turns right, so the value written to the game is negated.
"""

import math

STEER_LOCK_DEG = 110.0
STEER_RATIO = 2.74
SPEED_GAIN = 0.5


def pedals(target_m_s, speed_m_s, gain=SPEED_GAIN):
    error = target_m_s - speed_m_s
    if error >= 0.0:
        return min(1.0, gain * error), 0.0
    return 0.0, min(1.0, -gain * error)


def steer_input(steer_rad, lock_deg=STEER_LOCK_DEG, ratio=STEER_RATIO):
    road_max = math.radians(lock_deg / ratio)
    if road_max <= 0.0:
        return 0.0
    return min(1.0, max(-1.0, -steer_rad / road_max))
