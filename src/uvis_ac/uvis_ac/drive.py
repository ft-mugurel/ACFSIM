"""Map a pure-pursuit command onto Assetto Corsa pedals and steering.

The MFT C3 lock is 110° at the wheel and the steering ratio is 2.74, so the
road-wheel limit is about 40°. Positive ROS steering is a left turn. The
game's positive steer turns right, so the value written to the game is negated.
"""

import math

STEER_LOCK_DEG = 110.0
STEER_RATIO = 2.74
SPEED_GAIN = 0.5
# A driver can flick the wheel near 180 RPM. Drive-by-wire is slower, so
# the actuator is limited to 90 RPM. This wheel is 270° from lock to lock.
HANDWHEEL_RPM = 90.0
LOCK_TO_LOCK_DEG = 270.0


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


def steer_rate(rpm=HANDWHEEL_RPM, lock_to_lock_deg=LOCK_TO_LOCK_DEG):
    """Max change per second of the game steer, which is −1..1 lock to lock.

    90 RPM is 540°/s. The full −1..1 range is 270°, so the limit is
    540/270×2 = 4 per second, and lock to lock takes 0.5 s.
    """
    if rpm <= 0.0 or lock_to_lock_deg <= 0.0:
        return 0.0
    return (rpm * 360.0 / 60.0) / lock_to_lock_deg * 2.0


def approach(current, target, max_step):
    if max_step <= 0.0:
        return current
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target
