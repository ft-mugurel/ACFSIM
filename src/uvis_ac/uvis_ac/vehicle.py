"""Bicycle model from the Assetto Corsa wheel speeds and handwheel.

Wheel order is front-left, front-right, rear-left, rear-right. The game's
steerAngle is the handwheel in radians, positive to the right. The road-wheel
angle here is positive to the left, matching the rest of the stack.
"""

import math

# Front tyre RADIUS from madformulateam_mftc3 data/tyres.ini. The rear tyre is 0.2062 m.
WHEEL_RADIUS = 0.20574
WHEELBASE = 1.580
STEER_RATIO = 2.74


def road_wheel_angle(handwheel_rad, ratio=STEER_RATIO):
    if ratio == 0.0:
        return 0.0
    return -float(handwheel_rad) / ratio


def forward_speed(wheel_rad_s, radius=WHEEL_RADIUS, road_rad=0.0):
    """Forward speed from the front axle.

    The car is rear-wheel drive with a locked differential, so the rear tires
    slip and their average is not the car speed. The front tires roll freely.
    They steer, so their rolling speed is the car speed divided by cos of the
    road-wheel angle.
    """
    front = 0.5 * (float(wheel_rad_s[0]) + float(wheel_rad_s[1]))
    return front * radius * math.cos(road_rad)


def yaw_rate(speed, road_rad, wheelbase=WHEELBASE):
    if wheelbase == 0.0:
        return 0.0
    return speed * math.tan(road_rad) / wheelbase
