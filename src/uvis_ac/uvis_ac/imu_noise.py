"""Gazebo IMU noise, the model EUFS turns on.

gz-sim-imu-system adds independent gaussian noise on each axis:

    measured = truth + bias + N(mean, stddev)

The bias is drawn once when the sensor starts. EUFS sets the mean to 0
and the standard deviation to 0.0002 rad/s on angular velocity and
0.01 m/s² on linear acceleration, and it leaves the bias at 0. White
noise that small integrates to about a hundredth of a degree over a
minute, so a lap still closes. The bias standard deviations here are
the term EUFS left out. They are not a datasheet for a named IMU.

The map pose and the wheel speeds are not passed through this.
"""

import math
import random

# EUFS sensor imu in world_from_csv.py. Per sample, same as Gazebo.
GYRO_WHITE = 0.0002  # rad/s
ACCEL_WHITE = 0.01  # m/s^2

# Turn-on bias, one draw per axis. 0.002 rad/s is about 0.11 deg/s.
GYRO_BIAS = 0.002  # rad/s
ACCEL_BIAS = 0.05  # m/s^2


class ImuNoise:
    """One sensor life. The bias stays put. White noise is a new draw."""

    def __init__(self, rng=None, enabled=True,
                 gyro_white=GYRO_WHITE, accel_white=ACCEL_WHITE,
                 gyro_bias=GYRO_BIAS, accel_bias=ACCEL_BIAS):
        self.enabled = bool(enabled)
        self._rng = rng if rng is not None else random.Random()
        self._gyro_white = float(gyro_white)
        self._accel_white = float(accel_white)
        if self.enabled:
            self.gyro_bias = tuple(self._draw(gyro_bias) for _ in range(3))
            self.accel_bias = tuple(self._draw(accel_bias) for _ in range(3))
        else:
            self.gyro_bias = (0.0, 0.0, 0.0)
            self.accel_bias = (0.0, 0.0, 0.0)

    def _draw(self, stddev):
        stddev = float(stddev)
        if stddev == 0.0:
            return 0.0
        return self._rng.gauss(0.0, stddev)

    def _axis(self, value, bias, white):
        if not self.enabled:
            return float(value)
        noise = 0.0 if white == 0.0 else self._rng.gauss(0.0, white)
        return float(value) + bias + noise

    def apply(self, angular, linear):
        gyro = tuple(
            self._axis(value, bias, self._gyro_white)
            for value, bias in zip(angular, self.gyro_bias))
        accel = tuple(
            self._axis(value, bias, self._accel_white)
            for value, bias in zip(linear, self.accel_bias))
        return gyro, accel


def check():
    """The EUFS white levels, a fixed bias, and a clean bypass."""
    assert GYRO_WHITE == 0.0002
    assert ACCEL_WHITE == 0.01
    assert GYRO_BIAS == 0.002
    assert ACCEL_BIAS == 0.05

    off = ImuNoise(enabled=False)
    assert off.gyro_bias == (0.0, 0.0, 0.0)
    assert off.apply((1, 2, 3), (4, 5, 6)) == ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0))

    quiet = ImuNoise(
        rng=random.Random(0), gyro_white=0.0, accel_white=0.0,
        gyro_bias=0.0, accel_bias=0.0)
    assert quiet.apply((1.0, -2.0, 0.5), (0.0, 0.0, 9.81)) == (
        (1.0, -2.0, 0.5), (0.0, 0.0, 9.81))

    one = ImuNoise(rng=random.Random(4), gyro_white=0.0, accel_white=0.0)
    two = ImuNoise(rng=random.Random(4), gyro_white=0.0, accel_white=0.0)
    assert one.gyro_bias == two.gyro_bias
    assert one.accel_bias == two.accel_bias
    assert any(abs(value) > 1e-6 for value in one.gyro_bias)
    held = one.apply((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert held == one.apply((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert held[0] == one.gyro_bias

    white = ImuNoise(
        rng=random.Random(2), gyro_bias=0.0, accel_bias=0.0)
    samples = [white.apply((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))[0][2] for _ in range(4000)]
    mean = sum(samples) / len(samples)
    variance = sum((sample - mean) ** 2 for sample in samples) / len(samples)
    assert abs(mean) < 2e-5
    assert abs(math.sqrt(variance) - GYRO_WHITE) < 2e-5
    print('imu noise check ok')


if __name__ == '__main__':
    check()
