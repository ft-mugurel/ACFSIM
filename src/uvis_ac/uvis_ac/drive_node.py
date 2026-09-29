"""Send /motion/drive to the Assetto Corsa car script."""

import struct
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

from uvis_ac.drive import pedals, steer_input
from uvis_ac.pages import physics_from_bytes
from uvis_ac.paths import bridge_dir

PREFIX = bridge_dir()
HOLD_S = 0.4


class DriveNode(Node):
    def __init__(self):
        super().__init__('ac_drive')
        self.declare_parameter('drive_topic', '/motion/drive')
        self.declare_parameter('speed_gain', 0.5)
        self._path = PREFIX / 'drive.bin'
        self._target_speed = None
        self._steer = 0.0
        self._last = None
        PREFIX.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            self._on_drive, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().info('Writing %s from /motion/drive' % self._path)

    def _on_drive(self, msg: AckermannDriveStamped):
        self._target_speed = float(msg.drive.speed)
        self._steer = float(msg.drive.steering_angle)
        self._last = self.get_clock().now()

    def _speed(self):
        physics_path = PREFIX / 'physics.bin'
        if not physics_path.exists():
            return 0.0
        physics = physics_from_bytes(physics_path.read_bytes())
        if physics is None:
            return 0.0
        return float(physics.speed_kmh) / 3.6

    def _write(self, gas, brake, steer):
        payload = struct.pack('<3f', gas, brake, steer)
        temporary = self._path.with_suffix('.bin.tmp')
        temporary.write_bytes(payload)
        temporary.replace(self._path)

    def _tick(self):
        if self._target_speed is None or self._last is None:
            return
        age = (self.get_clock().now() - self._last).nanoseconds * 1e-9
        if age > HOLD_S:
            self._write(0.0, 1.0, 0.0)
            return
        gain = float(self.get_parameter('speed_gain').value)
        gas, brake = pedals(self._target_speed, self._speed(), gain)
        self._write(gas, brake, steer_input(self._steer))


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
