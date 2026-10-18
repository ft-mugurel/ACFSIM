"""Publish the Assetto Corsa view.

The in-game render was measured at 332 ms per frame. Grabbing the virtual
screen the game is on takes about 20 ms and shows the same picture. The
panel writes that output name in ``capture_output.txt``. One line is one
view. Only the first line is published for now.
"""

import json
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from uvis_ac.paths import bridge_dir

_HOST_LD = Path('/run/host/usr/lib/ld-linux-x86-64.so.2')
_HOST_LIB = '/run/host/usr/lib:/run/host/usr/lib64'


def _host(name):
    binary = Path('/run/host/usr/bin') / name
    if binary.is_file() and _HOST_LD.is_file():
        return [str(_HOST_LD), '--library-path', _HOST_LIB, str(binary)]
    return [name]


def _ppm_bgr(data):
    if not data.startswith(b'P6'):
        return None
    header, _, rest = data.partition(b'\n')
    lines = []
    blob = rest
    while len(lines) < 2:
        line, _, blob = blob.partition(b'\n')
        if line.startswith(b'#'):
            continue
        lines.append(line)
    width, height = (int(v) for v in lines[0].split())
    count = width * height * 3
    if len(blob) < count:
        return None
    rgb = np.frombuffer(blob, dtype=np.uint8, count=count).reshape((height, width, 3))
    return np.ascontiguousarray(rgb[:, :, ::-1])


def _game_rect():
    try:
        raw = subprocess.check_output(
            _host('hyprctl') + ['clients', '-j'], timeout=0.3)
        clients = json.loads(raw)
    except (subprocess.SubprocessError, json.JSONDecodeError, OSError):
        return None
    for client in clients:
        if client.get('title') == 'Assetto Corsa':
            x, y = client['at']
            w, h = client['size']
            if w > 32 and h > 32:
                return '%d,%d %dx%d' % (x, y, w, h)
    return None


def _pose_numbers():
    """fov and the AC camera vectors, read at the same moment as the grab."""
    path = bridge_dir() / 'camera.txt'
    try:
        parts = path.read_text(errors='ignore').split()
    except OSError:
        return None
    if len(parts) < 11:
        return None
    try:
        return [float(v) for v in parts[1:11]]
    except ValueError:
        return None


def _capture_output():
    """First view in capture_output.txt. Later lines would be more cameras."""
    path = bridge_dir() / 'capture_output.txt'
    try:
        lines = [line.strip() for line in path.read_text(errors='ignore').splitlines()]
    except OSError:
        return None
    lines = [line for line in lines if line and not line.startswith('#')]
    return lines[0] if lines else None


def _grab(args):
    try:
        done = subprocess.run(
            _host('grim') + args + ['-t', 'ppm', '-'],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=0.6,
        )
    except (subprocess.SubprocessError, OSError):
        return None
    if done.returncode != 0:
        return None
    return _ppm_bgr(done.stdout)


class CameraNode(Node):
    def __init__(self):
        super().__init__('ac_camera')
        self._missing = False
        self._rect = None
        self._rect_at = 0.0
        shown = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Image, '/camera/image', shown)
        self._pose_pub = self.create_publisher(Float64MultiArray, '/camera/pose', shown)
        self.create_timer(1.0 / 20.0, self._tick)
        self.get_logger().info('Grabbing the Assetto Corsa virtual screen')

    def _tick(self):
        now = time.monotonic()
        output = _capture_output()
        # Pose first, then the pixels, so both belong to this grab.
        pose = _pose_numbers()
        if output:
            frame = _grab(['-o', output])
        else:
            if self._rect is None or now - self._rect_at > 0.5:
                self._rect = _game_rect()
                self._rect_at = now
            frame = _grab(['-g', self._rect]) if self._rect else None
        if frame is None:
            if not self._missing:
                where = output or 'the Assetto Corsa window'
                self.get_logger().warning('No picture from %s' % where)
                self._missing = True
            return
        self._missing = False
        stamp = self.get_clock().now().to_msg()
        if pose is not None:
            pose_msg = Float64MultiArray()
            pose_msg.layout.dim = [MultiArrayDimension(label='camera', size=10, stride=10)]
            pose_msg.data = pose
            self._pose_pub.publish(pose_msg)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
