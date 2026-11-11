"""Own only the camera process and expose bounded, requested clock recovery."""

import json
from pathlib import Path
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .camera_time_health import CameraTimeHealth


class CameraProcess:
    """Stop/reap our child before opening the device again; never kill by name."""

    MAX_RESTARTS = 2
    STOP_TIMEOUT_SEC = 5.0
    KILL_TIMEOUT_SEC = 8.0

    def __init__(self, command, now, *, popen=subprocess.Popen):
        self.command = command
        self.popen = popen
        self.child = None
        self.attempts = 0
        self.generation = 0
        self.stopping_since = None
        self.kill_sent = False
        self.failed = None
        self.start(now)

    def start(self, now):
        try:
            self.child = self.popen(self.command, stdin=subprocess.DEVNULL)
            self.generation += 1
        except OSError as exc:
            self.failed = f'camera_start_failed: {exc}'
        self.stopping_since = None
        self.kill_sent = False

    def request_restart(self, now):
        if self.failed or self.stopping_since is not None:
            return False
        if self.attempts >= self.MAX_RESTARTS:
            self.failed = 'camera_restart_limit'
            return False
        self.attempts += 1
        self.stopping_since = now
        if self.child is not None and self.child.poll() is None:
            try:
                self.child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        return True

    def tick(self, now):
        if self.failed:
            return False
        if self.stopping_since is None:
            return False
        if self.child is None or self.child.poll() is not None:
            self.start(now)
            return self.failed is None
        elapsed = now - self.stopping_since
        if elapsed >= self.KILL_TIMEOUT_SEC:
            self.failed = 'camera_stop_timeout'
        elif elapsed >= self.STOP_TIMEOUT_SEC and not self.kill_sent:
            try:
                self.child.kill()
            except ProcessLookupError:
                pass
            self.kill_sent = True
        return False

    def close(self):
        if self.child is None or self.child.poll() is not None:
            return
        try:
            self.child.send_signal(signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            self.child.wait(timeout=self.STOP_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            self.child.kill()
            self.child.wait(timeout=3.0)


class CameraSupervisor(Node):
    def __init__(self):
        super().__init__('camera_supervisor')
        self.declare_parameter('camera_topic_prefix', '/camera/camera')
        prefix = str(self.get_parameter('camera_topic_prefix').value).rstrip('/')
        namespace, _, name = prefix.rpartition('/')
        if not prefix.startswith('/') or not name:
            raise ValueError('camera_topic_prefix must be an absolute node path')
        executable = Path(get_package_prefix('realsense2_camera')) / 'lib/realsense2_camera/realsense2_camera_node'
        command = [str(executable), '--ros-args', '-r', f'__ns:={namespace or "/"}',
                   '-r', f'__node:={name}']
        # rs_launch.py disables infrared streams; the driver's defaults do not.
        for key, value in {
            'enable_color': 'true', 'enable_depth': 'true',
            'enable_infra': 'false', 'enable_infra1': 'false', 'enable_infra2': 'false',
            'align_depth.enable': 'true', 'enable_gyro': 'true', 'enable_accel': 'true',
            'rgb_camera.color_profile': '1280,720,30',
            'depth_module.depth_profile': '848,480,30',
        }.items():
            command.extend(['-p', f'{key}:={value}'])
        self.session = uuid.uuid4().hex
        self.health = CameraTimeHealth(time.monotonic())
        self.publisher = self.create_publisher(String, '/navigation/camera_time_status', 1)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, prefix + '/color/image_raw',
                                 lambda msg: self.observe('rgb', msg), image_qos)
        self.create_subscription(Image, prefix + '/aligned_depth_to_color/image_raw',
                                 lambda msg: self.observe('depth', msg), image_qos)
        self.create_service(Trigger, '/navigation/restart_camera', self.restart)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.process = CameraProcess(command, time.monotonic())
        self.create_timer(.1, self.tick, clock=self.steady_clock)
        self.previous_state = None

    def observe(self, source, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.health.observe(source, stamp, self.get_clock().now().nanoseconds, time.monotonic())

    def restart(self, request, response):
        # The decision node requests only after execution and submitted queues drain.
        now = time.monotonic()
        state, needed = self.health.status(self.get_clock().now().nanoseconds, now)
        exited = self.process.child is not None and self.process.child.poll() is not None
        if state == 'READY' and not exited and not self.process.failed:
            # The driver can recover between the status and the service request.
            response.success = True
            response.message = 'camera_already_ready'
            return response
        eligible = state not in {'WAIT_SYSTEM_TIME', 'WAIT_CLOCK_STABLE'} and (needed or exited)
        response.success = eligible and self.process.request_restart(now)
        response.message = 'camera_restart_accepted' if response.success else 'camera_restart_not_available'
        self.tick()
        return response

    def tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        if self.process.tick(now):
            self.health.restarted(ros_now, now)
        state, needed = self.health.status(ros_now, now)
        if self.process.failed:
            state, needed = 'FAILED', False
        elif self.process.stopping_since is not None:
            state, needed = 'RESTARTING', False
        elif self.process.child is None or self.process.child.poll() is not None:
            state, needed = 'CAMERA_EXITED', state not in {'WAIT_SYSTEM_TIME', 'WAIT_CLOCK_STABLE'}
        elif needed and self.process.attempts >= self.process.MAX_RESTARTS:
            self.process.failed = 'camera_restart_limit'
            state, needed = 'FAILED', False
        payload = {'state': state, 'restart_needed': needed, 'session': self.session,
                   'generation': self.process.generation, 'attempts': self.process.attempts,
                   'failure': self.process.failed, 'minimum_stamp_ns': self.health.minimum_stamp_ns,
                   'frame_failure': self.health.frame_failure,
                   'capture_age_ms': {
                       source: round((ros_now - frame[0]) / 1e6, 3)
                       for source, frame in self.health.frames.items()},
                   'latest_pair_delta_ms': (
                       round(abs(self.health.frames['rgb'][0] - self.health.frames['depth'][0]) / 1e6, 3)
                       if all(source in self.health.frames for source in ('rgb', 'depth')) else None)}
        self.publisher.publish(String(data=json.dumps(payload)))
        if state != self.previous_state:
            self.get_logger().info('Camera time: ' + json.dumps(payload))
            self.previous_state = state

    def destroy_node(self):
        self.process.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraSupervisor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
