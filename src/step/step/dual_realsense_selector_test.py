#!/usr/bin/env python3
"""Test-only RGB selector for two continuously streaming RealSense cameras."""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


CAMERA_NAMES = ("front", "down")


def normalize_camera_name(value: str) -> str:
    """Return a canonical camera name or reject an unsafe selection."""
    camera_name = str(value).strip().lower()
    if camera_name not in CAMERA_NAMES:
        choices = ", ".join(CAMERA_NAMES)
        raise ValueError(
            f"camera selection must be one of: {choices}; got {value!r}"
        )
    return camera_name


class CameraSelectorState:
    """Hardware-independent front/down selection state."""

    def __init__(self, initial_camera: str) -> None:
        self._active_camera = normalize_camera_name(initial_camera)

    @property
    def active_camera(self) -> str:
        """Return the camera whose frames should currently be forwarded."""
        return self._active_camera

    def select(self, requested_camera: str) -> bool:
        """Select a camera and report whether the selection changed."""
        selected_camera = normalize_camera_name(requested_camera)
        changed = selected_camera != self._active_camera
        self._active_camera = selected_camera
        return changed

    def should_forward(self, source_camera: str) -> bool:
        """Return whether a frame from ``source_camera`` is active."""
        return normalize_camera_name(source_camera) == self._active_camera


class DualRealSenseSelectorTest(Node):
    """Forward only the selected RealSense RGB stream to one test topic."""

    def __init__(self) -> None:
        super().__init__("dual_realsense_selector_test")

        self.declare_parameter(
            "front_image_topic",
            "/vision_test/front_camera/color/image_raw",
        )
        self.declare_parameter(
            "down_image_topic",
            "/vision_test/down_camera/color/image_raw",
        )
        self.declare_parameter(
            "active_image_topic",
            "/vision_test/active/color/image_raw",
        )
        self.declare_parameter(
            "camera_select_topic",
            "/vision_test/camera_select",
        )
        self.declare_parameter(
            "active_camera_topic",
            "/vision_test/active_camera",
        )
        self.declare_parameter("initial_active_camera", "front")
        self.declare_parameter("state_publish_period_sec", 1.0)

        front_image_topic = self._string_parameter("front_image_topic")
        down_image_topic = self._string_parameter("down_image_topic")
        active_image_topic = self._string_parameter("active_image_topic")
        camera_select_topic = self._string_parameter("camera_select_topic")
        active_camera_topic = self._string_parameter("active_camera_topic")
        initial_camera = self._string_parameter("initial_active_camera")
        state_publish_period_sec = float(
            self.get_parameter("state_publish_period_sec").value
        )
        if state_publish_period_sec <= 0.0:
            raise ValueError("state_publish_period_sec must be greater than 0")

        self._state = CameraSelectorState(initial_camera)
        self._state_lock = threading.Lock()
        self._forwarded_frames = {camera: 0 for camera in CAMERA_NAMES}

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._active_image_publisher = self.create_publisher(
            Image,
            active_image_topic,
            image_qos,
        )
        self._active_camera_publisher = self.create_publisher(
            String,
            active_camera_topic,
            state_qos,
        )
        self._front_subscription = self.create_subscription(
            Image,
            front_image_topic,
            lambda message: self._image_callback(message, "front"),
            image_qos,
        )
        self._down_subscription = self.create_subscription(
            Image,
            down_image_topic,
            lambda message: self._image_callback(message, "down"),
            image_qos,
        )
        self._select_subscription = self.create_subscription(
            String,
            camera_select_topic,
            self._camera_select_callback,
            10,
        )
        self._state_timer = self.create_timer(
            state_publish_period_sec,
            self._publish_active_camera,
        )

        self._publish_active_camera()
        self.get_logger().info(f"Front RGB: {front_image_topic}")
        self.get_logger().info(f"Down RGB: {down_image_topic}")
        self.get_logger().info(f"Active RGB output: {active_image_topic}")
        self.get_logger().info(
            f"Camera selection input: {camera_select_topic}"
        )
        self.get_logger().info(
            f"Initial active camera: {self._state.active_camera}"
        )

    def _string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f"{name} must not be empty")
        return value

    def _image_callback(self, message: Image, source_camera: str) -> None:
        # Forward the ROS message directly. Avoiding cv_bridge here prevents an
        # unnecessary decode/copy/encode cycle in this test-only hot path.
        with self._state_lock:
            if not self._state.should_forward(source_camera):
                return
            self._active_image_publisher.publish(message)
            self._forwarded_frames[source_camera] += 1

    def _camera_select_callback(self, message: String) -> None:
        try:
            with self._state_lock:
                previous_camera = self._state.active_camera
                changed = self._state.select(message.data)
                active_camera = self._state.active_camera
                forwarded_frames = dict(self._forwarded_frames)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        self._publish_active_camera()
        if changed:
            self.get_logger().info(
                f"Active camera changed: {previous_camera} -> "
                f"{active_camera}; forwarded={forwarded_frames}"
            )
        else:
            self.get_logger().info(
                f"Active camera remains: {active_camera}"
            )

    def _publish_active_camera(self) -> None:
        with self._state_lock:
            active_camera = self._state.active_camera
        message = String()
        message.data = active_camera
        self._active_camera_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the test-only dual RealSense RGB selector."""
    rclpy.init(args=args)
    node: DualRealSenseSelectorTest | None = None
    try:
        node = DualRealSenseSelectorTest()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
