#!/usr/bin/env python3
"""ROS 2 node for YOLO26 object detection on a RealSense color topic."""

from __future__ import annotations

import ast
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time
from typing import Any

import cv2
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from step.tensorrt_backend import TensorRTBackend


def _default_model_path() -> str:
    """Return the installed model, with a source-tree development fallback."""
    try:
        package_share = Path(get_package_share_directory("step"))
        return str(package_share / "models" / "best.onnx")
    except PackageNotFoundError:
        source_root = Path(__file__).resolve().parents[1]
        return str(source_root / "models" / "best.onnx")


DEFAULT_MODEL_PATH = _default_model_path()
DEFAULT_CLASS_NAMES = ["line", "ball", "goal", "backboard", "hurdle"]
DISPLAY_WINDOW_NAME = "YOLO26 RealSense Detection"


@dataclass(frozen=True)
class CameraControlSpec:
    """Mapping from one OpenCV trackbar to a RealSense ROS parameter."""

    label: str
    parameter_name: str
    minimum: int
    maximum: int
    default: int
    value_type: str = "integer"

    @property
    def maximum_position(self) -> int:
        return (
            self.maximum - self.minimum
            if self.minimum < 0
            else self.maximum
        )

    def position_from_value(self, value: int) -> int:
        position = value - self.minimum if self.minimum < 0 else value
        return max(0, min(self.maximum_position, position))

    def value_from_position(self, position: int) -> bool | int | float:
        value = self.minimum + position if self.minimum < 0 else position
        value = max(self.minimum, min(self.maximum, value))
        if self.value_type == "boolean":
            return bool(value)
        if self.value_type == "double":
            return float(value)
        return int(value)


@dataclass(frozen=True)
class Detection:
    """One object detection in original-image pixel coordinates."""

    class_id: int
    class_name: str
    confidence: float
    bbox: list[int]
    center: list[int]


@dataclass(frozen=True)
class LetterboxInfo:
    """Geometry used to map model coordinates back to the source image."""

    scale: float
    pad_x: float
    pad_y: float


class Yolo26Detector(Node):
    """Run a YOLO26 ONNX model on incoming ROS color images."""

    def __init__(self) -> None:
        super().__init__("yolo26_detector")

        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter(
            "image_topic", "/camera/color/image_raw"
        )
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "annotated_image_topic", "/vision/detections/image"
        )
        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("line_info_timeout_sec", 0.8)
        self.declare_parameter("show_line_metrics", True)
        self.declare_parameter("ball_info_topic", "/vision/ball_info")
        self.declare_parameter("ball_info_timeout_sec", 0.8)
        self.declare_parameter("show_ball_metrics", True)
        self.declare_parameter("ball_control_range_m", 0.9)
        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("goal_info_timeout_sec", 0.8)
        self.declare_parameter("show_goal_metrics", True)
        self.declare_parameter("goal_control_range_m", 0.5)
        self.declare_parameter("hurdle_info_topic", "/vision/hurdle_info")
        self.declare_parameter("hurdle_info_timeout_sec", 0.8)
        self.declare_parameter("show_hurdle_metrics", True)
        self.declare_parameter(
            "motion_command_topic",
            "/navigation/motion_command",
        )
        self.declare_parameter("motion_command_timeout_sec", 0.8)
        self.declare_parameter("metrics_mode", "auto")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("max_detections", 300)
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("device", "auto")
        self.declare_parameter("display", True)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("show_camera_controls", True)
        self.declare_parameter("camera_node_name", "/camera/camera")
        self.declare_parameter("camera_control_apply_interval_sec", 0.15)

        self.model_path = Path(
            self.get_parameter("model_path").value
        ).expanduser()
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.max_detections = int(
            self.get_parameter("max_detections").value
        )
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.display = bool(self.get_parameter("display").value)
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.show_camera_controls = bool(
            self.get_parameter("show_camera_controls").value
        )
        self.camera_node_name = str(
            self.get_parameter("camera_node_name").value
        )
        self.camera_control_apply_interval_sec = max(
            0.05,
            float(
                self.get_parameter(
                    "camera_control_apply_interval_sec"
                ).value
            ),
        )
        self.line_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("line_info_timeout_sec").value),
        )
        self.show_line_metrics = bool(
            self.get_parameter("show_line_metrics").value
        )
        self.ball_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("ball_info_timeout_sec").value),
        )
        self.show_ball_metrics = bool(
            self.get_parameter("show_ball_metrics").value
        )
        self.ball_control_range_m = float(
            self.get_parameter("ball_control_range_m").value
        )
        self.goal_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("goal_info_timeout_sec").value),
        )
        self.show_goal_metrics = bool(
            self.get_parameter("show_goal_metrics").value
        )
        self.goal_control_range_m = float(
            self.get_parameter("goal_control_range_m").value
        )
        self.hurdle_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("hurdle_info_timeout_sec").value),
        )
        self.show_hurdle_metrics = bool(
            self.get_parameter("show_hurdle_metrics").value
        )
        self.motion_command_timeout_sec = max(
            0.1,
            float(self.get_parameter("motion_command_timeout_sec").value),
        )
        self.metrics_mode = str(
            self.get_parameter("metrics_mode").value
        ).strip().lower()
        if self.metrics_mode not in {
            "auto",
            "line",
            "ball",
            "goal",
            "hurdle",
        }:
            raise ValueError(
                "metrics_mode must be auto, line, ball, goal, or hurdle"
            )

        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"YOLO26 model was not found: {self.model_path}"
            )
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")

        self.bridge = CvBridge()
        self.session: ort.InferenceSession | None = None
        self.tensorrt_backend: TensorRTBackend | None = None
        requested_device = str(self.get_parameter("device").value)
        model_suffix = self.model_path.suffix.lower()
        if model_suffix == ".onnx":
            self.backend_name = "ONNX Runtime"
            self.session, self.active_provider = self._create_session(
                requested_device
            )
            self.input_name = self.session.get_inputs()[0].name
            input_shape = self.session.get_inputs()[0].shape
            self.class_names = self._read_class_names()
        elif model_suffix == ".engine":
            device = requested_device.strip().lower()
            if device not in {"auto", "tensorrt", "cuda", "cpu"}:
                raise ValueError("device must be auto, tensorrt, cuda, or cpu")
            if device == "cpu":
                self.get_logger().warning(
                    "device=cpu is ignored for a TensorRT engine; using GPU"
                )
            self.backend_name = "TensorRT engine"
            self.tensorrt_backend = TensorRTBackend(self.model_path)
            self.active_provider = "TensorRT"
            self.input_name = self.tensorrt_backend.input_name
            input_shape = self.tensorrt_backend.input_shape
            self.class_names = DEFAULT_CLASS_NAMES.copy()
        else:
            raise ValueError(
                "model_path must end in .onnx or .engine; "
                f"got {self.model_path}"
            )
        self.input_height = self._fixed_dimension(input_shape[2], 640)
        self.input_width = self._fixed_dimension(input_shape[3], 640)

        detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        annotated_topic = str(
            self.get_parameter("annotated_image_topic").value
        )
        image_topic = str(self.get_parameter("image_topic").value)
        line_info_topic = str(
            self.get_parameter("line_info_topic").value
        )
        ball_info_topic = str(
            self.get_parameter("ball_info_topic").value
        )
        goal_info_topic = str(
            self.get_parameter("goal_info_topic").value
        )
        hurdle_info_topic = str(
            self.get_parameter("hurdle_info_topic").value
        )
        motion_command_topic = str(
            self.get_parameter("motion_command_topic").value
        )

        self.detections_publisher = self.create_publisher(
            String, detections_topic, 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, annotated_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            line_info_topic,
            self._line_info_callback,
            10,
        )
        self.create_subscription(
            String,
            ball_info_topic,
            self._ball_info_callback,
            10,
        )
        self.create_subscription(
            String,
            goal_info_topic,
            self._goal_info_callback,
            10,
        )
        self.create_subscription(
            String,
            hurdle_info_topic,
            self._hurdle_info_callback,
            10,
        )
        self.create_subscription(
            String,
            motion_command_topic,
            self._motion_command_callback,
            10,
        )

        self.last_inference_time = 0.0
        self.smoothed_fps = 0.0
        self.processing = False
        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_info_time: float | None = None
        self.latest_ball_info: dict[str, Any] | None = None
        self.latest_ball_info_time: float | None = None
        self.latest_goal_info: dict[str, Any] | None = None
        self.latest_goal_info_time: float | None = None
        self.latest_hurdle_info: dict[str, Any] | None = None
        self.latest_hurdle_info_time: float | None = None
        self.latest_motion_command: dict[str, Any] | None = None
        self.latest_motion_command_time: float | None = None

        self.camera_control_specs = self._camera_control_specs()
        self.camera_control_values = {
            spec.parameter_name: spec.value_from_position(
                spec.position_from_value(spec.default)
            )
            for spec in self.camera_control_specs
        }
        self.camera_control_regions: dict[
            str, tuple[CameraControlSpec, str, int, int, int, int]
        ] = {}
        self.camera_control_dragging: str | None = None
        self.pending_camera_controls: dict[str, bool | int | float] = {}
        self.camera_control_future = None
        camera_service = (
            self.camera_node_name.rstrip("/") + "/set_parameters"
        )
        self.camera_control_client = self.create_client(
            SetParameters,
            camera_service,
        )
        self.camera_control_timer = self.create_timer(
            self.camera_control_apply_interval_sec,
            self._apply_pending_camera_controls,
        )
        if self.display and self.show_camera_controls:
            self._create_camera_control_panel()

        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Backend: {self.backend_name}")
        self.get_logger().info(f"Provider: {self.active_provider}")
        self.get_logger().info(
            f"Input: {self.input_width}x{self.input_height}"
        )
        self.get_logger().info(f"Classes: {self.class_names}")
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Line metrics: {line_info_topic}")
        self.get_logger().info(f"Ball metrics: {ball_info_topic}")
        self.get_logger().info(f"Goal metrics: {goal_info_topic}")
        self.get_logger().info(f"Hurdle metrics: {hurdle_info_topic}")
        self.get_logger().info(f"Motion decision: {motion_command_topic}")
        if self.display and self.show_camera_controls:
            self.get_logger().info(
                f"Camera sliders: {self.camera_node_name}"
            )
        self.get_logger().info(f"Publishing: {detections_topic}")

    @staticmethod
    def _camera_control_specs() -> tuple[CameraControlSpec, ...]:
        """Return the live color controls exposed by the D435i driver."""
        return (
            CameraControlSpec(
                "Auto Exposure",
                "rgb_camera.enable_auto_exposure",
                0,
                1,
                1,
                "boolean",
            ),
            CameraControlSpec(
                "Exposure",
                "rgb_camera.exposure",
                1,
                10000,
                156,
            ),
            CameraControlSpec(
                "Gain",
                "rgb_camera.gain",
                0,
                128,
                64,
            ),
            CameraControlSpec(
                "Brightness",
                "rgb_camera.brightness",
                -64,
                64,
                0,
            ),
            CameraControlSpec(
                "Contrast",
                "rgb_camera.contrast",
                0,
                100,
                50,
            ),
            CameraControlSpec(
                "Saturation",
                "rgb_camera.saturation",
                0,
                100,
                64,
            ),
            CameraControlSpec(
                "Sharpness",
                "rgb_camera.sharpness",
                0,
                100,
                50,
            ),
            CameraControlSpec(
                "Auto White Balance",
                "rgb_camera.enable_auto_white_balance",
                0,
                1,
                1,
                "boolean",
            ),
            CameraControlSpec(
                "White Balance",
                "rgb_camera.white_balance",
                2800,
                6500,
                4000,
                "double",
            ),
            CameraControlSpec(
                "Power Line",
                "rgb_camera.power_line_frequency",
                0,
                3,
                3,
            ),
        )

    def _create_camera_control_panel(self) -> None:
        """Create the YOLO window and attach the compact panel mouse handler."""
        cv2.namedWindow(DISPLAY_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(
            DISPLAY_WINDOW_NAME,
            self._camera_control_mouse_callback,
        )

    def _set_camera_control_value(
        self,
        spec: CameraControlSpec,
        value: bool | int | float,
        *,
        manual: bool = True,
    ) -> None:
        """Update the panel state and queue one RealSense parameter."""
        if spec.value_type == "boolean":
            converted: bool | int | float = bool(value)
        else:
            numeric = max(spec.minimum, min(spec.maximum, float(value)))
            converted = float(numeric) if spec.value_type == "double" else int(
                round(numeric)
            )
        self.camera_control_values[spec.parameter_name] = converted
        self.pending_camera_controls[spec.parameter_name] = converted

        if not manual:
            return
        if spec.parameter_name in {
            "rgb_camera.exposure",
            "rgb_camera.gain",
        }:
            self._disable_auto_control("rgb_camera.enable_auto_exposure")
        elif spec.parameter_name == "rgb_camera.white_balance":
            self._disable_auto_control(
                "rgb_camera.enable_auto_white_balance"
            )

    def _disable_auto_control(self, parameter_name: str) -> None:
        """Switch the matching auto mode off after a manual adjustment."""
        self.camera_control_values[parameter_name] = False
        self.pending_camera_controls[parameter_name] = False

    def _camera_control_mouse_callback(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        _userdata: Any,
    ) -> None:
        """Handle compact sliders, toggle buttons, and power-line choices."""
        if event == cv2.EVENT_LBUTTONDOWN:
            for name, region in self.camera_control_regions.items():
                spec, kind, x1, y1, x2, y2 = region
                if x1 <= x <= x2 and y1 <= y <= y2:
                    if kind == "toggle":
                        current = bool(self.camera_control_values[name])
                        self._set_camera_control_value(spec, not current)
                    elif kind == "segments":
                        width = max(1, x2 - x1 + 1)
                        choice = min(3, max(0, int(4 * (x - x1) / width)))
                        self._set_camera_control_value(spec, choice)
                    else:
                        self.camera_control_dragging = name
                        self._update_camera_slider(spec, x, x1, x2)
                    return
        elif event == cv2.EVENT_MOUSEMOVE:
            if not (flags & cv2.EVENT_FLAG_LBUTTON):
                return
            name = self.camera_control_dragging
            if name is None or name not in self.camera_control_regions:
                return
            spec, kind, x1, _y1, x2, _y2 = self.camera_control_regions[name]
            if kind == "slider":
                self._update_camera_slider(spec, x, x1, x2)
        elif event == cv2.EVENT_LBUTTONUP:
            self.camera_control_dragging = None

    def _update_camera_slider(
        self,
        spec: CameraControlSpec,
        x: int,
        x1: int,
        x2: int,
    ) -> None:
        """Convert a mouse position on a compact slider to its real value."""
        ratio = (max(x1, min(x2, x)) - x1) / max(1, x2 - x1)
        value = spec.minimum + ratio * (spec.maximum - spec.minimum)
        self._set_camera_control_value(spec, value)

    def _apply_pending_camera_controls(self) -> None:
        """Send coalesced slider updates to the RealSense node."""
        if self.camera_control_future is not None:
            if not self.camera_control_future.done():
                return
            try:
                response = self.camera_control_future.result()
                failures = [
                    result.reason
                    for result in response.results
                    if not result.successful
                ]
                if failures:
                    self.get_logger().warning(
                        "Camera control rejected: " + "; ".join(failures)
                    )
            except Exception as exc:
                self.get_logger().warning(
                    f"Camera control request failed: {exc}"
                )
            self.camera_control_future = None

        if not self.pending_camera_controls:
            return
        if not self.camera_control_client.service_is_ready():
            return

        updates = self.pending_camera_controls
        self.pending_camera_controls = {}
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in updates.items()
        ]
        self.camera_control_future = self.camera_control_client.call_async(
            request
        )

    def _reset_camera_controls(self) -> None:
        """Restore the documented D435i color defaults after pressing R."""
        defaults: dict[str, bool | int | float] = {}
        for spec in self.camera_control_specs:
            position = spec.position_from_value(spec.default)
            value = spec.value_from_position(position)
            defaults[spec.parameter_name] = value
            self.camera_control_values[spec.parameter_name] = value
        auto_names = {
            "rgb_camera.enable_auto_exposure",
            "rgb_camera.enable_auto_white_balance",
        }
        for name, value in defaults.items():
            if name not in auto_names:
                self.pending_camera_controls[name] = value
        for name in auto_names:
            self.pending_camera_controls[name] = defaults[name]
        self.get_logger().info("Camera color controls reset to defaults")

    def _draw_camera_control_panel(self, image: np.ndarray) -> np.ndarray:
        """Append a compact two-column camera control panel to the image."""
        panel_height = 188
        height, width = image.shape[:2]
        canvas = np.full(
            (height + panel_height, width, 3),
            (28, 31, 36),
            dtype=np.uint8,
        )
        canvas[:height, :width] = image
        cv2.line(canvas, (0, height), (width, height), (70, 75, 82), 1)
        cv2.putText(
            canvas,
            "CAMERA CONTROLS",
            (16, height + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "drag sliders  |  click buttons  |  R reset",
            (205, height + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (155, 160, 168),
            1,
            cv2.LINE_AA,
        )

        specs = {spec.parameter_name: spec for spec in self.camera_control_specs}
        columns = (
            (
                "rgb_camera.enable_auto_exposure",
                "rgb_camera.exposure",
                "rgb_camera.gain",
                "rgb_camera.brightness",
                "rgb_camera.contrast",
            ),
            (
                "rgb_camera.enable_auto_white_balance",
                "rgb_camera.white_balance",
                "rgb_camera.saturation",
                "rgb_camera.sharpness",
                "rgb_camera.power_line_frequency",
            ),
        )
        self.camera_control_regions = {}
        column_width = width // 2
        for column_index, names in enumerate(columns):
            column_x = column_index * column_width
            label_x = column_x + 16
            control_x1 = column_x + min(190, max(145, column_width // 3))
            control_x2 = column_x + column_width - 24
            for row_index, name in enumerate(names):
                spec = specs[name]
                center_y = height + 48 + row_index * 28
                value = self.camera_control_values[name]
                cv2.putText(
                    canvas,
                    spec.label,
                    (label_x, center_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (215, 218, 222),
                    1,
                    cv2.LINE_AA,
                )
                if spec.value_type == "boolean":
                    self._draw_camera_toggle(
                        canvas,
                        spec,
                        bool(value),
                        control_x1,
                        center_y,
                    )
                elif name == "rgb_camera.power_line_frequency":
                    self._draw_power_line_choices(
                        canvas,
                        spec,
                        int(value),
                        control_x1,
                        control_x2,
                        center_y,
                    )
                else:
                    self._draw_camera_slider(
                        canvas,
                        spec,
                        float(value),
                        control_x1,
                        control_x2,
                        center_y,
                    )
        return canvas

    def _draw_camera_toggle(
        self,
        image: np.ndarray,
        spec: CameraControlSpec,
        enabled: bool,
        x: int,
        center_y: int,
    ) -> None:
        """Draw one clickable ON/OFF button."""
        x2 = x + 82
        y1, y2 = center_y - 10, center_y + 11
        color = (50, 165, 70) if enabled else (65, 70, 78)
        cv2.rectangle(image, (x, y1), (x2, y2), color, -1)
        cv2.rectangle(image, (x, y1), (x2, y2), (115, 120, 128), 1)
        text = "ON" if enabled else "OFF"
        cv2.putText(
            image,
            text,
            (x + 27, center_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        self.camera_control_regions[spec.parameter_name] = (
            spec,
            "toggle",
            x,
            y1,
            x2,
            y2,
        )

    def _draw_camera_slider(
        self,
        image: np.ndarray,
        spec: CameraControlSpec,
        value: float,
        x1: int,
        x2: int,
        center_y: int,
    ) -> None:
        """Draw one normalized compact slider and its real numeric value."""
        value_width = 62
        rail_x2 = max(x1 + 40, x2 - value_width)
        ratio = (value - spec.minimum) / max(1, spec.maximum - spec.minimum)
        thumb_x = int(round(x1 + ratio * (rail_x2 - x1)))
        cv2.line(image, (x1, center_y), (rail_x2, center_y), (92, 98, 108), 4)
        cv2.line(image, (x1, center_y), (thumb_x, center_y), (0, 190, 255), 4)
        cv2.circle(image, (thumb_x, center_y), 7, (245, 245, 245), -1)
        cv2.circle(image, (thumb_x, center_y), 7, (70, 75, 82), 1)
        shown_value = int(round(value))
        cv2.putText(
            image,
            str(shown_value),
            (rail_x2 + 13, center_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        self.camera_control_regions[spec.parameter_name] = (
            spec,
            "slider",
            x1,
            center_y - 11,
            rail_x2,
            center_y + 11,
        )

    def _draw_power_line_choices(
        self,
        image: np.ndarray,
        spec: CameraControlSpec,
        selected: int,
        x1: int,
        x2: int,
        center_y: int,
    ) -> None:
        """Draw OFF/50/60/AUTO as four compact selectable buttons."""
        labels = ("OFF", "50", "60", "AUTO")
        total_width = max(120, x2 - x1)
        segment_width = total_width / 4.0
        y1, y2 = center_y - 10, center_y + 11
        for index, label in enumerate(labels):
            left = int(round(x1 + index * segment_width))
            right = int(round(x1 + (index + 1) * segment_width))
            color = (0, 150, 215) if index == selected else (55, 60, 68)
            cv2.rectangle(image, (left, y1), (right, y2), color, -1)
            cv2.rectangle(image, (left, y1), (right, y2), (105, 110, 118), 1)
            text_x = left + max(4, (right - left - len(label) * 8) // 2)
            cv2.putText(
                image,
                label,
                (text_x, center_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        self.camera_control_regions[spec.parameter_name] = (
            spec,
            "segments",
            x1,
            y1,
            x1 + total_width,
            y2,
        )

    def _line_info_callback(self, message: String) -> None:
        """Store the latest analyzed line geometry for the display."""
        payload = self._read_json_object(message, "line info")
        if payload is None:
            return
        self.latest_line_info = payload
        self.latest_line_info_time = time.monotonic()

    def _fresh_line_info(self) -> dict[str, Any] | None:
        """Return fresh line information, or None after its timeout."""
        if self.latest_line_info is None or self.latest_line_info_time is None:
            return None
        age = time.monotonic() - self.latest_line_info_time
        if age > self.line_info_timeout_sec:
            return None
        return self.latest_line_info

    def _motion_command_callback(self, message: String) -> None:
        """Store the command actually selected by motion_decision_node."""
        payload = self._read_json_object(message, "motion command")
        if payload is None:
            return
        self.latest_motion_command = payload
        self.latest_motion_command_time = time.monotonic()

    def _fresh_motion_command(self) -> dict[str, Any] | None:
        if (
            self.latest_motion_command is None
            or self.latest_motion_command_time is None
        ):
            return None
        age = time.monotonic() - self.latest_motion_command_time
        if age > self.motion_command_timeout_sec:
            return None
        return self.latest_motion_command

    def _read_json_object(
        self,
        message: String,
        label: str,
    ) -> dict[str, Any] | None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError(f"{label} must be a JSON object")
            return payload
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid {label}: {exc}")
            return None

    def _ball_info_callback(self, message: String) -> None:
        """Store the latest analyzed ball geometry for the display."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("ball info must be a JSON object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid ball info: {exc}")
            return
        self.latest_ball_info = payload
        self.latest_ball_info_time = time.monotonic()

    def _fresh_ball_info(self) -> dict[str, Any] | None:
        """Return fresh ball information, or None after its timeout."""
        if (
            self.latest_ball_info is None
            or self.latest_ball_info_time is None
        ):
            return None
        age = time.monotonic() - self.latest_ball_info_time
        if age > self.ball_info_timeout_sec:
            return None
        return self.latest_ball_info

    def _goal_info_callback(self, message: String) -> None:
        """Store the latest analyzed goal geometry for the display."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("goal info must be a JSON object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid goal info: {exc}")
            return
        self.latest_goal_info = payload
        self.latest_goal_info_time = time.monotonic()

    def _fresh_goal_info(self) -> dict[str, Any] | None:
        """Return fresh goal information, or None after its timeout."""
        if (
            self.latest_goal_info is None
            or self.latest_goal_info_time is None
        ):
            return None
        age = time.monotonic() - self.latest_goal_info_time
        if age > self.goal_info_timeout_sec:
            return None
        return self.latest_goal_info

    def _hurdle_info_callback(self, message: String) -> None:
        """Store the latest analyzed hurdle geometry for the display."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("hurdle info must be a JSON object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid hurdle info: {exc}")
            return
        self.latest_hurdle_info = payload
        self.latest_hurdle_info_time = time.monotonic()

    def _fresh_hurdle_info(self) -> dict[str, Any] | None:
        """Return fresh hurdle information, or None after its timeout."""
        if (
            self.latest_hurdle_info is None
            or self.latest_hurdle_info_time is None
        ):
            return None
        age = time.monotonic() - self.latest_hurdle_info_time
        if age > self.hurdle_info_timeout_sec:
            return None
        return self.latest_hurdle_info

    @staticmethod
    def _fixed_dimension(value: Any, fallback: int) -> int:
        return int(value) if isinstance(value, int) and value > 0 else fallback

    def _create_session(
        self, requested_device: str
    ) -> tuple[ort.InferenceSession, str]:
        available = ort.get_available_providers()
        device = requested_device.strip().lower()
        provider_preferences = {
            "auto": [
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            "tensorrt": [
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            "cuda": ["CUDAExecutionProvider", "CPUExecutionProvider"],
            "cpu": ["CPUExecutionProvider"],
        }
        if device not in provider_preferences:
            raise ValueError("device must be auto, tensorrt, cuda, or cpu")

        providers = [
            provider
            for provider in provider_preferences[device]
            if provider in available
        ]
        if not providers:
            raise RuntimeError(
                f"No usable ONNX Runtime provider. Available: {available}"
            )

        try:
            session = ort.InferenceSession(
                str(self.model_path), providers=providers
            )
        except Exception as exc:
            if "CPUExecutionProvider" not in available or device == "cpu":
                raise RuntimeError(
                    f"Could not load YOLO26 model: {exc}"
                ) from exc
            self.get_logger().warning(
                "GPU provider initialization failed; falling back to CPU: "
                f"{exc}"
            )
            session = ort.InferenceSession(
                str(self.model_path), providers=["CPUExecutionProvider"]
            )

        active_provider = session.get_providers()[0]
        requested_gpu = device in {"cuda", "tensorrt"}
        if requested_gpu and active_provider == "CPUExecutionProvider":
            self.get_logger().warning(
                f"Requested {device}, but ONNX Runtime is using CPU"
            )
        return session, active_provider

    def _read_class_names(self) -> list[str]:
        if self.session is None:
            return DEFAULT_CLASS_NAMES.copy()
        metadata = self.session.get_modelmeta().custom_metadata_map
        raw_names = metadata.get("names")
        if raw_names:
            try:
                names = ast.literal_eval(raw_names)
                if isinstance(names, dict):
                    return [str(names[index]) for index in sorted(names)]
                if isinstance(names, (list, tuple)):
                    return [str(name) for name in names]
            except (SyntaxError, ValueError, KeyError, TypeError):
                self.get_logger().warning(
                    "Could not parse class names from ONNX metadata"
                )
        return DEFAULT_CLASS_NAMES.copy()

    def _letterbox(
        self, image: np.ndarray
    ) -> tuple[np.ndarray, LetterboxInfo]:
        height, width = image.shape[:2]
        scale = min(self.input_width / width, self.input_height / height)
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        resized = cv2.resize(
            image,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )

        pad_width = self.input_width - resized_width
        pad_height = self.input_height - resized_height
        left = pad_width // 2
        right = pad_width - left
        top = pad_height // 2
        bottom = pad_height - top
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        return padded, LetterboxInfo(scale, float(left), float(top))

    def _preprocess(
        self, image: np.ndarray
    ) -> tuple[np.ndarray, LetterboxInfo]:
        padded, info = self._letterbox(image)
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return np.ascontiguousarray(blob), info

    def _postprocess(
        self,
        output: np.ndarray,
        info: LetterboxInfo,
        image_shape: tuple[int, ...],
    ) -> list[Detection]:
        predictions = (
            np.squeeze(output, axis=0) if output.ndim == 3 else output
        )
        if predictions.ndim != 2 or predictions.shape[1] < 6:
            raise RuntimeError(
                f"Unexpected YOLO26 output shape: {output.shape}"
            )

        height, width = image_shape[:2]
        detections: list[Detection] = []
        for prediction in predictions[: self.max_detections]:
            confidence = float(prediction[4])
            if confidence < self.confidence_threshold:
                continue

            class_id = int(round(float(prediction[5])))
            if not 0 <= class_id < len(self.class_names):
                self.get_logger().warning(
                    f"Ignoring invalid class id: {class_id}"
                )
                continue

            x1, y1, x2, y2 = (float(value) for value in prediction[:4])
            x1 = (x1 - info.pad_x) / info.scale
            x2 = (x2 - info.pad_x) / info.scale
            y1 = (y1 - info.pad_y) / info.scale
            y2 = (y2 - info.pad_y) / info.scale

            left = int(np.clip(round(x1), 0, width - 1))
            top = int(np.clip(round(y1), 0, height - 1))
            right = int(np.clip(round(x2), 0, width - 1))
            bottom = int(np.clip(round(y2), 0, height - 1))
            if right <= left or bottom <= top:
                continue

            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=self.class_names[class_id],
                    confidence=confidence,
                    bbox=[left, top, right, bottom],
                    center=[(left + right) // 2, (top + bottom) // 2],
                )
            )
        return detections

    def _run_inference(self, blob: np.ndarray) -> np.ndarray:
        """Run the selected backend while keeping one output contract."""
        if self.tensorrt_backend is not None:
            return self.tensorrt_backend.infer(blob)
        if self.session is None:
            raise RuntimeError("No inference backend is initialized")
        outputs = self.session.run(None, {self.input_name: blob})
        if not outputs:
            raise RuntimeError("ONNX Runtime returned no outputs")
        return outputs[0]

    @staticmethod
    def _color_for_class(class_id: int) -> tuple[int, int, int]:
        colors = [
            (255, 255, 0),
            (0, 140, 255),
            (0, 255, 0),
            (255, 0, 255),
            (0, 255, 255),
        ]
        return colors[class_id % len(colors)]

    @staticmethod
    def _number(data: dict[str, Any], key: str) -> float | None:
        """Read one finite numeric display value from ball information."""
        value = data.get(key)
        if value is None or isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if np.isfinite(number) else None

    @staticmethod
    def _confirmation_badge(info: dict[str, Any] | None) -> str:
        """Return a short RAW/CANDIDATE/CONFIRMED display badge."""
        if info is None:
            return "RAW"
        if bool(info.get("confirmation_confirmed", False)):
            return "CONFIRMED"
        if not bool(info.get("raw_detected", False)):
            return "RAW"
        hits = int(info.get("confirmation_hits", 0))
        required = int(info.get("confirmation_required_hits", 1))
        return f"CANDIDATE {hits}/{required}"

    @staticmethod
    def _metric_text(
        value: float | None,
        unit: str,
        digits: int = 2,
        signed: bool = False,
    ) -> str:
        """Format one optional numeric metric for the image panel."""
        if value is None:
            return "N/A"
        sign = "+" if signed else ""
        return f"{value:{sign}.{digits}f}{unit}"

    @staticmethod
    def _action_banner(
        source: str,
        action: str,
    ) -> tuple[str, tuple[int, int, int]] | None:
        """Map one selected planner command to a large screen banner."""
        normalized_source = source.strip().lower()
        normalized_action = action.strip().upper()
        line_labels = {
            "STRAIGHT": "STRAIGHT",
            "FINE_LEFT": "FINE LEFT",
            "FINE_RIGHT": "FINE RIGHT",
            "LEFT": "LEFT",
            "RIGHT": "RIGHT",
            "STOP": "STOP",
        }
        ball_labels = {
            "PICKUP_NOW": "PICK UP BALL",
            "TURN_LEFT": "BALL TURN LEFT",
            "TURN_RIGHT": "BALL TURN RIGHT",
            "APPROACH": "BALL APPROACH",
            "SLOW_APPROACH": "BALL SLOW APPROACH",
            "FINE_FORWARD_STEP": "BALL FINE STEP",
            "BALL_LOST_STOP": "BALL STOP",
            "RECOVER_TURN_LEFT": "FIND BALL LEFT",
            "RECOVER_TURN_RIGHT": "FIND BALL RIGHT",
            "STOP": "BALL STOP",
        }
        if normalized_source == "line":
            label = line_labels.get(normalized_action)
            return (label, (130, 105, 0)) if label is not None else None
        if normalized_source == "ball":
            label = ball_labels.get(normalized_action)
            if label is None:
                return None
            color = (
                (0, 120, 0)
                if normalized_action == "PICKUP_NOW"
                else (0, 105, 190)
            )
            return label, color
        return None

    @staticmethod
    def _draw_action_banner(
        image: np.ndarray,
        text: str,
        background_color: tuple[int, int, int],
    ) -> None:
        """Draw one centered command banner while fitting long labels."""
        height, width = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 4
        font_scale = 1.35
        max_text_width = max(120, width - 48)
        while font_scale > 0.65:
            (text_width, text_height), _ = cv2.getTextSize(
                text,
                font,
                font_scale,
                thickness,
            )
            if text_width <= max_text_width:
                break
            font_scale -= 0.05
        text_x = max(12, (width - text_width) // 2)
        text_y = min(76, max(text_height + 18, height - 12))
        cv2.rectangle(
            image,
            (text_x - 18, text_y - text_height - 14),
            (text_x + text_width + 18, text_y + 14),
            background_color,
            -1,
        )
        cv2.putText(
            image,
            text,
            (text_x, text_y),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _draw_ball_metrics(self, image: np.ndarray) -> None:
        """Draw analyzed ball distance and alignment data in-place."""
        if not self.show_ball_metrics:
            return

        height, width = image.shape[:2]
        center_x = width // 2
        center_color = (0, 255, 255)
        cv2.line(
            image,
            (center_x, max(45, int(height * 0.42))),
            (center_x, height - 1),
            center_color,
            1,
            cv2.LINE_AA,
        )

        info = self._fresh_ball_info()
        decision = self._fresh_motion_command()
        if decision is None:
            planner_action = "NO COMMAND"
        else:
            planner_source = str(decision.get("source", "none")).upper()
            selected_action = str(decision.get("action", "WAIT")).upper()
            planner_action = f"{planner_source} / {selected_action}"
        detected = bool(info and info.get("detected", False))
        if detected and info is not None:
            target_x = self._number(info, "center_x")
            target_y = self._number(info, "center_y")
            if target_x is not None and target_y is not None:
                point_x = int(np.clip(round(target_x), 0, width - 1))
                point_y = int(np.clip(round(target_y), 0, height - 1))
                cv2.arrowedLine(
                    image,
                    (center_x, point_y),
                    (point_x, point_y),
                    (0, 140, 255),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.12,
                )
                offset_x = self._number(info, "offset_x_px")
                offset_text = self._metric_text(
                    offset_x,
                    "px",
                    digits=0,
                    signed=True,
                )
                text_x = min(center_x, point_x) + 8
                cv2.putText(
                    image,
                    f"dx {offset_text}",
                    (text_x, max(22, point_y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 140, 255),
                    2,
                    cv2.LINE_AA,
                )

        panel_width = min(390, max(250, width - 24))
        panel_height = 264
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        image[panel_y:panel_bottom, panel_x:panel_x + panel_width] = (
            cv2.addWeighted(
                overlay[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.72,
                image[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.28,
                0.0,
            )
        )

        if info is None:
            rows = [
                "BALL METRICS",
                f"Planner     : {planner_action}",
                "NO BALL INFO",
            ]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = [
                "BALL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state}",
            ]
        else:
            distance = self._number(info, "distance_m")
            depth = self._number(info, "depth_m")
            offset_px = self._number(info, "offset_x_px")
            offset_norm = self._number(info, "offset_x_norm")
            bearing = self._number(info, "bearing_deg")
            lateral = self._number(info, "lateral_offset_m")
            direction = str(info.get("horizontal_direction", "UNKNOWN"))
            state = str(info.get("state", "UNKNOWN"))
            camera_ready = bool(info.get("camera_info_ready", False))
            rows = [
                "BALL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state} / {direction}",
                "Distance    : "
                + self._metric_text(distance, "m"),
                "Depth Z     : " + self._metric_text(depth, "m"),
                "Offset X    : "
                + self._metric_text(offset_px, "px", 0, signed=True),
                "Offset norm : "
                + self._metric_text(offset_norm, "", 3, signed=True),
                "Bearing     : "
                + self._metric_text(bearing, "deg", 1, signed=True),
                "Lateral X   : "
                + self._metric_text(lateral, "m", 3, signed=True),
                f"Camera info : {'OK' if camera_ready else 'MISSING'}",
            ]

        for index, row in enumerate(rows):
            is_title = index == 0
            color = (0, 200, 255) if is_title else (245, 245, 245)
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if is_title else 0.5,
                color,
                2 if is_title else 1,
                cv2.LINE_AA,
            )

        decision_source = (
            str(decision.get("source", "")) if decision is not None else ""
        )
        decision_action = (
            str(decision.get("action", "")) if decision is not None else ""
        )
        banner = self._action_banner(decision_source, decision_action)
        if banner is None and detected and info is not None:
            if bool(info.get("pickup_now")):
                banner = ("PICK UP BALL", (0, 120, 0))
        if banner is not None:
            self._draw_action_banner(image, banner[0], banner[1])

    def _active_metrics_mode(self) -> str:
        """Choose the panel that matches the planner actually in use."""
        if self.metrics_mode != "auto":
            return self.metrics_mode

        decision = self._fresh_motion_command()
        if decision is not None:
            selected = str(decision.get("source", "")).strip().lower()
            enabled = {
                "line": self.show_line_metrics,
                "ball": self.show_ball_metrics,
                "goal": self.show_goal_metrics,
                "hurdle": self.show_hurdle_metrics,
            }
            if selected in enabled and enabled[selected]:
                return selected

        line_info = self._fresh_line_info()
        if (
            self.show_line_metrics
            and line_info
            and bool(line_info.get("detected", False))
        ):
            return "line"

        choices = (
            ("hurdle", self.show_hurdle_metrics, self._fresh_hurdle_info()),
            ("goal", self.show_goal_metrics, self._fresh_goal_info()),
            ("ball", self.show_ball_metrics, self._fresh_ball_info()),
        )
        for mode, enabled, info in choices:
            if enabled and info and bool(info.get("detected", False)):
                return mode
        return "line" if self.show_line_metrics else "ball"

    def _draw_line_metrics(self, image: np.ndarray) -> None:
        """Draw line geometry and the command chosen by the unified planner."""
        if not self.show_line_metrics:
            return

        height, width = image.shape[:2]
        decision = self._fresh_motion_command()
        info = self._fresh_line_info()
        self._draw_line_path_geometry(image, info)
        panel_width = min(410, max(270, width - 24))
        panel_height = 238
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        panel_slice = np.s_[
            panel_y:panel_bottom,
            panel_x:panel_x + panel_width,
        ]
        image[panel_slice] = cv2.addWeighted(
            overlay[panel_slice],
            0.72,
            image[panel_slice],
            0.28,
            0.0,
        )

        planner_source = (
            str(decision.get("source", "none")).upper()
            if decision is not None
            else "WAITING"
        )
        planner_action = (
            str(decision.get("action", "WAIT")).upper()
            if decision is not None
            else "WAIT"
        )
        detected = bool(info and info.get("detected", False))
        if not detected or info is None:
            rows = [
                "LINE METRICS",
                f"Planner     : {planner_source} / {planner_action}",
                "State       : SEARCH",
            ]
        else:
            heading = self._number(info, "filtered_heading_error_deg")
            if heading is None:
                heading = self._number(info, "heading_error_deg")
            offset = self._number(info, "filtered_lateral_offset_norm")
            if offset is None:
                offset = self._number(info, "lateral_offset_norm")
            turn = self._number(info, "turn_angle_deg")
            qualities = [
                self._number(info, "heading_quality"),
                self._number(info, "geometry_quality"),
                self._number(info, "detection_quality"),
            ]
            valid_qualities = [
                value for value in qualities if value is not None
            ]
            quality = min(valid_qualities) if valid_qualities else None
            rows = [
                "LINE METRICS",
                f"Planner     : {planner_source} / {planner_action}",
                "State       : TRACKING",
                "Heading     : "
                + self._metric_text(heading, "deg", 1, signed=True),
                "Offset norm : "
                + self._metric_text(offset, "", 3, signed=True),
                "Turn preview: "
                + self._metric_text(turn, "deg", 1, signed=True),
                "Quality     : " + self._metric_text(quality, "", 3),
            ]

        for index, row in enumerate(rows):
            title = index == 0
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if title else 0.5,
                (255, 255, 0) if title else (245, 245, 245),
                2 if title else 1,
                cv2.LINE_AA,
            )

        banner = self._action_banner(planner_source, planner_action)
        if banner is not None:
            self._draw_action_banner(image, banner[0], banner[1])

    def _draw_line_path_geometry(
        self,
        image: np.ndarray,
        info: dict[str, Any] | None,
    ) -> None:
        """Draw the near-to-far line path used by the line planner."""
        if info is None or not bool(info.get("detected", False)):
            return

        height, width = image.shape[:2]
        center_x = width // 2
        cv2.line(
            image,
            (center_x, 0),
            (center_x, height),
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "CAMERA CENTER",
            (center_x + 10, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        points = self._parse_line_center_points(
            info.get("center_points_px", [])
        )
        if len(points) >= 2:
            polyline = np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                image,
                [polyline],
                False,
                (0, 0, 0),
                8,
                cv2.LINE_AA,
            )
            cv2.polylines(
                image,
                [polyline],
                False,
                (0, 255, 0),
                4,
                cv2.LINE_AA,
            )

        for index, point in enumerate(points):
            radius = 10 if index == 0 else 7
            color = (0, 0, 255) if index == 0 else (0, 255, 255)
            cv2.circle(image, point, radius + 2, (0, 0, 0), -1)
            cv2.circle(image, point, radius, color, -1, cv2.LINE_AA)
            cv2.putText(
                image,
                str(index),
                (point[0] + 11, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if points:
            near = points[0]
            far = points[-1]
            cv2.putText(
                image,
                "NEAR",
                (near[0] + 14, min(height - 10, near[1] + 26)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                "FAR",
                (far[0] + 14, max(20, far[1] - 14)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self._draw_line_heading_arrow(image, info, near)

        offset_px = self._number(info, "lateral_offset_px")
        if offset_px is not None:
            eval_y = int(height * 0.82)
            line_x = int(np.clip(center_x + offset_px, 0, width - 1))
            cv2.line(
                image,
                (center_x, eval_y),
                (line_x, eval_y),
                (0, 165, 255),
                5,
                cv2.LINE_AA,
            )
            cv2.circle(
                image,
                (line_x, eval_y),
                9,
                (0, 165, 255),
                -1,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                f"OFFSET {offset_px:+.1f}px",
                (min(center_x, line_x), max(25, eval_y - 14)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

    @staticmethod
    def _parse_line_center_points(raw_points: Any) -> list[tuple[int, int]]:
        """Parse the analyzer's already ordered near-to-far points."""
        if not isinstance(raw_points, list):
            return []
        points: list[tuple[int, int]] = []
        for raw_point in raw_points:
            if not isinstance(raw_point, list) or len(raw_point) != 2:
                continue
            try:
                points.append(
                    (
                        int(round(float(raw_point[0]))),
                        int(round(float(raw_point[1]))),
                    )
                )
            except (TypeError, ValueError):
                continue
        return points

    def _draw_line_heading_arrow(
        self,
        image: np.ndarray,
        info: dict[str, Any],
        near: tuple[int, int],
    ) -> None:
        """Draw the filtered local heading from the nearest path point."""
        heading = self._number(info, "filtered_heading_error_deg")
        if heading is None:
            heading = self._number(info, "heading_error_deg")
        if heading is None:
            return
        arrow_length = max(90, min(180, int(image.shape[0] * 0.25)))
        angle_rad = np.deg2rad(heading)
        end = (
            int(round(near[0] + arrow_length * np.sin(angle_rad))),
            int(round(near[1] - arrow_length * np.cos(angle_rad))),
        )
        cv2.arrowedLine(
            image,
            near,
            end,
            (255, 0, 255),
            5,
            cv2.LINE_AA,
            tipLength=0.15,
        )
        cv2.putText(
            image,
            f"HEADING {heading:+.1f}deg",
            (end[0] + 8, max(22, end[1])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_goal_metrics(self, image: np.ndarray) -> None:
        """Draw analyzed goal distance and scoring data in-place."""
        height, width = image.shape[:2]
        center_x = width // 2
        cv2.line(
            image,
            (center_x, max(45, int(height * 0.32))),
            (center_x, height - 1),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        info = self._fresh_goal_info()
        decision = self._fresh_motion_command()
        if decision is None:
            planner_action = "NO COMMAND"
        else:
            planner_source = str(decision.get("source", "none")).upper()
            selected_action = str(decision.get("action", "WAIT")).upper()
            planner_action = f"{planner_source} / {selected_action}"
        detected = bool(info and info.get("detected", False))
        if detected and info is not None:
            target_x = self._number(info, "center_x")
            target_y = self._number(info, "center_y")
            if target_x is not None and target_y is not None:
                point_x = int(np.clip(round(target_x), 0, width - 1))
                point_y = int(np.clip(round(target_y), 0, height - 1))
                cv2.arrowedLine(
                    image,
                    (center_x, point_y),
                    (point_x, point_y),
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.12,
                )
                offset_x = self._number(info, "offset_x_px")
                offset_text = self._metric_text(
                    offset_x,
                    "px",
                    digits=0,
                    signed=True,
                )
                text_x = min(center_x, point_x) + 8
                cv2.putText(
                    image,
                    f"dx {offset_text}",
                    (text_x, max(22, point_y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        panel_width = min(390, max(250, width - 24))
        panel_height = 310
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        image[panel_y:panel_bottom, panel_x:panel_x + panel_width] = (
            cv2.addWeighted(
                overlay[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.72,
                image[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.28,
                0.0,
            )
        )

        if info is None:
            rows = [
                "GOAL METRICS",
                f"Planner     : {planner_action}",
                "NO GOAL INFO",
            ]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = [
                "GOAL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state}",
            ]
        else:
            distance = self._number(info, "distance_m")
            depth = self._number(info, "depth_m")
            offset_px = self._number(info, "offset_x_px")
            offset_norm = self._number(info, "offset_x_norm")
            bearing = self._number(info, "bearing_deg")
            lateral = self._number(info, "lateral_offset_m")
            sample_count = self._number(info, "depth_sample_count")
            aim_source = str(info.get("aim_source", "goal")).upper()
            direction = str(info.get("horizontal_direction", "UNKNOWN"))
            state = str(info.get("state", "UNKNOWN"))
            rows = [
                "GOAL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state} / {direction}",
                f"Aim source  : {aim_source}",
                "Distance    : " + self._metric_text(distance, "m"),
                "Depth Z     : " + self._metric_text(depth, "m"),
                "Offset X    : "
                + self._metric_text(offset_px, "px", 0, signed=True),
                "Offset norm : "
                + self._metric_text(offset_norm, "", 3, signed=True),
                "Bearing     : "
                + self._metric_text(bearing, "deg", 1, signed=True),
                "Lateral X   : "
                + self._metric_text(lateral, "m", 3, signed=True),
                "Depth points: "
                + self._metric_text(sample_count, "", 0),
                f"Shot now    : {'YES' if info.get('score_now') else 'NO'}",
            ]

        for index, row in enumerate(rows):
            is_title = index == 0
            color = (0, 255, 0) if is_title else (245, 245, 245)
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if is_title else 0.5,
                color,
                2 if is_title else 1,
                cv2.LINE_AA,
            )

        if detected and info is not None and bool(info.get("score_now")):
            score_text = "SHOT"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 3
            (text_width, text_height), _ = cv2.getTextSize(
                score_text,
                font,
                font_scale,
                thickness,
            )
            text_x = max(12, (width - text_width) // 2)
            text_y = 72
            cv2.rectangle(
                image,
                (text_x - 14, text_y - text_height - 14),
                (text_x + text_width + 14, text_y + 14),
                (0, 120, 0),
                -1,
            )
            cv2.putText(
                image,
                score_text,
                (text_x, text_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )

    def _draw_hurdle_metrics(self, image: np.ndarray) -> None:
        """Draw hurdle alignment, depth, width, and jump readiness."""
        if not self.show_hurdle_metrics:
            return

        height, width = image.shape[:2]
        info = self._fresh_hurdle_info()
        detected = bool(info and info.get("detected", False))

        panel_width = min(410, max(260, width - 24))
        panel_height = 310
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        panel_slice = np.s_[
            panel_y:panel_bottom,
            panel_x:panel_x + panel_width,
        ]
        image[panel_slice] = cv2.addWeighted(
            overlay[panel_slice],
            0.72,
            image[panel_slice],
            0.28,
            0.0,
        )

        if info is None:
            rows = ["HURDLE METRICS", "NO HURDLE INFO"]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = ["HURDLE METRICS", f"State       : {state}"]
        else:
            state = str(info.get("state", "UNKNOWN"))
            rows = [
                "HURDLE METRICS",
                f"State       : {state}",
                "Depth Z     : "
                + self._metric_text(self._number(info, "depth_m"), "m"),
                "Ground gap  : "
                + self._metric_text(
                    self._number(info, "ground_gap_m"),
                    "m",
                ),
                "Bottom gap  : "
                + self._metric_text(
                    self._number(info, "camera_bottom_gap_m"),
                    "m",
                ),
                "Left depth  : "
                + self._metric_text(
                    self._number(info, "left_depth_m"),
                    "m",
                ),
                "Right depth : "
                + self._metric_text(
                    self._number(info, "right_depth_m"),
                    "m",
                ),
                "Width est.  : "
                + self._metric_text(
                    self._number(info, "estimated_width_m"),
                    "m",
                ),
                "Parallel err: "
                + self._metric_text(
                    self._number(info, "hurdle_angle_deg"),
                    "deg",
                    1,
                    signed=True,
                ),
                f"Parallel    : {'YES' if info.get('is_parallel') else 'NO'}",
                "Depth points: "
                + str(int(info.get("depth_sample_count", 0))),
                f"Go now      : {'YES' if info.get('go_now') else 'NO'}",
            ]

        for index, row in enumerate(rows):
            title = index == 0
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if title else 0.5,
                (0, 255, 255) if title else (245, 245, 245),
                2 if title else 1,
                cv2.LINE_AA,
            )

        if detected and info is not None and bool(info.get("go_now")):
            go_text = "GO!"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.4
            thickness = 4
            (text_width, text_height), _ = cv2.getTextSize(
                go_text,
                font,
                font_scale,
                thickness,
            )
            text_x = max(12, (width - text_width) // 2)
            text_y = 76
            cv2.rectangle(
                image,
                (text_x - 18, text_y - text_height - 14),
                (text_x + text_width + 18, text_y + 14),
                (0, 140, 0),
                -1,
            )
            cv2.putText(
                image,
                go_text,
                (text_x, text_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )

    def _draw_detections(
        self, image: np.ndarray, detections: list[Detection]
    ) -> np.ndarray:
        annotated = image.copy()
        metrics_mode = self._active_metrics_mode()
        decision = self._fresh_motion_command()
        ball_info = self._fresh_ball_info()
        goal_info = self._fresh_goal_info()
        hurdle_info = self._fresh_hurdle_info()
        line_info = self._fresh_line_info()
        ball_detected = bool(ball_info and ball_info.get("detected", False))
        goal_detected = bool(goal_info and goal_info.get("detected", False))
        decision_source = (
            str(decision.get("source", "")).lower()
            if decision is not None
            else ""
        )
        decision_action = (
            str(decision.get("action", "")).upper()
            if decision is not None
            else ""
        )
        ball_recovery = (
            metrics_mode == "ball"
            and decision_source == "ball"
            and not ball_detected
            and (
                decision_action == "BALL_LOST_STOP"
                or decision_action.startswith("RECOVER_TURN_")
            )
        )
        goal_recovery = (
            metrics_mode == "goal"
            and decision_source == "goal"
            and not goal_detected
            and (
                decision_action == "GOAL_LOST_STOP"
                or decision_action.startswith("RECOVER_GOAL_TURN_")
            )
        )
        show_recovery_line = (
            (ball_recovery or goal_recovery)
            and line_info is not None
            and bool(line_info.get("detected", False))
        )
        for detection in detections:
            if (
                metrics_mode == "line" or show_recovery_line
            ) and detection.class_name == "line":
                continue
            left, top, right, bottom = detection.bbox
            color = self._color_for_class(detection.class_id)
            label = f"{detection.class_name} {detection.confidence:.2f}"
            if metrics_mode == "line" and detection.class_name == "ball":
                ball_info = self._fresh_ball_info()
                ball_depth = (
                    self._number(ball_info, "depth_m")
                    if ball_info is not None
                    else None
                )
                if (
                    ball_depth is not None
                    and ball_depth > self.ball_control_range_m
                ):
                    label = f"ball TRACK ONLY {ball_depth:.2f}m"
            if metrics_mode == "line" and detection.class_name == "goal":
                goal_depth = (
                    self._number(goal_info, "depth_m")
                    if goal_info is not None
                    else None
                )
                if (
                    goal_depth is not None
                    and goal_depth > self.goal_control_range_m
                ):
                    label = f"goal TRACK ONLY {goal_depth:.2f}m"
            confirmation_info = {
                "line": line_info,
                "ball": ball_info,
                "goal": goal_info,
                "backboard": goal_info,
                "hurdle": hurdle_info,
            }.get(detection.class_name)
            badge = self._confirmation_badge(confirmation_info)
            label = f"{label} | {badge}"
            cv2.rectangle(annotated, (left, top), (right, bottom), color, 2)
            cv2.circle(annotated, tuple(detection.center), 4, color, -1)
            cv2.putText(
                annotated,
                label,
                (left, max(20, top - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        if metrics_mode == "line":
            self._draw_line_metrics(annotated)
        elif metrics_mode == "hurdle":
            self._draw_hurdle_metrics(annotated)
        elif metrics_mode == "goal":
            if show_recovery_line and line_info is not None:
                self._draw_line_path_geometry(annotated, line_info)
            self._draw_goal_metrics(annotated)
        else:
            if show_recovery_line and line_info is not None:
                self._draw_line_path_geometry(annotated, line_info)
            self._draw_ball_metrics(annotated)
        cv2.putText(
            annotated,
            f"YOLO26 | {self.active_provider} | {self.smoothed_fps:.1f} FPS",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return annotated

    def _publish_detections(
        self, message: Image, detections: list[Detection]
    ) -> None:
        payload = {
            "stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "frame_id": message.header.frame_id,
            "image_width": int(message.width),
            "image_height": int(message.height),
            "detections": [asdict(detection) for detection in detections],
        }
        output = String()
        output.data = json.dumps(
            payload, ensure_ascii=True, separators=(",", ":")
        )
        self.detections_publisher.publish(output)

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        minimum_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
        waiting_for_interval = (
            now - self.last_inference_time < minimum_interval
        )
        if self.processing or waiting_for_interval:
            return

        self.processing = True
        started = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            blob, info = self._preprocess(image)
            output = self._run_inference(blob)
            detections = self._postprocess(output, info, image.shape)
            elapsed = max(time.perf_counter() - started, 1e-6)
            current_fps = 1.0 / elapsed
            self.smoothed_fps = (
                current_fps
                if self.smoothed_fps == 0.0
                else self.smoothed_fps * 0.9 + current_fps * 0.1
            )

            self._publish_detections(message, detections)
            if self.publish_annotated_image or self.display:
                annotated = self._draw_detections(image, detections)

                if self.publish_annotated_image:
                    annotated_message = self.bridge.cv2_to_imgmsg(
                        annotated, encoding="bgr8"
                    )
                    annotated_message.header = message.header
                    self.annotated_publisher.publish(annotated_message)

                if self.display:
                    display_image = annotated
                    if self.show_camera_controls:
                        display_image = self._draw_camera_control_panel(
                            annotated
                        )
                    cv2.imshow(DISPLAY_WINDOW_NAME, display_image)
                    key = cv2.waitKey(1) & 0xFF
                    if (
                        self.show_camera_controls
                        and key in {ord("r"), ord("R")}
                    ):
                        self._reset_camera_controls()
        except Exception as exc:
            self.get_logger().error(f"YOLO26 inference failed: {exc}")
        finally:
            self.last_inference_time = time.monotonic()
            self.processing = False

    def destroy_node(self) -> bool:
        if self.tensorrt_backend is not None:
            self.tensorrt_backend.close()
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: Yolo26Detector | None = None
    try:
        node = Yolo26Detector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        rclpy.logging.get_logger("yolo26_detector").fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
