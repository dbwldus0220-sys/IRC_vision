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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


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
        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("goal_info_timeout_sec", 0.8)
        self.declare_parameter("show_goal_metrics", True)
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
        self.goal_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("goal_info_timeout_sec").value),
        )
        self.show_goal_metrics = bool(
            self.get_parameter("show_goal_metrics").value
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
        self.session, self.active_provider = self._create_session(
            str(self.get_parameter("device").value)
        )
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        self.input_height = self._fixed_dimension(input_shape[2], 640)
        self.input_width = self._fixed_dimension(input_shape[3], 640)
        self.class_names = self._read_class_names()

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

        self.get_logger().info(f"Model: {self.model_path}")
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
        self.get_logger().info(f"Publishing: {detections_topic}")

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
            rows = ["BALL METRICS", "NO BALL INFO"]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = ["BALL METRICS", f"State       : {state}"]
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

        if detected and info is not None and bool(info.get("pickup_now")):
            pickup_text = "PICK UP BALL"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 3
            (text_width, text_height), _ = cv2.getTextSize(
                pickup_text,
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
                pickup_text,
                (text_x, text_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )

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

        choices = (
            ("hurdle", self.show_hurdle_metrics, self._fresh_hurdle_info()),
            ("goal", self.show_goal_metrics, self._fresh_goal_info()),
            ("ball", self.show_ball_metrics, self._fresh_ball_info()),
        )
        for mode, enabled, info in choices:
            if enabled and info and bool(info.get("detected", False)):
                return mode
        line_info = self._fresh_line_info()
        if (
            self.show_line_metrics
            and line_info
            and bool(line_info.get("detected", False))
        ):
            return "line"
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
        panel_height = 286
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
            rows = ["GOAL METRICS", "NO GOAL INFO"]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = ["GOAL METRICS", f"State       : {state}"]
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
                f"Score now   : {'YES' if info.get('score_now') else 'NO'}",
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
            score_text = "SCORE GOAL"
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
        center_x = width // 2
        cv2.line(
            image,
            (center_x, max(45, int(height * 0.32))),
            (center_x, height - 1),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        info = self._fresh_hurdle_info()
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
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.12,
                )
                offset = self._number(info, "offset_x_px")
                cv2.putText(
                    image,
                    "dx "
                    + self._metric_text(offset, "px", 0, signed=True),
                    (min(center_x, point_x) + 8, max(22, point_y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

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
            direction = str(info.get("horizontal_direction", "UNKNOWN"))
            rows = [
                "HURDLE METRICS",
                f"State       : {state} / {direction}",
                "Distance XZ : "
                + self._metric_text(
                    self._number(info, "horizontal_distance_m"),
                    "m",
                ),
                "Depth Z     : "
                + self._metric_text(self._number(info, "depth_m"), "m"),
                "Offset X    : "
                + self._metric_text(
                    self._number(info, "offset_x_px"),
                    "px",
                    0,
                    signed=True,
                ),
                "Offset norm : "
                + self._metric_text(
                    self._number(info, "offset_x_norm"),
                    "",
                    3,
                    signed=True,
                ),
                "Bearing     : "
                + self._metric_text(
                    self._number(info, "bearing_deg"),
                    "deg",
                    1,
                    signed=True,
                ),
                "Lateral X   : "
                + self._metric_text(
                    self._number(info, "lateral_offset_m"),
                    "m",
                    3,
                    signed=True,
                ),
                "Width est.  : "
                + self._metric_text(
                    self._number(info, "estimated_width_m"),
                    "m",
                ),
                "Hurdle angle: "
                + self._metric_text(
                    self._number(info, "hurdle_angle_deg"),
                    "deg",
                    1,
                    signed=True,
                ),
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
                (255, 255, 0) if title else (245, 245, 245),
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
        for detection in detections:
            if metrics_mode == "line" and detection.class_name == "line":
                continue
            left, top, right, bottom = detection.bbox
            color = self._color_for_class(detection.class_id)
            label = f"{detection.class_name} {detection.confidence:.2f}"
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
            self._draw_goal_metrics(annotated)
        else:
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
            outputs = self.session.run(None, {self.input_name: blob})
            detections = self._postprocess(outputs[0], info, image.shape)
            elapsed = max(time.perf_counter() - started, 1e-6)
            current_fps = 1.0 / elapsed
            self.smoothed_fps = (
                current_fps
                if self.smoothed_fps == 0.0
                else self.smoothed_fps * 0.9 + current_fps * 0.1
            )

            self._publish_detections(message, detections)
            annotated = self._draw_detections(image, detections)

            if self.publish_annotated_image:
                annotated_message = self.bridge.cv2_to_imgmsg(
                    annotated, encoding="bgr8"
                )
                annotated_message.header = message.header
                self.annotated_publisher.publish(annotated_message)

            if self.display:
                cv2.imshow("YOLO26 RealSense Detection", annotated)
                cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f"YOLO26 inference failed: {exc}")
        finally:
            self.last_inference_time = time.monotonic()
            self.processing = False

    def destroy_node(self) -> bool:
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
