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
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


DEFAULT_MODEL_PATH = "/home/geonwoo/Desktop/realsense/dataset/best.onnx"
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
            "image_topic", "/camera/camera/color/image_raw"
        )
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "annotated_image_topic", "/vision/detections/image"
        )
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

        self.last_inference_time = 0.0
        self.smoothed_fps = 0.0
        self.processing = False

        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Provider: {self.active_provider}")
        self.get_logger().info(
            f"Input: {self.input_width}x{self.input_height}"
        )
        self.get_logger().info(f"Classes: {self.class_names}")
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Publishing: {detections_topic}")

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

    def _draw_detections(
        self, image: np.ndarray, detections: list[Detection]
    ) -> np.ndarray:
        annotated = image.copy()
        for detection in detections:
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
