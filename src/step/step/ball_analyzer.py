#!/usr/bin/env python3
"""Analyze YOLO26 ball detections and RealSense depth for pickup logic."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


@dataclass(frozen=True)
class BallCandidate:
    """One ball candidate with image geometry and optional depth."""

    confidence: float
    bbox: list[int]
    center: list[int]
    width_px: int
    height_px: int
    area_px: int
    offset_x_px: int
    offset_y_px: int
    offset_x_norm: float
    offset_y_norm: float
    horizontal_direction: str
    bearing_deg: float | None
    elevation_deg: float | None
    depth_m: float | None
    lateral_offset_m: float | None
    vertical_offset_m: float | None
    horizontal_distance_m: float | None
    distance_m: float | None
    depth_valid: bool
    score: float


@dataclass(frozen=True)
class BallInfo:
    """Ball target information for future mission/behavior nodes."""

    detected: bool
    state: str
    confidence: float
    center_x: int | None
    center_y: int | None
    bbox: list[int] | None
    width_px: int | None
    height_px: int | None
    area_px: int | None
    offset_x_px: int | None
    offset_y_px: int | None
    offset_x_norm: float | None
    offset_y_norm: float | None
    horizontal_direction: str
    bearing_deg: float | None
    elevation_deg: float | None
    depth_m: float | None
    lateral_offset_m: float | None
    vertical_offset_m: float | None
    horizontal_distance_m: float | None
    distance_m: float | None
    depth_valid: bool
    is_centered: bool
    is_close: bool
    approach_ready: bool
    pickup_ready: bool
    pickup_now: bool
    target_priority_score: float
    candidate_count: int
    candidates: list[BallCandidate]
    image_width: int | None
    image_height: int | None
    image_center_x: float | None
    image_center_y: float | None
    camera_info_ready: bool
    depth_age_sec: float | None
    note: str


class BallAnalyzer(Node):
    """Convert raw YOLO ball detections into stable ball target information.

    This node does not decide robot motion. The mission/behavior layer should
    still check the current mission phase before allowing a pickup command.
    That is what prevents a far Ball B from stealing priority while the robot
    is scoring at Goal A.
    """

    def __init__(self) -> None:
        super().__init__("ball_analyzer")

        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter("output_topic", "/vision/ball_info")
        self.declare_parameter("ball_class_name", "ball")
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("depth_timeout_sec", 0.7)
        self.declare_parameter("depth_window_px", 9)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("detect_depth_m", 2.0)
        self.declare_parameter("approach_depth_m", 1.4)
        self.declare_parameter("pickup_ready_depth_m", 0.9)
        self.declare_parameter("pickup_now_depth_m", 0.8)
        self.declare_parameter("pickup_depth_tolerance_m", 0.05)
        self.declare_parameter("pickup_center_tolerance_norm", 0.08)
        self.declare_parameter("horizontal_deadband_px", 20)
        self.declare_parameter("center_tolerance_px", 140)
        self.declare_parameter("center_y_min_ratio_for_pickup", 0.45)
        self.declare_parameter("publish_empty_when_missing", True)

        self.detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.ball_class_name = str(
            self.get_parameter("ball_class_name").value
        )
        self.min_confidence = float(
            self.get_parameter("min_confidence").value
        )
        self.depth_timeout_sec = float(
            self.get_parameter("depth_timeout_sec").value
        )
        self.depth_window_px = max(
            3,
            int(self.get_parameter("depth_window_px").value),
        )
        if self.depth_window_px % 2 == 0:
            self.depth_window_px += 1
        self.max_valid_depth_m = float(
            self.get_parameter("max_valid_depth_m").value
        )
        self.detect_depth_m = float(self.get_parameter("detect_depth_m").value)
        self.approach_depth_m = float(
            self.get_parameter("approach_depth_m").value
        )
        self.pickup_ready_depth_m = float(
            self.get_parameter("pickup_ready_depth_m").value
        )
        self.pickup_now_depth_m = float(
            self.get_parameter("pickup_now_depth_m").value
        )
        self.pickup_depth_tolerance_m = max(
            0.0,
            float(self.get_parameter("pickup_depth_tolerance_m").value),
        )
        self.pickup_center_tolerance_norm = max(
            0.0,
            float(
                self.get_parameter("pickup_center_tolerance_norm").value
            ),
        )
        self.horizontal_deadband_px = max(
            0,
            int(self.get_parameter("horizontal_deadband_px").value),
        )
        self.center_tolerance_px = int(
            self.get_parameter("center_tolerance_px").value
        )
        self.center_y_min_ratio_for_pickup = float(
            self.get_parameter("center_y_min_ratio_for_pickup").value
        )
        self.publish_empty_when_missing = bool(
            self.get_parameter("publish_empty_when_missing").value
        )

        self.bridge = CvBridge()
        self.latest_depth_image: np.ndarray | None = None
        self.latest_depth_time: float | None = None
        self.latest_image_width: int | None = None
        self.latest_image_height: int | None = None
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(
            String,
            self.detections_topic,
            self._detections_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
        )

        self.get_logger().info(
            f"Subscribing detections: {self.detections_topic}"
        )
        self.get_logger().info(
            f"Subscribing aligned depth: {self.depth_topic}"
        )
        self.get_logger().info(
            f"Subscribing camera info: {self.camera_info_topic}"
        )
        self.get_logger().info(f"Publishing ball info: {self.output_topic}")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Store color-camera intrinsics used for angle and 3-D position."""
        if len(message.k) < 6:
            return
        fx = float(message.k[0])
        fy = float(message.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return
        self.fx = fx
        self.fy = fy
        self.cx = float(message.k[2])
        self.cy = float(message.k[5])

    def _depth_callback(self, message: Image) -> None:
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
            self.latest_depth_image = np.asarray(depth_image)
            self.latest_depth_time = time.monotonic()
            self.latest_image_width = int(message.width)
            self.latest_image_height = int(message.height)
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")

    def _depth_age_sec(self) -> float | None:
        if self.latest_depth_time is None:
            return None
        return time.monotonic() - self.latest_depth_time

    def _depth_is_fresh(self) -> bool:
        age = self._depth_age_sec()
        return age is not None and age <= self.depth_timeout_sec

    def _sample_depth_m(
        self,
        center_x: int,
        center_y: int,
    ) -> tuple[float | None, bool]:
        if self.latest_depth_image is None or not self._depth_is_fresh():
            return None, False

        depth_image = self.latest_depth_image
        if depth_image.ndim != 2:
            return None, False

        height, width = depth_image.shape[:2]
        if not (0 <= center_x < width and 0 <= center_y < height):
            return None, False

        radius = self.depth_window_px // 2
        x1 = max(0, center_x - radius)
        x2 = min(width, center_x + radius + 1)
        y1 = max(0, center_y - radius)
        y2 = min(height, center_y + radius + 1)

        crop = depth_image[y1:y2, x1:x2]
        if crop.dtype == np.uint16:
            crop_m = crop.astype(np.float32) * 0.001
        else:
            crop_m = crop.astype(np.float32)
        valid_mask = (
            np.isfinite(crop_m)
            & (crop_m > 0.05)
            & (crop_m <= self.max_valid_depth_m)
        )
        valid = crop_m[valid_mask]
        if valid.size == 0:
            return None, False

        return float(np.median(valid)), True

    def _image_size_from_payload(
        self,
        payload: dict[str, Any],
        detections: list[dict[str, Any]],
    ) -> tuple[int | None, int | None]:
        """Get the source image size without estimating it from one object."""
        try:
            payload_width = int(payload.get("image_width", 0))
            payload_height = int(payload.get("image_height", 0))
        except (TypeError, ValueError):
            payload_width = 0
            payload_height = 0
        if payload_width > 0 and payload_height > 0:
            return payload_width, payload_height

        if (
            self.latest_image_width is not None
            and self.latest_image_height is not None
        ):
            return self.latest_image_width, self.latest_image_height

        max_x = 0
        max_y = 0
        for detection in detections:
            bbox = detection.get("bbox", [])
            if isinstance(bbox, list) and len(bbox) == 4:
                max_x = max(max_x, int(bbox[2]))
                max_y = max(max_y, int(bbox[3]))
        if max_x > 0 and max_y > 0:
            return max_x + 1, max_y + 1
        return None, None

    @staticmethod
    def _closeness_score(depth_m: float | None, valid: bool) -> float:
        if not valid or depth_m is None:
            return 0.0
        return max(0.0, min(1.0, 1.0 - depth_m / 2.0))

    def _project_ball_position(
        self,
        center_x: int,
        center_y: int,
        depth_m: float | None,
        depth_valid: bool,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
    ]:
        """Convert one image point to camera-relative angles and position.

        Camera coordinates follow the ROS optical convention: x is right,
        y is down, and z is forward. Angles are positive right/down.
        """
        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return None, None, None, None, None, None

        x_ratio = (center_x - self.cx) / self.fx
        y_ratio = (center_y - self.cy) / self.fy
        bearing_deg = math.degrees(math.atan(x_ratio))
        elevation_deg = math.degrees(math.atan(y_ratio))
        if not depth_valid or depth_m is None:
            return bearing_deg, elevation_deg, None, None, None, None

        lateral_offset_m = x_ratio * depth_m
        vertical_offset_m = y_ratio * depth_m
        horizontal_distance_m = math.hypot(lateral_offset_m, depth_m)
        distance_m = math.sqrt(
            lateral_offset_m * lateral_offset_m
            + vertical_offset_m * vertical_offset_m
            + depth_m * depth_m
        )
        return (
            bearing_deg,
            elevation_deg,
            lateral_offset_m,
            vertical_offset_m,
            horizontal_distance_m,
            distance_m,
        )

    def _build_candidate(
        self,
        detection: dict[str, Any],
        image_width: int | None,
        image_height: int | None,
    ) -> BallCandidate | None:
        try:
            confidence = float(detection.get("confidence", 0.0))
            bbox = [int(value) for value in detection.get("bbox", [])]
            center = [int(value) for value in detection.get("center", [])]
        except (TypeError, ValueError):
            return None

        if (
            confidence < self.min_confidence
            or len(bbox) != 4
            or len(center) != 2
        ):
            return None

        left, top, right, bottom = bbox
        width_px = max(0, right - left)
        height_px = max(0, bottom - top)
        if width_px <= 0 or height_px <= 0:
            return None

        center_x, center_y = center
        offset_x_px = 0
        offset_y_px = 0
        offset_x_norm = 0.0
        offset_y_norm = 0.0
        if image_width and image_height:
            offset_x_px = int(center_x - image_width / 2)
            offset_y_px = int(center_y - image_height / 2)
            offset_x_norm = offset_x_px / max(image_width / 2, 1.0)
            offset_y_norm = offset_y_px / max(image_height / 2, 1.0)

        depth_m, depth_valid = self._sample_depth_m(center_x, center_y)
        (
            bearing_deg,
            elevation_deg,
            lateral_offset_m,
            vertical_offset_m,
            horizontal_distance_m,
            distance_m,
        ) = self._project_ball_position(
            center_x,
            center_y,
            depth_m,
            depth_valid,
        )
        if offset_x_px < -self.horizontal_deadband_px:
            horizontal_direction = "LEFT"
        elif offset_x_px > self.horizontal_deadband_px:
            horizontal_direction = "RIGHT"
        else:
            horizontal_direction = "CENTER"
        area_px = width_px * height_px
        center_score = max(0.0, 1.0 - abs(offset_x_norm))
        depth_score = self._closeness_score(depth_m, depth_valid)
        area_score = min(1.0, math.sqrt(area_px) / 180.0)
        score = (
            confidence * 0.45
            + center_score * 0.25
            + depth_score * 0.20
            + area_score * 0.10
        )

        return BallCandidate(
            confidence=round(confidence, 4),
            bbox=bbox,
            center=center,
            width_px=width_px,
            height_px=height_px,
            area_px=area_px,
            offset_x_px=offset_x_px,
            offset_y_px=offset_y_px,
            offset_x_norm=round(offset_x_norm, 4),
            offset_y_norm=round(offset_y_norm, 4),
            horizontal_direction=horizontal_direction,
            bearing_deg=(
                round(bearing_deg, 3) if bearing_deg is not None else None
            ),
            elevation_deg=(
                round(elevation_deg, 3)
                if elevation_deg is not None
                else None
            ),
            depth_m=round(depth_m, 3) if depth_m is not None else None,
            lateral_offset_m=(
                round(lateral_offset_m, 3)
                if lateral_offset_m is not None
                else None
            ),
            vertical_offset_m=(
                round(vertical_offset_m, 3)
                if vertical_offset_m is not None
                else None
            ),
            horizontal_distance_m=(
                round(horizontal_distance_m, 3)
                if horizontal_distance_m is not None
                else None
            ),
            distance_m=(
                round(distance_m, 3) if distance_m is not None else None
            ),
            depth_valid=depth_valid,
            score=round(score, 4),
        )

    def _state_for_candidate(
        self,
        candidate: BallCandidate,
        image_height: int | None,
    ) -> tuple[str, bool, bool, bool, bool, bool, str]:
        centered = abs(candidate.offset_x_px) <= self.center_tolerance_px
        close = (
            candidate.depth_valid
            and candidate.depth_m is not None
            and candidate.depth_m <= self.detect_depth_m
        )
        approach_ready = (
            candidate.depth_valid
            and candidate.depth_m is not None
            and candidate.depth_m <= self.approach_depth_m
        )

        low_enough = True
        if image_height:
            center_y_ratio = candidate.center[1] / max(image_height, 1)
            low_enough = (
                center_y_ratio >= self.center_y_min_ratio_for_pickup
            )

        pickup_ready = (
            candidate.depth_valid
            and candidate.depth_m is not None
            and candidate.depth_m <= self.pickup_ready_depth_m
            and centered
            and low_enough
        )
        pickup_now = (
            candidate.depth_valid
            and candidate.depth_m is not None
            and candidate.depth_m
            <= self.pickup_now_depth_m + self.pickup_depth_tolerance_m
            and abs(candidate.offset_x_norm)
            <= self.pickup_center_tolerance_norm
            and low_enough
        )

        if pickup_now:
            return (
                "PICKUP_NOW",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                "close_centered_ball_ready_to_pick",
            )
        if pickup_ready:
            return (
                "PICKUP_READY",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                "slow_down_and_prepare_pickup",
            )
        if approach_ready:
            return (
                "APPROACH",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                "ball_close_enough_to_approach",
            )
        if close:
            return (
                "TRACK",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                "ball_visible_with_depth",
            )
        if candidate.depth_valid:
            return (
                "FAR",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                "ball_detected_but_far",
            )
        return (
            "NO_DEPTH",
            centered,
            close,
            approach_ready,
            pickup_ready,
            pickup_now,
            "ball_detected_without_valid_depth",
        )

    def _publish(self, info: BallInfo) -> None:
        message = String()
        message.data = json.dumps(
            asdict(info),
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)

    def _empty_info(self, note: str = "no_ball_detection") -> BallInfo:
        age = self._depth_age_sec()
        return BallInfo(
            detected=False,
            state="SEARCH",
            confidence=0.0,
            center_x=None,
            center_y=None,
            bbox=None,
            width_px=None,
            height_px=None,
            area_px=None,
            offset_x_px=None,
            offset_y_px=None,
            offset_x_norm=None,
            offset_y_norm=None,
            horizontal_direction="UNKNOWN",
            bearing_deg=None,
            elevation_deg=None,
            depth_m=None,
            lateral_offset_m=None,
            vertical_offset_m=None,
            horizontal_distance_m=None,
            distance_m=None,
            depth_valid=False,
            is_centered=False,
            is_close=False,
            approach_ready=False,
            pickup_ready=False,
            pickup_now=False,
            target_priority_score=0.0,
            candidate_count=0,
            candidates=[],
            image_width=self.latest_image_width,
            image_height=self.latest_image_height,
            image_center_x=(
                self.latest_image_width / 2.0
                if self.latest_image_width is not None
                else None
            ),
            image_center_y=(
                self.latest_image_height / 2.0
                if self.latest_image_height is not None
                else None
            ),
            camera_info_ready=self.fx is not None,
            depth_age_sec=round(age, 3) if age is not None else None,
            note=note,
        )

    def _detections_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "Invalid detections message: "
                f"{type(exc).__name__}: {exc}"
            )
            return

        image_width, image_height = self._image_size_from_payload(
            payload,
            detections,
        )
        candidates: list[BallCandidate] = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            if str(detection.get("class_name", "")) != self.ball_class_name:
                continue
            candidate = self._build_candidate(
                detection,
                image_width,
                image_height,
            )
            if candidate is not None:
                candidates.append(candidate)

        candidates.sort(key=lambda item: item.score, reverse=True)
        if not candidates:
            if self.publish_empty_when_missing:
                self._publish(self._empty_info())
            return

        target = candidates[0]
        (
            state,
            centered,
            close,
            approach_ready,
            pickup_ready,
            pickup_now,
            note,
        ) = self._state_for_candidate(target, image_height)
        age = self._depth_age_sec()

        self._publish(
            BallInfo(
                detected=True,
                state=state,
                confidence=target.confidence,
                center_x=target.center[0],
                center_y=target.center[1],
                bbox=target.bbox,
                width_px=target.width_px,
                height_px=target.height_px,
                area_px=target.area_px,
                offset_x_px=target.offset_x_px,
                offset_y_px=target.offset_y_px,
                offset_x_norm=target.offset_x_norm,
                offset_y_norm=target.offset_y_norm,
                horizontal_direction=target.horizontal_direction,
                bearing_deg=target.bearing_deg,
                elevation_deg=target.elevation_deg,
                depth_m=target.depth_m,
                lateral_offset_m=target.lateral_offset_m,
                vertical_offset_m=target.vertical_offset_m,
                horizontal_distance_m=target.horizontal_distance_m,
                distance_m=target.distance_m,
                depth_valid=target.depth_valid,
                is_centered=centered,
                is_close=close,
                approach_ready=approach_ready,
                pickup_ready=pickup_ready,
                pickup_now=pickup_now,
                target_priority_score=target.score,
                candidate_count=len(candidates),
                candidates=candidates[:5],
                image_width=image_width,
                image_height=image_height,
                image_center_x=(
                    image_width / 2.0 if image_width is not None else None
                ),
                image_center_y=(
                    image_height / 2.0 if image_height is not None else None
                ),
                camera_info_ready=self.fx is not None,
                depth_age_sec=round(age, 3) if age is not None else None,
                note=note,
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: BallAnalyzer | None = None
    try:
        node = BallAnalyzer()
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
