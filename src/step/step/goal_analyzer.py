#!/usr/bin/env python3
"""Analyze YOLO goal detections and aligned depth for scoring logic."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .goal_scoring import (
    GOAL_SCORE_MIN_DEPTH_M, GOAL_SCORE_TARGET_DEPTH_M, GOAL_SCORE_MAX_DEPTH_M,
    goal_shot_pixels_aligned, goal_shot_region,
)
from .depth_frame_cache import DepthFrameCache
from .depth_frame_cache import DepthFrameConsumer
from .temporal_confirmation import depth_is_within_range
from .temporal_confirmation import TemporalConfirmationFilter
from .yolo_line_analyzer import calibrated_robot_center_x


@dataclass(frozen=True)
class GoalCandidate:
    """One goal candidate with image and camera-relative geometry."""

    confidence: float
    bbox: list[int]
    aim_source: str
    aim_bbox: list[int]
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
    distance_m: float | None
    ground_distance_m: float | None
    lateral_offset_m: float | None
    left_depth_m: float | None
    center_depth_m: float | None
    right_depth_m: float | None
    backboard_angle_deg: float | None
    parallel_correction_deg: float | None
    backboard_angle_direction: str
    backboard_angle_valid: bool
    backboard_angle_fit_error_m: float | None
    backboard_measured_width_m: float | None
    backboard_line_width_m: float
    backboard_sample_points_px: list[list[int]]
    backboard_depth_sample_count: int
    waist_depth_m: float | None
    waist_aim_deg: float | None
    waist_turn_direction: str
    ring_center_lateral_m: float | None
    ring_center_depth_m: float | None
    waist_ring_lateral_m: float | None
    waist_ring_depth_m: float | None
    waist_ring_aim_deg: float | None
    waist_ring_turn_direction: str
    depth_valid: bool
    depth_sample_count: int
    score: float


@dataclass(frozen=True)
class GoalInfo:
    """Selected goal geometry and provisional scoring conditions."""

    detected: bool
    state: str
    confidence: float
    aim_source: str
    aim_bbox: list[int] | None
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
    distance_m: float | None
    ground_distance_m: float | None
    lateral_offset_m: float | None
    left_depth_m: float | None
    center_depth_m: float | None
    right_depth_m: float | None
    backboard_angle_deg: float | None
    parallel_correction_deg: float | None
    backboard_angle_direction: str
    backboard_angle_valid: bool
    backboard_angle_fit_error_m: float | None
    backboard_measured_width_m: float | None
    backboard_line_width_m: float
    backboard_sample_points_px: list[list[int]]
    backboard_depth_sample_count: int
    waist_depth_m: float | None
    waist_aim_deg: float | None
    waist_turn_direction: str
    ring_center_lateral_m: float | None
    ring_center_depth_m: float | None
    waist_ring_lateral_m: float | None
    waist_ring_depth_m: float | None
    waist_ring_aim_deg: float | None
    waist_ring_turn_direction: str
    depth_valid: bool
    depth_sample_count: int
    is_centered: bool
    depth_in_score_range: bool
    score_depth_error_m: float | None
    score_now: bool
    approach_motion: str
    approach_level: int | None
    approach_target_distance_m: float | None
    target_priority_score: float
    candidate_count: int
    candidates: list[GoalCandidate]
    image_width: int | None
    image_height: int | None
    camera_info_ready: bool
    depth_age_sec: float | None
    note: str


class GoalAnalyzer(DepthFrameConsumer, Node):
    """Convert raw goal detections into an SDK-friendly scoring signal."""

    def __init__(self, depth_cache: DepthFrameCache | None = None) -> None:
        super().__init__("goal_analyzer")

        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter("output_topic", "/vision/goal_info")
        self.declare_parameter("goal_class_name", "goal")
        self.declare_parameter("backboard_class_name", "backboard")
        # Keep the parameter name; it now prefers goal-associated backboards.
        self.declare_parameter("prefer_backboard_center", True)
        self.declare_parameter("goal_robot_center_offset_px", 96.0)
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("depth_timeout_sec", 0.7)
        self.declare_parameter("depth_window_px", 11)
        self.declare_parameter("backboard_line_width_m", 0.155)
        self.declare_parameter("backboard_sample_y_ratio", 0.18)
        self.declare_parameter("backboard_edge_inset_ratio", 0.10)
        self.declare_parameter("backboard_parallel_tolerance_deg", 3.0)
        self.declare_parameter("camera_to_waist_depth_offset_m", 0.10)
        # Positive means the waist center is to image-right of the camera.
        self.declare_parameter("camera_to_waist_lateral_offset_m", 0.030)
        # Assumes the ring's rear edge is at the backboard plane, so its
        # 10 cm radius is also the board-plane-to-ring-center distance.
        self.declare_parameter("backboard_to_ring_center_forward_m", 0.10)
        self.declare_parameter("waist_aim_deadband_deg", 1.0)
        self.declare_parameter("max_valid_depth_m", 6.0)
        self.declare_parameter("detect_depth_m", 2.0)
        self.declare_parameter("approach_depth_m", 1.0)
        self.declare_parameter("score_target_depth_m", GOAL_SCORE_TARGET_DEPTH_M)
        self.declare_parameter("direction_deadband_norm", 0.04)
        self.declare_parameter("confirmation_window_size", 40)
        self.declare_parameter("confirmation_required_hits", 10)
        self.declare_parameter("confirmation_max_missed_frames", 2)
        self.declare_parameter("confirmation_max_center_shift_norm", 0.18)
        self.declare_parameter("confirmation_min_area_ratio", 0.40)
        self.declare_parameter("score_confirmation_window_size", 5)
        self.declare_parameter("score_confirmation_required_hits", 3)
        self.declare_parameter("publish_empty_when_missing", True)

        self.detections_topic = self._string_parameter("detections_topic")
        self.depth_topic = self._string_parameter("depth_topic")
        self.camera_info_topic = self._string_parameter(
            "camera_info_topic"
        )
        self.output_topic = self._string_parameter("output_topic")
        self.goal_class_name = self._string_parameter("goal_class_name")
        self.backboard_class_name = self._string_parameter(
            "backboard_class_name"
        )
        self.prefer_backboard_center = bool(
            self.get_parameter("prefer_backboard_center").value
        )
        self.robot_center_offset_px = self._float_parameter(
            "goal_robot_center_offset_px"
        )
        if not math.isfinite(self.robot_center_offset_px):
            raise ValueError("goal_robot_center_offset_px must be finite")
        self.min_confidence = self._float_parameter("min_confidence")
        self.depth_timeout_sec = self._float_parameter(
            "depth_timeout_sec"
        )
        self.depth_window_px = max(
            3,
            int(self.get_parameter("depth_window_px").value),
        )
        if self.depth_window_px % 2 == 0:
            self.depth_window_px += 1
        self.backboard_line_width_m = max(
            1e-6,
            self._float_parameter("backboard_line_width_m"),
        )
        self.backboard_sample_y_ratio = float(np.clip(
            self._float_parameter("backboard_sample_y_ratio"),
            0.0,
            1.0,
        ))
        self.backboard_edge_inset_ratio = float(np.clip(
            self._float_parameter("backboard_edge_inset_ratio"),
            0.0,
            0.45,
        ))
        self.backboard_parallel_tolerance_deg = max(
            0.0,
            self._float_parameter("backboard_parallel_tolerance_deg"),
        )
        self.camera_to_waist_depth_offset_m = self._float_parameter(
            "camera_to_waist_depth_offset_m"
        )
        if not math.isfinite(self.camera_to_waist_depth_offset_m):
            raise ValueError(
                "camera_to_waist_depth_offset_m must be finite"
            )
        self.camera_to_waist_lateral_offset_m = self._float_parameter(
            "camera_to_waist_lateral_offset_m"
        )
        if not math.isfinite(self.camera_to_waist_lateral_offset_m):
            raise ValueError(
                "camera_to_waist_lateral_offset_m must be finite"
            )
        self.backboard_to_ring_center_forward_m = self._float_parameter(
            "backboard_to_ring_center_forward_m"
        )
        if (
            not math.isfinite(self.backboard_to_ring_center_forward_m)
            or self.backboard_to_ring_center_forward_m < 0.0
        ):
            raise ValueError(
                "backboard_to_ring_center_forward_m must be finite and "
                "non-negative"
            )
        self.waist_aim_deadband_deg = max(
            0.0,
            self._float_parameter("waist_aim_deadband_deg"),
        )
        self.max_valid_depth_m = self._float_parameter(
            "max_valid_depth_m"
        )
        self.detect_depth_m = self._float_parameter("detect_depth_m")
        self.approach_depth_m = self._float_parameter("approach_depth_m")
        self.score_target_depth_m = self._float_parameter(
            "score_target_depth_m"
        )

        self.direction_deadband_norm = max(
            0.0,
            self._float_parameter("direction_deadband_norm"),
        )
        self.publish_empty_when_missing = bool(
            self.get_parameter("publish_empty_when_missing").value
        )

        self.confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter("confirmation_window_size").value
            ),
            required_hits=int(
                self.get_parameter("confirmation_required_hits").value
            ),
            max_missed_frames=int(
                self.get_parameter("confirmation_max_missed_frames").value
            ),
            max_center_shift_norm=float(
                self.get_parameter(
                    "confirmation_max_center_shift_norm"
                ).value
            ),
            min_area_ratio=float(
                self.get_parameter("confirmation_min_area_ratio").value
            ),
        )
        self.score_confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter("score_confirmation_window_size").value
            ),
            required_hits=int(
                self.get_parameter("score_confirmation_required_hits").value
            ),
            max_missed_frames=0,
            spatial_matching=False,
        )
        self.confirmation_fields: dict[str, object] = {}
        self.score_confirmation_fields: dict[str, object] = {}

        self.depth_cache = depth_cache or DepthFrameCache()
        self._owns_depth_subscription = depth_cache is None
        self.bridge = CvBridge() if self._owns_depth_subscription else None
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None

        self._detection_callback_group = MutuallyExclusiveCallbackGroup()
        self._depth_callback_group = MutuallyExclusiveCallbackGroup()
        self._camera_info_callback_group = MutuallyExclusiveCallbackGroup()
        latest_detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_depth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(
            String,
            self.detections_topic,
            self._detections_callback,
            latest_detection_qos,
            callback_group=self._detection_callback_group,
        )
        if self._owns_depth_subscription:
            self.create_subscription(
                Image,
                self.depth_topic,
                self._depth_callback,
                latest_depth_qos,
                callback_group=self._depth_callback_group,
            )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
            callback_group=self._camera_info_callback_group,
        )

        self.get_logger().info(
            f"Subscribing detections: {self.detections_topic}"
        )
        if self._owns_depth_subscription:
            self.get_logger().info(
                f"Subscribing aligned depth: {self.depth_topic}"
            )
        self.get_logger().info(f"Publishing goal info: {self.output_topic}")

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Store color-camera intrinsics for angle and 3-D projection."""
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
        """Store the latest aligned depth image."""
        try:
            if self.bridge is not None:
                self._store_depth_message(message, self.bridge)
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")

    def _depth_age_sec(self) -> float | None:
        if self.latest_depth_time is None:
            return None
        return time.monotonic() - self.latest_depth_time

    def _depth_is_fresh(self) -> bool:
        age = self._depth_age_sec()
        return age is not None and age <= self.depth_timeout_sec

    def _valid_patch_m(self, center_x: int, center_y: int) -> np.ndarray:
        """Return valid metric depth values around one image point."""
        if self.latest_depth_image is None:
            return np.empty(0, dtype=np.float32)
        depth = self.latest_depth_image
        height, width = depth.shape[:2]
        if not (0 <= center_x < width and 0 <= center_y < height):
            return np.empty(0, dtype=np.float32)
        radius = self.depth_window_px // 2
        patch = depth[
            max(0, center_y - radius):min(height, center_y + radius + 1),
            max(0, center_x - radius):min(width, center_x + radius + 1),
        ]
        if patch.dtype == np.uint16:
            patch_m = patch.astype(np.float32) * 0.001
        else:
            patch_m = patch.astype(np.float32)
        mask = (
            np.isfinite(patch_m)
            & (patch_m > 0.05)
            & (patch_m <= self.max_valid_depth_m)
        )
        return patch_m[mask]

    def _sample_goal_depth_m(
        self,
        bbox: list[int],
    ) -> tuple[float | None, bool, int]:
        """Sample several goal regions to avoid an empty goal center."""
        if (
            self.latest_depth_image is None
            or not self._depth_is_fresh()
            or self.latest_depth_image.ndim != 2
        ):
            return None, False, 0
        left, top, right, bottom = bbox
        width = right - left
        height = bottom - top
        ratios = [
            (0.50, 0.50),
            (0.25, 0.35),
            (0.75, 0.35),
            (0.25, 0.65),
            (0.75, 0.65),
        ]
        medians: list[float] = []
        for x_ratio, y_ratio in ratios:
            values = self._valid_patch_m(
                int(round(left + width * x_ratio)),
                int(round(top + height * y_ratio)),
            )
            if values.size:
                medians.append(float(np.median(values)))
        if not medians:
            return None, False, 0
        return float(np.median(medians)), True, len(medians)

    def _sample_backboard_line_depths(
        self,
        bbox: list[int],
        source_width: int | None,
        source_height: int | None,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        int,
        list[list[int]],
    ]:
        """Read left, center, and right depth on one backboard line."""
        if (
            self.latest_depth_image is None
            or not self._depth_is_fresh()
            or self.latest_depth_image.ndim != 2
        ):
            return None, None, None, 0, []

        depth_height, depth_width = self.latest_depth_image.shape[:2]
        source_width = int(source_width or depth_width)
        source_height = int(source_height or depth_height)
        if source_width <= 0 or source_height <= 0:
            return None, None, None, 0, []

        left, top, right, bottom = bbox
        width = right - left
        height = bottom - top
        if width <= 0 or height <= 0:
            return None, None, None, 0, []

        inset = self.backboard_edge_inset_ratio
        x_ratios = (inset, 0.50, 1.0 - inset)
        source_y = int(round(top + height * self.backboard_sample_y_ratio))
        source_points = [
            [int(round(left + width * ratio)), source_y]
            for ratio in x_ratios
        ]
        scale_x = depth_width / source_width
        scale_y = depth_height / source_height
        medians: list[float | None] = []
        for source_x, point_y in source_points:
            depth_x = int(np.clip(
                round(source_x * scale_x), 0, depth_width - 1,
            ))
            depth_y = int(np.clip(
                round(point_y * scale_y), 0, depth_height - 1,
            ))
            values = self._valid_patch_m(depth_x, depth_y)
            medians.append(
                float(np.median(values)) if values.size else None
            )
        return (
            medians[0],
            medians[1],
            medians[2],
            sum(value is not None for value in medians),
            source_points,
        )

    def _estimate_backboard_angle(
        self,
        sample_points: list[list[int]],
        depths_m: tuple[float | None, float | None, float | None],
    ) -> tuple[
        float | None,
        float | None,
        str,
        float | None,
        float | None,
    ]:
        """Fit three depths over the known line width and return yaw."""
        if (
            len(sample_points) != 3
            or any(depth is None for depth in depths_m)
        ):
            return None, None, "UNKNOWN", None, None

        depths = np.asarray(depths_m, dtype=np.float64)
        if not np.all(np.isfinite(depths)):
            return None, None, "UNKNOWN", None, None
        # The inset maps the YOLO backboard bbox to the visible red line ends;
        # the physical distance between the selected end points is still the
        # measured full red-line width.
        sampled_width = self.backboard_line_width_m
        line_positions = np.asarray(
            (-sampled_width / 2.0, 0.0, sampled_width / 2.0),
            dtype=np.float64,
        )
        denominator = float(np.dot(line_positions, line_positions))
        if denominator <= 1e-10:
            return None, None, "UNKNOWN", None, None

        centered_z = depths - float(np.mean(depths))
        depth_per_line_meter = float(
            np.dot(line_positions, centered_z) / denominator
        )
        # A point at coordinate s on a board yawed by theta has
        # Z(s) = Z(center) + s * sin(theta).
        if abs(depth_per_line_meter) > 1.0 + 1e-6:
            return None, None, "UNKNOWN", None, None
        angle = math.degrees(math.asin(float(np.clip(
            depth_per_line_meter,
            -1.0,
            1.0,
        ))))
        fitted_z = (
            float(np.mean(depths))
            + depth_per_line_meter * line_positions
        )
        fit_error = float(np.sqrt(np.mean(np.square(depths - fitted_z))))
        correction = -angle

        if abs(angle) <= self.backboard_parallel_tolerance_deg:
            direction = "PARALLEL"
        elif angle > 0.0:
            direction = "RIGHT_FAR"
        else:
            direction = "LEFT_FAR"

        measured_width: float | None = None
        if self.fx is not None and self.cx is not None and self.fx > 0.0:
            image_x = np.asarray(
                [point[0] for point in sample_points],
                dtype=np.float64,
            )
            camera_x = (image_x - self.cx) * depths / self.fx
            endpoint_span = math.hypot(
                float(camera_x[2] - camera_x[0]),
                float(depths[2] - depths[0]),
            )
            measured_width = endpoint_span
        return angle, correction, direction, fit_error, measured_width

    def _waist_center_aim(
        self,
        depth_m: float | None,
        lateral_m: float | None,
    ) -> tuple[float | None, float | None, str]:
        """Return waist-pivot depth and yaw needed to face board center.

        ``lateral_m`` is measured from the camera optical center.  A positive
        camera-to-waist lateral offset means the waist lies image-right of
        the camera, so it is subtracted to express the target from the waist.
        """
        if depth_m is None or lateral_m is None:
            return None, None, "UNKNOWN"
        waist_depth = depth_m + self.camera_to_waist_depth_offset_m
        if (
            not math.isfinite(waist_depth)
            or not math.isfinite(lateral_m)
            or waist_depth <= 0.0
        ):
            return None, None, "UNKNOWN"
        waist_lateral = (
            lateral_m - self.camera_to_waist_lateral_offset_m
        )
        aim = math.degrees(math.atan2(waist_lateral, waist_depth))
        if abs(aim) <= self.waist_aim_deadband_deg:
            direction = "CENTER"
        elif aim > 0.0:
            direction = "TURN_RIGHT"
        else:
            direction = "TURN_LEFT"
        return waist_depth, aim, direction

    def _waist_ring_aim(
        self,
        board_depth_m: float | None,
        board_lateral_m: float | None,
        board_angle_deg: float | None,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        str,
    ]:
        """Return ring geometry and a motor-signed waist yaw delta.

        Only horizontal-plane geometry is used; the ring's vertical drop
        below the sampled backboard line does not affect a waist yaw joint.
        Positive board yaw means the board's right side is farther away.
        The returned aim uses the waist motor convention, which is the
        opposite sign of the geometric yaw: motor_delta = -geometric_yaw.
        """
        if (
            board_depth_m is None
            or board_lateral_m is None
            or board_angle_deg is None
        ):
            return None, None, None, None, None, "UNKNOWN"
        values = (board_depth_m, board_lateral_m, board_angle_deg)
        if not all(math.isfinite(value) for value in values):
            return None, None, None, None, None, "UNKNOWN"

        angle_rad = math.radians(board_angle_deg)
        forward = self.backboard_to_ring_center_forward_m
        # Toward-camera board normal: (sin(yaw), -cos(yaw)) in camera X/Z.
        ring_lateral = board_lateral_m + forward * math.sin(angle_rad)
        ring_depth = board_depth_m - forward * math.cos(angle_rad)
        waist_ring_lateral = (
            ring_lateral - self.camera_to_waist_lateral_offset_m
        )
        waist_ring_depth = (
            ring_depth + self.camera_to_waist_depth_offset_m
        )
        if ring_depth <= 0.0 or waist_ring_depth <= 0.0:
            return None, None, None, None, None, "UNKNOWN"

        geometric_aim = math.degrees(math.atan2(
            waist_ring_lateral,
            waist_ring_depth,
        ))
        motor_delta = -geometric_aim
        if abs(geometric_aim) <= self.waist_aim_deadband_deg:
            direction = "CENTER"
        elif geometric_aim > 0.0:
            direction = "TURN_RIGHT"
        else:
            direction = "TURN_LEFT"
        return (
            ring_lateral,
            ring_depth,
            waist_ring_lateral,
            waist_ring_depth,
            motor_delta,
            direction,
        )

    def _image_size(
        self,
        payload: dict[str, Any],
    ) -> tuple[int | None, int | None]:
        try:
            width = int(payload.get("image_width", 0))
            height = int(payload.get("image_height", 0))
        except (TypeError, ValueError):
            width = 0
            height = 0
        if width > 0 and height > 0:
            return width, height
        return self.latest_image_width, self.latest_image_height

    def _project(
        self,
        center_x: int,
        center_y: int,
        depth_m: float | None,
        reference_x: float | None = None,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
    ]:
        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return None, None, None, None, None
        # Goal yaw control must use the same calibrated axis as pixel alignment.
        origin_x = self.cx if reference_x is None else reference_x
        x_ratio = (center_x - origin_x) / self.fx
        y_ratio = (center_y - self.cy) / self.fy
        bearing = math.degrees(math.atan(x_ratio))
        elevation = math.degrees(math.atan(y_ratio))
        if depth_m is None:
            return bearing, elevation, None, None, None
        lateral = x_ratio * depth_m
        # GOAL already controls from raw Depth Z.  Keep legacy distance output
        # as a depth alias and do not calculate slant or floor distance.
        return bearing, elevation, lateral, depth_m, None

    def _build_candidate(
        self,
        detection: dict[str, Any],
        image_width: int | None,
        image_height: int | None,
    ) -> GoalCandidate | None:
        """Build geometry and confidence from one backboard detection."""
        try:
            confidence = float(detection.get("confidence", 0.0))
            bbox = [int(value) for value in detection.get("bbox", [])]
            center = [
                int(value) for value in detection.get("center", [])
            ]
        except (TypeError, ValueError, OverflowError):
            return None
        if (
            not math.isfinite(confidence)
            or not self.min_confidence <= confidence <= 1.0
            or len(bbox) != 4
            or len(center) != 2
        ):
            return None
        left, top, right, bottom = bbox
        width_px = right - left
        height_px = bottom - top
        if width_px <= 0 or height_px <= 0:
            return None

        aim_source = "backboard"
        aim_bbox = bbox

        center_x, center_y = center
        offset_x_px = 0
        offset_y_px = 0
        offset_x_norm = 0.0
        offset_y_norm = 0.0
        robot_center_x = None
        if image_width and image_height:
            robot_center_x = calibrated_robot_center_x(
                image_width, self.robot_center_offset_px
            )
            offset_x_px = int(round(center_x - robot_center_x))
            offset_y_px = int(center_y - image_height / 2)
            offset_x_norm = offset_x_px / max(image_width / 2, 1.0)
            offset_y_norm = offset_y_px / max(image_height / 2, 1.0)

        depth_m, depth_valid, sample_count = self._sample_goal_depth_m(
            aim_bbox
        )
        (
            left_depth,
            center_depth,
            right_depth,
            line_sample_count,
            sample_points,
        ) = self._sample_backboard_line_depths(
            aim_bbox,
            image_width,
            image_height,
        )
        (
            backboard_angle,
            parallel_correction,
            angle_direction,
            angle_fit_error,
            measured_width,
        ) = self._estimate_backboard_angle(
            sample_points,
            (left_depth, center_depth, right_depth),
        )
        bearing, elevation, lateral, distance, ground_distance = self._project(
            center_x,
            center_y,
            depth_m,
            reference_x=robot_center_x,
        )
        _, _, camera_lateral, _, _ = self._project(
            center_x,
            center_y,
            depth_m,
        )
        waist_depth, waist_aim, waist_direction = self._waist_center_aim(
            depth_m,
            camera_lateral,
        )
        ring_board_depth = (
            center_depth if center_depth is not None else depth_m
        )
        _, _, ring_board_lateral, _, _ = self._project(
            center_x,
            center_y,
            ring_board_depth,
        )
        (
            ring_lateral,
            ring_depth,
            waist_ring_lateral,
            waist_ring_depth,
            waist_ring_aim,
            waist_ring_direction,
        ) = self._waist_ring_aim(
            ring_board_depth,
            ring_board_lateral,
            backboard_angle,
        )
        if offset_x_norm < -self.direction_deadband_norm:
            direction = "LEFT"
        elif offset_x_norm > self.direction_deadband_norm:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        area_px = width_px * height_px
        center_score = max(0.0, 1.0 - abs(offset_x_norm))
        depth_score = (
            max(0.0, 1.0 - depth_m / self.max_valid_depth_m)
            if depth_m is not None
            else 0.0
        )
        area_score = min(1.0, math.sqrt(area_px) / 300.0)
        priority = (
            confidence * 0.50
            + center_score * 0.25
            + depth_score * 0.15
            + area_score * 0.10
        )
        return GoalCandidate(
            confidence=round(confidence, 4),
            bbox=bbox,
            aim_source=aim_source,
            aim_bbox=aim_bbox,
            center=center,
            width_px=width_px,
            height_px=height_px,
            area_px=area_px,
            offset_x_px=offset_x_px,
            offset_y_px=offset_y_px,
            offset_x_norm=round(offset_x_norm, 4),
            offset_y_norm=round(offset_y_norm, 4),
            horizontal_direction=direction,
            bearing_deg=round(bearing, 3) if bearing is not None else None,
            elevation_deg=(
                round(elevation, 3) if elevation is not None else None
            ),
            depth_m=round(depth_m, 3) if depth_m is not None else None,
            distance_m=(
                round(distance, 3) if distance is not None else None
            ),
            ground_distance_m=(
                round(ground_distance, 3)
                if ground_distance is not None
                else None
            ),
            lateral_offset_m=(
                round(lateral, 3) if lateral is not None else None
            ),
            left_depth_m=(
                round(left_depth, 4) if left_depth is not None else None
            ),
            center_depth_m=(
                round(center_depth, 4) if center_depth is not None else None
            ),
            right_depth_m=(
                round(right_depth, 4) if right_depth is not None else None
            ),
            backboard_angle_deg=(
                round(backboard_angle, 3)
                if backboard_angle is not None else None
            ),
            parallel_correction_deg=(
                round(parallel_correction, 3)
                if parallel_correction is not None else None
            ),
            backboard_angle_direction=angle_direction,
            backboard_angle_valid=backboard_angle is not None,
            backboard_angle_fit_error_m=(
                round(angle_fit_error, 4)
                if angle_fit_error is not None else None
            ),
            backboard_measured_width_m=(
                round(measured_width, 4)
                if measured_width is not None else None
            ),
            backboard_line_width_m=round(self.backboard_line_width_m, 4),
            backboard_sample_points_px=sample_points,
            backboard_depth_sample_count=line_sample_count,
            waist_depth_m=(
                round(waist_depth, 4) if waist_depth is not None else None
            ),
            waist_aim_deg=(
                round(waist_aim, 3) if waist_aim is not None else None
            ),
            waist_turn_direction=waist_direction,
            ring_center_lateral_m=(
                round(ring_lateral, 4) if ring_lateral is not None else None
            ),
            ring_center_depth_m=(
                round(ring_depth, 4) if ring_depth is not None else None
            ),
            waist_ring_lateral_m=(
                round(waist_ring_lateral, 4)
                if waist_ring_lateral is not None else None
            ),
            waist_ring_depth_m=(
                round(waist_ring_depth, 4)
                if waist_ring_depth is not None else None
            ),
            waist_ring_aim_deg=(
                round(waist_ring_aim, 3)
                if waist_ring_aim is not None else None
            ),
            waist_ring_turn_direction=waist_ring_direction,
            depth_valid=depth_valid,
            depth_sample_count=sample_count,
            score=round(priority, 4),
        )

    def _goal_state(
        self,
        target: GoalCandidate,
    ) -> tuple[str, bool, bool, float | None, bool, str]:
        centered = goal_shot_pixels_aligned(
            target.ring_center_depth_m if target.ring_center_depth_m is not None else GOAL_SCORE_TARGET_DEPTH_M,
            target.offset_x_px,
        )
        if not target.depth_valid or target.ring_center_depth_m is None:
            return (
                "NO_DEPTH",
                centered,
                False,
                None,
                False,
                "goal_detected_without_valid_depth",
            )
        error = target.ring_center_depth_m - self.score_target_depth_m
        depth_in_range = (
            GOAL_SCORE_MIN_DEPTH_M <= target.ring_center_depth_m < GOAL_SCORE_MAX_DEPTH_M
        )
        score_now = (goal_shot_region(target.ring_center_depth_m) is not None
                     and target.waist_ring_aim_deg is not None
                     and math.isfinite(target.waist_ring_aim_deg)
                     and abs(target.waist_ring_aim_deg) < 90.0)
        if score_now:
            return (
                "SCORE_READY",
                centered,
                depth_in_range,
                error,
                True,
                "goal_ring_at_scoring_depth",
            )
        if not centered:
            return (
                "ALIGN",
                centered,
                depth_in_range,
                error,
                False,
                "align_goal_horizontally",
            )
        if target.ring_center_depth_m >= GOAL_SCORE_MAX_DEPTH_M:
            state = (
                "APPROACH"
                if target.ring_center_depth_m <= self.approach_depth_m
                else "FAR"
            )
            return state, centered, False, error, False, "goal_too_far"
        return (
            "TOO_CLOSE",
            centered,
            False,
            error,
            False,
            "goal_too_close",
        )

    def _publish(self, info: GoalInfo) -> None:
        message = String()
        payload = asdict(info)
        payload["camera_fx_px"] = self.fx
        payload["rgb_stamp_ns"] = getattr(self, "_active_rgb_stamp_ns", None)
        payload["robot_center_offset_px"] = self.robot_center_offset_px
        payload["camera_to_waist_depth_offset_m"] = (
            self.camera_to_waist_depth_offset_m
        )
        payload["camera_to_waist_lateral_offset_m"] = (
            self.camera_to_waist_lateral_offset_m
        )
        payload["backboard_to_ring_center_forward_m"] = (
            self.backboard_to_ring_center_forward_m
        )
        payload["robot_center_x_px"] = (
            calibrated_robot_center_x(info.image_width, self.robot_center_offset_px)
            if info.image_width is not None and info.image_width > 0
            else None
        )
        payload["camera_cx_px"] = self.cx
        payload.update(self.confirmation_fields)
        payload.update(self.score_confirmation_fields)
        message.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)

    def _empty_info(self, note: str = "no_goal_detection") -> GoalInfo:
        age = self._depth_age_sec()
        return GoalInfo(
            detected=False,
            state="SEARCH",
            confidence=0.0,
            aim_source="none",
            aim_bbox=None,
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
            distance_m=None,
            ground_distance_m=None,
            lateral_offset_m=None,
            left_depth_m=None,
            center_depth_m=None,
            right_depth_m=None,
            backboard_angle_deg=None,
            parallel_correction_deg=None,
            backboard_angle_direction="UNKNOWN",
            backboard_angle_valid=False,
            backboard_angle_fit_error_m=None,
            backboard_measured_width_m=None,
            backboard_line_width_m=round(self.backboard_line_width_m, 4),
            backboard_sample_points_px=[],
            backboard_depth_sample_count=0,
            waist_depth_m=None,
            waist_aim_deg=None,
            waist_turn_direction="UNKNOWN",
            ring_center_lateral_m=None,
            ring_center_depth_m=None,
            waist_ring_lateral_m=None,
            waist_ring_depth_m=None,
            waist_ring_aim_deg=None,
            waist_ring_turn_direction="UNKNOWN",
            depth_valid=False,
            depth_sample_count=0,
            is_centered=False,
            depth_in_score_range=False,
            score_depth_error_m=None,
            score_now=False,
            approach_motion="STRAIGHT",
            approach_level=None,
            approach_target_distance_m=None,
            target_priority_score=0.0,
            candidate_count=0,
            candidates=[],
            image_width=self.latest_image_width,
            image_height=self.latest_image_height,
            camera_info_ready=self.fx is not None,
            depth_age_sec=round(age, 3) if age is not None else None,
            note=note,
        )

    def _has_matching_goal(
        self,
        candidate: GoalCandidate,
        goals: list[dict[str, Any]],
    ) -> bool:
        """Prefer a backboard whose center is inside a confident goal box."""
        for goal in goals:
            try:
                confidence = float(goal.get("confidence", 0.0))
                bbox = [int(value) for value in goal.get("bbox", [])]
            except (TypeError, ValueError, OverflowError):
                continue
            if (
                not math.isfinite(confidence)
                or not self.min_confidence <= confidence <= 1.0
                or len(bbox) != 4
            ):
                continue
            left, top, right, bottom = bbox
            if (
                left < right and top < bottom
                and left <= candidate.center[0] <= right
                and top <= candidate.center[1] <= bottom
            ):
                return True
        return False

    def _detections_callback(self, message: String) -> None:
        """Select and publish the strongest valid goal target."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("detections JSON must be an object")
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections JSON must contain a list")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid detections message: {exc}")
            return

        stamp = payload.get("stamp")
        sec = stamp.get("sec") if isinstance(stamp, dict) else None
        nanosec = stamp.get("nanosec") if isinstance(stamp, dict) else None
        self._active_rgb_stamp_ns = (
            sec * 1_000_000_000 + nanosec
            if (isinstance(sec, int) and not isinstance(sec, bool) and sec >= 0
                and isinstance(nanosec, int) and not isinstance(nanosec, bool)
                and 0 <= nanosec < 1_000_000_000) else None
        )
        image_width, image_height = self._image_size(payload)
        backboards = [
            detection
            for detection in detections
            if isinstance(detection, dict)
            and str(detection.get("class_name", ""))
            == self.backboard_class_name
        ]
        goals = [
            detection
            for detection in detections
            if isinstance(detection, dict)
            and str(detection.get("class_name", "")) == self.goal_class_name
        ]
        ranked_candidates: list[tuple[bool, GoalCandidate]] = []
        outside_tracking_range = False
        for detection in backboards:
            candidate = self._build_candidate(
                detection,
                image_width,
                image_height,
            )
            if candidate is None:
                continue
            if depth_is_within_range(
                candidate.depth_valid,
                candidate.depth_m,
                self.detect_depth_m,
            ):
                goal_associated = (
                    self.prefer_backboard_center
                    and self._has_matching_goal(candidate, goals)
                )
                ranked_candidates.append((goal_associated, candidate))
            elif candidate.depth_valid and candidate.depth_m is not None:
                outside_tracking_range = True
        ranked_candidates.sort(
            key=lambda item: (item[0], item[1].score), reverse=True
        )
        candidates = [candidate for _, candidate in ranked_candidates]
        if not candidates:
            confirmation = self.confirmation_filter.update(False)
            score_confirmation = self.score_confirmation_filter.update(False)
            self.confirmation_fields = confirmation.as_dict()
            self.score_confirmation_fields = score_confirmation.as_dict(
                "score_confirmation"
            )
            if self.publish_empty_when_missing:
                note = (
                    "goal_outside_tracking_range"
                    if outside_tracking_range
                    else "no_goal_detection"
                )
                self._publish(self._empty_info(note))
            return

        target = candidates[0]
        confirmation = self.confirmation_filter.update(
            True,
            bbox=target.aim_bbox,
            image_width=image_width,
            image_height=image_height,
        )
        self.confirmation_fields = confirmation.as_dict()
        if not confirmation.confirmed:
            score_confirmation = self.score_confirmation_filter.update(False)
            self.score_confirmation_fields = score_confirmation.as_dict(
                "score_confirmation"
            )
            if self.publish_empty_when_missing:
                self._publish(self._empty_info("goal_confirmation_pending"))
            return

        (
            state,
            centered,
            depth_in_range,
            depth_error,
            score_now,
            note,
        ) = self._goal_state(target)
        raw_score_now = score_now
        score_confirmation = self.score_confirmation_filter.update(
            raw_score_now
        )
        self.score_confirmation_fields = score_confirmation.as_dict(
            "score_confirmation"
        )
        score_now = raw_score_now and score_confirmation.confirmed
        if raw_score_now and not score_now:
            note = "score_confirmation_pending"
        age = self._depth_age_sec()
        self._publish(
            GoalInfo(
                detected=True,
                state=state,
                confidence=target.confidence,
                aim_source=target.aim_source,
                aim_bbox=target.aim_bbox,
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
                distance_m=target.distance_m,
                ground_distance_m=target.ground_distance_m,
                lateral_offset_m=target.lateral_offset_m,
                left_depth_m=target.left_depth_m,
                center_depth_m=target.center_depth_m,
                right_depth_m=target.right_depth_m,
                backboard_angle_deg=target.backboard_angle_deg,
                parallel_correction_deg=target.parallel_correction_deg,
                backboard_angle_direction=target.backboard_angle_direction,
                backboard_angle_valid=target.backboard_angle_valid,
                backboard_angle_fit_error_m=(
                    target.backboard_angle_fit_error_m
                ),
                backboard_measured_width_m=(
                    target.backboard_measured_width_m
                ),
                backboard_line_width_m=target.backboard_line_width_m,
                backboard_sample_points_px=(
                    target.backboard_sample_points_px
                ),
                backboard_depth_sample_count=(
                    target.backboard_depth_sample_count
                ),
                waist_depth_m=target.waist_depth_m,
                waist_aim_deg=target.waist_aim_deg,
                waist_turn_direction=target.waist_turn_direction,
                ring_center_lateral_m=target.ring_center_lateral_m,
                ring_center_depth_m=target.ring_center_depth_m,
                waist_ring_lateral_m=target.waist_ring_lateral_m,
                waist_ring_depth_m=target.waist_ring_depth_m,
                waist_ring_aim_deg=target.waist_ring_aim_deg,
                waist_ring_turn_direction=(
                    target.waist_ring_turn_direction
                ),
                depth_valid=target.depth_valid,
                depth_sample_count=target.depth_sample_count,
                is_centered=centered,
                depth_in_score_range=depth_in_range,
                score_depth_error_m=(
                    round(depth_error, 3)
                    if depth_error is not None
                    else None
                ),
                score_now=score_now,
                approach_motion="WAIT",
                approach_level=None,
                approach_target_distance_m=target.depth_m,
                target_priority_score=target.score,
                candidate_count=len(candidates),
                candidates=candidates[:5],
                image_width=image_width,
                image_height=image_height,
                camera_info_ready=self.fx is not None,
                depth_age_sec=round(age, 3) if age is not None else None,
                note=note,
            )
        )


def main(args: list[str] | None = None) -> None:
    """Run the ROS 2 goal analyzer node."""
    rclpy.init(args=args)
    node: GoalAnalyzer | None = None
    try:
        node = GoalAnalyzer()
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
