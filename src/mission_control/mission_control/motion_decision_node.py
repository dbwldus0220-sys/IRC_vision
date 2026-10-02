#!/usr/bin/env python3
"""ROS 2 node selecting one command from line, ball, goal, and hurdle."""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import json
import math
import os
import statistics
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from step.approach_distance import APPROACH_DISTANCE_LIMITS_M
from step.approach_distance import BALL_HURDLE_FINE_DISTANCE_M
from step.line_navigation_planner import line_heading, straight_heading_is_aligned
from .sparse_line_recovery import SparseLineRecovery
from step.hurdle_navigation_planner import HurdleNavigationPlanner

from .ball_approach_observation import BallApproachObservation
from .ball_loss_confirmation import BallLossConfirmation
from .executor_heartbeat_watchdog import ExecutorHeartbeatWatchdog
from .executor_heartbeat_watchdog import HeartbeatStartupDelay
from .lost_search_turn_limiter import LostSearchTurnLimiter
from .mission_phase_manager import MissionPhaseManager
from .motion_command_gate import GeneralMotionCommandGate
from .motion_command_gate import normalize_general_action
from .motion_decision_planner import MotionDecision
from .motion_decision_planner import MotionDecisionConfig
from .motion_decision_planner import MotionDecisionPlanner
from .safety_interlock import SafetyInterlock


class MotionDecisionNode(Node):
    """Replace four navigation controllers with one command publisher."""

    SOURCES = ("line", "ball", "goal", "hurdle", "finish")
    SHOT_PRE_MOTION_SETTLE_SEC = 1.0
    LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.0
    # Match the line analyzer's default corner_turn_margin_m.
    LINE_CORNER_TURN_DISTANCE_M = 0.15
    FINE_FORWARD_PRE_MOTION_SETTLE_SEC = 1.0

    PRE_MOTION_SETTLE_ACTIONS = frozenset(
        {
            "LEFT",
            "RIGHT",
            "PICKUP_NOW",
            "GO",
            "SHOT",
        }
    )

    SPECIAL_ACTIONS = {
        "PICKUP_NOW",
        "POST_BALL_GOAL_TRANSITION",
        "SHOT",
        "GO",
        "CROSS_FINISH",
    }

    SPECIAL_ACTION_SOURCES = {
        "PICKUP_NOW": "ball",
        "SHOT": "goal",
        "GO": "hurdle",
        "CROSS_FINISH": "finish",
    }

    SPECIAL_FAILURE_REASONS = {
        "PICKUP_NOW": "pickup_failures_exhausted",
        "SHOT": "shot_failures_exhausted",
        "GO": "go_failures_exhausted",
    }

    DEFAULT_EXECUTOR_HEARTBEAT_TIMEOUT_SEC = 2.0
    DEFAULT_EXECUTOR_HEARTBEAT_STARTUP_GRACE_SEC = 5.0
    EXECUTOR_WATCHDOG_PERIOD_SEC = 0.1
    LINE_MOTION_CAPTURE_CONFIG = {
        "STRAIGHT": (2.467, 10),
        "STRAIGHT_1": (0.822, 5),
        "STRAIGHT_2": (1.644, 10),
        "STRAIGHT_3": (2.467, 10),
        "STRAIGHT_4": (3.289, 15),
        "LEFT": (7.365, 30),
        "RIGHT": (7.105, 30),
        **{
            f"RECOVER_{line_side}_TURN_LEFT_{suffix}": timing
            for line_side in ("LEFT", "RIGHT")
            for suffix, timing in {
                2: (2.272, 10),
                4: (3.198, 15),
                6: (4.124, 20),
                8: (5.050, 25),
                10: (5.976, 30),
                13: (7.365, 30),
            }.items()
        },
        **{
            f"RECOVER_{line_side}_TURN_RIGHT_{suffix}": timing
            for line_side in ("LEFT", "RIGHT")
            for suffix, timing in {
                4: (1.895, 5),
                6: (2.842, 10),
                8: (3.789, 15),
                10: (4.737, 20),
                12: (5.684, 25),
                15: (7.105, 30),
            }.items()
        },
    }
    LINE_MOTION_CAPTURE_START_RATIO = 0.70
    LINE_TIMEOUT_RECOVERY_FRAMES = 10
    BALL_POST_MOTION_DWELL_SEC = 1.0  # Approach turns only.
    HURDLE_POST_MOTION_DWELL_SEC = 1.0
    GOAL_POST_MOTION_DWELL_SEC = 1.0  # Stationary turns only.
    POST_BALL_LINE_DWELL_SEC = 1.0
    POST_BALL_LINE_RUN_SEC = 10.0
    POST_SHOT_DWELL_SEC = 1.0
    CORRECTION_POST_MOTION_DWELL_SEC = 1.0
    BALL_RAW_CONFIRMATION_RELEASE_SEC = 0.5
    BALL_NAVIGATION_BLOCKED_PHASES = frozenset({
        "POST_BALL_LINE_ALIGN", "LINE_TRACK_AFTER_PICKUP", "POST_BALL_GOAL_TRANSITION",
        "GOAL_SEARCH", "GOAL_APPROACH",
        "POST_SHOT_TURN", "POST_SHOT_LINE_ALIGN", "POST_SHOT_FORWARD",
    })
    PICKUP_DWELL_MARKER = "__NON_BLOCKING_DWELL__"
    PICKUP_GRASP_CHECK_MOTION_ID = "pickup_grasp_check_pose"
    GRASP_CONFIDENCE_THRESHOLD = 0.25
    GRASP_VERIFICATION_FRAME_WINDOW = 40
    GRASP_VERIFICATION_MIN_SUCCESSES = 15
    GRASP_RESULT_DISPLAY_SEC = 3.0
    PICKUP_INITIAL_ALIGN_MARKER = "__BALL_PICKUP_INITIAL_ALIGN_CHECK__"
    PICKUP_FINE_ALIGN_MARKER = "__BALL_PICKUP_FINE_ALIGN_CHECK__"
    PICKUP_POST_BACKWARD_ALIGN_MARKER = (
        "__BALL_PICKUP_POST_BACKWARD_ALIGN_CHECK__"
    )
    PICKUP_POSITIONING_LOSS_LATCH_ACTION = (
        "BALL_PICKUP_POSITIONING_LOSS_LATCH"
    )
    PICKUP_FIXED_SEQUENCE_FIRST_MOTION = "pickup_pre_backward_camera_down"
    PICKUP_INITIAL_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_INITIAL_ALIGN_CONTINUE",
            "BALL_PICKUP_INITIAL_SEARCH_FORWARD",
            "BALL_PICKUP_INITIAL_SEARCH_FORWARD_4",
            "BALL_PICKUP_INITIAL_SEARCH_BACKWARD",
            "BALL_PICKUP_INITIAL_CRAB_LEFT",
            "BALL_PICKUP_INITIAL_CRAB_RIGHT",
            *{
                f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{count}"
                for direction in ("LEFT", "RIGHT")
                for count in (
                    range(1, 7) if direction == "LEFT"
                    else (2, 3, 5, 7, 9)
                )
            },
        }
    )
    PICKUP_FINE_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_FINE_ALIGN_CONTINUE",
            "BALL_PICKUP_FINE_FORWARD",
            "BALL_PICKUP_FINE_SEARCH_LEFT",
            "BALL_PICKUP_FINE_SEARCH_RIGHT",
            *{
                f"BALL_PICKUP_FINE_SEARCH_{direction}_{count}"
                for direction, counts in (("LEFT", (1, 3)), ("RIGHT", (2, 5)))
                for count in counts
            },
            "BALL_PICKUP_FINE_SEARCH_FORWARD",
            "BALL_PICKUP_FINE_SEARCH_FORWARD_4",
            "BALL_PICKUP_FINE_SEARCH_BACKWARD",
            "BALL_PICKUP_CRAB_RIGHT",
            "BALL_PICKUP_CRAB_LEFT",
        }
    )
    PICKUP_POST_BACKWARD_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE",
            "BALL_PICKUP_POST_BACKWARD_CRAB_RIGHT",
            "BALL_PICKUP_POST_BACKWARD_CRAB_LEFT",
            *{
                f"BALL_PICKUP_POST_BACKWARD_TURN_{direction}_{count}"
                for direction in ("LEFT", "RIGHT")
                for count in (
                    range(1, 7) if direction == "LEFT"
                    else (2, 3, 5, 7, 9)
                )
            },
        }
    )

    def __init__(self) -> None:
        """Initialize mission decision state, topics, and timers."""
        super().__init__("motion_decision_node")

        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("ball_info_topic", "/vision/ball_info")
        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("hurdle_info_topic", "/vision/hurdle_info")
        self.declare_parameter("finish_info_topic", "/vision/finish_info")
        self.declare_parameter(
            "grasp_detections_topic", "/vision/detections"
        )

        self.declare_parameter("mission_phase_topic", "/mission/phase")
        self.declare_parameter(
            "command_topic",
            "/navigation/motion_command",
        )
        self.declare_parameter(
            "decision_debug_topic",
            "/navigation/decision_debug",
        )
        self.declare_parameter(
            "motion_status_topic",
            "/motion/status",
        )
        self.declare_parameter(
            "executor_heartbeat_topic",
            "/motion/executor/heartbeat",
        )
        self.declare_parameter(
            "executor_heartbeat_timeout_sec",
            self.DEFAULT_EXECUTOR_HEARTBEAT_TIMEOUT_SEC,
        )
        self.declare_parameter(
            "executor_heartbeat_startup_grace_sec",
            self.DEFAULT_EXECUTOR_HEARTBEAT_STARTUP_GRACE_SEC,
        )

        self.declare_parameter("wait_for_enter", False)
        self.decision_started = not bool(self.get_parameter("wait_for_enter").value)
        self.start_terminal_fd: int | None = None

        self.declare_parameter("initial_mission_phase", "AUTO")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pre_motion_settle_sec", 0.0)
        self.declare_parameter(
            "line_corner_turn_distance_m", self.LINE_CORNER_TURN_DISTANCE_M,
        )
        self.declare_parameter("required_pickups", 2)
        self.declare_parameter("required_shots", 2)
        self.declare_parameter("required_hurdles", 2)
        self.declare_parameter("required_ball_sections", 2)
        self.declare_parameter("finish_min_confidence", 0.70)
        self.declare_parameter("general_motion_transient_retry_limit", 2)

        self.declare_parameter("line_heading_source", "ground")
        self.declare_parameter("line_offset_align_enter_px", -1.0)
        self.declare_parameter("line_timeout_sec", 0.50)
        self.declare_parameter("ball_timeout_sec", 0.50)
        self.declare_parameter("goal_timeout_sec", 0.50)
        self.declare_parameter("hurdle_timeout_sec", 0.50)
        self.declare_parameter("finish_timeout_sec", 0.50)

        self.declare_parameter("lost_search_max_turns", 3)
        self.declare_parameter("lost_search_max_angle_deg", 90.0)
        self.lost_search_turn_limiter = LostSearchTurnLimiter(
            staged_sources=("line", "ball", "goal"),
            max_turns=int(self.get_parameter("lost_search_max_turns").value),
            max_angle_deg=float(self.get_parameter("lost_search_max_angle_deg").value),
        )
        self.declare_parameter("enable_ball_lost_recovery", True)
        self.declare_parameter("recovery_heading_turn_deg", 10.0)
        self.declare_parameter("recovery_away_heading_turn_deg", 3.0)
        self.declare_parameter("curve_follow_max_offset_norm", 0.55)
        self.declare_parameter("ball_tracking_range_m", 1.5)
        self.declare_parameter("ball_control_range_m", 1.5)
        self.declare_parameter(
            "pickup_fine_step_distance_m",
            0.550,
        )
        self.declare_parameter(
            "pickup_fine_align_bottom_distance_px",
            140,
        )
        self.declare_parameter("ball_lost_stop_sec", 0.35)
        self.declare_parameter("ball_recovery_timeout_sec", 8.0)
        self.declare_parameter("ball_recovery_turn_rad_s", 0.22)
        self.declare_parameter("ball_recovery_command_sec", 0.40)
        self.declare_parameter("ball_reacquire_center_deg", 5.0)
        self.declare_parameter("ball_reacquire_center_norm", 0.08)

        self.declare_parameter("goal_tracking_range_m", 2.0)
        self.declare_parameter("goal_control_range_m", 2.0)
        self.declare_parameter("hurdle_control_range_m", 1.0)
        self.declare_parameter("hurdle_path_reference_hold_sec", 0.50)
        self.declare_parameter("goal_lost_stop_sec", 0.35)
        self.declare_parameter("goal_recovery_timeout_sec", 8.0)
        self.declare_parameter("goal_recovery_turn_rad_s", 0.22)
        self.declare_parameter("goal_recovery_command_sec", 0.40)
        self.declare_parameter("goal_reacquire_center_deg", 5.0)
        self.declare_parameter("goal_reacquire_center_norm", 0.10)

        self.planner = MotionDecisionPlanner(
            MotionDecisionConfig(
                line_heading_source=str(self.get_parameter("line_heading_source").value),
                line_offset_align_enter_px=self._float_parameter("line_offset_align_enter_px"),
                enable_ball_lost_recovery=bool(
                    self.get_parameter(
                        "enable_ball_lost_recovery"
                    ).value
                ),
                recovery_heading_turn_deg=self._float_parameter(
                    "recovery_heading_turn_deg"
                ),
                recovery_away_heading_turn_deg=self._float_parameter(
                    "recovery_away_heading_turn_deg"
                ),
                curve_follow_max_offset_norm=self._float_parameter(
                    "curve_follow_max_offset_norm"
                ),
                ball_tracking_range_m=self._float_parameter(
                    "ball_tracking_range_m"
                ),
                ball_control_range_m=self._float_parameter(
                    "ball_control_range_m"
                ),
                pickup_fine_step_distance_m=max(
                    0.0,
                    self._float_parameter("pickup_fine_step_distance_m"),
                ),
                pickup_fine_align_bottom_distance_px=max(
                    0,
                    int(
                        self.get_parameter(
                            "pickup_fine_align_bottom_distance_px"
                        ).value
                    ),
                ),
                ball_lost_stop_sec=self._float_parameter(
                    "ball_lost_stop_sec"
                ),
                ball_recovery_timeout_sec=self._float_parameter(
                    "ball_recovery_timeout_sec"
                ),
                ball_recovery_turn_rad_s=self._float_parameter(
                    "ball_recovery_turn_rad_s"
                ),
                ball_recovery_command_sec=self._float_parameter(
                    "ball_recovery_command_sec"
                ),
                ball_reacquire_center_deg=self._float_parameter(
                    "ball_reacquire_center_deg"
                ),
                ball_reacquire_center_norm=self._float_parameter(
                    "ball_reacquire_center_norm"
                ),
                goal_tracking_range_m=self._float_parameter(
                    "goal_tracking_range_m"
                ),
                goal_control_range_m=self._float_parameter(
                    "goal_control_range_m"
                ),
                hurdle_control_range_m=self._float_parameter(
                    "hurdle_control_range_m"
                ),
                hurdle_path_reference_hold_sec=self._float_parameter(
                    "hurdle_path_reference_hold_sec"
                ),
                goal_lost_stop_sec=self._float_parameter(
                    "goal_lost_stop_sec"
                ),
                goal_recovery_timeout_sec=self._float_parameter(
                    "goal_recovery_timeout_sec"
                ),
                goal_recovery_turn_rad_s=self._float_parameter(
                    "goal_recovery_turn_rad_s"
                ),
                goal_recovery_command_sec=self._float_parameter(
                    "goal_recovery_command_sec"
                ),
                goal_reacquire_center_deg=self._float_parameter(
                    "goal_reacquire_center_deg"
                ),
                goal_reacquire_center_norm=self._float_parameter(
                    "goal_reacquire_center_norm"
                ),
            )
        )

        self.phase_manager = MissionPhaseManager(
            initial_phase=str(
                self.get_parameter("initial_mission_phase").value
            ),
            required_pickups=max(
                0,
                int(self.get_parameter("required_pickups").value),
            ),
            required_shots=max(
                0,
                int(self.get_parameter("required_shots").value),
            ),
            required_hurdles=max(
                0,
                int(self.get_parameter("required_hurdles").value),
            ),
            required_ball_sections=max(
                0,
                int(
                    self.get_parameter(
                        "required_ball_sections"
                    ).value
                ),
            ),
        )
        self.finish_min_confidence = max(
            0.0,
            min(
                1.0,
                self._float_parameter("finish_min_confidence"),
            ),
        )

        self.latest_info: dict[str, dict[str, Any] | None] = {
            source: None for source in self.SOURCES
        }

        self.latest_time: dict[str, float | None] = {
            source: None for source in self.SOURCES
        }

        self.timeouts = {
            source: max(
                0.05,
                float(
                    self.get_parameter(
                        f"{source}_timeout_sec"
                    ).value
                ),
            )
            for source in self.SOURCES
        }

        self.previous_publish_time = time.monotonic()

        self.command_id = 0
        self.event_id = 0
        self.last_candidate_decision: MotionDecision | None = None
        self.last_selected_decision: MotionDecision | None = None
        self.executor_active: bool | None = None
        self.executor_auto_ready = False
        self.executor_ready_requires_fresh_vision = True
        self.pre_motion_settle_sec = max(
            0.0,
            self._float_parameter("pre_motion_settle_sec"),
        )
        # Below the shortest approach band, STRAIGHT_0 would have no line gait.
        self.line_corner_turn_distance_m = max(
            APPROACH_DISTANCE_LIMITS_M[0],
            self._float_parameter("line_corner_turn_distance_m"),
        )
        self.correction_post_motion_dwell_until: float | None = None
        self.correction_post_motion_source: str | None = None
        self.line_offset_alignment_refresh_pending = False
        self.line_offset_min_rgb_stamp_ns: int | None = None
        self.line_ground_recovery_command_id: int | None = None
        self.line_ground_recovery_failed = False
        self.line_ground_recovery_active = False
        self.pre_motion_settle_source: str | None = None
        self.pre_motion_settle_action: str | None = None
        self.pre_motion_settle_started_at: float | None = None
        self.last_published_vision_stamp: dict[str, float] = {}
        self.active_general_source: str | None = None
        self.ball_approach_entry_pending = False
        self.ball_approach_alignment_pending = False
        self.ball_loss_confirmation = BallLossConfirmation()
        self.ball_lost_during_motion_pending = False
        self.ball_last_visible_approach_info: dict[str, Any] | None = None
        self.ball_last_visible_line_info: dict[str, Any] | None = None
        self.ball_pickup_entry_pending = False
        self.ball_post_motion_dwell_until: float | None = None
        self.hurdle_positioning_entry_pending = False
        self.pending_hurdle_final_sequence: dict[str, Any] | None = None
        self.hurdle_post_motion_dwell_until: float | None = None
        self.hurdle_stationary_since: float | None = None
        self.goal_min_rgb_stamp_ns: int | None = None
        self.goal_last_rgb_stamp_ns: int | None = None
        self.goal_post_motion_dwell_until: float | None = None
        self.goal_stationary_since: float | None = None
        self.post_ball_line_dwell_until: float | None = None
        self.post_ball_line_run_until: float | None = None
        self.post_ball_line_run_failed = False
        self.post_shot_dwell_until: float | None = None
        self.post_shot_turn_settled = False
        self.post_shot_line_search_action: str | None = None
        self.post_shot_line_search_failed = False
        self.ball_confirmation_pending_latched = False
        self.ball_confirmation_last_raw_at: float | None = None
        self.ball_approach_observation = BallApproachObservation()
        self.ball_prepared_info = None
        self.ball_prepared_observation = None
        self.ball_prepared_alignment = None
        self.ball_min_rgb_stamp_ns = None
        self.ball_motion_completed_at = None
        self.active_line_motion_action: str | None = None
        self.active_line_motion_started_at: float | None = None
        self.active_line_motion_duration_sec = 0.0
        self.active_line_motion_target_frames = 0
        self.active_line_motion_frames: list[dict[str, Any]] = []
        self.pending_line_decision: MotionDecision | None = None
        self.pending_line_corner: dict[str, Any] | None = None
        self.line_corner_rearm_required = False
        self.remembered_corner_command_id: int | None = None
        self.queued_general_action: str | None = None
        self.queued_general_command_id: int | None = None
        self.queued_general_source: str | None = None
        self.line_timeout_recovery_active = False
        self.line_timeout_recovery_frames: list[dict[str, Any]] = []
        self.terminal_latch: tuple[str, str] | None = None
        self.terminal_action_armed = {
            source: True
            for source in self.SPECIAL_ACTION_SOURCES.values()
        }

        # Special SDK/Dynamics motion lock state.
        self.active_special_event_id: int | None = None
        self.active_special_dynamics_command: int | None = None
        self.pickup_initial_align_waiting = False
        self.pickup_fine_align_waiting = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_positioning_motion_running = False
        self.pickup_positioning_motion_id: str | None = None
        self.pickup_positioning_ball_seen_during_motion = False
        self.pickup_positioning_ball_lost_pending = False
        self.pickup_positioning_loss_latch_sent = False
        self.pickup_fixed_sequence_started = False
        self.grasp_latest_detection_stamp_ns: int | None = None
        self.grasp_verification_active = False
        self.grasp_result_display_until: float | None = None
        self.grasp_verification_attempt: int | None = None
        self.grasp_verification_min_stamp_ns: int | None = None
        self.grasp_verification_result = MissionPhaseManager.GRASP_UNKNOWN
        self.grasp_verification_confidence: float | None = None
        self.grasp_verify_frame_votes: deque[bool] = deque(
            maxlen=MotionDecisionNode.GRASP_VERIFICATION_FRAME_WINDOW
        )
        self.grasp_verify_attempt_serial = 0
        self.grasp_verify_attempt_id: int | None = None
        self.grasp_verify_window_start_monotonic: float | None = None
        self.grasp_verify_window_start_ros_ns: int | None = None
        self.grasp_verify_motion_status_monotonic: float | None = None
        self.grasp_verify_accepted_frames = 0
        self.grasp_verify_rejected_frames = 0
        self.grasp_verify_grab_frames = 0
        self.grasp_verify_miss_frames = 0
        self.grasp_verify_confidences: list[float] = []
        self.grasp_verify_first_stamp_ns: int | None = None
        self.grasp_verify_last_stamp_ns: int | None = None
        self.general_motion_gate = GeneralMotionCommandGate(
            max_transient_retries=max(
                0,
                int(
                    self.get_parameter(
                        "general_motion_transient_retry_limit"
                    ).value
                ),
            )
        )
        self.safety_interlock = SafetyInterlock()
        heartbeat_started_at = time.monotonic()
        self.executor_heartbeat_watchdog = ExecutorHeartbeatWatchdog(
            started_at=heartbeat_started_at,
            startup_grace_sec=max(
                0.0,
                self._float_parameter(
                    "executor_heartbeat_startup_grace_sec"
                ),
            ),
            timeout_sec=max(
                0.0,
                self._float_parameter("executor_heartbeat_timeout_sec"),
            ),
        )

        for source in self.SOURCES:
            topic = str(
                self.get_parameter(
                    f"{source}_info_topic"
                ).value
            )

            self.create_subscription(
                String,
                topic,
                self._info_callback(source, dispatch_on_ball=(source == "ball")),
                10,
            )

            self.get_logger().info(
                f"{source} info: {topic}"
            )

        phase_topic = str(
            self.get_parameter(
                "mission_phase_topic"
            ).value
        )

        self.create_subscription(
            String,
            phase_topic,
            self._phase_callback,
            10,
        )

        motion_status_topic = str(
            self.get_parameter(
                "motion_status_topic"
            ).value
        )

        self.create_subscription(
            String,
            motion_status_topic,
            self._motion_status_callback,
            10,
        )

        grasp_detections_topic = str(
            self.get_parameter("grasp_detections_topic").value
        )
        self.create_subscription(
            String,
            grasp_detections_topic,
            self._grasp_detections_callback,
            10,
        )

        executor_heartbeat_topic = str(
            self.get_parameter("executor_heartbeat_topic").value
        )
        self.create_subscription(
            String,
            executor_heartbeat_topic,
            self._executor_heartbeat_callback,
            10,
        )

        command_topic = str(
            self.get_parameter(
                "command_topic"
            ).value
        )

        self.publisher = self.create_publisher(
            String,
            command_topic,
            10,
        )
        decision_debug_topic = str(
            self.get_parameter("decision_debug_topic").value
        )
        self.decision_debug_publisher = self.create_publisher(
            String,
            decision_debug_topic,
            10,
        )
        self._command_publisher_ready: bool | None = None

        publish_rate = max(
            1.0,
            float(
                self.get_parameter(
                    "publish_rate_hz"
                ).value
            ),
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_decision,
        )
        self.decision_debug_timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_decision_debug,
        )
        self.executor_watchdog_timer = self.create_timer(
            self.EXECUTOR_WATCHDOG_PERIOD_SEC,
            self._check_executor_heartbeat,
        )

        self.get_logger().info(
            f"Mission phase: {self.mission_phase}"
        )
        self.get_logger().info(
            f"Unified command: {command_topic}"
        )
        self.get_logger().info(
            f"Motion status: {motion_status_topic}"
        )
        self.get_logger().info(
            f"Grasp detections: {grasp_detections_topic}"
        )
        self.get_logger().info(
            f"Executor heartbeat: {executor_heartbeat_topic}"
        )
        self.get_logger().info(
            f"Decision debug: {decision_debug_topic}"
        )

        if not self.decision_started:
            try:
                # ROS launch pipes stdin; read the launching terminal directly.
                self.start_terminal_fd = os.open(
                    "/dev/tty", os.O_RDONLY | os.O_NONBLOCK
                )
            except OSError as exc:
                self.get_logger().warning(
                    f"Cannot open start terminal: {exc}; decisions remain blocked."
                )
            else:
                self.get_logger().info(
                    "Press ENTER in the launch terminal to start vision decisions "
                    "after the startup pose is ready."
                )

    def _poll_start_enter(self) -> None:
        """Read Enter without blocking ROS callbacks or the heartbeat watchdog."""
        if self.start_terminal_fd is None:
            return
        try:
            data = os.read(self.start_terminal_fd, 4096)
        except BlockingIOError:
            return
        except OSError as exc:
            self.get_logger().warning(
                f"Start terminal read failed: {exc}; decisions remain blocked."
            )
            MotionDecisionNode._close_start_terminal(self)
            return
        if not data:
            self.get_logger().warning(
                "Start terminal closed; decisions remain blocked."
            )
            MotionDecisionNode._close_start_terminal(self)
            return
        if b"\n" not in data:
            return
        MotionDecisionNode._close_start_terminal(self)
        self.decision_started = True
        self.executor_ready_requires_fresh_vision = True
        self.previous_publish_time = time.monotonic()
        self.get_logger().info(
            "ENTER received; waiting for startup readiness and fresh vision."
        )

    def _close_start_terminal(self) -> None:
        if self.start_terminal_fd is not None:
            os.close(self.start_terminal_fd)
            self.start_terminal_fd = None

    def destroy_node(self):
        MotionDecisionNode._close_start_terminal(self)
        return super().destroy_node()

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _executor_heartbeat_callback(self, message: String) -> None:
        """Record one valid executor process-liveness heartbeat."""
        try:
            payload = json.loads(message.data)
            sequence = payload.get("sequence") if isinstance(payload, dict) else None
            if (
                not isinstance(sequence, int)
                or isinstance(sequence, bool)
                or sequence < 0
            ):
                raise ValueError("heartbeat sequence must be a nonnegative integer")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "Invalid /motion/executor/heartbeat: "
                f"{type(exc).__name__}: {exc}"
            )
            return

        self.executor_heartbeat_watchdog.observe(
            sequence=sequence,
            observed_at=time.monotonic(),
        )
        active = payload.get("active")
        if isinstance(active, bool):
            self.executor_active = active
            if active:
                self.goal_stationary_since = None
        auto_ready = payload.get("auto_ready")
        if isinstance(auto_ready, bool):
            if not auto_ready or not self.executor_auto_ready:
                self.hurdle_stationary_since = None
                self.goal_stationary_since = None
            if not auto_ready:
                self.pending_line_corner = None
                self.line_corner_rearm_required = False
                self.pending_hurdle_final_sequence = None
                self._reset_pre_motion_settle()
                self.executor_ready_requires_fresh_vision = True
            self.executor_auto_ready = auto_ready

    def _check_executor_heartbeat(self, now: float | None = None) -> None:
        """Latch process loss while preserving any active command identity."""
        timeout = self.executor_heartbeat_watchdog.check(
            time.monotonic() if now is None else now
        )
        if timeout is None:
            return

        if isinstance(timeout, HeartbeatStartupDelay):
            self.get_logger().warning(timeout.message)
            return

        if self.active_special_command_id is not None:
            action = self.active_special_action
            command_id = self.active_special_command_id
            event_id = self.active_special_event_id
        elif self.general_motion_gate.locked:
            action = self.general_motion_gate.active_action
            command_id = self.general_motion_gate.active_command_id
            event_id = None
        else:
            action = None
            command_id = None
            event_id = None

        if self.safety_interlock.observe_critical_fault(
            error_code=timeout.error_code,
            message=timeout.message,
            source="executor_watchdog",
            action=action,
            command_id=command_id,
            event_id=event_id,
        ):
            self.get_logger().warning(
                "Executor heartbeat timeout latched; all motion publication "
                f"is blocked until node restart: executor_seen="
                f"{timeout.executor_seen}, action={action}, "
                f"command_id={command_id}"
            )

    @property
    def mission_phase(self) -> str:
        """Expose the Manager-owned phase for legacy readers."""
        return self.phase_manager.current_phase

    @property
    def required_pickups(self) -> int:
        return self.phase_manager.required_pickups

    @property
    def required_shots(self) -> int:
        return self.phase_manager.required_shots

    @property
    def required_ball_sections(self) -> int:
        return self.phase_manager.required_ball_sections

    @property
    def pickups_completed(self) -> int:
        return self.phase_manager.pickups_completed

    @property
    def shots_completed(self) -> int:
        return self.phase_manager.shots_completed

    @property
    def ball_sections_processed(self) -> int:
        return self.phase_manager.ball_sections_processed

    @property
    def finish_enabled(self) -> bool:
        return self.phase_manager.finish_enabled

    @property
    def mission_complete(self) -> bool:
        return self.phase_manager.mission_complete

    @property
    def active_special_action(self) -> str | None:
        return self.phase_manager.active_special_action

    @property
    def active_special_command_id(self) -> int | None:
        return self.phase_manager.active_special_command_id

    @property
    def special_motion_running(self) -> bool:
        return self.phase_manager.active_special_running

    def _info_callback(self, source: str, *, dispatch_on_ball: bool = False):
        def callback(message: String) -> None:
            if not self.decision_started:
                return
            try:
                payload = json.loads(message.data)

                if not isinstance(payload, dict):
                    raise ValueError(
                        "JSON must be an object"
                    )

                if not self.executor_auto_ready:
                    return

                received_at = time.monotonic()
                if source == "line":
                    boundary = getattr(self, "line_offset_min_rgb_stamp_ns", None)
                    if boundary is not None:
                        stamp = MotionDecisionNode._detection_stamp_ns(payload)
                        if stamp is None:
                            stamp = payload.get("rgb_stamp_ns")
                        if (not isinstance(stamp, int) or isinstance(stamp, bool)
                                or stamp <= boundary):
                            return
                if source == "goal":
                    stamp = payload.get("rgb_stamp_ns")
                    boundary = getattr(self, "goal_min_rgb_stamp_ns", None)
                    last_stamp = getattr(self, "goal_last_rgb_stamp_ns", None)
                    ros_now = MotionDecisionNode._current_ros_time_ns(self)
                    valid_stamp = isinstance(stamp, int) and not isinstance(stamp, bool) and stamp > 0
                    if boundary is not None and (not valid_stamp or stamp <= boundary):
                        return
                    if valid_stamp:
                        if last_stamp is not None and stamp <= last_stamp:
                            return
                        if ros_now is not None and not (
                            0 <= ros_now - stamp <= int(self.timeouts["goal"] * 1e9)
                        ):
                            return
                        self.goal_last_rgb_stamp_ns = stamp
                    timer = getattr(self.planner, "goal_loss_timer", None)
                    if timer is not None:
                        timer.max_gap_sec = self.timeouts["goal"]
                if source == "ball":
                    stamp = payload.get("rgb_stamp_ns")
                    boundary = getattr(self, "ball_min_rgb_stamp_ns", None)
                    ros_now = MotionDecisionNode._current_ros_time_ns(self)
                    valid_stamp = (
                        isinstance(stamp, int) and not isinstance(stamp, bool) and stamp > 0
                    )
                    if boundary is not None and (not valid_stamp or stamp <= boundary):
                        return
                    if valid_stamp and ros_now is not None and not (
                        0 <= ros_now - stamp <= int(self.timeouts["ball"] * 1e9)
                    ):
                        return
                if (
                    source == "ball"
                    and not MotionDecisionNode._ball_navigation_blocked(self)
                    and not self.pickup_fixed_sequence_started
                    and self.pickups_completed < self.required_pickups
                ):
                    loss = getattr(self, "ball_loss_confirmation", None)
                    if loss is None:
                        loss = self.ball_loss_confirmation = BallLossConfirmation()
                    if not loss.update(payload, received_at, self.timeouts["ball"]):
                        return
                    payload["ball_loss_confirmed"] = loss.confirmed
                self.latest_info[source] = payload
                self.latest_time[source] = received_at
                if (source == "goal" and payload.get("detected") is True
                        and getattr(self.planner, "goal_tracking_active", False)):
                    # Keep the last visible side current while a motion owns the gate.
                    self.planner._update_goal_tracking(payload, 0.0)
                if source == "hurdle":
                    MotionDecisionNode._latch_hurdle_positioning_entry(self, payload)
                    MotionDecisionNode._latch_hurdle_final_sequence(self, payload)
                if (
                    source == "ball"
                    and not MotionDecisionNode._ball_navigation_blocked(self)
                ):
                    if (
                        self.active_special_action == "PICKUP_NOW"
                        and not self.pickup_fixed_sequence_started
                    ):
                        self.planner._remember_pickup_close_ball(payload)
                    if (
                        self.pickups_completed < self.required_pickups
                        and not self.pickup_fixed_sequence_started
                    ):
                        # Vision callbacks continue while a motion owns the gate.
                        self.planner._update_ball_tracking(payload, 0.0)
                    MotionDecisionNode._update_ball_confirmation_pending(
                        self,
                        payload,
                        received_at,
                    )
                    MotionDecisionNode._latch_ball_approach_entry(
                        self,
                        payload,
                    )
                    MotionDecisionNode._latch_ball_pickup_entry(
                        self,
                        payload,
                    )
                    MotionDecisionNode._track_ball_loss_during_motion(
                        self,
                        payload,
                    )
                    MotionDecisionNode._track_pickup_positioning_ball_loss(
                        self,
                        payload,
                    )
                    MotionDecisionNode._prepare_ball_approach_observation(
                        self, payload, received_at,
                    )
                if source == "line":
                    MotionDecisionNode._remember_line_corner(self, payload)
                    if (
                        self.active_special_command_id is None
                        and (
                            self.active_general_source == "line"
                            or self.mission_phase == "POST_BALL_LINE_ALIGN"
                        )
                    ):
                        self.planner.observe_line_for_search(payload)
                    if self.line_timeout_recovery_active:
                        MotionDecisionNode._collect_timeout_recovery_frame(
                            self,
                            payload,
                        )
                    else:
                        capture_ready = (
                            MotionDecisionNode._collect_active_line_motion_frame(
                                self,
                                payload,
                                received_at,
                            )
                        )
                        if (
                            capture_ready
                            and str(self.active_line_motion_action).startswith(
                                "STRAIGHT"
                            )
                        ):
                            MotionDecisionNode._prepare_pending_line_decision(
                                self,
                                received_at,
                            )
                self.executor_ready_requires_fresh_vision = False
                self.general_motion_gate.on_new_vision_input()
                if (
                    dispatch_on_ball and source == "ball"
                    and getattr(self, "ball_prepared_info", None) is payload
                    and not self.general_motion_gate.locked
                ):
                    # Reuse all normal motion/dwell/safety gates, without
                    # waiting for the next 10 Hz decision timer tick.
                    MotionDecisionNode._publish_decision(self)

            except (
                json.JSONDecodeError,
                TypeError,
                ValueError,
            ) as exc:
                self.get_logger().warning(
                    f"Invalid {source}_info: "
                    f"{type(exc).__name__}: {exc}"
                )

        return callback

    @staticmethod
    def _detection_stamp_ns(payload: dict[str, Any]) -> int | None:
        """Read one valid ROS image stamp from detector JSON."""
        stamp = payload.get("stamp")
        if not isinstance(stamp, dict):
            return None
        sec = stamp.get("sec")
        nanosec = stamp.get("nanosec")
        if (
            isinstance(sec, bool)
            or not isinstance(sec, int)
            or sec < 0
            or isinstance(nanosec, bool)
            or not isinstance(nanosec, int)
            or not 0 <= nanosec < 1_000_000_000
        ):
            return None
        return sec * 1_000_000_000 + nanosec

    @staticmethod
    def _current_ros_time_ns(node: Any) -> int | None:
        """Return the node clock in nanoseconds when it is available."""
        try:
            return int(node.get_clock().now().nanoseconds)
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return None

    @staticmethod
    def _grasp_verify_log(node: Any, marker: str, **fields: Any) -> None:
        """Write one compact structured grasp-verification INFO record."""
        node.get_logger().info(
            f"[{marker}] "
            + json.dumps(
                fields,
                ensure_ascii=True,
                separators=(",", ":"),
                sort_keys=True,
            )
        )

    @staticmethod
    def _reject_grasp_verification_frame(
        node: Any,
        *,
        reason: str,
        stamp_ns: int | None,
        receive_monotonic: float,
        scope: str = "frame",
    ) -> None:
        """Log one rejected frame or malformed detection observation."""
        if not getattr(node, "grasp_verification_active", False):
            return
        if scope == "frame":
            node.grasp_verify_rejected_frames = (
                getattr(node, "grasp_verify_rejected_frames", 0) + 1
            )
        MotionDecisionNode._grasp_verify_log(
            node,
            "GRASP_VERIFY_REJECT",
            attempt_id=getattr(node, "grasp_verify_attempt_id", None),
            ball_index=getattr(node, "grasp_verification_attempt", None),
            minimum_stamp_ns=getattr(
                node, "grasp_verification_min_stamp_ns", None
            ),
            reason=reason,
            receive_monotonic=receive_monotonic,
            scope=scope,
            stamp_ns=stamp_ns,
        )

    def _grasp_detections_callback(self, message: String) -> None:
        """Collect fresh `grab` observations during the post-pose dwell."""
        receive_monotonic = time.monotonic()
        receive_ros_ns = MotionDecisionNode._current_ros_time_ns(self)
        payload: Any = None
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("JSON must be an object")
            stamp_ns = MotionDecisionNode._detection_stamp_ns(payload)
            detections = payload.get("detections")
            if stamp_ns is None or not isinstance(detections, list):
                raise ValueError("stamp and detections are required")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            MotionDecisionNode._reject_grasp_verification_frame(
                self,
                reason=(
                    "missing_or_invalid_stamp_or_detections"
                    if isinstance(payload, dict)
                    else "malformed_json"
                ),
                stamp_ns=(
                    MotionDecisionNode._detection_stamp_ns(payload)
                    if isinstance(payload, dict)
                    else None
                ),
                receive_monotonic=receive_monotonic,
            )
            self.get_logger().warning(
                "Invalid grasp detections: "
                f"{type(exc).__name__}: {exc}"
            )
            return

        previous_stamp = getattr(
            self, "grasp_latest_detection_stamp_ns", None
        )
        if previous_stamp is not None and stamp_ns <= previous_stamp:
            MotionDecisionNode._reject_grasp_verification_frame(
                self,
                reason=(
                    "same_timestamp"
                    if stamp_ns == previous_stamp
                    else "timestamp_regression"
                ),
                stamp_ns=stamp_ns,
                receive_monotonic=receive_monotonic,
            )
            return
        self.grasp_latest_detection_stamp_ns = stamp_ns

        if not getattr(self, "grasp_verification_active", False):
            return
        minimum_stamp = getattr(
            self, "grasp_verification_min_stamp_ns", None
        )
        window_start = getattr(self, "grasp_verify_window_start_ros_ns", None)
        if (
            (minimum_stamp is not None and stamp_ns <= minimum_stamp)
            or (window_start is not None and stamp_ns < window_start)
        ):
            MotionDecisionNode._reject_grasp_verification_frame(
                self,
                reason="pre_window_stamp",
                stamp_ns=stamp_ns,
                receive_monotonic=receive_monotonic,
            )
            return

        best_confidence: float | None = None
        observed_grab_count = 0
        observed_grab_max_confidence: float | None = None
        observed_grab_bbox: Any = None
        ball_detected = False
        for detection_index, detection in enumerate(detections):
            if isinstance(detection, dict):
                ball_detected = bool(
                    ball_detected or detection.get("class_name") == "ball"
                )
            if (
                not isinstance(detection, dict)
                or detection.get("class_name") != "grab"
            ):
                continue
            observed_grab_count += 1
            confidence = detection.get("confidence")
            if isinstance(confidence, bool):
                MotionDecisionNode._reject_grasp_verification_frame(
                    self,
                    reason=f"invalid_confidence[{detection_index}]",
                    stamp_ns=stamp_ns,
                    receive_monotonic=receive_monotonic,
                    scope="detection",
                )
                continue
            try:
                confidence_value = float(confidence)
            except (TypeError, ValueError):
                MotionDecisionNode._reject_grasp_verification_frame(
                    self,
                    reason=f"invalid_confidence[{detection_index}]",
                    stamp_ns=stamp_ns,
                    receive_monotonic=receive_monotonic,
                    scope="detection",
                )
                continue
            if not math.isfinite(confidence_value):
                MotionDecisionNode._reject_grasp_verification_frame(
                    self,
                    reason=f"invalid_confidence[{detection_index}]",
                    stamp_ns=stamp_ns,
                    receive_monotonic=receive_monotonic,
                    scope="detection",
                )
                continue
            if (
                observed_grab_max_confidence is None
                or confidence_value > observed_grab_max_confidence
            ):
                observed_grab_max_confidence = confidence_value
                observed_grab_bbox = detection.get("bbox")
            if confidence_value < self.GRASP_CONFIDENCE_THRESHOLD:
                continue
            best_confidence = (
                confidence_value
                if best_confidence is None
                else max(best_confidence, confidence_value)
            )

        frame_result = (
            MissionPhaseManager.GRASPED
            if best_confidence is not None
            else MissionPhaseManager.GRASP_NOT_GRABBED
        )
        self.grasp_verification_confidence = best_confidence
        # Each fresh frame contributes one vote, regardless of detection count.
        self.grasp_verify_frame_votes.append(best_confidence is not None)
        window_grabbed_frames = sum(self.grasp_verify_frame_votes)
        self.grasp_verification_result = (
            MissionPhaseManager.GRASPED
            if window_grabbed_frames
            >= MotionDecisionNode.GRASP_VERIFICATION_MIN_SUCCESSES
            else MissionPhaseManager.GRASP_NOT_GRABBED
        )

        accepted_frames = getattr(
            self, "grasp_verify_accepted_frames", 0
        ) + 1
        self.grasp_verify_accepted_frames = accepted_frames
        if frame_result == MissionPhaseManager.GRASPED:
            self.grasp_verify_grab_frames = (
                getattr(self, "grasp_verify_grab_frames", 0) + 1
            )
        else:
            self.grasp_verify_miss_frames = (
                getattr(self, "grasp_verify_miss_frames", 0) + 1
            )
        if observed_grab_max_confidence is not None:
            self.grasp_verify_confidences.append(
                observed_grab_max_confidence
            )
        prior_accepted_stamp = getattr(
            self, "grasp_verify_last_stamp_ns", None
        )
        if getattr(self, "grasp_verify_first_stamp_ns", None) is None:
            self.grasp_verify_first_stamp_ns = stamp_ns
        self.grasp_verify_last_stamp_ns = stamp_ns
        window_start_ros_ns = getattr(
            self, "grasp_verify_window_start_ros_ns", None
        )
        motion_status_monotonic = getattr(
            self, "grasp_verify_motion_status_monotonic", None
        )
        MotionDecisionNode._grasp_verify_log(
            self,
            "GRASP_VERIFY_FRAME",
            accepted_reason="fresh_stamp_after_window_boundary",
            attempt_id=getattr(self, "grasp_verify_attempt_id", None),
            ball_detected=ball_detected,
            ball_index=getattr(self, "grasp_verification_attempt", None),
            bbox=observed_grab_bbox,
            callback_receive_ros_ns=receive_ros_ns,
            callback_receive_monotonic=receive_monotonic,
            frame_age_at_receive_sec=(
                (receive_ros_ns - stamp_ns) / 1_000_000_000.0
                if receive_ros_ns is not None
                else None
            ),
            frame_idx=accepted_frames,
            frame_result=frame_result,
            frame_stamp_vs_window_sec=(
                (stamp_ns - window_start_ros_ns) / 1_000_000_000.0
                if window_start_ros_ns is not None
                else None
            ),
            grab_count=observed_grab_count,
            grab_max_confidence=observed_grab_max_confidence,
            grab_pass=best_confidence is not None,
            motion_status_to_receive_sec=(
                receive_monotonic - motion_status_monotonic
                if motion_status_monotonic is not None
                else None
            ),
            pending_result=self.grasp_verification_result,
            decision_window_frames=len(self.grasp_verify_frame_votes),
            decision_window_grabbed_frames=window_grabbed_frames,
            stamp_delta_sec=(
                (stamp_ns - prior_accepted_stamp) / 1_000_000_000.0
                if prior_accepted_stamp is not None
                else None
            ),
            stamp_ns=stamp_ns,
            window_start_monotonic=getattr(
                self, "grasp_verify_window_start_monotonic", None
            ),
        )

    def _start_grasp_verification_window(
        self,
        *,
        completed_motion: str = PICKUP_GRASP_CHECK_MOTION_ID,
        motion_status_receive_monotonic: float | None = None,
    ) -> None:
        """Open the pickup attempt's window after check-pose success."""
        attempt = self.phase_manager.active_pickup_attempt
        if attempt is None:
            return
        window_start_monotonic = time.monotonic()
        window_start_ros_ns = MotionDecisionNode._current_ros_time_ns(self)
        self.grasp_verification_active = True
        self.grasp_result_display_until = None
        self.grasp_verification_attempt = attempt
        self.grasp_verification_min_stamp_ns = getattr(
            self, "grasp_latest_detection_stamp_ns", None
        )
        self.grasp_verification_result = MissionPhaseManager.GRASP_UNKNOWN
        self.grasp_verification_confidence = None
        self.grasp_verify_frame_votes = deque(
            maxlen=MotionDecisionNode.GRASP_VERIFICATION_FRAME_WINDOW
        )
        self.grasp_verify_attempt_serial = (
            getattr(self, "grasp_verify_attempt_serial", 0) + 1
        )
        self.grasp_verify_attempt_id = self.grasp_verify_attempt_serial
        self.grasp_verify_window_start_monotonic = window_start_monotonic
        self.grasp_verify_window_start_ros_ns = window_start_ros_ns
        self.grasp_verify_motion_status_monotonic = (
            motion_status_receive_monotonic
            if motion_status_receive_monotonic is not None
            else window_start_monotonic
        )
        self.grasp_verify_accepted_frames = 0
        self.grasp_verify_rejected_frames = 0
        self.grasp_verify_grab_frames = 0
        self.grasp_verify_miss_frames = 0
        self.grasp_verify_confidences = []
        self.grasp_verify_first_stamp_ns = None
        self.grasp_verify_last_stamp_ns = None
        MotionDecisionNode._grasp_verify_log(
            self,
            "GRASP_VERIFY_START",
            attempt_id=self.grasp_verify_attempt_id,
            ball_index=attempt,
            completed_motion=completed_motion,
            confidence_threshold=self.GRASP_CONFIDENCE_THRESHOLD,
            frame_window_limit=MotionDecisionNode.GRASP_VERIFICATION_FRAME_WINDOW,
            required_grab_frames=MotionDecisionNode.GRASP_VERIFICATION_MIN_SUCCESSES,
            window_scope="post_check_pose_dwell",
            expected_window_sec=3.0,
            initial_result=self.grasp_verification_result,
            minimum_accepted_stamp_ns=self.grasp_verification_min_stamp_ns,
            motion_status_receive_monotonic=(
                self.grasp_verify_motion_status_monotonic
            ),
            window_start_monotonic=window_start_monotonic,
            window_start_ros_ns=window_start_ros_ns,
        )

    def _finish_grasp_verification_window(self) -> None:
        """Latch the rolling vote result, or UNKNOWN if no frame arrived."""
        if not getattr(self, "grasp_verification_active", False):
            return
        result = getattr(
            self,
            "grasp_verification_result",
            MissionPhaseManager.GRASP_UNKNOWN,
        )
        attempt = getattr(self, "grasp_verification_attempt", None)
        self.phase_manager.record_active_pickup_grasp_result(result)
        latched_result = (
            self.phase_manager.grasp_result_for_ball(attempt)
            if attempt is not None
            else result
        )
        end_monotonic = time.monotonic()
        self.grasp_verification_result = latched_result
        self.grasp_result_display_until = (
            end_monotonic + MotionDecisionNode.GRASP_RESULT_DISPLAY_SEC
            if latched_result in {
                MissionPhaseManager.GRASPED, MissionPhaseManager.GRASP_NOT_GRABBED,
            }
            else None
        )
        confidences = getattr(self, "grasp_verify_confidences", [])
        frame_votes = self.grasp_verify_frame_votes
        last_frame_result = MissionPhaseManager.GRASP_UNKNOWN
        if frame_votes:
            last_frame_result = (
                MissionPhaseManager.GRASPED
                if frame_votes[-1]
                else MissionPhaseManager.GRASP_NOT_GRABBED
            )
        MotionDecisionNode._grasp_verify_log(
            self,
            "GRASP_VERIFY_END",
            accepted_frames=getattr(
                self, "grasp_verify_accepted_frames", 0
            ),
            attempt_id=getattr(self, "grasp_verify_attempt_id", None),
            ball_index=attempt,
            duration_sec=(
                end_monotonic - self.grasp_verify_window_start_monotonic
                if getattr(
                    self, "grasp_verify_window_start_monotonic", None
                ) is not None
                else None
            ),
            final_confidence=getattr(
                self, "grasp_verification_confidence", None
            ),
            first_accepted_stamp_ns=getattr(
                self, "grasp_verify_first_stamp_ns", None
            ),
            fresh_frame_zero=(
                getattr(self, "grasp_verify_accepted_frames", 0) == 0
            ),
            grab_confidence_max=max(confidences) if confidences else None,
            grab_confidence_mean=(
                statistics.fmean(confidences) if confidences else None
            ),
            grab_confidence_median=(
                statistics.median(confidences) if confidences else None
            ),
            grab_confidence_min=min(confidences) if confidences else None,
            grabbed_frames=getattr(self, "grasp_verify_grab_frames", 0),
            last_accepted_stamp_ns=getattr(
                self, "grasp_verify_last_stamp_ns", None
            ),
            last_fresh_frame_result=last_frame_result,
            latched_result=latched_result,
            decision_window_frames=len(frame_votes),
            decision_window_grabbed_frames=sum(frame_votes),
            frame_window_limit=MotionDecisionNode.GRASP_VERIFICATION_FRAME_WINDOW,
            required_grab_frames=MotionDecisionNode.GRASP_VERIFICATION_MIN_SUCCESSES,
            not_grabbed_frames=getattr(
                self, "grasp_verify_miss_frames", 0
            ),
            rejected_frames=getattr(
                self, "grasp_verify_rejected_frames", 0
            ),
            window_end_monotonic=end_monotonic,
        )
        self.grasp_verification_active = False
        self.grasp_verification_attempt = None
        self.grasp_verification_min_stamp_ns = None

    def _ball_navigation_blocked(self) -> bool:
        """Block Ball input during its exit route or after its execution limit."""
        return (
            self.mission_phase in MotionDecisionNode.BALL_NAVIGATION_BLOCKED_PHASES
            or (
                self.phase_manager.execution_limit_reached("PICKUP_NOW")
                and self.active_special_action != "PICKUP_NOW"
            )
        )

    def _clear_ball_navigation_state(self) -> None:
        """Discard approach reservations without releasing an executing motion."""
        self.ball_approach_entry_pending = False
        self.ball_approach_alignment_pending = False
        self.ball_pickup_entry_pending = False
        self.ball_lost_during_motion_pending = False
        self.ball_last_visible_approach_info = None
        self.ball_last_visible_line_info = None
        self.ball_confirmation_pending_latched = False
        self.ball_confirmation_last_raw_at = None
        self.ball_post_motion_dwell_until = None
        observer = getattr(self, "ball_approach_observation", None)
        if observer is not None:
            observer.reset()
        self.ball_prepared_info = None
        self.ball_prepared_observation = None
        self.ball_prepared_alignment = None
        self.ball_min_rgb_stamp_ns = None
        self.ball_motion_completed_at = None
        loss = getattr(self, "ball_loss_confirmation", None)
        if loss is not None:
            loss.reset_missing()
        self.planner.clear_collected_ball_tracking()

    def _latch_ball_approach_entry(self, payload: dict[str, Any]) -> None:
        """Remember the first confirmed 1.5 m Ball entry."""
        if (
            self.ball_approach_entry_pending
            or bool(getattr(self.planner, "ball_lock_active", False))
            or self.active_general_source == "ball"
            or self.active_special_command_id is not None
            or payload.get("detected") is not True
            or payload.get("depth_valid") is not True
        ):
            return

        requested_source = self.planner.source_for_phase(
            self.phase_manager.current_phase
        )
        if (
            requested_source not in {None, "line", "ball"}
            or bool(getattr(self.planner, "goal_lock_active", False))
            or bool(getattr(self.planner, "hurdle_lock_active", False))
        ):
            return

        config = self.planner.ball_planner.config
        confidence = self.planner._number(payload, "confidence")
        distance = self.planner._number(payload, "distance_m")
        if (
            confidence is None
            or confidence < config.min_confidence
            or distance is None
            or distance <= 0.0
            or distance > config.control_start_depth_m
        ):
            return

        # A Ball already inside the pickup-entry range is handled by the
        # PICKUP_NOW sequence and its camera-down alignment checkpoints.
        if distance <= config.pickup_sequence_start_distance_m:
            return

        self.ball_approach_entry_pending = True
        self.planner.ball_lock_active = True
        entry_context = "during motion"
        if not self.general_motion_gate.locked:
            # The confirming stationary frame can start initial alignment.
            self.ball_approach_alignment_pending = True
            entry_context = "while stationary"
        self.get_logger().info(
            f"BALL approach entry latched {entry_context}: "
            f"distance_m={distance:.3f}"
        )

    def _update_ball_confirmation_pending(
        self,
        payload: dict[str, Any],
        received_at: float,
    ) -> None:
        """Confirm while walking; reserve a hold for near/unknown candidates."""
        if payload.get("detected") is True:
            if self.ball_confirmation_pending_latched:
                self.get_logger().info(
                    "BALL confirmation completed; releasing pending hold"
                )
            self.ball_confirmation_pending_latched = False
            self.ball_confirmation_last_raw_at = None
            return

        if payload.get("raw_detected") is True:
            candidate = payload.get("approach_candidate")
            config = self.planner.ball_planner.config
            distance = (
                MotionDecisionPlanner._number(candidate, "distance_m")
                if isinstance(candidate, dict) else None
            )
            depth_age = (
                MotionDecisionPlanner._number(candidate, "depth_age_sec")
                if isinstance(candidate, dict) else None
            )
            if (
                isinstance(candidate, dict) and candidate.get("depth_valid") is True
                and distance is not None and distance > config.slowdown_depth_m
                and depth_age is not None
                and 0 <= depth_age <= BallApproachObservation.MAX_DEPTH_AGE_SEC
            ):
                self.ball_confirmation_pending_latched = False
                self.ball_confirmation_last_raw_at = None
                return
            if not self.ball_confirmation_pending_latched:
                self.get_logger().info(
                    "BALL near/unknown candidate; waiting for confirmation "
                    "after the current motion"
                )
            self.ball_confirmation_pending_latched = True
            self.ball_confirmation_last_raw_at = received_at
            return

        last_raw_at = self.ball_confirmation_last_raw_at
        if (
            self.ball_confirmation_pending_latched
            and last_raw_at is not None
            and received_at - last_raw_at
            >= self.BALL_RAW_CONFIRMATION_RELEASE_SEC
        ):
            self.ball_confirmation_pending_latched = False
            self.ball_confirmation_last_raw_at = None
            self.get_logger().info(
                "BALL raw detection lost; releasing pending hold"
            )

    def _prepare_ball_approach_observation(
        self, payload: dict[str, Any], received_at: float,
    ) -> None:
        """Prepare alignment while moving, without advancing a stateful planner."""
        observer = getattr(self, "ball_approach_observation", None)
        if observer is None:
            observer = self.ball_approach_observation = BallApproachObservation()
        self.ball_prepared_info = None
        self.ball_prepared_observation = None
        self.ball_prepared_alignment = None
        if (
            self.active_special_command_id is not None
            or not getattr(self.planner, "ball_lock_active", False)
            or str(self.general_motion_gate.active_action).startswith("BALL_APPROACH_TURN_")
            or self.ball_post_motion_dwell_until is not None
        ):
            observer.reset()
            return
        observer.observe(payload, received_at, self.planner.ball_planner.config)
        observation = payload if payload.get("detected") is True else observer.observation
        if observation is not None:
            self.ball_prepared_info = payload
            self.ball_prepared_observation = observation
            self.ball_prepared_alignment = self.planner.plan_ball_approach_alignment(observation)

    def _latch_hurdle_positioning_entry(self, payload: dict[str, Any]) -> None:
        """Remember the near crossing without interrupting an active motion."""
        if (
            getattr(self, "hurdle_positioning_entry_pending", False)
            or self.mission_phase == "HURDLE_POSITIONING"
            or self.active_special_command_id is not None
            or self.phase_manager.hurdles_completed >= self.phase_manager.required_hurdles
            or self.phase_manager.execution_limit_reached("GO")
            or getattr(self.planner, "hurdle_ignore_until_clear", False)
            or not self.planner._hurdle_positioning_ready(payload)
        ):
            return
        if self.general_motion_gate.locked:
            if self.active_general_source not in {"line", "hurdle"}:
                return
            if self.mission_phase in MotionDecisionNode.BALL_NAVIGATION_BLOCKED_PHASES:
                return
        elif (
            self.mission_phase not in {"AUTO", "LINE_TRACK", "HURDLE_APPROACH"}
            and not getattr(self.planner, "hurdle_lock_active", False)
        ):
            return
        depth = MotionDecisionPlanner._number(payload, "depth_m")
        bottom = MotionDecisionPlanner._number(payload, "bottom_distance_px")
        self.hurdle_positioning_entry_pending = True
        if not self.general_motion_gate.locked:
            self.hurdle_stationary_since = time.monotonic()
        self.get_logger().info(
            f"HURDLE positioning entry latched: depth_m={depth}, "
            f"bottom_distance_px={bottom}, head_down_requested={payload.get('head_down_requested', False)}"
        )

    def _latch_hurdle_final_sequence(self, payload: dict[str, Any]) -> None:
        """Keep a confirmed final checkpoint across motion completion and occlusion."""
        if (
            getattr(self, "pending_hurdle_final_sequence", None) is not None
            or self.active_special_command_id is not None
            or not self.executor_auto_ready
            or self.phase_manager.hurdles_completed >= self.phase_manager.required_hurdles
            or self.phase_manager.execution_limit_reached("GO")
            or getattr(self.planner, "hurdle_ignore_until_clear", False)
            or not self.planner._hurdle_positioning_ready(payload)
            or not (
                self.mission_phase == "HURDLE_POSITIONING"
                or getattr(self, "hurdle_positioning_entry_pending", False)
            )
        ):
            return
        if (
            self.general_motion_gate.locked
            and self.active_general_source not in {"line", "hurdle"}
        ):
            return
        depth = MotionDecisionPlanner._number(payload, "depth_m")
        config = self.planner.hurdle_planner.config
        if depth > config.final_sequence_start_depth_m + 1e-9:
            return
        # Inspect without changing the active planner while another motion runs.
        checkpoint_planner = HurdleNavigationPlanner(config)
        checkpoint_planner.close_rotation_blocked = (
            self.planner.hurdle_planner.close_rotation_blocked
        )
        checkpoint = checkpoint_planner.plan(payload, positioning=True)
        if not checkpoint.valid or not checkpoint.fine_sequence_requested:
            return
        self.pending_hurdle_final_sequence = {
            **checkpoint.to_dict(),
            "hurdle_stage": "FINE_APPROACH",
            "hurdle_positioning_active": True,
            "hurdle_final_sequence_latched": True,
            "observed_during_action": self.general_motion_gate.active_action,
            "observed_during_command_id": self.general_motion_gate.active_command_id,
        }
        self.pending_line_decision = None
        self.get_logger().info(
            f"HURDLE final sequence latched: depth_m={depth}, "
            f"center_steering_deg={checkpoint.center_steering_deg}, "
            f"observed_during_action={self.general_motion_gate.active_action}"
        )

    def _latch_ball_pickup_entry(self, payload: dict[str, Any]) -> None:
        """Remember the first valid 57 cm crossing owned by BALL control."""
        if (
            self.ball_pickup_entry_pending
            or payload.get("detected") is not True
            or payload.get("depth_valid") is not True
        ):
            return

        if self.general_motion_gate.locked:
            if self.active_general_source != "ball":
                return
        else:
            requested_source = self.planner.source_for_phase(
                self.phase_manager.current_phase
            )
            if (
                self.active_special_command_id is not None
                or requested_source not in {None, "ball"}
                or bool(getattr(self.planner, "goal_lock_active", False))
                or bool(getattr(self.planner, "hurdle_lock_active", False))
            ):
                return

        config = self.planner.ball_planner.config
        confidence = self.planner._number(payload, "confidence")
        distance = self.planner._number(payload, "distance_m")
        depth_age = self.planner._number(payload, "depth_age_sec")
        if (
            confidence is None
            or confidence < config.min_confidence
            or distance is None
            or distance <= 0.0
            or distance > config.pickup_sequence_start_distance_m
            or depth_age is None
            or depth_age < 0.0
            or depth_age > config.max_pickup_depth_age_sec
        ):
            return

        self.ball_pickup_entry_pending = True
        self.ball_lost_during_motion_pending = False
        self.ball_last_visible_approach_info = None
        self.ball_last_visible_line_info = None
        if not self.general_motion_gate.locked:
            self.ball_approach_alignment_pending = False
        self.get_logger().info(
            "BALL pickup entry latched: "
            f"distance_m={distance:.3f}"
        )

    def _track_ball_loss_during_motion(
        self,
        payload: dict[str, Any],
    ) -> None:
        """Latch a 57-150 cm Ball loss until the active motion ends."""
        if (
            not self.general_motion_gate.locked
            or self.active_general_source != "ball"
            or self.ball_pickup_entry_pending
        ):
            return

        config = self.planner.ball_planner.config
        distance = self.planner._number(payload, "distance_m")
        in_recovery_range = bool(
            distance is not None
            and config.pickup_sequence_start_distance_m < distance
            <= config.control_start_depth_m
        )
        if (
            payload.get("detected") is True
            and payload.get("depth_valid") is True
            and in_recovery_range
        ):
            self.ball_last_visible_approach_info = dict(payload)
            return

        actually_lost = bool(
            payload.get("detected") is not True
            and payload.get("raw_detected") is not True
            and payload.get("ball_loss_confirmed") is True
        )
        if (
            actually_lost
            and self.ball_last_visible_approach_info is not None
            and not self.ball_lost_during_motion_pending
        ):
            self.ball_lost_during_motion_pending = True
            self.get_logger().info(
                "BALL lost during 57-150 cm approach motion; "
                "remembering the last Ball side for the motion boundary"
            )

    def _track_pickup_positioning_ball_loss(
        self,
        payload: dict[str, Any],
    ) -> None:
        """Latch Ball loss during a running pre-grasp pickup motion."""
        if (
            self.active_special_action != "PICKUP_NOW"
            or not self.pickup_positioning_motion_running
            or self.pickup_fixed_sequence_started
        ):
            return

        if payload.get("detected") is True:
            self.pickup_positioning_ball_seen_during_motion = True
            return

        actually_lost = bool(
            payload.get("detected") is not True
            and payload.get("raw_detected") is not True
            and payload.get("ball_loss_confirmed") is True
        )
        if (
            actually_lost
            and self.pickup_positioning_ball_seen_during_motion
            and not self.pickup_positioning_ball_lost_pending
        ):
            self.pickup_positioning_ball_lost_pending = True
            self.pickup_positioning_loss_latch_sent = False
            self.get_logger().info(
                "BALL lost during pickup positioning; preserving the active "
                "motion before camera-down reacquisition"
            )

    def _phase_callback(self, message: String) -> None:
        phase = message.data.strip()

        if not phase:
            return

        try:
            payload = json.loads(phase)

            if isinstance(payload, dict):
                phase = str(
                    payload.get("phase", "")
                ).strip()

        except json.JSONDecodeError:
            pass

        if not phase:
            self.get_logger().warning(
                "Mission phase override rejected: empty phase"
            )
            return

        if self.active_special_command_id is not None:
            self.get_logger().warning(
                "Mission phase override rejected: "
                "special motion is active "
                f"(action={self.active_special_action}, "
                f"command_id={self.active_special_command_id})"
            )
            return

        if not self.phase_manager.set_phase(phase):
            self.get_logger().warning(
                "Mission phase override rejected: "
                f"unsupported phase={phase!r}"
            )
            return

        self.pending_line_corner = None
        self.line_corner_rearm_required = False
        self.hurdle_positioning_entry_pending = False
        self.pending_hurdle_final_sequence = None
        self.hurdle_post_motion_dwell_until = None
        self.hurdle_stationary_since = None
        self.goal_stationary_since = None
        if MotionDecisionNode._ball_navigation_blocked(self):
            MotionDecisionNode._clear_ball_navigation_state(self)

        self.get_logger().info(
            f"Mission phase changed: {self.mission_phase}"
        )

    def _motion_status_callback(
        self,
        message: String,
    ) -> None:
        """Track execution state reported by the command bridge."""
        status_receive_monotonic = time.monotonic()
        try:
            payload = json.loads(message.data)

            if not isinstance(payload, dict):
                raise ValueError(
                    "JSON must be an object"
                )

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError,
        ) as exc:
            self.get_logger().warning(
                "Invalid /motion/status: "
                f"{type(exc).__name__}: {exc}"
            )
            return

        status = str(
            payload.get("status", "")
        ).strip().upper()

        action = payload.get("action")
        command_id = payload.get("command_id")
        event_id = payload.get("event_id")
        error_code = payload.get("error_code")
        status_message = payload.get("message")
        dynamics_command = payload.get(
            "dynamics_command"
        )

        if normalize_general_action(action) is not None:
            if status == "QUEUED":
                return
            if (
                status == "REJECTED"
                and action == getattr(self, "queued_general_action", None)
                and command_id == getattr(
                    self,
                    "queued_general_command_id",
                    None,
                )
            ):
                self.queued_general_action = None
                self.queued_general_command_id = None
                self.queued_general_source = None
                self.pending_line_decision = None
                self.pending_hurdle_final_sequence = None
                return
            if (
                status == "RUNNING"
                and not self.general_motion_gate.locked
                and action == self.queued_general_action
                and command_id == self.queued_general_command_id
            ):
                self.general_motion_gate.on_command_published(
                    action,
                    command_id,
                )
                self.active_general_source = self.queued_general_source
                self.queued_general_action = None
                self.queued_general_command_id = None
                self.queued_general_source = None
            transition = self.general_motion_gate.on_motion_status(
                action,
                status,
                command_id,
                error_code,
            )
            if transition.matched:
                if status == "RUNNING":
                    self.hurdle_stationary_since = None
                    self.goal_stationary_since = None
                if (
                    status == "RUNNING"
                    and self.active_general_source == "line"
                    and self.mission_phase == "LINE_TRACK_AFTER_PICKUP"
                    and action != "STOP"
                    and getattr(self, "post_ball_line_run_until", None) is None
                ):
                    self.post_ball_line_run_until = (
                        status_receive_monotonic + MotionDecisionNode.POST_BALL_LINE_RUN_SEC
                    )
                    self.get_logger().info("Post-pickup normal Line driving started: 10 seconds")
                self._latch_critical_executor_fault(
                    error_code=error_code,
                    message=status_message,
                    action=action,
                    command_id=command_id,
                    event_id=event_id,
                )
                if (
                    status == "RUNNING"
                    and self.active_general_source == "line"
                    and self.active_line_motion_started_at is None
                ):
                    MotionDecisionNode._start_line_motion_capture(
                        self,
                        str(action),
                    )
            if transition.released:
                if command_id == getattr(self, "line_ground_recovery_command_id", None):
                    self.line_ground_recovery_command_id = None
                    self.line_ground_recovery_failed = status != "SUCCEEDED"
                    if status == "SUCCEEDED":
                        self.line_offset_alignment_refresh_pending = True
                self.goal_stationary_since = (
                    status_receive_monotonic if status == "SUCCEEDED" else None
                )
                completed_source = self.active_general_source
                limiter = getattr(self, "lost_search_turn_limiter", None)
                if limiter is not None and status != "SUCCEEDED":
                    limiter.record_failure(completed_source, str(action))
                if completed_source == "line":
                    sparse = getattr(self, "sparse_line_recovery", None)
                    if sparse is not None and str(action).startswith("LINE_SPARSE_"):
                        sparse.failed = status != "SUCCEEDED"
                        sparse.reset_observations()
                    self.line_motion_completed_at = status_receive_monotonic
                    self.line_motion_min_rgb_stamp_ns = (
                        MotionDecisionNode._current_ros_time_ns(self)
                    )
                    corner = getattr(self, "pending_line_corner", None)
                    if corner is not None:
                        corner["motion_completed_at"] = status_receive_monotonic
                        corner["minimum_rgb_stamp_ns"] = self.line_motion_min_rgb_stamp_ns
                    if status != "SUCCEEDED":
                        self.pending_line_corner = None
                        self.line_corner_rearm_required = True
                    if command_id == getattr(self, "remembered_corner_command_id", None):
                        self.remembered_corner_command_id = None
                        if status == "SUCCEEDED":
                            # The remembered near point preceded this corner turn.
                            self.planner.last_line_seen_direction = None
                existing_pause = (
                    (completed_source == "hurdle" and action in {
                        "ALIGN_LEFT", "ALIGN_RIGHT", "TURN_LEFT", "TURN_RIGHT",
                    })
                    or (completed_source == "ball"
                        and str(action).startswith("BALL_APPROACH_TURN_"))
                    or (completed_source == "goal"
                        and str(action).startswith("GOAL_CAMERA90_TURN_"))
                    or (completed_source == "line" and str(action).startswith(
                        ("POST_BALL_LINE_TURN_", "POST_SHOT_")
                    ))
                )
                if (
                    status == "SUCCEEDED" and not existing_pause
                    and MotionDecisionNode._needs_correction_dwell(action)
                ):
                    self.correction_post_motion_dwell_until = (
                        status_receive_monotonic
                        + MotionDecisionNode.CORRECTION_POST_MOTION_DWELL_SEC
                    )
                    self.correction_post_motion_source = completed_source
                    self.line_offset_alignment_refresh_pending = str(action).startswith(
                        ("LINE_OFFSET_TURN_", "LINE_HEADING_TURN_", "LINE_SPARSE_"))
                self.active_general_source = None
                if (
                    completed_source == "hurdle"
                    and self.mission_phase == "HURDLE_POSITIONING"
                ) or (
                    completed_source in {"line", "hurdle"}
                    and getattr(self, "hurdle_positioning_entry_pending", False)
                ):
                    if status != "SUCCEEDED":
                        self.pending_hurdle_final_sequence = None
                        self.hurdle_positioning_entry_pending = False
                        self.hurdle_post_motion_dwell_until = None
                        self.hurdle_stationary_since = None
                    hurdle_turn = (
                        completed_source == "hurdle"
                        and action in {"ALIGN_LEFT", "ALIGN_RIGHT", "TURN_LEFT", "TURN_RIGHT"}
                    )
                    if status == "SUCCEEDED":
                        # Include the motion that first crossed the entry distance.
                        self.hurdle_stationary_since = status_receive_monotonic
                        self.hurdle_post_motion_dwell_until = (
                            status_receive_monotonic
                            + MotionDecisionNode.HURDLE_POST_MOTION_DWELL_SEC
                            if hurdle_turn else None
                        )
                        if not hurdle_turn:
                            self.latest_info["hurdle"] = None
                            self.latest_time["hurdle"] = None
                            gate = self.general_motion_gate
                            gate.required_vision_generation = gate.vision_generation + 1
                if (
                    completed_source == "line"
                    and self.mission_phase == "LINE_TRACK_AFTER_PICKUP"
                    and status != "SUCCEEDED"
                ):
                    self.post_ball_line_run_failed = True
                    MotionDecisionNode._discard_line_motion_capture(self)
                    self.pending_line_decision = None
                    self.get_logger().warning(
                        f"Post-pickup Line driving aborted: action={action}, status={status}"
                    )
                    return
                if (
                    completed_source == "line"
                    and self.mission_phase == "LINE_TRACK"
                    and action == getattr(self, "post_shot_line_search_action", None)
                ):
                    # Finish each search turn before checking stationary Vision again.
                    MotionDecisionNode._discard_line_motion_capture(self)
                    self.pending_line_decision = None
                    MotionDecisionNode._invalidate_post_ball_line_input(self)
                    self.post_shot_line_search_failed = status != "SUCCEEDED"
                    if status == "SUCCEEDED":
                        self.post_shot_dwell_until = (
                            status_receive_monotonic
                            + MotionDecisionNode.POST_SHOT_DWELL_SEC
                        )
                    return
                if (
                    completed_source == "line"
                    and self.phase_manager.handle_post_shot_motion(action, status)
                ):
                    MotionDecisionNode._discard_line_motion_capture(self)
                    self.pending_line_decision = None
                    MotionDecisionNode._invalidate_post_ball_line_input(self)
                    if status == "SUCCEEDED":
                        if self.mission_phase in {"AUTO", "LINE_TRACK"} or action in {
                            "POST_SHOT_TURN_RIGHT_9", "POST_SHOT_TURN_LEFT_4",
                        }:
                            MotionDecisionNode._clear_ball_navigation_state(self)
                            MotionDecisionNode._invalidate_pickup_ball_input(self)
                            # Release the scored Goal before normal Line planning.
                            self.planner._clear_goal_tracking()
                            self.planner.goal_lock_active = False
                            self.planner.goal_terminal_requested = False
                            self.planner.goal_ignore_until_clear = True
                            self.latest_info["goal"] = None
                            self.latest_time["goal"] = None
                            if action in {
                                "POST_SHOT_TURN_RIGHT_9", "POST_SHOT_TURN_LEFT_4",
                            }:
                                # Only the exit turn invalidates the pre-shot direction.
                                self.planner.last_line_seen_direction = None
                                self.post_shot_line_search_action = None
                                self.post_shot_line_search_failed = False
                                self.planner.post_ball_line_search_direction = (
                                    "RIGHT" if "_RIGHT_" in action else "LEFT"
                                )
                        if "_TURN_" in action:
                            self.post_shot_turn_settled = False
                            self.post_shot_dwell_until = (
                                status_receive_monotonic
                                + MotionDecisionNode.POST_SHOT_DWELL_SEC
                            )
                        self.planner.line_planner.stop("post_shot_motion_complete")
                    self.get_logger().info(
                        f"Post-shot motion ended: action={action}, "
                        f"status={status}, phase={self.mission_phase}"
                    )
                    return
                if completed_source == "goal":
                    MotionDecisionNode._invalidate_goal_input(self)
                    goal_turn = str(action).startswith("GOAL_CAMERA90_TURN_")
                    self.goal_post_motion_dwell_until = (
                        status_receive_monotonic + MotionDecisionNode.GOAL_POST_MOTION_DWELL_SEC
                        if status == "SUCCEEDED" and goal_turn else None
                    )
                    if status == "SUCCEEDED" and not goal_turn:
                        gate = self.general_motion_gate
                        gate.required_vision_generation = gate.vision_generation + 1
                if completed_source == "ball":
                    self.ball_motion_completed_at = status_receive_monotonic
                    completed_approach_turn = str(action).startswith(
                        "BALL_APPROACH_TURN_"
                    )
                    if status != "SUCCEEDED":
                        self.ball_pickup_entry_pending = False
                        self.ball_approach_alignment_pending = False
                        self.ball_lost_during_motion_pending = False
                        self.ball_last_visible_approach_info = None
                        self.ball_last_visible_line_info = None
                    pickup_entry_ready = bool(
                        status == "SUCCEEDED"
                        and self.ball_pickup_entry_pending
                    )
                    if status == "SUCCEEDED" and not pickup_entry_ready:
                        # A stationary correction consumes this motion
                        # boundary's single heading check. After the dwell,
                        # continue with the next distance-based approach;
                        # its completion schedules another heading check.
                        self.ball_approach_alignment_pending = (
                            not completed_approach_turn
                        )
                    dwell_sec = (
                        max(
                            0.0,
                            float(self.BALL_POST_MOTION_DWELL_SEC),
                        )
                        if status == "SUCCEEDED" and completed_approach_turn
                        else 0.0
                    )
                    self.ball_post_motion_dwell_until = (
                        time.monotonic() + dwell_sec
                        if dwell_sec > 0.0
                        else None
                    )
                    if status == "SUCCEEDED" and dwell_sec <= 0.0:
                        MotionDecisionNode._invalidate_pickup_ball_input(self)
                        gate = self.general_motion_gate
                        gate.required_vision_generation = gate.vision_generation + 1
                    if pickup_entry_ready:
                        self.ball_approach_alignment_pending = False
                        MotionDecisionNode._invalidate_pickup_ball_input(self)
                        self.get_logger().info(
                            "BALL pickup entry pending; waiting for fresh vision"
                        )
                    if self.ball_post_motion_dwell_until is not None:
                        self.get_logger().info(
                            "BALL post-motion dwell started: "
                            f"{dwell_sec:.1f}s after action={action}"
                        )
                    if status == "SUCCEEDED" and completed_approach_turn:
                        self.ball_lost_during_motion_pending = False
                        self.ball_last_visible_approach_info = None
                        self.ball_last_visible_line_info = None
                        self.get_logger().info(
                            "BALL stationary correction complete; "
                            "waiting for post-motion dwell before the next "
                            "approach motion without "
                            "another same-boundary alignment check"
                        )
                if completed_source == "line":
                    post_ball_line_correction = str(action).startswith(
                        "POST_BALL_LINE_TURN_"
                    )
                    normalized_error_code = (
                        error_code.strip().upper()
                        if isinstance(error_code, str)
                        else ""
                    )
                    timed_out = bool(
                        status == "TIMEOUT"
                        or (
                            status == "FAILED"
                            and normalized_error_code == "TIMEOUT"
                        )
                    )
                    if status == "SUCCEEDED":
                        MotionDecisionNode._finish_line_motion_capture(self)
                        if str(action).startswith(("LINE_LOST_TURN_", "LINE_OFFSET_TURN_", "LINE_HEADING_TURN_", "LINE_SPARSE_", "RECOVER_")):
                            MotionDecisionNode._invalidate_post_ball_line_input(self)
                        if post_ball_line_correction:
                            self.post_ball_line_dwell_until = (
                                time.monotonic()
                                + MotionDecisionNode.POST_BALL_LINE_DWELL_SEC
                            )
                            MotionDecisionNode._invalidate_post_ball_line_input(
                                self
                            )
                        elif self.ball_approach_entry_pending:
                            self.ball_approach_alignment_pending = True
                            MotionDecisionNode._invalidate_pickup_ball_input(
                                self
                            )
                            gate = self.general_motion_gate
                            gate.required_vision_generation = gate.vision_generation + 1
                            self.get_logger().info(
                                "BALL entry after Line motion; waiting for fresh vision"
                            )
                        elif not str(action).startswith(
                            ("POST_BALL_LINE_TURN_", "LINE_LOST_TURN_", "RECOVER_")
                        ):
                            gate = self.general_motion_gate
                            gate.required_vision_generation = (
                                gate.vision_generation
                            )
                    elif timed_out and not post_ball_line_correction:
                        MotionDecisionNode._discard_line_motion_capture(self)
                        self.line_timeout_recovery_active = True
                        self.line_timeout_recovery_frames = []
                    else:
                        MotionDecisionNode._discard_line_motion_capture(self)
                    self.pending_line_decision = None
                self.get_logger().info(
                    "General motion lock released: "
                    f"status={status}, action={action}"
                )
            return

        if action not in self.SPECIAL_ACTIONS:
            return

        event_id_mismatch = (
            self.active_special_event_id is not None
            and event_id != self.active_special_event_id
        )
        if event_id_mismatch:
            self.get_logger().warning(
                "Special motion status ignored: "
                "event_id mismatch "
                f"(active={self.active_special_event_id}, "
                f"received={event_id})"
            )
            return

        completed_action = self.active_special_action
        completed_command_id = self.active_special_command_id
        result = self.phase_manager.handle_motion_status(
            action,
            command_id,
            status,
            motion_id=payload.get("motion_id"),
        )

        if not result.handled:
            self.get_logger().warning(
                "Special motion status ignored: "
                f"reason={result.reason}, status={status}, "
                f"action={action}, command_id={command_id}"
            )
            return

        self._latch_critical_executor_fault(
            error_code=error_code,
            message=status_message,
            action=action,
            command_id=command_id,
            event_id=event_id,
        )

        self.goal_stationary_since = (
            status_receive_monotonic if result.terminal and status == "SUCCEEDED" else None
        )
        if not result.terminal:
            if (
                action == "PICKUP_NOW"
                and status == "RUNNING"
                and payload.get("motion_id")
                == MotionDecisionNode.PICKUP_DWELL_MARKER
                and payload.get("completed_motion_id")
                == MotionDecisionNode.PICKUP_GRASP_CHECK_MOTION_ID
            ):
                if payload.get("verification_window_complete") is True:
                    MotionDecisionNode._finish_grasp_verification_window(self)
                elif (
                    payload.get("verification_window_complete") is False
                    and not getattr(self, "grasp_verification_active", False)
                ):
                    MotionDecisionNode._start_grasp_verification_window(
                        self,
                        motion_status_receive_monotonic=status_receive_monotonic,
                    )
            self.active_special_event_id = (
                event_id
                if isinstance(event_id, int)
                else self.active_special_event_id
            )
            self.active_special_dynamics_command = (
                dynamics_command
                if isinstance(dynamics_command, int)
                else None
            )
            if (
                action == "PICKUP_NOW"
                and status == "RUNNING"
                and payload.get("motion_id")
                == MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER
            ):
                self.pickup_initial_align_waiting = True
                self.pickup_fine_align_waiting = False
                self.pickup_post_backward_align_waiting = False
                self.pickup_positioning_motion_running = False
                self.pickup_positioning_motion_id = None
                self.pickup_positioning_ball_seen_during_motion = False
                MotionDecisionNode._invalidate_pickup_ball_input(self)
            elif (
                action == "PICKUP_NOW"
                and status == "RUNNING"
                and payload.get("motion_id")
                == MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER
            ):
                self.pickup_initial_align_waiting = False
                self.pickup_fine_align_waiting = True
                self.pickup_post_backward_align_waiting = False
                self.pickup_positioning_motion_running = False
                self.pickup_positioning_motion_id = None
                self.pickup_positioning_ball_seen_during_motion = False
                MotionDecisionNode._invalidate_pickup_ball_input(self)
            elif (
                action == "PICKUP_NOW"
                and status == "RUNNING"
                and payload.get("motion_id")
                == MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_MARKER
            ):
                self.pickup_initial_align_waiting = False
                self.pickup_fine_align_waiting = False
                self.pickup_post_backward_align_waiting = True
                self.pickup_positioning_motion_running = False
                self.pickup_positioning_motion_id = None
                self.pickup_positioning_ball_seen_during_motion = False
                self.pickup_fixed_sequence_started = True
                MotionDecisionNode._invalidate_pickup_ball_input(self)
            elif action == "PICKUP_NOW" and status == "RUNNING":
                motion_id = payload.get("motion_id")
                if motion_id == self.PICKUP_FIXED_SEQUENCE_FIRST_MOTION:
                    self.pickup_fixed_sequence_started = True
                positioning_motion = bool(
                    not self.pickup_fixed_sequence_started
                    and isinstance(motion_id, str)
                    and not motion_id.startswith("__")
                )
                if positioning_motion:
                    if motion_id != self.pickup_positioning_motion_id:
                        latest_ball = self.latest_info.get("ball")
                        self.pickup_positioning_ball_seen_during_motion = bool(
                            isinstance(latest_ball, dict)
                            and latest_ball.get("detected") is True
                        )
                    self.pickup_positioning_motion_running = True
                    self.pickup_positioning_motion_id = motion_id
                elif isinstance(motion_id, str) and motion_id.startswith("__"):
                    self.pickup_positioning_motion_running = False
                    self.pickup_positioning_motion_id = None
                    self.pickup_positioning_ball_seen_during_motion = False

            self.get_logger().info(
                "Special motion lock enabled: "
                f"action={self.active_special_action}, "
                f"command_id="
                f"{self.active_special_command_id}, "
                f"event_id="
                f"{self.active_special_event_id}, "
                f"dynamics_command="
                f"{self.active_special_dynamics_command}"
            )
            return

        completed_event_id = self.active_special_event_id

        if completed_action == "PICKUP_NOW":
            limiter = getattr(self, "lost_search_turn_limiter", None)
            if limiter is not None and status != "SUCCEEDED":
                limiter.record_failure("ball", "PICKUP_NOW")
            MotionDecisionNode._finish_grasp_verification_window(self)
            self.planner.pickup_close_alignment_active = False
            self.planner.pickup_last_visible_bottom_distance_px = None

        self.active_special_event_id = None
        self.active_special_dynamics_command = None
        self.pickup_initial_align_waiting = False
        self.pickup_fine_align_waiting = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_positioning_motion_running = False
        self.pickup_positioning_motion_id = None
        self.pickup_positioning_ball_seen_during_motion = False
        self.pickup_positioning_ball_lost_pending = False
        self.pickup_positioning_loss_latch_sent = False
        self.pickup_fixed_sequence_started = False

        if MotionDecisionNode._ball_navigation_blocked(self):
            MotionDecisionNode._clear_ball_navigation_state(self)

        if completed_action == "SHOT" and status == "SUCCEEDED":
            self.post_shot_turn_settled = False
            self.post_shot_dwell_until = (
                status_receive_monotonic + MotionDecisionNode.POST_SHOT_DWELL_SEC
            )
            self.pending_line_decision = None
            MotionDecisionNode._invalidate_post_ball_line_input(self)

        if completed_action == "PICKUP_NOW" and status == "SUCCEEDED":
            if self.pickups_completed >= self.required_pickups:
                self.planner.disable_completed_ball_missions()
            if self.mission_phase == "POST_BALL_LINE_ALIGN":
                self.line_ground_recovery_failed = False
                self.line_ground_recovery_active = False
                # Align from fresh Line geometry after the composite exit and dwell.
                self.planner.last_line_seen_direction = None
                self.planner.post_ball_line_search_direction = (
                    "RIGHT" if self.pickups_completed == 1 else "LEFT"
                )
                self.planner.line_planner.stop("pickup_complete")
                self.planner._clear_goal_tracking()
                self.planner.goal_lock_active = False
                self.planner.goal_terminal_requested = False
                self.post_ball_line_run_until = None
                self.post_ball_line_run_failed = False
                self.post_ball_line_dwell_until = None
                self.pending_line_decision = None
                MotionDecisionNode._invalidate_post_ball_line_input(self)
                gate = self.general_motion_gate
                gate.required_vision_generation = gate.vision_generation + 1

        if completed_action == "GO" and status == "SUCCEEDED":
            self.hurdle_positioning_entry_pending = False
            self.hurdle_post_motion_dwell_until = None
            self.hurdle_stationary_since = None
            MotionDecisionNode._clear_ball_navigation_state(self)
            MotionDecisionNode._invalidate_pickup_ball_input(self)
            self.planner._clear_goal_tracking()
            self.planner.goal_lock_active = False
            self.planner.goal_terminal_requested = False
            self.planner.goal_ignore_until_clear = True
            self.goal_post_motion_dwell_until = None
            MotionDecisionNode._invalidate_goal_input(self)
            self.planner.hurdle_planner.reset()
            self.planner.hurdle_lock_active = False
            self.planner.hurdle_go_requested = False
            self.planner.hurdle_ignore_until_clear = True
            self.planner.last_hurdle_path_reference = None
            self.planner.hurdle_path_reference_age_sec = 0.0
            if (
                self.phase_manager.hurdles_completed
                >= self.phase_manager.required_hurdles
            ):
                self.planner.disable_completed_hurdle_missions()
            self.planner.last_line_seen_direction = None
            self.planner.line_planner.stop("hurdle_complete")
            self.pending_line_decision = None
            MotionDecisionNode._invalidate_post_ball_line_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1

        if (
            completed_action == "POST_BALL_GOAL_TRANSITION"
            and status == "SUCCEEDED"
            and self.mission_phase == "GOAL_APPROACH"
        ):
            # The bridge already waited while raising the camera. Only a new
            # goal observation after that completed pause may start approach.
            self.goal_post_motion_dwell_until = None
            MotionDecisionNode._invalidate_goal_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1

        self.get_logger().info(
            "Special motion lock released: "
            f"status={status}, "
            f"action={completed_action}, "
            f"command_id={completed_command_id}, "
            f"event_id={completed_event_id}, "
            f"error_code={error_code}, "
            f"motion_id={payload.get('motion_id')}, "
            f"message={status_message}"
        )

        if completed_action == "PICKUP_NOW" and status in {
            "REJECTED", "FAILED", "TIMEOUT",
        }:
            # A failed attempt did not consume the visible ball. Retry only
            # after fresh vision, subject to the existing failure limit.
            self.terminal_action_armed["ball"] = True
            self.ball_pickup_entry_pending = False
            MotionDecisionNode._invalidate_pickup_ball_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1

        if completed_action == "CROSS_FINISH" and status != "SUCCEEDED":
            self.terminal_action_armed["finish"] = True
        self.terminal_latch = None

        self.get_logger().info(
            "Mission phase advanced after special motion: "
            f"status={status}, "
            f"action={completed_action}, "
            f"previous_phase={result.previous_phase}, "
            f"next_phase={self.mission_phase}"
        )

    def _latch_critical_executor_fault(
        self,
        *,
        error_code: Any,
        message: Any,
        action: Any,
        command_id: Any,
        event_id: Any,
    ) -> None:
        """Latch one correlated critical executor/runtime fault."""
        if self.safety_interlock.observe_executor_status(
            error_code=error_code,
            message=message,
            action=action,
            command_id=command_id,
            event_id=event_id,
        ):
            snapshot = self.safety_interlock.snapshot
            self.get_logger().warning(
                "Critical executor fault latched; all motion publication "
                f"is blocked until node restart: error_code="
                f"{snapshot.error_code}, action={snapshot.action}, "
                f"command_id={snapshot.command_id}"
            )

    def _mission_progress(self) -> dict[str, int | bool | str]:
        """Return execution budgets, completion counts and course progress."""
        snapshot = self.phase_manager.snapshot()
        progress = {
            key: snapshot[key]
            for key in (
                "pickups_executed",
                "shots_executed",
                "hurdles_executed",
                "pickups_completed",
                "required_pickups",
                "shots_completed",
                "required_shots",
                "hurdles_completed",
                "required_hurdles",
                "ball_sections_processed",
                "required_ball_sections",
                "finish_enabled",
                "mission_complete",
                "ball_mode_active",
            )
        }
        for ball_index in range(1, self.required_pickups + 1):
            progress[f"ball_{ball_index}_grasp_result"] = (
                self.phase_manager.grasp_result_for_ball(ball_index)
            )
        return progress

    def _invalidate_post_ball_line_input(self) -> None:
        """Discard Line observations from before the current checkpoint."""
        latest_info = getattr(self, "latest_info", None)
        latest_time = getattr(self, "latest_time", None)
        if isinstance(latest_info, dict):
            latest_info["line"] = None
        if isinstance(latest_time, dict):
            latest_time["line"] = None
        observations = getattr(self, "observations", None)
        if isinstance(observations, dict):
            observations["line"] = None
        published_stamps = getattr(self, "last_published_vision_stamp", None)
        if isinstance(published_stamps, dict):
            published_stamps.pop("line", None)

    def _invalidate_pickup_ball_input(self) -> None:
        """Require a Ball frame captured after the pickup checkpoint."""
        self.ball_min_rgb_stamp_ns = MotionDecisionNode._current_ros_time_ns(self)
        self.ball_prepared_info = None
        self.ball_prepared_observation = None
        self.ball_prepared_alignment = None
        latest_info = getattr(self, "latest_info", None)
        latest_time = getattr(self, "latest_time", None)
        if isinstance(latest_info, dict):
            latest_info["ball"] = None
        if isinstance(latest_time, dict):
            latest_time["ball"] = None
        observations = getattr(self, "observations", None)
        if isinstance(observations, dict):
            observations["ball"] = None
        published_stamps = getattr(self, "last_published_vision_stamp", None)
        if isinstance(published_stamps, dict):
            published_stamps.pop("ball", None)

    def _invalidate_goal_input(self) -> None:
        """Require a new image after motion/dwell; motion time is not loss time."""
        self.latest_info["goal"] = None
        self.latest_time["goal"] = None
        self.goal_min_rgb_stamp_ns = MotionDecisionNode._current_ros_time_ns(self)
        reset = getattr(self.planner, "reset_goal_loss_timer", None)
        if reset is not None:
            reset()

    def _fresh_observations(
        self,
        now: float,
    ) -> tuple[
        dict[str, dict[str, Any] | None],
        dict[str, float | None],
    ]:
        observations: dict[
            str,
            dict[str, Any] | None,
        ] = {}

        ages: dict[str, float | None] = {}

        for source in self.SOURCES:
            stamp = self.latest_time[source]
            age = (
                now - stamp
                if stamp is not None
                else None
            )

            ages[source] = (
                round(age, 3)
                if age is not None
                else None
            )

            observations[source] = (
                self.latest_info[source]
                if (
                    age is not None
                    and age <= self.timeouts[source]
                )
                else None
            )

        return observations, ages

    def _recover_invalid_line_ground(
        self, decision: MotionDecision, info: dict[str, Any] | None, now: float,
    ) -> MotionDecision:
        """Reobserve a visible invalid ground fit after one calibrated small turn."""
        config = getattr(getattr(self.planner, "line_planner", None), "config", None)
        if (
            self.mission_phase not in {
                "AUTO", "LINE_TRACK", "LINE_TRACK_AFTER_PICKUP", "POST_BALL_LINE_ALIGN",
            }
            or decision.source != "line"
            or getattr(config, "heading_source", None) != "ground"
        ):
            return decision
        failed = getattr(self, "line_ground_recovery_failed", False)
        recovering = getattr(self, "line_ground_recovery_active", False)
        hold = replace(
            decision, action="WAIT", valid=False, sdk_motion_requested=False, requires_ack=False,
            reason=("line_ground_recovery_motion_failed" if failed
                    else "line_ground_recovery_waiting_for_fresh_full_fit"),
            source_command={**decision.source_command, "valid": False, "motion": "STOP"},
        )
        fallback = hold if failed else decision
        if info is None:
            # Do not turn a missing post-recovery frame into a larger blind search.
            return hold if recovering or failed else decision
        if (
            info.get("detected") is not True
            or info.get("ground_projection_enabled") is not True
            or self.general_motion_gate.locked
            or self.active_special_command_id is not None
            or getattr(self, "executor_active", None) is not False
            or self.mission_complete
        ):
            return fallback
        stamp = MotionDecisionNode._detection_stamp_ns(info)
        if stamp is None:
            stamp = info.get("rgb_stamp_ns")
        ros_now = MotionDecisionNode._current_ros_time_ns(self)
        received_at = self.latest_time.get("line")
        boundaries = [getattr(self, name, None) for name in (
            "line_offset_min_rgb_stamp_ns", "line_motion_min_rgb_stamp_ns",
        )]
        if (
            type(stamp) is not int or stamp <= 0 or type(ros_now) is not int
            or not 0 <= ros_now - stamp <= int(self.timeouts["line"] * 1e9)
            or received_at is None or not 0 <= now - received_at <= self.timeouts["line"]
            or any(boundary is not None and stamp <= boundary for boundary in boundaries)
        ):
            return hold if recovering or failed else decision
        qualities = [MotionDecisionPlanner._number(info, key) for key in (
            "heading_quality", "geometry_quality", "detection_quality",
        )]
        if any(q is None or q < config.min_line_quality for q in qualities):
            return hold if recovering or failed else decision
        if line_heading(info, "ground") is not None:
            if (MotionDecisionPlanner._number(info, "ground_fit_point_count") or 0) >= 3:
                self.line_ground_recovery_failed = False
                self.line_ground_recovery_active = False
                return decision
            return hold if recovering or failed else decision
        if (
            decision.valid or decision.reason not in {
                "invalid_ground_line_geometry", "invalid_post_ball_line_alignment_input",
                "line_corner_waiting_for_usable_line",
            }
            or info.get("ground_projection_valid") is not False
            or info.get("ground_fit_reason") not in {
                "too_few_segment_points", "too_few_projected_points", "too_few_inliers",
                "degenerate_forward_span", "fit_did_not_converge",
            }
            or all(MotionDecisionPlanner._number(info, key) is None for key in (
                "filtered_lateral_offset_norm", "lateral_offset_norm",
            ))
        ):
            return fallback
        if failed:
            return hold
        pickup_exit = self.mission_phase == "POST_BALL_LINE_ALIGN"
        direction = (
            self.planner.post_ball_line_search_direction if pickup_exit
            else self.planner.last_line_seen_direction
        )
        if direction not in {"LEFT", "RIGHT"}:
            return decision
        count = 2 if direction == "RIGHT" else 1
        prefix = "POST_BALL_LINE" if pickup_exit else "LINE_HEADING"
        action = f"{prefix}_TURN_{direction}_{count}"
        return replace(
            decision, action=action, valid=True, reason="line_ground_fit_recovery_turn",
            sdk_motion_requested=False, requires_ack=False,
            source_command={
                **decision.source_command, "valid": True, "motion": action,
                "reason": "line_ground_fit_recovery_turn", "invalid_ground_recovery": True,
                "turn_direction": direction, "turn_count": count,
                "turn_angle_deg": self.planner._turn_angle_deg(count, direction),
                "direction_source": "pickup_exit_turn" if pickup_exit else "last_seen_line",
                "ground_fit_reason": info.get("ground_fit_reason"),
                "ground_fit_input_point_count": info.get("ground_fit_input_point_count"),
                "catalog_motion_available": True,
                "linear_speed_mps": 0.0, "lateral_speed_mps": 0.0,
                "angular_speed_rad_s": 0.0, "travel_distance_m": 0.0,
                "lateral_travel_distance_m": 0.0,
                "target_heading_change_deg": 15.0 if direction == "RIGHT" else -15.0,
            },
        )

    def _remember_line_corner(self, info: dict[str, Any]) -> None:
        """Remember the first confirmed direction and refresh its observed distance."""
        if (
            self.mission_phase not in {"AUTO", "LINE_TRACK"}
            or self.active_special_command_id is not None
            or self.active_general_source not in {None, "line"}
        ):
            self.pending_line_corner = None
            self.line_corner_rearm_required = False
            return
        gate = self.general_motion_gate
        if gate.locked and gate.active_action in {"LEFT", "RIGHT"}:
            return
        if info.get("detected") is not True:
            return
        qualities = [MotionDecisionPlanner._number(info, key) for key in (
            "heading_quality", "geometry_quality", "detection_quality",
        )]
        qualities = [value for value in qualities if value is not None]
        if not qualities or min(qualities) < self.planner.line_planner.config.min_line_quality:
            return
        if getattr(self, "line_corner_rearm_required", False):
            # Loss does not prove the previous corner has been passed.
            if info.get("corner_preview_confirmed") is False:
                self.line_corner_rearm_required = False
            return
        if (
            info.get("corner_preview_confirmed") is not True
            or info.get("corner_direction") not in {"LEFT", "RIGHT"}
        ):
            return
        corner = getattr(self, "pending_line_corner", None)
        if corner is not None:
            if info["corner_direction"] == corner["corner_direction"]:
                corner["corner_start_distance_m"] = MotionDecisionPlanner._number(
                    info, "corner_start_distance_m",
                )
            return
        self.pending_line_corner = {
            "corner_direction": info["corner_direction"],
            "corner_start_distance_m": MotionDecisionPlanner._number(
                info, "corner_start_distance_m",
            ),
            "observed_during_action": gate.active_action,
            "observed_during_command_id": gate.active_command_id,
            "motion_completed_at": getattr(self, "line_motion_completed_at", None),
            "minimum_rgb_stamp_ns": getattr(self, "line_motion_min_rgb_stamp_ns", None),
        }
        self.pending_line_decision = None
        self.get_logger().info(
            f"Line corner remembered: {self.pending_line_corner}"
        )

    def _apply_pending_line_corner(
        self, decision: MotionDecision, info: dict[str, Any] | None = None,
    ) -> MotionDecision:
        """Gate an already selected corner turn without replacing normal line tracking."""
        if (
            self.mission_phase not in {"AUTO", "LINE_TRACK"}
            or self.active_special_command_id is not None
            or decision.source not in {"line", "none"}
            or self.mission_complete
        ):
            self.pending_line_corner = None
            self.line_corner_rearm_required = False
            return decision
        if decision.reason == "invalid_vision_boolean_type":
            return decision
        # Also guard corners first confirmed while stationary, before the planner's
        # ordinary LEFT/RIGHT can bypass the distance check.
        if info is not None:
            MotionDecisionNode._remember_line_corner(self, info)
        corner = getattr(self, "pending_line_corner", None)
        if corner is None:
            return decision
        distance = MotionDecisionPlanner._number(
            info or {}, "corner_start_distance_m",
        )
        received_at = self.latest_time.get("line")
        completed_at = corner.get("motion_completed_at")
        minimum_stamp = corner.get("minimum_rgb_stamp_ns")
        stamp = MotionDecisionNode._detection_stamp_ns(info or {})
        if stamp is None:
            stamp = (info or {}).get("rgb_stamp_ns")
        ros_now = MotionDecisionNode._current_ros_time_ns(self)
        valid_stamp = (
            isinstance(stamp, int) and not isinstance(stamp, bool) and stamp > 0
        )
        fresh_frame = bool(
            info is not None
            and received_at is not None
            and 0.0 <= time.monotonic() - received_at <= self.timeouts["line"]
            and (completed_at is None or received_at > completed_at)
            and (ros_now is None or (
                valid_stamp
                and 0 <= ros_now - stamp <= int(self.timeouts["line"] * 1e9)
            ))
            and (minimum_stamp is None or (valid_stamp and stamp > minimum_stamp))
        )
        if (
            fresh_frame and valid_stamp and info.get("detected") is False
            and decision.valid
            and decision.action.startswith("LINE_LOST_TURN_")
        ):
            # Preserve corner memory, but let the normal bounded search reacquire Line.
            # The publish path still enforces motion gates, settle time and search budget.
            return replace(decision, source_command={
                **decision.source_command, "corner_search_pending": True,
                "remembered_corner_direction": corner["corner_direction"],
            })
        metadata = {**decision.source_command, **corner, "corner_from_memory": True}
        if not fresh_frame or not MotionDecisionNode._line_frame_is_usable(self, info):
            # A visible line can still be rejected by capture-time checks.
            # Preserve the values needed to distinguish occlusion from clock skew.
            metadata["line_input_checks"] = {
                "received_age_sec": (
                    round(time.monotonic() - received_at, 6)
                    if received_at is not None else None
                ),
                "timeout_sec": self.timeouts["line"],
                "received_after_motion": (
                    completed_at is None
                    or (received_at is not None and received_at > completed_at)
                ),
                "capture_stamp_valid": valid_stamp,
                "capture_age_sec": (
                    round((ros_now - stamp) / 1e9, 6)
                    if ros_now is not None and valid_stamp else None
                ),
                "captured_after_motion": (
                    minimum_stamp is None or (valid_stamp and stamp > minimum_stamp)
                ),
                "geometry_usable": MotionDecisionNode._line_frame_is_usable(self, info or {}),
            }
            return MotionDecision(
                phase=self.mission_phase, source="line", action="WAIT", valid=False,
                reason="line_corner_waiting_for_usable_line",
                sdk_motion_requested=False, requires_ack=False, source_command=metadata,
            )
        # Remembering a bend must not replace normal tracking or local recovery.
        # Only a confirmed LEFT/RIGHT candidate needs the additional distance gate.
        if not decision.valid or decision.action not in {"LEFT", "RIGHT"}:
            return decision
        fresh_distance = bool(
            info.get("corner_preview_confirmed") is True
            and info.get("corner_preview_held") is not True
            and info.get("corner_start_depth_valid") is True
            and info.get("corner_direction") == corner["corner_direction"] == decision.action
            and distance is not None and distance > 0.0
        )
        if not fresh_distance:
            return MotionDecision(
                phase=self.mission_phase, source="line", action="WAIT", valid=False,
                reason="line_corner_waiting_for_fresh_distance",
                sdk_motion_requested=False, requires_ack=False, source_command=metadata,
            )
        turn_distance = getattr(
            self, "line_corner_turn_distance_m",
            MotionDecisionNode.LINE_CORNER_TURN_DISTANCE_M,
        )
        metadata.update(
            corner_start_distance_m=distance, corner_turn_distance_m=turn_distance,
        )
        if distance > turn_distance:
            if not straight_heading_is_aligned(info, self.planner.line_planner.config):
                return replace(
                    decision, action="WAIT", valid=False,
                    reason="line_corner_approach_heading_not_aligned",
                    sdk_motion_requested=False, requires_ack=False,
                    source_command={**metadata, "valid": False, "motion": "STOP"},
                )
            # A far bend may postpone a turn, but cannot command a long blind walk.
            return replace(
                decision, action="STRAIGHT_1", reason="line_corner_turn_too_far",
                source_command={**metadata, "corner_prepare": True,
                                "approach_motion": "STRAIGHT_1"},
            )
        return replace(decision, source_command=metadata)

    def _start_line_motion_capture(self, action: str) -> None:
        """Start the configured late-motion Vision capture window."""
        # Recovery is checked only after completion, never queued from moving frames.
        config = (
            None if action.startswith("RECOVER_")
            else MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG.get(action)
        )
        self.pending_line_decision = None
        self.active_line_motion_frames = []
        if config is None:
            self.active_line_motion_action = None
            self.active_line_motion_started_at = None
            self.active_line_motion_duration_sec = 0.0
            self.active_line_motion_target_frames = 0
            return
        duration_sec, target_frames = config
        self.active_line_motion_action = action
        self.active_line_motion_started_at = time.monotonic()
        self.active_line_motion_duration_sec = duration_sec
        self.active_line_motion_target_frames = target_frames

    def _collect_active_line_motion_frame(
        self,
        payload: dict[str, Any],
        received_at: float,
    ) -> bool:
        """Keep recent usable line frames and finalize after 70 percent."""
        started_at = self.active_line_motion_started_at
        if (
            started_at is None
            or getattr(self, "pending_line_decision", None) is not None
        ):
            return False
        if MotionDecisionNode._line_frame_is_usable(self, payload):
            self.active_line_motion_frames.append(dict(payload))
            del self.active_line_motion_frames[
                : -self.active_line_motion_target_frames
            ]
        capture_start = (
            started_at
            + self.active_line_motion_duration_sec
            * MotionDecisionNode.LINE_MOTION_CAPTURE_START_RATIO
        )
        if received_at < capture_start:
            return False
        return (
            len(self.active_line_motion_frames)
            >= self.active_line_motion_target_frames
        )

    def _line_frame_is_usable(self, payload: dict[str, Any]) -> bool:
        """Match the line planner's minimum executable input checks."""
        if payload.get("detected") is not True:
            return False

        def finite_number(*keys: str) -> float | None:
            for key in keys:
                value = payload.get(key)
                if value is None or isinstance(value, bool):
                    continue
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(number):
                    return number
            return None

        if line_heading(payload, getattr(self.planner.line_planner.config, "heading_source", "ground")) is None:
            return False
        if finite_number(
            "filtered_lateral_offset_norm",
            "lateral_offset_norm",
        ) is None:
            return False

        qualities = [
            finite_number(key)
            for key in (
                "heading_quality",
                "geometry_quality",
                "detection_quality",
            )
        ]
        valid_qualities = [value for value in qualities if value is not None]
        if not valid_qualities:
            return False
        minimum_quality = self.planner.line_planner.config.min_line_quality
        return min(valid_qualities) >= minimum_quality

    def _prepare_pending_line_decision(self, now: float) -> None:
        """Precompute the next decision from late-motion line frames."""
        if self.mission_phase == "LINE_TRACK_AFTER_PICKUP":
            # Check the ten-second boundary between motions, without a queued step.
            return
        frames = self.active_line_motion_frames
        if not frames:
            return
        line_planner = self.planner.line_planner
        line_planner._reset_turn_state()
        for frame in frames:
            line_planner.plan(frame, 1.0 / 30.0)
        self.active_line_motion_frames = []

        observations, _ = self._fresh_observations(now)
        observations = dict(observations)
        latest_line = observations.get("line")
        if latest_line is None or not MotionDecisionNode._line_frame_is_usable(
            self, latest_line,
        ):
            return
        observations["line"] = frames[-1]
        decision = self._select_mission_decision(
            observations,
            1.0 / 30.0,
        )
        if decision.source == "line":
            self.planner.observe_line_for_search(latest_line)
        decision = self._suppress_duplicate_terminal_action(decision)
        decision = self._suppress_exhausted_special_action(decision)
        if decision.valid:
            self.pending_line_decision = decision
            if decision.action.startswith("RECOVER_"):
                return
            # Only a second straight motion is safe to queue without a stop.
            # Turns and special actions retain the normal motion boundary so
            # late Ball observations can still take priority.
            if (
                decision.source == "line"
                and decision.action.startswith("STRAIGHT")
                and MotionDecisionNode._line_only_prequeue_allowed(
                    self,
                    observations,
                )
            ):
                MotionDecisionNode._publish_decision(
                    self,
                    decision,
                    queue_while_locked=True,
                )

    @staticmethod
    def _needs_correction_dwell(action: str | None) -> bool:
        """Hold after turns, fine steps, crab steps and retreats in every mission."""
        if action is None or action.startswith("RECOVER_"):
            return False
        if action.startswith("LINE_SPARSE_"):
            return True
        return action in {
            "LEFT", "RIGHT", "TURN_LEFT", "TURN_RIGHT", "ALIGN_LEFT", "ALIGN_RIGHT",
            "STRAIGHT_0", "SLOW_APPROACH", "FINE_FORWARD_STEP", "RETREAT_GOAL",
        } or any(part in action for part in (
            "TURN_", "FINE_FORWARD", "CRAB_", "BACKWARD_",
        ))

    def _line_only_prequeue_allowed(
        self,
        observations: dict[str, dict[str, Any] | None],
    ) -> bool:
        """Allow seamless STRAIGHT only before another mission is detected."""
        if getattr(self, "pending_line_corner", None) is not None:
            return False
        if MotionDecisionNode._needs_correction_dwell(self.general_motion_gate.active_action):
            return False
        if (
            getattr(self, "ball_approach_entry_pending", False)
            or getattr(self, "hurdle_positioning_entry_pending", False)
            or self.mission_phase == "HURDLE_POSITIONING"
            or getattr(self, "ball_confirmation_pending_latched", False)
            or getattr(self, "active_special_command_id", None) is not None
        ):
            return False
        for source in ("ball", "goal", "hurdle", "finish"):
            if source == "ball" and MotionDecisionNode._ball_navigation_blocked(self):
                continue
            if (
                source == "hurdle"
                and self.phase_manager.hurdles_completed
                >= self.phase_manager.required_hurdles
            ):
                continue
            info = observations.get(source)
            if source == "hurdle" and info is not None:
                depth = MotionDecisionPlanner._number(info, "depth_m")
                if (
                    info.get("depth_valid") is True
                    and depth is not None and depth > self.planner.config.hurdle_control_range_m
                    and not self.planner._hurdle_positioning_ready(info)
                ):
                    continue
            if info is not None and (
                info.get("detected") is True
                or info.get("raw_detected") is True
            ):
                return False
        return True

    def _finish_line_motion_capture(self) -> None:
        """Replay captured frames into the existing line planner."""
        frames = self.active_line_motion_frames
        if frames:
            line_planner = self.planner.line_planner
            line_planner._reset_turn_state()
            for frame in frames:
                line_planner.plan(frame, 1.0 / 30.0)
        MotionDecisionNode._discard_line_motion_capture(self)

    def _discard_line_motion_capture(self) -> None:
        """Clear late-motion frames without applying them to the planner."""
        self.active_line_motion_action = None
        self.active_line_motion_started_at = None
        self.active_line_motion_duration_sec = 0.0
        self.active_line_motion_target_frames = 0
        self.active_line_motion_frames = []

    def _collect_timeout_recovery_frame(
        self,
        payload: dict[str, Any],
    ) -> None:
        """Re-evaluate a timed-out line motion from ten new valid frames."""
        if payload.get("detected") is not True:
            return
        self.line_timeout_recovery_frames.append(dict(payload))
        if (
            len(self.line_timeout_recovery_frames)
            < MotionDecisionNode.LINE_TIMEOUT_RECOVERY_FRAMES
        ):
            return

        line_planner = self.planner.line_planner
        line_planner._reset_turn_state()
        for frame in self.line_timeout_recovery_frames:
            line_planner.plan(frame, 1.0 / 30.0)
        self.line_timeout_recovery_frames = []
        self.line_timeout_recovery_active = False
        self.general_motion_gate.rejected_action = None

    def _publish_decision(
        self,
        precomputed_decision: MotionDecision | None = None,
        queue_while_locked: bool = False,
    ) -> None:
        MotionDecisionNode._update_goal_stationary_time(self, time.monotonic())
        if not self.decision_started:
            MotionDecisionNode._poll_start_enter(self)
            return
        if self.safety_interlock.latched:
            return
        if queue_while_locked and self.mission_phase == "LINE_TRACK_AFTER_PICKUP":
            return

        if not self.executor_heartbeat_watchdog.executor_seen:
            # Do not race executor preflight/startup pose with navigation.
            self._reset_pre_motion_settle()
            return

        if not self.executor_auto_ready:
            # Startup HOLD observations are intentionally not queued.
            self._reset_pre_motion_settle()
            return

        if self.executor_ready_requires_fresh_vision:
            self._reset_pre_motion_settle()
            return

        if self.general_motion_gate.locked and not queue_while_locked:
            self._reset_pre_motion_settle()
            return
        if queue_while_locked and getattr(self, "pending_line_corner", None) is not None:
            return

        if (
            getattr(self, "queued_general_command_id", None) is not None
            and not queue_while_locked
        ):
            self._reset_pre_motion_settle()
            return

        if self.line_timeout_recovery_active:
            self._reset_pre_motion_settle()
            return

        now = time.monotonic()
        dwell_until = getattr(self, "correction_post_motion_dwell_until", None)
        if dwell_until is not None:
            if now < dwell_until:
                self._reset_pre_motion_settle()
                return
            self.correction_post_motion_dwell_until = None
            source = getattr(self, "correction_post_motion_source", None)
            if source == "line" and getattr(self, "line_offset_alignment_refresh_pending", False):
                self.line_offset_min_rgb_stamp_ns = MotionDecisionNode._current_ros_time_ns(self)
                self.line_offset_alignment_refresh_pending = False
            if source in self.latest_info:
                self.latest_info[source] = None
                self.latest_time[source] = None
            self.pending_line_decision = None
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1
            self._reset_pre_motion_settle()
            return
        dwell_until = getattr(self, "hurdle_post_motion_dwell_until", None)
        if dwell_until is not None:
            if now < dwell_until:
                self._reset_pre_motion_settle()
                return
            self.hurdle_post_motion_dwell_until = None
            self.latest_info["hurdle"] = None
            self.latest_time["hurdle"] = None
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1
            self._reset_pre_motion_settle()
            return
        if self.mission_phase == "LINE_TRACK_AFTER_PICKUP":
            if getattr(self, "post_ball_line_run_failed", False):
                self._reset_pre_motion_settle()
                return
            if MotionDecisionNode._advance_post_ball_line_run(self, now):
                precomputed_decision = None

        if MotionDecisionNode._ball_navigation_blocked(self):
            # Clear obsolete Ball pauses before evaluating route-specific dwells.
            MotionDecisionNode._clear_ball_navigation_state(self)

        dwell_until = getattr(self, "post_shot_dwell_until", None)
        if dwell_until is not None:
            if now < dwell_until:
                self._reset_pre_motion_settle()
                return
            self.post_shot_dwell_until = None
            if self.mission_phase == "POST_SHOT_LINE_ALIGN":
                self.post_shot_turn_settled = True
            MotionDecisionNode._invalidate_post_ball_line_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1
            self._reset_pre_motion_settle()
            return

        dwell_until = getattr(self, "goal_post_motion_dwell_until", None)
        if dwell_until is not None:
            if now < dwell_until:
                self._reset_pre_motion_settle()
                return
            self.goal_post_motion_dwell_until = None
            # Only an image captured after the stationary pause may act.
            MotionDecisionNode._invalidate_goal_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1
            self._reset_pre_motion_settle()
            return

        dwell_until = getattr(self, "post_ball_line_dwell_until", None)
        if dwell_until is not None:
            if now < dwell_until:
                self._reset_pre_motion_settle()
                return
            self.post_ball_line_dwell_until = None
            if getattr(self, "line_offset_alignment_refresh_pending", False):
                self.line_offset_min_rgb_stamp_ns = MotionDecisionNode._current_ros_time_ns(self)
                self.line_offset_alignment_refresh_pending = False
            MotionDecisionNode._invalidate_post_ball_line_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1
            self._reset_pre_motion_settle()
            return

        dwell_until = self.ball_post_motion_dwell_until
        if dwell_until is not None:
            if now < dwell_until:
                self._reset_pre_motion_settle()
                return
            self.ball_post_motion_dwell_until = None
            MotionDecisionNode._invalidate_pickup_ball_input(self)
            gate = self.general_motion_gate
            gate.required_vision_generation = gate.vision_generation + 1
            self._reset_pre_motion_settle()
            return

        dt_sec = max(
            1e-3,
            now - self.previous_publish_time,
        )

        self.previous_publish_time = now

        observations, ages = self._fresh_observations(
            now
        )

        sparse = getattr(self, "sparse_line_recovery", None)
        if sparse is None:
            sparse = self.sparse_line_recovery = SparseLineRecovery()
        line_info = observations.get("line")
        stamp = MotionDecisionNode._detection_stamp_ns(line_info or {})
        if stamp is None:
            stamp = (line_info or {}).get("rgb_stamp_ns")
        line_config = getattr(getattr(self.planner, "line_planner", None), "config", None)
        sparse_allowed = (
            self.mission_phase in {"AUTO", "LINE_TRACK"}
            and getattr(line_config, "heading_source", None) == "ground"
            and not queue_while_locked and not self.general_motion_gate.locked
            and self.active_special_command_id is None
            and not getattr(self, "executor_active", False)
            and not self.mission_complete
        )
        observations["line"] = sparse.prepare(
            line_info, stamp=stamp, ros_now=MotionDecisionNode._current_ros_time_ns(self),
            now=now, max_age=getattr(self, "timeouts", {}).get("line", 0.),
            # Visible invalid ground fits now recover by turning, never by
            # promoting a two-point estimate into a normal driving heading.
            stationary=(sparse_allowed and not (
                line_info is not None and line_info.get("ground_projection_valid") is False
            )),
            min_quality=getattr(line_config, "min_line_quality", 1.),
        )

        self._rearm_absent_terminal_targets(observations)

        decision = precomputed_decision
        if decision is None:
            decision = self._select_mission_decision(
                observations,
                dt_sec,
            )
        decision = MotionDecisionNode._apply_pending_line_corner(
            self, decision, observations.get("line"),
        )
        decision = sparse.constrain(decision, observations.get("line"), now)
        decision = MotionDecisionNode._recover_invalid_line_ground(self, decision, line_info, now)
        self.last_candidate_decision = decision

        decision = self._suppress_duplicate_terminal_action(
            decision
        )

        decision = self._suppress_exhausted_special_action(
            decision
        )
        decision = MotionDecisionNode._suppress_unverified_shot(
            self,
            decision,
        )
        limiter = getattr(self, "lost_search_turn_limiter", None)
        if limiter is None:
            limiter = self.lost_search_turn_limiter = LostSearchTurnLimiter(
                staged_sources=("line", "ball", "goal"),
            )
        decision = limiter.filter(decision, observations)
        self.last_selected_decision = decision

        if not self._command_publisher_has_subscriber():
            return

        if decision.action == "POST_SHOT_LINE_ALIGNED":
            if (
                not decision.valid
                or not self.general_motion_gate.has_required_fresh_vision()
                or MotionDecisionNode._same_vision_frame_was_published(self, decision)
            ):
                return
            if not self.phase_manager.complete_post_shot_line_align():
                return
            if not getattr(self, "post_shot_turn_settled", False):
                # Also protect direct/manual entry without a completed turn pause.
                self.post_shot_dwell_until = now + MotionDecisionNode.POST_SHOT_DWELL_SEC
                MotionDecisionNode._invalidate_post_ball_line_input(self)
                self.get_logger().info(
                    "Post-shot Line heading aligned; holding 3 seconds before forward"
                )
                return
            # The last turn already had its one-second pause. Use the fresh
            # aligned sample now instead of adding a second pause.
            self.post_shot_turn_settled = False
            decision = self._select_mission_decision(observations, dt_sec)
            self.last_candidate_decision = decision
            decision = limiter.filter(decision, observations)
            self.last_selected_decision = decision

        if decision.action == "POST_BALL_LINE_ALIGNED":
            if (
                not self.general_motion_gate.has_required_fresh_vision()
                or MotionDecisionNode._same_vision_frame_was_published(
                    self,
                    decision,
                )
            ):
                self._reset_pre_motion_settle()
                return
            if not self.phase_manager.complete_post_ball_line_align():
                self._reset_pre_motion_settle()
                return
            self.post_ball_line_run_until = None
            self.post_ball_line_run_failed = False
            self.planner._clear_goal_tracking()
            self.planner.goal_lock_active = False
            self.planner.goal_terminal_requested = False
            decision = self._select_mission_decision(observations, dt_sec)
            self.last_candidate_decision = decision
            decision = limiter.filter(decision, observations)
            self.last_selected_decision = decision

        if (
            decision.valid
            and self.general_motion_gate.locked
            and not queue_while_locked
        ):
            # A RUNNING general request owns the executor regardless of what
            # a newer Vision frame would otherwise select.
            self._reset_pre_motion_settle()
            return

        # Only fixed post-pickup search may proceed without a Line sample.
        # Motion locks, executor readiness, failure blocks and dwell still apply.
        missing_line_search = bool(
            self.mission_phase == "POST_BALL_LINE_ALIGN"
            and observations.get("line") is None
            and decision.action.startswith("POST_BALL_LINE_TURN_")
            and decision.source_command.get("lost_search_stage") in {"INITIAL", "EXTEND", "REVERSE"}
        )

        reserved_hurdle_sequence = bool(
            self.mission_phase == "HURDLE_POSITIONING"
            and decision.source == "hurdle" and decision.action == "GO"
            and decision.source_command.get("hurdle_final_sequence_latched") is True
            and getattr(self, "pending_hurdle_final_sequence", None) is not None
        )
        if (
            decision.valid
            and not missing_line_search
            and not reserved_hurdle_sequence
            and not self.general_motion_gate.has_required_fresh_vision()
        ):
            # An already confirmed hurdle checkpoint may finish its fixed sequence
            # without another frame. Other executable decisions need new Vision.
            self._reset_pre_motion_settle()
            return

        is_general_motion = (
            decision.valid
            and normalize_general_action(decision.action) is not None
        )
        is_pickup_initial_align_action = (
            decision.valid
            and decision.action in self.PICKUP_INITIAL_ALIGN_ACTIONS
        )
        is_pickup_fine_align_action = (
            decision.valid
            and decision.action in self.PICKUP_FINE_ALIGN_ACTIONS
        )
        is_pickup_post_backward_align_action = (
            decision.valid
            and decision.action in self.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
        )
        is_pickup_positioning_loss_latch = bool(
            decision.valid
            and decision.action
            == self.PICKUP_POSITIONING_LOSS_LATCH_ACTION
        )
        is_pickup_checkpoint_action = bool(
            is_pickup_initial_align_action
            or is_pickup_fine_align_action
            or is_pickup_post_backward_align_action
            or is_pickup_positioning_loss_latch
        )
        if (
            is_general_motion
            and not queue_while_locked
            and not self.general_motion_gate.can_publish(
                decision.action, allow_missing_line_search=missing_line_search,
            )
        ):
            # Keep receiving Vision and running the planner, but do not publish
            # another executable command while the current motion is locked.
            self._reset_pre_motion_settle()
            return

        if (
            is_pickup_checkpoint_action
            and MotionDecisionNode._same_vision_frame_was_published(
                self,
                decision,
            )
        ):
            self._reset_pre_motion_settle()
            return

        if (
            is_general_motion
            and not missing_line_search
            and MotionDecisionNode._same_vision_frame_was_published(
                self,
                decision,
            )
        ):
            self._reset_pre_motion_settle()
            return

        if (
            not queue_while_locked
            and not self._pre_motion_settle_ready(decision, now)
        ):
            return

        terminal_key = (
            decision.source,
            decision.action,
        )

        trigger = False

        next_command_id = self.command_id + 1
        if decision.requires_ack:
            if self.terminal_latch != terminal_key:
                trigger = True
                if not self.phase_manager.start_special_action(
                    decision.action,
                    next_command_id,
                ):
                    self.get_logger().warning(
                        "Special motion command suppressed: "
                        "another special command is active "
                        f"(action={decision.action}, "
                        f"command_id={next_command_id})"
                    )
                    return
                self.event_id += 1
                source = self.SPECIAL_ACTION_SOURCES.get(decision.action)
                if source is not None:
                    self.terminal_action_armed[source] = False
                self.active_special_event_id = self.event_id
                self.active_special_dynamics_command = None

            self.terminal_latch = terminal_key

        elif self.active_special_command_id is None:
            self.terminal_latch = None

        self.command_id = next_command_id

        payload = decision.to_dict()

        payload.update(
            {
                "command_id": self.command_id,
                "event_id": (
                    self.event_id
                    if decision.requires_ack
                    else None
                ),
                "sdk_motion_requested": bool(
                    trigger
                    or (
                        is_pickup_checkpoint_action
                        and decision.sdk_motion_requested
                    )
                ),
                "request_latched": (
                    decision.requires_ack
                ),
                "sdk_motion_id": None,
                "input_age_sec": ages,
                "ball_tracking": (
                    self.planner.ball_tracking_status()
                ),
                "goal_tracking": (
                    self.planner.goal_tracking_status()
                ),
                "special_motion_running": (
                    self.special_motion_running
                ),
                "active_special_action": (
                    self.active_special_action
                ),
                "active_special_event_id": (
                    self.active_special_event_id
                ),
                "active_special_command_id": (
                    self.active_special_command_id
                ),
                "mission_progress": self._mission_progress(),
                "source_node": (
                    "motion_decision_node"
                ),
            }
        )

        output = String()

        output.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )

        self.publisher.publish(output)
        if decision.valid and decision.source_command.get("invalid_ground_recovery") is True:
            self.line_ground_recovery_active = True
            self.line_ground_recovery_command_id = self.command_id
            self.line_offset_min_rgb_stamp_ns = MotionDecisionNode._current_ros_time_ns(self)
        if decision.valid and decision.action.startswith("LINE_OFFSET_TURN_"):
            geometry = decision.source_command
            self.get_logger().info(
                f"Line offset alignment: action={decision.action}, "
                f"offset_px={geometry.get('lateral_offset_px')}, "
                f"reference_x_px={geometry.get('offset_reference_x_px')}, "
                f"reference_y_px={geometry.get('offset_reference_y_px')}, "
                f"steering_deg={geometry.get('offset_reference_steering_deg')}, "
                f"turn_count={geometry.get('turn_count')}, "
                f"turn_angle_deg={geometry.get('turn_angle_deg')}"
            )
        if is_general_motion or (decision.valid and decision.sdk_motion_requested):
            # Motion time must never count toward the next stationary pause.
            self.hurdle_stationary_since = None
            self.goal_stationary_since = None
        if decision.valid and decision.source == "hurdle" and decision.action == "GO":
            self.pending_hurdle_final_sequence = None
        limiter.record_published(decision, observations)
        sparse.published(decision, MotionDecisionNode._current_ros_time_ns(self))
        if (
            decision.valid and decision.source == "line"
            and decision.source_command.get("lost_search_stage") in {"INITIAL", "EXTEND", "REVERSE"}
            and getattr(self, "post_shot_line_search_action", None) is not None
        ):
            self.post_shot_line_search_action = decision.action
        if (
            decision.valid and decision.source == "line"
            and decision.action in {"LEFT", "RIGHT"}
        ):
            self.pending_line_corner = None
            self.line_corner_rearm_required = True
            if decision.source_command.get("corner_from_memory") is True:
                self.remembered_corner_command_id = self.command_id
        elif decision.source not in {"line", "none"}:
            self.pending_line_corner = None
            self.line_corner_rearm_required = False
        if decision.valid and decision.action.startswith("POST_BALL_LINE_TURN_"):
            command = decision.source_command
            self.get_logger().info(
                "Post-ball line turn requested: "
                f"command_id={self.command_id}, action={decision.action}, "
                f"reason={decision.reason}, "
                f"heading_deg={command.get('heading_error_deg')}, "
                f"offset_norm={command.get('lateral_offset_norm')}, "
                f"reference={command.get('alignment_reference')}, "
                f"target_px={command.get('target_point_px')}, "
                f"steering_deg={command.get('steering_error_deg')}, "
                f"tolerance_deg={command.get('heading_tolerance_deg')}, "
                f"turn_angle_deg={command.get('turn_angle_deg')}, "
                f"input_age_sec={ages.get('line')}"
            )
        if decision.source_command.get("lost_ball_interior_forward") is True:
            self.planner.ball_interior_loss_forward_sent = True
        if decision.source_command.get("lost_ball_top_forward") is True:
            self.planner.mark_ball_top_loss_forward_sent()

        if decision.action == "PICKUP_NOW":
            self.planner.pickup_close_alignment_active = False
            self.planner.pickup_last_visible_bottom_distance_px = None
            self.planner._remember_pickup_close_ball(observations.get("ball"))
            self.ball_pickup_entry_pending = False
            self.pickup_positioning_motion_running = False
            self.pickup_positioning_motion_id = None
            self.pickup_positioning_ball_seen_during_motion = False
            self.pickup_positioning_ball_lost_pending = False
            self.pickup_positioning_loss_latch_sent = False
            self.pickup_fixed_sequence_started = False

        self._reset_pre_motion_settle()

        if is_general_motion:
            if queue_while_locked:
                self.queued_general_action = decision.action
                self.queued_general_command_id = self.command_id
                self.queued_general_source = decision.source
            else:
                self.active_general_source = decision.source
                self.general_motion_gate.on_command_published(
                    decision.action,
                    self.command_id,
                )
            MotionDecisionNode._remember_published_vision_frame(
                self,
                decision,
            )
            if decision.source == "ball":
                completed_at = getattr(self, "ball_motion_completed_at", None)
                if completed_at is not None:
                    self.get_logger().info(
                        f"BALL motion boundary: wait_sec={now - completed_at:.3f}, "
                        f"next_action={decision.action}, "
                        f"reobserved={decision.source_command.get('approach_reobserved', False)}"
                    )
                    self.ball_motion_completed_at = None
                if decision.action.startswith("BALL_APPROACH_TURN_"):
                    observer = getattr(self, "ball_approach_observation", None)
                    if observer is not None:
                        observer.reset()
                self.ball_approach_entry_pending = False
                if decision.source_command.get(
                    "lost_ball_alignment_from_memory"
                ) is True:
                    self.ball_lost_during_motion_pending = False
                    self.ball_last_visible_approach_info = None
        elif is_pickup_checkpoint_action:
            MotionDecisionNode._remember_published_vision_frame(
                self,
                decision,
            )
            if is_pickup_positioning_loss_latch:
                self.pickup_positioning_loss_latch_sent = True
            elif is_pickup_initial_align_action:
                self.pickup_initial_align_waiting = False
                if self.pickup_positioning_ball_lost_pending:
                    self.pickup_positioning_ball_lost_pending = False
                    self.pickup_positioning_loss_latch_sent = False
            elif is_pickup_fine_align_action:
                self.pickup_fine_align_waiting = False
                if decision.action == "BALL_PICKUP_FINE_ALIGN_CONTINUE":
                    self.pickup_fixed_sequence_started = True
            else:
                self.pickup_post_backward_align_waiting = False

    def _publish_decision_debug(self) -> None:
        """Publish existing decision state without affecting motion control."""
        try:
            now = time.monotonic()
            fresh_vision = {
                source: bool(
                    self.latest_time[source] is not None
                    and now - self.latest_time[source] <= self.timeouts[source]
                )
                for source in MotionDecisionNode.SOURCES
            }
            line_info = self.latest_info.get("line") or {}
            line_planner = self.planner.line_planner
            line_config = line_planner.config
            candidate = self.last_candidate_decision
            selected = self.last_selected_decision

            heading_source = getattr(line_config, "heading_source", "ground")
            heading_deg = line_heading(line_info, heading_source)
            center_offset = line_info.get("filtered_lateral_offset_norm")
            if center_offset is None:
                center_offset = line_info.get("lateral_offset_norm")

            payload = {
                "phase": self.mission_phase,
                "sparse_line": (
                    self.sparse_line_recovery.diagnostics()
                    if getattr(self, "sparse_line_recovery", None) is not None else None
                ),
                "shot_settle": {
                    "required_sec": self.SHOT_PRE_MOTION_SETTLE_SEC,
                    "stationary_elapsed_sec": (
                        round(max(0.0, now - self.goal_stationary_since), 3)
                        if getattr(self, "goal_stationary_since", None) is not None else None
                    ),
                },
                "safety": self.safety_interlock.snapshot.to_dict(),
                "source": (
                    selected.source.upper() if selected is not None else "NONE"
                ),
                "fresh_vision": fresh_vision,
                "ball_tracking": {
                    **self.planner.ball_tracking_status(),
                    "lost": bool(
                        fresh_vision["ball"]
                        and self.latest_info.get("ball") is not None
                        and self.latest_info["ball"].get("detected") is not True
                        and self.latest_info["ball"].get("raw_detected") is not True
                        and self.latest_info["ball"].get("ball_loss_confirmed") is True
                        and self.planner.ball_tracking_active
                        and not self.pickup_fixed_sequence_started
                        and self.pickups_completed < self.required_pickups
                    ),
                },
                "source_fresh": bool(
                    selected is not None
                    and fresh_vision.get(selected.source, False)
                ),
                "hurdle_final_sequence_pending": getattr(
                    self, "pending_hurdle_final_sequence", None,
                ),
                "line": {
                    "line_detected": bool(line_info.get("detected", False)),
                    "last_seen_direction": getattr(self.planner, "last_line_seen_direction", None),
                    "heading_deg": heading_deg,
                    "heading_source": heading_source,
                    "image_heading_deg": line_heading(line_info, "image"),
                    "ground_heading_deg": line_heading(line_info, "ground"),
                    "ground_fit_reason": line_info.get("ground_fit_reason"),
                    "straight_heading_aligned": straight_heading_is_aligned(line_info, line_config),
                    "in_place_ground_heading_deg": line_config.in_place_ground_heading_deg,
                    "center_offset": center_offset,
                    "offset_reference_steering_deg": line_info.get("offset_reference_steering_deg"),
                    "lateral_offset_px": line_info.get("lateral_offset_px"),
                    "pending_direction": line_planner.turn_candidate,
                    "direction_confirmation_current": (
                        line_planner.turn_candidate_hits
                    ),
                    "direction_confirmation_required": (
                        line_config.direction_confirmation_frames
                    ),
                    "turn_enter_deg": line_config.turn_enter_deg,
                    "turn_exit_deg": line_config.turn_exit_deg,
                    "line_large_heading_threshold_deg": (
                        line_config.recovery_heading_turn_deg
                    ),
                },
                "decision": {
                    "candidate_action": (
                        candidate.action if candidate is not None else None
                    ),
                    "selected_action": (
                        selected.action if selected is not None else None
                    ),
                    "reason": selected.reason if selected is not None else None,
                },
                "line_corner": {
                    "pending": getattr(self, "pending_line_corner", None),
                    "rearm_required": getattr(self, "line_corner_rearm_required", False),
                },
                "execution": {
                    "motion_locked": self.general_motion_gate.locked,
                    "mission_locked": self.active_special_command_id is not None,
                    "ball_confirmation_pending": (
                        self.ball_confirmation_pending_latched
                    ),
                    "executor_seen": (
                        self.executor_heartbeat_watchdog.executor_seen
                    ),
                    "executor_state": (
                        "RUNNING"
                        if self.executor_active is True
                        else "IDLE"
                        if self.executor_active is False
                        else "UNKNOWN"
                    ),
                    "active_special_dynamics_command": (
                        self.active_special_dynamics_command
                    ),
                },
                "grasp_verification": {
                    "finalized": bool(
                        not getattr(self, "grasp_verification_active", False)
                        and getattr(self, "grasp_result_display_until", None) is not None
                        and now < self.grasp_result_display_until
                    ),
                    "active": bool(
                        getattr(self, "grasp_verification_active", False)
                    ),
                    "result": getattr(
                        self,
                        "grasp_verification_result",
                        MissionPhaseManager.GRASP_UNKNOWN,
                    ),
                    "confidence": getattr(
                        self, "grasp_verification_confidence", None
                    ),
                    "accepted_frames": getattr(
                        self, "grasp_verify_accepted_frames", 0
                    ),
                    "decision_window_frames": len(
                        getattr(self, "grasp_verify_frame_votes", ())
                    ),
                    "decision_window_grabbed_frames": sum(
                        getattr(self, "grasp_verify_frame_votes", ())
                    ),
                    "frame_window_limit": (
                        MotionDecisionNode.GRASP_VERIFICATION_FRAME_WINDOW
                    ),
                    "required_grab_frames": (
                        MotionDecisionNode.GRASP_VERIFICATION_MIN_SUCCESSES
                    ),
                },
            }
            output = String()
            output.data = json.dumps(
                payload,
                ensure_ascii=True,
                separators=(",", ":"),
            )
            self.decision_debug_publisher.publish(output)
        except Exception as exc:  # noqa: B902 - debug must not affect motion
            self.get_logger().warning(
                "Decision debug publication failed: "
                f"{type(exc).__name__}: {exc}"
            )

    def _update_goal_stationary_time(self, now: float) -> None:
        """Count idle executor time independently of goal detection and candidates."""
        if (
            not self.decision_started
            or self.safety_interlock.latched
            or not self.executor_heartbeat_watchdog.executor_seen
            or not self.executor_auto_ready
            or getattr(self, "executor_active", False) is not False
            or self.general_motion_gate.locked
            or self.active_special_command_id is not None
            or getattr(self, "queued_general_command_id", None) is not None
            or self.line_timeout_recovery_active
        ):
            self.goal_stationary_since = None
        elif getattr(self, "goal_stationary_since", None) is None:
            self.goal_stationary_since = now

    def _reset_pre_motion_settle(self) -> None:
        """Discard a pending pre-motion settle candidate."""
        self.pre_motion_settle_source = None
        self.pre_motion_settle_action = None
        self.pre_motion_settle_started_at = None

    def _pre_motion_settle_ready(
        self,
        decision: MotionDecision,
        now: float,
    ) -> bool:
        """Recheck fresh decisions while settling before turns or fine steps."""
        fine_forward = (
            decision.action in {
                "STRAIGHT_0", "BALL_FINE_FORWARD_8", "BALL_PICKUP_FINE_FORWARD",
                "SLOW_APPROACH", "FINE_FORWARD_STEP",
            }
            or decision.action.startswith("GOAL_CAMERA90_FINE_FORWARD_")
            or (decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
                and decision.source_command.get("pickup_approach_motion") == "STRAIGHT_0")
            or (decision.action == "GO" and any(
                decision.source_command.get(key) is True
                for key in ("fine_sequence_requested", "depth_fallback_requested")
            ))
        )
        pickup_fine_checkpoint = bool(
            fine_forward and self.active_special_action == "PICKUP_NOW"
            and (
                (decision.action == "BALL_PICKUP_FINE_FORWARD"
                 and self.pickup_fine_align_waiting)
                or (decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
                    and self.pickup_initial_align_waiting)
            )
        )
        hurdle_turn = decision.source == "hurdle" and decision.action in {
            "ALIGN_LEFT", "ALIGN_RIGHT", "TURN_LEFT", "TURN_RIGHT",
        }
        line_turn = decision.source == "line" and (
            decision.action in {"LEFT", "RIGHT"}
            or decision.action.startswith(("LINE_LOST_TURN_", "LINE_OFFSET_TURN_", "LINE_HEADING_TURN_", "LINE_SPARSE_"))
            or decision.action.startswith(("POST_BALL_LINE_TURN_", "POST_SHOT_LINE_TURN_"))
        )
        # Fixed post-shot exit turns already have their own one-second pause.
        if (
            not decision.valid
            or (
                not line_turn and not hurdle_turn and not fine_forward
                and decision.action not in self.PRE_MOTION_SETTLE_ACTIONS
            )
        ):
            self._reset_pre_motion_settle()
            return True

        if (
            self.general_motion_gate.locked
            or (self.active_special_command_id is not None and not pickup_fine_checkpoint)
        ):
            self._reset_pre_motion_settle()
            return False

        if decision.source == "goal" and decision.action == "SHOT":
            # The current planner decision authorizes the shot; this only checks rest.
            MotionDecisionNode._update_goal_stationary_time(self, now)
            self._reset_pre_motion_settle()
            stationary_since = self.goal_stationary_since
            return (
                stationary_since is not None
                and now - stationary_since >= self.SHOT_PRE_MOTION_SETTLE_SEC
            )

        settle_sec = (
            getattr(self, "LINE_TURN_PRE_MOTION_SETTLE_SEC", MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC)
            if line_turn else self.pre_motion_settle_sec
        )
        if fine_forward:
            settle_sec = getattr(
                self, "FINE_FORWARD_PRE_MOTION_SETTLE_SEC",
                MotionDecisionNode.FINE_FORWARD_PRE_MOTION_SETTLE_SEC,
            )
        elif hurdle_turn:
            settle_sec = MotionDecisionNode.HURDLE_POST_MOTION_DWELL_SEC
        if settle_sec <= 0.0:
            self._reset_pre_motion_settle()
            return True
        if decision.source == "hurdle" and (fine_forward or hurdle_turn):
            # Brief detection loss does not undo time already spent stationary.
            # Only a valid current decision or the existing latched GO reaches here.
            stationary_since = getattr(self, "hurdle_stationary_since", None)
            if stationary_since is None:
                self.hurdle_stationary_since = now
                stationary_since = now
            self._reset_pre_motion_settle()
            return now - stationary_since >= settle_sec

        candidate = (decision.source, decision.action)
        pending = (
            self.pre_motion_settle_source,
            self.pre_motion_settle_action,
        )
        if pending != candidate or self.pre_motion_settle_started_at is None:
            if self.pre_motion_settle_action is not None:
                self.get_logger().info(
                    "Pre-motion settle reset: "
                    f"previous={self.pre_motion_settle_action}, "
                    f"next={decision.action}"
                )
            self.pre_motion_settle_source = decision.source
            self.pre_motion_settle_action = decision.action
            self.pre_motion_settle_started_at = now
            self.get_logger().info(
                "Pre-motion settle started: "
                f"action={decision.action}, source={decision.source}, "
                f"duration={settle_sec:.3f}"
            )
            return settle_sec <= 0.0

        if now - self.pre_motion_settle_started_at < settle_sec:
            return False

        self.get_logger().info(
            f"Pre-motion settle complete: action={decision.action}"
        )
        self._reset_pre_motion_settle()
        return True

    def _same_vision_frame_was_published(
        self,
        decision: MotionDecision,
    ) -> bool:
        """Prevent one source frame from creating the same command twice."""
        stamp = getattr(self, "latest_time", {}).get(decision.source)
        return bool(
            stamp is not None
            and self.last_published_vision_stamp.get(decision.source) == stamp
        )

    def _remember_published_vision_frame(
        self,
        decision: MotionDecision,
    ) -> None:
        """Record the source frame consumed by an executable command."""
        stamp = getattr(self, "latest_time", {}).get(decision.source)
        if stamp is not None:
            self.last_published_vision_stamp[decision.source] = stamp

    def _command_publisher_has_subscriber(self) -> bool:
        """Report readiness changes without publishing into a disconnected topic."""
        ready = self.publisher.get_subscription_count() >= 1
        previous = self._command_publisher_ready

        if not ready and previous is not False:
            self.get_logger().warning(
                "Motion command publication deferred: "
                "no /navigation/motion_command subscriber"
            )
        elif ready and previous is False:
            self.get_logger().info(
                "Motion command publication resumed: "
                "/navigation/motion_command subscriber connected"
            )

        self._command_publisher_ready = ready
        return ready

    def _advance_post_ball_line_run(self, now: float) -> bool:
        """Leave the timed Line stage only at an idle motion boundary."""
        deadline = getattr(self, "post_ball_line_run_until", None)
        if (
            deadline is None or now < deadline
            or self.general_motion_gate.locked
            or getattr(self, "queued_general_command_id", None) is not None
            or self.active_special_command_id is not None
        ):
            return False
        if not self.phase_manager.complete_post_ball_line_run():
            return False
        self.post_ball_line_run_until = None
        MotionDecisionNode._discard_line_motion_capture(self)
        self.pending_line_decision = None
        MotionDecisionNode._clear_ball_navigation_state(self)
        MotionDecisionNode._invalidate_pickup_ball_input(self)
        self.planner._clear_goal_tracking()
        self.planner.goal_lock_active = False
        self.planner.goal_terminal_requested = False
        self.latest_info["goal"] = None
        self.latest_time["goal"] = None
        self._reset_pre_motion_settle()
        self.get_logger().info(
            "Post-pickup Line driving completed: "
            f"grasp={self.phase_manager.grasp_result_for_ball(self.pickups_completed)}, "
            f"next_phase={self.mission_phase}"
        )
        return True

    def _select_mission_decision(
        self,
        observations: dict[str, dict[str, Any] | None],
        dt_sec: float,
    ) -> MotionDecision:
        """Keep compatibility phases while disabling automatic finish actions."""
        if self.mission_complete or self.mission_phase == "FINISHED":
            return MotionDecision(
                phase="FINISHED",
                source="none",
                action="STOP",
                valid=True,
                reason="mission_complete_stop",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        planning_phase = self.phase_manager.current_phase
        if (
            self.phase_manager.execution_limit_reached("SHOT")
            and self.active_special_action != "SHOT"
        ):
            self.planner._clear_goal_tracking()
            self.planner.goal_lock_active = False
            self.planner.goal_terminal_requested = False
            observations = dict(observations)
            observations["goal"] = None
            if (
                self.planner.source_for_phase(planning_phase) == "goal"
                or planning_phase == "POST_BALL_GOAL_TRANSITION"
            ):
                self.phase_manager.set_phase("AUTO")
                planning_phase = "AUTO"
        if (
            self.active_special_action != "GO"
            and (
                self.phase_manager.hurdles_completed
                >= self.phase_manager.required_hurdles
                or self.phase_manager.execution_limit_reached("GO")
            )
        ):
            self.planner.disable_completed_hurdle_missions()
            self.hurdle_positioning_entry_pending = False
            self.pending_hurdle_final_sequence = None
            self.hurdle_post_motion_dwell_until = None
            observations = dict(observations)
            observations["hurdle"] = None
            if self.planner.source_for_phase(planning_phase) == "hurdle":
                self.phase_manager.set_phase("LINE_TRACK")
                planning_phase = "LINE_TRACK"
        if MotionDecisionNode._ball_navigation_blocked(self):
            MotionDecisionNode._clear_ball_navigation_state(self)
            observations = dict(observations)
            observations["ball"] = None

        if (
            self.pickups_completed > 0
            and self.ball_sections_processed >= self.pickups_completed
            and self.phase_manager.grasp_result_for_ball(self.pickups_completed)
            != MissionPhaseManager.GRASPED
        ):
            # Continue the course after an empty pickup; its Goal must not
            # reacquire control before a later pickup supplies a verified ball.
            observations = dict(observations)
            observations["goal"] = None

        if (
            getattr(self, "hurdle_positioning_entry_pending", False)
            and self.active_special_command_id is None
            and not self.general_motion_gate.locked
            and getattr(self, "hurdle_post_motion_dwell_until", None) is None
        ):
            self.hurdle_positioning_entry_pending = False
            self.phase_manager.set_phase("HURDLE_POSITIONING")
            self.planner.hurdle_lock_active = True
            MotionDecisionNode._clear_ball_navigation_state(self)
            self.get_logger().info("HURDLE positioning started after current motion")
            planning_phase = "HURDLE_POSITIONING"
        search_action = getattr(self, "post_shot_line_search_action", None)
        if planning_phase != "LINE_TRACK":
            self.post_shot_line_search_action = None
            self.post_shot_line_search_failed = False
        elif search_action is not None:
            if getattr(self, "post_shot_line_search_failed", False):
                return MotionDecision(
                    phase=planning_phase, source="line", action="WAIT", valid=False,
                    reason="post_shot_line_search_failed",
                    sdk_motion_requested=False, requires_ack=False, source_command={},
                )
            line_info = observations.get("line")
            if line_info is not None and line_info.get("detected") is True:
                # Once reacquired, later losses belong to normal Line navigation.
                self.post_shot_line_search_action = None
            else:
                has_fresh_line = line_info is not None
                return MotionDecision(
                    phase=planning_phase, source="line",
                    action=search_action if has_fresh_line else "STOP",
                    valid=has_fresh_line,
                    reason=("post_shot_line_search" if has_fresh_line
                            else "waiting_for_line_info"),
                    sdk_motion_requested=False, requires_ack=False, source_command={},
                )

        if (
            planning_phase == "HURDLE_POSITIONING"
            and self.active_special_command_id is None
        ):
            checkpoint = getattr(self, "pending_hurdle_final_sequence", None)
            if checkpoint is not None:
                self.planner.hurdle_go_requested = True
                return MotionDecision(
                    phase=planning_phase, source="hurdle", action="GO", valid=True,
                    reason="hurdle_final_sequence_latched",
                    sdk_motion_requested=True, requires_ack=True,
                    source_command=dict(checkpoint),
                )
            return self.planner.plan(planning_phase, observations, dt_sec)

        if planning_phase == "LINE_TRACK_AFTER_PICKUP":
            if getattr(self, "post_ball_line_run_failed", False):
                return MotionDecision(
                    phase=planning_phase, source="line", action="WAIT",
                    valid=False, reason="post_pickup_line_motion_failed",
                    sdk_motion_requested=False, requires_ack=False, source_command={},
                )
            if self.active_special_command_id is not None:
                return MotionDecision(
                    phase=planning_phase, source="none", action="WAIT",
                    valid=False, reason="post_pickup_line_special_motion_running",
                    sdk_motion_requested=False, requires_ack=False, source_command={},
                )
            line_observations = dict(observations)
            line_observations["ball"] = None
            line_observations["goal"] = None
            return self.planner.plan(planning_phase, line_observations, dt_sec)

        if planning_phase.startswith("POST_SHOT_"):
            if self.phase_manager.post_shot_failed:
                return MotionDecision(
                    phase=planning_phase, source="line", action="WAIT",
                    valid=False, reason="post_shot_motion_failed",
                    sdk_motion_requested=False, requires_ack=False,
                    source_command={},
                )
            if planning_phase == "POST_SHOT_TURN":
                return MotionDecision(
                    phase=planning_phase, source="line",
                    action=self.phase_manager.post_shot_turn_action(),
                    valid=True, reason="post_shot_fixed_exit_turn",
                    sdk_motion_requested=False, requires_ack=False,
                    source_command={},
                )
            alignment = self.planner.plan(
                "POST_SHOT_LINE_ALIGN", {"line": observations.get("line")}, dt_sec,
            )
            if planning_phase == "POST_SHOT_FORWARD":
                if alignment.action == "POST_SHOT_LINE_ALIGNED":
                    return MotionDecision(
                        phase=planning_phase, source="line",
                        action="POST_SHOT_FORWARD", valid=True,
                        reason="post_shot_aligned_forward_after_dwell",
                        sdk_motion_requested=False, requires_ack=False,
                        source_command=alignment.source_command,
                    )
                # Recheck after the pause; changed heading must be corrected
                # and pass a new stationary dwell before walking.
                if alignment.valid:
                    self.phase_manager.set_phase("POST_SHOT_LINE_ALIGN")
            return alignment
        if planning_phase == "POST_BALL_GOAL_TRANSITION":
            if self.active_special_command_id is not None:
                return MotionDecision(
                    phase=planning_phase, source="none", action="WAIT",
                    valid=False, reason="post_ball_goal_transition_running",
                    sdk_motion_requested=False, requires_ack=False,
                    source_command={},
                )
            failed = self.phase_manager.post_ball_goal_transition_failed
            return MotionDecision(
                phase=planning_phase,
                source="none",
                action=(
                    "WAIT"
                    if failed
                    else "POST_BALL_GOAL_TRANSITION"
                ),
                valid=not failed,
                reason=(
                    "post_ball_goal_transition_aborted"
                    if failed
                    else "start_post_ball_goal_transition"
                ),
                sdk_motion_requested=not failed,
                requires_ack=not failed,
                source_command={},
            )
        ball_missions_complete = (
            self.active_special_action != "PICKUP_NOW"
            and (
                self.pickups_completed >= self.required_pickups
                or self.phase_manager.execution_limit_reached("PICKUP_NOW")
            )
        )
        if ball_missions_complete:
            self.planner.disable_completed_ball_missions()
            observations = dict(observations)
            observations["ball"] = None
            if self.planner.source_for_phase(planning_phase) == "ball":
                self.phase_manager.set_phase("AUTO")
                planning_phase = "AUTO"
        ball_info = observations.get("ball")
        if (
            not ball_missions_complete
            and not self.pickup_fixed_sequence_started
            and not MotionDecisionNode._ball_navigation_blocked(self)
            and not getattr(self.planner, "hurdle_lock_active", False)
            and (
                not getattr(self.planner, "goal_lock_active", False)
                or getattr(self.planner, "ball_lock_active", False)
            )
            and (
                getattr(self.planner, "ball_tracking_active", False)
                or getattr(self.planner, "ball_lock_active", False)
                or self.active_special_action == "PICKUP_NOW"
                or self.planner.source_for_phase(planning_phase) == "ball"
            )
            and ball_info is not None
            and ball_info.get("detected") is not True
            and ball_info.get("raw_detected") is not True
            and ball_info.get("ball_loss_confirmed") is not True
        ):
            # Keep an executing motion intact; do not start another approach or
            # recovery motion until distinct fresh frames confirm the loss.
            return MotionDecision(
                phase=planning_phase, source="ball", action="WAIT", valid=False,
                reason="ball_loss_confirmation_pending",
                sdk_motion_requested=False, requires_ack=False, source_command={},
            )
        if (
            self.pickup_initial_align_waiting
            and self.active_special_action == "PICKUP_NOW"
        ):
            ball_info = observations.get("ball")
            return self.planner.plan_ball_pickup_initial_alignment(
                ball_info
            )
        if (
            self.pickup_positioning_ball_lost_pending
            and not self.pickup_positioning_loss_latch_sent
            and self.active_special_action == "PICKUP_NOW"
        ):
            return MotionDecision(
                phase="BALL_PICKUP_POSITIONING",
                source="ball",
                action=self.PICKUP_POSITIONING_LOSS_LATCH_ACTION,
                valid=True,
                reason="pickup_positioning_ball_loss_latched",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={
                    "pickup_positioning_ball_lost": True,
                    "pickup_positioning_motion_id": (
                        self.pickup_positioning_motion_id
                    ),
                },
            )
        if (
            self.pickup_fine_align_waiting
            and self.active_special_action == "PICKUP_NOW"
        ):
            return self.planner.plan_ball_pickup_fine_alignment(
                observations.get("ball")
            )
        if (
            self.pickup_post_backward_align_waiting
            and self.active_special_action == "PICKUP_NOW"
        ):
            return self.planner.plan_ball_pickup_post_backward_alignment(
                observations.get("ball")
            )
        if self.active_special_command_id is not None:
            locked_phase = f"{planning_phase}_LOCK"
            return self.planner.plan(
                locked_phase,
                observations,
                dt_sec,
            )

        ball_info = observations.get("ball")
        prepared_alignment = None
        if (
            ball_info is not None
            and ball_info is getattr(self, "ball_prepared_info", None)
            and getattr(self.planner, "ball_lock_active", False)
            and self.active_special_command_id is None
            and (
                ball_info.get("detected") is True
                or self.ball_approach_observation.is_current(time.monotonic())
            )
        ):
            ball_info = self.ball_prepared_observation
            observations = {**observations, "ball": ball_info}
            prepared_alignment = self.ball_prepared_alignment
        if self.ball_pickup_entry_pending:
            return MotionDecision(
                phase=planning_phase,
                source="ball",
                action="PICKUP_NOW",
                valid=True,
                reason="ball_pickup_entry_latched_during_motion",
                sdk_motion_requested=True,
                requires_ack=True,
                source_command={"pickup_entry_latched": True},
            )

        if self.ball_approach_alignment_pending:
            ball_confirmed_for_alignment = bool(
                ball_info is not None
                and ball_info.get("detected") is True
            )
            if not ball_confirmed_for_alignment:
                if ball_info is None or ball_info.get("raw_detected") is True:
                    return MotionDecision(
                        phase="BALL_APPROACH_LOST_ALIGN",
                        source="ball",
                        action="WAIT",
                        valid=False,
                        reason="ball_recovery_waiting_for_fresh_confirmed_frame",
                        sdk_motion_requested=False,
                        requires_ack=False,
                        source_command={},
                    )
                return self.planner.plan_lost_ball_approach_alignment(ball_info)
            alignment = prepared_alignment or self.planner.plan_ball_approach_alignment(ball_info)
            if alignment.action != "BALL_APPROACH_ALIGNED":
                return alignment
            self.ball_approach_alignment_pending = False
            self.ball_last_visible_line_info = None

        ball_confirmed = bool(
            ball_info is not None
            and ball_info.get("detected") is True
        )
        if (
            not ball_missions_complete
            and self.ball_confirmation_pending_latched
            and not ball_confirmed
        ):
            return MotionDecision(
                phase=planning_phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_confirmation_pending_hold",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={
                    "raw_detected": bool(
                        ball_info is not None
                        and ball_info.get("raw_detected") is True
                    ),
                    "confirmation_hits": (
                        ball_info.get("confirmation_hits")
                        if ball_info is not None
                        else None
                    ),
                    "confirmation_required_hits": (
                        ball_info.get("confirmation_required_hits")
                        if ball_info is not None
                        else None
                    ),
                },
            )

        if planning_phase == "LINE_TRACK" and self.finish_enabled:
            final_line_observations = dict(observations)
            final_line_observations["ball"] = None
            final_line_observations["goal"] = None

            return self.planner.plan(
                "LINE_TRACK",
                final_line_observations,
                dt_sec,
            )

        approach_phase = self.planner.approach_phase_for_search(
            planning_phase,
            observations,
        )
        if approach_phase is not None:
            self.phase_manager.set_phase(approach_phase)
            planning_phase = approach_phase

        if planning_phase == "WALK_TO_FINISH":
            line_decision = self.planner.plan(
                "LINE_TRACK",
                observations,
                dt_sec,
            )
            return MotionDecision(
                phase="WALK_TO_FINISH",
                source=line_decision.source,
                action=line_decision.action,
                valid=line_decision.valid,
                reason=line_decision.reason,
                sdk_motion_requested=line_decision.sdk_motion_requested,
                requires_ack=line_decision.requires_ack,
                source_command=line_decision.source_command,
            )

        if ball_info is not None:
            observations = dict(observations)
            observations["ball"] = dict(ball_info)
            observations["ball"]["ball_occurrence"] = (
                1 if self.pickups_completed == 0 else 2
            )

        decision = self.planner.plan(
            planning_phase,
            observations,
            dt_sec,
        )
        if ball_info is not None and ball_info.get("approach_reobserved") is True:
            decision = replace(decision, source_command={
                **decision.source_command, "approach_reobserved": True,
            })
        return decision

    def _rearm_absent_terminal_targets(
        self,
        observations: dict[str, dict[str, Any] | None],
    ) -> None:
        """Re-arm a target only after it is absent or explicitly lost."""
        for source in self.terminal_action_armed:
            info = observations.get(source)
            if info is None or not bool(info.get("detected", False)):
                self.terminal_action_armed[source] = True

    def _suppress_duplicate_terminal_action(
        self,
        decision: MotionDecision,
    ) -> MotionDecision:
        """Replace a disarmed target's repeated terminal action with WAIT."""
        source = self.SPECIAL_ACTION_SOURCES.get(decision.action)
        if (
            source is None
            or decision.source != source
            or not decision.requires_ack
            or self.terminal_action_armed[source]
        ):
            return decision

        return MotionDecision(
            phase=decision.phase,
            source=decision.source,
            action="WAIT",
            valid=False,
            reason="duplicate_terminal_action_suppressed",
            sdk_motion_requested=False,
            requires_ack=False,
            source_command=decision.source_command,
        )


    def _suppress_exhausted_special_action(
        self,
        decision: MotionDecision,
    ) -> MotionDecision:
        """Block special actions whose execution or failure limit is exhausted."""
        if (
            decision.requires_ack
            and self.phase_manager.execution_limit_reached(decision.action)
        ):
            return MotionDecision(
                phase=decision.phase, source=decision.source, action="WAIT",
                valid=False, reason="mission_execution_limit_reached",
                sdk_motion_requested=False, requires_ack=False,
                source_command=decision.source_command,
            )
        if (
            not decision.requires_ack
            or not self.phase_manager.special_action_exhausted(
                decision.action
            )
        ):
            return decision

        reason = self.SPECIAL_FAILURE_REASONS.get(
            decision.action,
            "special_action_failures_exhausted",
        )

        return MotionDecision(
            phase=decision.phase,
            source=decision.source,
            action="WAIT",
            valid=False,
            reason=reason,
            sdk_motion_requested=False,
            requires_ack=False,
            source_command=decision.source_command,
        )

    def _suppress_unverified_shot(
        self,
        decision: MotionDecision,
    ) -> MotionDecision:
        """Fail closed when the corresponding BALL was not verified in hand."""
        if decision.action != "SHOT" or not decision.requires_ack:
            return decision
        grasp_result = self.phase_manager.grasp_result_for_next_shot()
        if grasp_result == MissionPhaseManager.GRASPED:
            return decision
        return MotionDecision(
            phase=decision.phase,
            source=decision.source,
            action="WAIT",
            valid=False,
            reason=f"shot_blocked_grasp_{grasp_result.lower()}",
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={
                **decision.source_command,
                "grasp_result": grasp_result,
            },
        )


def main(args: list[str] | None = None) -> None:
    """Run the unified motion decision node."""
    rclpy.init(args=args)

    node: MotionDecisionNode | None = None

    try:
        node = MotionDecisionNode()
        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
