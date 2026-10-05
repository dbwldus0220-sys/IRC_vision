#!/usr/bin/env python3
"""Bridge navigation JSON commands to the C++ motion executor."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .safety_interlock import RECOVERABLE_MOTOR_ERROR_CODES


LEFT_RECOVERY_MOTION_IDS = {
    count: f"line_recovery_left_{count}"
    for count in (4,)
}
RIGHT_RECOVERY_MOTION_IDS = {
    count: f"line_recovery_right_{count}"
    for count in (4,)
}


class MotionCommandBridgeNode(Node):
    """Translate supported navigation actions into SDK executor requests."""

    DWELL_MARKER = "__NON_BLOCKING_DWELL__"
    SHOT_PREPARE_DWELL_MARKER = "__SHOT_PREPARE_DWELL__"
    SHOT_PREPARE_MOTION_ID = "goal_fine_to_default"
    SHOT_PREPARE_DWELL_SEC = 2.0
    GOAL_CRAB_ACTIONS = frozenset(
        {"GOAL_CAMERA90_CRAB_LEFT", "GOAL_CAMERA90_CRAB_RIGHT"}
    )
    GOAL_CRAB_PRE_DWELL_MARKER = "__GOAL_CRAB_PRE_DWELL__"
    GOAL_CRAB_PRE_DWELL_SEC = 1.0
    GOAL_FORWARD_CRAB_PREPARE_MOTION_ID = "goal_forward_to_crab_right_90"
    GOAL_FINE_RIGHT_CRAB_PREPARE_MOTION_ID = "goal_fine_to_crab_right_90"
    GOAL_FINE_LEFT_CRAB_PREPARE_MOTION_ID = "goal_fine_to_crab_right_90"
    POST_BALL_CAMERA_DWELL_MARKER = "__POST_BALL_CAMERA_DWELL__"
    POST_BALL_CAMERA_PAUSE_SEC = 0.0
    PICKUP_INITIAL_ALIGN_DWELL_MARKER = (
        "__BALL_PICKUP_INITIAL_ALIGN_DWELL__"
    )
    PICKUP_INITIAL_ALIGN_MARKER = "__BALL_PICKUP_INITIAL_ALIGN_CHECK__"
    FINE_ALIGN_MARKER = "__BALL_PICKUP_FINE_ALIGN_CHECK__"
    POST_BACKWARD_ALIGN_MARKER = (
        "__BALL_PICKUP_POST_BACKWARD_ALIGN_CHECK__"
    )
    PICKUP_POSITIONING_LOSS_LATCH_ACTION = (
        "BALL_PICKUP_POSITIONING_LOSS_LATCH"
    )
    PICKUP_FIXED_SEQUENCE_FIRST_MOTION = "pickup_pre_backward_camera_down"
    PICKUP_GRASP_CHECK_MOTION_ID = "pickup_grasp_check_pose"
    PICKUP_FINE_PREPARE_MOTION_ID = "pickup_fine_prepare"
    PICKUP_FINE_PRE_DWELL_MARKER = "__PICKUP_FINE_PRE_DWELL__"
    PICKUP_FINE_PRE_DWELL_SEC = 1.0
    TRANSITION_DWELL_SEC = 1.0
    TRANSITION_MOTION_IDS = frozenset({
        "pickup_fine_prepare", "pickup_fine_to_crab_right_0",
        "pickup_forward_to_crab_0", "goal_fine_to_crab_right_90",
        "goal_forward_to_crab_right_90",
        "fine_to_turn_ready_0", "fine_to_turn_ready_45", "fine_to_turn_ready_90",
    })
    PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID = "pickup_fine_to_crab_right_0"
    PICKUP_DWELL_SEC = 1.0
    PICKUP_TURN_DWELL_SEC = 1.0
    PICKUP_GRASP_DWELL_SEC = 3.0
    # One skipped fault may recover at the next normal SDK start; a second
    # consecutive failed motion must not consume the rest of the sequence.
    PICKUP_MOTOR_FAILURE_LIMIT = 2
    HURDLE_PRE_GO_DWELL_MARKER = "__HURDLE_PRE_GO_DWELL__"
    HURDLE_PRE_GO_DWELL_SEC = 1.0
    HURDLE_FINAL_FINE_MOTION_ID = "hurdle_fine_forward_10"
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
    PICKUP_CAMERA_DOWN_TURN_MOTION_IDS = {
        **{
            f"BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_{count}": (
                f"pickup_camera_down_turn_left_{count}"
            )
            for count in range(1, 7)
        },
        **{
            f"BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_{count}": (
                f"pickup_camera_down_turn_right_{count}"
            )
            for count in (2, 3, 5, 7, 9)
        },
    }
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
    PICKUP_FINE_ALIGN_MOTION_IDS = {
        "BALL_PICKUP_FINE_FORWARD": "pickup_fine_forward_0",
        "BALL_PICKUP_FINE_SEARCH_LEFT": "pickup_camera_down_turn_left_2",
        "BALL_PICKUP_FINE_SEARCH_RIGHT": "pickup_camera_down_turn_right_5",
        **{
            f"BALL_PICKUP_FINE_SEARCH_{direction}_{count}": f"pickup_camera_down_turn_{direction.lower()}_{count}"
            for direction, counts in (("LEFT", (1, 3)), ("RIGHT", (2, 5)))
            for count in counts
        },
        "BALL_PICKUP_FINE_SEARCH_FORWARD": "ball_camera_down_forward_2",
        "BALL_PICKUP_FINE_SEARCH_FORWARD_4": "ball_camera_down_forward_4",
        "BALL_PICKUP_FINE_SEARCH_BACKWARD": "pickup_lost_ball_backward_1",
        "BALL_PICKUP_CRAB_RIGHT": "pickup_crab_right_0",
        "BALL_PICKUP_CRAB_LEFT": "pickup_crab_left_0",
    }
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
    ATOMIC_SEQUENCE_ACTIONS = frozenset(
        {"PICKUP_NOW", "POST_BALL_GOAL_TRANSITION", "SHOT", "GO", *GOAL_CRAB_ACTIONS}
    )

    PICKUP_CAMERA_DOWN_MOTION_IDS = {
        "STRAIGHT": "line_forward_4",
        "STRAIGHT_0": "ball_general_fine_forward_8",
        "STRAIGHT_2": "ball_camera_down_forward_4",
    }
    PICKUP_MOTION_TAIL = (
        # Fresh fine-alignment confirmation already completes the approach.
        "pickup_pre_backward_camera_down",
        "pickup",
        PICKUP_GRASP_CHECK_MOTION_ID,
        "pickup_retreat_2",
    )
    PICKUP_NO_BALL_MOTION_TAIL = (
        "pickup_fine_forward_0",
        "pickup_pre_backward_camera_down",
        POST_BACKWARD_ALIGN_MARKER,
        "pickup",
        PICKUP_GRASP_CHECK_MOTION_ID,
        "pickup_retreat_2",
    )
    POST_BALL_GOAL_TRANSITION_SEQUENCE = (
        "post_ball_camera_90",
        POST_BALL_CAMERA_DWELL_MARKER,
    )
    ACTION_TO_MOTION_ID = {
        "STRAIGHT": "line_forward_6",
        "STRAIGHT_1": "line_forward_2",
        "STRAIGHT_2": "line_forward_4",
        "STRAIGHT_3": "line_forward_6",
        "STRAIGHT_4": "line_forward_8",
        "APPROACH": "forward",
        "LEFT": "line_recovery_left_4",
        "RIGHT": "line_recovery_right_4",
        "POST_BALL_GOAL_TRANSITION": "post_ball_camera_90",
        # Keep legacy action IDs; each target now includes its pose transition.
        "POST_SHOT_TURN_RIGHT_9": "post_shot_default_turn_right",
        # Keep the legacy action ID; the composite contains six left turns.
        "POST_SHOT_TURN_LEFT_4": "post_shot_default_turn_left",
        "POST_SHOT_FORWARD": "line_forward_6",
        **{
            f"POST_SHOT_LINE_TURN_{direction}_{count}": (
                f"post_ball_line_turn_{direction.lower()}_{count}"
            )
            for direction, counts in (
                ("RIGHT", (2, 3, 5, 7, 9)), ("LEFT", (1, 2, 3, 4, 5, 6)),
            )
            for count in counts
        },
        "BALL_FINE_FORWARD_8": "ball_general_fine_forward_8",
        "BALL_LOST_FORWARD_2": "ball_camera_down_forward_2",
        "BALL_LOST_FORWARD_4": "ball_camera_down_forward_4",
        # Reuse the existing one-cycle retreat without entering the pickup sequence.
        "HURDLE_LOST_BACKWARD_1": "pickup_lost_ball_backward_1",
        "BALL_APPROACH_RECOVER_LEFT_4": "line_recovery_left_4",
        "BALL_APPROACH_RECOVER_RIGHT_4": "line_recovery_right_4",
        "LINE_LOST_TURN_LEFT": "line_search_left_2",
        "LINE_LOST_TURN_RIGHT": "line_search_right_5",
        "LINE_SPARSE_FORWARD": "line_forward_2",
        "LINE_SPARSE_TURN_LEFT_1": "post_ball_line_turn_left_1",
        "LINE_SPARSE_TURN_RIGHT_2": "post_ball_line_turn_right_2",
        **{
            f"LINE_OFFSET_TURN_{direction}_{count}": (
                f"post_ball_line_turn_{direction.lower()}_{count}"
            )
            for direction, counts in (
                ("LEFT", (1, 2, 3, 4, 5, 6)), ("RIGHT", (2, 3, 5, 7, 9)),
            )
            for count in counts
        },
        **{
            f"LINE_HEADING_TURN_{direction}_{count}": (
                f"post_ball_line_turn_{direction.lower()}_{count}"
            )
            for direction, counts in (
                ("LEFT", (1, 2, 3, 4, 5, 6)), ("RIGHT", (2, 3, 5, 7, 9)),
            )
            for count in counts
        },
        **{
            f"LINE_LOST_TURN_{direction}_{count}": f"post_ball_line_turn_{direction.lower()}_{count}"
            for direction, counts in (("LEFT", (1, 3)), ("RIGHT", (2, 5)))
            for count in counts
        },
        "GOAL_CAMERA_90_FORWARD": "goal_camera_90_forward_6",
        "GOAL_CAMERA_90_FORWARD_1": "goal_camera_90_forward_2",
        "GOAL_CAMERA_90_FORWARD_2": "goal_camera_90_forward_4",
        "GOAL_CAMERA90_BACKWARD_1": "goal_camera_90_backward_1",
        **{
            f"GOAL_CAMERA90_FINE_FORWARD_{count}": f"goal_camera_90_fine_forward_{count}"
            for count in range(1, 5)
        },
        "GOAL_CAMERA90_CRAB_RIGHT": "goal_camera_90_crab_right",
        "GOAL_CAMERA90_CRAB_LEFT": "goal_camera_90_crab_left",
        **{
            f"POST_BALL_LINE_TURN_RIGHT_{count}": (
                f"post_ball_line_turn_right_{count}"
            )
            for count in (2, 3, 5, 7, 9)
        },
        **{
            f"POST_BALL_LINE_TURN_LEFT_{count}": (
                f"post_ball_line_turn_left_{count}"
            )
            for count in (1, 2, 3, 4, 5, 6)
        },
        **{
            f"BALL_APPROACH_TURN_RIGHT_{count}": (
                f"post_ball_line_turn_right_{count}"
            )
            for count in (2, 3, 5, 7, 9)
        },
        **{
            f"BALL_APPROACH_TURN_LEFT_{count}": (
                f"post_ball_line_turn_left_{count}"
            )
            for count in (1, 2, 3, 4, 5, 6)
        },
        **{
            f"GOAL_CAMERA90_TURN_RIGHT_{count}": (
                f"goal_camera_90_turn_right_{count}"
            )
            for count in (2, 3, 5, 7, 9)
        },
        **{
            f"GOAL_CAMERA90_TURN_LEFT_{count}": (
                f"goal_camera_90_turn_left_{count}"
            )
            for count in range(1, 7)
        },
        "SHOT": "goal_shot",
        "GO": "hurdle",
        **{
            f"RECOVER_{line_side}_TURN_LEFT_{suffix}": motion_id
            for line_side in ("LEFT", "RIGHT")
            for suffix, motion_id in LEFT_RECOVERY_MOTION_IDS.items()
        },
        **{
            f"RECOVER_{line_side}_TURN_RIGHT_{suffix}": motion_id
            for line_side in ("LEFT", "RIGHT")
            for suffix, motion_id in RIGHT_RECOVERY_MOTION_IDS.items()
        },
        "TURN_LEFT": "stationary_turn_left",
        "TURN_RIGHT": "stationary_turn_right",
        "ALIGN_LEFT": "stationary_turn_left",
        "ALIGN_RIGHT": "stationary_turn_right",
    }
    TERMINAL_STATUSES = {
        "SUCCEEDED",
        "FAILED",
        "TIMEOUT",
        "CANCELLED",
        "REJECTED",
    }
    FINE_FORWARD_MOTION_IDS = frozenset({
        "ball_general_fine_forward_8", "pickup_fine_forward_0", HURDLE_FINAL_FINE_MOTION_ID,
        *(f"goal_camera_90_fine_forward_{count}" for count in range(1, 5)),
    })
    DEFAULT_TIMEOUT_MS = 12000

    def __init__(self) -> None:
        """Initialize bridge state, publishers, and subscriptions."""
        super().__init__("motion_command_bridge")

        self.last_sent_command_id: int | None = None
        self.motion_in_progress = False
        self.active_command_id: int | None = None
        self.active_event_id: int | None = None
        self.active_action: str | None = None
        self.active_request_id: int | None = None
        self.active_motion_id: str | None = None
        self.active_timeout_ms: int | None = None
        self.active_pickup_sequence: tuple[str, ...] = ()
        self.active_sequence_index = 0
        self.active_dwell_until: float | None = None
        self.post_ball_camera_pause_until: float | None = None
        self.goal_fine_forward_completed = False
        self.hurdle_depth_fine_completed = False
        self.hurdle_sequence_fine_completed = 0
        self.hurdle_fine_sequence_pending = False
        self.goal_crab_completed = False
        self.last_completed_motion_id: str | None = None
        self.last_physical_motion_id: str | None = None
        self.pending_turn_request: dict[str, Any] | None = None
        self.turn_prepare_motion_id: str | None = None
        self.transition_dwell_until: float | None = None
        self.transition_pending_request: dict[str, Any] | None = None
        self.transition_pending_status: dict[str, Any] | None = None
        self.transition_motion_id: str | None = None
        self.head_override_state: dict[str, Any] = {}
        self.head_override_received_at: float | None = None
        self.pickup_initial_align_dwell_until: float | None = None
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_checkpoint_after_dwell: str | None = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.last_pickup_motion_id = None
        self.pickup_consecutive_motor_failures = 0
        self.pickup_motor_failures: list[dict[str, Any]] = []
        self.pickup_last_stage_succeeded = True
        self.pending_pickup_crab_motion_id = None
        self.pickup_positioning_dwell_motion_id: str | None = None
        self.queued_command_id: int | None = None
        self.queued_event_id: int | None = None
        self.queued_action: str | None = None
        self.queued_request_id: int | None = None
        self.queued_motion_id: str | None = None
        self.queued_timeout_ms: int | None = None
        self.queued_request_deferred = False
        self.queued_pickup_sequence: tuple[str, ...] = ()

        self.navigation_subscription = self.create_subscription(
            String,
            "/navigation/motion_command",
            self.navigation_command_callback,
            10,
        )
        self.executor_status_subscription = self.create_subscription(
            String,
            "/motion/executor/status",
            self.executor_status_callback,
            10,
        )
        self.executor_heartbeat_subscription = self.create_subscription(
            String, "/motion/executor/heartbeat", self.executor_heartbeat_callback, 10,
        )
        self.executor_request_publisher = self.create_publisher(
            String,
            "/motion/executor/request",
            10,
        )
        self.motion_status_publisher = self.create_publisher(
            String,
            "/motion/status",
            10,
        )
        self.dwell_timer = self.create_timer(
            0.05,
            self._check_atomic_dwell,
        )

        self.get_logger().info(
            "Motion command bridge ready: /navigation/motion_command -> "
            "/motion/executor/request, /motion/executor/status -> "
            "/motion/status"
        )

    @staticmethod
    def _is_integer(value: Any) -> bool:
        """Return true only for JSON integers, excluding booleans."""
        return isinstance(value, int) and not isinstance(value, bool)

    @classmethod
    def motion_id_for_action(cls, action: str) -> str | None:
        """Return a catalog-backed motion ID for one navigation action."""
        return cls.ACTION_TO_MOTION_ID.get(action)

    @classmethod
    def timeout_ms_from_payload(cls, payload: dict[str, Any]) -> int:
        """Read a positive timeout or use the safe default."""
        source_command = payload.get("source_command")
        candidates = []
        if isinstance(source_command, dict):
            candidates.append(source_command.get("timeout_ms"))
        candidates.append(payload.get("timeout_ms"))

        for candidate in candidates:
            if cls._is_integer(candidate) and candidate > 0:
                return candidate
        return cls.DEFAULT_TIMEOUT_MS

    @classmethod
    def _pickup_motion_sequence(
        cls,
        payload: dict[str, Any],
    ) -> tuple[str, ...] | None:
        """Build one catalog-backed sequence from planner and mission state."""
        source_command = payload.get("source_command")
        mission_progress = payload.get("mission_progress")
        if not isinstance(source_command, dict) or not isinstance(
            mission_progress, dict
        ):
            return None

        completed = mission_progress.get("pickups_completed")
        if (
            isinstance(completed, bool)
            or not isinstance(completed, int)
            or completed not in {0, 1}
        ):
            return None

        if completed == 0:
            final_stages = (
                "pickup_first_backward_turn_right",
                cls.DWELL_MARKER,
            )
        else:
            final_stages = (
                "pickup_second_backward_turn_left",
                cls.DWELL_MARKER,
            )
        return (*cls.PICKUP_MOTION_TAIL, *final_stages)

    def publish_motion_status(
        self,
        *,
        status: str,
        command_id: int | None,
        event_id: int | None,
        request_id: int | None,
        motion_id: str | None,
        action: str | None,
        error_code: str = "",
        message: str = "",
        completed_motion_id: str | None = None,
        verification_window_complete: bool | None = None,
        motor_failures: list[dict[str, Any]] | None = None,
    ) -> None:
        """Publish normalized status while preserving executor fields."""
        payload = {
            "status": status,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "error_code": error_code,
            "message": message,
            "action": action,
            "source_node": "motion_command_bridge_node",
            "motion_in_progress": self.motion_in_progress,
        }
        if completed_motion_id is not None:
            payload["completed_motion_id"] = completed_motion_id
        if verification_window_complete is not None:
            payload["verification_window_complete"] = (
                verification_window_complete
            )
        if (
            motor_failures is None
            and action == "PICKUP_NOW"
            and command_id == self.active_command_id
        ):
            motor_failures = self.pickup_motor_failures
        if motor_failures:
            # Sequence progress and the executor's actual result are separate.
            payload["motor_failures"] = motor_failures
        output = String()
        output.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.motion_status_publisher.publish(output)

    def _publish_local_rejection(
        self,
        *,
        status: str,
        command_id: int,
        event_id: int | None,
        action: str,
        error_code: str,
        message: str,
    ) -> None:
        """Report a command that is not forwarded to the executor."""
        self.publish_motion_status(
            status=status,
            command_id=command_id,
            event_id=event_id,
            request_id=command_id,
            motion_id=self.motion_id_for_action(action),
            action=action,
            error_code=error_code,
            message=message,
        )

    def _record_goal_motion_success(self, motion_id: str) -> None:
        """Track completed goal approach motions until scoring or a new pickup."""
        self.last_completed_motion_id = motion_id
        if motion_id.startswith("goal_camera_90_fine_forward_"):
            self.goal_fine_forward_completed = True
        elif motion_id in {"goal_camera_90_crab_left", "goal_camera_90_crab_right"}:
            self.goal_crab_completed = True
        elif motion_id in {"goal_shot", "post_ball_camera_90"}:
            self.goal_fine_forward_completed = False
            self.goal_crab_completed = False
        elif motion_id == self.SHOT_PREPARE_MOTION_ID:
            # A failed shot retry must not repeat a completed pose transition.
            self.goal_fine_forward_completed = False

    def _enter_pickup_fine_align_checkpoint(self) -> None:
        """Pause the atomic pickup while retaining its command ownership."""
        self.active_motion_id = self.FINE_ALIGN_MARKER
        self.pickup_fine_align_waiting = True
        self.pickup_fine_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.FINE_ALIGN_MARKER,
            action=self.active_action,
            message="waiting for fresh BALL pickup fine alignment",
        )

    def _enter_pickup_initial_align_checkpoint(self) -> None:
        """Request a fresh Ball frame while retaining pickup ownership."""
        self.active_motion_id = self.PICKUP_INITIAL_ALIGN_MARKER
        self.pickup_initial_align_waiting = True
        self.pickup_initial_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.PICKUP_INITIAL_ALIGN_MARKER,
            action=self.active_action,
            message="waiting for fresh BALL pickup heading alignment",
        )

    def _enter_pickup_post_backward_align_checkpoint(self) -> None:
        """Align the camera-down robot after the no-Ball fallback."""
        self.active_motion_id = self.POST_BACKWARD_ALIGN_MARKER
        self.pickup_post_backward_align_waiting = True
        self.pickup_post_backward_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.POST_BACKWARD_ALIGN_MARKER,
            action=self.active_action,
            message="waiting for BALL alignment after pickup backward motion",
        )

    @classmethod
    def _with_pickup_motion_dwells(
        cls,
        sequence: tuple[str, ...],
    ) -> tuple[str, ...]:
        """Insert one non-blocking dwell after every catalog motion."""
        expanded: list[str] = []
        for index, stage in enumerate(sequence):
            expanded.append(stage)
            next_stage = (
                sequence[index + 1]
                if index + 1 < len(sequence)
                else None
            )
            if (
                not stage.startswith("__")
                and next_stage != cls.DWELL_MARKER
            ):
                expanded.append(cls.DWELL_MARKER)
        return tuple(expanded)

    def _start_pickup_checkpoint_dwell(self, checkpoint: str) -> None:
        """Wait one second after a positioning correction before fresh Vision."""
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_positioning_dwell_motion_id = self.active_motion_id
        self.active_motion_id = self.DWELL_MARKER
        dwell_sec = (
            MotionCommandBridgeNode.PICKUP_TURN_DWELL_SEC
            if self.pickup_positioning_dwell_motion_id.startswith("pickup_camera_down_turn_")
            else 0.0
            if self.pickup_positioning_dwell_motion_id.startswith(("line_forward_", "ball_camera_down_forward_"))
            else self.PICKUP_DWELL_SEC
        )
        self.active_dwell_until = time.monotonic() + dwell_sec
        self.pickup_checkpoint_after_dwell = checkpoint
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.DWELL_MARKER,
            action=self.active_action,
            message="pickup correction non-blocking dwell active",
        )
        self.get_logger().info(
            "Pickup correction dwell started: "
            f"{dwell_sec:.1f}s before {checkpoint}"
        )
        if dwell_sec <= 0.0:
            self._check_atomic_dwell(self.active_dwell_until)

    def _start_pickup_initial_align_dwell(self) -> None:
        """Enter initial alignment without a stationary delay."""
        self.active_motion_id = self.PICKUP_INITIAL_ALIGN_DWELL_MARKER
        self.pickup_initial_align_dwell_until = (
            time.monotonic()
        )
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.PICKUP_INITIAL_ALIGN_DWELL_MARKER,
            action=self.active_action,
            message="holding still before BALL pickup heading alignment",
        )
        self._check_atomic_dwell(self.pickup_initial_align_dwell_until)

    def _handle_pickup_initial_align_command(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Apply one fresh heading decision inside the pickup checkpoint."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        checkpoint_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and self.pickup_initial_align_waiting
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
        )
        if not checkpoint_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="PICKUP_INITIAL_ALIGNMENT_NOT_ACTIVE",
                message="pickup initial-alignment checkpoint is not active",
            )
            return

        source_command = payload.get("source_command")

        self.last_sent_command_id = command_id
        self.pickup_initial_align_waiting = False
        if action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE":
            use_no_ball_pickup_path = bool(
                isinstance(source_command, dict)
                and source_command.get("use_no_ball_pickup_path") is True
            )
            approach_motion = (
                source_command.get("pickup_approach_motion")
                if isinstance(source_command, dict)
                else None
            )
            first_motion = self.PICKUP_CAMERA_DOWN_MOTION_IDS.get(
                approach_motion
            )
            if first_motion is None:
                self.pickup_initial_align_waiting = True
                self._publish_local_rejection(
                    status="REJECTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="UNSUPPORTED_PICKUP_DISTANCE_APPROACH",
                    message=(
                        "fresh pickup depth did not select a supported motion"
                    ),
                )
                return
            if use_no_ball_pickup_path:
                if (
                    self.active_pickup_sequence[
                        :len(self.PICKUP_MOTION_TAIL)
                    ] != self.PICKUP_MOTION_TAIL
                ):
                    self.pickup_initial_align_waiting = True
                    self._publish_local_rejection(
                        status="REJECTED",
                        command_id=command_id,
                        event_id=event_id,
                        action=action,
                        error_code="INVALID_PICKUP_NO_BALL_SEQUENCE",
                        message=(
                            "pickup sequence does not contain the expected "
                            "motion tail"
                        ),
                    )
                    return
                final_stages = self.active_pickup_sequence[
                    len(self.PICKUP_MOTION_TAIL):
                ]
                next_sequence = (
                    first_motion,
                    *self.PICKUP_NO_BALL_MOTION_TAIL[1:],
                    *final_stages,
                )
                self.active_pickup_sequence = self._with_pickup_motion_dwells(
                    next_sequence
                )
                self.active_sequence_index = 0
                self.active_motion_id = first_motion
                self._publish_executor_request(
                    action=self.active_action,
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=first_motion,
                    timeout_ms=(
                        self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS
                    ),
                )
                return

            if (
                approach_motion == "STRAIGHT_0"
                and self.pickup_fine_positioning_complete
            ):
                self._enter_pickup_fine_align_checkpoint()
                return

            self.pickup_fine_positioning_complete = bool(
                approach_motion == "STRAIGHT_0"
            )
            self.pickup_initial_align_correction_active = True
            self.active_motion_id = first_motion
            # The distance-selected step is still approach, not pre-grasp alignment.
            self._publish_executor_request(
                action=self.active_action,
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=first_motion,
                timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
                prepare_pickup_fine=False,
            )
            return

        if action == "BALL_PICKUP_INITIAL_SEARCH_BACKWARD":
            motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[
                "BALL_PICKUP_FINE_SEARCH_BACKWARD"
            ]
        elif action == "BALL_PICKUP_INITIAL_SEARCH_FORWARD_4":
            motion_id = "ball_camera_down_forward_4"
        elif action == "BALL_PICKUP_INITIAL_SEARCH_FORWARD":
            motion_id = "ball_camera_down_forward_2"
        elif action.startswith("BALL_PICKUP_INITIAL_CRAB_"):
            fine_action = action.replace("INITIAL_", "", 1)
            motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[fine_action]
        else:
            motion_id = self.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS[action]
        self.pickup_initial_align_correction_active = True
        self.active_motion_id = motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )

    def _handle_pickup_fine_align_command(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Apply one Vision decision inside the active pickup checkpoint."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        checkpoint_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and self.pickup_fine_align_waiting
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
        )
        if not checkpoint_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="PICKUP_FINE_ALIGNMENT_NOT_ACTIVE",
                message="pickup fine-alignment checkpoint is not active",
            )
            return

        self.last_sent_command_id = command_id
        self.pickup_fine_align_waiting = False
        if action == "BALL_PICKUP_FINE_ALIGN_CONTINUE":
            self.pickup_fixed_sequence_started = True
            self.active_pickup_sequence = self._with_pickup_motion_dwells(
                self.active_pickup_sequence
            )
            self.active_sequence_index = -1
            if not self._start_next_pickup_motion():
                self.pickup_fixed_sequence_started = False
                self.pickup_fine_align_waiting = True
                self._publish_local_rejection(
                    status="REJECTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="PICKUP_FINE_ALIGNMENT_RESUME_FAILED",
                    message="pickup sequence could not resume after alignment",
                )
            return

        motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[action]
        # Intentionally keep no crab retry counter. Each completed correction
        # returns to the fresh pixel checkpoint until Vision reports centered.
        self.pickup_fine_align_correction_active = True
        self.active_motion_id = motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )

    def _enter_pickup_positioning_loss_reacquisition(
        self,
        checkpoint: str | None,
    ) -> None:
        """Resume pickup positioning at the existing camera-down checkpoint."""
        if checkpoint == self.FINE_ALIGN_MARKER:
            remaining_index = self.active_sequence_index
            if remaining_index >= 0:
                self.active_pickup_sequence = self.active_pickup_sequence[
                    remaining_index:
                ]
        elif checkpoint != self.PICKUP_INITIAL_ALIGN_MARKER:
            remaining_index = self.active_sequence_index + 1
            self.active_pickup_sequence = self.active_pickup_sequence[
                remaining_index:
            ]
        self.active_sequence_index = 0
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        if (
            checkpoint == self.FINE_ALIGN_MARKER
            or self.pickup_fine_positioning_complete
        ):
            self._enter_pickup_fine_align_checkpoint()
        else:
            self._enter_pickup_initial_align_checkpoint()

    def _handle_pickup_positioning_loss_latch(
        self,
        *,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Latch Vision loss without interrupting the active pickup motion."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        source_command = payload.get("source_command")
        source_motion_id = (
            source_command.get("pickup_positioning_motion_id")
            if isinstance(source_command, dict)
            else None
        )
        actual_motion_running = bool(
            self.active_dwell_until is None
            and self.pickup_initial_align_dwell_until is None
            and isinstance(self.active_motion_id, str)
            and not self.active_motion_id.startswith("__")
        )
        request_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and not self.pickup_fixed_sequence_started
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
            and isinstance(source_command, dict)
            and source_command.get("pickup_positioning_ball_lost") is True
            and (
                actual_motion_running
                or self.pickup_fine_align_waiting
                or (
                    self.active_dwell_until is not None
                    and source_motion_id
                    == self.pickup_positioning_dwell_motion_id
                )
            )
        )
        if not request_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=self.PICKUP_POSITIONING_LOSS_LATCH_ACTION,
                error_code="PICKUP_POSITIONING_LOSS_LATCH_NOT_ACTIVE",
                message="pickup positioning motion is not active",
            )
            return

        self.last_sent_command_id = command_id
        self.pickup_positioning_loss_pending = True
        if self.pickup_fine_align_waiting:
            self.pickup_fine_align_waiting = False
            self._enter_pickup_positioning_loss_reacquisition(
                self.FINE_ALIGN_MARKER
            )

    def _handle_pickup_post_backward_align_command(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Apply camera-down heading or lateral pickup alignment."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        checkpoint_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and self.pickup_post_backward_align_waiting
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
        )
        if not checkpoint_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="PICKUP_POST_BACKWARD_ALIGNMENT_NOT_ACTIVE",
                message="post-backward pickup alignment is not active",
            )
            return

        self.last_sent_command_id = command_id
        self.pickup_post_backward_align_waiting = False
        if action == "BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE":
            if not self._start_next_pickup_motion():
                self.pickup_post_backward_align_waiting = True
                self._publish_local_rejection(
                    status="REJECTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="PICKUP_POST_BACKWARD_ALIGNMENT_RESUME_FAILED",
                    message="pickup sequence could not resume after alignment",
                )
            return

        if action.startswith("BALL_PICKUP_POST_BACKWARD_TURN_"):
            initial_action = action.replace(
                "BALL_PICKUP_POST_BACKWARD_TURN_",
                "BALL_PICKUP_CAMERA_DOWN_TURN_",
                1,
            )
            motion_id = self.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS[initial_action]
        else:
            fine_action = action.replace(
                "BALL_PICKUP_POST_BACKWARD_CRAB_",
                "BALL_PICKUP_CRAB_",
                1,
            )
            motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[fine_action]
        self.pickup_post_backward_align_correction_active = True
        self.active_motion_id = motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )

    def executor_heartbeat_callback(self, msg: String) -> None:
        """Read camera ownership for the pre-turn posture choice."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.head_override_state = payload
            self.head_override_received_at = time.monotonic()

    def _is_stationary_turn(self, motion_id: str) -> bool:
        """Exclude walking corrections and composite retreat/exit motions."""
        return motion_id.startswith((
            "stationary_turn_", "line_turn_", "line_search_",
            "post_ball_line_turn_", "goal_camera_90_turn_",
            "pickup_camera_down_turn_", "pickup_first_turn_", "pickup_second_turn_",
        )) or motion_id == "post_shot_turn_right_9"

    def _is_forward_motion(self, motion_id: str) -> bool:
        return motion_id in {"forward", "sdk_forward_4"} or motion_id.startswith((
            "line_forward_", "ball_camera_down_forward_", "post_ball_forward_",
            "goal_camera_90_forward_",
        ))

    def _turn_prepare_motion(self, motion_id: str) -> str | None:
        # Forward walking and stationary turns share the same right-back posture.
        if (self.last_physical_motion_id not in self.FINE_FORWARD_MOTION_IDS
                or not (self._is_stationary_turn(motion_id)
                        or self._is_forward_motion(motion_id))):
            return None
        state = self.head_override_state
        if (self.head_override_received_at is None
                or time.monotonic() - self.head_override_received_at > 1.0):
            state = {}
        if (state.get("goal_head_override_active") is True
                or motion_id.startswith("goal_camera_90_")):
            return "fine_to_turn_ready_90"
        if (state.get("ball_head_override_active") is True
                or state.get("hurdle_head_override_active") is True
                or motion_id.startswith("pickup_camera_down_turn_")
                or self.last_physical_motion_id in {
                    "pickup_fine_forward_0", self.HURDLE_FINAL_FINE_MOTION_ID,
                }):
            return "fine_to_turn_ready_0"
        return "fine_to_turn_ready_45"

    def _transition_prepare_motion(self, motion_id: str) -> str | None:
        """Choose a posture transition from the last successfully completed gait."""
        previous = self.last_physical_motion_id or ""
        prepare = self._turn_prepare_motion(motion_id)
        if prepare is not None:
            return prepare
        if (motion_id in self.FINE_FORWARD_MOTION_IDS
                and (self._is_forward_motion(previous)
                     or self._is_stationary_turn(previous))):
            return self.PICKUP_FINE_PREPARE_MOTION_ID
        if motion_id in {"pickup_crab_left_0", "pickup_crab_right_0"}:
            if previous in self.FINE_FORWARD_MOTION_IDS:
                return self.PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID
            if self._is_forward_motion(previous):
                return "pickup_forward_to_crab_0"
        if motion_id in {"goal_camera_90_crab_left", "goal_camera_90_crab_right"}:
            if previous in self.FINE_FORWARD_MOTION_IDS:
                return self.GOAL_FINE_RIGHT_CRAB_PREPARE_MOTION_ID
            if self._is_forward_motion(previous):
                return self.GOAL_FORWARD_CRAB_PREPARE_MOTION_ID
        return None

    def _publish_executor_request(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        request_id: int,
        motion_id: str,
        timeout_ms: int,
        prepare_pickup_fine: bool = True,
    ) -> None:
        """Publish one validated request to the C++ executor."""
        if action == "POST_BALL_GOAL_TRANSITION" and motion_id == "post_ball_camera_90":
            self.post_ball_camera_pause_until = (
                time.monotonic() + MotionCommandBridgeNode.POST_BALL_CAMERA_PAUSE_SEC
            )
        if action == "PICKUP_NOW" and motion_id in {
            "pickup_crab_left_0", "pickup_crab_right_0",
        }:
            prepare = self._transition_prepare_motion(motion_id)
            if prepare is not None:
                self.pending_pickup_crab_motion_id = motion_id
                motion_id = prepare
                self.active_motion_id = prepare
        if (
            prepare_pickup_fine
            and action == "PICKUP_NOW"
            and motion_id == "pickup_fine_forward_0"
        ):
            # Keep the existing checkpoint/sequence pending until both motions finish.
            motion_id = self.PICKUP_FINE_PREPARE_MOTION_ID
            self.active_motion_id = motion_id
        request_payload = {
            "action": action,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "timeout_ms": timeout_ms,
        }
        prepare_motion = (
            self._transition_prepare_motion(motion_id)
            if not self.motion_in_progress or request_id == self.active_request_id
            else None
        )
        if prepare_motion is not None:
            # Reuse the existing preparation lock for all six gait transitions.
            self.pending_turn_request = dict(request_payload)
            self.turn_prepare_motion_id = prepare_motion
            request_payload["motion_id"] = prepare_motion
            self.get_logger().info(
                f"Preparing motion transition: {prepare_motion} -> {motion_id}"
            )
        if request_payload["motion_id"] in MotionCommandBridgeNode.TRANSITION_MOTION_IDS:
            self.transition_motion_id = request_payload["motion_id"]
            self.transition_pending_request = request_payload
            self.transition_dwell_until = (
                time.monotonic() + MotionCommandBridgeNode.TRANSITION_DWELL_SEC
            )
            self.publish_motion_status(
                status="RUNNING", action=action, command_id=command_id,
                event_id=event_id, request_id=request_id,
                motion_id="__TRANSITION_PRE_DWELL__",
                message="holding still for one second before posture transition",
            )
            return
        self._send_executor_payload(request_payload)
        if action == "PICKUP_NOW" and motion_id in {
            self.PICKUP_FINE_PREPARE_MOTION_ID, "pickup_fine_forward_0",
        }:
            self.get_logger().info(
                f"Pickup fine motion requested: motion_id={motion_id}, "
                f"command_id={command_id}, request_id={request_id}"
            )

    def _send_executor_payload(self, request_payload: dict[str, Any]) -> None:
        """Send an already prepared payload without inserting another transition."""
        request_message = String()
        request_message.data = json.dumps(
            request_payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.executor_request_publisher.publish(request_message)

    def _check_transition_dwell(self, now: float | None = None) -> bool:
        """Keep the command locked through both sides of a posture transition."""
        deadline = self.transition_dwell_until
        if deadline is None:
            return False
        if (time.monotonic() if now is None else now) < deadline:
            return True
        self.transition_dwell_until = None
        if self.transition_pending_request is not None:
            request = self.transition_pending_request
            self.transition_pending_request = None
            self._send_executor_payload(request)
        elif self.transition_pending_status is not None:
            status = self.transition_pending_status
            self.transition_pending_status = None
            self.transition_motion_id = None
            # Resume the existing sequence/checkpoint logic only after settling.
            message = String()
            message.data = json.dumps(status)
            self.executor_status_callback(message)
        return True

    def _transition_has_existing_post_dwell(self, motion_id: str) -> bool:
        if self.active_motion_id != motion_id:
            return False
        if (
            self.active_action == "PICKUP_NOW"
            and motion_id == self.PICKUP_FINE_PREPARE_MOTION_ID
        ):
            return True
        next_index = self.active_sequence_index + 1
        return bool(
            self.active_action in self.GOAL_CRAB_ACTIONS
            and next_index < len(self.active_pickup_sequence)
            and self.active_pickup_sequence[next_index]
            == MotionCommandBridgeNode.GOAL_CRAB_PRE_DWELL_MARKER
        )

    def navigation_command_callback(self, msg: String) -> None:
        """Validate and translate one navigation command."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(
                f"Invalid /navigation/motion_command JSON: {exc}"
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().warning(
                "Invalid /navigation/motion_command: "
                "JSON root must be an object"
            )
            return

        action = payload.get("action")
        command_id = payload.get("command_id")
        event_id = payload.get("event_id")
        if payload.get("valid") is not True:
            command = payload.get("source_command")
            checks = command.get("line_input_checks") if isinstance(command, dict) else None
            if checks and checks.get("failed_checks"):
                self.get_logger().info(f"Line input rejected: {checks['failed_checks']}")
            self.get_logger().info(
                f"Command ignored: valid is not true, action={action}, "
                f"reason={payload.get('reason')}, line_input_checks={checks}"
            )
            return
        if not self._is_integer(command_id):
            self.get_logger().warning(
                "Command ignored: command_id is missing or not an integer"
            )
            return
        if event_id is not None and not self._is_integer(event_id):
            self.get_logger().warning(
                "Command ignored: event_id is not an integer or null"
            )
            return
        if not isinstance(action, str) or not action:
            self.get_logger().warning(
                "Command ignored: action is missing or invalid"
            )
            return

        if action == self.PICKUP_POSITIONING_LOSS_LATCH_ACTION:
            self._handle_pickup_positioning_loss_latch(
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return

        if action in self.PICKUP_FINE_ALIGN_ACTIONS:
            self._handle_pickup_fine_align_command(
                action=action,
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return
        if action in self.PICKUP_POST_BACKWARD_ALIGN_ACTIONS:
            self._handle_pickup_post_backward_align_command(
                action=action,
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return
        if action in self.PICKUP_INITIAL_ALIGN_ACTIONS:
            self._handle_pickup_initial_align_command(
                action=action,
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return

        if command_id == self.last_sent_command_id:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="DUPLICATE_COMMAND_ID",
                message="command_id was already sent to the executor",
            )
            return
        if (
            self.motion_in_progress
            and (self.active_action in self.ATOMIC_SEQUENCE_ACTIONS
                 or self.pending_turn_request is not None)
        ):
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="ATOMIC_SEQUENCE_LOCKED",
                message="an atomic motion sequence owns the motion executor",
            )
            return
        if self.motion_in_progress and action in {
            "SHOT", "GO", "POST_BALL_GOAL_TRANSITION", *self.GOAL_CRAB_ACTIONS,
        }:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="REJECTED_BUSY",
                message=f"{action} requires the current motion to finish first",
            )
            return
        if self.motion_in_progress and self.queued_request_id is not None:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="QUEUE_FULL",
                message="one next motion is already queued",
            )
            return

        pickup_sequence: tuple[str, ...] = ()
        if action == "PICKUP_NOW":
            built_sequence = self._pickup_motion_sequence(payload)
            if built_sequence is None:
                self._publish_local_rejection(
                    status="UNSUPPORTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="UNSUPPORTED_PICKUP_SEQUENCE",
                    message=(
                        "pickup sequence requires ball mission index 0 or 1"
                    ),
                )
                return
            pickup_sequence = built_sequence
            motion_id = self.PICKUP_INITIAL_ALIGN_MARKER
            self.goal_fine_forward_completed = False
            self.goal_crab_completed = False
            self.last_completed_motion_id = None
        elif action == "GO":
            # Approach steps are separate commands and do not count toward this tail.
            # On a failed tail, retry only the stages not already completed.
            self.hurdle_fine_sequence_pending = True
            fine_steps = (
                (self.HURDLE_FINAL_FINE_MOTION_ID,)
                if self.hurdle_sequence_fine_completed == 0 else ()
            )
            pickup_sequence = fine_steps + (
                self.HURDLE_PRE_GO_DWELL_MARKER, "hurdle",
            )
            motion_id = pickup_sequence[0]
        elif action == "POST_BALL_GOAL_TRANSITION":
            pickup_sequence = self.POST_BALL_GOAL_TRANSITION_SEQUENCE
            motion_id = pickup_sequence[0]
        elif action in self.GOAL_CRAB_ACTIONS:
            motion_id = self.motion_id_for_action(action)
            prepare_motion = self._transition_prepare_motion(motion_id)
            pickup_sequence = (
                MotionCommandBridgeNode.GOAL_CRAB_PRE_DWELL_MARKER, motion_id,
            )
            if prepare_motion is not None:
                pickup_sequence = (prepare_motion,) + pickup_sequence
            motion_id = pickup_sequence[0]
        elif (
            action == "SHOT"
            and self.goal_fine_forward_completed
            and not self.goal_crab_completed
        ):
            pickup_sequence = (
                self.SHOT_PREPARE_MOTION_ID,
                self.SHOT_PREPARE_DWELL_MARKER,
                "goal_shot",
            )
            motion_id = pickup_sequence[0]
        elif payload.get("source") == "hurdle" and action in {
            "ALIGN_LEFT", "ALIGN_RIGHT",
        }:
            source_command = payload.get("source_command")
            count = (
                source_command.get("turn_count")
                if isinstance(source_command, dict) else None
            )
            direction = "LEFT" if action == "ALIGN_LEFT" else "RIGHT"
            # Reuse calibrated camera-45 turns, keeping the hurdle action ID.
            motion_id = (
                self.motion_id_for_action(f"POST_BALL_LINE_TURN_{direction}_{count}")
                if self._is_integer(count) else None
            )
        elif payload.get("source") in ("ball", "hurdle") and action in {
            "STRAIGHT", "STRAIGHT_0",
        }:
            # Hurdle fine approach uses camera-0; Ball keeps its camera-45 gait.
            motion_id = (
                "line_forward_4" if action == "STRAIGHT"
                else "pickup_fine_forward_0" if payload.get("source") == "hurdle"
                else "ball_general_fine_forward_8"
            )
        else:
            motion_id = self.motion_id_for_action(action)
        if motion_id is None:
            goal_left_unavailable = action.startswith(
                "GOAL_CAMERA90_TURN_LEFT_"
            )
            self._publish_local_rejection(
                status="UNSUPPORTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code=(
                    "GOAL_CAMERA90_LEFT_TURN_NOT_AVAILABLE"
                    if goal_left_unavailable
                    else "UNSUPPORTED_ACTION"
                ),
                message=(
                    "camera-90 goal left-turn motion is not in the catalog"
                    if goal_left_unavailable
                    else "action has no configured motion alias"
                ),
            )
            return

        request_id = command_id
        timeout_ms = self.timeout_ms_from_payload(payload)
        defer_until_active_finishes = bool(
            self.motion_in_progress
            and (action == "PICKUP_NOW" or self._is_stationary_turn(motion_id)
                 or (motion_id in self.FINE_FORWARD_MOTION_IDS
                     and (self._is_forward_motion(self.active_motion_id or "")
                          or self._is_stationary_turn(self.active_motion_id or "")))
                 or (self.active_motion_id in self.FINE_FORWARD_MOTION_IDS
                     and self._is_forward_motion(motion_id)))
        )
        starts_with_initial_align_checkpoint = action == "PICKUP_NOW"
        starts_with_hurdle_dwell = (
            action == "GO" and motion_id == self.HURDLE_PRE_GO_DWELL_MARKER
        )
        starts_with_crab_dwell = (
            motion_id == MotionCommandBridgeNode.GOAL_CRAB_PRE_DWELL_MARKER
        )
        if (
            not defer_until_active_finishes
            and not starts_with_initial_align_checkpoint
            and not starts_with_hurdle_dwell
            and not starts_with_crab_dwell
        ):
            self._publish_executor_request(
                action=action,
                command_id=command_id,
                event_id=event_id,
                request_id=request_id,
                motion_id=motion_id,
                timeout_ms=timeout_ms,
            )

        self.last_sent_command_id = command_id
        if self.motion_in_progress:
            self.queued_command_id = command_id
            self.queued_event_id = event_id
            self.queued_action = action
            self.queued_request_id = request_id
            self.queued_motion_id = motion_id
            self.queued_timeout_ms = timeout_ms
            self.queued_request_deferred = defer_until_active_finishes
            self.queued_pickup_sequence = pickup_sequence
        else:
            self.motion_in_progress = True
            self.active_command_id = command_id
            self.active_event_id = event_id
            self.active_action = action
            self.active_request_id = request_id
            self.active_motion_id = motion_id
            self.active_timeout_ms = timeout_ms
            self.active_pickup_sequence = pickup_sequence
            self.active_sequence_index = 0
            self.active_dwell_until = None
            self.pickup_positioning_loss_pending = False
            self.pickup_fixed_sequence_started = False
            self.pickup_fine_positioning_complete = False
            self.last_pickup_motion_id = None
            self.pickup_consecutive_motor_failures = 0
            self.pickup_motor_failures = []
            self.pickup_last_stage_succeeded = True
            self.pending_pickup_crab_motion_id = None
            self.pickup_positioning_dwell_motion_id = None
            if starts_with_initial_align_checkpoint:
                self.active_sequence_index = -1
                self._start_pickup_initial_align_dwell()
            elif starts_with_hurdle_dwell or starts_with_crab_dwell:
                self.active_sequence_index = -1
                self._start_next_pickup_motion()

    def _start_next_pickup_motion(self) -> bool:
        """Advance an atomic pickup, goal, transition, shot, or hurdle sequence."""
        if self.active_action not in self.ATOMIC_SEQUENCE_ACTIONS:
            return False
        if self.active_dwell_until is not None:
            return True
        next_index = self.active_sequence_index + 1
        if next_index >= len(self.active_pickup_sequence):
            return False

        next_motion_id = self.active_pickup_sequence[next_index]
        self.active_sequence_index = next_index
        if next_motion_id == MotionCommandBridgeNode.GOAL_CRAB_PRE_DWELL_MARKER:
            self.active_motion_id = next_motion_id
            self.active_dwell_until = (
                time.monotonic() + MotionCommandBridgeNode.GOAL_CRAB_PRE_DWELL_SEC
            )
            self.publish_motion_status(
                status="RUNNING",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=next_motion_id,
                action=self.active_action,
                message="holding still for one second before goal crab step",
            )
            self.get_logger().info("Goal crab pre-motion dwell started: 1.0s")
            return True
        if next_motion_id == MotionCommandBridgeNode.POST_BALL_CAMERA_DWELL_MARKER:
            self.active_motion_id = next_motion_id
            self.active_dwell_until = self.post_ball_camera_pause_until
            self.publish_motion_status(
                status="RUNNING",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=next_motion_id,
                action=self.active_action,
                message="camera raised; completing goal transition",
            )
            if MotionCommandBridgeNode.POST_BALL_CAMERA_PAUSE_SEC <= 0.0:
                self._check_atomic_dwell()
            return True
        if next_motion_id == self.HURDLE_PRE_GO_DWELL_MARKER:
            self.active_motion_id = next_motion_id
            self.active_dwell_until = time.monotonic() + self.HURDLE_PRE_GO_DWELL_SEC
            self.publish_motion_status(
                status="RUNNING",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=next_motion_id,
                action=self.active_action,
                message="holding still for one second before hurdle",
            )
            return True
        if next_motion_id == self.SHOT_PREPARE_DWELL_MARKER:
            self.active_motion_id = next_motion_id
            self.active_dwell_until = time.monotonic() + self.SHOT_PREPARE_DWELL_SEC
            self.publish_motion_status(
                status="RUNNING",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=next_motion_id,
                action=self.active_action,
                message="holding still for two seconds after goal pose preparation",
            )
            return True
        if next_motion_id == self.FINE_ALIGN_MARKER:
            self._enter_pickup_fine_align_checkpoint()
            return True
        if next_motion_id == self.POST_BACKWARD_ALIGN_MARKER:
            self._enter_pickup_post_backward_align_checkpoint()
            return True
        if next_motion_id == self.DWELL_MARKER:
            if self.active_action == "GO":
                # Ignore repeated completion messages while the fine-step pause owns the sequence.
                self.active_motion_id = self.DWELL_MARKER
            if self.active_action == "PICKUP_NOW":
                self.pickup_positioning_dwell_motion_id = (
                    self.active_motion_id
                )
            grasp_check = (
                self.active_action == "PICKUP_NOW"
                and self.pickup_positioning_dwell_motion_id
                == self.PICKUP_GRASP_CHECK_MOTION_ID
            )
            dwell_sec = (
                MotionCommandBridgeNode.PICKUP_GRASP_DWELL_SEC
                if grasp_check else self.PICKUP_DWELL_SEC
            )
            self.active_dwell_until = time.monotonic() + dwell_sec
            if self.active_action == "PICKUP_NOW":
                self.publish_motion_status(
                    status="RUNNING",
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=self.DWELL_MARKER,
                    action=self.active_action,
                    message="pickup sequence non-blocking dwell active",
                    completed_motion_id=(
                        self.pickup_positioning_dwell_motion_id
                        if self.pickup_last_stage_succeeded else None
                    ),
                    verification_window_complete=(
                        False if grasp_check and self.pickup_last_stage_succeeded else None
                    ),
                )
            self.get_logger().info(
                f"Atomic sequence dwell started: {dwell_sec:.1f}s"
            )
            if dwell_sec <= 0.0:
                self._check_atomic_dwell(self.active_dwell_until)
            return True
        if (
            self.active_action == "PICKUP_NOW"
            and next_motion_id == self.PICKUP_FIXED_SEQUENCE_FIRST_MOTION
        ):
            self.pickup_fixed_sequence_started = True
        self.active_motion_id = next_motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=next_motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )
        self.get_logger().info(
            f"Atomic sequence advanced to {next_motion_id}"
        )
        return True

    def _check_atomic_dwell(self, now: float | None = None) -> None:
        """Advance a dwell stage without blocking ROS callbacks."""
        if self._check_transition_dwell(now):
            return
        if (
            self.active_dwell_until is None
            and self.pickup_initial_align_dwell_until is None
        ):
            return
        current_time = time.monotonic() if now is None else now
        if self.pickup_initial_align_dwell_until is not None:
            if current_time < self.pickup_initial_align_dwell_until:
                return
            self.pickup_initial_align_dwell_until = None
            self._enter_pickup_initial_align_checkpoint()
            return

        assert self.active_dwell_until is not None
        if current_time < self.active_dwell_until:
            return

        self.active_dwell_until = None
        if self.active_motion_id == MotionCommandBridgeNode.PICKUP_FINE_PRE_DWELL_MARKER:
            self.pickup_positioning_dwell_motion_id = None
            if self.pickup_positioning_loss_pending:
                checkpoint = (
                    self.PICKUP_INITIAL_ALIGN_MARKER
                    if self.pickup_initial_align_correction_active else self.FINE_ALIGN_MARKER
                )
                self._enter_pickup_positioning_loss_reacquisition(checkpoint)
                return
            self.active_motion_id = "pickup_fine_forward_0"
            self._publish_executor_request(
                action=self.active_action, command_id=self.active_command_id,
                event_id=self.active_event_id, request_id=self.active_request_id,
                motion_id=self.active_motion_id,
                timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
                prepare_pickup_fine=False,
            )
            return
        completed_motion_id = self.pickup_positioning_dwell_motion_id
        if (
            self.active_action == "PICKUP_NOW"
            and completed_motion_id == self.PICKUP_GRASP_CHECK_MOTION_ID
            and self.pickup_last_stage_succeeded
        ):
            self.publish_motion_status(
                status="RUNNING",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=self.DWELL_MARKER,
                action=self.active_action,
                message="grasp verification dwell completed",
                completed_motion_id=completed_motion_id,
                verification_window_complete=True,
            )
        self.pickup_positioning_dwell_motion_id = None
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        checkpoint = self.pickup_checkpoint_after_dwell
        self.pickup_checkpoint_after_dwell = None
        if self.pickup_positioning_loss_pending:
            self._enter_pickup_positioning_loss_reacquisition(checkpoint)
            return
        if checkpoint == self.PICKUP_INITIAL_ALIGN_MARKER:
            self._enter_pickup_initial_align_checkpoint()
            return
        if checkpoint == self.FINE_ALIGN_MARKER:
            self._enter_pickup_fine_align_checkpoint()
            return
        if checkpoint == self.POST_BACKWARD_ALIGN_MARKER:
            self._enter_pickup_post_backward_align_checkpoint()
            return
        if self._start_next_pickup_motion():
            return

        command_id = self.active_command_id
        event_id = self.active_event_id
        request_id = self.active_request_id
        motion_id = self.active_motion_id
        action = self.active_action
        motor_failures = self.pickup_motor_failures
        self._clear_active_request()
        self._promote_queued_request()
        self.publish_motion_status(
            status="SUCCEEDED",
            command_id=command_id,
            event_id=event_id,
            request_id=request_id,
            motion_id=motion_id,
            action=action,
            message="atomic sequence completed after non-blocking dwell",
            motor_failures=motor_failures,
        )

    def _valid_executor_status(self, payload: dict[str, Any]) -> bool:
        """Validate fields emitted by the C++ executor."""
        status = payload.get("status")
        command_id = payload.get("command_id")
        event_id = payload.get("event_id")
        request_id = payload.get("request_id")
        motion_id = payload.get("motion_id")
        error_code = payload.get("error_code")
        message = payload.get("message")
        return (
            status in {"QUEUED", "RUNNING", *self.TERMINAL_STATUSES}
            and (command_id is None or self._is_integer(command_id))
            and (event_id is None or self._is_integer(event_id))
            and self._is_integer(request_id)
            and isinstance(motion_id, str)
            and isinstance(error_code, str)
            and isinstance(message, str)
        )

    def _clear_active_request(self) -> None:
        """Release the bridge after a terminal executor status."""
        self.motion_in_progress = False
        self.pending_turn_request = None
        self.turn_prepare_motion_id = None
        self.transition_dwell_until = None
        self.transition_pending_request = None
        self.transition_pending_status = None
        self.transition_motion_id = None
        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_request_id = None
        self.active_motion_id = None
        self.active_timeout_ms = None
        self.active_pickup_sequence = ()
        self.active_sequence_index = 0
        self.active_dwell_until = None
        self.post_ball_camera_pause_until = None
        self.pickup_initial_align_dwell_until = None
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_checkpoint_after_dwell = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.last_pickup_motion_id = None
        self.pickup_consecutive_motor_failures = 0
        self.pickup_motor_failures = []
        self.pickup_last_stage_succeeded = True
        self.pending_pickup_crab_motion_id = None
        self.pickup_positioning_dwell_motion_id = None

    def _clear_queued_request(self) -> None:
        """Discard one queued request and any associated pickup sequence."""
        self.queued_command_id = None
        self.queued_event_id = None
        self.queued_action = None
        self.queued_request_id = None
        self.queued_motion_id = None
        self.queued_timeout_ms = None
        self.queued_request_deferred = False
        self.queued_pickup_sequence = ()

    def _promote_queued_request(self) -> None:
        self.active_command_id = self.queued_command_id
        self.active_event_id = self.queued_event_id
        self.active_action = self.queued_action
        self.active_request_id = self.queued_request_id
        self.active_motion_id = self.queued_motion_id
        self.active_timeout_ms = self.queued_timeout_ms
        self.active_pickup_sequence = self.queued_pickup_sequence
        self.active_sequence_index = 0
        self.active_dwell_until = None
        self.post_ball_camera_pause_until = None
        self.pickup_initial_align_dwell_until = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.last_pickup_motion_id = None
        self.pickup_consecutive_motor_failures = 0
        self.pickup_motor_failures = []
        self.pickup_last_stage_succeeded = True
        self.pending_pickup_crab_motion_id = None
        self.pickup_positioning_dwell_motion_id = None
        queued_request_deferred = self.queued_request_deferred
        self._clear_queued_request()
        self.motion_in_progress = self.active_request_id is not None
        if queued_request_deferred and self.motion_in_progress:
            if self.active_action == "PICKUP_NOW":
                self._start_pickup_initial_align_dwell()
            else:
                self._publish_executor_request(
                    action=self.active_action,
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=self.active_motion_id,
                    timeout_ms=(
                        self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS
                    ),
                )

    def executor_status_callback(self, msg: String) -> None:
        """Forward matching executor status and release terminal requests."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(
                f"Invalid /motion/executor/status JSON: {exc}"
            )
            return
        if (
            not isinstance(payload, dict)
            or not self._valid_executor_status(payload)
        ):
            self.get_logger().warning(
                "Invalid /motion/executor/status fields"
            )
            return
        if not self.motion_in_progress:
            self.get_logger().info(
                "Executor status ignored: bridge has no active request"
            )
            return
        is_active = payload["request_id"] == self.active_request_id
        is_queued = payload["request_id"] == self.queued_request_id
        if not is_active and not is_queued:
            self.get_logger().warning(
                "Executor status ignored: request_id mismatch"
            )
            return

        action = self.active_action if is_active else self.queued_action
        if is_active and self.transition_motion_id is not None:
            if payload["motion_id"] != self.transition_motion_id:
                return
            if self.transition_dwell_until is not None:
                # Neither an unsent preparation nor duplicate completion advances a pause.
                return
            if payload["status"] == "SUCCEEDED":
                if not self._transition_has_existing_post_dwell(payload["motion_id"]):
                    self.transition_pending_status = dict(payload)
                    self.transition_dwell_until = (
                        time.monotonic() + MotionCommandBridgeNode.TRANSITION_DWELL_SEC
                    )
                    self.publish_motion_status(
                        status="RUNNING", action=action, command_id=self.active_command_id,
                        event_id=self.active_event_id, request_id=self.active_request_id,
                        motion_id="__TRANSITION_POST_DWELL__",
                        message="holding still for one second after posture transition",
                    )
                    return
                self.transition_motion_id = None
            elif payload["status"] in self.TERMINAL_STATUSES:
                self.transition_motion_id = None
        if (
            is_active and action == "PICKUP_NOW"
            and self.active_dwell_until is not None
        ):
            # A duplicate terminal status is not another failed motion.
            return
        if is_active and self.pending_turn_request is not None:
            if payload["motion_id"] != self.turn_prepare_motion_id:
                return
            pending = self.pending_turn_request
            if payload["status"] == "SUCCEEDED":
                self.last_physical_motion_id = self.turn_prepare_motion_id
                if action == "PICKUP_NOW":
                    self.pickup_consecutive_motor_failures = 0
                self.pending_turn_request = None
                self.turn_prepare_motion_id = None
                self._publish_executor_request(**pending, prepare_pickup_fine=False)
                return
            payload["motion_id"] = pending["motion_id"]
            if payload["status"] in self.TERMINAL_STATUSES:
                self.last_physical_motion_id = None
                self.pending_turn_request = None
                self.turn_prepare_motion_id = None
                # Never let pickup's recoverable-motor policy skip preparation.
                payload["message"] = (
                    "motion transition preparation failed: "
                    + payload["error_code"] + " " + payload["message"]
                )
                payload["error_code"] = (
                    "TURN_PREPARATION_FAILED"
                    if self._is_stationary_turn(pending["motion_id"])
                    else "MOTION_PREPARATION_FAILED"
                )
        elif is_active and payload["motion_id"] == self.active_motion_id:
            if payload["status"] == "SUCCEEDED":
                self.last_physical_motion_id = payload["motion_id"]
            elif payload["status"] in self.TERMINAL_STATUSES:
                self.last_physical_motion_id = None
        elif is_active and payload["motion_id"] in {
            "fine_to_turn_ready_0", "fine_to_turn_ready_45", "fine_to_turn_ready_90",
            self.PICKUP_FINE_PREPARE_MOTION_ID,
            self.PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID,
            "pickup_forward_to_crab_0",
        }:
            # Duplicate preparation completions must not finish the target motion.
            return
        if (
            is_active
            and action in self.ATOMIC_SEQUENCE_ACTIONS
            and payload["motion_id"] != self.active_motion_id
        ):
            self.get_logger().warning(
                "Executor status ignored: atomic motion_id mismatch"
            )
            return
        if is_active and action == "PICKUP_NOW":
            if payload["status"] == "SUCCEEDED":
                self.pickup_consecutive_motor_failures = 0
                self.pickup_last_stage_succeeded = True
            elif payload["status"] == "FAILED":
                self.pickup_last_stage_succeeded = False
                self.last_pickup_motion_id = None
                self.pickup_consecutive_motor_failures += 1
                self.pickup_motor_failures.append({
                    "motion_id": payload["motion_id"],
                    "status": "FAILED",
                    "error_code": payload["error_code"],
                    "message": payload["message"],
                })
        continue_after_motor_fault = False
        if (
            is_active
            and action == "PICKUP_NOW"
            and payload["motion_id"] != "post_ball_camera_90"
            and payload["motion_id"] != self.PICKUP_FINE_PREPARE_MOTION_ID
            and payload["motion_id"] != self.PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID
            and payload["motion_id"] != "pickup_forward_to_crab_0"
            and not (
                self.pickup_fixed_sequence_started
                and payload["motion_id"] == "pickup_fine_forward_0"
            )
            and payload["status"] == "FAILED"
            and payload["error_code"] in RECOVERABLE_MOTOR_ERROR_CODES
        ):
            continue_after_motor_fault = (
                self.pickup_consecutive_motor_failures
                < MotionCommandBridgeNode.PICKUP_MOTOR_FAILURE_LIMIT
            )
            if not continue_after_motor_fault:
                payload["message"] = (
                    "consecutive motor failures; pickup sequence stopped: "
                    + payload["message"]
                )
            self.get_logger().warning(
                f"Pickup motor fault: motion_id={payload['motion_id']}, "
                f"error_code={payload['error_code']}, "
                f"continue_sequence={continue_after_motor_fault}"
            )
        stage_can_advance = (
            payload["status"] == "SUCCEEDED" or continue_after_motor_fault
        )
        if is_active and action == "GO" and payload["status"] == "SUCCEEDED":
            if payload["motion_id"] == self.HURDLE_FINAL_FINE_MOTION_ID:
                self.hurdle_depth_fine_completed = True
                self.hurdle_sequence_fine_completed = 1
            elif payload["motion_id"] == "hurdle":
                self.hurdle_depth_fine_completed = False
                self.hurdle_sequence_fine_completed = 0
                self.hurdle_fine_sequence_pending = False
        if (
            is_active
            and action == "PICKUP_NOW"
            and payload["status"] == "SUCCEEDED"
        ):
            self.last_pickup_motion_id = payload["motion_id"]
            if (
                payload["motion_id"] in {
                    self.PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID,
                    "pickup_forward_to_crab_0",
                }
                and self.pending_pickup_crab_motion_id is not None
            ):
                self.active_motion_id = self.pending_pickup_crab_motion_id
                self.pending_pickup_crab_motion_id = None
                self._publish_executor_request(
                    action=action,
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=self.active_motion_id,
                    timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
                )
                return
        if (
            is_active
            and action == "PICKUP_NOW"
            and payload["motion_id"] == self.PICKUP_FINE_PREPARE_MOTION_ID
            and self.active_motion_id == self.PICKUP_FINE_PREPARE_MOTION_ID
            and payload["status"] == "SUCCEEDED"
        ):
            self.active_motion_id = MotionCommandBridgeNode.PICKUP_FINE_PRE_DWELL_MARKER
            self.pickup_positioning_dwell_motion_id = self.PICKUP_FINE_PREPARE_MOTION_ID
            self.active_dwell_until = (
                time.monotonic() + MotionCommandBridgeNode.PICKUP_FINE_PRE_DWELL_SEC
            )
            self.publish_motion_status(
                status="RUNNING", action=action,
                command_id=self.active_command_id, event_id=self.active_event_id,
                request_id=self.active_request_id, motion_id=self.active_motion_id,
                message="holding still for one second before pickup fine forward",
            )
            return
        if (
            is_active
            and payload["status"] == "SUCCEEDED"
            and payload["motion_id"] == self.active_motion_id
        ):
            self._record_goal_motion_success(payload["motion_id"])
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_initial_align_correction_active
            and stage_can_advance
        ):
            self._start_pickup_checkpoint_dwell(
                self.FINE_ALIGN_MARKER
                if self.pickup_fine_positioning_complete
                else self.PICKUP_INITIAL_ALIGN_MARKER
            )
            return
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_fine_align_correction_active
            and stage_can_advance
        ):
            self._start_pickup_checkpoint_dwell(
                self.FINE_ALIGN_MARKER
            )
            return
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_post_backward_align_correction_active
            and stage_can_advance
        ):
            self._start_pickup_checkpoint_dwell(
                self.POST_BACKWARD_ALIGN_MARKER
            )
            return
        if (
            is_active
            and stage_can_advance
            and self._start_next_pickup_motion()
        ):
            return
        motor_failures = (
            self.pickup_motor_failures
            if is_active and action == "PICKUP_NOW" else None
        )
        if is_queued and payload["status"] in self.TERMINAL_STATUSES:
            self._clear_queued_request()
        elif is_active and payload["status"] in self.TERMINAL_STATUSES:
            self._clear_active_request()
            if payload["status"] == "SUCCEEDED":
                self._promote_queued_request()
            else:
                self._clear_queued_request()

        self.publish_motion_status(
            status=payload["status"],
            command_id=payload["command_id"],
            event_id=payload["event_id"],
            request_id=payload["request_id"],
            motion_id=payload["motion_id"],
            action=action,
            error_code=payload["error_code"],
            message=payload["message"],
            motor_failures=motor_failures,
        )


def main(args: list[str] | None = None) -> None:
    """Run the bridge node."""
    rclpy.init(args=args)
    node = MotionCommandBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
