#!/usr/bin/env python3
"""Bridge navigation JSON commands to the C++ motion executor."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


LEFT_RECOVERY_MOTION_IDS = {
    2: "line_turn_left_4",
    4: "line_recovery_left_4",
    5: "line_recovery_left_5",
    6: "line_recovery_left_6",
    7: "line_recovery_left_7",
    8: "line_recovery_left_8",
    10: "line_turn_left_12",
    13: "line_turn_left_15",
}
RIGHT_RECOVERY_MOTION_IDS = {
    4: "line_recovery_right_4",
    5: "line_recovery_right_5",
    6: "line_recovery_right_6",
    7: "line_recovery_right_7",
    8: "line_recovery_right_8",
    10: "line_turn_right_8",
    12: "line_turn_right_10",
    15: "line_turn_right_large",
}


class MotionCommandBridgeNode(Node):
    """Translate supported navigation actions into SDK executor requests."""

    DWELL_MARKER = "__NON_BLOCKING_DWELL__"
    FINE_ALIGN_MARKER = "__BALL_PICKUP_FINE_ALIGN_CHECK__"
    PICKUP_DWELL_SEC = 3.0
    PICKUP_FINE_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_FINE_ALIGN_CONTINUE",
            "BALL_PICKUP_CRAB_RIGHT",
            "BALL_PICKUP_CRAB_LEFT",
        }
    )
    PICKUP_FINE_ALIGN_MOTION_IDS = {
        "RIGHT": "pickup_crab_right_0",
        "LEFT": "pickup_crab_left_0",
    }
    ATOMIC_SEQUENCE_ACTIONS = frozenset(
        {"PICKUP_NOW", "POST_BALL_GOAL_TRANSITION"}
    )

    PICKUP_CAMERA_DOWN_MOTION_IDS = {
        "STRAIGHT_1": "ball_camera_down_forward_2",
        "STRAIGHT_2": "ball_camera_down_forward_4",
        "STRAIGHT_3": "ball_camera_down_forward_6",
        "STRAIGHT_4": "ball_camera_down_forward_8",
    }
    PICKUP_MOTION_TAIL = (
        "pickup_fine_forward_0",
        FINE_ALIGN_MARKER,
        "pickup_pre_backward_camera_down",
        "pickup_left_back_to_default_90",
        "pickup",
        "pickup_retreat_3",
    )
    POST_BALL_GOAL_TRANSITION_SEQUENCE = (
        "post_ball_forward_4",
        "post_ball_forward_8",
        "post_ball_camera_90",
    )
    ACTION_TO_MOTION_ID = {
        "STRAIGHT": "line_forward_6",
        "STRAIGHT_1": "line_forward_2",
        "STRAIGHT_2": "line_forward_4",
        "STRAIGHT_3": "line_forward_6",
        "STRAIGHT_4": "line_forward_8",
        "STRAIGHT_5": "line_forward_10",
        "APPROACH": "forward",
        "LEFT": "line_turn_left_15",
        "RIGHT": "line_turn_right_large",
        "PICKUP_NOW": "ball_camera_down_forward_4",
        "POST_BALL_GOAL_TRANSITION": "post_ball_forward_4",
        "BALL_FINE_FORWARD_8": "ball_general_fine_forward_8",
        "GOAL_CAMERA_90_FORWARD": "goal_camera_90_forward_6",
        "GOAL_CAMERA_90_FORWARD_1": "goal_camera_90_forward_2",
        "GOAL_CAMERA_90_FORWARD_2": "goal_camera_90_forward_4",
        "GOAL_CAMERA_90_FORWARD_4": "goal_camera_90_forward_8",
        "GOAL_CAMERA90_CRAB_RIGHT": "goal_camera_90_crab_right",
        "GOAL_CAMERA90_CRAB_LEFT": "goal_camera_90_crab_left",
        **{
            f"POST_BALL_LINE_TURN_RIGHT_{count}": (
                f"post_ball_line_turn_right_{count}"
            )
            for count in range(1, 10)
        },
        **{
            f"POST_BALL_LINE_TURN_LEFT_{count}": (
                f"post_ball_line_turn_left_{count}"
            )
            for count in (2, 3, 4, 6)
        },
        **{
            f"GOAL_CAMERA90_TURN_RIGHT_{count}": (
                f"goal_camera_90_turn_right_{count}"
            )
            for count in range(1, 10)
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
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
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

        approach_motion = source_command.get("pickup_approach_motion")
        first_motion = cls.PICKUP_CAMERA_DOWN_MOTION_IDS.get(approach_motion)
        completed = mission_progress.get("pickups_completed")
        if (
            first_motion is None
            or isinstance(completed, bool)
            or not isinstance(completed, int)
            or completed not in {0, 1}
        ):
            return None

        if completed == 0:
            final_stages = (
                "pickup_first_turn_right_9",
                cls.DWELL_MARKER,
                "pickup_first_to_right_back_camera_45",
            )
        else:
            final_stages = (
                "stationary_turn_left",
                cls.DWELL_MARKER,
            )
        return (first_motion, *cls.PICKUP_MOTION_TAIL, *final_stages)

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
            if not self._start_next_pickup_motion():
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

        direction = (
            "RIGHT" if action == "BALL_PICKUP_CRAB_RIGHT" else "LEFT"
        )
        motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[direction]
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

    def _publish_executor_request(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        request_id: int,
        motion_id: str,
        timeout_ms: int,
    ) -> None:
        """Publish one validated request to the C++ executor."""
        request_payload = {
            "action": action,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "timeout_ms": timeout_ms,
        }
        request_message = String()
        request_message.data = json.dumps(
            request_payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.executor_request_publisher.publish(request_message)

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
            self.get_logger().info("Command ignored: valid is not true")
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

        if action in self.PICKUP_FINE_ALIGN_ACTIONS:
            self._handle_pickup_fine_align_command(
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
            and self.active_action in self.ATOMIC_SEQUENCE_ACTIONS
        ):
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="ATOMIC_SEQUENCE_LOCKED",
                message="an atomic BALL sequence owns the motion executor",
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
                        "pickup sequence requires a supported camera-down "
                        "approach bucket and ball mission index 0 or 1"
                    ),
                )
                return
            pickup_sequence = built_sequence
            motion_id = pickup_sequence[0]
        elif action == "POST_BALL_GOAL_TRANSITION":
            pickup_sequence = self.POST_BALL_GOAL_TRANSITION_SEQUENCE
            motion_id = pickup_sequence[0]
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
        defer_until_active_finishes = (
            self.motion_in_progress and action == "PICKUP_NOW"
        )
        if not defer_until_active_finishes:
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

    def _start_next_pickup_motion(self) -> bool:
        """Advance one atomic BALL sequence after a successful stage."""
        if self.active_action not in self.ATOMIC_SEQUENCE_ACTIONS:
            return False
        if self.active_dwell_until is not None:
            return True
        next_index = self.active_sequence_index + 1
        if next_index >= len(self.active_pickup_sequence):
            return False

        next_motion_id = self.active_pickup_sequence[next_index]
        self.active_sequence_index = next_index
        if next_motion_id == self.FINE_ALIGN_MARKER:
            self._enter_pickup_fine_align_checkpoint()
            return True
        if next_motion_id == self.DWELL_MARKER:
            self.active_dwell_until = time.monotonic() + self.PICKUP_DWELL_SEC
            self.get_logger().info(
                f"Atomic sequence dwell started: {self.PICKUP_DWELL_SEC:.1f}s"
            )
            return True
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
            f"Pickup sequence advanced to {next_motion_id}"
        )
        return True

    def _check_atomic_dwell(self, now: float | None = None) -> None:
        """Advance a dwell stage without blocking ROS callbacks."""
        if self.active_dwell_until is None:
            return
        current_time = time.monotonic() if now is None else now
        if current_time < self.active_dwell_until:
            return

        self.active_dwell_until = None
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        next_index = self.active_sequence_index + 1
        if next_index < len(self.active_pickup_sequence):
            self.active_sequence_index = next_index
            next_motion_id = self.active_pickup_sequence[next_index]
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
            return

        command_id = self.active_command_id
        event_id = self.active_event_id
        request_id = self.active_request_id
        motion_id = self.active_motion_id
        action = self.active_action
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
            status in {"RUNNING", *self.TERMINAL_STATUSES}
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
        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_request_id = None
        self.active_motion_id = None
        self.active_timeout_ms = None
        self.active_pickup_sequence = ()
        self.active_sequence_index = 0
        self.active_dwell_until = None
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False

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
        queued_request_deferred = self.queued_request_deferred
        self._clear_queued_request()
        self.motion_in_progress = self.active_request_id is not None
        if queued_request_deferred and self.motion_in_progress:
            self._publish_executor_request(
                action=self.active_action,
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=self.active_motion_id,
                timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
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
        if (
            is_active
            and action in self.ATOMIC_SEQUENCE_ACTIONS
            and payload["motion_id"] != self.active_motion_id
        ):
            self.get_logger().warning(
                "Executor status ignored: pickup motion_id mismatch"
            )
            return
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_fine_align_correction_active
            and payload["status"] == "SUCCEEDED"
        ):
            self._enter_pickup_fine_align_checkpoint()
            return
        if (
            is_active
            and payload["status"] == "SUCCEEDED"
            and self._start_next_pickup_motion()
        ):
            return
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
