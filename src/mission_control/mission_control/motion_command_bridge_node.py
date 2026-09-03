#!/usr/bin/env python3
"""Bridge navigation JSON commands to the C++ motion executor."""

from __future__ import annotations

import json
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

    ACTION_TO_MOTION_ID = {
        "STRAIGHT": "forward",
        "STRAIGHT_1": "line_forward_2",
        "STRAIGHT_2": "line_forward_4",
        "STRAIGHT_3": "line_forward_6",
        "STRAIGHT_4": "line_forward_8",
        "STRAIGHT_5": "line_forward_10",
        "APPROACH": "forward",
        "LEFT": "line_turn_left_15",
        "RIGHT": "line_turn_right_large",
        "PICKUP_NOW": "pickup",
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
    TERMINAL_STATUSES = {"SUCCEEDED", "FAILED", "CANCELLED", "REJECTED"}
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
        self.queued_command_id: int | None = None
        self.queued_event_id: int | None = None
        self.queued_action: str | None = None
        self.queued_request_id: int | None = None
        self.queued_motion_id: str | None = None

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
        """Publish normalized bridge status while preserving executor fields."""
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
                "Invalid /navigation/motion_command: JSON root must be an object"
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

        motion_id = self.motion_id_for_action(action)
        if motion_id is None:
            self._publish_local_rejection(
                status="UNSUPPORTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="UNSUPPORTED_ACTION",
                message="action has no configured motion alias",
            )
            return

        request_id = command_id
        request_payload = {
            "action": action,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "timeout_ms": self.timeout_ms_from_payload(payload),
        }
        request_message = String()
        request_message.data = json.dumps(
            request_payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.executor_request_publisher.publish(request_message)

        self.last_sent_command_id = command_id
        if self.motion_in_progress:
            self.queued_command_id = command_id
            self.queued_event_id = event_id
            self.queued_action = action
            self.queued_request_id = request_id
            self.queued_motion_id = motion_id
        else:
            self.motion_in_progress = True
            self.active_command_id = command_id
            self.active_event_id = event_id
            self.active_action = action
            self.active_request_id = request_id
            self.active_motion_id = motion_id

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

    def _promote_queued_request(self) -> None:
        self.active_command_id = self.queued_command_id
        self.active_event_id = self.queued_event_id
        self.active_action = self.queued_action
        self.active_request_id = self.queued_request_id
        self.active_motion_id = self.queued_motion_id
        self.queued_command_id = None
        self.queued_event_id = None
        self.queued_action = None
        self.queued_request_id = None
        self.queued_motion_id = None
        self.motion_in_progress = self.active_request_id is not None

    def executor_status_callback(self, msg: String) -> None:
        """Forward matching executor status and release terminal requests."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(
                f"Invalid /motion/executor/status JSON: {exc}"
            )
            return
        if not isinstance(payload, dict) or not self._valid_executor_status(payload):
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
        if is_queued and payload["status"] in self.TERMINAL_STATUSES:
            self.queued_command_id = None
            self.queued_event_id = None
            self.queued_action = None
            self.queued_request_id = None
            self.queued_motion_id = None
        elif is_active and payload["status"] in self.TERMINAL_STATUSES:
            self._clear_active_request()
            if payload["status"] == "SUCCEEDED":
                self._promote_queued_request()
            else:
                self.queued_command_id = None
                self.queued_event_id = None
                self.queued_action = None
                self.queued_request_id = None
                self.queued_motion_id = None

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
