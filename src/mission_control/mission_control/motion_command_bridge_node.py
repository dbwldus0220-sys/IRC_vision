#!/usr/bin/env python3
"""Bridge navigation JSON commands to STEP Dynamics messages."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from robot_msgs.msg import MotionCommand
from robot_msgs.msg import MotionEnd
from std_msgs.msg import String


class MotionCommandBridgeNode(Node):
    """Convert abstract navigation actions into Dynamics commands."""

    SPECIAL_ACTIONS = {
        "PICKUP_NOW",
        "SHOT",
        "GO",
    }

    def __init__(self) -> None:
        """Initialize bridge state, publishers, and subscriptions."""
        super().__init__('motion_command_bridge')

        self.last_sent_command_id: int | None = None
        self.motion_in_progress = False

        self.active_command_id: int | None = None
        self.active_event_id: int | None = None
        self.active_action: str | None = None
        self.active_dynamics_command: int | None = None

        self.navigation_subscription = self.create_subscription(
            String,
            "/navigation/motion_command",
            self.navigation_command_callback,
            10,
        )

        self.motion_end_subscription = self.create_subscription(
            MotionEnd,
            "/motion_end",
            self.motion_end_callback,
            10,
        )

        self.motion_command_publisher = self.create_publisher(
            MotionCommand,
            "/motion_command",
            10,
        )

        self.motion_status_publisher = self.create_publisher(
            String,
            "/motion/status",
            10,
        )

        self.get_logger().info(
            "Motion command bridge ready: "
            "/navigation/motion_command -> /motion_command, "
            "/motion_end subscribed, "
            "/motion/status publisher ready"
        )

    def map_action_to_dynamics(
        self,
        payload: dict[str, Any],
    ) -> tuple[int, int] | None:
        """Convert one abstract action into a Dynamics command and angle."""
        action = payload.get("action")
        source_command = payload.get("source_command")

        if not isinstance(source_command, dict):
            source_command = {}

        fixed_map: dict[str, tuple[int, int]] = {
            "STRAIGHT": (1, 0),
            "APPROACH": (12, 0),
            "SLOW_APPROACH": (6, 0),
            "FINE_FORWARD_STEP": (27, 0),
            "APPROACH_GOAL": (6, 0),
            "APPROACH_HURDLE": (13, 0),
            "ALIGN_LEFT": (15, 0),
            "ALIGN_RIGHT": (16, 0),
            "RETREAT_GOAL": (5, 0),

            # Special terminal motions
            "PICKUP_NOW": (9, 0),
            "SHOT": (17, 0),
            "GO": (14, 0),
        }

        if action in fixed_map:
            return fixed_map[action]

        if action in {"TURN_LEFT", "TURN_RIGHT", "LEFT", "RIGHT"}:
            raw_angle = source_command.get(
                "target_heading_change_deg",
                0.0,
            )

            try:
                angle = round(abs(float(raw_angle)))
            except (TypeError, ValueError):
                angle = 0

            angle = max(1, min(angle, 55))

            if action in {"TURN_LEFT", "LEFT"}:
                return 2, angle

            return 3, angle

        return None

    def publish_motion_status(
        self,
        *,
        status: str,
        command_id: int | None,
        event_id: int | None,
        action: str | None,
        dynamics_command: int | None,
        reason: str,
    ) -> None:
        """Publish the current bridge and Dynamics execution state."""
        payload = {
            "status": status,
            "command_id": command_id,
            "event_id": event_id,
            "action": action,
            "dynamics_command": dynamics_command,
            "reason": reason,
            "motion_in_progress": self.motion_in_progress,
            "source_node": "motion_command_bridge_node",
        }

        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )

        self.motion_status_publisher.publish(message)

        self.get_logger().info(
            "Motion status published: "
            f"status={status}, "
            f"command_id={command_id}, "
            f"event_id={event_id}, "
            f"action={action}, "
            f"dynamics_command={dynamics_command}"
        )

    def navigation_command_callback(self, msg: String) -> None:
        """Parse one navigation command and publish a Dynamics command."""
        try:
            payload: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError as exc:
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
        valid = payload.get("valid")

        self.get_logger().info(
            "Navigation command received: "
            f"command_id={command_id}, "
            f"event_id={event_id}, "
            f"action={action}, "
            f"valid={valid}"
        )

        if valid is not True:
            self.get_logger().info(
                "Command ignored: valid is not true"
            )
            return

        if not isinstance(command_id, int):
            self.get_logger().warning(
                "Command ignored: command_id is missing "
                "or not an integer"
            )
            return

        if event_id is not None and not isinstance(event_id, int):
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
            self.get_logger().info(
                f"Duplicate command_id ignored: {command_id}"
            )
            return

        if self.motion_in_progress:
            self.get_logger().info(
                "Command ignored for now: "
                "Dynamics motion is in progress"
            )
            return

        if action in self.SPECIAL_ACTIONS:
            sdk_motion_requested = payload.get(
                "sdk_motion_requested",
                False,
            )

            if sdk_motion_requested is not True:
                self.get_logger().info(
                    "Special motion ignored: "
                    f"action={action}, "
                    "sdk_motion_requested is not true"
                )
                return

        mapped = self.map_action_to_dynamics(payload)

        if mapped is None:
            self.get_logger().warning(
                f"Unsupported action: {action}"
            )
            return

        dynamics_command, dynamics_angle = mapped

        dynamics_msg = MotionCommand()
        dynamics_msg.command = dynamics_command
        dynamics_msg.angle = dynamics_angle

        self.motion_command_publisher.publish(dynamics_msg)

        self.last_sent_command_id = command_id
        self.motion_in_progress = True
        self.active_command_id = command_id
        self.active_event_id = event_id
        self.active_action = action
        self.active_dynamics_command = dynamics_command

        self.get_logger().info(
            "Dynamics command published: "
            f"command_id={command_id}, "
            f"event_id={event_id}, "
            f"action={action}, "
            f"command={dynamics_command}, "
            f"angle={dynamics_angle}"
        )

        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            action=self.active_action,
            dynamics_command=self.active_dynamics_command,
            reason="dynamics_command_published",
        )

    def motion_end_callback(self, msg: MotionEnd) -> None:
        """Release the bridge only for the matching completed command."""
        self.get_logger().info(
            "Motion end received: "
            f"finished={msg.finished}, "
            f"command={msg.command}, "
            f"motion_end_detect={msg.motion_end_detect}"
        )

        if not self.motion_in_progress:
            self.get_logger().info(
                "Motion end ignored: bridge has no active command"
            )
            return

        if not msg.motion_end_detect or not msg.finished:
            self.get_logger().warning(
                "Motion end ignored: completion flags are not true"
            )
            return

        if msg.command != self.active_dynamics_command:
            self.get_logger().warning(
                "Motion end ignored: command mismatch "
                f"(active={self.active_dynamics_command}, "
                f"received={msg.command})"
            )

            self.publish_motion_status(
                status="IGNORED",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                action=self.active_action,
                dynamics_command=self.active_dynamics_command,
                reason=(
                    "motion_end_command_mismatch:"
                    f"received={msg.command},"
                    f"expected={self.active_dynamics_command}"
                ),
            )
            return

            

        completed_command_id = self.active_command_id
        completed_event_id = self.active_event_id
        completed_action = self.active_action
        completed_dynamics_command = self.active_dynamics_command

        self.motion_in_progress = False
        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_dynamics_command = None

        self.publish_motion_status(
            status="SUCCEEDED",
            command_id=completed_command_id,
            event_id=completed_event_id,
            action=completed_action,
            dynamics_command=completed_dynamics_command,
            reason="matching_motion_end_received",
        )

        self.get_logger().info(
            "Bridge state changed to IDLE: "
            f"completed_command={completed_dynamics_command}"
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
