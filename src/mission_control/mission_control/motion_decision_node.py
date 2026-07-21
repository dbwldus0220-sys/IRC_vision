#!/usr/bin/env python3
"""ROS 2 node selecting one command from line, ball, goal, and hurdle."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .motion_decision_planner import MotionDecisionConfig
from .motion_decision_planner import MotionDecisionPlanner


class MotionDecisionNode(Node):
    """Replace four navigation controllers with one command publisher."""

    SOURCES = ("line", "ball", "goal", "hurdle")

    def __init__(self) -> None:
        super().__init__("motion_decision_node")
        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("ball_info_topic", "/vision/ball_info")
        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("hurdle_info_topic", "/vision/hurdle_info")
        self.declare_parameter("mission_phase_topic", "/mission/phase")
        self.declare_parameter(
            "command_topic",
            "/navigation/motion_command",
        )
        self.declare_parameter("initial_mission_phase", "AUTO")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("line_timeout_sec", 0.50)
        self.declare_parameter("ball_timeout_sec", 0.50)
        self.declare_parameter("goal_timeout_sec", 0.50)
        self.declare_parameter("hurdle_timeout_sec", 0.50)
        self.declare_parameter("enable_ball_lost_recovery", False)
        self.declare_parameter("ball_tracking_range_m", 3.0)
        self.declare_parameter("ball_control_range_m", 0.9)
        self.declare_parameter("ball_lost_stop_sec", 0.35)
        self.declare_parameter("ball_recovery_timeout_sec", 8.0)
        self.declare_parameter("ball_recovery_turn_rad_s", 0.22)
        self.declare_parameter("ball_recovery_command_sec", 0.40)
        self.declare_parameter("ball_reacquire_center_deg", 5.0)
        self.declare_parameter("ball_reacquire_center_norm", 0.08)
        self.declare_parameter("goal_tracking_range_m", 3.0)
        self.declare_parameter("goal_control_range_m", 0.5)
        self.declare_parameter("goal_lost_stop_sec", 0.35)
        self.declare_parameter("goal_recovery_timeout_sec", 8.0)
        self.declare_parameter("goal_recovery_turn_rad_s", 0.22)
        self.declare_parameter("goal_recovery_command_sec", 0.40)
        self.declare_parameter("goal_reacquire_center_deg", 5.0)
        self.declare_parameter("goal_reacquire_center_norm", 0.10)

        self.planner = MotionDecisionPlanner(
            MotionDecisionConfig(
                enable_ball_lost_recovery=bool(
                    self.get_parameter(
                        "enable_ball_lost_recovery"
                    ).value
                ),
                ball_tracking_range_m=self._float_parameter(
                    "ball_tracking_range_m"
                ),
                ball_control_range_m=self._float_parameter(
                    "ball_control_range_m"
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
        self.mission_phase = str(
            self.get_parameter("initial_mission_phase").value
        ).strip().upper()
        self.latest_info: dict[str, dict[str, Any] | None] = {
            source: None for source in self.SOURCES
        }
        self.latest_time: dict[str, float | None] = {
            source: None for source in self.SOURCES
        }
        self.timeouts = {
            source: max(
                0.05,
                float(self.get_parameter(f"{source}_timeout_sec").value),
            )
            for source in self.SOURCES
        }
        self.previous_publish_time = time.monotonic()
        self.command_id = 0
        self.event_id = 0
        self.terminal_latch: tuple[str, str] | None = None

        for source in self.SOURCES:
            topic = str(self.get_parameter(f"{source}_info_topic").value)
            self.create_subscription(
                String,
                topic,
                self._info_callback(source),
                10,
            )
            self.get_logger().info(f"{source} info: {topic}")

        phase_topic = str(
            self.get_parameter("mission_phase_topic").value
        )
        self.create_subscription(
            String,
            phase_topic,
            self._phase_callback,
            10,
        )
        command_topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(String, command_topic, 10)
        publish_rate = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_decision,
        )
        self.get_logger().info(f"Mission phase: {self.mission_phase}")
        self.get_logger().info(f"Unified command: {command_topic}")

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _info_callback(self, source: str):
        def callback(message: String) -> None:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("JSON must be an object")
                self.latest_info[source] = payload
                self.latest_time[source] = time.monotonic()
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f"Invalid {source}_info: {type(exc).__name__}: {exc}"
                )

        return callback

    def _phase_callback(self, message: String) -> None:
        phase = message.data.strip()
        if not phase:
            return
        try:
            payload = json.loads(phase)
            if isinstance(payload, dict):
                phase = str(payload.get("phase", "")).strip()
        except json.JSONDecodeError:
            pass
        if phase:
            self.mission_phase = phase.upper()

    def _fresh_observations(
        self,
        now: float,
    ) -> tuple[dict[str, dict[str, Any] | None], dict[str, float | None]]:
        observations: dict[str, dict[str, Any] | None] = {}
        ages: dict[str, float | None] = {}
        for source in self.SOURCES:
            stamp = self.latest_time[source]
            age = now - stamp if stamp is not None else None
            ages[source] = round(age, 3) if age is not None else None
            observations[source] = (
                self.latest_info[source]
                if age is not None and age <= self.timeouts[source]
                else None
            )
        return observations, ages

    def _publish_decision(self) -> None:
        now = time.monotonic()
        dt_sec = max(1e-3, now - self.previous_publish_time)
        self.previous_publish_time = now
        observations, ages = self._fresh_observations(now)
        decision = self.planner.plan(
            self.mission_phase,
            observations,
            dt_sec,
        )

        terminal_key = (decision.source, decision.action)
        trigger = False
        if decision.requires_ack:
            if self.terminal_latch != terminal_key:
                self.event_id += 1
                trigger = True
            self.terminal_latch = terminal_key
        else:
            self.terminal_latch = None

        self.command_id += 1
        payload = decision.to_dict()
        payload.update(
            {
                "command_id": self.command_id,
                "event_id": self.event_id if decision.requires_ack else None,
                "sdk_motion_requested": trigger,
                "request_latched": decision.requires_ack,
                "sdk_motion_id": None,
                "input_age_sec": ages,
                "ball_tracking": self.planner.ball_tracking_status(),
                "goal_tracking": self.planner.goal_tracking_status(),
                "source_node": "motion_decision_node",
            }
        )
        output = String()
        output.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    """Run the unified motion decision node."""
    rclpy.init(args=args)
    node: MotionDecisionNode | None = None
    try:
        node = MotionDecisionNode()
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
