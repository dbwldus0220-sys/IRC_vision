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

from .motion_decision_planner import MotionDecision
from .motion_decision_planner import MotionDecisionConfig
from .motion_decision_planner import MotionDecisionPlanner


class MotionDecisionNode(Node):
    """Replace four navigation controllers with one command publisher."""

    SOURCES = ("line", "ball", "goal", "hurdle", "finish")

    SPECIAL_ACTIONS = {
        "PICKUP_NOW",
        "SHOT",
        "GO",
        "CROSS_FINISH",
    }

    SPECIAL_COMPLETION_PHASES = {
        "PICKUP_NOW": "GOAL_APPROACH",
        "SHOT": "AUTO",
        "GO": "AUTO",
        "CROSS_FINISH": "FINISHED",
    }

    SPECIAL_ACTION_SOURCES = {
        "PICKUP_NOW": "ball",
        "SHOT": "goal",
        "GO": "hurdle",
        "CROSS_FINISH": "finish",
    }

    def __init__(self) -> None:
        """Initialize mission decision state, topics, and timers."""
        super().__init__("motion_decision_node")

        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("ball_info_topic", "/vision/ball_info")
        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("hurdle_info_topic", "/vision/hurdle_info")
        self.declare_parameter("finish_info_topic", "/vision/finish_info")

        self.declare_parameter("mission_phase_topic", "/mission/phase")
        self.declare_parameter(
            "command_topic",
            "/navigation/motion_command",
        )
        self.declare_parameter(
            "motion_status_topic",
            "/motion/status",
        )

        self.declare_parameter("initial_mission_phase", "AUTO")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("required_pickups", 2)
        self.declare_parameter("required_shots", 2)
        self.declare_parameter("required_ball_sections", 2)
        self.declare_parameter("finish_min_confidence", 0.70)

        self.declare_parameter("line_timeout_sec", 0.50)
        self.declare_parameter("ball_timeout_sec", 0.50)
        self.declare_parameter("goal_timeout_sec", 0.50)
        self.declare_parameter("hurdle_timeout_sec", 0.50)
        self.declare_parameter("finish_timeout_sec", 0.50)

        self.declare_parameter("enable_ball_lost_recovery", True)
        self.declare_parameter("ball_tracking_range_m", 3.0)
        self.declare_parameter("ball_control_range_m", 0.9)
        self.declare_parameter("ball_lost_stop_sec", 0.5)
        self.declare_parameter("ball_recovery_timeout_sec", 2.5)
        self.declare_parameter("ball_recovery_max_distance_m", 1.0)
        self.declare_parameter("ball_reacquire_confirm_frames", 3)
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
                ball_recovery_max_distance_m=self._float_parameter(
                    "ball_recovery_max_distance_m"
                ),
                ball_reacquire_confirm_frames=max(
                    1,
                    int(
                        self.get_parameter(
                            "ball_reacquire_confirm_frames"
                        ).value
                    ),
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
        self.required_pickups = max(
            0,
            int(self.get_parameter("required_pickups").value),
        )
        self.required_shots = max(
            0,
            int(self.get_parameter("required_shots").value),
        )
        self.required_ball_sections = max(
            0,
            int(
                self.get_parameter(
                    "required_ball_sections"
                ).value
            ),
        )
        self.pickups_completed = 0
        self.shots_completed = 0
        self.ball_sections_processed = 0
        self.finish_enabled = (
            self.ball_sections_processed
            >= self.required_ball_sections
        )
        self.mission_complete = False
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
        self.terminal_latch: tuple[str, str] | None = None
        self.terminal_action_armed = {
            source: True
            for source in self.SPECIAL_ACTION_SOURCES.values()
        }

        # Special SDK/Dynamics motion lock state.
        self.special_motion_running = False
        self.active_special_action: str | None = None
        self.active_special_command_id: int | None = None
        self.active_special_event_id: int | None = None
        self.active_special_dynamics_command: int | None = None

        for source in self.SOURCES:
            topic = str(
                self.get_parameter(
                    f"{source}_info_topic"
                ).value
            )

            self.create_subscription(
                String,
                topic,
                self._info_callback(source),
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

        self.get_logger().info(
            f"Mission phase: {self.mission_phase}"
        )
        self.get_logger().info(
            f"Unified command: {command_topic}"
        )
        self.get_logger().info(
            f"Motion status: {motion_status_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _info_callback(self, source: str):
        def callback(message: String) -> None:
            try:
                payload = json.loads(message.data)

                if not isinstance(payload, dict):
                    raise ValueError(
                        "JSON must be an object"
                    )

                self.latest_info[source] = payload
                self.latest_time[source] = time.monotonic()

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

        if phase:
            self.mission_phase = phase.upper()

            self.get_logger().info(
                f"Mission phase changed: "
                f"{self.mission_phase}"
            )

    def _motion_status_callback(
        self,
        message: String,
    ) -> None:
        """Track execution state reported by the command bridge."""
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
        dynamics_command = payload.get(
            "dynamics_command"
        )

        if action not in self.SPECIAL_ACTIONS:
            return

        if status == "RUNNING":
            self.special_motion_running = True
            self.active_special_action = action
            self.active_special_command_id = (
                command_id
                if isinstance(command_id, int)
                else None
            )
            self.active_special_event_id = (
                event_id
                if isinstance(event_id, int)
                else None
            )
            self.active_special_dynamics_command = (
                dynamics_command
                if isinstance(dynamics_command, int)
                else None
            )

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

        if status not in {
            "SUCCEEDED",
            "FAILED",
            "TIMEOUT",
        }:
            return

        if not self.special_motion_running:
            self.get_logger().info(
                "Terminal motion status ignored: "
                "no special motion is currently locked"
            )
            return

        if (
            self.active_special_event_id is not None
            and event_id != self.active_special_event_id
        ):
            self.get_logger().warning(
                "Terminal motion status ignored: "
                "event_id mismatch "
                f"(active={self.active_special_event_id}, "
                f"received={event_id})"
            )
            return

        if (
            self.active_special_action is not None
            and action != self.active_special_action
        ):
            self.get_logger().warning(
                "Terminal motion status ignored: "
                "action mismatch "
                f"(active={self.active_special_action}, "
                f"received={action})"
            )
            return

        completed_action = self.active_special_action
        completed_event_id = self.active_special_event_id
        completed_command_id = (
            self.active_special_command_id
        )

        self.special_motion_running = False
        self.active_special_action = None
        self.active_special_command_id = None
        self.active_special_event_id = None
        self.active_special_dynamics_command = None

        self.get_logger().info(
            "Special motion lock released: "
            f"status={status}, "
            f"action={completed_action}, "
            f"command_id={completed_command_id}, "
            f"event_id={completed_event_id}"
        )

        self._update_action_progress(completed_action, status)

        if status == "SUCCEEDED":
            next_phase = self.SPECIAL_COMPLETION_PHASES.get(
                completed_action,
                "AUTO",
            )
            if completed_action == "SHOT" and self.finish_enabled:
                next_phase = "WALK_TO_FINISH"
            elif completed_action == "CROSS_FINISH":
                self.mission_complete = True
                next_phase = "FINISHED"
        else:
            if completed_action == "CROSS_FINISH":
                self.mission_complete = False
                self.terminal_action_armed["finish"] = True
                next_phase = "WALK_TO_FINISH"
            elif (
                completed_action in {"PICKUP_NOW", "SHOT"}
                and self.finish_enabled
            ):
                next_phase = "WALK_TO_FINISH"
            else:
                next_phase = "AUTO"

        previous_phase = self.mission_phase
        self.mission_phase = next_phase
        self.terminal_latch = None

        self.get_logger().info(
            "Mission phase advanced after special motion: "
            f"status={status}, "
            f"action={completed_action}, "
            f"previous_phase={previous_phase}, "
            f"next_phase={self.mission_phase}"
        )

    def _update_action_progress(
        self,
        completed_action: str | None,
        status: str,
    ) -> None:
        """Record success scores and processed ball-course sections."""
        previous_pickups = self.pickups_completed
        previous_shots = self.shots_completed
        previous_sections = self.ball_sections_processed
        previous_finish_enabled = self.finish_enabled

        if completed_action == "PICKUP_NOW" and status == "SUCCEEDED":
            self.pickups_completed = min(
                self.pickups_completed + 1,
                self.required_pickups,
            )
        elif completed_action == "PICKUP_NOW":
            self.ball_sections_processed = min(
                self.ball_sections_processed + 1,
                self.required_ball_sections,
            )
        elif completed_action == "SHOT":
            if status == "SUCCEEDED":
                self.shots_completed = min(
                    self.shots_completed + 1,
                    self.required_shots,
                )
            self.ball_sections_processed = min(
                self.ball_sections_processed + 1,
                self.required_ball_sections,
            )

        if completed_action in {"PICKUP_NOW", "SHOT"}:
            self.finish_enabled = (
                self.ball_sections_processed
                >= self.required_ball_sections
            )

        counters_changed = (
            self.pickups_completed != previous_pickups
            or self.shots_completed != previous_shots
            or self.ball_sections_processed != previous_sections
            or self.finish_enabled != previous_finish_enabled
        )
        if counters_changed:
            self.get_logger().info(
                "Mission progress updated: "
                f"completed_action={completed_action}, "
                f"status={status}, "
                f"pickups={self.pickups_completed}/"
                f"{self.required_pickups}, "
                f"shots={self.shots_completed}/"
                f"{self.required_shots}, "
                f"ball_sections="
                f"{self.ball_sections_processed}/"
                f"{self.required_ball_sections}, "
                f"finish_enabled={self.finish_enabled}"
            )

    def _mission_progress(self) -> dict[str, int | bool]:
        """Return success scores and independent course progress."""
        return {
            "pickups_completed": self.pickups_completed,
            "required_pickups": self.required_pickups,
            "shots_completed": self.shots_completed,
            "required_shots": self.required_shots,
            "ball_sections_processed": self.ball_sections_processed,
            "required_ball_sections": self.required_ball_sections,
            "finish_enabled": self.finish_enabled,
            "mission_complete": self.mission_complete,
        }

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

    def _publish_decision(self) -> None:
        now = time.monotonic()

        dt_sec = max(
            1e-3,
            now - self.previous_publish_time,
        )

        self.previous_publish_time = now

        observations, ages = self._fresh_observations(
            now
        )

        self._rearm_absent_terminal_targets(observations)

        decision = self._select_mission_decision(
            observations,
            dt_sec,
        )

        decision = self._suppress_duplicate_terminal_action(
            decision
        )

        terminal_key = (
            decision.source,
            decision.action,
        )

        trigger = False

        if decision.requires_ack:
            if self.terminal_latch != terminal_key:
                self.event_id += 1
                trigger = True

                source = self.SPECIAL_ACTION_SOURCES.get(
                    decision.action
                )
                if source is not None:
                    self.terminal_action_armed[source] = False

            self.terminal_latch = terminal_key

        elif not self.special_motion_running:
            self.terminal_latch = None

        self.command_id += 1

        payload = decision.to_dict()

        payload.update(
            {
                "command_id": self.command_id,
                "event_id": (
                    self.event_id
                    if decision.requires_ack
                    else None
                ),
                "sdk_motion_requested": trigger,
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

    def _select_mission_decision(
        self,
        observations: dict[str, dict[str, Any] | None],
        dt_sec: float,
    ) -> MotionDecision:
        """Apply finish priorities before falling back to normal planning."""
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

        planning_phase = self.mission_phase
        if self.special_motion_running:
            locked_phase = (
                planning_phase
                if planning_phase.endswith("_LOCK")
                else f"{planning_phase}_LOCK"
            )
            return self.planner.plan(
                locked_phase,
                observations,
                dt_sec,
            )

        finish_info = observations.get("finish")
        if self._finish_crossing_ready(finish_info):
            return MotionDecision(
                phase="WALK_TO_FINISH",
                source="finish",
                action="CROSS_FINISH",
                valid=True,
                reason="confirmed_finish_ready",
                sdk_motion_requested=True,
                requires_ack=True,
                source_command=dict(finish_info or {}),
            )

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

        return self.planner.plan(
            planning_phase,
            observations,
            dt_sec,
        )

    def _finish_crossing_ready(
        self,
        finish_info: dict[str, Any] | None,
    ) -> bool:
        """Return whether a fresh finish observation can trigger crossing."""
        if (
            self.mission_phase != "WALK_TO_FINISH"
            or not self.finish_enabled
            or finish_info is None
            or not bool(finish_info.get("detected", False))
            or not bool(finish_info.get("confirmed", False))
        ):
            return False

        confidence = finish_info.get("confidence")
        return (
            isinstance(confidence, (int, float))
            and not isinstance(confidence, bool)
            and float(confidence) >= self.finish_min_confidence
        )

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
