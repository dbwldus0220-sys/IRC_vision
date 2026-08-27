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

from .executor_heartbeat_watchdog import ExecutorHeartbeatWatchdog
from .executor_heartbeat_watchdog import HeartbeatStartupDelay
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

    PRE_MOTION_SETTLE_ACTIONS = frozenset(
        {
            "LEFT",
            "RIGHT",
            "PICKUP_NOW",
            "GO",
        }
    )

    SPECIAL_ACTIONS = {
        "PICKUP_NOW",
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
        "STRAIGHT": (4.111, 20),
        "STRAIGHT_1": (0.822, 5),
        "STRAIGHT_2": (1.644, 10),
        "STRAIGHT_3": (2.467, 10),
        "STRAIGHT_4": (3.289, 15),
        "STRAIGHT_5": (4.111, 20),
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
    LINE_MOTION_CAPTURE_START_RATIO = 0.80
    LINE_TIMEOUT_RECOVERY_FRAMES = 10

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

        self.declare_parameter("initial_mission_phase", "AUTO")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pre_motion_settle_sec", 0.5)
        self.declare_parameter("required_pickups", 2)
        self.declare_parameter("required_shots", 2)
        self.declare_parameter("required_ball_sections", 2)
        self.declare_parameter("finish_min_confidence", 0.70)
        self.declare_parameter("general_motion_transient_retry_limit", 2)

        self.declare_parameter("line_timeout_sec", 0.50)
        self.declare_parameter("ball_timeout_sec", 0.50)
        self.declare_parameter("goal_timeout_sec", 0.50)
        self.declare_parameter("hurdle_timeout_sec", 0.50)
        self.declare_parameter("finish_timeout_sec", 0.50)

        self.declare_parameter("enable_ball_lost_recovery", False)
        self.declare_parameter("recovery_heading_turn_deg", 10.0)
        self.declare_parameter("recovery_away_heading_turn_deg", 3.0)
        self.declare_parameter("curve_follow_max_offset_norm", 0.55)
        self.declare_parameter("ball_tracking_range_m", 1.5)
        self.declare_parameter("ball_control_range_m", 0.9)
        self.declare_parameter("ball_lost_stop_sec", 0.35)
        self.declare_parameter("ball_recovery_timeout_sec", 8.0)
        self.declare_parameter("ball_recovery_turn_rad_s", 0.22)
        self.declare_parameter("ball_recovery_command_sec", 0.40)
        self.declare_parameter("ball_reacquire_center_deg", 5.0)
        self.declare_parameter("ball_reacquire_center_norm", 0.08)

        self.declare_parameter("goal_tracking_range_m", 1.0)
        self.declare_parameter("goal_control_range_m", 0.5)
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
        self.pre_motion_settle_source: str | None = None
        self.pre_motion_settle_action: str | None = None
        self.pre_motion_settle_started_at: float | None = None
        self.last_published_vision_stamp: dict[str, float] = {}
        self.active_general_source: str | None = None
        self.active_line_motion_action: str | None = None
        self.active_line_motion_started_at: float | None = None
        self.active_line_motion_duration_sec = 0.0
        self.active_line_motion_target_frames = 0
        self.active_line_motion_frames: list[dict[str, Any]] = []
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
            f"Executor heartbeat: {executor_heartbeat_topic}"
        )
        self.get_logger().info(
            f"Decision debug: {decision_debug_topic}"
        )

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
        auto_ready = payload.get("auto_ready")
        if isinstance(auto_ready, bool):
            if not auto_ready:
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

    def _info_callback(self, source: str):
        def callback(message: String) -> None:
            try:
                payload = json.loads(message.data)

                if not isinstance(payload, dict):
                    raise ValueError(
                        "JSON must be an object"
                    )

                if not self.executor_auto_ready:
                    return

                self.latest_info[source] = payload
                received_at = time.monotonic()
                self.latest_time[source] = received_at
                if source == "line":
                    if self.line_timeout_recovery_active:
                        MotionDecisionNode._collect_timeout_recovery_frame(
                            self,
                            payload,
                        )
                    else:
                        MotionDecisionNode._collect_active_line_motion_frame(
                            self,
                            payload,
                            received_at,
                        )
                self.executor_ready_requires_fresh_vision = False
                self.general_motion_gate.on_new_vision_input()

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

        self.get_logger().info(
            f"Mission phase changed: {self.mission_phase}"
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
        error_code = payload.get("error_code")
        status_message = payload.get("message")
        dynamics_command = payload.get(
            "dynamics_command"
        )

        if normalize_general_action(action) is not None:
            transition = self.general_motion_gate.on_motion_status(
                action,
                status,
                command_id,
                error_code,
            )
            if transition.matched:
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
                completed_source = self.active_general_source
                self.active_general_source = None
                if completed_source == "line":
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
                        self.general_motion_gate.required_vision_generation = (
                            self.general_motion_gate.vision_generation
                        )
                    elif timed_out:
                        MotionDecisionNode._discard_line_motion_capture(self)
                        self.line_timeout_recovery_active = True
                        self.line_timeout_recovery_frames = []
                    else:
                        MotionDecisionNode._discard_line_motion_capture(self)
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

        if not result.terminal:
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

        self.active_special_event_id = None
        self.active_special_dynamics_command = None

        if completed_action == "PICKUP_NOW" and status == "SUCCEEDED":
            # The tracked ball has been collected. Do not resurrect its stale
            # recovery state after GOAL_APPROACH and the following SHOT.
            self.planner.clear_collected_ball_tracking()

        self.get_logger().info(
            "Special motion lock released: "
            f"status={status}, "
            f"action={completed_action}, "
            f"command_id={completed_command_id}, "
            f"event_id={completed_event_id}, "
            f"error_code={error_code}, "
            f"message={status_message}"
        )

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

    def _mission_progress(self) -> dict[str, int | bool]:
        """Return success scores and independent course progress."""
        snapshot = self.phase_manager.snapshot()
        return {
            key: snapshot[key]
            for key in (
                "pickups_completed",
                "required_pickups",
                "shots_completed",
                "required_shots",
                "ball_sections_processed",
                "required_ball_sections",
                "finish_enabled",
                "mission_complete",
            )
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

    def _start_line_motion_capture(self, action: str) -> None:
        """Start the configured late-motion Vision capture window."""
        config = MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG.get(action)
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
    ) -> None:
        """Keep distinct line frames received after 80 percent execution."""
        started_at = self.active_line_motion_started_at
        if started_at is None:
            return
        capture_start = (
            started_at
            + self.active_line_motion_duration_sec
            * MotionDecisionNode.LINE_MOTION_CAPTURE_START_RATIO
        )
        if received_at < capture_start:
            return
        if (
            len(self.active_line_motion_frames)
            >= self.active_line_motion_target_frames
        ):
            return
        self.active_line_motion_frames.append(dict(payload))

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

    def _publish_decision(self) -> None:
        if self.safety_interlock.latched:
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

        if self.general_motion_gate.locked:
            self._reset_pre_motion_settle()
            return

        if self.line_timeout_recovery_active:
            self._reset_pre_motion_settle()
            return

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
        self.last_candidate_decision = decision

        decision = self._suppress_duplicate_terminal_action(
            decision
        )

        decision = self._suppress_exhausted_special_action(
            decision
        )
        self.last_selected_decision = decision

        if not self._command_publisher_has_subscriber():
            return

        if decision.valid and self.general_motion_gate.locked:
            # A RUNNING general request owns the executor regardless of what
            # a newer Vision frame would otherwise select.
            self._reset_pre_motion_settle()
            return

        if (
            decision.valid
            and not self.general_motion_gate.has_required_fresh_vision()
        ):
            # After a general motion terminates, require new Vision before
            # publishing any executable decision, including special actions.
            self._reset_pre_motion_settle()
            return

        is_general_motion = (
            decision.valid
            and normalize_general_action(decision.action) is not None
        )
        if (
            is_general_motion
            and not self.general_motion_gate.can_publish(decision.action)
        ):
            # Keep receiving Vision and running the planner, but do not publish
            # another executable command while the current motion is locked.
            self._reset_pre_motion_settle()
            return

        if (
            is_general_motion
            and MotionDecisionNode._same_vision_frame_was_published(
                self,
                decision,
            )
        ):
            self._reset_pre_motion_settle()
            return

        if not self._pre_motion_settle_ready(decision, now):
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

        self._reset_pre_motion_settle()

        if is_general_motion:
            self.active_general_source = decision.source
            self.general_motion_gate.on_command_published(
                decision.action,
                self.command_id,
            )
            MotionDecisionNode._remember_published_vision_frame(
                self,
                decision,
            )

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

            heading_deg = line_info.get("filtered_heading_error_deg")
            if heading_deg is None:
                heading_deg = line_info.get("heading_error_deg")
            center_offset = line_info.get("filtered_lateral_offset_norm")
            if center_offset is None:
                center_offset = line_info.get("lateral_offset_norm")

            payload = {
                "phase": self.mission_phase,
                "source": (
                    selected.source.upper() if selected is not None else "NONE"
                ),
                "fresh_vision": fresh_vision,
                "source_fresh": bool(
                    selected is not None
                    and fresh_vision.get(selected.source, False)
                ),
                "line": {
                    "line_detected": bool(line_info.get("detected", False)),
                    "heading_deg": heading_deg,
                    "center_offset": center_offset,
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
                        line_config.line_large_heading_threshold_deg
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
                "execution": {
                    "motion_locked": self.general_motion_gate.locked,
                    "mission_locked": self.active_special_command_id is not None,
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
        """Return true when a stable motion candidate has settled long enough."""
        if (
            not decision.valid
            or decision.action not in self.PRE_MOTION_SETTLE_ACTIONS
        ):
            self._reset_pre_motion_settle()
            return True

        if (
            self.general_motion_gate.locked
            or self.active_special_command_id is not None
        ):
            self._reset_pre_motion_settle()
            return False

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
                f"duration={self.pre_motion_settle_sec:.3f}"
            )
            return self.pre_motion_settle_sec <= 0.0

        if now - self.pre_motion_settle_started_at < self.pre_motion_settle_sec:
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
        if self.active_special_command_id is not None:
            locked_phase = f"{planning_phase}_LOCK"
            return self.planner.plan(
                locked_phase,
                observations,
                dt_sec,
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

        return self.planner.plan(
            planning_phase,
            observations,
            dt_sec,
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


    def _suppress_exhausted_special_action(
        self,
        decision: MotionDecision,
    ) -> MotionDecision:
        """Block special actions whose mission failure limit is exhausted."""
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
