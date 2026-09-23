"""End-to-end mission phase flow tests without ROS graph execution."""

import json

import pytest
from std_msgs.msg import String

from mission_control.legacy_motion_executor_adapter import (
    build_executor_request,
    map_action_to_motion_id,
)
from mission_control.legacy_motion_status_adapter import (
    convert_executor_status,
)
from mission_control.executor_heartbeat_watchdog import (
    ExecutorHeartbeatWatchdog,
)
from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_command_gate import GeneralMotionCommandGate
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_executor_core import (
    ExecutorState,
    MotionExecutionResult,
)
from mission_control.motion_executor_node import (
    ExecutionPublicationState,
    parse_motion_request,
)
from mission_control.safety_interlock import SafetyInterlock


class CapturePublisher:
    """Collect JSON String messages in publication order."""

    def __init__(self):
        self.messages = []
        self.subscription_count = 1

    def publish(self, message):
        self.messages.append(json.loads(message.data))

    def get_subscription_count(self):
        return self.subscription_count


class FakeLogger:
    """Capture warnings while keeping flow tests quiet."""

    def __init__(self):
        self.warnings = []

    def info(self, _message):
        pass

    def warning(self, message):
        self.warnings.append(message)


class MissionFlowHarness:
    """Run real node decision methods against deterministic in-memory inputs."""

    # Timing tests explicitly enable the production shot delay.
    SHOT_PRE_MOTION_SETTLE_SEC = 0.0

    SOURCES = MotionDecisionNode.SOURCES
    PRE_MOTION_SETTLE_ACTIONS = MotionDecisionNode.PRE_MOTION_SETTLE_ACTIONS
    SPECIAL_ACTIONS = MotionDecisionNode.SPECIAL_ACTIONS
    SPECIAL_ACTION_SOURCES = MotionDecisionNode.SPECIAL_ACTION_SOURCES
    PICKUP_INITIAL_ALIGN_MARKER = (
        MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER
    )
    PICKUP_INITIAL_ALIGN_ACTIONS = (
        MotionDecisionNode.PICKUP_INITIAL_ALIGN_ACTIONS
    )
    PICKUP_FINE_ALIGN_MARKER = MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER
    PICKUP_FINE_ALIGN_ACTIONS = MotionDecisionNode.PICKUP_FINE_ALIGN_ACTIONS
    PICKUP_POST_BACKWARD_ALIGN_MARKER = (
        MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_MARKER
    )
    PICKUP_POST_BACKWARD_ALIGN_ACTIONS = (
        MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    )
    PICKUP_POSITIONING_LOSS_LATCH_ACTION = (
        MotionDecisionNode.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    )
    PICKUP_FIXED_SEQUENCE_FIRST_MOTION = (
        MotionDecisionNode.PICKUP_FIXED_SEQUENCE_FIRST_MOTION
    )
    BALL_POST_MOTION_DWELL_SEC = 0.0
    BALL_RAW_CONFIRMATION_RELEASE_SEC = (
        MotionDecisionNode.BALL_RAW_CONFIRMATION_RELEASE_SEC
    )
    SPECIAL_FAILURE_REASONS = (
        MotionDecisionNode.SPECIAL_FAILURE_REASONS
    )

    mission_phase = MotionDecisionNode.mission_phase
    required_pickups = MotionDecisionNode.required_pickups
    required_shots = MotionDecisionNode.required_shots
    required_ball_sections = MotionDecisionNode.required_ball_sections
    pickups_completed = MotionDecisionNode.pickups_completed
    shots_completed = MotionDecisionNode.shots_completed
    ball_sections_processed = MotionDecisionNode.ball_sections_processed
    finish_enabled = MotionDecisionNode.finish_enabled
    mission_complete = MotionDecisionNode.mission_complete
    active_special_action = MotionDecisionNode.active_special_action
    active_special_command_id = MotionDecisionNode.active_special_command_id
    special_motion_running = MotionDecisionNode.special_motion_running

    def __init__(
        self,
        phase="AUTO",
        required_ball_sections=2,
        max_pickup_failures=3,
        max_shot_failures=3,
        max_go_failures=3,
    ):
        self.phase_manager = MissionPhaseManager(
            initial_phase=phase,
            required_pickups=2,
            required_shots=2,
            required_ball_sections=required_ball_sections,
            max_pickup_failures=max_pickup_failures,
            max_shot_failures=max_shot_failures,
            max_go_failures=max_go_failures,
        )
        self.planner = MotionDecisionPlanner()
        self.general_motion_gate = GeneralMotionCommandGate()
        self.safety_interlock = SafetyInterlock()
        self.executor_heartbeat_watchdog = ExecutorHeartbeatWatchdog(
            started_at=0.0,
            startup_grace_sec=5.0,
            timeout_sec=2.0,
        )
        self.executor_heartbeat_watchdog.observe(
            sequence=0,
            observed_at=0.0,
        )
        self.executor_auto_ready = True
        self.executor_ready_requires_fresh_vision = False
        self.executor_active = False
        self.last_published_vision_stamp = {}
        self.active_general_source = None
        self.ball_approach_entry_pending = False
        self.ball_approach_alignment_pending = False
        self.ball_lost_during_motion_pending = False
        self.ball_last_visible_approach_info = None
        self.ball_last_visible_line_info = None
        self.ball_pickup_entry_pending = False
        self.ball_post_motion_dwell_until = None
        self.ball_confirmation_pending_latched = False
        self.ball_confirmation_last_raw_at = None
        self.active_line_motion_action = None
        self.active_line_motion_started_at = None
        self.active_line_motion_duration_sec = 0.0
        self.active_line_motion_target_frames = 0
        self.active_line_motion_frames = []
        self.line_timeout_recovery_active = False
        self.line_timeout_recovery_frames = []
        self.publisher = CapturePublisher()
        self._command_publisher_ready = None
        self.logger = FakeLogger()
        self.observations = {
            source: None for source in self.SOURCES
        }
        self.latest_info = self.observations
        self.latest_time = {source: None for source in self.SOURCES}
        self.command_id = 0
        self.event_id = 0
        self.pre_motion_settle_sec = 0.0
        self.pre_motion_settle_source = None
        self.pre_motion_settle_action = None
        self.pre_motion_settle_started_at = None
        self.terminal_latch = None
        self.terminal_action_armed = {
            source: True
            for source in self.SPECIAL_ACTION_SOURCES.values()
        }
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
        self.finish_min_confidence = 0.70
        self.previous_publish_time = 0.0
        self.last_candidate_decision = None
        self.last_selected_decision = None

    def get_logger(self):
        return self.logger

    def _fresh_observations(self, _now):
        return dict(self.observations), {
            source: 0.0 if info is not None else None
            for source, info in self.observations.items()
        }

    def _rearm_absent_terminal_targets(self, observations):
        MotionDecisionNode._rearm_absent_terminal_targets(
            self, observations
        )

    def _select_mission_decision(self, observations, dt_sec):
        return MotionDecisionNode._select_mission_decision(
            self, observations, dt_sec
        )

    def _suppress_duplicate_terminal_action(self, decision):
        return MotionDecisionNode._suppress_duplicate_terminal_action(
            self, decision
        )

    def _suppress_exhausted_special_action(self, decision):
        return MotionDecisionNode._suppress_exhausted_special_action(
            self, decision
        )

    def _mission_progress(self):
        return MotionDecisionNode._mission_progress(self)

    def _latch_critical_executor_fault(self, **kwargs):
        return MotionDecisionNode._latch_critical_executor_fault(self, **kwargs)

    def _command_publisher_has_subscriber(self):
        return MotionDecisionNode._command_publisher_has_subscriber(self)

    def _reset_pre_motion_settle(self):
        return MotionDecisionNode._reset_pre_motion_settle(self)

    def _pre_motion_settle_ready(self, decision, now):
        return MotionDecisionNode._pre_motion_settle_ready(
            self,
            decision,
            now,
        )

    def publish_vision(self, **observations):
        self.observations.update(observations)
        self.general_motion_gate.on_new_vision_input()
        before = len(self.publisher.messages)
        MotionDecisionNode._publish_decision(self)
        return self.publisher.messages[before:]

    def send_status(self, action, command_id, status, motion_id=None):
        message = String()
        event_id = (
            self.active_special_event_id
            if command_id == self.active_special_command_id
            else None
        )
        message.data = json.dumps(
            {
                "action": action,
                "command_id": command_id,
                "event_id": event_id,
                "dynamics_command": None,
                "status": status,
                "motion_id": motion_id,
            }
        )
        MotionDecisionNode._motion_status_callback(self, message)

    def send_phase(self, phase):
        message = String()
        message.data = phase
        MotionDecisionNode._phase_callback(self, message)


def line_info(heading=0.0, offset=0.0):
    return {
        "detected": True,
        "filtered_heading_error_deg": heading,
        "filtered_lateral_offset_norm": offset,
        "heading_quality": 0.95,
        "geometry_quality": 0.95,
        "detection_quality": 0.95,
        "turn_angle_deg": 0.0,
        "turn_consistency": 1.0,
    }


def complete_post_shot_exit(harness, monkeypatch, correction_heading=0.0):
    """Exercise the SHOT pause and every turn's fresh-frame checkpoint."""
    assert harness.mission_phase == "POST_SHOT_TURN"
    deadline = harness.post_shot_dwell_until
    clock = [deadline - 0.01]
    with monkeypatch.context() as patch:
        patch.setattr(
            "mission_control.motion_decision_node.time.monotonic", lambda: clock[0],
        )
        assert harness.publish_vision(line=line_info()) == []
        clock[0] = deadline
        assert harness.publish_vision(line=line_info()) == []
        turn = harness.publish_vision(line=line_info(), goal=None, hurdle=None)[-1]
        expected = (
            "POST_SHOT_TURN_RIGHT_9" if harness.ball_sections_processed <= 1
            else "POST_SHOT_TURN_LEFT_4"
        )
        assert turn["action"] == expected
        assert harness.publish_vision(line=line_info()) == []
        release_general(harness, turn)
        assert harness.mission_phase == "POST_SHOT_LINE_ALIGN"
        assert harness.observations["line"] is None
        assert harness.post_shot_dwell_until == clock[0] + 3.0
        clock[0] += 2.99
        assert harness.publish_vision(line=line_info(heading=correction_heading)) == []
        clock[0] = harness.post_shot_dwell_until
        assert harness.publish_vision(line=line_info(heading=correction_heading)) == []
        assert harness.observations["line"] is None

        if correction_heading:
            correction = harness.publish_vision(
                line=line_info(heading=correction_heading),
            )[-1]
            expected = (
                "POST_SHOT_LINE_TURN_RIGHT_3" if correction_heading > 0
                else "POST_SHOT_LINE_TURN_LEFT_2"
            )
            assert correction["action"] == expected
            release_general(harness, correction)
            assert harness.mission_phase == "POST_SHOT_LINE_ALIGN"
            assert harness.observations["line"] is None
            assert harness.post_shot_dwell_until == clock[0] + 3.0
            clock[0] += 2.99
            assert harness.publish_vision(line=line_info()) == []
            clock[0] = harness.post_shot_dwell_until
            assert harness.publish_vision(line=line_info()) == []

        # Forward immediately from the fresh aligned frame after the turn pause.
        forward = harness.publish_vision(line=line_info())[-1]
        assert forward["action"] == "POST_SHOT_FORWARD"
        assert harness.post_shot_dwell_until is None
        release_general(harness, forward)


@pytest.mark.parametrize("section", [1, 2])
@pytest.mark.parametrize("heading", [0.0, 34.0, -58.0])
def test_post_shot_exit_turn_alignment_and_forward(monkeypatch, section, heading):
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    harness.phase_manager.ball_sections_processed = section - 1
    mark_next_ball_grabbed(harness)
    shot = publish_special(harness, "goal", score_ready_goal(), "SHOT")
    complete_active(harness, "SHOT", shot["command_id"])
    deadline = harness.post_shot_dwell_until
    harness.send_status("SHOT", shot["command_id"], "SUCCEEDED")
    assert harness.post_shot_dwell_until == deadline
    complete_post_shot_exit(harness, monkeypatch, heading)
    assert harness.mission_phase == ("AUTO" if section == 1 else "LINE_TRACK")
    actions = [m["action"] for m in harness.publisher.messages]
    corrections = [a for a in actions if a.startswith("POST_SHOT_LINE_TURN_")]
    assert len(corrections) == (1 if heading else 0)
    assert actions[-1] == "POST_SHOT_FORWARD"


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED", "UNSUPPORTED"])
@pytest.mark.parametrize("phase,action", [
    ("POST_SHOT_TURN", "POST_SHOT_TURN_RIGHT_9"),
    ("POST_SHOT_LINE_ALIGN", "POST_SHOT_LINE_TURN_RIGHT_3"),
    ("POST_SHOT_FORWARD", "POST_SHOT_FORWARD"),
])
def test_post_shot_failure_holds_without_replay_or_forward(phase, action, status):
    harness = MissionFlowHarness(phase=phase)
    harness.phase_manager.ball_sections_processed = 1
    harness.general_motion_gate.on_new_vision_input()
    harness.general_motion_gate.on_command_published(action, 100)
    harness.active_general_source = "line"
    harness.send_status(action, 100, "RUNNING")
    harness.send_status(action, 100, status)
    decision = harness.publish_vision(line=line_info())[-1]
    assert decision["action"] == "WAIT"
    assert decision["reason"] == "post_shot_motion_failed"
    assert harness.mission_phase == phase
    assert harness.line_timeout_recovery_active is False


def test_post_shot_forward_rechecks_heading_after_dwell(monkeypatch):
    harness = MissionFlowHarness(phase="POST_SHOT_LINE_ALIGN")
    clock = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: clock[0],
    )
    assert harness.publish_vision(line=line_info()) == []
    assert harness.post_shot_dwell_until == 13.0
    clock[0] = 13.0
    assert harness.publish_vision(line=line_info(heading=34.0)) == []
    # Missing Line must not release the forward checkpoint.
    missing = harness.publish_vision(line=None)[-1]
    assert missing["action"] == "WAIT"
    correction = harness.publish_vision(line=line_info(heading=34.0))[-1]
    assert correction["action"] == "POST_SHOT_LINE_TURN_RIGHT_3"
    assert harness.mission_phase == "POST_SHOT_LINE_ALIGN"
    release_general(harness, correction)
    assert harness.publish_vision(line=line_info()) == []
    assert harness.post_shot_dwell_until == 16.0


def test_every_post_shot_correction_and_search_turn_waits_three_seconds(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: clock[0],
    )
    harness = MissionFlowHarness(phase="POST_SHOT_TURN")
    motion = harness.publish_vision(line=line_info())[-1]
    assert motion["action"] == "POST_SHOT_TURN_RIGHT_9"
    for line, expected in [
        (line_info(heading=34.0), "POST_SHOT_LINE_TURN_RIGHT_3"),
        (line_info(heading=34.0), "POST_SHOT_LINE_TURN_RIGHT_3"),
        ({"detected": False}, "POST_SHOT_LINE_TURN_RIGHT_2"),
        (line_info(heading=-58.0), "POST_SHOT_LINE_TURN_LEFT_2"),
        (line_info(), "POST_SHOT_FORWARD"),
    ]:
        clock[0] += 0.7
        release_general(harness, motion)
        deadline = clock[0] + 3.0
        assert harness.post_shot_dwell_until == deadline
        assert harness.post_shot_turn_settled is False
        clock[0] += 0.5
        harness.send_status(motion["action"], motion["command_id"], "SUCCEEDED")
        assert harness.post_shot_dwell_until == deadline
        clock[0] = deadline - 0.001
        assert harness.publish_vision(line=line) == []
        clock[0] = deadline
        assert harness.publish_vision(line=line) == []
        assert harness.observations["line"] is None
        motion = harness.publish_vision(line=line)[-1]
        assert motion["action"] == expected
        assert harness.post_shot_dwell_until is None


@pytest.mark.parametrize("phase", [
    "POST_SHOT_TURN", "POST_SHOT_LINE_ALIGN", "POST_SHOT_FORWARD",
])
def test_ball_callback_does_not_interrupt_post_shot_exit(phase):
    harness = MissionFlowHarness(phase=phase)
    message = String()
    message.data = json.dumps(approaching_ball())
    MotionDecisionNode._info_callback(harness, "ball")(message)
    assert harness.latest_info["ball"]["detected"] is True
    assert harness.ball_approach_entry_pending is False
    assert harness.ball_post_motion_dwell_until is None
    assert harness.mission_phase == phase


@pytest.mark.parametrize("phase", [
    "POST_BALL_LINE_ALIGN", "POST_BALL_GOAL_TRANSITION",
    "GOAL_SEARCH", "GOAL_APPROACH",
    "POST_SHOT_TURN", "POST_SHOT_LINE_ALIGN", "POST_SHOT_FORWARD",
])
@pytest.mark.parametrize("ball", [
    {"detected": False, "raw_detected": True},
    {"detected": True, "distance_m": 0.774},
    {"detected": True, "distance_m": 0.50},
])
def test_collected_ball_route_ignores_new_ball_navigation(phase, ball):
    harness = MissionFlowHarness(phase=phase)
    harness.phase_manager.pickups_completed = 1
    # A still-running command must retain its execution lock.
    harness.general_motion_gate.on_new_vision_input()
    harness.general_motion_gate.on_command_published("STRAIGHT_3", command_id=42)
    harness.active_general_source = "ball"
    receive = MotionDecisionNode._info_callback(harness, "ball")
    for sample in ({**approaching_ball(), **ball}, {"detected": False}):
        receive(String(data=json.dumps(sample)))

    assert harness.latest_info["ball"] == {"detected": False}
    assert harness.ball_approach_entry_pending is False
    assert harness.ball_approach_alignment_pending is False
    assert harness.ball_pickup_entry_pending is False
    assert harness.ball_confirmation_pending_latched is False
    assert harness.ball_lost_during_motion_pending is False
    assert harness.ball_post_motion_dwell_until is None
    assert harness.planner.ball_lock_active is False
    assert harness.planner.ball_tracking_active is False
    assert harness.planner.last_ball_turn_direction is None
    assert harness.general_motion_gate.locked is True
    assert harness.active_general_source == "ball"


def seed_old_ball_navigation(harness):
    """Reproduce reservations left behind by the previous Ball approach."""
    ball = {**approaching_ball(), "camera_center_offset_x_px": -100}
    harness.planner._update_ball_tracking(ball, 0.0)
    harness.planner.ball_lock_active = True
    harness.ball_approach_entry_pending = True
    harness.ball_approach_alignment_pending = True
    harness.ball_pickup_entry_pending = True
    harness.ball_lost_during_motion_pending = True
    harness.ball_last_visible_approach_info = ball
    harness.ball_last_visible_line_info = line_info()
    harness.ball_confirmation_pending_latched = True
    harness.ball_confirmation_last_raw_at = 10.0
    harness.ball_post_motion_dwell_until = 1000.0


@pytest.mark.parametrize("phase,expected_action", [
    ("POST_BALL_LINE_ALIGN", "POST_BALL_LINE_TURN_RIGHT_3"),
    ("GOAL_APPROACH", "GOAL_CAMERA90_FINE_FORWARD_2"),
])
def test_collected_ball_route_clears_stale_reservations_before_dwell(
    monkeypatch, phase, expected_action,
):
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: 10.0)
    harness = MissionFlowHarness(phase=phase)
    harness.phase_manager.pickups_completed = 1
    harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    seed_old_ball_navigation(harness)

    command = harness.publish_vision(
        line=line_info(heading=34.0), goal=approaching_goal(),
        ball={"detected": False, "raw_detected": False},
    )[-1]

    assert command["action"] == expected_action
    assert harness.ball_post_motion_dwell_until is None
    assert harness.ball_approach_entry_pending is False
    assert harness.ball_approach_alignment_pending is False
    assert harness.ball_pickup_entry_pending is False
    assert harness.ball_lost_during_motion_pending is False
    assert harness.ball_last_visible_approach_info is None
    assert harness.ball_last_visible_line_info is None
    assert harness.ball_confirmation_pending_latched is False
    assert harness.ball_confirmation_last_raw_at is None
    assert harness.planner.ball_tracking_active is False
    assert harness.planner.last_ball_turn_direction is None
    assert harness.planner.ball_lock_active is False
    assert harness.pickups_completed == 1
    assert harness.phase_manager.ball_grasp_results[1] == "GRABBED"


@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED"])
def test_pickup_terminal_clears_ball_reservations_only_on_success(status):
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(harness, "ball", pickup_ready_ball(), "PICKUP_NOW")
    seed_old_ball_navigation(harness)
    harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    complete_active(harness, "PICKUP_NOW", pickup["command_id"], terminal=status)
    assert harness.phase_manager.ball_grasp_results[1] == "GRABBED"
    if status == "SUCCEEDED":
        assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
        assert harness.ball_approach_alignment_pending is False
        assert harness.ball_pickup_entry_pending is False
        assert harness.ball_confirmation_pending_latched is False
        assert harness.ball_post_motion_dwell_until is None
        assert harness.planner.ball_tracking_active is False
    else:
        assert harness.mission_phase == "BALL_APPROACH"
        assert harness.planner.ball_tracking_active is True


def test_post_ball_detection_loss_cannot_interrupt_timed_line_or_goal(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(harness, "ball", pickup_ready_ball(), "PICKUP_NOW")
    harness.phase_manager.record_active_pickup_grasp_result("GRABBED")
    complete_active(harness, "PICKUP_NOW", pickup["command_id"])
    receive = MotionDecisionNode._info_callback(harness, "ball")
    receive(String(data=json.dumps({"detected": False, "raw_detected": True})))
    line = harness.publish_vision(line=line_info())[-1]
    assert line["source"] == "line"
    assert line["mission_progress"]["ball_mode_active"] is False
    release_general(harness, line)
    receive(String(data=json.dumps(approaching_ball())))
    now[0] = 20.0
    transition = harness.publish_vision(line=line_info())[-1]
    assert transition["action"] == "POST_BALL_GOAL_TRANSITION"
    receive(String(data=json.dumps({"detected": False, "raw_detected": True})))
    assert all(c["action"] == "WAIT" for c in harness.publish_vision())
    complete_active(harness, transition["action"], transition["command_id"])
    assert harness.mission_phase == "GOAL_APPROACH"
    assert harness.goal_post_motion_dwell_until is None
    assert harness.publish_vision(goal=approaching_goal())[-1]["action"] == "GOAL_CAMERA90_FINE_FORWARD_2"


def test_post_shot_return_requires_new_ball_frame_and_resumes_acquisition(monkeypatch):
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    harness.phase_manager.pickups_completed = 1
    mark_next_ball_grabbed(harness)
    shot = publish_special(harness, "goal", score_ready_goal(), "SHOT")
    complete_active(harness, "SHOT", shot["command_id"])
    receive = MotionDecisionNode._info_callback(harness, "ball")
    receive(String(data=json.dumps(approaching_ball())))
    complete_post_shot_exit(harness, monkeypatch)

    assert harness.mission_phase == "AUTO"
    assert harness.latest_info["ball"] is None
    assert harness.latest_time["ball"] is None
    assert harness.ball_approach_alignment_pending is False
    assert harness.planner.ball_tracking_active is False
    assert harness.planner.goal_lock_active is False
    assert harness.planner.goal_tracking_active is False
    assert harness.latest_info["goal"] is None
    receive(String(data=json.dumps(approaching_ball())))
    assert harness.ball_approach_entry_pending is True
    assert harness.planner.ball_tracking_active is True
    command = harness.publish_vision(line=line_info())[-1]
    assert command["source"] == "ball"
    assert command["action"] != "WAIT"


def test_post_shot_exit_ignores_mismatched_and_duplicate_turn_status():
    harness = MissionFlowHarness(phase="POST_SHOT_TURN")
    turn = harness.publish_vision(line=line_info())[-1]
    action = turn["action"]
    harness.send_status(action, turn["command_id"] + 1, "SUCCEEDED")
    assert harness.mission_phase == "POST_SHOT_TURN"
    release_general(harness, turn)
    assert harness.mission_phase == "POST_SHOT_LINE_ALIGN"
    harness.send_status(action, turn["command_id"], "SUCCEEDED")
    assert harness.mission_phase == "POST_SHOT_LINE_ALIGN"


def pickup_ready_ball():
    return {
        "detected": True,
        "confidence": 0.95,
        "bearing_deg": 0.0,
        "offset_x_px": 0,
        "offset_x_norm": 0.0,
        "camera_center_offset_x_px": 0,
        "bottom_distance_px": 500,
        "pickup_x_tolerance_norm": 0.08,
        "depth_m": 0.40,
        "ground_distance_m": 0.15,
        "distance_m": 0.40,
        "depth_valid": True,
        "depth_age_sec": 0.05,
        "pickup_ready": True,
        "pickup_now": True,
        "is_in_pickup_window": True,
    }


def score_ready_goal():
    return {
        "detected": True,
        "confidence": 0.95,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "offset_x_px": 0,
        "depth_m": 0.795,
        "ground_distance_m": 0.795,
        "distance_m": 0.795,
        "depth_valid": True,
        "score_now": True,
    }


def go_ready_hurdle():
    return {
        "detected": True,
        "confirmation_confirmed": True,
        "confidence": 0.95,
        "depth_m": 0.10,
        "distance_m": 0.10,
        "depth_valid": True,
        "ground_gap_m": 0.10,
        "camera_bottom_gap_m": 0.05,
        "hurdle_angle_deg": 0.0,
        "go_now": True,
    }


def confirmed_finish():
    return {
        "detected": True,
        "confirmed": True,
        "confidence": 0.95,
    }


def approaching_ball():
    info = pickup_ready_ball()
    info.update(
        {
            "depth_m": 0.80,
            "ground_distance_m": 0.80,
            "distance_m": 0.80,
            "pickup_ready": False,
            "pickup_now": False,
        }
    )
    return info


def approaching_goal():
    info = score_ready_goal()
    info.update(
        {
            "depth_m": 0.9,
            "ground_distance_m": 0.9,
            "distance_m": 0.9,
            "score_now": False,
        }
    )
    return info


def aligning_goal():
    info = score_ready_goal()
    info.update(
        {
            "bearing_deg": 8.0,
            "offset_x_norm": 0.30,
            "offset_x_px": 192,
            "depth_m": 0.9,
            "score_now": False,
        }
    )
    return info


def approaching_hurdle():
    info = go_ready_hurdle()
    info.update(
        {
            "depth_m": 0.35,
            "distance_m": 0.35,
            "ground_gap_m": 0.35,
            "camera_bottom_gap_m": 0.20,
            "go_now": False,
        }
    )
    return info


def publish_special(harness, source, observation, action):
    published = harness.publish_vision(**{source: observation})
    matching = [
        payload for payload in published if payload["action"] == action
    ]
    assert len(matching) == 1
    payload = matching[0]
    assert payload["command_id"] == harness.active_special_command_id
    assert payload["sdk_motion_requested"] is True
    return payload


def mark_next_ball_grabbed(harness):
    """Seed the verified precondition for existing SHOT flow tests."""
    ball_index = harness.phase_manager.ball_sections_processed + 1
    harness.phase_manager.ball_grasp_results[ball_index] = "GRABBED"


def complete_active(harness, action, command_id, terminal="SUCCEEDED"):
    harness.send_status(action, command_id, "RUNNING")
    harness.send_status(action, command_id, terminal)


def release_general(harness, payload):
    harness.send_status(
        payload["action"],
        payload["command_id"],
        "RUNNING",
    )
    harness.send_status(
        payload["action"],
        payload["command_id"],
        "SUCCEEDED",
    )


def test_ball_pickup_entry_waits_for_general_motion_dwell(monkeypatch):
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic",
        lambda: now[0],
    )
    approach = harness.publish_vision(ball=approaching_ball())[-1]
    assert approach["action"] == "STRAIGHT_2"

    close_ball = pickup_ready_ball()
    close_ball.update(
        {
            "depth_m": 0.56,
            "distance_m": 0.56,
            "pickup_ready": False,
            "pickup_now": False,
        }
    )
    MotionDecisionNode._latch_ball_pickup_entry(harness, close_ball)

    assert harness.ball_pickup_entry_pending is True
    release_general(harness, approach)

    assert harness.ball_post_motion_dwell_until == pytest.approx(13.0)
    now[0] = 12.999
    assert harness.publish_vision(ball={"detected": False}) == []
    now[0] = 13.0
    assert harness.publish_vision(ball={"detected": False}) == []
    now[0] = 13.1
    published = harness.publish_vision(ball=close_ball)
    assert published[-1]["action"] == "PICKUP_NOW"
    assert published[-1]["reason"] == (
        "ball_pickup_entry_latched_during_motion"
    )
    assert harness.active_special_action == "PICKUP_NOW"
    assert harness.ball_pickup_entry_pending is False

    harness.send_status(
        "PICKUP_NOW",
        published[-1]["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    assert harness.pickup_initial_align_waiting is True
    assert harness.observations["ball"] is None

    waiting = harness.publish_vision(line=line_info())[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["source"] == "ball"

    right_ball = pickup_ready_ball()
    right_ball["steering_angle_deg"] = 26.0
    right_ball["offset_x_norm"] = 0.2
    alignment = harness.publish_vision(ball=right_ball)[-1]
    assert alignment["action"] == (
        "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_2"
    )


def test_ball_loss_recovery_turn_dwells_then_approaches_on_fresh_ball(
    monkeypatch,
):
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic",
        lambda: now[0],
    )
    visible_ball = approaching_ball()
    visible_ball["steering_angle_deg"] = -31.0
    visible_ball["camera_center_offset_x_px"] = -100
    remembered_line = line_info(offset=-0.24)
    approach = harness.publish_vision(
        ball=visible_ball,
        line=remembered_line,
    )[-1]
    harness.ball_last_visible_line_info = remembered_line

    MotionDecisionNode._track_ball_loss_during_motion(
        harness,
        visible_ball,
    )
    lost_ball = {"detected": False, "raw_detected": False}
    MotionDecisionNode._track_ball_loss_during_motion(harness, lost_ball)
    release_general(harness, approach)

    assert harness.ball_post_motion_dwell_until == pytest.approx(13.0)
    now[0] = 12.999
    assert harness.publish_vision(ball=lost_ball) == []
    now[0] = 13.0
    assert harness.publish_vision(ball=lost_ball) == []
    now[0] = 13.1
    recovery = harness.publish_vision(ball=lost_ball)[-1]
    assert recovery["action"] == "BALL_APPROACH_TURN_LEFT_2"

    now[0] = 14.0
    release_general(harness, recovery)
    assert harness.ball_post_motion_dwell_until == pytest.approx(17.0)
    assert harness.ball_approach_alignment_pending is False
    misaligned_ball = approaching_ball()
    misaligned_ball["steering_angle_deg"] = 26.0
    now[0] = 16.999
    assert harness.publish_vision(ball=misaligned_ball) == []
    now[0] = 17.0
    assert harness.publish_vision(ball=misaligned_ball) == []
    assert harness.latest_info["ball"] is None
    now[0] = 17.1
    fresh_decision = harness.publish_vision(ball=misaligned_ball)[-1]
    assert fresh_decision["action"] == "STRAIGHT_2"


def test_ball_motion_without_pickup_entry_keeps_three_second_dwell():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    approach = harness.publish_vision(ball=approaching_ball())[-1]

    release_general(harness, approach)

    assert harness.ball_pickup_entry_pending is False
    assert harness.ball_post_motion_dwell_until is not None
    assert harness.publish_vision(ball=approaching_ball()) == []


def test_idle_ball_pickup_entry_crossing_is_latched_without_general_dwell():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    close_ball = pickup_ready_ball()
    close_ball.update(
        {
            "depth_m": 0.56,
            "distance_m": 0.56,
            "pickup_ready": False,
            "pickup_now": False,
        }
    )

    MotionDecisionNode._latch_ball_pickup_entry(harness, close_ball)

    assert harness.ball_pickup_entry_pending is True
    assert harness.ball_post_motion_dwell_until is None
    published = harness.publish_vision(ball=close_ball)[-1]
    assert published["action"] == "PICKUP_NOW"


def test_failed_ball_motion_discards_latched_pickup_entry():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    approach = harness.publish_vision(ball=approaching_ball())[-1]
    close_ball = pickup_ready_ball()
    close_ball["depth_m"] = 0.56
    close_ball["distance_m"] = 0.56
    MotionDecisionNode._latch_ball_pickup_entry(harness, close_ball)

    harness.send_status(
        approach["action"],
        approach["command_id"],
        "RUNNING",
    )
    harness.send_status(
        approach["action"],
        approach["command_id"],
        "FAILED",
    )

    assert harness.ball_pickup_entry_pending is False


def test_pickup_fine_alignment_rechecks_fresh_ball_under_atomic_lock():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER,
    )
    assert harness.observations["ball"] is None
    assert harness.pickup_fine_align_waiting is True
    assert harness.active_special_command_id == pickup["command_id"]

    waiting = harness.publish_vision(
        line=line_info(),
        goal=score_ready_goal(),
        hurdle=go_ready_hurdle(),
    )[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["source"] == "ball"

    right_ball = pickup_ready_ball()
    right_ball.update(
        {
            "offset_x_norm": 0.2,
            "offset_x_px": 56,
            "bottom_distance_px": 200,
            "pickup_ready": False,
            "is_in_pickup_window": False,
        }
    )
    right = harness.publish_vision(ball=right_ball)[-1]
    assert right["action"] == "BALL_PICKUP_CRAB_RIGHT"
    assert right["active_special_command_id"] == pickup["command_id"]
    assert harness.pickup_fine_align_waiting is False

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER,
    )
    assert harness.observations["ball"] is None

    centered_ball = {**pickup_ready_ball(), "bottom_distance_px": 200}
    centered = harness.publish_vision(ball=centered_ball)[-1]
    assert centered["action"] == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    assert centered["active_special_command_id"] == pickup["command_id"]
    assert harness.active_special_action == "PICKUP_NOW"


def test_pickup_positioning_loss_returns_to_fresh_camera_down_alignment():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    harness.pickup_positioning_motion_running = True
    harness.pickup_positioning_motion_id = "ball_camera_down_forward_2"
    harness.pickup_positioning_ball_seen_during_motion = True
    lost_ball = {"detected": False, "raw_detected": False}
    MotionDecisionNode._track_pickup_positioning_ball_loss(
        harness,
        lost_ball,
    )

    latch = harness.publish_vision(ball=lost_ball)[-1]
    assert latch["action"] == harness.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    assert latch["sdk_motion_requested"] is False
    assert latch["active_special_command_id"] == pickup["command_id"]

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    assert harness.pickup_initial_align_waiting is True
    waiting = harness.publish_vision(ball=lost_ball)[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["sdk_motion_requested"] is False

    left_ball = pickup_ready_ball()
    left_ball["steering_angle_deg"] = -30.0
    left_ball["offset_x_norm"] = -0.2
    turn = harness.publish_vision(ball=left_ball)[-1]
    assert turn["action"] == "BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_1"
    assert harness.pickup_positioning_ball_lost_pending is False


def test_pickup_initial_alignment_requires_fresh_ball_and_depth():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    assert harness.observations["ball"] is None
    assert harness.pickup_initial_align_waiting is True
    assert harness.active_special_command_id == pickup["command_id"]

    waiting = harness.publish_vision(line=line_info())[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["source"] == "ball"

    right_ball = pickup_ready_ball()
    right_ball.update(
        {
            "steering_angle_deg": 24.0,
            "bearing_deg": -20.0,
            "offset_x_norm": 0.2,
            "pickup_ready": False,
        }
    )
    right = harness.publish_vision(ball=right_ball)[-1]
    assert right["action"] == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_2"
    assert right["active_special_command_id"] == pickup["command_id"]
    assert harness.pickup_initial_align_waiting is False

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    centered = pickup_ready_ball()
    centered.update(
        {"depth_m": 0.55, "distance_m": 0.44, "pickup_ready": False}
    )
    approach = harness.publish_vision(ball=centered)[-1]
    assert approach["action"] == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert approach["source_command"]["pickup_approach_motion"] == "STRAIGHT_0"
    assert approach["source_command"]["distance_m"] == 0.44


def test_full_course_mock_flow_without_ros_graph(monkeypatch):
    harness = MissionFlowHarness(required_ball_sections=1)

    straight = harness.publish_vision(line=line_info())
    assert [item["action"] for item in straight] == ["STRAIGHT"]
    repeated = harness.publish_vision(line=line_info())
    assert repeated == []
    release_general(harness, straight[0])

    recovery = harness.publish_vision(line=line_info(offset=-0.24))
    assert [item["action"] for item in recovery] == ["STRAIGHT"]
    assert recovery[0]["valid"] is True
    assert recovery[0]["reason"] == "line_tracking"
    assert map_action_to_motion_id(recovery[0]["action"]) == "forward"
    assert harness.general_motion_gate.locked is True
    release_general(harness, recovery[0])

    large = harness.publish_vision(line=line_info(offset=0.35))
    assert [item["action"] for item in large] == ["STRAIGHT"]
    assert large[0]["valid"] is True
    assert map_action_to_motion_id(large[0]["action"]) == "forward"
    release_general(harness, large[0])

    approach_ball = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
    )
    assert approach_ball[-1]["source"] == "ball"
    assert approach_ball[-1]["action"] == "STRAIGHT_2"
    release_general(harness, approach_ball[-1])

    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    pickup_id = pickup["command_id"]

    harness.send_status("PICKUP_NOW", pickup_id + 10, "RUNNING")
    harness.send_status("SHOT", pickup_id, "RUNNING")
    assert harness.active_special_command_id == pickup_id
    assert harness.pickups_completed == 0

    harness.send_status("PICKUP_NOW", pickup_id, "RUNNING")
    assert harness.special_motion_running is True
    locked = harness.publish_vision(
        line=line_info(offset=0.35),
        ball=None,
        hurdle=None,
    )
    assert locked[-1]["action"] == "WAIT"
    assert locked[-1]["requires_ack"] is False
    assert harness.active_special_command_id == pickup_id

    harness.phase_manager.record_active_pickup_grasp_result("GRABBED")
    harness.send_status("PICKUP_NOW", pickup_id, "SUCCEEDED")
    harness.send_status("PICKUP_NOW", pickup_id, "SUCCEEDED")
    assert harness.pickups_completed == 1
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
    assert harness.active_special_command_id is None

    line = harness.publish_vision(line=line_info())[-1]
    assert line["action"] == "STRAIGHT"
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"
    release_general(harness, line)
    now = [harness.post_ball_line_run_until]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: max(
        now[0], getattr(harness, "goal_post_motion_dwell_until", None) or 0.0,
    ))
    line_aligned = harness.publish_vision(line=line_info())
    assert line_aligned[-1]["action"] == "POST_BALL_GOAL_TRANSITION"

    transition = line_aligned[-1]
    complete_active(
        harness,
        "POST_BALL_GOAL_TRANSITION",
        transition["command_id"],
    )
    assert harness._mission_progress()["ball_mode_active"] is False
    assert harness.mission_phase == "GOAL_APPROACH"
    assert harness.goal_post_motion_dwell_until is None

    goal_approach = harness.publish_vision(
        ball=None,
        hurdle=None,
        goal=approaching_goal(),
    )
    assert goal_approach[-1]["action"] == "GOAL_CAMERA90_FINE_FORWARD_2"
    release_general(harness, goal_approach[-1])
    MotionDecisionNode._publish_decision(harness)
    goal_align = harness.publish_vision(goal={**aligning_goal(), "bearing_deg": 15.0})
    assert goal_align[-1]["action"] == "GOAL_CAMERA90_TURN_RIGHT_2"
    harness.send_status(
        "GOAL_CAMERA90_TURN_RIGHT_2",
        goal_align[-1]["command_id"],
        "UNSUPPORTED",
    )

    mark_next_ball_grabbed(harness)
    shot = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    shot_id = shot["command_id"]
    harness.send_status("SHOT", shot_id, "RUNNING")
    shot_locked = harness.publish_vision(
        line=line_info(),
        goal=score_ready_goal(),
        hurdle=go_ready_hurdle(),
    )
    assert shot_locked[-1]["action"] == "WAIT"
    harness.send_status("SHOT", shot_id, "SUCCEEDED")

    complete_post_shot_exit(harness, monkeypatch)
    assert harness.shots_completed == 1
    assert harness.ball_sections_processed == 1
    assert harness.finish_enabled is True
    assert harness.mission_phase == "LINE_TRACK"
    assert harness.mission_complete is False

    hurdle_approach = harness.publish_vision(
        ball=None,
        goal=None,
        hurdle=approaching_hurdle(),
    )
    assert hurdle_approach[-1]["source"] == "hurdle"
    assert hurdle_approach[-1]["action"] == "STRAIGHT_2"
    release_general(harness, hurdle_approach[-1])

    go = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    go_id = go["command_id"]
    assert go["action"] == "GO"
    assert map_action_to_motion_id("GO") == "forward"
    assert go["action"] != "STRAIGHT"

    harness.send_status("GO", go_id, "RUNNING")
    go_locked = harness.publish_vision(
        line=line_info(),
        hurdle=go_ready_hurdle(),
    )
    assert go_locked[-1]["action"] == "WAIT"
    harness.send_status("GO", go_id, "SUCCEEDED")
    assert harness.mission_phase == "LINE_TRACK"

    harness.planner._clear_ball_tracking()
    harness.planner._clear_goal_tracking()
    final_line = harness.publish_vision(
        line=line_info(),
        ball=None,
        goal=None,
        hurdle=None,
        finish=confirmed_finish(),
    )
    assert final_line[-1]["action"] == "STRAIGHT"
    assert final_line[-1]["phase"] == "LINE_TRACK"
    assert harness.mission_phase == "LINE_TRACK"
    assert harness.mission_complete is False
    assert all(
        item["action"] != "CROSS_FINISH"
        for item in harness.publisher.messages
    )
    assert all(
        item["phase"] != "FINISHED"
        for item in harness.publisher.messages
    )


@pytest.mark.parametrize(
    ("action", "phase", "source", "observation"),
    [
        ("PICKUP_NOW", "BALL_APPROACH", "ball", pickup_ready_ball()),
        ("SHOT", "GOAL_APPROACH", "goal", score_ready_goal()),
        ("GO", "HURDLE_APPROACH", "hurdle", go_ready_hurdle()),
    ],
)
@pytest.mark.parametrize("terminal", ["FAILED", "TIMEOUT"])
def test_special_failure_or_timeout_returns_to_safe_approach_phase(
    action,
    phase,
    source,
    observation,
    terminal,
):
    harness = MissionFlowHarness(phase=phase)
    if action == "SHOT":
        mark_next_ball_grabbed(harness)
    command = publish_special(
        harness,
        source,
        observation,
        action,
    )
    complete_active(
        harness,
        action,
        command["command_id"],
        terminal,
    )

    expected_phase = {
        "PICKUP_NOW": "BALL_APPROACH",
        "SHOT": "GOAL_APPROACH",
        "GO": "HURDLE_APPROACH",
    }[action]
    assert harness.mission_phase == expected_phase
    assert harness.ball_sections_processed == 0
    assert harness.active_special_command_id is None


@pytest.mark.parametrize(
    (
        "action",
        "phase",
        "source",
        "observation",
        "reason",
    ),
    [
        (
            "PICKUP_NOW",
            "BALL_APPROACH",
            "ball",
            pickup_ready_ball(),
            "pickup_failures_exhausted",
        ),
        (
            "SHOT",
            "GOAL_APPROACH",
            "goal",
            score_ready_goal(),
            "shot_failures_exhausted",
        ),
        (
            "GO",
            "HURDLE_APPROACH",
            "hurdle",
            go_ready_hurdle(),
            "go_failures_exhausted",
        ),
    ],
)
def test_special_failure_limit_blocks_reexecution_after_target_reappears(
    action,
    phase,
    source,
    observation,
    reason,
):
    harness = MissionFlowHarness(
        phase=phase,
        max_pickup_failures=1,
        max_shot_failures=1,
        max_go_failures=1,
    )
    if action == "SHOT":
        mark_next_ball_grabbed(harness)

    first = publish_special(
        harness,
        source,
        observation,
        action,
    )

    complete_active(
        harness,
        action,
        first["command_id"],
        "FAILED",
    )

    assert harness.phase_manager.special_action_exhausted(action)

    if source == "ball":
        harness.planner._clear_ball_tracking()
    elif source == "goal":
        harness.planner._clear_goal_tracking()

    harness.publish_vision(
        **{source: None}
    )

    assert harness.terminal_action_armed[source] is True

    reappeared = harness.publish_vision(
        **{source: observation}
    )

    decision = reappeared[-1]

    assert decision["action"] == "WAIT"
    assert decision["valid"] is False
    assert decision["reason"] == reason
    assert harness.active_special_command_id is None

    special_commands = [
        item
        for item in harness.publisher.messages
        if item["action"] == action
    ]
    assert len(special_commands) == 1


def test_full_flow_line_loss_stops_until_reacquisition():
    planner = MotionDecisionPlanner()
    observations = {
        source: None for source in MotionDecisionNode.SOURCES
    }

    observations["line"] = line_info(offset=-0.35)
    large = planner.plan("LINE_TRACK", observations, 0.1)
    assert large.action == "STRAIGHT"

    observations["line"] = None
    first_missing = planner.plan("LINE_TRACK", observations, 0.1)
    recovered = planner.plan("LINE_TRACK", observations, 0.1)
    assert first_missing.action == "STOP"
    assert recovered.action == "STOP"

    observations["line"] = line_info()
    reacquired = planner.plan("LINE_TRACK", observations, 0.1)
    assert reacquired.action == "STRAIGHT"

    observations["line"] = line_info(offset=0.35)
    planner.plan("LINE_TRACK", observations, 0.1)
    observations["line"] = None
    repeated_missing = [
        planner.plan("LINE_TRACK", observations, 0.1)
        for _ in range(4)
    ]
    assert all(item.action == "STOP" for item in repeated_missing)
    assert all(
        item.reason == "waiting_for_line_info"
        for item in repeated_missing
    )


@pytest.mark.parametrize(
    "phase",
    ["AUTO", "BALL_APPROACH", "LINE_TRACK"],
)
def test_hurdle_selection_does_not_depend_on_course_phase(phase):
    harness = MissionFlowHarness(phase=phase)

    published = harness.publish_vision(
        line=line_info(),
        hurdle=go_ready_hurdle(),
    )
    commands = [item for item in published if item["action"] == "GO"]
    assert len(commands) == 1
    command = commands[0]

    assert command["phase"] == phase
    assert command["source"] == "hurdle"


def test_go_ready_confirmed_hurdle_beats_pickup_ready_ball():
    harness = MissionFlowHarness()

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        hurdle=go_ready_hurdle(),
    )

    assert published[-1]["source"] == "hurdle"
    assert published[-1]["action"] == "GO"
    assert harness.active_special_action == "GO"


def test_confirmed_approaching_hurdle_beats_approaching_ball():
    harness = MissionFlowHarness()

    published = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
        hurdle=approaching_hurdle(),
    )

    assert published[-1]["source"] == "hurdle"
    assert published[-1]["action"] == "STRAIGHT_2"


def test_goal_approach_resumes_after_hurdle_go_and_ignores_new_ball():
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )

    harness.send_status("GO", command["command_id"], "RUNNING")
    locked = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=score_ready_goal(),
        hurdle=go_ready_hurdle(),
    )
    assert locked[-1]["action"] == "WAIT"

    harness.send_status("GO", command["command_id"], "SUCCEEDED")
    assert harness.mission_phase == "GOAL_APPROACH"

    mark_next_ball_grabbed(harness)
    resumed = harness.publish_vision(
        ball=pickup_ready_ball(),
        goal=score_ready_goal(),
        hurdle=None,
    )
    assert resumed[-1]["source"] == "goal"
    assert resumed[-1]["action"] == "SHOT"


def test_hurdle_can_appear_after_shot_without_fixed_order(monkeypatch):
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=1,
    )
    mark_next_ball_grabbed(harness)
    shot = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", shot["command_id"])
    complete_post_shot_exit(harness, monkeypatch)
    assert harness.mission_phase == "LINE_TRACK"

    hurdle = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )

    assert hurdle["source"] == "hurdle"
    assert hurdle["command_id"] > shot["command_id"]


def test_missing_or_stale_hurdle_preserves_active_hurdle_lock():
    harness = MissionFlowHarness()
    no_hurdle = harness.publish_vision(
        line=line_info(),
        hurdle=None,
    )
    assert no_hurdle[-1]["action"] == "STRAIGHT"
    release_general(harness, no_hurdle[-1])

    seen = harness.publish_vision(
        line=line_info(),
        hurdle=approaching_hurdle(),
    )
    assert seen[-1]["source"] == "hurdle"
    assert seen[-1]["action"] == "STRAIGHT_2"
    release_general(harness, seen[-1])

    missing = harness.publish_vision(hurdle=None)
    assert missing[-1]["source"] == "hurdle"
    assert missing[-1]["action"] == "WAIT"
    assert missing[-1]["reason"] == "waiting_for_hurdle_info"

    harness.latest_info = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    harness.latest_time = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    harness.timeouts = {
        source: 0.5 for source in MotionDecisionNode.SOURCES
    }
    harness.latest_info["line"] = line_info()
    harness.latest_time["line"] = 1.5
    harness.latest_info["hurdle"] = go_ready_hurdle()
    harness.latest_time["hurdle"] = 1.0
    harness.terminal_action_armed["hurdle"] = False

    observations, ages = MotionDecisionNode._fresh_observations(
        harness,
        now=1.6,
    )
    MotionDecisionNode._rearm_absent_terminal_targets(
        harness,
        observations,
    )
    decision = harness.planner.plan("AUTO", observations, 0.1)

    assert observations["hurdle"] is None
    assert ages["hurdle"] == pytest.approx(0.6)
    assert decision.source == "hurdle"
    assert decision.action == "WAIT"
    assert decision.reason == "waiting_for_hurdle_info"
    assert harness.terminal_action_armed["hurdle"] is True


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        (
            {"confirmation_confirmed": False},
            "hurdle_confirmation_pending",
        ),
        (
            {"depth_valid": False},
            "hurdle_depth_invalid_wait",
        ),
    ],
)
def test_unconfirmed_or_invalid_depth_hurdle_does_not_preempt_ball(
    overrides,
    reason,
):
    harness = MissionFlowHarness()
    hurdle = go_ready_hurdle()
    hurdle.update(overrides)

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        hurdle=hurdle,
    )
    decision = published[-1]

    assert decision["source"] == "ball"
    assert decision["action"] == "PICKUP_NOW"
    assert decision["valid"] is True
    assert harness.active_special_action == "PICKUP_NOW"


def test_unconfirmed_hurdle_disappearance_resumes_ball_mission():
    harness = MissionFlowHarness()
    hurdle = go_ready_hurdle()
    hurdle["confirmation_confirmed"] = False

    held = harness.publish_vision(
        ball=approaching_ball(),
        hurdle=hurdle,
    )
    assert held[-1]["source"] == "ball"
    assert held[-1]["action"] == "STRAIGHT_2"
    release_general(harness, held[-1])

    resumed = harness.publish_vision(hurdle=None)
    assert resumed[-1]["source"] == "ball"
    assert resumed[-1]["action"] == "STRAIGHT_2"


def test_unconfirmed_hurdle_can_become_confirmed_and_take_priority():
    harness = MissionFlowHarness()
    hurdle = approaching_hurdle()
    hurdle["confirmation_confirmed"] = False

    held = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
        hurdle=hurdle,
    )
    assert held[-1]["source"] == "ball"
    assert held[-1]["action"] == "STRAIGHT_2"
    release_general(harness, held[-1])

    hurdle["confirmation_confirmed"] = True
    selected = harness.publish_vision(hurdle=hurdle)
    assert selected[-1]["source"] == "hurdle"
    assert selected[-1]["action"] == "STRAIGHT_2"


def test_absent_hurdle_keeps_ball_priority_over_line():
    harness = MissionFlowHarness()

    published = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
        hurdle=None,
    )

    assert published[-1]["source"] == "ball"
    assert published[-1]["action"] == "STRAIGHT_2"


def test_go_running_and_terminal_suppress_same_hurdle_observation():
    harness = MissionFlowHarness()
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    command_id = command["command_id"]

    harness.send_status("GO", command_id, "RUNNING")
    running = harness.publish_vision(hurdle=go_ready_hurdle())
    assert running[-1]["action"] == "WAIT"
    assert running[-1]["reason"] == (
        "mission_locked_waiting_for_motion_status"
    )

    harness.send_status("GO", command_id, "SUCCEEDED")
    retained = harness.publish_vision(hurdle=go_ready_hurdle())
    assert retained[-1]["action"] == "WAIT"
    assert retained[-1]["reason"] == (
        "duplicate_terminal_action_suppressed"
    )
    assert harness.active_special_command_id is None
    assert harness.terminal_action_armed["hurdle"] is False
    assert not any(
        item["action"] == "GO"
        and item["command_id"] > command_id
        for item in harness.publisher.messages
    )


def test_completed_hurdle_mission_lock_waits_when_hurdle_disappears():
    harness = MissionFlowHarness()
    hurdle = go_ready_hurdle()
    first = publish_special(harness, "hurdle", hurdle, "GO")
    complete_active(harness, "GO", first["command_id"])

    retained = harness.publish_vision(hurdle=hurdle)
    assert retained[-1]["action"] == "WAIT"

    resumed = harness.publish_vision(
        ball=pickup_ready_ball(),
        hurdle=None,
    )
    assert resumed[-1]["source"] == "hurdle"
    assert resumed[-1]["action"] == "WAIT"
    assert resumed[-1]["reason"] == "waiting_for_hurdle_info"


def test_hurdle_can_reappear_at_a_different_position_after_absence():
    harness = MissionFlowHarness()
    first_hurdle = go_ready_hurdle()
    first_hurdle.update({"offset_x_norm": -0.70, "depth_m": 0.10})
    first = publish_special(harness, "hurdle", first_hurdle, "GO")
    complete_active(harness, "GO", first["command_id"])

    retained = harness.publish_vision(hurdle=first_hurdle)
    assert retained[-1]["action"] == "WAIT"

    absent = harness.publish_vision(hurdle=None)
    assert absent[-1]["action"] == "WAIT"
    assert harness.terminal_action_armed["hurdle"] is True

    second_hurdle = go_ready_hurdle()
    second_hurdle.update({"offset_x_norm": 0.75, "depth_m": 0.12})
    second = publish_special(harness, "hurdle", second_hurdle, "GO")

    assert second["command_id"] > first["command_id"]
    assert harness.observations["hurdle"]["offset_x_norm"] == 0.75
    assert second["source_command"]["depth_m"] == 0.12


def test_pickup_success_publishes_once_and_advances_to_goal():
    harness = MissionFlowHarness()
    command = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    command_id = command["command_id"]
    repeated = harness.publish_vision(ball=pickup_ready_ball())

    assert not any(
        payload["action"] == "PICKUP_NOW" for payload in repeated
    )
    assert harness.active_special_action == "PICKUP_NOW"
    assert harness.pickups_completed == 0

    harness.send_status("PICKUP_NOW", command_id, "RUNNING")
    assert harness.mission_phase == "AUTO"
    assert harness.pickups_completed == 0
    assert harness.special_motion_running is True

    harness.send_status("PICKUP_NOW", command_id, "SUCCEEDED")
    assert harness.active_special_action is None
    assert harness.active_special_command_id is None
    assert harness.pickups_completed == 1
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"


def test_first_ball_line_alignment_is_closed_loop_before_forward_transition(
    monkeypatch,
):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0]
    )
    harness = MissionFlowHarness()
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    complete_active(harness, "PICKUP_NOW", pickup["command_id"])
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"

    missing = harness.publish_vision(line=None)
    assert missing[-1]["action"] == "WAIT"
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"

    correction = harness.publish_vision(line=line_info(heading=34.0))[-1]
    assert correction["action"] == "POST_BALL_LINE_TURN_RIGHT_3"
    assert correction["source_command"]["turn_count"] == 3
    release_general(harness, correction)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"

    now[0] = 13.0
    assert harness.publish_vision(line=line_info(heading=0.0)) == []
    aligned = harness.publish_vision(line=line_info(heading=0.0))[-1]
    assert aligned["action"] == "STRAIGHT"
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"


@pytest.mark.parametrize(
    "completed_before,direction,count", [(0, "RIGHT", 2), (1, "LEFT", 1)]
)
def test_post_pickup_search_finishes_turn_then_dwells_before_fresh_line_forward(
    monkeypatch, completed_before, direction, count
):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0]
    )
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.phase_manager.pickups_completed = completed_before
    pickup = publish_special(
        harness, "ball", pickup_ready_ball(), "PICKUP_NOW"
    )
    # PICKUP_NOW success follows the fixed turn and the bridge's 3s dwell.
    complete_active(harness, "PICKUP_NOW", pickup["command_id"])
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
    search = harness.publish_vision(line={"detected": False})[-1]
    assert search["action"] == f"POST_BALL_LINE_TURN_{direction}_{count}"

    harness.send_status(search["action"], search["command_id"], "RUNNING")
    assert harness.publish_vision(line=line_info(heading=0.0)) == []
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
    now[0] = 11.0
    harness.send_status(search["action"], search["command_id"], "SUCCEEDED")
    assert harness.post_ball_line_dwell_until == pytest.approx(14.0)
    now[0] = 13.999
    assert harness.publish_vision(line=line_info(heading=0.0)) == []
    now[0] = 14.0
    assert harness.publish_vision(line=line_info(heading=0.0)) == []
    assert harness.observations["line"] is None
    assert harness.publish_vision(line=None)[-1]["action"] == "WAIT"
    transition = harness.publish_vision(line=line_info(heading=0.0))[-1]
    assert transition["action"] == "STRAIGHT"
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"


def test_post_pickup_search_repeats_only_after_dwell_and_new_missing_line(
    monkeypatch,
):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0]
    )
    harness = MissionFlowHarness(phase="POST_BALL_LINE_ALIGN")
    first = harness.publish_vision(line={"detected": False})[-1]
    release_general(harness, first)
    assert harness.publish_vision(line={"detected": False}) == []
    now[0] = 13.0
    assert harness.publish_vision(line={"detected": False}) == []
    second = harness.publish_vision(line={"detected": False})[-1]
    assert second["action"] == first["action"]
    assert second["command_id"] != first["command_id"]
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"


def test_post_ball_heading_alignment_resumes_existing_sequence_after_dwell(
    monkeypatch,
):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0]
    )
    harness = MissionFlowHarness(phase="POST_BALL_LINE_ALIGN")
    off_center = {
        **line_info(heading=24.591, offset=-0.928512),
        "image_width": 1280, "image_height": 720, "robot_center_x_px": 710,
        "center_points_px": [[80, 680], [120, 590], [155, 510]],
    }
    correction = harness.publish_vision(line=off_center)[-1]
    assert correction["action"] == "POST_BALL_LINE_TURN_RIGHT_2"
    release_general(harness, correction)
    assert harness.publish_vision(line=off_center) == []
    now[0] = 13.0
    aligned = {
        **off_center,
        "filtered_heading_error_deg": 13.656,
        "filtered_lateral_offset_norm": -0.756465,
    }
    assert harness.publish_vision(line=aligned) == []
    harness.publish_vision(line=aligned)
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"
    assert harness._mission_progress()["ball_mode_active"] is False


def test_post_ball_line_turn_timeout_does_not_enter_line_lost_recovery():
    harness = MissionFlowHarness(phase="POST_BALL_LINE_ALIGN")
    correction = harness.publish_vision(line=line_info(heading=34.0))[-1]

    harness.send_status(
        correction["action"],
        correction["command_id"],
        "RUNNING",
    )
    harness.send_status(
        correction["action"],
        correction["command_id"],
        "TIMEOUT",
    )

    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
    assert harness.line_timeout_recovery_active is False


def test_post_pickup_lost_line_search_follows_last_executed_correction(
    monkeypatch,
):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0]
    )
    harness = MissionFlowHarness(phase="POST_BALL_LINE_ALIGN")
    assert harness.planner.post_ball_line_search_direction == "RIGHT"
    correction = harness.publish_vision(line=line_info(heading=-45.0))[-1]
    assert correction["action"] == "POST_BALL_LINE_TURN_LEFT_2"
    release_general(harness, correction)
    now[0] = 13.0
    assert harness.publish_vision(line={"detected": False}) == []
    search = harness.publish_vision(line={"detected": False})[-1]
    assert search["action"] == "POST_BALL_LINE_TURN_LEFT_1"


@pytest.mark.parametrize("initial_bearing,initial_action,remaining_bearing,next_action", [
    (34.0, "GOAL_CAMERA90_TURN_RIGHT_3", 0.0, "GOAL_CAMERA90_FINE_FORWARD_2"),
    (-35.0, "GOAL_CAMERA90_TURN_LEFT_1", -20.0, "GOAL_CAMERA90_TURN_LEFT_1"),
    (-20.0, "GOAL_CAMERA90_TURN_LEFT_1", -15.0, "GOAL_CAMERA90_FINE_FORWARD_2"),
])
def test_goal_camera90_turn_dwells_then_rechecks_fresh_goal(
    monkeypatch, initial_bearing, initial_action, remaining_bearing, next_action,
):
    harness = MissionFlowHarness(phase="GOAL_APPROACH")

    correction = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.9,
            "ground_distance_m": 0.9,
            "distance_m": 0.9,
            "bearing_deg": initial_bearing,
            "offset_x_norm": 0.3 if initial_bearing > 0 else -0.3,
        }
    )[-1]
    assert correction["action"] == initial_action
    release_general(harness, correction)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count

    deadline = harness.goal_post_motion_dwell_until
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: deadline)
    MotionDecisionNode._publish_decision(harness)
    forward = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.9,
            "ground_distance_m": 0.9,
            "distance_m": 0.9,
            "bearing_deg": remaining_bearing,
            "offset_x_norm": 0.0,
        }
    )[-1]
    assert forward["action"] == next_action
    assert forward["reason"] == (
        "align_goal_yaw_camera90" if "TURN" in next_action
        else "approach_goal_by_depth"
    )


def test_goal_camera90_crab_rechecks_full_priority_with_fresh_goal(monkeypatch):
    harness = MissionFlowHarness(phase="GOAL_APPROACH")

    crab = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.820,
            "ground_distance_m": 0.820,
            "distance_m": 0.820,
            "bearing_deg": 0.0,
            "offset_x_norm": 0.2,
            "offset_x_px": 128,
        }
    )[-1]
    assert crab["action"] == "GOAL_CAMERA90_CRAB_RIGHT"
    release_general(harness, crab)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count

    deadline = harness.goal_post_motion_dwell_until
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: deadline)
    MotionDecisionNode._publish_decision(harness)
    yaw = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.9,
            "ground_distance_m": 0.9,
            "distance_m": 0.9,
            "bearing_deg": -30.0,
            "offset_x_norm": 0.2,
        }
    )[-1]
    assert yaw["action"] == "GOAL_CAMERA90_TURN_LEFT_1"


def test_goal_approach_defers_crab_until_scoring_depth_after_dwell(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0],
    )
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    goal = {
        **approaching_goal(), "depth_m": 0.825,
        "bearing_deg": 14.0, "offset_x_norm": 0.2, "offset_x_px": 128,
    }
    forward = harness.publish_vision(goal=goal)[-1]
    assert forward["action"] == "GOAL_CAMERA90_FINE_FORWARD_1"
    release_general(harness, forward)
    assert harness.goal_post_motion_dwell_until == 13.0
    goal = {**goal, "depth_m": 0.820}
    now[0] = 12.99
    assert harness.publish_vision(goal=goal) == []
    now[0] = 13.0
    assert harness.publish_vision(goal=goal) == []
    crab = harness.publish_vision(goal=goal)[-1]
    assert crab["action"] == "GOAL_CAMERA90_CRAB_RIGHT"


def test_goal_too_close_blocks_old_depth_buckets():
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    far = {
        **approaching_goal(),
        "depth_m": 0.65,
        "ground_distance_m": 0.65,
        "distance_m": 0.65,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
    }

    first = harness.publish_vision(goal=far)[-1]
    assert first["action"] == "WAIT"
    assert first["valid"] is False

    closer = dict(far)
    closer["depth_m"] = 0.50
    closer["ground_distance_m"] = 0.50
    closer["distance_m"] = 0.50
    second = harness.publish_vision(goal=closer)[-1]
    assert second["action"] == "WAIT"
    assert second["valid"] is False


def test_stale_wrong_and_duplicate_pickup_statuses_are_ignored():
    harness = MissionFlowHarness()
    command = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    command_id = command["command_id"]

    harness.send_status("PICKUP_NOW", command_id + 1, "SUCCEEDED")
    harness.send_status("SHOT", command_id, "SUCCEEDED")
    assert harness.pickups_completed == 0
    assert harness.active_special_command_id == command_id

    complete_active(harness, "PICKUP_NOW", command_id)
    harness.send_status("PICKUP_NOW", command_id, "SUCCEEDED")
    assert harness.pickups_completed == 1
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"


def test_shot_success_updates_one_section_and_returns_to_auto(monkeypatch):
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=2,
    )
    mark_next_ball_grabbed(harness)
    command = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", command["command_id"])
    complete_post_shot_exit(harness, monkeypatch)

    assert harness.shots_completed == 1
    assert harness.ball_sections_processed == 1
    assert harness.finish_enabled is False
    assert harness.mission_phase == "AUTO"


def test_last_shot_enables_finish_flag_and_continues_line_driving(monkeypatch):
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=2,
    )
    harness.phase_manager.ball_sections_processed = 1
    mark_next_ball_grabbed(harness)
    command = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", command["command_id"])
    complete_post_shot_exit(harness, monkeypatch)

    assert harness.ball_sections_processed == 2
    assert harness.finish_enabled is True
    assert harness.mission_phase == "LINE_TRACK"

    # Model the goal becoming stale after the existing recovery window.
    harness.planner._clear_goal_tracking()
    line_commands = harness.publish_vision(
        line=line_info(),
        goal=None,
    )
    assert any(
        command["action"] == "STRAIGHT" for command in line_commands
    )
    assert all(command["phase"] == "LINE_TRACK" for command in line_commands)
    assert all(
        command["action"] != "CROSS_FINISH"
        for command in line_commands
    )


def test_final_line_track_ignores_new_ball_after_all_sections_complete():
    harness = MissionFlowHarness()

    harness.phase_manager.ball_sections_processed = (
        harness.phase_manager.required_ball_sections
    )
    harness.phase_manager.finish_enabled = True
    harness.phase_manager.set_phase("LINE_TRACK")

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=None,
        hurdle=None,
    )

    decision = published[-1]

    assert decision["phase"] == "LINE_TRACK"
    assert decision["source"] == "line"
    assert decision["action"] == "STRAIGHT"


def test_final_line_track_still_prioritizes_confirmed_hurdle():
    harness = MissionFlowHarness()

    harness.phase_manager.ball_sections_processed = (
        harness.phase_manager.required_ball_sections
    )
    harness.phase_manager.finish_enabled = True
    harness.phase_manager.set_phase("LINE_TRACK")

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=None,
        hurdle=approaching_hurdle(),
    )

    decision = published[-1]

    assert decision["phase"] == "LINE_TRACK"
    assert decision["source"] == "hurdle"
    assert decision["action"] == "STRAIGHT_2"



def test_final_line_track_still_holds_for_unconfirmed_hurdle():
    harness = MissionFlowHarness()

    harness.phase_manager.ball_sections_processed = (
        harness.phase_manager.required_ball_sections
    )
    harness.phase_manager.finish_enabled = True
    harness.phase_manager.set_phase("LINE_TRACK")

    hurdle = approaching_hurdle()
    hurdle["confirmation_confirmed"] = False

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=None,
        hurdle=hurdle,
    )

    decision = published[-1]

    assert decision["source"] == "line"
    assert decision["action"] == "STRAIGHT"



def test_go_round_trip_preserves_action_and_origin_phase():
    harness = MissionFlowHarness(phase="HURDLE_APPROACH")
    before = harness.phase_manager.snapshot()
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    command_id = command["command_id"]

    request_dict = build_executor_request(
        request_id=7,
        motion_id="forward",
        command_id=command_id,
        action="GO",
    )
    request = parse_motion_request(json.dumps(request_dict))
    publication = ExecutionPublicationState()
    publication.begin(request)
    running = convert_executor_status(publication.running_payload())
    terminal = publication.terminal_payload(
        MotionExecutionResult(
            motion_id="forward",
            final_status=ExecutorState.SUCCEEDED,
            success=True,
            error_code="NONE",
            message="done",
        )
    )
    succeeded = convert_executor_status(terminal)

    assert running["action"] == "GO"
    assert succeeded["action"] == "GO"
    assert running["command_id"] == command_id
    assert succeeded["command_id"] == command_id

    harness.send_status(
        running["action"], running["command_id"], running["status"]
    )
    assert harness.special_motion_running is True
    assert harness.phase_manager.current_phase == "HURDLE_APPROACH"
    harness.send_status(
        succeeded["action"],
        succeeded["command_id"],
        succeeded["status"],
    )

    assert harness.mission_phase == "HURDLE_APPROACH"
    assert harness.active_special_action is None
    assert harness.pickups_completed == before["pickups_completed"]
    assert harness.shots_completed == before["shots_completed"]
    assert (
        harness.ball_sections_processed
        == before["ball_sections_processed"]
    )


def test_manual_cross_finish_status_compatibility_still_enters_finished():
    harness = MissionFlowHarness(
        phase="WALK_TO_FINISH",
        required_ball_sections=0,
    )
    assert harness.phase_manager.start_special_action("CROSS_FINISH", 10)
    complete_active(harness, "CROSS_FINISH", 10)

    assert harness.mission_complete is True
    assert harness.mission_phase == "FINISHED"
    after_finish = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=score_ready_goal(),
        hurdle=None,
        finish=None,
    )
    assert after_finish
    assert all(
        payload["action"] not in {"STRAIGHT", "LEFT", "RIGHT"}
        for payload in after_finish
    )
    assert after_finish[-1]["action"] == "STOP"


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT"])
@pytest.mark.parametrize(
    (
        "action",
        "initial_phase",
        "expected_phase",
        "expected_sections",
    ),
    [
        ("PICKUP_NOW", "BALL_APPROACH", "BALL_APPROACH", 0),
        ("SHOT", "GOAL_APPROACH", "GOAL_APPROACH", 0),
        ("GO", "HURDLE_APPROACH", "HURDLE_APPROACH", 0),
        (
            "CROSS_FINISH",
            "WALK_TO_FINISH",
            "WALK_TO_FINISH",
            0,
        ),
    ],
)
def test_special_failure_and_timeout_follow_manager_policy(
    status,
    action,
    initial_phase,
    expected_phase,
    expected_sections,
):
    harness = MissionFlowHarness(
        phase=initial_phase,
        required_ball_sections=2,
    )
    assert harness.phase_manager.start_special_action(action, 10)
    complete_active(harness, action, 10, status)

    assert harness.mission_phase == expected_phase
    assert harness.ball_sections_processed == expected_sections
    assert harness.active_special_command_id is None


def test_general_gate_blocks_overlap_and_releases_after_terminal():
    harness = MissionFlowHarness()
    first = harness.publish_vision(line=line_info())
    assert len(first) == 1
    assert first[0]["action"] == "STRAIGHT"
    command_id = first[0]["command_id"]

    repeated = harness.publish_vision(line=line_info())
    assert repeated == []
    assert harness.general_motion_gate.locked is True

    harness.send_status("STRAIGHT", command_id, "RUNNING")
    harness.send_status("STRAIGHT", command_id, "SUCCEEDED")
    next_command = harness.publish_vision(line=line_info())
    assert len(next_command) == 1
    assert next_command[0]["action"] == "STRAIGHT"
    assert next_command[0]["command_id"] > command_id
    assert all(
        payload.get("error_code") != "REJECTED_BUSY"
        for payload in harness.publisher.messages
    )


def test_ball_stationary_turn_requires_fresh_ball_before_next_approach():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 0.0
    off_center = approaching_ball()
    off_center.update(
        {
            "depth_m": 0.70,
            "ground_distance_m": 0.70,
            "distance_m": 0.70,
            "bearing_deg": 20.0,
        }
    )

    approach = harness.publish_vision(ball=off_center)
    assert len(approach) == 1
    assert approach[0]["action"] == "STRAIGHT_2"
    command_id = approach[0]["command_id"]

    harness.send_status(
        approach[0]["action"],
        command_id,
        "RUNNING",
    )
    harness.send_status(
        approach[0]["action"],
        command_id,
        "SUCCEEDED",
    )

    before = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == before

    turn = harness.publish_vision(ball=off_center)
    assert len(turn) == 1
    assert turn[0]["action"] == "BALL_APPROACH_TURN_RIGHT_2"
    release_general(harness, turn[0])

    before = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == before

    centered = dict(off_center)
    centered["depth_m"] = 0.60
    centered["ground_distance_m"] = 0.60
    centered["distance_m"] = 0.60
    centered["bearing_deg"] = 0.0
    next_command = harness.publish_vision(ball=centered)
    assert len(next_command) == 1
    assert next_command[0]["action"] == "STRAIGHT_2"


def test_second_ball_uses_same_straight_distance_policy():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.phase_manager.pickups_completed = 1
    ball = approaching_ball()
    ball.update(
        {
            "depth_m": 0.70,
            "ground_distance_m": 0.70,
            "distance_m": 0.70,
            "bearing_deg": -20.0,
        }
    )

    command = harness.publish_vision(ball=ball)

    assert len(command) == 1
    assert command[0]["action"] == "STRAIGHT_2"


def test_failed_first_pickup_keeps_straight_distance_policy():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    harness.send_status("PICKUP_NOW", pickup["command_id"], "RUNNING")
    harness.send_status("PICKUP_NOW", pickup["command_id"], "FAILED")
    assert harness.pickups_completed == 0

    ball = approaching_ball()
    ball.update(
        {
            "depth_m": 0.70,
            "ground_distance_m": 0.70,
            "distance_m": 0.70,
            "bearing_deg": 20.0,
        }
    )
    command = harness.publish_vision(ball=ball)

    assert len(command) == 1
    assert command[0]["action"] == "STRAIGHT_2"


def test_active_special_uses_temporary_lock_without_changing_manager_phase():
    harness = MissionFlowHarness(phase="HURDLE_APPROACH")
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    locked = harness.publish_vision(hurdle=go_ready_hurdle())

    assert harness.phase_manager.current_phase == "HURDLE_APPROACH"
    assert locked[-1]["phase"] == "HURDLE_APPROACH_LOCK"
    assert locked[-1]["action"] == "WAIT"

    complete_active(harness, "GO", command["command_id"])
    assert harness.phase_manager.current_phase == "HURDLE_APPROACH"


def test_external_phase_override_accepts_only_valid_idle_phase():
    harness = MissionFlowHarness(phase="AUTO")
    harness.send_phase('{"phase": "goal_approach"}')
    assert harness.phase_manager.current_phase == "GOAL_APPROACH"

    harness.send_phase("")
    harness.send_phase("UNKNOWN")
    assert harness.phase_manager.current_phase == "GOAL_APPROACH"


@pytest.mark.parametrize("completed_before", [0, 1])
@pytest.mark.parametrize("grasp", ["GRABBED", "NOT_GRABBED", "UNKNOWN"])
def test_post_pickup_line_run_then_camera_only_with_verified_ball(monkeypatch, completed_before, grasp):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.phase_manager.pickups_completed = completed_before
    harness.phase_manager.ball_sections_processed = completed_before
    harness.phase_manager.shots_completed = completed_before
    pickup = publish_special(harness, "ball", pickup_ready_ball(), "PICKUP_NOW")
    assert harness.phase_manager.record_active_pickup_grasp_result(grasp)
    complete_active(harness, "PICKUP_NOW", pickup["command_id"])
    correction = harness.publish_vision(line=line_info(heading=34.0))[-1]
    assert correction["action"] == "POST_BALL_LINE_TURN_RIGHT_3"
    assert correction["mission_progress"]["ball_mode_active"] is True
    release_general(harness, correction)
    now[0] = 12.999
    assert harness.publish_vision(line=line_info()) == []
    now[0] = 13.0
    assert harness.publish_vision(line=line_info()) == []
    first = harness.publish_vision(line=line_info())[-1]
    assert first["action"] == "STRAIGHT"
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"
    assert first["mission_progress"]["ball_mode_active"] is False
    assert harness.post_ball_line_run_until is None
    harness.send_status(first["action"], first["command_id"], "RUNNING")
    assert harness.post_ball_line_run_until == 23.0
    now[0] = 16.0
    harness.send_status(first["action"], first["command_id"], "SUCCEEDED")
    second = harness.publish_vision(line=line_info(), goal=approaching_goal(), ball=approaching_ball())[-1]
    assert second["source"] == "line"
    harness.send_status(second["action"], second["command_id"], "RUNNING")
    assert harness.post_ball_line_run_until == 23.0
    now[0] = 23.0
    assert harness.publish_vision(line=line_info()) == []
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"
    now[0] = 23.1
    harness.send_status(second["action"], second["command_id"], "SUCCEEDED")
    branch = harness.publish_vision(line=line_info(), goal=approaching_goal())[-1]
    assert branch["mission_progress"]["ball_mode_active"] is False
    assert harness.phase_manager.shots_completed == completed_before
    if grasp == "GRABBED":
        assert branch["action"] == "POST_BALL_GOAL_TRANSITION"
        assert harness.phase_manager.ball_sections_processed == completed_before
        assert harness.publish_vision(goal=approaching_goal())[-1]["action"] == "WAIT"
        complete_active(harness, branch["action"], branch["command_id"])
        assert harness.mission_phase == "GOAL_APPROACH"
        assert getattr(harness, "goal_post_motion_dwell_until", None) is None
        assert harness.latest_info["goal"] is None
        assert harness.publish_vision()[-1]["action"] == "WAIT"
        assert harness.publish_vision(goal=approaching_goal())[-1]["source"] == "goal"
    else:
        assert harness.mission_phase == ("AUTO" if completed_before == 0 else "LINE_TRACK")
        assert branch["action"] == "STRAIGHT"
        assert harness.phase_manager.ball_sections_processed == completed_before + 1
        assert getattr(harness, "goal_post_motion_dwell_until", None) is None
        release_general(harness, branch)
        following = harness.publish_vision(line=line_info(), goal=score_ready_goal())[-1]
        assert following["source"] == "line"
        assert not any(c["action"] == "POST_BALL_GOAL_TRANSITION" for c in harness.publisher.messages)
        if completed_before == 0:
            release_general(harness, following)
            next_ball = harness.publish_vision(ball=approaching_ball())[-1]
            assert next_ball["source"] == "ball"


def test_top_loss_forward_waits_for_motion_dwell_and_new_vision(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic", lambda: now[0]
    )
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    harness.planner._update_ball_tracking({
        **approaching_ball(), "bbox": [610, 0, 670, 60],
        "image_height": 720, "camera_center_offset_x_px": 80,
    }, 0.0)
    harness.ball_approach_alignment_pending = True
    lost = {"detected": False, "raw_detected": False}
    forward = harness.publish_vision(ball=lost)[-1]
    assert forward["action"] == "BALL_LOST_FORWARD_2"
    assert harness.planner.ball_top_loss_forward_sent is True
    harness.send_status(forward["action"], forward["command_id"], "RUNNING")
    assert harness.publish_vision(ball=lost) == []
    harness.send_status(forward["action"], forward["command_id"], "SUCCEEDED")
    now[0] = 12.999
    assert harness.publish_vision(ball=lost) == []
    now[0] = 13.0
    assert harness.publish_vision(ball=lost) == []
    assert harness.publish_vision(ball=None)[-1]["action"] == "WAIT"
    search = harness.publish_vision(ball=lost)[-1]
    assert search["action"] == "BALL_APPROACH_TURN_RIGHT_5"


@pytest.mark.parametrize("stage", ["INITIAL", "FINE"])
def test_pickup_top_loss_publication_consumes_forward_inside_special_lock(stage):
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(harness, "ball", pickup_ready_ball(), "PICKUP_NOW")
    marker = (MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER if stage == "INITIAL"
              else MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER)
    harness.send_status("PICKUP_NOW", pickup["command_id"], "RUNNING", motion_id=marker)
    harness.planner._update_ball_tracking({
        **pickup_ready_ball(), "bbox": [610, 0, 670, 60],
        "image_height": 720,
    }, 0.0)
    forward = harness.publish_vision(ball={"detected": False})[-1]
    assert forward["action"] == f"BALL_PICKUP_{stage}_SEARCH_FORWARD"
    assert forward["active_special_command_id"] == pickup["command_id"]
    assert harness.planner.ball_top_loss_forward_sent is True
    assert not harness.pickup_fixed_sequence_started
    assert not harness.pickup_initial_align_waiting
    assert not harness.pickup_fine_align_waiting


def test_goal_depth_approach_remeasures_after_each_motion_before_shot(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    harness = MissionFlowHarness(phase='GOAL_APPROACH')
    mark_next_ball_grabbed(harness)
    for depth, action in [
        (1.5, 'GOAL_CAMERA_90_FORWARD'),
        (1.28, 'GOAL_CAMERA_90_FORWARD_2'),
        (1.025, 'GOAL_CAMERA90_FINE_FORWARD_4'),
        (0.985, 'GOAL_CAMERA90_FINE_FORWARD_3'),
        (0.950, 'GOAL_CAMERA90_FINE_FORWARD_2'),
        (0.875, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    ]:
        sample = {**approaching_goal(), 'depth_m': depth, 'distance_m': 0.2}
        command = harness.publish_vision(goal=sample)[-1]
        assert command['action'] == action
        published_count = len(harness.publisher.messages)
        harness.publish_vision(goal=sample)
        assert len(harness.publisher.messages) == published_count
        release_general(harness, command)
        now[0] += 2.9
        harness.publish_vision(goal=sample)
        assert len(harness.publisher.messages) == published_count
        now[0] = harness.goal_post_motion_dwell_until
        MotionDecisionNode._publish_decision(harness)
        assert harness.latest_info['goal'] is None
        assert len(harness.publisher.messages) == published_count
    command = harness.publish_vision(goal=score_ready_goal())[-1]
    assert command['action'] == 'SHOT'


@pytest.mark.parametrize('lost_confirmation', [
    None, {'detected': False}, {**score_ready_goal(), 'score_now': False},
])
def test_shot_settle_restarts_after_scoring_condition_is_lost(monkeypatch, lost_confirmation):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    harness = MissionFlowHarness(phase='GOAL_APPROACH')
    harness.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    mark_next_ball_grabbed(harness)
    assert harness.publish_vision(goal=score_ready_goal()) == []
    now[0] = 12.9
    interrupted = harness.publish_vision(goal=lost_confirmation)
    assert all(command['action'] != 'SHOT' for command in interrupted)
    assert harness.pre_motion_settle_started_at is None
    now[0] = 13.0
    assert harness.publish_vision(goal=score_ready_goal()) == []
    now[0] = 15.999
    assert harness.publish_vision(goal=score_ready_goal()) == []
    now[0] = 16.0
    shot = harness.publish_vision(goal=score_ready_goal())[-1]
    assert shot['action'] == 'SHOT'
    assert shot['sdk_motion_requested'] is True
    assert harness.active_special_command_id == shot['command_id']


@pytest.mark.parametrize('offset,action', [
    (-0.8, 'LINE_LOST_TURN_LEFT'), (0.8, 'LINE_LOST_TURN_RIGHT'),
])
def test_line_search_rechecks_new_line_after_turn_success(offset, action):
    harness = MissionFlowHarness(phase='LINE_TRACK')
    straight = harness.publish_vision(line=line_info(offset=offset))[-1]
    assert straight['action'] == 'STRAIGHT'
    release_general(harness, straight)
    search = harness.publish_vision(line={'detected': False})[-1]
    assert search['action'] == action
    assert harness.publish_vision(line={'detected': False}) == []
    # A frame arriving during rotation cannot trigger the following action.
    assert harness.publish_vision(line=line_info()) == []
    release_general(harness, search)
    assert harness.latest_info['line'] is None
    before = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert all(not message['valid'] for message in harness.publisher.messages[before:])
    resumed = harness.publish_vision(line=line_info())[-1]
    assert resumed['action'] == 'STRAIGHT'


def test_line_search_repeats_only_after_new_negative_frame():
    harness = MissionFlowHarness(phase='AUTO')
    harness.planner.observe_line_for_search(line_info(offset=-0.8))
    search = harness.publish_vision(line={'detected': False})[-1]
    release_general(harness, search)
    assert harness.latest_info['line'] is None
    search_again = harness.publish_vision(line={'detected': False})[-1]
    assert search_again['action'] == 'LINE_LOST_TURN_LEFT'
    assert search_again['command_id'] != search['command_id']


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED"])
def test_post_pickup_line_failure_blocks_camera_and_further_walking(monkeypatch, status):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    harness = MissionFlowHarness(phase="LINE_TRACK_AFTER_PICKUP")
    harness.phase_manager.pickups_completed = 1
    harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    line = harness.publish_vision(line=line_info())[-1]
    harness.send_status(line["action"], line["command_id"], "RUNNING")
    harness.send_status(line["action"], line["command_id"], status)
    now[0] = 30.0
    assert harness.publish_vision(line=line_info(), goal=approaching_goal()) == []
    assert harness.post_ball_line_run_failed is True
    assert harness.mission_phase == "LINE_TRACK_AFTER_PICKUP"
    assert harness.active_special_command_id is None


def test_post_pickup_line_timer_waits_for_actual_motion_and_disables_prequeue(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    harness = MissionFlowHarness(phase="LINE_TRACK_AFTER_PICKUP")
    harness.phase_manager.pickups_completed = 1
    harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    assert harness.publish_vision(line=None)[-1]["action"] in {"WAIT", "STOP"}
    now[0] = 30.0
    line = harness.publish_vision(line=line_info())[-1]
    assert getattr(harness, "post_ball_line_run_until", None) is None
    now[0] = 31.0
    harness.send_status(line["action"], line["command_id"], "RUNNING")
    assert harness.post_ball_line_run_until == 41.0
    before = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness, harness.last_selected_decision, queue_while_locked=True)
    assert len(harness.publisher.messages) == before
    assert getattr(harness, "queued_general_command_id", None) is None


def test_post_pickup_run_uses_the_normal_line_steering_and_loss_logic():
    normal = MissionFlowHarness(phase="LINE_TRACK")
    timed = MissionFlowHarness(phase="LINE_TRACK_AFTER_PICKUP")
    for harness in (normal, timed):
        harness.phase_manager.pickups_completed = 1
        harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    frames = [line_info()] + [line_info(heading=34.0, offset=0.4)] * 4
    frames += [{"detected": False}] * 3
    for frame in frames:
        observations = {"line": frame, "ball": None, "goal": None, "hurdle": None}
        expected = normal._select_mission_decision(observations, 0.1)
        actual = timed._select_mission_decision(observations, 0.1)
        assert (actual.source, actual.action, actual.reason, actual.valid) == (
            expected.source, expected.action, expected.reason, expected.valid,
        )


def test_post_pickup_corner_frames_cannot_queue_large_turn_after_forward(monkeypatch):
    from test_motion_command_bridge import FakeBridge, decoded_messages

    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    harness = MissionFlowHarness(phase="LINE_TRACK_AFTER_PICKUP")
    harness.phase_manager.pickups_completed = 1
    harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    first = harness.publish_vision(line=line_info())[-1]
    harness.send_status(first["action"], first["command_id"], "RUNNING")
    corner = dict(
        line_info(heading=8.9, offset=-0.256),
        turn_angle_deg=63.0,
        corner_preview_confirmed=True,
        corner_start_distance_m=0.74,
        corner_approach_motion="STRAIGHT_5",
    )
    now[0] = 12.5
    callback = MotionDecisionNode._info_callback(harness, "line")
    for _ in range(15):
        callback(String(data=json.dumps(corner)))
    assert harness.pending_line_decision is None
    assert len(harness.publisher.messages) == 1

    now[0] = 13.2
    harness.send_status(first["action"], first["command_id"], "SUCCEEDED")
    assert harness.planner.line_planner.previous_motion == "STRAIGHT"
    following = harness.publish_vision(line=corner)[-1]
    assert following["action"] == "STRAIGHT"
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(following)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == "line_forward_6"


def test_post_pickup_camera_pause_finishes_before_fresh_goal_approach(monkeypatch):
    from test_motion_command_bridge import FakeBridge, complete_active_motion, decoded_messages

    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    harness = MissionFlowHarness(phase="LINE_TRACK_AFTER_PICKUP")
    harness.phase_manager.pickups_completed = 1
    harness.phase_manager.ball_grasp_results[1] = "GRABBED"
    line = harness.publish_vision(line=line_info())[-1]
    release_general(harness, line)
    now[0] = 20.0
    camera = harness.publish_vision(line=line_info(), goal=approaching_goal())[-1]
    assert camera["action"] == "POST_BALL_GOAL_TRANSITION"
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(camera)))
    complete_active_motion(bridge, "RUNNING")
    MotionDecisionNode._motion_status_callback(harness, bridge.motion_status_publisher.messages[-1])
    now[0] = 20.8
    complete_active_motion(bridge)
    MotionDecisionNode._motion_status_callback(harness, bridge.motion_status_publisher.messages[-1])
    assert harness.mission_phase == "POST_BALL_GOAL_TRANSITION"
    assert harness._mission_progress()["ball_mode_active"] is False
    now[0] = 22.999
    bridge._check_atomic_dwell(now[0])
    assert harness.publish_vision(goal=approaching_goal())[-1]["action"] == "WAIT"
    now[0] = 23.0
    bridge._check_atomic_dwell(now[0])
    MotionDecisionNode._motion_status_callback(harness, bridge.motion_status_publisher.messages[-1])
    assert harness.mission_phase == "GOAL_APPROACH"
    assert harness.goal_post_motion_dwell_until is None
    assert harness.latest_info["goal"] is None
    assert harness.publish_vision()[-1]["action"] == "WAIT"
    assert harness.publish_vision(goal=approaching_goal())[-1]["source"] == "goal"
    assert [m["motion_id"] for m in decoded_messages(bridge.executor_request_publisher)] == ["post_ball_camera_90"]
