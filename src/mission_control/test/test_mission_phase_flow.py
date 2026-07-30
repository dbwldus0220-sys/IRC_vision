"""End-to-end mission phase flow tests without ROS graph execution."""

import json

import pytest
from std_msgs.msg import String

from mission_control.legacy_motion_executor_adapter import (
    build_executor_request,
)
from mission_control.legacy_motion_status_adapter import (
    convert_executor_status,
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


class CapturePublisher:
    """Collect JSON String messages in publication order."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


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

    SOURCES = MotionDecisionNode.SOURCES
    SPECIAL_ACTIONS = MotionDecisionNode.SPECIAL_ACTIONS
    SPECIAL_ACTION_SOURCES = MotionDecisionNode.SPECIAL_ACTION_SOURCES

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
    ):
        self.phase_manager = MissionPhaseManager(
            initial_phase=phase,
            required_pickups=2,
            required_shots=2,
            required_ball_sections=required_ball_sections,
        )
        self.planner = MotionDecisionPlanner()
        self.general_motion_gate = GeneralMotionCommandGate()
        self.publisher = CapturePublisher()
        self.logger = FakeLogger()
        self.observations = {
            source: None for source in self.SOURCES
        }
        self.command_id = 0
        self.event_id = 0
        self.terminal_latch = None
        self.terminal_action_armed = {
            source: True
            for source in self.SPECIAL_ACTION_SOURCES.values()
        }
        self.active_special_event_id = None
        self.active_special_dynamics_command = None
        self.finish_min_confidence = 0.70
        self.previous_publish_time = 0.0

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

    def _mission_progress(self):
        return MotionDecisionNode._mission_progress(self)

    def publish_vision(self, **observations):
        self.observations.update(observations)
        self.general_motion_gate.on_new_vision_input()
        before = len(self.publisher.messages)
        MotionDecisionNode._publish_decision(self)
        return self.publisher.messages[before:]

    def send_status(self, action, command_id, status):
        message = String()
        message.data = json.dumps(
            {
                "action": action,
                "command_id": command_id,
                "event_id": None,
                "dynamics_command": None,
                "status": status,
            }
        )
        MotionDecisionNode._motion_status_callback(self, message)

    def send_phase(self, phase):
        message = String()
        message.data = phase
        MotionDecisionNode._phase_callback(self, message)


def line_info(heading=0.0):
    return {
        "detected": True,
        "filtered_heading_error_deg": heading,
        "filtered_lateral_offset_norm": 0.0,
        "heading_quality": 0.95,
        "geometry_quality": 0.95,
        "detection_quality": 0.95,
        "turn_angle_deg": 0.0,
        "turn_consistency": 1.0,
    }


def pickup_ready_ball():
    return {
        "detected": True,
        "confidence": 0.95,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.80,
        "distance_m": 0.80,
        "depth_valid": True,
        "pickup_ready": True,
        "pickup_now": True,
    }


def score_ready_goal():
    return {
        "detected": True,
        "confidence": 0.95,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.25,
        "distance_m": 0.25,
        "depth_valid": True,
        "score_now": True,
    }


def go_ready_hurdle():
    return {
        "detected": True,
        "confirmation_confirmed": True,
        "confidence": 0.95,
        "depth_m": 0.70,
        "distance_m": 0.70,
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


def complete_active(harness, action, command_id, terminal="SUCCEEDED"):
    harness.send_status(action, command_id, "RUNNING")
    harness.send_status(action, command_id, terminal)


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
    assert harness.mission_phase == "GOAL_APPROACH"


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
    assert harness.mission_phase == "GOAL_APPROACH"


def test_shot_success_updates_one_section_and_returns_to_auto():
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=2,
    )
    command = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", command["command_id"])

    assert harness.shots_completed == 1
    assert harness.ball_sections_processed == 1
    assert harness.finish_enabled is False
    assert harness.mission_phase == "AUTO"


def test_last_shot_enables_finish_flag_and_continues_line_driving():
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=2,
    )
    harness.phase_manager.ball_sections_processed = 1
    command = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", command["command_id"])

    assert harness.ball_sections_processed == 2
    assert harness.finish_enabled is True
    assert harness.mission_phase == "AUTO"

    # Model the goal becoming stale after the existing recovery window.
    harness.planner._clear_goal_tracking()
    line_commands = harness.publish_vision(
        line=line_info(),
        goal=None,
    )
    assert any(
        command["action"] == "STRAIGHT" for command in line_commands
    )
    assert all(command["phase"] == "AUTO" for command in line_commands)
    assert all(
        command["action"] != "CROSS_FINISH"
        for command in line_commands
    )


def test_go_round_trip_preserves_action_and_returns_to_auto():
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

    assert harness.mission_phase == "AUTO"
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
        ("PICKUP_NOW", "BALL_APPROACH", "AUTO", 1),
        ("SHOT", "GOAL_APPROACH", "AUTO", 1),
        ("GO", "HURDLE_APPROACH", "AUTO", 0),
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
    assert harness.phase_manager.current_phase == "AUTO"


def test_external_phase_override_accepts_only_valid_idle_phase():
    harness = MissionFlowHarness(phase="AUTO")
    harness.send_phase('{"phase": "goal_approach"}')
    assert harness.phase_manager.current_phase == "GOAL_APPROACH"

    harness.send_phase("")
    harness.send_phase("UNKNOWN")
    assert harness.phase_manager.current_phase == "GOAL_APPROACH"
