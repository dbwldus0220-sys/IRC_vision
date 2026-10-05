"""Tests for special-motion state handling in the motion decision node."""

import json
from dataclasses import replace
import os
from pathlib import Path
import sys
import time

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecision, MotionDecisionPlanner
from mission_control.motion_command_gate import GeneralMotionCommandGate
from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.executor_heartbeat_watchdog import (
    ExecutorHeartbeatWatchdog,
)
from mission_control.safety_interlock import SafetyInterlock

import pytest

from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_mission_input_node import build_mock_vision_input  # noqa: E402


class FakeLogger:
    """Provide no-op logger methods used by the callback."""

    def __init__(self):
        """Collect log messages for state-change logging tests."""
        self.infos = []
        self.warnings = []

    def info(self, _message):
        """Record an informational log message."""
        self.infos.append(_message)

    def warning(self, _message):
        """Record a warning log message."""
        self.warnings.append(_message)


class FakePlanner:
    """Return deterministic line or lock decisions for node-level tests."""

    def __init__(self):
        planner = MotionDecisionPlanner()
        self.line_planner = planner.line_planner
        self.hurdle_planner = planner.hurdle_planner

    def _clear_goal_tracking(self):
        pass

    def _remember_pickup_close_ball(self, _info):
        """Leave proximity handling to tests using the real planner."""
        pass

    def clear_collected_ball_tracking(self):
        """Match the planner lifecycle hook used after a successful pickup."""
        pass

    def disable_completed_hurdle_missions(self):
        self.hurdle_mission_disabled = True

    def disable_completed_ball_missions(self):
        """Record no state in tests that use this minimal planner."""
        pass

    @staticmethod
    def source_for_phase(phase):
        """Match the production BALL phase classification."""
        normalized = phase.strip().upper()
        return "ball" if normalized.startswith(("BALL", "PICK")) else "line"

    def approach_phase_for_search(self, _phase, _observations):
        """Keep search phases unchanged in tests unrelated to acquisition."""
        return None

    def plan(self, phase, _observations, _dt_sec):
        """Return WAIT for locks and STRAIGHT for normal line planning."""
        locked = phase.endswith('_LOCK')
        return MotionDecision(
            phase=phase,
            source='none' if locked else 'line',
            action='WAIT' if locked else 'STRAIGHT',
            valid=not locked,
            reason=(
                'mission_locked_waiting_for_motion_status'
                if locked
                else 'line_ready'
            ),
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={},
        )

    def plan_ball_pickup_fine_alignment(self, _info):
        """Return a safe wait in tests unrelated to pickup fine alignment."""
        return MotionDecision(
            phase='BALL_PICKUP_FINE_ALIGN',
            source='ball',
            action='WAIT',
            valid=False,
            reason='ball_pickup_fine_alignment_waiting_for_ball',
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={},
        )

    def plan_ball_pickup_post_backward_alignment(self, _info):
        """Return a safe wait in tests unrelated to post-backward alignment."""
        return MotionDecision(
            phase='BALL_PICKUP_POST_BACKWARD_ALIGN',
            source='ball',
            action='WAIT',
            valid=False,
            reason='ball_pickup_post_backward_alignment_waiting_for_ball',
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={},
        )


class FakeDecisionNode:
    """Provide only the state required by the motion-status callback."""

    # Timing tests explicitly enable the production shot delay.
    SHOT_PRE_MOTION_SETTLE_SEC = 0.0
    LINE_TURN_PRE_MOTION_SETTLE_SEC = 0.0
    FINE_FORWARD_PRE_MOTION_SETTLE_SEC = 0.0

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
    PICKUP_DWELL_MARKER = MotionDecisionNode.PICKUP_DWELL_MARKER
    PICKUP_GRASP_CHECK_MOTION_ID = (
        MotionDecisionNode.PICKUP_GRASP_CHECK_MOTION_ID
    )
    GRASP_CONFIDENCE_THRESHOLD = (
        MotionDecisionNode.GRASP_CONFIDENCE_THRESHOLD
    )
    BALL_POST_MOTION_DWELL_SEC = 0.0
    BALL_RAW_CONFIRMATION_RELEASE_SEC = (
        MotionDecisionNode.BALL_RAW_CONFIRMATION_RELEASE_SEC
    )

    def __init__(
        self,
        mission_phase='AUTO',
        required_pickups=2,
        required_shots=2,
        required_ball_sections=2,
    ):
        """Initialize the minimal state used by node callback tests."""
        self.phase_manager = MissionPhaseManager(
            initial_phase=mission_phase,
            required_pickups=required_pickups,
            required_shots=required_shots,
            required_ball_sections=required_ball_sections,
        )
        self.finish_min_confidence = 0.70
        self.terminal_latch = ('test', 'terminal')
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
        self.grasp_latest_detection_stamp_ns = None
        self.grasp_verification_active = False
        self.grasp_verification_attempt = None
        self.grasp_verification_min_stamp_ns = None
        self.grasp_verification_result = MissionPhaseManager.GRASP_UNKNOWN
        self.grasp_verification_confidence = None
        self.latest_info = {
            source: None for source in MotionDecisionNode.SOURCES
        }
        self.latest_time = {
            source: None for source in MotionDecisionNode.SOURCES
        }
        self.last_published_vision_stamp = {}
        self.pre_motion_settle_sec = 0.0
        self.pre_motion_settle_source = None
        self.pre_motion_settle_action = None
        self.pre_motion_settle_started_at = None
        self.general_motion_gate = GeneralMotionCommandGate()
        self.safety_interlock = SafetyInterlock()
        self.executor_heartbeat_watchdog = ExecutorHeartbeatWatchdog(
            started_at=0.0,
            startup_grace_sec=5.0,
            timeout_sec=2.0,
        )
        self.decision_started = True
        self.start_terminal_fd = None
        self.executor_auto_ready = True
        self.executor_ready_requires_fresh_vision = False
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
        self.pending_line_decision = None
        self.pending_line_recover_decision = None
        self.line_recover_dwell_until = None
        self.line_timeout_recovery_active = False
        self.line_timeout_recovery_frames = []
        self.planner = FakePlanner()

        self.logger = FakeLogger()

    def get_logger(self):
        """Return the fake logger."""
        return self.logger

    mission_phase = MotionDecisionNode.mission_phase
    required_pickups = MotionDecisionNode.required_pickups
    required_shots = MotionDecisionNode.required_shots
    required_ball_sections = MotionDecisionNode.required_ball_sections
    pickups_completed = MotionDecisionNode.pickups_completed
    shots_completed = MotionDecisionNode.shots_completed
    ball_sections_processed = MotionDecisionNode.ball_sections_processed
    active_special_action = MotionDecisionNode.active_special_action
    active_special_command_id = MotionDecisionNode.active_special_command_id

    @property
    def finish_enabled(self):
        return self.phase_manager.finish_enabled

    @finish_enabled.setter
    def finish_enabled(self, value):
        self.phase_manager.finish_enabled = value

    @property
    def mission_complete(self):
        return self.phase_manager.mission_complete

    @mission_complete.setter
    def mission_complete(self, value):
        self.phase_manager.mission_complete = value

    @property
    def special_motion_running(self):
        return self.phase_manager.active_special_running

    @special_motion_running.setter
    def special_motion_running(self, value):
        self.phase_manager.active_special_running = value

    def _finish_crossing_ready(self, finish_info):
        """Delegate finish validation to the real node implementation."""
        return MotionDecisionNode._finish_crossing_ready(
            self,
            finish_info,
        )

    def _latch_critical_executor_fault(self, **kwargs):
        """Match the production node's safety-latch callback interface."""
        return MotionDecisionNode._latch_critical_executor_fault(self, **kwargs)

    def _check_executor_heartbeat(self, now=None):
        """Match the production node's executor watchdog interface."""
        return MotionDecisionNode._check_executor_heartbeat(self, now)

    def _suppress_unverified_shot(self, decision):
        """Apply the production BALL-grasp shot gate."""
        return MotionDecisionNode._suppress_unverified_shot(self, decision)


def status_message(
    *,
    status,
    action,
    command_id,
    event_id,
    dynamics_command,
    error_code=None,
    message=None,
    motion_id=None,
    completed_motion_id=None,
    verification_window_complete=None,
):
    """Create one /motion/status JSON message."""
    payload = {
        'status': status,
        'action': action,
        'command_id': command_id,
        'event_id': event_id,
        'dynamics_command': dynamics_command,
        'error_code': error_code,
        'message': message,
        'motion_id': motion_id,
        'completed_motion_id': completed_motion_id,
        'verification_window_complete': verification_window_complete,
    }

    message = String()
    message.data = json.dumps(payload)
    return message


def send_status(node, **kwargs):
    """Call the real motion-status callback using a fake node."""
    MotionDecisionNode._motion_status_callback(
        node,
        status_message(**kwargs),
    )


def send_phase(node, phase):
    message = String()
    message.data = phase
    MotionDecisionNode._phase_callback(node, message)


def grasp_detections_message(stamp_ns, detections):
    """Build the detector's stamped String JSON contract."""
    message = String()
    message.data = json.dumps(
        {
            "stamp": {
                "sec": stamp_ns // 1_000_000_000,
                "nanosec": stamp_ns % 1_000_000_000,
            },
            "detections": detections,
        }
    )
    return message


def send_grasp_detections(node, stamp_ns, detections):
    MotionDecisionNode._grasp_detections_callback(
        node,
        grasp_detections_message(stamp_ns, detections),
    )


def grasp_log_records(node, marker):
    """Return decoded structured records for one grasp log marker."""
    prefix = f"[{marker}] "
    return [
        json.loads(item[len(prefix):])
        for item in node.logger.infos
        if item.startswith(prefix)
    ]


def set_grasp_result_for_first_ball(node, result):
    assert node.phase_manager.start_special_action("PICKUP_NOW", 10)
    if result != MissionPhaseManager.GRASP_UNKNOWN:
        assert node.phase_manager.record_active_pickup_grasp_result(result)
    node.phase_manager.handle_motion_status("PICKUP_NOW", 10, "RUNNING")
    node.phase_manager.handle_motion_status("PICKUP_NOW", 10, "SUCCEEDED")


def test_grasp_window_ignores_pre_pose_detection_and_latches_fresh_grab():
    node = FakeDecisionNode("BALL_APPROACH")
    send_grasp_detections(
        node,
        10,
        [{"class_name": "grab", "confidence": 0.9}],
    )
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    send_status(
        node,
        status="RUNNING",
        action="PICKUP_NOW",
        command_id=10,
        event_id=1,
        dynamics_command=None,
        motion_id=node.PICKUP_DWELL_MARKER,
        completed_motion_id=node.PICKUP_GRASP_CHECK_MOTION_ID,
        verification_window_complete=False,
    )

    send_grasp_detections(
        node,
        10,
        [{"class_name": "grab", "confidence": 0.9}],
    )
    assert node.grasp_verification_result == "UNKNOWN"
    for stamp in range(11, 26):
        send_grasp_detections(
            node,
            stamp,
            [{"class_name": "grab", "confidence": 0.25}],
        )
    send_status(
        node,
        status="RUNNING",
        action="PICKUP_NOW",
        command_id=10,
        event_id=1,
        dynamics_command=None,
        motion_id=node.PICKUP_DWELL_MARKER,
        completed_motion_id=node.PICKUP_GRASP_CHECK_MOTION_ID,
        verification_window_complete=True,
    )

    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"
    assert node.grasp_verification_active is False


def test_grasp_window_latches_not_grabbed_without_successful_frames():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    send_grasp_detections(node, 20, [])
    MotionDecisionNode._finish_grasp_verification_window(node)

    assert node.phase_manager.grasp_result_for_ball(1) == "NOT_GRABBED"


def test_grasp_dwell_updates_do_not_restart_verification():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    pose = dict(
        status="RUNNING", action="PICKUP_NOW", command_id=10, event_id=1,
        dynamics_command=None, motion_id=node.PICKUP_DWELL_MARKER,
        completed_motion_id=node.PICKUP_GRASP_CHECK_MOTION_ID,
    )
    send_status(node, **pose, verification_window_complete=False)
    send_grasp_detections(node, 1, [{"class_name": "grab", "confidence": 0.9}])
    send_status(node, **pose, verification_window_complete=False)
    assert node.grasp_verify_accepted_frames == 1
    assert len(grasp_log_records(node, "GRASP_VERIFY_START")) == 1
    send_status(node, **pose, verification_window_complete=True)
    assert not node.grasp_verification_active
    send_grasp_detections(node, 2, [{"class_name": "grab", "confidence": 0.9}])
    assert node.grasp_verify_accepted_frames == 1


@pytest.mark.parametrize("command_id,event_id", [(11, 1), (10, 2)])
def test_unmatched_check_pose_status_cannot_start_verification(command_id, event_id):
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    send_status(
        node, status="RUNNING", action="PICKUP_NOW", command_id=command_id,
        event_id=event_id, dynamics_command=None,
        motion_id=node.PICKUP_DWELL_MARKER,
        completed_motion_id=node.PICKUP_GRASP_CHECK_MOTION_ID,
        verification_window_complete=False,
    )
    assert not getattr(node, "grasp_verification_active", False)


def test_grasp_dwell_rejects_delayed_frames_captured_before_pause(monkeypatch):
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda self: 100)
    MotionDecisionNode._start_grasp_verification_window(node)
    send_grasp_detections(node, 99, [{"class_name": "grab", "confidence": 0.9}])
    assert node.grasp_verify_accepted_frames == 0
    send_grasp_detections(node, 100, [{"class_name": "grab", "confidence": 0.9}])
    assert node.grasp_verify_accepted_frames == 1


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED"])
def test_pickup_failure_closes_grasp_dwell_verification(status):
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    pose = dict(
        action="PICKUP_NOW", command_id=10, event_id=1,
        dynamics_command=None, motion_id=node.PICKUP_DWELL_MARKER,
        completed_motion_id=node.PICKUP_GRASP_CHECK_MOTION_ID,
        verification_window_complete=False,
    )
    send_status(node, status="RUNNING", **pose)
    assert node.grasp_verification_active
    send_status(node, status=status, **pose)
    assert not node.grasp_verification_active


@pytest.mark.parametrize(
    ("success_count", "frame_count", "last_success", "expected"),
    [
        (15, 40, False, "GRABBED"),
        (14, 40, True, "NOT_GRABBED"),
        (15, 15, True, "GRABBED"),
        (14, 14, True, "NOT_GRABBED"),
    ],
)
def test_grasp_window_requires_fifteen_successful_frames(
    success_count, frame_count, last_success, expected
):
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    successes_before_last = success_count - int(last_success)
    for index in range(frame_count):
        successful = (
            last_success if index == frame_count - 1
            else index < successes_before_last
        )
        send_grasp_detections(
            node,
            index + 1,
            [{"class_name": "grab", "confidence": 0.25}]
            if successful else [],
        )

    MotionDecisionNode._finish_grasp_verification_window(node)

    assert node.phase_manager.grasp_result_for_ball(1) == expected
    # Later observations cannot change a completed verification result.
    send_grasp_detections(node, frame_count + 1, [])
    assert node.grasp_verification_result == expected
    assert node.phase_manager.grasp_result_for_ball(1) == expected


def test_grasp_window_counts_nonconsecutive_successes_for_shot_permission():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    for index in range(40):
        send_grasp_detections(
            node,
            index + 1,
            [{"class_name": "grab", "confidence": 0.8}]
            if index < 30 and index % 2 == 0 else [],
        )
    MotionDecisionNode._finish_grasp_verification_window(node)
    node.phase_manager.handle_motion_status("PICKUP_NOW", 10, "RUNNING")
    node.phase_manager.handle_motion_status("PICKUP_NOW", 10, "SUCCEEDED")
    shot = MotionDecision(
        phase="GOAL_APPROACH",
        source="goal",
        action="SHOT",
        valid=True,
        reason="goal_centered_at_scoring_depth",
        sdk_motion_requested=True,
        requires_ack=True,
        source_command={},
    )

    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"
    assert MotionDecisionNode._suppress_unverified_shot(node, shot) == shot


def test_grasp_window_uses_latest_forty_frames():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    for stamp in range(1, 41):
        send_grasp_detections(
            node,
            stamp,
            [{"class_name": "grab", "confidence": 0.8}]
            if stamp <= 15 else [],
        )
    assert node.grasp_verification_result == "GRABBED"

    send_grasp_detections(node, 41, [])
    MotionDecisionNode._finish_grasp_verification_window(node)

    assert node.phase_manager.grasp_result_for_ball(1) == "NOT_GRABBED"
    summary = grasp_log_records(node, "GRASP_VERIFY_END")[0]
    assert summary["accepted_frames"] == 41
    assert summary["grabbed_frames"] == 15
    assert summary["decision_window_frames"] == 40
    assert summary["decision_window_grabbed_frames"] == 14


def test_grasp_window_counts_one_vote_per_frame_and_keeps_confidence_threshold():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    send_grasp_detections(
        node, 1, [{"class_name": "grab", "confidence": 0.9}] * 15
    )
    for stamp in range(2, 41):
        send_grasp_detections(node, stamp, [
            {"class_name": "grab", "confidence": 0.24},
            {"class_name": "ball", "confidence": 0.99},
        ])
    MotionDecisionNode._finish_grasp_verification_window(node)

    assert node.phase_manager.grasp_result_for_ball(1) == "NOT_GRABBED"
    summary = grasp_log_records(node, "GRASP_VERIFY_END")[0]
    assert summary["decision_window_frames"] == 40
    assert summary["decision_window_grabbed_frames"] == 1


def test_grasp_window_clears_votes_for_the_next_ball():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    for stamp in range(1, 16):
        send_grasp_detections(
            node, stamp, [{"class_name": "grab", "confidence": 0.8}]
        )
    MotionDecisionNode._finish_grasp_verification_window(node)
    node.phase_manager.handle_motion_status("PICKUP_NOW", 10, "RUNNING")
    node.phase_manager.handle_motion_status("PICKUP_NOW", 10, "SUCCEEDED")
    assert node.phase_manager.start_special_action("SHOT", 11)
    node.phase_manager.handle_motion_status("SHOT", 11, "RUNNING")
    node.phase_manager.handle_motion_status("SHOT", 11, "SUCCEEDED")
    assert node.phase_manager.start_special_action("PICKUP_NOW", 12)
    MotionDecisionNode._start_grasp_verification_window(node)
    assert node.grasp_verification_result == "UNKNOWN"
    send_grasp_detections(node, 16, [])
    MotionDecisionNode._finish_grasp_verification_window(node)

    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"
    assert node.phase_manager.grasp_result_for_ball(2) == "NOT_GRABBED"
    summary = grasp_log_records(node, "GRASP_VERIFY_END")[-1]
    assert summary["ball_index"] == 2
    assert summary["decision_window_frames"] == 1
    assert summary["decision_window_grabbed_frames"] == 0


def test_grasp_window_without_fresh_frame_remains_unknown():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    MotionDecisionNode._finish_grasp_verification_window(node)

    assert node.phase_manager.grasp_result_for_ball(1) == "UNKNOWN"


def test_grasp_verification_start_log_is_emitted_once():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)

    MotionDecisionNode._start_grasp_verification_window(node)

    records = grasp_log_records(node, "GRASP_VERIFY_START")
    assert len(records) == 1
    assert records[0]["attempt_id"] == 1
    assert records[0]["ball_index"] == 1
    assert records[0]["completed_motion"] == "pickup_grasp_check_pose"
    assert records[0]["confidence_threshold"] == 0.25
    assert records[0]["frame_window_limit"] == 40
    assert records[0]["required_grab_frames"] == 15
    assert records[0]["window_scope"] == "post_check_pose_dwell"
    assert records[0]["expected_window_sec"] == 3.0
    assert records[0]["initial_result"] == "UNKNOWN"


def test_grasp_verification_frame_logs_grab_and_miss_observations():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)

    send_grasp_detections(
        node,
        20,
        [
            {
                "class_name": "grab",
                "confidence": 0.63,
                "bbox": [1, 2, 3, 4],
            },
            {"class_name": "ball", "confidence": 0.8},
        ],
    )
    send_grasp_detections(node, 30, [])

    records = grasp_log_records(node, "GRASP_VERIFY_FRAME")
    assert len(records) == 2
    assert records[0]["frame_idx"] == 1
    assert records[0]["grab_count"] == 1
    assert records[0]["grab_max_confidence"] == 0.63
    assert records[0]["grab_pass"] is True
    assert records[0]["frame_result"] == "GRABBED"
    assert records[0]["pending_result"] == "NOT_GRABBED"
    assert records[0]["decision_window_frames"] == 1
    assert records[0]["decision_window_grabbed_frames"] == 1
    assert records[0]["bbox"] == [1, 2, 3, 4]
    assert records[0]["ball_detected"] is True
    assert records[1]["frame_idx"] == 2
    assert records[1]["grab_count"] == 0
    assert records[1]["grab_max_confidence"] is None
    assert records[1]["grab_pass"] is False
    assert records[1]["frame_result"] == "NOT_GRABBED"
    assert records[1]["pending_result"] == "NOT_GRABBED"
    assert records[1]["decision_window_frames"] == 2
    assert records[1]["decision_window_grabbed_frames"] == 1


def test_grasp_verification_reject_logs_preserve_pending_result():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    for stamp in range(20, 35):
        send_grasp_detections(
            node,
            stamp,
            [{"class_name": "grab", "confidence": 0.7}],
        )

    send_grasp_detections(node, 34, [])
    send_grasp_detections(node, 33, [])

    records = grasp_log_records(node, "GRASP_VERIFY_REJECT")
    assert [record["reason"] for record in records] == [
        "same_timestamp",
        "timestamp_regression",
    ]
    assert node.grasp_verify_rejected_frames == 2
    assert node.grasp_verification_result == "GRABBED"
    assert node.grasp_verify_accepted_frames == 15
    assert len(node.grasp_verify_frame_votes) == 15


def test_grasp_verification_end_distinguishes_last_frame_from_vote_result():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    for stamp, confidence in enumerate([0.4, 0.8] * 7 + [0.6], start=1):
        send_grasp_detections(
            node,
            stamp,
            [{"class_name": "grab", "confidence": confidence}],
        )
    for stamp in range(16, 41):
        send_grasp_detections(node, stamp, [])

    MotionDecisionNode._finish_grasp_verification_window(node)
    MotionDecisionNode._finish_grasp_verification_window(node)

    records = grasp_log_records(node, "GRASP_VERIFY_END")
    assert len(records) == 1
    assert records[0]["accepted_frames"] == 40
    assert records[0]["grabbed_frames"] == 15
    assert records[0]["not_grabbed_frames"] == 25
    assert records[0]["decision_window_frames"] == 40
    assert records[0]["decision_window_grabbed_frames"] == 15
    assert records[0]["frame_window_limit"] == 40
    assert records[0]["required_grab_frames"] == 15
    assert records[0]["grab_confidence_min"] == 0.4
    assert records[0]["grab_confidence_max"] == 0.8
    assert records[0]["grab_confidence_mean"] == pytest.approx(0.6)
    assert records[0]["grab_confidence_median"] == pytest.approx(0.6)
    assert records[0]["last_fresh_frame_result"] == "NOT_GRABBED"
    assert records[0]["latched_result"] == "GRABBED"
    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"


def test_grasp_verification_end_reports_zero_fresh_frames():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)

    MotionDecisionNode._finish_grasp_verification_window(node)

    record = grasp_log_records(node, "GRASP_VERIFY_END")[0]
    assert record["accepted_frames"] == 0
    assert record["fresh_frame_zero"] is True
    assert record["latched_result"] == "UNKNOWN"
    assert record["last_fresh_frame_result"] == "UNKNOWN"
    assert record["decision_window_frames"] == 0
    assert record["decision_window_grabbed_frames"] == 0


def test_grasp_summary_keeps_below_threshold_confidence_for_analysis():
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    send_grasp_detections(
        node,
        10,
        [{"class_name": "grab", "confidence": 0.24}],
    )

    MotionDecisionNode._finish_grasp_verification_window(node)

    frame = grasp_log_records(node, "GRASP_VERIFY_FRAME")[0]
    summary = grasp_log_records(node, "GRASP_VERIFY_END")[0]
    assert frame["grab_max_confidence"] == 0.24
    assert frame["grab_pass"] is False
    assert frame["frame_result"] == "NOT_GRABBED"
    assert summary["grab_confidence_min"] == 0.24
    assert summary["grab_confidence_max"] == 0.24
    assert summary["latched_result"] == "NOT_GRABBED"


def test_grasp_frame_detail_log_is_silent_outside_verification_window():
    node = FakeDecisionNode("LINE_TRACK")

    send_grasp_detections(
        node,
        10,
        [{"class_name": "grab", "confidence": 0.9}],
    )

    assert grasp_log_records(node, "GRASP_VERIFY_FRAME") == []
    assert grasp_log_records(node, "GRASP_VERIFY_REJECT") == []


@pytest.mark.parametrize("result", ["NOT_GRABBED", "UNKNOWN"])
def test_shot_is_blocked_without_verified_grab(result):
    node = FakeDecisionNode("GOAL_APPROACH")
    set_grasp_result_for_first_ball(node, result)
    shot = MotionDecision(
        phase="GOAL_APPROACH",
        source="goal",
        action="SHOT",
        valid=True,
        reason="goal_centered_at_scoring_depth",
        sdk_motion_requested=True,
        requires_ack=True,
        source_command={},
    )

    selected = MotionDecisionNode._suppress_unverified_shot(node, shot)

    assert selected.action == "WAIT"
    assert selected.sdk_motion_requested is False
    assert selected.source_command["grasp_result"] == result


def test_shot_remains_enabled_for_matching_grabbed_ball():
    node = FakeDecisionNode("GOAL_APPROACH")
    set_grasp_result_for_first_ball(node, "GRABBED")
    shot = MotionDecision(
        phase="GOAL_APPROACH",
        source="goal",
        action="SHOT",
        valid=True,
        reason="goal_centered_at_scoring_depth",
        sdk_motion_requested=True,
        requires_ack=True,
        source_command={},
    )

    assert MotionDecisionNode._suppress_unverified_shot(node, shot) == shot


def test_second_shot_does_not_reuse_first_ball_grab_result():
    node = FakeDecisionNode("GOAL_APPROACH")
    set_grasp_result_for_first_ball(node, "GRABBED")

    assert node.phase_manager.start_special_action("SHOT", 11)
    node.phase_manager.handle_motion_status("SHOT", 11, "RUNNING")
    node.phase_manager.handle_motion_status("SHOT", 11, "SUCCEEDED")

    assert node.phase_manager.start_special_action("PICKUP_NOW", 12)
    assert node.phase_manager.record_active_pickup_grasp_result("NOT_GRABBED")
    node.phase_manager.handle_motion_status("PICKUP_NOW", 12, "RUNNING")
    node.phase_manager.handle_motion_status("PICKUP_NOW", 12, "SUCCEEDED")

    shot = MotionDecision(
        phase="GOAL_APPROACH",
        source="goal",
        action="SHOT",
        valid=True,
        reason="goal_centered_at_scoring_depth",
        sdk_motion_requested=True,
        requires_ack=True,
        source_command={},
    )
    selected = MotionDecisionNode._suppress_unverified_shot(node, shot)

    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"
    assert node.phase_manager.grasp_result_for_ball(2) == "NOT_GRABBED"
    assert selected.action == "WAIT"
    assert selected.source_command["grasp_result"] == "NOT_GRABBED"


def test_valid_external_phase_updates_manager():
    node = FakeDecisionNode()
    send_phase(node, '{"phase": " goal_approach "}')
    assert node.phase_manager.current_phase == 'GOAL_APPROACH'


@pytest.mark.parametrize('phase', ['', '   ', '{"phase": ""}', 'UNKNOWN'])
def test_invalid_external_phase_keeps_manager_phase(phase):
    node = FakeDecisionNode('BALL_SEARCH')
    send_phase(node, phase)
    assert node.phase_manager.current_phase == 'BALL_SEARCH'


def test_external_phase_override_is_rejected_while_special_is_active():
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    send_phase(node, 'GOAL_APPROACH')
    assert node.phase_manager.current_phase == 'BALL_APPROACH'


def arm_special_command(node, action, command_id, event_id):
    """Model the active metadata stored when a special command is published."""
    assert node.phase_manager.start_special_action(action, command_id)
    node.active_special_event_id = event_id


def test_planner_uses_manager_current_phase():
    node = FakeDecisionNode('AUTO')
    assert node.phase_manager.set_phase('GOAL_APPROACH')
    decision = MotionDecisionNode._select_mission_decision(
        node,
        {'finish': None},
        0.1,
    )
    assert decision.phase == 'GOAL_APPROACH'


def test_planning_lock_does_not_change_manager_phase():
    node = FakeDecisionNode('HURDLE_APPROACH')
    arm_special_command(node, 'GO', 10, 1)
    decision = MotionDecisionNode._select_mission_decision(
        node,
        {'finish': None},
        0.1,
    )
    assert decision.phase == 'HURDLE_APPROACH_LOCK'
    assert node.phase_manager.current_phase == 'HURDLE_APPROACH'


def test_pickup_fine_checkpoint_discards_pre_motion_ball_frame():
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase('BALL_APPROACH')
    node.latest_info['ball'] = {
        'detected': True,
        'confidence': 0.9,
        'offset_x_norm': 0.0,
        'pickup_x_tolerance_norm': 0.08,
    }
    node.latest_time['ball'] = 10.0
    node.last_published_vision_stamp['ball'] = 10.0
    arm_special_command(node, 'PICKUP_NOW', 10, 1)

    send_status(
        node,
        status='RUNNING',
        action='PICKUP_NOW',
        command_id=10,
        event_id=1,
        dynamics_command=None,
        motion_id=MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER,
    )

    assert node.pickup_fine_align_waiting is True
    assert node.latest_info['ball'] is None
    assert node.latest_time['ball'] is None
    assert 'ball' not in node.last_published_vision_stamp
    assert node.active_special_command_id == 10


def test_pickup_fine_wait_uses_only_ball_and_blocks_other_sources():
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    node.pickup_fine_align_waiting = True

    missing = select_decision(
        node,
        line={'detected': True},
        goal={'detected': True},
        hurdle={'detected': True},
    )
    right = select_decision(
        node,
        ball={
            'detected': True,
            'confidence': 0.9,
            'offset_x_px': 56,
            'bottom_distance_px': 40,
            'offset_x_norm': 0.2,
            'pickup_x_tolerance_norm': 0.08,
            'ground_distance_m': 0.20,
            'depth_valid': False,
            'pickup_ready': False,
            'is_in_pickup_window': False,
        },
        line={'detected': True},
        goal={'detected': True},
        hurdle={'detected': True},
    )

    assert missing.action == 'WAIT'
    assert missing.source == 'ball'
    assert right.action == 'BALL_PICKUP_CRAB_RIGHT'
    assert right.source == 'ball'
    assert node.active_special_command_id == 10


def test_pickup_fine_terminal_status_clears_checkpoint_flag():
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    send_status(
        node,
        status='RUNNING',
        action='PICKUP_NOW',
        command_id=10,
        event_id=1,
        dynamics_command=None,
        motion_id=MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER,
    )

    send_status(
        node,
        status='FAILED',
        action='PICKUP_NOW',
        command_id=10,
        event_id=1,
        dynamics_command=None,
        motion_id='pickup_crab_right_0',
    )

    assert node.pickup_fine_align_waiting is False
    assert node.active_special_command_id is None


def test_legacy_state_properties_read_manager_state():
    node = FakeDecisionNode(
        required_pickups=3,
        required_shots=4,
        required_ball_sections=1,
    )
    complete_motion(node, 'SHOT', event_id=10)
    assert node.mission_phase == node.phase_manager.current_phase
    assert node.pickups_completed == node.phase_manager.pickups_completed
    assert node.shots_completed == node.phase_manager.shots_completed
    assert (
        node.ball_sections_processed
        == node.phase_manager.ball_sections_processed
    )
    assert node.finish_enabled == node.phase_manager.finish_enabled
    assert node.mission_complete == node.phase_manager.mission_complete
    assert node.active_special_action is None
    assert node.active_special_command_id is None


def complete_motion(
    node,
    action,
    event_id,
    status='SUCCEEDED',
):
    """Send matching RUNNING and terminal statuses for one event."""
    arm_special_command(node, action, event_id, event_id)
    send_status(
        node,
        status='RUNNING',
        action=action,
        command_id=event_id,
        event_id=event_id,
        dynamics_command=0,
    )
    send_status(
        node,
        status=status,
        action=action,
        command_id=event_id,
        event_id=event_id,
        dynamics_command=0,
    )


def test_general_status_callback_ignores_other_command_id():
    node = FakeDecisionNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("LEFT", command_id=700)

    send_status(
        node,
        status="RUNNING",
        action="TURN_LEFT",
        command_id=699,
        event_id=None,
        dynamics_command=None,
    )
    send_status(
        node,
        status="SUCCEEDED",
        action="TURN_LEFT",
        command_id=699,
        event_id=None,
        dynamics_command=None,
    )

    assert node.general_motion_gate.locked
    assert not node.general_motion_gate.running_seen


def test_general_status_callback_releases_same_command_id():
    node = FakeDecisionNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("LEFT", command_id=700)

    for status in ("RUNNING", "SUCCEEDED"):
        send_status(
            node,
            status=status,
            action="LEFT",
            command_id=700,
            event_id=None,
            dynamics_command=None,
        )

    assert not node.general_motion_gate.locked


def test_general_status_callback_preserves_transient_error_code():
    node = FakeDecisionNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published(
        "STRAIGHT",
        command_id=701,
    )

    send_status(
        node,
        status="REJECTED",
        action="STRAIGHT",
        command_id=701,
        event_id=None,
        dynamics_command=None,
        error_code="REJECTED_BUSY",
    )

    assert not node.general_motion_gate.can_publish("STRAIGHT")
    node.general_motion_gate.on_new_vision_input()
    assert node.general_motion_gate.can_publish("STRAIGHT")


def terminal_decision(source, action, phase):
    """Create one terminal planner decision for guard tests."""
    return MotionDecision(
        phase=phase,
        source=source,
        action=action,
        valid=True,
        reason='ready',
        sdk_motion_requested=True,
        requires_ack=True,
        source_command={},
    )


class ReadinessPublisher:
    """Capture commands while exposing the rclpy publisher readiness API."""

    def __init__(self, subscription_count=1):
        """Initialize one controllable subscription count."""
        self.messages = []
        self.subscription_count = subscription_count

    def publish(self, message):
        """Record one published command."""
        self.messages.append(message)

    def get_subscription_count(self):
        """Return the simulated matched subscription count."""
        return self.subscription_count


class ReadinessPublishNode(FakeDecisionNode):
    """Run the real publication path with one deterministic decision."""

    SPECIAL_FAILURE_REASONS = (
        MotionDecisionNode.SPECIAL_FAILURE_REASONS
    )

    def __init__(self, decision, subscription_count=1):
        """Initialize publication state without creating a ROS graph."""
        super().__init__(decision.phase)
        self.decision = decision
        self.planner = type(
            'PlannerTelemetry',
            (),
            {
                'ball_tracking_status': staticmethod(lambda: {}),
                'goal_tracking_status': staticmethod(lambda: {}),
                '_remember_pickup_close_ball': staticmethod(lambda _info: None),
                'clear_collected_ball_tracking': staticmethod(lambda: None),
            },
        )()
        self.previous_publish_time = 0.0
        self.command_id = 0
        self.event_id = 0
        self.terminal_latch = None
        self.publisher = ReadinessPublisher(subscription_count)
        self._command_publisher_ready = None
        self.executor_heartbeat_watchdog.observe(
            sequence=0,
            observed_at=0.0,
        )

    @staticmethod
    def _fresh_observations(_now):
        return {}, {}

    @staticmethod
    def _rearm_absent_terminal_targets(_observations):
        pass

    def _select_mission_decision(self, _observations, _dt_sec):
        return self.decision

    @staticmethod
    def _suppress_duplicate_terminal_action(selected):
        return selected

    def _suppress_exhausted_special_action(self, decision):
        return MotionDecisionNode._suppress_exhausted_special_action(
            self,
            decision,
        )

    def _latch_critical_executor_fault(self, **kwargs):
        return MotionDecisionNode._latch_critical_executor_fault(self, **kwargs)

    @staticmethod
    def _mission_progress():
        return {}

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


def general_decision(action='STRAIGHT'):
    """Create one executable general decision for publication tests."""
    return MotionDecision(
        phase='AUTO',
        source='line',
        action=action,
        valid=True,
        reason='line_ready',
        sdk_motion_requested=False,
        requires_ack=False,
        source_command={},
    )


def pickup_fine_decision(action):
    """Create one executable pickup-checkpoint control decision."""
    return MotionDecision(
        phase='BALL_PICKUP_FINE_ALIGN',
        source='ball',
        action=action,
        valid=True,
        reason='pickup_fine_test',
        sdk_motion_requested=action in {
            'BALL_PICKUP_CRAB_RIGHT',
            'BALL_PICKUP_CRAB_LEFT',
        },
        requires_ack=False,
        source_command={
            'offset_x_norm': 0.2,
            'pickup_x_tolerance_norm': 0.08,
        },
    )


def test_pickup_fine_command_keeps_parent_identity_and_consumes_frame():
    node = ReadinessPublishNode(general_decision())
    node.decision = pickup_fine_decision('BALL_PICKUP_CRAB_RIGHT')
    arm_special_command(node, 'PICKUP_NOW', 40, 4)
    node.pickup_fine_align_waiting = True
    node.latest_time = {'ball': 12.0}
    node.last_published_vision_stamp = {}

    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    payload = json.loads(node.publisher.messages[0].data)
    assert payload['action'] == 'BALL_PICKUP_CRAB_RIGHT'
    assert payload['event_id'] is None
    assert payload['active_special_action'] == 'PICKUP_NOW'
    assert payload['active_special_command_id'] == 40
    assert payload['active_special_event_id'] == 4
    assert payload['sdk_motion_requested'] is True
    assert node.last_published_vision_stamp['ball'] == 12.0
    assert node.pickup_fine_align_waiting is False


def test_pickup_left_crab_is_executable_and_consumes_its_frame():
    node = ReadinessPublishNode(general_decision())
    node.decision = pickup_fine_decision('BALL_PICKUP_CRAB_LEFT')
    arm_special_command(node, 'PICKUP_NOW', 40, 4)
    node.pickup_fine_align_waiting = True
    node.latest_time = {'ball': 12.0}
    node.last_published_vision_stamp = {}

    MotionDecisionNode._publish_decision(node)
    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    payload = json.loads(node.publisher.messages[0].data)
    assert payload['sdk_motion_requested'] is True
    assert node.pickup_fine_align_waiting is False


def enable_test_settle(node, monkeypatch, start=0.0):
    """Enable deterministic pre-motion settle timing for one fake node."""
    clock = [start]
    node.pre_motion_settle_sec = 0.5
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 0.5
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    return clock


@pytest.mark.parametrize('action', ['LEFT', 'RIGHT'])
def test_turn_waits_for_stable_half_second(action, monkeypatch):
    node = ReadinessPublishNode(general_decision(action))
    clock = enable_test_settle(node, monkeypatch)

    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages == []
    assert node.command_id == 0

    clock[0] = 0.49
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages == []

    clock[0] = 0.5
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1
    assert node.command_id == 1
    assert node.general_motion_gate.locked
    assert node.pre_motion_settle_started_at is None


def test_changed_turn_candidate_restarts_settle(monkeypatch):
    node = ReadinessPublishNode(general_decision('LEFT'))
    clock = enable_test_settle(node, monkeypatch)

    MotionDecisionNode._publish_decision(node)
    clock[0] = 0.3
    node.decision = general_decision('RIGHT')
    MotionDecisionNode._publish_decision(node)

    clock[0] = 0.55
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages == []

    clock[0] = 0.8
    MotionDecisionNode._publish_decision(node)
    payload = json.loads(node.publisher.messages[0].data)
    assert payload['action'] == 'RIGHT'


def test_new_frame_keeps_same_candidate_start_time(monkeypatch):
    node = ReadinessPublishNode(general_decision('LEFT'))
    node.latest_time = {'line': 1.0}
    node.last_published_vision_stamp = {}
    clock = enable_test_settle(node, monkeypatch)

    MotionDecisionNode._publish_decision(node)
    clock[0] = 0.3
    node.latest_time['line'] = 2.0
    node.general_motion_gate.on_new_vision_input()
    MotionDecisionNode._publish_decision(node)
    assert node.pre_motion_settle_started_at == 0.0

    clock[0] = 0.5
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1


def test_straight_publishes_without_settle(monkeypatch):
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    enable_test_settle(node, monkeypatch)

    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    assert node.command_id == 1


def test_decision_debug_reports_existing_state(monkeypatch):
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    monkeypatch.setattr(time, 'monotonic', lambda: 10.0)
    MotionDecisionNode._publish_decision(node)
    node.latest_info = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    node.latest_info['line'] = {
        'detected': True,
        'filtered_heading_error_deg': -19.5,
        'ground_projection_valid': True,
        'ground_heading_error_deg': 19.5,
        'filtered_lateral_offset_norm': 0.12,
    }
    node.latest_time = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    node.last_published_vision_stamp = {}
    node.latest_time['line'] = 9.9
    node.timeouts = {
        source: 0.5 for source in MotionDecisionNode.SOURCES
    }
    line_planner = type(
        'LinePlannerDebugState',
        (),
        {
            'turn_candidate': 'RIGHT',
            'turn_candidate_hits': 3,
            'config': type(
                'LinePlannerDebugConfig',
                (),
                {
                    'direction_confirmation_frames': 3,
                    'turn_enter_deg': 12.0,
                    'turn_exit_deg': 7.0,
                    'recovery_heading_turn_deg': 10.0,
                    'heading_source': 'ground',
                    'straight_max_heading_deg': 10.0,
                    'straight_max_image_heading_deg': 25.0,
                    'in_place_ground_heading_deg': 45.0,
                },
            )(),
        },
    )()
    node.planner.line_planner = line_planner
    node.executor_active = False
    node.decision_debug_publisher = ReadinessPublisher()

    MotionDecisionNode._publish_decision_debug(node)

    assert node.logger.warnings == []
    payload = json.loads(node.decision_debug_publisher.messages[0].data)
    assert payload['phase'] == 'AUTO'
    assert payload['source'] == 'LINE'
    assert payload['fresh_vision']['line'] is True
    assert payload['line'] == {
        'line_detected': True,
        'last_seen_direction': None,
        'heading_deg': 19.5,
        'heading_source': 'ground',
        'image_heading_deg': -19.5,
        'ground_heading_deg': 19.5,
        'ground_fit_reason': None,
        'straight_heading_aligned': False,
        'in_place_ground_heading_deg': 45.0,
        'center_offset': 0.12,
        'offset_reference_steering_deg': None,
        'lateral_offset_px': None,
        'pending_direction': 'RIGHT',
        'direction_confirmation_current': 3,
        'direction_confirmation_required': 3,
        'turn_enter_deg': 12.0,
        'turn_exit_deg': 7.0,
        'line_large_heading_threshold_deg': 10.0,
    }
    assert payload['decision']['candidate_action'] == 'STRAIGHT'
    assert payload['decision']['selected_action'] == 'STRAIGHT'
    assert payload['execution']['executor_state'] == 'IDLE'
    assert payload['grasp_verification'] == {
        'finalized': False,
        'active': False,
        'result': 'UNKNOWN',
        'confidence': None,
        'accepted_frames': 0,
        'decision_window_frames': 0,
        'decision_window_grabbed_frames': 0,
        'frame_window_limit': 40,
        'required_grab_frames': 15,
    }
    assert payload['safety']['latched'] is False
    node.safety_interlock.observe_executor_status(
        error_code='SDK_HARDWARE_NOT_READY',
        message='direct profile restore: Profile Velocity address=112 result=-1001',
        action='PICKUP_NOW', command_id=10, event_id=1,
    )
    MotionDecisionNode._publish_decision_debug(node)
    stopped = json.loads(node.decision_debug_publisher.messages[-1].data)
    assert stopped['safety']['latched'] is True
    assert stopped['safety']['error_code'] == 'SDK_HARDWARE_NOT_READY'
    assert 'address=112' in stopped['safety']['message']


def test_decision_debug_failure_does_not_escape():
    node = ReadinessPublishNode(general_decision())
    node.latest_info = {}
    node.decision_debug_publisher = type(
        'FailingPublisher',
        (),
        {'publish': staticmethod(lambda _message: (_ for _ in ()).throw(RuntimeError('debug failed')))},
    )()

    MotionDecisionNode._publish_decision_debug(node)

    assert any('Decision debug publication failed' in item for item in node.logger.warnings)


def test_only_current_production_motions_require_settle():
    assert MotionDecisionNode.PRE_MOTION_SETTLE_ACTIONS == frozenset(
        {'LEFT', 'RIGHT', 'PICKUP_NOW', 'GO', 'SHOT'}
    )


@pytest.mark.parametrize(
    'action',
    [
        'TURN_LEFT',
        'TURN_RIGHT',
        'ALIGN_LEFT',
        'ALIGN_RIGHT',
    ],
)
def test_unsupported_action_does_not_start_settle(action):
    node = ReadinessPublishNode(general_decision(action))
    node.pre_motion_settle_sec = 0.5

    assert node._pre_motion_settle_ready(node.decision, now=10.0)
    assert node.pre_motion_settle_started_at is None


def test_line_recover_publishes_immediately_once(monkeypatch):
    node = ReadinessPublishNode(general_decision("RECOVER_LEFT_TURN_LEFT_4"))
    node.pre_motion_settle_sec = 10.0
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 3.0
    monkeypatch.setattr(time, "monotonic", lambda: 10.0)
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1
    assert json.loads(node.publisher.messages[0].data)["action"] == "RECOVER_LEFT_TURN_LEFT_4"
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1


def test_stop_cancels_pending_turn_and_is_not_delayed(monkeypatch):
    node = ReadinessPublishNode(general_decision('LEFT'))
    clock = enable_test_settle(node, monkeypatch)
    MotionDecisionNode._publish_decision(node)

    clock[0] = 0.2
    node.decision = general_decision('STOP')
    MotionDecisionNode._publish_decision(node)

    payload = json.loads(node.publisher.messages[0].data)
    assert payload['action'] == 'STOP'
    assert node.pre_motion_settle_started_at is None


def test_invalid_decision_cancels_pending_settle(monkeypatch):
    node = ReadinessPublishNode(general_decision('LEFT'))
    clock = enable_test_settle(node, monkeypatch)
    MotionDecisionNode._publish_decision(node)

    clock[0] = 0.2
    node.decision = MotionDecision(
        phase='AUTO',
        source='line',
        action='LEFT',
        valid=False,
        reason='invalid_candidate',
        sdk_motion_requested=False,
        requires_ack=False,
        source_command={},
    )
    MotionDecisionNode._publish_decision(node)

    assert node.pre_motion_settle_started_at is None


def test_cross_finish_is_not_delayed(monkeypatch):
    node = ReadinessPublishNode(
        terminal_decision('finish', 'CROSS_FINISH', 'WALK_TO_FINISH')
    )
    enable_test_settle(node, monkeypatch)

    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    assert node.active_special_action == 'CROSS_FINISH'


@pytest.mark.parametrize(
    ('source', 'action', 'phase'),
    [
        ('ball', 'PICKUP_NOW', 'BALL_APPROACH'),
        ('hurdle', 'GO', 'HURDLE_APPROACH'),
    ],
)
def test_special_lock_and_ids_are_deferred_until_settle(
    source,
    action,
    phase,
    monkeypatch,
):
    node = ReadinessPublishNode(terminal_decision(source, action, phase))
    clock = enable_test_settle(node, monkeypatch)

    MotionDecisionNode._publish_decision(node)
    clock[0] = 0.49
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages == []
    assert node.command_id == 0
    assert node.event_id == 0
    assert node.active_special_command_id is None
    assert node.terminal_latch is None

    clock[0] = 0.5
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1
    assert node.command_id == 1
    assert node.event_id == 1
    assert node.active_special_command_id == 1


def test_general_lock_does_not_precount_next_turn(monkeypatch):
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    clock = enable_test_settle(node, monkeypatch, start=10.0)
    MotionDecisionNode._publish_decision(node)
    node.decision = general_decision('LEFT')

    clock[0] = 20.0
    MotionDecisionNode._publish_decision(node)
    assert node.pre_motion_settle_started_at is None

    for status in ('RUNNING', 'SUCCEEDED'):
        send_status(
            node,
            status=status,
            action='STRAIGHT',
            command_id=1,
            event_id=None,
            dynamics_command=None,
        )
    MotionDecisionNode._publish_decision(node)
    assert node.pre_motion_settle_started_at == 20.0

    clock[0] = 20.49
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1
    clock[0] = 20.5
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 2


def test_active_special_lock_does_not_precount_next_settle(monkeypatch):
    node = ReadinessPublishNode(general_decision('LEFT'))
    clock = enable_test_settle(node, monkeypatch, start=10.0)
    assert node.phase_manager.start_special_action('PICKUP_NOW', 7)

    MotionDecisionNode._publish_decision(node)
    assert node.pre_motion_settle_started_at is None
    assert node.publisher.messages == []

    node.phase_manager.handle_motion_status('PICKUP_NOW', 7, 'UNSUPPORTED')
    clock[0] = 20.0
    MotionDecisionNode._publish_decision(node)
    assert node.pre_motion_settle_started_at == 20.0


def test_published_turn_requires_a_new_settle_after_completion(monkeypatch):
    node = ReadinessPublishNode(general_decision('LEFT'))
    clock = enable_test_settle(node, monkeypatch)
    MotionDecisionNode._publish_decision(node)
    clock[0] = 0.5
    MotionDecisionNode._publish_decision(node)

    for status in ('RUNNING', 'SUCCEEDED'):
        send_status(
            node,
            status=status,
            action='LEFT',
            command_id=1,
            event_id=None,
            dynamics_command=None,
        )
    assert node.correction_post_motion_dwell_until == 1.5
    clock[0] = 1.499
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1
    assert node.pre_motion_settle_started_at is None
    clock[0] = 1.5
    MotionDecisionNode._publish_decision(node)
    assert node.latest_info['line'] is None
    node.latest_time = {'line': 2.0}
    node.last_published_vision_stamp = {'line': 1.0}
    node.general_motion_gate.on_new_vision_input()
    clock[0] = 2.0
    MotionDecisionNode._publish_decision(node)
    assert node.pre_motion_settle_started_at == 2.0
    clock[0] = 2.49
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1
    clock[0] = 2.5
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 2


def test_correlated_motor_failure_releases_and_allows_other_action():
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    MotionDecisionNode._publish_decision(node)
    send_status(
        node,
        status='RUNNING',
        action='STRAIGHT',
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )
    send_status(
        node,
        status='FAILED',
        action='STRAIGHT',
        command_id=1,
        event_id=None,
        dynamics_command=None,
        error_code='SDK_COMMUNICATION_ERROR',
        message='communication lost',
    )

    assert not node.safety_interlock.latched
    assert not node.general_motion_gate.locked
    node.decision = general_decision('LEFT')
    for _ in range(3):
        node.general_motion_gate.on_new_vision_input()
        MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 2


def test_mismatched_critical_general_status_does_not_latch():
    node = FakeDecisionNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published('STRAIGHT', command_id=10)
    send_status(
        node,
        status='FAILED',
        action='STRAIGHT',
        command_id=9,
        event_id=None,
        dynamics_command=None,
        error_code='SDK_COMMUNICATION_ERROR',
    )
    assert not node.safety_interlock.latched
    assert node.general_motion_gate.locked


def test_command_local_rejection_does_not_latch_safety():
    node = FakeDecisionNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published('STRAIGHT', command_id=10)
    send_status(
        node,
        status='REJECTED',
        action='STRAIGHT',
        command_id=10,
        event_id=None,
        dynamics_command=None,
        error_code='SDK_MOTION_NOT_FOUND',
    )
    assert not node.safety_interlock.latched


def test_sdk_hardware_not_ready_latches_instead_of_retrying():
    node = FakeDecisionNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published('STRAIGHT', command_id=10)
    send_status(
        node,
        status='REJECTED',
        action='STRAIGHT',
        command_id=10,
        event_id=None,
        dynamics_command=None,
        error_code='SDK_HARDWARE_NOT_READY',
    )
    assert node.safety_interlock.latched
    assert node.general_motion_gate.rejected_action == 'STRAIGHT'
    assert node.general_motion_gate.transient_rejection_count == 0


def test_non_motor_critical_special_failure_keeps_terminal_policy_and_blocks_motion():
    node = ReadinessPublishNode(
        terminal_decision('ball', 'PICKUP_NOW', 'BALL_APPROACH')
    )
    MotionDecisionNode._publish_decision(node)
    send_status(
        node,
        status='RUNNING',
        action='PICKUP_NOW',
        command_id=1,
        event_id=1,
        dynamics_command=9,
    )
    send_status(
        node,
        status='FAILED',
        action='PICKUP_NOW',
        command_id=1,
        event_id=1,
        dynamics_command=9,
        error_code='BACKEND_EXCEPTION',
    )

    assert node.safety_interlock.latched
    assert node.mission_phase == 'BALL_APPROACH'
    assert node.phase_manager.pickup_failure_count == 1
    assert node.active_special_command_id is None

    node.decision = general_decision('APPROACH')
    node.general_motion_gate.on_new_vision_input()
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1


def test_success_and_fresh_vision_do_not_clear_critical_latch():
    node = ReadinessPublishNode(general_decision())
    node.safety_interlock.observe_executor_status(
        error_code='BACKEND_EXCEPTION',
        message='exception',
        action='STRAIGHT',
        command_id=1,
        event_id=None,
    )
    for _ in range(3):
        node.general_motion_gate.on_new_vision_input()
        send_status(
            node,
            status='SUCCEEDED',
            action='STRAIGHT',
            command_id=1,
            event_id=None,
            dynamics_command=None,
        )
        MotionDecisionNode._publish_decision(node)
    assert node.safety_interlock.latched
    assert node.publisher.messages == []


def test_never_seen_executor_after_grace_does_not_latch():
    node = FakeDecisionNode()
    node._check_executor_heartbeat(now=5.0)
    assert not node.safety_interlock.latched

    node._check_executor_heartbeat(now=5.001)
    assert not node.safety_interlock.latched
    assert any(
        'waiting for executor readiness' in warning
        for warning in node.logger.warnings
    )


def test_motion_is_blocked_until_first_executor_heartbeat():
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.executor_heartbeat_watchdog = ExecutorHeartbeatWatchdog(
        started_at=0.0,
        startup_grace_sec=5.0,
        timeout_sec=2.0,
    )

    node._check_executor_heartbeat(now=6.0)
    MotionDecisionNode._publish_decision(node)

    assert not node.safety_interlock.latched
    assert node.publisher.messages == []
    assert node.command_id == 0


def test_late_first_heartbeat_releases_motion_publication():
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.executor_heartbeat_watchdog = ExecutorHeartbeatWatchdog(
        started_at=0.0,
        startup_grace_sec=5.0,
        timeout_sec=2.0,
    )
    node._check_executor_heartbeat(now=6.0)

    node.executor_heartbeat_watchdog.observe(sequence=1, observed_at=7.0)
    MotionDecisionNode._publish_decision(node)

    assert not node.safety_interlock.latched
    assert len(node.publisher.messages) == 1


def test_startup_hold_discards_commands_until_ready_and_fresh_vision(monkeypatch):
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.last_published_vision_stamp = {}
    node.latest_info = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    node.latest_time = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    node.timeouts = {
        source: 0.5 for source in MotionDecisionNode.SOURCES
    }
    clock = [1.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])

    hold_heartbeat = String()
    hold_heartbeat.data = json.dumps(
        {'sequence': 1, 'active': False, 'auto_ready': False}
    )
    MotionDecisionNode._executor_heartbeat_callback(node, hold_heartbeat)

    line_message = String()
    line_message.data = json.dumps({'detected': True})
    MotionDecisionNode._info_callback(node, 'line')(line_message)
    MotionDecisionNode._publish_decision(node)

    assert node.publisher.messages == []
    assert node.command_id == 0
    assert node.latest_info['line'] is None
    assert not node.safety_interlock.latched

    ready_heartbeat = String()
    ready_heartbeat.data = json.dumps(
        {'sequence': 2, 'active': False, 'auto_ready': True}
    )
    MotionDecisionNode._executor_heartbeat_callback(node, ready_heartbeat)
    MotionDecisionNode._publish_decision(node)

    assert node.publisher.messages == []
    assert node.command_id == 0

    clock[0] = 1.01
    MotionDecisionNode._info_callback(node, 'line')(line_message)
    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    assert node.command_id == 1


def test_runtime_loss_is_detected_after_startup_delay_and_late_heartbeat():
    node = FakeDecisionNode()
    node._check_executor_heartbeat(now=6.0)
    assert not node.safety_interlock.latched

    node.executor_heartbeat_watchdog.observe(sequence=1, observed_at=7.0)
    node._check_executor_heartbeat(now=9.001)

    snapshot = node.safety_interlock.snapshot
    assert snapshot.latched
    assert snapshot.error_code == 'EXECUTOR_HEARTBEAT_TIMEOUT'
    assert snapshot.source == 'executor_watchdog'


def test_heartbeat_loss_preserves_active_general_correlation_and_blocks():
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    MotionDecisionNode._publish_decision(node)
    node.executor_heartbeat_watchdog.observe(
        sequence=1,
        observed_at=1.0,
    )
    node._check_executor_heartbeat(now=3.001)

    snapshot = node.safety_interlock.snapshot
    assert snapshot.error_code == 'EXECUTOR_HEARTBEAT_TIMEOUT'
    assert snapshot.action == 'STRAIGHT'
    assert snapshot.command_id == 1
    assert snapshot.event_id is None

    node.decision = general_decision('LEFT')
    node.general_motion_gate.on_new_vision_input()
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1


def test_heartbeat_loss_preserves_active_special_correlation_and_blocks():
    node = ReadinessPublishNode(
        terminal_decision('ball', 'PICKUP_NOW', 'BALL_APPROACH')
    )
    MotionDecisionNode._publish_decision(node)
    node.executor_heartbeat_watchdog.observe(
        sequence=1,
        observed_at=1.0,
    )
    node._check_executor_heartbeat(now=3.001)

    snapshot = node.safety_interlock.snapshot
    assert snapshot.action == 'PICKUP_NOW'
    assert snapshot.command_id == 1
    assert snapshot.event_id == 1
    assert node.active_special_command_id == 1

    node.decision = general_decision('APPROACH')
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1


def test_returning_heartbeat_does_not_clear_timeout_latch():
    node = FakeDecisionNode()
    node.executor_heartbeat_watchdog.observe(
        sequence=0,
        observed_at=1.0,
    )
    node._check_executor_heartbeat(now=3.001)
    original = node.safety_interlock.snapshot
    node.executor_heartbeat_watchdog.observe(
        sequence=0,
        observed_at=7.0,
    )
    node._check_executor_heartbeat(now=7.1)
    assert node.safety_interlock.snapshot == original


def test_heartbeat_timeout_does_not_overwrite_first_critical_fault():
    node = FakeDecisionNode()
    node.safety_interlock.observe_executor_status(
        error_code='BACKEND_EXCEPTION',
        message='first fault',
        action='STRAIGHT',
        command_id=4,
        event_id=None,
    )
    node._check_executor_heartbeat(now=6.0)
    snapshot = node.safety_interlock.snapshot
    assert snapshot.error_code == 'BACKEND_EXCEPTION'
    assert snapshot.message == 'first fault'


def test_special_command_metadata_is_stored_when_first_published():
    decision = terminal_decision('hurdle', 'GO', 'HURDLE_APPROACH')
    node = ReadinessPublishNode(decision)
    node.command_id = 99
    node.event_id = 4

    MotionDecisionNode._publish_decision(node)
    payload = json.loads(node.publisher.messages[0].data)

    assert payload['command_id'] == 100
    assert payload['event_id'] == 5
    assert node.active_special_action == 'GO'
    assert node.active_special_command_id == 100
    assert node.active_special_event_id == 5
    assert node.special_motion_running is False


def test_general_command_waits_for_subscriber_without_consuming_state():
    decision = MotionDecision(
        phase='AUTO',
        source='line',
        action='STRAIGHT',
        valid=True,
        reason='line_ready',
        sdk_motion_requested=False,
        requires_ack=False,
        source_command={},
    )
    node = ReadinessPublishNode(decision, subscription_count=0)

    MotionDecisionNode._publish_decision(node)
    MotionDecisionNode._publish_decision(node)

    assert node.publisher.messages == []
    assert node.command_id == 0
    assert not node.general_motion_gate.locked
    assert len(node.logger.warnings) == 1

    node.publisher.subscription_count = 1
    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    assert json.loads(node.publisher.messages[0].data)['command_id'] == 1
    assert node.command_id == 1
    assert node.general_motion_gate.locked
    assert len(node.logger.infos) == 1


def test_same_vision_frame_is_not_republished_after_success():
    decision = MotionDecision(
        phase='AUTO',
        source='line',
        action='STRAIGHT',
        valid=True,
        reason='line_ready',
        sdk_motion_requested=False,
        requires_ack=False,
        source_command={},
    )
    node = ReadinessPublishNode(decision)
    node.latest_time = {'line': 1.0}
    node.last_published_vision_stamp = {}

    MotionDecisionNode._publish_decision(node)
    for status in ('RUNNING', 'SUCCEEDED'):
        send_status(
            node,
            status=status,
            action='STRAIGHT',
            command_id=1,
            event_id=None,
            dynamics_command=None,
        )

    node.general_motion_gate.on_new_vision_input()
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1

    node.latest_time['line'] = 2.0
    node.general_motion_gate.on_new_vision_input()
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 2


def test_running_general_motion_suppresses_new_special_command():
    general = MotionDecision(
        phase='AUTO',
        source='line',
        action='STRAIGHT',
        valid=True,
        reason='line_ready',
        sdk_motion_requested=False,
        requires_ack=False,
        source_command={},
    )
    node = ReadinessPublishNode(general)
    MotionDecisionNode._publish_decision(node)
    send_status(
        node,
        status='RUNNING',
        action='STRAIGHT',
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )

    node.decision = terminal_decision(
        'ball',
        'PICKUP_NOW',
        'BALL_APPROACH',
    )
    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    assert node.active_special_command_id is None


def test_completed_line_motion_can_publish_next_decision_without_fresh_vision():
    general = MotionDecision(
        phase='AUTO',
        source='line',
        action='STRAIGHT',
        valid=True,
        reason='line_ready',
        sdk_motion_requested=False,
        requires_ack=False,
        source_command={},
    )
    node = ReadinessPublishNode(general)

    MotionDecisionNode._publish_decision(node)

    for status in ('RUNNING', 'SUCCEEDED'):
        send_status(
            node,
            status=status,
            action='STRAIGHT',
            command_id=1,
            event_id=None,
            dynamics_command=None,
        )

    node.decision = terminal_decision(
        'ball',
        'PICKUP_NOW',
        'BALL_APPROACH',
    )

    # The late-motion capture replaces post-terminal fresh-Vision gating.
    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 2
    assert node.active_special_action == 'PICKUP_NOW'
    assert node.active_special_command_id == 2


@pytest.mark.parametrize("latest_detected", [True, False])
@pytest.mark.parametrize("latest_ground_valid", [True, False])
def test_line_motion_uses_recent_valid_frames_at_capture_threshold(
    monkeypatch, latest_detected, latest_ground_valid,
):
    node = ReadinessPublishNode(general_decision("STRAIGHT"))
    clock = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])

    class RecordingLinePlanner:
        """Record replayed line frames for one capture-window test."""

        def __init__(self):
            self.frames = []
            self.config = type(
                "Config",
                (),
                {"min_line_quality": 0.35, "heading_source": "ground"},
            )()

        def _reset_turn_state(self):
            pass

        def plan(self, frame, _dt_sec, *, allow_corner_turns=True):
            assert allow_corner_turns is True
            self.frames.append(frame)

    recording_planner = RecordingLinePlanner()
    node.planner.line_planner = recording_planner
    node.planner.observe_line_for_search = lambda _info: None

    MotionDecisionNode._publish_decision(node)
    send_status(
        node,
        status="RUNNING",
        action="STRAIGHT",
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )

    assert node.active_line_motion_duration_sec == pytest.approx(2.467)
    assert node.active_line_motion_target_frames == 10

    def valid_frame(frame_id):
        return {
            "frame": frame_id,
            "detected": True,
            "ground_projection_valid": True,
            "ground_heading_error_deg": 0.0,
            "filtered_lateral_offset_norm": 0.0,
            "heading_quality": 0.9,
            "geometry_quality": 0.9,
            "detection_quality": 0.9,
        }

    clock[0] = 11.0
    for frame_id in range(10):
        assert not MotionDecisionNode._collect_active_line_motion_frame(
            node,
            valid_frame(frame_id),
            clock[0],
        )
    assert len(node.active_line_motion_frames) == 10

    clock[0] = 11.73
    capture_ready = MotionDecisionNode._collect_active_line_motion_frame(
        node,
        {"detected": False},
        clock[0],
    )
    assert capture_ready
    latest_frame = {
        **valid_frame(10),
        "detected": latest_detected,
        "ground_projection_valid": latest_ground_valid,
    }
    node._fresh_observations = lambda _now: ({"line": latest_frame}, {})
    MotionDecisionNode._prepare_pending_line_decision(
        node,
        clock[0],
    )
    assert (node.pending_line_decision is not None) is (
        latest_detected and latest_ground_valid
    )
    assert len(node.publisher.messages) == (2 if latest_detected and latest_ground_valid else 1)
    if not latest_detected:
        node.decision = general_decision("LINE_LOST_TURN_LEFT")

    send_status(
        node,
        status="SUCCEEDED",
        action="STRAIGHT",
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )

    assert len(recording_planner.frames) == (10 if latest_detected and latest_ground_valid else 20)
    assert node.active_line_motion_frames == []
    assert len(node.publisher.messages) == (2 if latest_detected and latest_ground_valid else 1)
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 2
    expected = "STRAIGHT" if latest_detected else "LINE_LOST_TURN_LEFT_3"
    assert json.loads(node.publisher.messages[-1].data)["action"] == expected


def test_line_motion_capture_table_matches_deployed_timelines():
    expected = {
        "STRAIGHT": (2.467, 10),
        "STRAIGHT_1": (0.822, 5),
        "STRAIGHT_2": (1.644, 10),
        "STRAIGHT_3": (2.467, 10),
        "STRAIGHT_4": (3.289, 15),
        "LEFT": (7.365, 30),
        "RIGHT": (7.105, 30),
    }
    left_timelines = {
        2: (2.272, 10),
        4: (3.198, 15),
        6: (4.124, 20),
        8: (5.050, 25),
        10: (5.976, 30),
        13: (7.365, 30),
    }
    right_timelines = {
        4: (1.895, 5),
        6: (2.842, 10),
        8: (3.789, 15),
        10: (4.737, 20),
        12: (5.684, 25),
        15: (7.105, 30),
    }
    for recovery_side in ("LEFT", "RIGHT"):
        for suffix, timeline in left_timelines.items():
            expected[
                f"RECOVER_{recovery_side}_TURN_LEFT_{suffix}"
            ] = timeline
        for suffix, timeline in right_timelines.items():
            expected[
                f"RECOVER_{recovery_side}_TURN_RIGHT_{suffix}"
            ] = timeline

    assert MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG == expected


def test_line_prequeue_retries_after_temporary_target_disappears(monkeypatch):
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.planner = MotionDecisionPlanner()
    clock = [10.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    line = {
        'detected': True, 'ground_projection_valid': True,
        'ground_heading_error_deg': 0., 'filtered_lateral_offset_norm': 0.,
        'heading_quality': .9, 'geometry_quality': .9, 'detection_quality': .9,
    }
    observations = {'line': line, 'goal': {'raw_detected': True}}
    node._fresh_observations = lambda _now: (dict(observations), {})
    MotionDecisionNode._publish_decision(node)
    send_status(node, status='RUNNING', action='STRAIGHT', command_id=1,
                event_id=None, dynamics_command=None)
    clock[0] = 11.8
    for _ in range(10):
        MotionDecisionNode._collect_active_line_motion_frame(node, line, clock[0])
    MotionDecisionNode._prepare_pending_line_decision(node, clock[0])
    assert node.pending_line_decision is not None
    assert len(node.publisher.messages) == 1

    # Still in the SAME running motion: the next frame may now queue straight.
    observations['goal'] = None
    clock[0] = 11.9
    assert MotionDecisionNode._collect_active_line_motion_frame(node, line, clock[0])
    MotionDecisionNode._prepare_pending_line_decision(node, clock[0])
    assert len(node.publisher.messages) == 2
    assert node.queued_general_command_id == 2
    assert node.general_motion_gate.active_command_id == 1
    assert not MotionDecisionNode._collect_active_line_motion_frame(node, line, clock[0])


@pytest.mark.parametrize("source", ["goal", "hurdle", "finish"])
def test_line_seamless_prequeue_stops_when_other_mission_is_detected(source):
    node = ReadinessPublishNode(general_decision("STRAIGHT"))
    observations = {name: None for name in MotionDecisionNode.SOURCES}
    observations[source] = {"raw_detected": True, "detected": False}

    assert not MotionDecisionNode._line_only_prequeue_allowed(
        node,
        observations,
    )


def test_line_seamless_prequeue_stops_at_ball_approach_entry():
    node = ReadinessPublishNode(general_decision("STRAIGHT"))
    observations = {name: None for name in MotionDecisionNode.SOURCES}
    node.ball_approach_entry_pending = True

    assert not MotionDecisionNode._line_only_prequeue_allowed(
        node,
        observations,
    )


def test_line_timeout_discards_capture_and_requires_ten_new_valid_frames():
    node = ReadinessPublishNode(general_decision("STRAIGHT"))

    class RecordingLinePlanner:
        """Record only the frames used for timeout recovery."""

        def __init__(self):
            self.frames = []

        def _reset_turn_state(self):
            self.frames = []

        def plan(self, frame, _dt_sec, *, allow_corner_turns=True):
            assert allow_corner_turns is True
            self.frames.append(frame)

    recording_planner = RecordingLinePlanner()
    node.planner.line_planner = recording_planner

    MotionDecisionNode._publish_decision(node)
    send_status(
        node,
        status="RUNNING",
        action="STRAIGHT",
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )
    node.active_line_motion_frames = [{"frame": "old"}]

    send_status(
        node,
        status="FAILED",
        action="STRAIGHT",
        command_id=1,
        event_id=None,
        dynamics_command=None,
        error_code=" timeout ",
    )

    assert node.active_line_motion_frames == []
    assert node.line_timeout_recovery_active is True
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 1

    MotionDecisionNode._collect_timeout_recovery_frame(
        node,
        {"detected": False, "frame": "invalid"},
    )
    assert node.line_timeout_recovery_frames == []

    for frame_id in range(9):
        node.general_motion_gate.on_new_vision_input()
        MotionDecisionNode._collect_timeout_recovery_frame(
            node,
            {"detected": True, "frame": frame_id},
        )
    assert node.line_timeout_recovery_active is True
    assert recording_planner.frames == []

    node.general_motion_gate.on_new_vision_input()
    MotionDecisionNode._collect_timeout_recovery_frame(
        node,
        {"detected": True, "frame": 9},
    )

    assert node.line_timeout_recovery_active is False
    assert [frame["frame"] for frame in recording_planner.frames] == list(
        range(10)
    )
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == 2



def test_special_unsupported_status_releases_lock_without_substitution():
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 41, 4)

    send_status(
        node,
        status='UNSUPPORTED',
        action='PICKUP_NOW',
        command_id=41,
        event_id=4,
        dynamics_command=None,
        error_code='UNSUPPORTED_ACTION',
    )

    assert node.active_special_command_id is None
    assert node.active_special_event_id is None
    assert node.mission_phase == 'BALL_APPROACH'


@pytest.mark.parametrize(
    ('source', 'action', 'phase'),
    [
        ('ball', 'PICKUP_NOW', 'BALL_APPROACH'),
        ('goal', 'SHOT', 'GOAL_APPROACH'),
        ('hurdle', 'GO', 'HURDLE_APPROACH'),
    ],
)
def test_special_command_waits_for_subscriber_before_registering_lock(
    source,
    action,
    phase,
):
    node = ReadinessPublishNode(
        terminal_decision(source, action, phase),
        subscription_count=0,
    )
    if action == 'SHOT':
        set_grasp_result_for_first_ball(node, 'GRABBED')
        assert node.phase_manager.set_phase(phase)

    MotionDecisionNode._publish_decision(node)

    assert node.publisher.messages == []
    assert node.command_id == 0
    assert node.event_id == 0
    assert node.active_special_action is None
    assert node.active_special_command_id is None
    assert node.active_special_event_id is None
    assert node.special_motion_running is False
    assert node.terminal_latch is None
    assert node.mission_phase == phase

    node.publisher.subscription_count = 1
    MotionDecisionNode._publish_decision(node)

    assert len(node.publisher.messages) == 1
    payload = json.loads(node.publisher.messages[0].data)
    assert payload['command_id'] == 1
    assert payload['event_id'] == 1
    assert node.command_id == 1
    assert node.event_id == 1
    assert node.active_special_action == action
    assert node.active_special_command_id == 1
    assert node.active_special_event_id == 1
    assert node.terminal_latch == (source, action)


def test_reconnected_special_command_keeps_running_status_correlation():
    node = ReadinessPublishNode(
        terminal_decision('hurdle', 'GO', 'HURDLE_APPROACH'),
        subscription_count=0,
    )
    MotionDecisionNode._publish_decision(node)
    node.publisher.subscription_count = 1
    MotionDecisionNode._publish_decision(node)

    send_status(
        node,
        status='RUNNING',
        action='GO',
        command_id=1,
        event_id=1,
        dynamics_command=None,
    )

    assert node.special_motion_running is True
    assert node.active_special_action == 'GO'
    assert node.active_special_command_id == 1
    assert node.active_special_event_id == 1


def select_decision(node, finish=None, **observations):
    """Run the real node priority selection with deterministic inputs."""
    inputs = {
        'line': None,
        'ball': None,
        'goal': None,
        'hurdle': None,
        'finish': finish,
    }
    inputs.update(observations)
    return MotionDecisionNode._select_mission_decision(
        node,
        inputs,
        0.1,
    )


def receive_confirmed_ball_loss(node, monkeypatch, start=10.0):
    """Feed five distinct misses spanning the complete confirmation window."""
    receive = MotionDecisionNode._info_callback(node, "ball")
    for i in range(5):
        now = start + i * 0.125
        monkeypatch.setattr(time, "monotonic", lambda now=now: now)
        receive(String(data=json.dumps({
            "detected": False, "raw_detected": False,
            "rgb_stamp_ns": round(now * 1e9),
        })))
        assert node.latest_info["ball"]["ball_loss_confirmed"] is (i == 4)
    return node.latest_info["ball"]


def ball_info_for_node(**overrides):
    """Build one confirmed, fresh BALL sample for node flow tests."""
    sample = {
        "detected": True,
        "confidence": 0.95,
        "depth_valid": True,
        "depth_age_sec": 0.05,
        "depth_m": 0.9,
        "distance_m": 0.9,
        "ground_distance_m": 0.9,
        "steering_angle_deg": 0.0,
        "bearing_deg": 0.0,
        "offset_x_px": 0,
        "offset_x_norm": 0.0,
        "camera_center_offset_x_px": 0,
        "bottom_distance_px": 600,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    if "camera_center_offset_x_px" not in overrides and "offset_x_px" in overrides:
        sample["camera_center_offset_x_px"] = overrides["offset_x_px"]
    return sample


class FreshMockInputNode(FakeDecisionNode):
    """Provide the real freshness and planner state for mock input tests."""

    SOURCES = MotionDecisionNode.SOURCES

    def __init__(self):
        super().__init__()
        from mission_control.motion_decision_planner import (
            MotionDecisionPlanner,
        )

        self.planner = MotionDecisionPlanner()
        self.latest_info = {
            source: None for source in self.SOURCES
        }
        self.latest_time = {
            source: None for source in self.SOURCES
        }
        self.timeouts = {
            source: 0.5 for source in self.SOURCES
        }


def test_raw_ball_latches_confirmation_hold_until_confirmed():
    node = FreshMockInputNode()

    MotionDecisionNode._update_ball_confirmation_pending(
        node,
        {
            "detected": False,
            "raw_detected": True,
            "confirmation_hits": 5,
            "confirmation_required_hits": 12,
        },
        10.0,
    )

    assert node.ball_confirmation_pending_latched is True
    decision = select_decision(
        node,
        ball={
            "detected": False,
            "raw_detected": True,
            "confirmation_hits": 5,
            "confirmation_required_hits": 12,
        },
        line={"detected": True},
    )
    assert decision.action == "WAIT"
    assert decision.reason == "ball_confirmation_pending_hold"

    MotionDecisionNode._update_ball_confirmation_pending(
        node,
        {"detected": True, "confirmation_confirmed": True},
        10.1,
    )
    assert node.ball_confirmation_pending_latched is False


def test_raw_ball_confirmation_hold_releases_after_detection_is_lost():
    node = FreshMockInputNode()

    MotionDecisionNode._update_ball_confirmation_pending(
        node,
        {"detected": False, "raw_detected": True},
        10.0,
    )
    MotionDecisionNode._update_ball_confirmation_pending(
        node,
        {"detected": False, "raw_detected": False},
        10.49,
    )
    assert node.ball_confirmation_pending_latched is True

    MotionDecisionNode._update_ball_confirmation_pending(
        node,
        {"detected": False, "raw_detected": False},
        10.5,
    )
    assert node.ball_confirmation_pending_latched is False


def test_ball_callback_latches_valid_pickup_entry_during_ball_motion():
    node = FreshMockInputNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published(
        "STRAIGHT_3",
        command_id=1,
    )
    node.active_general_source = "ball"
    message = String()
    message.data = json.dumps(
        {
            "detected": True,
            "confidence": 0.95,
            "depth_valid": True,
            "depth_m": 0.55,
            "distance_m": 0.55,
            "depth_age_sec": 0.05,
        }
    )

    MotionDecisionNode._info_callback(node, "ball")(message)

    assert node.ball_pickup_entry_pending is True


def test_ball_approach_entry_during_line_motion_requires_fresh_frame_without_dwell(
    monkeypatch,
):
    node = FreshMockInputNode()
    node.BALL_POST_MOTION_DWELL_SEC = 3.0
    monkeypatch.setattr(time, "monotonic", lambda: 10.0)
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("STRAIGHT_3", command_id=1)
    node.active_general_source = "line"
    message = String()
    message.data = json.dumps(ball_info_for_node(distance_m=1.4))

    MotionDecisionNode._info_callback(node, "ball")(message)

    assert node.general_motion_gate.locked is True
    assert node.ball_approach_entry_pending is True
    assert node.planner.ball_lock_active is True

    send_status(
        node,
        status="RUNNING",
        action="STRAIGHT_3",
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )
    send_status(
        node,
        status="SUCCEEDED",
        action="STRAIGHT_3",
        command_id=1,
        event_id=None,
        dynamics_command=None,
    )

    assert node.general_motion_gate.locked is False
    assert node.latest_info["ball"] is None
    assert node.ball_approach_alignment_pending is True
    assert node.ball_post_motion_dwell_until is None
    waiting = select_decision(node)
    assert waiting.action == "WAIT"

    MotionDecisionNode._info_callback(node, "ball")(message)
    decision = select_decision(
        node,
        ball=ball_info_for_node(
            distance_m=1.4,
            steering_angle_deg=-30.0,
        ),
    )
    assert decision.source == "ball"
    assert decision.action == "BALL_APPROACH_RECOVER_LEFT_4"


def test_ball_approach_entry_after_line_release_has_no_initial_dwell(
    monkeypatch,
):
    node = FreshMockInputNode()
    node.BALL_POST_MOTION_DWELL_SEC = 3.0
    monkeypatch.setattr(time, "monotonic", lambda: 20.0)
    message = String()
    message.data = json.dumps(ball_info_for_node(distance_m=0.76))

    MotionDecisionNode._info_callback(node, "ball")(message)

    assert node.general_motion_gate.locked is False
    assert node.ball_approach_entry_pending is True
    assert node.ball_approach_alignment_pending is True
    assert node.ball_post_motion_dwell_until is None
    assert node.planner.ball_lock_active is True


def test_ball_straight_success_requires_fresh_frame_without_dwell(monkeypatch):
    node = FreshMockInputNode()
    node.BALL_POST_MOTION_DWELL_SEC = 3.0
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("STRAIGHT_4", command_id=2)
    node.active_general_source = "ball"
    monkeypatch.setattr(time, "monotonic", lambda: 10.0)

    send_status(
        node,
        status="RUNNING",
        action="STRAIGHT_4",
        command_id=2,
        event_id=None,
        dynamics_command=None,
    )
    send_status(
        node,
        status="SUCCEEDED",
        action="STRAIGHT_4",
        command_id=2,
        event_id=None,
        dynamics_command=None,
    )

    assert node.ball_approach_alignment_pending is True
    assert node.ball_post_motion_dwell_until is None
    assert node.latest_info["ball"] is None
    assert not node.general_motion_gate.has_required_fresh_vision()


def test_legacy_ball_turn_ack_keeps_dwell_but_next_plan_uses_moving_recover(
    monkeypatch,
):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    node.BALL_POST_MOTION_DWELL_SEC = 3.0
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published(
        "BALL_APPROACH_TURN_LEFT_1",
        command_id=3,
    )
    node.active_general_source = "ball"
    monkeypatch.setattr(time, "monotonic", lambda: 10.0)

    send_status(
        node,
        status="RUNNING",
        action="BALL_APPROACH_TURN_LEFT_1",
        command_id=3,
        event_id=None,
        dynamics_command=None,
    )
    send_status(
        node,
        status="SUCCEEDED",
        action="BALL_APPROACH_TURN_LEFT_1",
        command_id=3,
        event_id=None,
        dynamics_command=None,
    )

    assert node.ball_approach_alignment_pending is False
    assert node.ball_post_motion_dwell_until == pytest.approx(13.0)
    decision = select_decision(
        node,
        ball=ball_info_for_node(
            distance_m=0.9,
            steering_angle_deg=-26.0,
        ),
    )
    assert decision.action == "BALL_APPROACH_RECOVER_LEFT_4"


@pytest.mark.parametrize("angle,recover_action", [
    (-30.0, "BALL_APPROACH_RECOVER_LEFT_4"),
    (26.0, "BALL_APPROACH_RECOVER_RIGHT_4"),
])
def test_ball_recover_rechecks_new_angle_without_dwell_or_forced_forward(
    monkeypatch, angle, recover_action,
):
    from mission_control.motion_decision_planner import MotionDecisionPlanner

    class BallApproachPublishNode(ReadinessPublishNode):
        SOURCES = MotionDecisionNode.SOURCES
        BALL_POST_MOTION_DWELL_SEC = MotionDecisionNode.BALL_POST_MOTION_DWELL_SEC
        _fresh_observations = MotionDecisionNode._fresh_observations
        _select_mission_decision = MotionDecisionNode._select_mission_decision

    node = BallApproachPublishNode(general_decision())
    node.planner = MotionDecisionPlanner()
    node.timeouts = {source: 0.5 for source in node.SOURCES}
    assert node.phase_manager.set_phase("BALL_APPROACH")
    clock = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    ball_callback = MotionDecisionNode._info_callback(node, "ball")

    def receive_ball(steering, distance=0.9):
        ball_callback(String(data=json.dumps(ball_info_for_node(
            distance_m=distance, depth_m=distance, steering_angle_deg=steering,
        ))))

    def complete_last_motion():
        payload = json.loads(node.publisher.messages[-1].data)
        for status in ("RUNNING", "SUCCEEDED"):
            send_status(
                node, status=status, action=payload["action"],
                command_id=payload["command_id"], event_id=None,
                dynamics_command=None,
            )

    # The same large error needs another recover, not a forced straight step.
    for steering, expected in ((angle, recover_action), (angle, recover_action),
                               (-8.0, "STRAIGHT")):
        clock[0] += 0.1
        receive_ball(steering)
        MotionDecisionNode._publish_decision(node)
        assert json.loads(node.publisher.messages[-1].data)["action"] == expected
        assert node.general_motion_gate.locked
        count = len(node.publisher.messages)
        receive_ball(steering)
        MotionDecisionNode._publish_decision(node)
        assert len(node.publisher.messages) == count
        complete_last_motion()
        assert node.ball_approach_alignment_pending is True
        assert node.ball_post_motion_dwell_until is None
        assert node.latest_info["ball"] is None
        assert not node.general_motion_gate.has_required_fresh_vision()
        MotionDecisionNode._publish_decision(node)
        assert all(
            json.loads(message.data)["valid"] is False
            for message in node.publisher.messages[count:]
        )

    clock[0] += 0.1
    receive_ball(angle, distance=0.55)
    MotionDecisionNode._publish_decision(node)
    assert json.loads(node.publisher.messages[-1].data)["action"] == "PICKUP_NOW"


def test_confirmed_ball_loss_during_approach_selects_last_side_turn(
    monkeypatch,
):
    node = FreshMockInputNode()
    node.BALL_POST_MOTION_DWELL_SEC = 3.0
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("STRAIGHT_3", command_id=2)
    node.active_general_source = "ball"
    node.latest_info["line"] = {
        "detected": True,
        "filtered_lateral_offset_norm": -0.24,
    }
    monkeypatch.setattr(time, "monotonic", lambda: 10.0)

    receive = MotionDecisionNode._info_callback(node, "ball")
    receive(String(data=json.dumps(ball_info_for_node(
        distance_m=0.9, steering_angle_deg=31.0, offset_x_px=100,
    ))))
    missing = receive_confirmed_ball_loss(node, monkeypatch)

    assert node.ball_lost_during_motion_pending is True
    send_status(
        node,
        status="RUNNING",
        action="STRAIGHT_3",
        command_id=2,
        event_id=None,
        dynamics_command=None,
    )
    send_status(
        node,
        status="SUCCEEDED",
        action="STRAIGHT_3",
        command_id=2,
        event_id=None,
        dynamics_command=None,
    )

    assert node.ball_post_motion_dwell_until is None
    decision = select_decision(node, ball=missing)
    assert decision.action == "BALL_APPROACH_TURN_RIGHT_5"
    assert decision.valid is True


def test_raw_ball_confirmation_does_not_count_as_motion_time_loss():
    node = FreshMockInputNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("STRAIGHT_3", command_id=2)
    node.active_general_source = "ball"
    MotionDecisionNode._track_ball_loss_during_motion(
        node,
        ball_info_for_node(distance_m=0.9),
    )

    MotionDecisionNode._track_ball_loss_during_motion(
        node,
        {"detected": False, "raw_detected": True},
    )

    assert node.ball_lost_during_motion_pending is False


def test_pickup_positioning_loss_latches_without_general_recovery(monkeypatch):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_positioning_motion_running = True
    node.pickup_positioning_motion_id = "ball_camera_down_forward_2"

    MotionDecisionNode._track_pickup_positioning_ball_loss(
        node,
        ball_info_for_node(distance_m=0.4),
    )
    missing = receive_confirmed_ball_loss(node, monkeypatch)

    assert node.pickup_positioning_ball_lost_pending is True
    decision = select_decision(node, ball=missing)
    assert decision.action == node.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    assert decision.sdk_motion_requested is False
    assert decision.source_command["pickup_positioning_motion_id"] == (
        "ball_camera_down_forward_2"
    )
    assert not decision.action.startswith("BALL_APPROACH_TURN_")


@pytest.mark.parametrize(
    ("steering_angle_deg", "expected_action"),
    [
        (-30.0, "BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_2"),
        (26.0, "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_2"),
    ],
)
def test_pickup_loss_reacquisition_reuses_camera_down_heading_planner(
    steering_angle_deg,
    expected_action,
):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_initial_align_waiting = True
    node.pickup_positioning_ball_lost_pending = True
    node.pickup_positioning_loss_latch_sent = True

    missing = select_decision(node, ball={"detected": False})
    assert missing.action == "WAIT"
    assert missing.reason == (
        "ball_loss_confirmation_pending"
    )

    reacquired = select_decision(
        node,
        ball=ball_info_for_node(
            distance_m=0.4, depth_m=0.4, bottom_distance_px=300,
            steering_angle_deg=steering_angle_deg,
            offset_x_norm=0.2,
        ),
    )
    assert reacquired.action == expected_action
    assert reacquired.sdk_motion_requested is True


@pytest.mark.parametrize(
    ("distance_m", "expected_motion"),
    [
        (0.670, "STRAIGHT"),
        (0.571, "STRAIGHT"),
        (0.570, "STRAIGHT"),
        (0.551, "STRAIGHT"),
        (0.550, "STRAIGHT_0"),
        (0.549, "STRAIGHT_0"),
        (0.470, "STRAIGHT_0"),
    ],
)
def test_pickup_loss_reacquisition_uses_metric_distance_threshold(
    distance_m,
    expected_motion,
):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_initial_align_waiting = True
    node.pickup_positioning_ball_lost_pending = True
    node.pickup_positioning_loss_latch_sent = True

    decision = select_decision(
        node,
        ball=ball_info_for_node(
            distance_m=distance_m,
            steering_angle_deg=0.0,
        ),
    )

    assert decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert decision.source_command["pickup_approach_motion"] == expected_motion


@pytest.mark.parametrize("bottom_distance,expected_action", [
    (100, "BALL_PICKUP_INITIAL_CRAB_RIGHT"),
    (120, "BALL_PICKUP_INITIAL_CRAB_RIGHT"),
    (121, "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_3"),
])
def test_pickup_initial_offset_selects_action_by_close_distance(
    bottom_distance, expected_action,
):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_initial_align_waiting = True

    decision = select_decision(
        node,
        ball=ball_info_for_node(
            offset_x_px=71, offset_x_norm=0.2, depth_m=0.5, distance_m=0.5,
            bottom_distance_px=bottom_distance,
            steering_angle_deg=35.0,
        ),
    )

    assert decision.action == expected_action
    assert decision.action in node.PICKUP_INITIAL_ALIGN_ACTIONS


def test_pickup_positioning_loss_does_not_latch_in_fixed_or_line_motion():
    node = FreshMockInputNode()
    node.pickup_positioning_motion_running = True
    node.pickup_positioning_ball_seen_during_motion = True
    node.pickup_fixed_sequence_started = True

    MotionDecisionNode._track_pickup_positioning_ball_loss(
        node,
        {"detected": False, "raw_detected": False},
    )

    assert node.pickup_positioning_ball_lost_pending is False
    assert node.ball_lost_during_motion_pending is False


def test_ball_settle_expiry_discards_frames_received_during_settle(monkeypatch):
    node = ReadinessPublishNode(general_decision())
    node.ball_post_motion_dwell_until = 13.0
    node.latest_info["ball"] = ball_info_for_node()
    node.latest_time["ball"] = 12.5
    node.general_motion_gate.vision_generation = 5
    monkeypatch.setattr(time, "monotonic", lambda: 13.0)

    MotionDecisionNode._publish_decision(node)

    assert node.publisher.messages == []
    assert node.latest_info["ball"] is None
    assert node.latest_time["ball"] is None
    assert node.general_motion_gate.required_vision_generation == 6


@pytest.mark.parametrize('action', [
    'GOAL_CAMERA_90_FORWARD',
    *(f'GOAL_CAMERA90_FINE_FORWARD_{count}' for count in range(1, 5)),
    'GOAL_CAMERA90_TURN_RIGHT_2', 'GOAL_CAMERA90_TURN_LEFT_1',
    'GOAL_CAMERA90_CRAB_LEFT', 'GOAL_CAMERA90_CRAB_RIGHT',
    'GOAL_CAMERA90_BACKWARD_1',
])
def test_goal_motion_success_dwells_only_after_turns(monkeypatch, action):
    node = FreshMockInputNode()
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published(action, command_id=31)
    node.active_general_source = 'goal'
    monkeypatch.setattr(time, 'monotonic', lambda: 10.0)
    send_status(node, status='RUNNING', action=action, command_id=31,
                event_id=None, dynamics_command=None)
    send_status(node, status='SUCCEEDED', action=action, command_id=31,
                event_id=None, dynamics_command=None)
    assert node.goal_post_motion_dwell_until == (
        11.0 if action.startswith('GOAL_CAMERA90_TURN_') else None
    )
    if not action.startswith('GOAL_CAMERA90_TURN_'):
        assert node.latest_info['goal'] is None
        assert node.latest_time['goal'] is None
        assert not node.general_motion_gate.has_required_fresh_vision()
    monkeypatch.setattr(time, 'monotonic', lambda: 11.0)
    send_status(node, status='SUCCEEDED', action=action, command_id=31,
                event_id=None, dynamics_command=None)
    assert node.goal_post_motion_dwell_until == (
        11.0 if action.startswith('GOAL_CAMERA90_TURN_') else None
    )



def test_goal_dwell_blocks_commands_then_discards_old_goal(monkeypatch):
    node = ReadinessPublishNode(general_decision())
    node.goal_post_motion_dwell_until = 13.0
    node.latest_info['goal'] = {'detected': True}
    node.latest_time['goal'] = 12.9
    node.general_motion_gate.vision_generation = 5
    monkeypatch.setattr(time, 'monotonic', lambda: 12.99)
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages == []
    assert node.goal_post_motion_dwell_until == 13.0
    monkeypatch.setattr(time, 'monotonic', lambda: 13.0)
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages == []
    assert node.latest_info['goal'] is None
    assert node.latest_time['goal'] is None
    assert node.general_motion_gate.required_vision_generation == 6


def test_post_ball_transition_requires_fresh_goal_without_an_extra_dwell(monkeypatch):
    node = FakeDecisionNode(mission_phase='POST_BALL_GOAL_TRANSITION')
    node.latest_info['goal'] = {'detected': True}
    node.latest_time['goal'] = 9.9
    node.general_motion_gate.vision_generation = 5
    node.phase_manager.start_special_action('POST_BALL_GOAL_TRANSITION', 32)
    monkeypatch.setattr(time, 'monotonic', lambda: 10.0)
    send_status(node, status='RUNNING', action='POST_BALL_GOAL_TRANSITION',
                command_id=32, event_id=None, dynamics_command=None)
    send_status(node, status='SUCCEEDED', action='POST_BALL_GOAL_TRANSITION',
                command_id=32, event_id=None, dynamics_command=None)
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.goal_post_motion_dwell_until is None
    assert node.latest_info['goal'] is None
    assert node.latest_time['goal'] is None
    assert node.general_motion_gate.required_vision_generation == 6


def test_550mm_entry_during_general_settle_preserves_motion_dwell():
    node = FreshMockInputNode()
    node.ball_post_motion_dwell_until = 20.0
    node.ball_approach_alignment_pending = True

    MotionDecisionNode._latch_ball_pickup_entry(
        node,
        ball_info_for_node(distance_m=0.550),
    )

    assert node.ball_pickup_entry_pending is True
    assert node.ball_post_motion_dwell_until == 20.0
    assert node.ball_approach_alignment_pending is False


def test_general_ball_alignment_pending_uses_fresh_angle_then_distance():
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    node.ball_approach_alignment_pending = True

    turn = select_decision(
        node,
        ball=ball_info_for_node(
            distance_m=0.9,
            steering_angle_deg=26.0,
        ),
    )
    assert turn.action == "BALL_APPROACH_RECOVER_RIGHT_4"

    node.ball_approach_alignment_pending = True
    straight = select_decision(
        node,
        ball=ball_info_for_node(
            distance_m=0.9,
            steering_angle_deg=2.0,
        ),
    )
    assert straight.action == "STRAIGHT"
    assert node.ball_approach_alignment_pending is False


@pytest.mark.parametrize(
    ('search_phase', 'source', 'payload', 'approach_phase'),
    [
        (
            'BALL_SEARCH',
            'ball',
            {
                'detected': True,
                'depth_valid': True,
                'depth_m': 0.9,
                'ground_distance_m': 0.9,
                'distance_m': 0.9,
            },
            'BALL_APPROACH',
        ),
        (
            'GOAL_SEARCH',
            'goal',
            {
                'detected': True,
                'depth_valid': True,
                'depth_m': 0.5,
                'ground_distance_m': 0.5,
            },
            'GOAL_APPROACH',
        ),
    ],
)
def test_search_phase_advances_on_fresh_controllable_target(
    search_phase,
    source,
    payload,
    approach_phase,
):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase(search_phase)

    decision = select_decision(node, **{source: payload})

    assert node.mission_phase == approach_phase
    assert decision.phase == approach_phase


@pytest.mark.parametrize('search_phase', ['BALL_SEARCH', 'GOAL_SEARCH'])
def test_search_phase_does_not_advance_without_fresh_target(search_phase):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase(search_phase)

    select_decision(node)

    assert node.mission_phase == search_phase


@pytest.mark.parametrize(
    ("scenario", "expected_action"),
    [
        ("straight", "STRAIGHT"),
        ("turn_left", "RECOVER_LEFT_TURN_LEFT_4"),
        ("turn_right", "RECOVER_RIGHT_TURN_RIGHT_4"),
    ],
)
def test_mock_line_input_is_fresh_and_produces_action(
    scenario, expected_action
):
    node = FreshMockInputNode()
    topic, payload = build_mock_vision_input(scenario)
    assert topic == "/vision/line_info"

    message = String()
    message.data = json.dumps(payload)
    MotionDecisionNode._info_callback(node, "line")(message)

    received_at = node.latest_time["line"]
    assert received_at is not None
    observations, ages = MotionDecisionNode._fresh_observations(
        node,
        now=received_at + 0.1,
    )
    assert observations["line"] == payload
    assert ages["line"] == pytest.approx(0.1)

    decision = MotionDecisionNode._select_mission_decision(
        node,
        observations,
        0.1,
    )
    assert decision.reason != "no_fresh_detected_target"
    assert decision.action == expected_action
    assert decision.valid is True


@pytest.mark.parametrize(
    (
        'initial_phase',
        'action',
        'dynamics_command',
        'expected_phase',
    ),
    [
        ('AUTO', 'PICKUP_NOW', 9, 'POST_BALL_LINE_ALIGN'),
        ('GOAL_APPROACH', 'SHOT', 17, 'POST_SHOT_TURN'),
        ('HURDLE_APPROACH', 'GO', 14, 'LINE_TRACK'),
    ],
)
def test_special_motion_success_advances_phase(
    initial_phase,
    action,
    dynamics_command,
    expected_phase,
):
    """Advance to the configured phase after a matching success."""
    node = FakeDecisionNode(initial_phase)
    arm_special_command(node, action, 100, 10)

    send_status(
        node,
        status='RUNNING',
        action=action,
        command_id=100,
        event_id=10,
        dynamics_command=dynamics_command,
    )

    assert node.special_motion_running is True
    assert node.active_special_action == action
    assert node.mission_phase == initial_phase

    send_status(
        node,
        status='SUCCEEDED',
        action=action,
        command_id=100,
        event_id=10,
        dynamics_command=dynamics_command,
    )

    assert node.special_motion_running is False
    assert node.mission_phase == expected_phase
    assert node.terminal_latch is None

    assert node.active_special_action is None
    assert node.active_special_command_id is None
    assert node.active_special_event_id is None
    assert node.active_special_dynamics_command is None


def test_ignored_status_keeps_special_motion_locked():
    """Keep the active phase and lock after an ignored status."""
    node = FakeDecisionNode('AUTO')
    arm_special_command(node, 'PICKUP_NOW', 200, 20)

    send_status(
        node,
        status='RUNNING',
        action='PICKUP_NOW',
        command_id=200,
        event_id=20,
        dynamics_command=9,
    )

    send_status(
        node,
        status='IGNORED',
        action='PICKUP_NOW',
        command_id=200,
        event_id=20,
        dynamics_command=9,
    )

    assert node.special_motion_running is True
    assert node.active_special_action == 'PICKUP_NOW'
    assert node.mission_phase == 'AUTO'
    assert node.terminal_latch == ('test', 'terminal')


def test_stale_special_terminal_with_other_command_id_is_ignored():
    node = FakeDecisionNode('AUTO')
    arm_special_command(node, 'PICKUP_NOW', 200, 20)
    node.special_motion_running = True

    send_status(
        node,
        status='SUCCEEDED',
        action='PICKUP_NOW',
        command_id=199,
        event_id=None,
        dynamics_command=9,
    )

    assert node.mission_phase == 'AUTO'
    assert node.pickups_completed == 0
    assert node.special_motion_running is True
    assert node.active_special_action == 'PICKUP_NOW'
    assert node.active_special_command_id == 200


def test_special_status_without_command_id_is_ignored():
    node = FakeDecisionNode('AUTO')
    arm_special_command(node, 'PICKUP_NOW', 200, 20)

    send_status(
        node,
        status='RUNNING',
        action='PICKUP_NOW',
        command_id=None,
        event_id=None,
        dynamics_command=9,
    )

    assert node.special_motion_running is False
    assert node.active_special_command_id == 200


def test_special_status_with_matching_id_but_wrong_action_is_ignored():
    node = FakeDecisionNode('AUTO')
    arm_special_command(node, 'PICKUP_NOW', 200, 20)
    node.special_motion_running = True

    send_status(
        node,
        status='SUCCEEDED',
        action='SHOT',
        command_id=200,
        event_id=None,
        dynamics_command=17,
    )

    assert node.mission_phase == 'AUTO'
    assert node.pickups_completed == 0
    assert node.shots_completed == 0
    assert node.special_motion_running is True
    assert node.active_special_action == 'PICKUP_NOW'
    assert node.active_special_command_id == 200


def test_matching_pickup_status_completes_once_and_clears_active_command():
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 200, 20)

    for status in ('RUNNING', 'SUCCEEDED', 'SUCCEEDED'):
        send_status(
            node,
            status=status,
            action='PICKUP_NOW',
            command_id=200,
            event_id=20,
            dynamics_command=9,
        )

    assert node.pickups_completed == 1
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'
    assert node.special_motion_running is False
    assert node.active_special_action is None
    assert node.active_special_command_id is None


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_failed_or_timed_out_shot_returns_to_goal_approach(status):
    """Release the lock without completing the failed goal section."""
    node = FakeDecisionNode('GOAL_APPROACH')
    arm_special_command(node, 'SHOT', 300, 30)

    send_status(
        node,
        status='RUNNING',
        action='SHOT',
        command_id=300,
        event_id=30,
        dynamics_command=17,
    )

    send_status(
        node,
        status=status,
        action='SHOT',
        command_id=300,
        event_id=30,
        dynamics_command=17,
    )

    assert node.special_motion_running is False
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.terminal_latch is None
    assert node.pickups_completed == 0
    assert node.shots_completed == 0


@pytest.mark.parametrize(
    ('initial_phase', 'action', 'expected_phase'),
    [
        ('BALL_APPROACH', 'PICKUP_NOW', 'BALL_APPROACH'),
        ('GOAL_APPROACH', 'SHOT', 'GOAL_APPROACH'),
        ('HURDLE_APPROACH', 'GO', 'HURDLE_APPROACH'),
        ('GOAL_APPROACH', 'GO', 'HURDLE_APPROACH'),
    ],
)
def test_matching_cancelled_after_running_releases_special_lock(
    initial_phase,
    action,
    expected_phase,
):
    node = FakeDecisionNode(initial_phase)
    arm_special_command(node, action, 350, 35)

    for status in ('RUNNING', 'CANCELLED'):
        send_status(
            node,
            status=status,
            action=action,
            command_id=350,
            event_id=35,
            dynamics_command=None,
        )

    assert node.special_motion_running is False
    assert node.active_special_action is None
    assert node.active_special_command_id is None
    assert node.active_special_event_id is None
    assert node.mission_phase == expected_phase
    assert node.pickups_completed == 0
    assert node.shots_completed == 0
    assert node.terminal_latch is None


def test_cancelled_before_running_keeps_special_lock():
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 350, 35)

    send_status(
        node,
        status='CANCELLED',
        action='PICKUP_NOW',
        command_id=350,
        event_id=35,
        dynamics_command=None,
    )

    assert node.active_special_action == 'PICKUP_NOW'
    assert node.active_special_command_id == 350
    assert node.active_special_event_id == 35
    assert node.special_motion_running is False
    assert node.mission_phase == 'BALL_APPROACH'


@pytest.mark.parametrize(
    ('action', 'command_id', 'event_id'),
    [
        ('SHOT', 350, 35),
        ('PICKUP_NOW', 349, 35),
        ('PICKUP_NOW', 350, 34),
    ],
)
def test_wrong_or_stale_cancelled_keeps_special_lock(
    action,
    command_id,
    event_id,
):
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 350, 35)
    send_status(
        node,
        status='RUNNING',
        action='PICKUP_NOW',
        command_id=350,
        event_id=35,
        dynamics_command=None,
    )

    send_status(
        node,
        status='CANCELLED',
        action=action,
        command_id=command_id,
        event_id=event_id,
        dynamics_command=None,
    )

    assert node.active_special_action == 'PICKUP_NOW'
    assert node.active_special_command_id == 350
    assert node.active_special_event_id == 35
    assert node.special_motion_running is True
    assert node.mission_phase == 'BALL_APPROACH'


def test_duplicate_and_stale_cancelled_do_not_end_new_special_command():
    node = FakeDecisionNode('BALL_APPROACH', required_ball_sections=2)
    arm_special_command(node, 'PICKUP_NOW', 350, 35)
    for status in ('RUNNING', 'CANCELLED', 'CANCELLED'):
        send_status(
            node,
            status=status,
            action='PICKUP_NOW',
            command_id=350,
            event_id=35,
            dynamics_command=None,
        )

    assert node.ball_sections_processed == 0
    assert node.mission_phase == 'BALL_APPROACH'

    arm_special_command(node, 'GO', 351, 36)
    send_status(
        node,
        status='RUNNING',
        action='GO',
        command_id=351,
        event_id=36,
        dynamics_command=None,
    )
    send_status(
        node,
        status='CANCELLED',
        action='PICKUP_NOW',
        command_id=350,
        event_id=35,
        dynamics_command=None,
    )

    assert node.active_special_action == 'GO'
    assert node.active_special_command_id == 351
    assert node.active_special_event_id == 36
    assert node.special_motion_running is True


@pytest.mark.parametrize(
    ('initial_phase', 'action', 'expected_phase'),
    [
        ('BALL_APPROACH', 'PICKUP_NOW', 'BALL_APPROACH'),
        ('GOAL_APPROACH', 'SHOT', 'GOAL_APPROACH'),
        ('HURDLE_APPROACH', 'GO', 'HURDLE_APPROACH'),
        ('GOAL_APPROACH', 'GO', 'HURDLE_APPROACH'),
    ],
)
def test_matching_rejected_without_running_releases_special_lock(
    initial_phase,
    action,
    expected_phase,
):
    node = FakeDecisionNode(initial_phase)
    arm_special_command(node, action, 400, 40)

    send_status(
        node,
        status='REJECTED',
        action=action,
        command_id=400,
        event_id=40,
        dynamics_command=None,
        error_code='HARDWARE_NOT_READY',
        message='SDK backend is not ready',
    )

    assert node.special_motion_running is False
    assert node.active_special_action is None
    assert node.active_special_command_id is None
    assert node.active_special_event_id is None
    assert node.mission_phase == expected_phase
    assert node.pickups_completed == 0
    assert node.shots_completed == 0
    assert node.terminal_latch is None


@pytest.mark.parametrize(
    ('action', 'command_id', 'event_id'),
    [
        ('SHOT', 400, 40),
        ('PICKUP_NOW', 399, 40),
        ('PICKUP_NOW', 400, 39),
        ('PICKUP_NOW', 400, None),
    ],
)
def test_wrong_or_stale_rejected_keeps_special_lock(
    action,
    command_id,
    event_id,
):
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 400, 40)

    send_status(
        node,
        status='REJECTED',
        action=action,
        command_id=command_id,
        event_id=event_id,
        dynamics_command=None,
        error_code='HARDWARE_NOT_READY',
    )

    assert node.active_special_action == 'PICKUP_NOW'
    assert node.active_special_command_id == 400
    assert node.active_special_event_id == 40
    assert node.mission_phase == 'BALL_APPROACH'
    assert node.terminal_latch == ('test', 'terminal')


def test_duplicate_rejected_does_not_change_recovered_phase():
    node = FakeDecisionNode('BALL_APPROACH', required_ball_sections=2)
    arm_special_command(node, 'PICKUP_NOW', 400, 40)
    status = {
        'status': 'REJECTED',
        'action': 'PICKUP_NOW',
        'command_id': 400,
        'event_id': 40,
        'dynamics_command': None,
        'error_code': 'HARDWARE_NOT_READY',
    }

    send_status(node, **status)
    send_status(node, **status)

    assert node.mission_phase == 'BALL_APPROACH'
    assert node.ball_sections_processed == 0
    assert node.active_special_action is None
    assert node.active_special_event_id is None


@pytest.mark.parametrize(
    ('source', 'action', 'initial_phase', 'expected_phase'),
    [
        ('hurdle', 'GO', 'HURDLE_APPROACH', 'LINE_TRACK'),
        (
            'ball',
            'PICKUP_NOW',
            'BALL_APPROACH',
            'POST_BALL_LINE_ALIGN',
        ),
        ('goal', 'SHOT', 'GOAL_APPROACH', 'POST_SHOT_TURN'),
    ],
)
def test_terminal_action_rearms_only_after_target_disappears(
    source,
    action,
    initial_phase,
    expected_phase,
):
    """Suppress a successful action until its target is lost and reacquired."""
    node = FakeDecisionNode(initial_phase)
    decision = terminal_decision(source, action, initial_phase)

    first = MotionDecisionNode._suppress_duplicate_terminal_action(
        node,
        decision,
    )
    assert first.action == action

    # This is the state change made when the first SDK request is published.
    node.terminal_action_armed[source] = False
    arm_special_command(node, action, 400, 40)
    send_status(
        node,
        status='RUNNING',
        action=action,
        command_id=400,
        event_id=40,
        dynamics_command=14,
    )
    send_status(
        node,
        status='SUCCEEDED',
        action=action,
        command_id=400,
        event_id=40,
        dynamics_command=14,
    )
    assert node.mission_phase == expected_phase

    MotionDecisionNode._rearm_absent_terminal_targets(
        node,
        {source: {'detected': True}},
    )
    duplicate = MotionDecisionNode._suppress_duplicate_terminal_action(
        node,
        decision,
    )
    assert duplicate.action == 'WAIT'
    assert duplicate.requires_ack is False
    assert duplicate.sdk_motion_requested is False
    assert duplicate.reason == 'duplicate_terminal_action_suppressed'

    MotionDecisionNode._rearm_absent_terminal_targets(
        node,
        {source: {'detected': False}},
    )
    reacquired = MotionDecisionNode._suppress_duplicate_terminal_action(
        node,
        decision,
    )
    assert reacquired.action == action


def test_terminal_targets_rearm_independently_on_observation_timeout():
    """A stale hurdle observation must not re-arm ball or goal."""
    node = FakeDecisionNode()
    node.terminal_action_armed = {
        'ball': False,
        'goal': False,
        'hurdle': False,
    }

    MotionDecisionNode._rearm_absent_terminal_targets(
        node,
        {
            'ball': {'detected': True},
            'goal': {'detected': True},
            'hurdle': None,
        },
    )

    assert node.terminal_action_armed == {
        'ball': False,
        'goal': False,
        'hurdle': True,
    }


def test_pickup_success_increments_score_but_not_section():
    """A successful pickup still needs its following shot section."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=501)

    assert node.pickups_completed == 1
    assert node.shots_completed == 0
    assert node.ball_sections_processed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'


def test_completed_two_pickups_block_ball_phase_reentry():
    node = FakeDecisionNode(mission_phase='BALL_APPROACH')
    node.phase_manager.pickups_completed = node.required_pickups

    decision = MotionDecisionNode._select_mission_decision(
        node,
        {'ball': {'detected': True}, 'line': {'detected': True}},
        0.1,
    )

    assert decision.source == 'line'
    assert node.mission_phase == 'AUTO'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_pickup_failure_preserves_section_progress(status):
    """Return to ball approach without completing the failed section."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=502, status=status)

    assert node.pickups_completed == 0
    assert node.ball_sections_processed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == 'BALL_APPROACH'


def test_shot_success_increments_score_and_section():
    """Count both success score and section for a successful shot."""
    node = FakeDecisionNode('GOAL_APPROACH')
    complete_motion(node, 'SHOT', event_id=503)

    assert node.shots_completed == 1
    assert node.ball_sections_processed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'POST_SHOT_TURN'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_shot_failure_preserves_section_progress(status):
    """Return to goal approach without completing the failed section."""
    node = FakeDecisionNode('GOAL_APPROACH')
    complete_motion(node, 'SHOT', event_id=504, status=status)

    assert node.shots_completed == 0
    assert node.ball_sections_processed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == 'GOAL_APPROACH'


@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED'])
def test_go_terminal_status_does_not_change_section(status):
    """A hurdle action is independent from ball-section progress."""
    node = FakeDecisionNode('HURDLE_APPROACH')
    complete_motion(node, 'GO', event_id=505, status=status)

    assert node.pickups_completed == 0
    assert node.shots_completed == 0
    assert node.ball_sections_processed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == ('LINE_TRACK' if status == 'SUCCEEDED' else 'HURDLE_APPROACH')


def test_pickup_failure_does_not_complete_last_section():
    """Keep the last section incomplete after a failed pickup."""
    node = FakeDecisionNode()
    complete_motion(node, 'SHOT', event_id=506)
    complete_motion(node, 'PICKUP_NOW', event_id=507, status='FAILED')

    assert node.ball_sections_processed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'BALL_APPROACH'


def test_shot_failure_does_not_complete_last_section():
    """Keep the last section incomplete after a failed shot."""
    node = FakeDecisionNode()
    complete_motion(node, 'SHOT', event_id=508)
    complete_motion(node, 'SHOT', event_id=509, status='FAILED')

    assert node.shots_completed == 1
    assert node.ball_sections_processed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'GOAL_APPROACH'


def test_duplicate_terminal_status_does_not_increment_section_twice():
    """Ignore a retransmitted terminal result after its lock is released."""
    node = FakeDecisionNode('GOAL_APPROACH')
    complete_motion(node, 'SHOT', event_id=510, status='FAILED')

    send_status(
        node,
        status='FAILED',
        action='SHOT',
        command_id=510,
        event_id=510,
        dynamics_command=0,
    )

    assert node.ball_sections_processed == 0


def test_section_progress_is_capped_at_requirement():
    """Never report more processed sections than configured."""
    node = FakeDecisionNode(required_ball_sections=2)
    complete_motion(node, 'SHOT', event_id=511)
    complete_motion(node, 'SHOT', event_id=512)
    assert not node.phase_manager.start_special_action('SHOT', 513)

    assert node.ball_sections_processed == 2
    assert node.finish_enabled is True


def test_mission_progress_contains_exact_fields_and_values():
    """Expose success scores and course progress in command JSON shape."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=514)
    complete_motion(node, 'SHOT', event_id=515)

    progress = MotionDecisionNode._mission_progress(node)

    assert progress == {
        'ball_mode_active': False,
        'pickups_executed': 1,
        'shots_executed': 1,
        'hurdles_executed': 0,
        'pickups_completed': 1,
        'required_pickups': 2,
        'shots_completed': 1,
        'required_shots': 2,
        'hurdles_completed': 0,
        'required_hurdles': 2,
        'ball_sections_processed': 1,
        'required_ball_sections': 2,
        'finish_enabled': False,
        'mission_complete': False,
        'ball_1_grasp_result': 'UNKNOWN',
        'ball_2_grasp_result': 'UNKNOWN',
    }


def finish_info(**overrides):
    """Create one confirmed finish observation."""
    info = {
        'detected': True,
        'confidence': 0.95,
        'confirmed': True,
        'distance_m': 0.25,
    }
    info.update(overrides)
    return info


@pytest.mark.parametrize("finish_enabled", [False, True])
def test_finish_detection_never_requests_cross_finish(finish_enabled):
    """Treat WALK_TO_FINISH as line-only compatibility phase."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = finish_enabled
    decision = select_decision(node, finish=finish_info())

    assert decision.phase == 'WALK_TO_FINISH'
    assert decision.action == 'STRAIGHT'
    assert decision.source == 'line'
    assert decision.requires_ack is False


@pytest.mark.parametrize(
    'observation',
    [
        finish_info(confirmed=False),
        finish_info(confidence=0.69),
        None,
    ],
)
def test_unready_or_stale_finish_keeps_line_command(observation):
    """Continue line walking until a fresh confirmed finish is ready."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    decision = select_decision(node, finish=observation)

    assert decision.phase == 'WALK_TO_FINISH'
    assert decision.source == 'line'
    assert decision.action == 'STRAIGHT'
    assert decision.requires_ack is False


def test_finish_observation_timeout_removes_stale_input():
    """Exclude finish information older than its configured timeout."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    node.SOURCES = MotionDecisionNode.SOURCES
    node.latest_info = {
        source: None for source in node.SOURCES
    }
    node.latest_time = {
        source: None for source in node.SOURCES
    }
    node.timeouts = {
        source: 0.5 for source in node.SOURCES
    }
    node.latest_info['finish'] = finish_info()
    node.latest_time['finish'] = 1.0

    observations, _ages = MotionDecisionNode._fresh_observations(
        node,
        now=1.6,
    )
    decision = select_decision(
        node,
        finish=observations['finish'],
    )

    assert observations['finish'] is None
    assert decision.action == 'STRAIGHT'


def test_cross_finish_running_blocks_other_commands():
    """Use the existing special lock while finish crossing is running."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    arm_special_command(node, 'CROSS_FINISH', 601, 601)
    send_status(
        node,
        status='RUNNING',
        action='CROSS_FINISH',
        command_id=601,
        event_id=601,
        dynamics_command=0,
    )

    decision = select_decision(
        node,
        finish=finish_info(),
        ball={'detected': True},
        goal={'detected': True},
        hurdle={'detected': True},
    )

    assert decision.action == 'WAIT'
    assert decision.reason == 'mission_locked_waiting_for_motion_status'


def test_cross_finish_success_enters_finished_and_ignores_duplicate():
    """Complete once and ignore a retransmitted finish success."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    complete_motion(node, 'CROSS_FINISH', event_id=602)

    assert node.mission_complete is True
    assert node.mission_phase == 'FINISHED'

    send_status(
        node,
        status='SUCCEEDED',
        action='CROSS_FINISH',
        command_id=602,
        event_id=602,
        dynamics_command=0,
    )

    assert node.mission_complete is True
    assert node.mission_phase == 'FINISHED'


def test_finished_always_stops_even_with_special_targets():
    """Prioritize mission-complete STOP over every perception target."""
    node = FakeDecisionNode('FINISHED')
    node.mission_complete = True
    decision = select_decision(
        node,
        finish=finish_info(),
        ball={'detected': True},
        goal={'detected': True},
        hurdle={'detected': True},
    )

    assert decision.phase == 'FINISHED'
    assert decision.source == 'none'
    assert decision.action == 'STOP'
    assert decision.valid is True
    assert decision.reason == 'mission_complete_stop'
    assert decision.sdk_motion_requested is False
    assert decision.requires_ack is False


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_manual_cross_finish_failure_returns_to_line_compatibility(status):
    """Keep manual status compatibility without automatically retrying."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    node.terminal_action_armed['finish'] = False
    complete_motion(
        node,
        'CROSS_FINISH',
        event_id=603,
        status=status,
    )

    assert node.mission_complete is False
    assert node.mission_phase == 'WALK_TO_FINISH'
    assert node.terminal_action_armed['finish'] is True

    next_decision = select_decision(node, finish=finish_info())
    assert next_decision.action == 'STRAIGHT'
    assert next_decision.requires_ack is False


@pytest.mark.parametrize("fine", [False, True])
@pytest.mark.parametrize("offset,direction", [(-100, "LEFT"), (100, "RIGHT")])
def test_ball_loss_memory_updates_while_pickup_motion_is_locked(fine, offset, direction, monkeypatch):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_positioning_motion_running = True
    node.pickup_positioning_motion_id = "pickup_crab_right_0"
    receive = MotionDecisionNode._info_callback(node, "ball")
    receive(String(data=json.dumps(ball_info_for_node(distance_m=0.4, offset_x_px=offset))))
    missing = receive_confirmed_ball_loss(node, monkeypatch)
    assert node.planner.last_ball_turn_direction == direction
    assert select_decision(node, ball=missing).action == node.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    node.pickup_positioning_loss_latch_sent = True
    assert select_decision(node, ball=missing).action == "WAIT"
    node.pickup_positioning_motion_running = False
    node.pickup_initial_align_waiting = not fine
    node.pickup_fine_align_waiting = fine
    count = 2 if direction == "LEFT" else 5
    expected = (f"BALL_PICKUP_FINE_SEARCH_{direction}" if fine
                else f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{count}")
    decision = select_decision(node, ball=missing)
    assert decision.action == expected
    assert decision.sdk_motion_requested is True


@pytest.mark.parametrize("fine", [False, True])
@pytest.mark.parametrize("raw_only", [False, True])
def test_close_ball_seen_during_pickup_motion_backs_up_after_loss(fine, raw_only, monkeypatch):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_positioning_motion_running = True
    node.pickup_positioning_motion_id = "pickup_fine_forward_0"
    receive = MotionDecisionNode._info_callback(node, "ball")
    receive(String(data=json.dumps(ball_info_for_node(distance_m=0.4, offset_x_px=100))))
    receive(String(data=json.dumps(ball_info_for_node(
        distance_m=0.4, offset_x_px=100, bottom_distance_px=100,
        detected=not raw_only, raw_detected=True,
    ))))
    missing = receive_confirmed_ball_loss(node, monkeypatch)
    assert node.planner.pickup_close_alignment_active
    node.pickup_positioning_motion_running = False
    node.pickup_positioning_loss_latch_sent = True
    node.pickup_initial_align_waiting = not fine
    node.pickup_fine_align_waiting = fine
    decision = select_decision(node, ball=missing)
    stage = "FINE" if fine else "INITIAL"
    assert decision.action == f"BALL_PICKUP_{stage}_SEARCH_BACKWARD"
    assert decision.reason == "close_ball_lost_backward_reacquisition"
    assert decision.sdk_motion_requested


@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED", "TIMEOUT", "CANCELLED"])
def test_pickup_terminal_status_clears_close_ball_latch(status):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    node.planner.pickup_close_alignment_active = True
    node.planner.pickup_fine_approach_complete = True
    node.planner.pickup_last_visible_bottom_distance_px = 34.0
    complete_motion(node, "PICKUP_NOW", event_id=10, status=status)
    assert not node.planner.pickup_close_alignment_active
    assert not node.planner.pickup_fine_approach_complete
    assert node.planner.pickup_last_visible_bottom_distance_px is None


@pytest.mark.parametrize("was_close,bottom_distance,expected", [
    (True, None, False),
    (True, 101, False),
    (False, 100, True),
])
def test_new_pickup_resets_close_latch_and_uses_current_observation(
    was_close, bottom_distance, expected,
):
    node = ReadinessPublishNode(
        terminal_decision("ball", "PICKUP_NOW", "BALL_APPROACH")
    )
    node.planner = FreshMockInputNode().planner
    node.planner.pickup_close_alignment_active = was_close
    node.planner.pickup_fine_approach_complete = True
    ball = (
        ball_info_for_node(bottom_distance_px=bottom_distance)
        if bottom_distance is not None else None
    )
    node._fresh_observations = lambda _now: ({"ball": ball}, {})

    MotionDecisionNode._publish_decision(node)

    assert node.active_special_action == "PICKUP_NOW"
    assert node.planner.pickup_close_alignment_active is expected
    assert not node.planner.pickup_fine_approach_complete


def test_raw_ball_crossing_image_center_updates_side_during_active_left_turn():
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published("BALL_APPROACH_TURN_LEFT_1", command_id=3)
    node.active_general_source = "ball"
    receive = MotionDecisionNode._info_callback(node, "ball")
    receive(String(data=json.dumps(ball_info_for_node(camera_center_offset_x_px=-300))))
    assert node.planner.last_ball_turn_direction == "LEFT"
    receive(String(data=json.dumps({
        "detected": False, "raw_detected": True, "confidence": 0.9,
        "camera_center_offset_x_px": 500, "depth_valid": False,
    })))
    receive(String(data=json.dumps({"detected": False, "raw_detected": False})))
    assert node.general_motion_gate.locked is True
    assert node.planner.last_ball_turn_direction == "RIGHT"
    assert node.planner.plan_lost_ball_approach_alignment().action == "WAIT"


@pytest.mark.parametrize("fixed,completed,fresh,raw,detected,expected", [
    (False, 0, True, False, False, True),
    (True, 0, True, False, False, False),
    (False, 2, True, False, False, False),
    (False, 0, False, False, False, False),
    (False, 0, True, True, False, False),
    (False, 0, True, True, True, False),
])
def test_ball_lost_debug_uses_fresh_loss_and_excludes_fixed_grasp(
    fixed, completed, fresh, raw, detected, expected,
):
    node = FreshMockInputNode()
    node.planner._update_ball_tracking(ball_info_for_node(offset_x_px=-100), 0.0)
    node.latest_info["ball"] = {"detected": detected, "raw_detected": raw,
                                "ball_loss_confirmed": True}
    node.latest_time["ball"] = time.monotonic() - (0.0 if fresh else 1.0)
    node.pickup_fixed_sequence_started = fixed
    node.phase_manager.pickups_completed = completed
    node.last_candidate_decision = None
    node.last_selected_decision = None
    node.executor_active = False
    node.decision_debug_publisher = ReadinessPublisher()
    MotionDecisionNode._publish_decision_debug(node)
    payload = json.loads(node.decision_debug_publisher.messages[-1].data)
    assert payload["ball_tracking"]["lost"] is expected
    assert payload["ball_tracking"]["last_direction"] == "LEFT"


def test_shot_waits_one_second_even_when_general_settle_is_disabled(monkeypatch):
    node = ReadinessPublishNode(terminal_decision('goal', 'SHOT', 'GOAL_APPROACH'))
    node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    assert node.SHOT_PRE_MOTION_SETTLE_SEC == 1.0
    node.pre_motion_settle_sec = 0.0
    set_grasp_result_for_first_ball(node, 'GRABBED')
    clock = [10.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    for stamp in (10.0, 10.999):
        clock[0] = stamp
        MotionDecisionNode._publish_decision(node)
        assert node.publisher.messages == []
        assert node.active_special_command_id is None
        assert node.command_id == 0
    clock[0] = 11.0
    MotionDecisionNode._publish_decision(node)
    assert json.loads(node.publisher.messages[-1].data)['action'] == 'SHOT'
    assert node.active_special_command_id == 1



def test_line_search_memory_updates_during_running_motion():
    node = FreshMockInputNode()
    node.active_general_source = 'line'
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published('STRAIGHT', command_id=71)
    callback = MotionDecisionNode._info_callback(node, 'line')
    for offset in (-0.8, 0.8):
        callback(String(data=json.dumps({
            'detected': True, 'lateral_offset_norm': offset,
            'detection_quality': 0.9, 'geometry_quality': 0.9,
        })))
    callback(String(data=json.dumps({'detected': False})))
    assert node.planner.last_line_seen_direction == 'RIGHT'
    assert node.general_motion_gate.locked


def test_line_lost_turn_waits_even_when_generic_settle_is_disabled():
    node = ReadinessPublishNode(general_decision("LINE_LOST_TURN_RIGHT"))
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.pre_motion_settle_sec = 0.0
    assert not node._pre_motion_settle_ready(node.decision, 10.0)
    assert not node._pre_motion_settle_ready(node.decision, 10.999)
    assert node._pre_motion_settle_ready(node.decision, 11.0)



@pytest.mark.parametrize("active_source", ["line", None])
def test_post_pickup_line_side_updates_during_turn_and_dwell(active_source):
    node = FreshMockInputNode()
    node.phase_manager.current_phase = "POST_BALL_LINE_ALIGN"
    node.active_general_source = active_source
    node.post_ball_line_dwell_until = time.monotonic() + 3.0
    callback = MotionDecisionNode._info_callback(node, "line")
    for x in (1000, 120):
        callback(String(data=json.dumps({
            "detected": True,
            "lateral_offset_norm": 0.8,
            "heading_error_deg": 18.77,
            "detection_quality": 0.9,
            "image_width": 1280,
            "center_points_px": [[x, 680]],
        })))
    callback(String(data=json.dumps({"detected": False})))
    assert node.planner.last_line_seen_direction == "LEFT"
    decision = node.planner.plan("POST_BALL_LINE_ALIGN", {
        "line": {"detected": False},
    }, 0.1)
    assert decision.action == "POST_BALL_LINE_TURN_RIGHT_5"


@pytest.mark.parametrize("fine", [False, True])
def test_close_ball_retreat_checkpoint_requires_new_vision_before_repeat(fine):
    node = FreshMockInputNode()
    assert node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    close = ball_info_for_node(distance_m=0.4, offset_x_px=141, bottom_distance_px=34)
    node.planner._update_ball_tracking(close, 0.0)
    node.planner._remember_pickup_close_ball(close)
    missing = {"detected": False, "raw_detected": False, "ball_loss_confirmed": True}
    node.latest_info["ball"] = missing
    node.latest_time["ball"] = time.monotonic()
    send_status(
        node, status="RUNNING", action="PICKUP_NOW", command_id=10,
        event_id=1, dynamics_command=None,
        motion_id="pickup_lost_ball_backward_1",
    )
    assert select_decision(node, ball=missing).action == "WAIT"
    marker = (MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER if fine
              else MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER)
    send_status(
        node, status="RUNNING", action="PICKUP_NOW", command_id=10,
        event_id=1, dynamics_command=None, motion_id=marker,
    )
    observations, _ = MotionDecisionNode._fresh_observations(node, time.monotonic())
    assert observations["ball"] is None
    assert select_decision(node, **observations).action == "WAIT"
    # A raw reappearance is enough to withhold another retreat pending confirmation.
    assert select_decision(node, ball={"detected": False, "raw_detected": True}).action == "WAIT"
    assert select_decision(node, ball=missing).action.endswith("SEARCH_BACKWARD")
    assert select_decision(node, ball=close).action.endswith("CRAB_RIGHT")
    assert not node.pickup_fixed_sequence_started


@pytest.mark.parametrize("stage", ["approach", "initial", "fine"])
def test_ball_loss_confirmation_gates_motion_and_banner_together(stage, monkeypatch):
    node = FreshMockInputNode()
    node.phase_manager.set_phase("BALL_APPROACH")
    if stage != "approach":
        arm_special_command(node, "PICKUP_NOW", 10, 1)
        node.pickup_initial_align_waiting = stage == "initial"
        node.pickup_fine_align_waiting = stage == "fine"
    node.decision_debug_publisher = ReadinessPublisher()
    node.last_candidate_decision = None
    node.last_selected_decision = None
    node.executor_active = False
    receive = MotionDecisionNode._info_callback(node, "ball")
    monkeypatch.setattr(time, "monotonic", lambda: 9.9)
    receive(String(data=json.dumps(ball_info_for_node(
        distance_m=0.9 if stage == "approach" else 0.4,
        offset_x_px=100, bottom_distance_px=34, rgb_stamp_ns=9_900_000_000,
    ))))
    for i, now in enumerate((10.0, 10.1, 10.2, 10.3, 10.49, 10.5)):
        monkeypatch.setattr(time, "monotonic", lambda now=now: now)
        receive(String(data=json.dumps({
            "detected": False, "raw_detected": False,
            "rgb_stamp_ns": round(now * 1e9),
        })))
        decision = select_decision(node, ball=node.latest_info["ball"])
        MotionDecisionNode._publish_decision_debug(node)
        debug = json.loads(node.decision_debug_publisher.messages[-1].data)
        assert debug["ball_tracking"]["lost"] is (i == 5)
        if i < 5:
            assert decision.action == "WAIT"
            assert decision.reason == "ball_loss_confirmation_pending"
        elif stage == "approach":
            assert decision.action == "BALL_APPROACH_TURN_RIGHT_5"
        else:
            assert decision.action == f"BALL_PICKUP_{stage.upper()}_SEARCH_BACKWARD"

    # A raw reappearance cancels loss immediately, without waiting for 12 hits.
    monkeypatch.setattr(time, "monotonic", lambda: 10.6)
    receive(String(data=json.dumps({
        "detected": False, "raw_detected": True, "rgb_stamp_ns": 10_600_000_000,
    })))
    assert not node.latest_info["ball"]["ball_loss_confirmed"]
    assert select_decision(node, ball=node.latest_info["ball"]).action == "WAIT"
    MotionDecisionNode._publish_decision_debug(node)
    assert not json.loads(node.decision_debug_publisher.messages[-1].data)["ball_tracking"]["lost"]


def test_duplicate_ball_frames_do_not_refresh_node_observation(monkeypatch):
    node = FreshMockInputNode()
    receive = MotionDecisionNode._info_callback(node, "ball")
    for now in (10.0, 10.1, 10.2, 10.3, 10.4, 10.6):
        monkeypatch.setattr(time, "monotonic", lambda now=now: now)
        receive(String(data=json.dumps({
            "detected": False, "raw_detected": False, "rgb_stamp_ns": 10_000_000_000,
        })))
    assert node.latest_time["ball"] == 10.0
    assert not node.latest_info["ball"]["ball_loss_confirmed"]
    observations, _ = MotionDecisionNode._fresh_observations(node, 10.6)
    assert observations["ball"] is None


@pytest.mark.parametrize("pickup", [False, True])
def test_short_loss_does_not_latch_or_interrupt_running_motion(pickup, monkeypatch):
    node = FreshMockInputNode()
    node.phase_manager.set_phase("BALL_APPROACH")
    if pickup:
        arm_special_command(node, "PICKUP_NOW", 10, 1)
        node.pickup_positioning_motion_running = True
        node.pickup_positioning_motion_id = "pickup_crab_right_0"
    else:
        node.general_motion_gate.on_new_vision_input()
        node.general_motion_gate.on_command_published("STRAIGHT_3", command_id=2)
        node.active_general_source = "ball"
    receive = MotionDecisionNode._info_callback(node, "ball")
    monkeypatch.setattr(time, "monotonic", lambda: 9.9)
    receive(String(data=json.dumps(ball_info_for_node(
        distance_m=0.4 if pickup else 0.9, rgb_stamp_ns=9_900_000_000,
    ))))
    for now in (10.0, 10.1, 10.2, 10.3, 10.4):
        monkeypatch.setattr(time, "monotonic", lambda now=now: now)
        receive(String(data=json.dumps({
            "detected": False, "raw_detected": False, "rgb_stamp_ns": round(now * 1e9),
        })))
        assert not node.ball_lost_during_motion_pending
        assert not node.pickup_positioning_ball_lost_pending
        assert select_decision(node, ball=node.latest_info["ball"]).action == "WAIT"
    assert node.active_special_action == "PICKUP_NOW" if pickup else node.general_motion_gate.locked


def test_ball_loss_pending_does_not_replace_fixed_grasp_sequence():
    node = FreshMockInputNode()
    node.phase_manager.set_phase("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 10, 1)
    node.pickup_fixed_sequence_started = True
    decision = select_decision(node, ball={
        "detected": False, "raw_detected": False, "ball_loss_confirmed": False,
    })
    assert decision.action == "WAIT"
    assert decision.reason == "mission_locked_waiting_for_motion_status"
    assert node.active_special_action == "PICKUP_NOW"


def test_line_recover_does_not_wait_when_turn_direction_changes():
    node = ReadinessPublishNode(general_decision("RECOVER_LEFT_TURN_LEFT_4"))
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 3.0
    assert node._pre_motion_settle_ready(node.decision, 10.0)
    node.decision = general_decision("RECOVER_LEFT_TURN_RIGHT_4")
    assert node._pre_motion_settle_ready(node.decision, 10.0)


@pytest.mark.parametrize("action", [
    "LEFT", "RIGHT", "LINE_LOST_TURN_LEFT", "LINE_LOST_TURN_RIGHT",
    "POST_BALL_LINE_TURN_LEFT_2", "POST_BALL_LINE_TURN_RIGHT_3",
    "POST_SHOT_LINE_TURN_LEFT_2", "POST_SHOT_LINE_TURN_RIGHT_3",
])
def test_all_line_stationary_turns_pause_one_second_before_publish(action, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    node = ReadinessPublishNode(general_decision(action))
    node._fresh_observations = lambda _now: ({"line": {"detected": not action.startswith("LINE_LOST_TURN_")}}, {})
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.pre_motion_settle_sec = 0.0
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages
    clock[0] = 10.999
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages
    clock[0] = 11.0
    MotionDecisionNode._publish_decision(node)
    expected = {"LINE_LOST_TURN_LEFT": "LINE_LOST_TURN_LEFT_3",
                "LINE_LOST_TURN_RIGHT": "LINE_LOST_TURN_RIGHT_5"}.get(action, action)
    assert json.loads(node.publisher.messages[-1].data)["action"] == expected



@pytest.mark.parametrize("replacement", ["STRAIGHT", "RECOVER_LEFT_TURN_LEFT_4"])
def test_line_turn_pause_cancels_without_delaying_new_walk(replacement, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    node = ReadinessPublishNode(general_decision("LINE_LOST_TURN_LEFT"))
    node._fresh_observations = lambda _now: ({"line": {"detected": False}}, {})
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages
    clock[0] = 11.0
    node.decision = general_decision(replacement)
    MotionDecisionNode._publish_decision(node)
    assert json.loads(node.publisher.messages[-1].data)["action"] == replacement
    assert node.pre_motion_settle_started_at is None


def test_line_turn_pause_starts_after_running_motion_ends():
    node = ReadinessPublishNode(general_decision("LINE_LOST_TURN_RIGHT"))
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.general_motion_gate.locked = True
    assert not node._pre_motion_settle_ready(node.decision, 10.0)
    assert node.pre_motion_settle_started_at is None
    node.general_motion_gate.locked = False
    assert not node._pre_motion_settle_ready(node.decision, 20.0)
    assert not node._pre_motion_settle_ready(node.decision, 20.999)
    assert node._pre_motion_settle_ready(node.decision, 21.0)



@pytest.mark.parametrize('executor_ready_at_enter', [False, True])
def test_enter_gate_discards_vision_until_enter_and_executor_ready(
    monkeypatch, executor_ready_at_enter,
):
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.decision_started = False
    node.executor_auto_ready = executor_ready_at_enter
    node.timeouts = {source: 0.5 for source in MotionDecisionNode.SOURCES}
    monkeypatch.setattr(time, 'monotonic', lambda: 1.0)
    read_fd, write_fd = os.pipe2(os.O_NONBLOCK)
    node.start_terminal_fd = read_fd
    message = String()
    message.data = json.dumps({'detected': True})
    try:
        for source in MotionDecisionNode.SOURCES:
            MotionDecisionNode._info_callback(node, source)(message)
        MotionDecisionNode._publish_decision(node)
        assert all(info is None for info in node.latest_info.values())
        assert node.publisher.messages == []
        assert node.command_id == 0
        assert not node.decision_started

        os.write(write_fd, b'\n')
        MotionDecisionNode._publish_decision(node)
        assert node.decision_started
        assert node.start_terminal_fd is None
        MotionDecisionNode._publish_decision(node)
        assert node.publisher.messages == []

        if not executor_ready_at_enter:
            MotionDecisionNode._info_callback(node, 'line')(message)
            assert node.latest_info['line'] is None
            heartbeat = String()
            heartbeat.data = json.dumps({'sequence': 1, 'auto_ready': True})
            MotionDecisionNode._executor_heartbeat_callback(node, heartbeat)
            MotionDecisionNode._publish_decision(node)
            assert node.publisher.messages == []

        MotionDecisionNode._info_callback(node, 'line')(message)
        MotionDecisionNode._publish_decision(node)
        assert len(node.publisher.messages) == 1
    finally:
        MotionDecisionNode._close_start_terminal(node)
        os.close(write_fd)


def test_enter_gate_does_not_start_on_terminal_eof():
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.decision_started = False
    read_fd, write_fd = os.pipe2(os.O_NONBLOCK)
    node.start_terminal_fd = read_fd
    os.close(write_fd)
    try:
        MotionDecisionNode._publish_decision(node)
        MotionDecisionNode._publish_decision(node)
        assert not node.decision_started
        assert node.publisher.messages == []
        assert node.start_terminal_fd is None
    finally:
        MotionDecisionNode._close_start_terminal(node)


@pytest.mark.parametrize("field,value", [
    ("ground_projection_valid", False),
    ("ground_heading_error_deg", None),
    ("ground_heading_error_deg", float("nan")),
])
def test_line_capture_rejects_invalid_ground_even_with_valid_image(field, value):
    node = FreshMockInputNode()
    node.planner.line_planner.config = replace(node.planner.line_planner.config, heading_source="ground")
    _, sample = build_mock_vision_input("straight")
    sample[field] = value
    sample["filtered_heading_error_deg"] = 20.0
    assert not MotionDecisionNode._line_frame_is_usable(node, sample)


def test_line_capture_accepts_valid_ground_without_image_heading():
    node = FreshMockInputNode()
    node.planner.line_planner.config = replace(node.planner.line_planner.config, heading_source="ground")
    _, sample = build_mock_vision_input("straight")
    sample.pop("filtered_heading_error_deg")
    assert MotionDecisionNode._line_frame_is_usable(node, sample)


@pytest.mark.parametrize("status", ["REJECTED", "FAILED", "TIMEOUT"])
def test_failed_pickup_rearms_visible_ball_but_requires_fresh_vision(status):
    node = FakeDecisionNode("BALL_APPROACH")
    node.terminal_action_armed["ball"] = False
    node.latest_info["ball"] = {"detected": True}
    node.latest_time["ball"] = 10.0
    arm_special_command(node, "PICKUP_NOW", 400, 40)
    if status != "REJECTED":
        send_status(node, status="RUNNING", action="PICKUP_NOW", command_id=400,
                    event_id=40, dynamics_command=None)
    send_status(node, status=status, action="PICKUP_NOW", command_id=400,
                event_id=40, dynamics_command=None,
                error_code="INVALID_MOTION" if status == "REJECTED" else "")
    assert node.active_special_command_id is None
    assert node.terminal_action_armed["ball"] is True
    assert node.latest_info["ball"] is None
    assert not node.general_motion_gate.has_required_fresh_vision()
    assert node.phase_manager.pickup_failure_count == 1
    retry = terminal_decision("ball", "PICKUP_NOW", "BALL_APPROACH")
    assert MotionDecisionNode._suppress_duplicate_terminal_action(node, retry).action == "PICKUP_NOW"
    node.general_motion_gate.on_new_vision_input()
    assert node.general_motion_gate.has_required_fresh_vision()


def test_shot_pause_survives_geometry_valid_reconfirmation_without_firing(monkeypatch):
    planner = MotionDecisionPlanner()
    def decision(confirmed):
        return planner.plan("GOAL_APPROACH", {"goal": {
            "detected": True, "confidence": 0.9, "depth_valid": True,
            "depth_m": 0.42, "offset_x_px": 18, "bearing_deg": 1.1,
            "score_now": confirmed,
        }}, 0.1)
    node = ReadinessPublishNode(decision(True))
    node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    set_grasp_result_for_first_ball(node, "GRABBED")
    clock = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    MotionDecisionNode._publish_decision(node)
    for stamp, confirmed in [(10.5, False), (10.9, True), (11.5, False)]:
        clock[0] = stamp
        node.decision = decision(confirmed)
        MotionDecisionNode._publish_decision(node)
        assert all(json.loads(m.data)["action"] != "SHOT" for m in node.publisher.messages)
        assert node.goal_stationary_since == 10.0
        assert node.active_special_command_id is None
    clock[0] = 11.6
    node.decision = decision(True)
    MotionDecisionNode._publish_decision(node)
    assert json.loads(node.publisher.messages[-1].data)["action"] == "SHOT"


@pytest.mark.parametrize("overrides", [
    {"depth_m": 0.471}, {"offset_x_px": 91}, {"depth_valid": False},
    {"detected": False},
])
def test_shot_rest_survives_candidate_changes_without_actual_motion(overrides):
    node = ReadinessPublishNode(terminal_decision("goal", "SHOT", "GOAL_APPROACH"))
    node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    assert not node._pre_motion_settle_ready(node.decision, 10.0)
    info = {"detected": True, "confidence": 0.9, "depth_valid": True,
            "depth_m": 0.42, "offset_x_px": 18, "score_now": False, **overrides}
    changed = MotionDecisionPlanner().plan("GOAL_APPROACH", {"goal": info}, 0.1)
    node._pre_motion_settle_ready(changed, 10.5)
    assert node.goal_stationary_since == 10.0
    assert not node._pre_motion_settle_ready(node.decision, 10.999)
    assert node._pre_motion_settle_ready(node.decision, 11.0)


@pytest.mark.parametrize("distance,pickup_expected", [
    (0.549, True), (0.550, True), (0.550001, False), (0.570, False),
])
def test_ball_pickup_entry_uses_550mm_boundary(distance, pickup_expected):
    node = FreshMockInputNode()
    MotionDecisionNode._latch_ball_pickup_entry(
        node, ball_info_for_node(distance_m=distance),
    )
    assert node.ball_pickup_entry_pending is pickup_expected


def test_go_success_clears_near_hurdle_rotation_block():
    node = FakeDecisionNode('HURDLE_APPROACH')
    node.planner.hurdle_planner.close_rotation_blocked = True
    complete_motion(node, 'GO', event_id=505, status='SUCCEEDED')
    assert not node.planner.hurdle_planner.close_rotation_blocked


def hurdle_info_for_node():
    return {
        "detected": True, "confirmation_confirmed": True, "confidence": 0.9,
        "depth_valid": True, "depth_m": 0.1, "distance_m": 0.1,
        "hurdle_angle_deg": 0.0, "bottom_distance_px": 200, "go_now": True,
        "camera_center_offset_x_px": 0,
    }


@pytest.mark.parametrize("next_source", ["line", "ball", "goal"])
def test_two_hurdles_disable_only_hurdle_and_resume_other_modes(next_source):
    node = FreshMockInputNode()
    complete_motion(node, "GO", event_id=801)
    assert node.phase_manager.hurdles_completed == 1
    assert not node.planner.hurdle_mission_disabled
    # A clear frame releases the first hurdle's existing re-entry guard.
    select_decision(node)
    assert select_decision(node, hurdle=hurdle_info_for_node()).action == "GO"
    complete_motion(node, "GO", event_id=802)
    assert node.phase_manager.hurdles_completed == 2
    assert node.planner.hurdle_mission_disabled
    select_decision(node)
    inputs = {"hurdle": hurdle_info_for_node(), "line": {
        "detected": True, "ground_projection_valid": True,
        "ground_heading_error_deg": 0.0, "filtered_lateral_offset_norm": 0.0,
        "heading_quality": 0.9, "geometry_quality": 0.9, "detection_quality": 0.9,
    }}
    if next_source == "ball":
        inputs["ball"] = ball_info_for_node()
    elif next_source == "goal":
        inputs["goal"] = {
            "detected": True, "confidence": 0.9, "depth_valid": True,
            "depth_m": 0.43, "distance_m": 0.43, "ground_distance_m": 0.43,
            "bearing_deg": 0.0, "offset_x_norm": 0.0, "offset_x_px": 0,
        }
    assert select_decision(node, **inputs).source == next_source
    assert node.pickups_completed == node.shots_completed == 0
    assert node.ball_sections_processed == 0
    assert not node.mission_complete


def test_completed_ball_and_goal_leave_remaining_hurdles_available():
    node = FreshMockInputNode()
    for index in range(2):
        complete_motion(node, "PICKUP_NOW", event_id=810 + index * 2)
        complete_motion(node, "SHOT", event_id=811 + index * 2)
    node.phase_manager.set_phase("LINE_TRACK")
    assert node.finish_enabled
    assert node.phase_manager.hurdles_completed == 0
    for index in range(2):
        select_decision(node)
        decision = select_decision(node, hurdle=hurdle_info_for_node())
        assert decision.action == "GO"
        complete_motion(node, "GO", event_id=820 + index)
        assert node.phase_manager.hurdles_completed == index + 1
        assert node.pickups_completed == node.shots_completed == 2
    assert not node.mission_complete
    assert select_decision(node, hurdle=hurdle_info_for_node()).source != "hurdle"


@pytest.mark.parametrize("required_hurdles", [0, 2])
def test_completed_hurdle_phase_cannot_reenter(required_hurdles):
    node = FreshMockInputNode()
    node.phase_manager.required_hurdles = required_hurdles
    node.phase_manager.hurdles_completed = required_hurdles
    node.phase_manager.set_phase("HURDLE_APPROACH")
    decision = select_decision(node, hurdle=hurdle_info_for_node())
    assert decision.source != "hurdle"
    assert node.mission_phase == "LINE_TRACK"


@pytest.mark.parametrize("completed", [0, 1, 2])
def test_line_prequeue_ignores_only_completed_hurdles(completed):
    node = ReadinessPublishNode(general_decision("STRAIGHT"))
    node.planner = MotionDecisionPlanner()
    node.phase_manager.hurdles_completed = completed
    observations = {"hurdle": hurdle_info_for_node()}
    assert MotionDecisionNode._line_only_prequeue_allowed(
        node, observations,
    ) is (completed == 2)
    observations["goal"] = {"detected": True}
    assert not MotionDecisionNode._line_only_prequeue_allowed(node, observations)
