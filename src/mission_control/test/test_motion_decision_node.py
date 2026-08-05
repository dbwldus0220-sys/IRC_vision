"""Tests for special-motion state handling in the motion decision node."""

import json
from pathlib import Path
import sys

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecision
from mission_control.motion_command_gate import GeneralMotionCommandGate
from mission_control.mission_phase_manager import MissionPhaseManager

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

    def clear_collected_ball_tracking(self):
        """Match the planner lifecycle hook used after a successful pickup."""
        pass

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


class FakeDecisionNode:
    """Provide only the state required by the motion-status callback."""

    SPECIAL_ACTIONS = MotionDecisionNode.SPECIAL_ACTIONS
    SPECIAL_ACTION_SOURCES = MotionDecisionNode.SPECIAL_ACTION_SOURCES

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
        self.general_motion_gate = GeneralMotionCommandGate()
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


def status_message(
    *,
    status,
    action,
    command_id,
    event_id,
    dynamics_command,
    error_code=None,
    message=None,
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
            action="TURN_LEFT",
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
            },
        )()
        self.previous_publish_time = 0.0
        self.command_id = 0
        self.event_id = 0
        self.terminal_latch = None
        self.publisher = ReadinessPublisher(subscription_count)
        self._command_publisher_ready = None

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

    @staticmethod
    def _mission_progress():
        return {}

    def _command_publisher_has_subscriber(self):
        return MotionDecisionNode._command_publisher_has_subscriber(self)


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
    assert node.mission_phase == 'AUTO'


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


@pytest.mark.parametrize(
    ("scenario", "expected_action"),
    [
        ("straight", "STRAIGHT"),
        ("turn_left", "FINE_LEFT"),
        ("turn_right", "FINE_RIGHT"),
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
    assert decision.valid is (expected_action == "STRAIGHT")


@pytest.mark.parametrize(
    (
        'initial_phase',
        'action',
        'dynamics_command',
        'expected_phase',
    ),
    [
        ('AUTO', 'PICKUP_NOW', 9, 'GOAL_APPROACH'),
        ('GOAL_APPROACH', 'SHOT', 17, 'AUTO'),
        ('HURDLE_APPROACH', 'GO', 14, 'AUTO'),
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
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.special_motion_running is False
    assert node.active_special_action is None
    assert node.active_special_command_id is None


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_failed_or_timed_out_special_motion_returns_to_auto(status):
    """Release the lock and return to AUTO after failure or timeout."""
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
    assert node.mission_phase == 'AUTO'
    assert node.terminal_latch is None
    assert node.pickups_completed == 0
    assert node.shots_completed == 0


@pytest.mark.parametrize(
    ('initial_phase', 'action', 'expected_phase'),
    [
        ('BALL_APPROACH', 'PICKUP_NOW', 'AUTO'),
        ('GOAL_APPROACH', 'SHOT', 'AUTO'),
        ('HURDLE_APPROACH', 'GO', 'AUTO'),
        ('GOAL_APPROACH', 'GO', 'GOAL_APPROACH'),
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

    assert node.ball_sections_processed == 1
    assert node.mission_phase == 'AUTO'

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
        ('BALL_APPROACH', 'PICKUP_NOW', 'AUTO'),
        ('GOAL_APPROACH', 'SHOT', 'AUTO'),
        ('HURDLE_APPROACH', 'GO', 'AUTO'),
        ('GOAL_APPROACH', 'GO', 'GOAL_APPROACH'),
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

    assert node.mission_phase == 'AUTO'
    assert node.ball_sections_processed == 1
    assert node.active_special_action is None
    assert node.active_special_event_id is None


@pytest.mark.parametrize(
    ('source', 'action', 'initial_phase', 'expected_phase'),
    [
        ('hurdle', 'GO', 'HURDLE_APPROACH', 'AUTO'),
        ('ball', 'PICKUP_NOW', 'BALL_APPROACH', 'GOAL_APPROACH'),
        ('goal', 'SHOT', 'GOAL_APPROACH', 'AUTO'),
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
    assert node.mission_phase == 'GOAL_APPROACH'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_pickup_failure_processes_section_without_score(status):
    """Abandon a failed pickup and continue course progress."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=502, status=status)

    assert node.pickups_completed == 0
    assert node.ball_sections_processed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'


def test_shot_success_increments_score_and_section():
    """Count both success score and section for a successful shot."""
    node = FakeDecisionNode('GOAL_APPROACH')
    complete_motion(node, 'SHOT', event_id=503)

    assert node.shots_completed == 1
    assert node.ball_sections_processed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_shot_failure_processes_section_without_score(status):
    """Complete course progress even when the shot does not score."""
    node = FakeDecisionNode('GOAL_APPROACH')
    complete_motion(node, 'SHOT', event_id=504, status=status)

    assert node.shots_completed == 0
    assert node.ball_sections_processed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'


@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED'])
def test_go_terminal_status_does_not_change_section(status):
    """A hurdle action is independent from ball-section progress."""
    node = FakeDecisionNode('HURDLE_APPROACH')
    complete_motion(node, 'GO', event_id=505, status=status)

    assert node.pickups_completed == 0
    assert node.shots_completed == 0
    assert node.ball_sections_processed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'


def test_second_pickup_failure_keeps_automatic_flow_active():
    """Keep AUTO when a failed pickup processes the last section."""
    node = FakeDecisionNode()
    complete_motion(node, 'SHOT', event_id=506)
    complete_motion(node, 'PICKUP_NOW', event_id=507, status='FAILED')

    assert node.ball_sections_processed == 2
    assert node.finish_enabled is True
    assert node.mission_phase == 'AUTO'


def test_second_shot_failure_keeps_automatic_flow_active():
    """Keep AUTO when a failed shot processes the last section."""
    node = FakeDecisionNode()
    complete_motion(node, 'SHOT', event_id=508)
    complete_motion(node, 'SHOT', event_id=509, status='FAILED')

    assert node.shots_completed == 1
    assert node.ball_sections_processed == 2
    assert node.finish_enabled is True
    assert node.mission_phase == 'AUTO'


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

    assert node.ball_sections_processed == 1


def test_section_progress_is_capped_at_requirement():
    """Never report more processed sections than configured."""
    node = FakeDecisionNode(required_ball_sections=2)
    complete_motion(node, 'SHOT', event_id=511)
    complete_motion(node, 'SHOT', event_id=512)
    complete_motion(node, 'SHOT', event_id=513)

    assert node.ball_sections_processed == 2
    assert node.finish_enabled is True


def test_mission_progress_contains_exact_fields_and_values():
    """Expose success scores and course progress in command JSON shape."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=514)
    complete_motion(node, 'SHOT', event_id=515)

    progress = MotionDecisionNode._mission_progress(node)

    assert progress == {
        'pickups_completed': 1,
        'required_pickups': 2,
        'shots_completed': 1,
        'required_shots': 2,
        'ball_sections_processed': 1,
        'required_ball_sections': 2,
        'finish_enabled': False,
        'mission_complete': False,
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
