"""Tests for special-motion state handling in the motion decision node."""

import json

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecision

import pytest

from std_msgs.msg import String


class FakeLogger:
    """Provide no-op logger methods used by the callback."""

    def info(self, _message):
        """Ignore an informational log message."""
        pass

    def warning(self, _message):
        """Ignore a warning log message."""
        pass


class FakeDecisionNode:
    """Provide only the state required by the motion-status callback."""

    SPECIAL_ACTIONS = MotionDecisionNode.SPECIAL_ACTIONS
    SPECIAL_COMPLETION_PHASES = (
        MotionDecisionNode.SPECIAL_COMPLETION_PHASES
    )
    SPECIAL_ACTION_SOURCES = MotionDecisionNode.SPECIAL_ACTION_SOURCES

    def __init__(
        self,
        mission_phase='AUTO',
        required_pickups=2,
        required_shots=2,
    ):
        """Initialize the minimal state used by node callback tests."""
        self.mission_phase = mission_phase
        self.required_pickups = required_pickups
        self.required_shots = required_shots
        self.pickups_completed = 0
        self.shots_completed = 0
        self.finish_enabled = False
        self.terminal_latch = ('test', 'terminal')
        self.terminal_action_armed = {
            source: True
            for source in self.SPECIAL_ACTION_SOURCES.values()
        }

        self.special_motion_running = False
        self.active_special_action = None
        self.active_special_command_id = None
        self.active_special_event_id = None
        self.active_special_dynamics_command = None

        self.logger = FakeLogger()

    def get_logger(self):
        """Return the fake logger."""
        return self.logger

    def _update_successful_action_progress(self, completed_action):
        """Delegate progress updates to the real node implementation."""
        MotionDecisionNode._update_successful_action_progress(
            self,
            completed_action,
        )


def status_message(
    *,
    status,
    action,
    command_id,
    event_id,
    dynamics_command,
):
    """Create one /motion/status JSON message."""
    payload = {
        'status': status,
        'action': action,
        'command_id': command_id,
        'event_id': event_id,
        'dynamics_command': dynamics_command,
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


def complete_motion(
    node,
    action,
    event_id,
    status='SUCCEEDED',
):
    """Send matching RUNNING and terminal statuses for one event."""
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


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT'])
def test_failed_or_timed_out_special_motion_returns_to_auto(status):
    """Release the lock and return to AUTO after failure or timeout."""
    node = FakeDecisionNode('GOAL_APPROACH')

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


def test_pickup_success_increments_once_and_duplicate_is_ignored():
    """Count one pickup and ignore its retransmitted terminal status."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=501)

    assert node.pickups_completed == 1
    assert node.shots_completed == 0

    send_status(
        node,
        status='SUCCEEDED',
        action='PICKUP_NOW',
        command_id=501,
        event_id=501,
        dynamics_command=0,
    )

    assert node.pickups_completed == 1


def test_failed_shot_does_not_increment_progress():
    """Do not count a shot whose terminal status is FAILED."""
    node = FakeDecisionNode('GOAL_APPROACH')
    complete_motion(node, 'SHOT', event_id=502, status='FAILED')

    assert node.shots_completed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'


def test_one_shot_after_two_pickups_keeps_finish_disabled():
    """Stay in AUTO until the required second shot succeeds."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=503)
    complete_motion(node, 'PICKUP_NOW', event_id=504)
    complete_motion(node, 'SHOT', event_id=505)

    assert node.pickups_completed == 2
    assert node.shots_completed == 1
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'


def test_two_shots_after_two_pickups_enable_walk_to_finish():
    """Enable finish walking when both completion requirements are met."""
    node = FakeDecisionNode()
    complete_motion(node, 'PICKUP_NOW', event_id=506)
    complete_motion(node, 'PICKUP_NOW', event_id=507)
    complete_motion(node, 'SHOT', event_id=508)
    complete_motion(node, 'SHOT', event_id=509)

    assert node.pickups_completed == 2
    assert node.shots_completed == 2
    assert node.finish_enabled is True
    assert node.mission_phase == 'WALK_TO_FINISH'

    complete_motion(node, 'PICKUP_NOW', event_id=511)
    complete_motion(node, 'SHOT', event_id=512)

    assert node.pickups_completed == 2
    assert node.shots_completed == 2
    assert node.finish_enabled is True
    assert node.mission_phase == 'WALK_TO_FINISH'


def test_go_success_does_not_change_pickup_or_shot_progress():
    """Keep pickup and shot counters unchanged after a successful GO."""
    node = FakeDecisionNode('HURDLE_APPROACH')
    complete_motion(node, 'GO', event_id=510)

    assert node.pickups_completed == 0
    assert node.shots_completed == 0
    assert node.finish_enabled is False
    assert node.mission_phase == 'AUTO'
