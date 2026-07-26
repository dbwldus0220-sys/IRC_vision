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


class FakePlanner:
    """Return deterministic line or lock decisions for node-level tests."""

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
    SPECIAL_COMPLETION_PHASES = (
        MotionDecisionNode.SPECIAL_COMPLETION_PHASES
    )
    SPECIAL_ACTION_SOURCES = MotionDecisionNode.SPECIAL_ACTION_SOURCES

    def __init__(
        self,
        mission_phase='AUTO',
        required_pickups=2,
        required_shots=2,
        required_ball_sections=2,
    ):
        """Initialize the minimal state used by node callback tests."""
        self.mission_phase = mission_phase
        self.required_pickups = required_pickups
        self.required_shots = required_shots
        self.required_ball_sections = required_ball_sections
        self.pickups_completed = 0
        self.shots_completed = 0
        self.ball_sections_processed = 0
        self.finish_enabled = required_ball_sections == 0
        self.mission_complete = False
        self.finish_min_confidence = 0.70
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
        self.planner = FakePlanner()

        self.logger = FakeLogger()

    def get_logger(self):
        """Return the fake logger."""
        return self.logger

    def _update_action_progress(self, completed_action, status):
        """Delegate progress updates to the real node implementation."""
        MotionDecisionNode._update_action_progress(
            self,
            completed_action,
            status,
        )

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


def test_second_pickup_failure_enables_walk_to_finish():
    """Walk to finish when a failed pickup processes the last section."""
    node = FakeDecisionNode()
    complete_motion(node, 'SHOT', event_id=506)
    complete_motion(node, 'PICKUP_NOW', event_id=507, status='FAILED')

    assert node.ball_sections_processed == 2
    assert node.finish_enabled is True
    assert node.mission_phase == 'WALK_TO_FINISH'


def test_second_shot_failure_enables_walk_to_finish():
    """Walk to finish when a failed shot processes the last section."""
    node = FakeDecisionNode()
    complete_motion(node, 'SHOT', event_id=508)
    complete_motion(node, 'SHOT', event_id=509, status='FAILED')

    assert node.shots_completed == 1
    assert node.ball_sections_processed == 2
    assert node.finish_enabled is True
    assert node.mission_phase == 'WALK_TO_FINISH'


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


def test_finish_detection_is_ignored_until_finish_is_enabled():
    """Keep normal phase planning when mission progress is incomplete."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    decision = select_decision(node, finish=finish_info())

    assert decision.phase == 'WALK_TO_FINISH'
    assert decision.action == 'STRAIGHT'
    assert decision.source == 'line'
    assert decision.requires_ack is False


def test_confirmed_finish_requests_cross_finish():
    """Request acknowledged crossing for an enabled confirmed finish."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    decision = select_decision(node, finish=finish_info())

    assert decision.phase == 'WALK_TO_FINISH'
    assert decision.source == 'finish'
    assert decision.action == 'CROSS_FINISH'
    assert decision.sdk_motion_requested is True
    assert decision.requires_ack is True


def test_continuous_finish_detection_does_not_retrigger_crossing():
    """Suppress CROSS_FINISH while the same finish target is disarmed."""
    node = FakeDecisionNode('WALK_TO_FINISH')
    node.finish_enabled = True
    decision = select_decision(node, finish=finish_info())
    node.terminal_action_armed['finish'] = False

    suppressed = MotionDecisionNode._suppress_duplicate_terminal_action(
        node,
        decision,
    )

    assert suppressed.action == 'WAIT'
    assert suppressed.reason == 'duplicate_terminal_action_suppressed'
    assert suppressed.sdk_motion_requested is False
    assert suppressed.requires_ack is False


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
def test_cross_finish_failure_returns_to_walk_and_allows_retry(status):
    """Return to finish walking and re-arm crossing after failure."""
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

    retry = select_decision(node, finish=finish_info())
    assert retry.action == 'CROSS_FINISH'
    assert retry.requires_ack is True
