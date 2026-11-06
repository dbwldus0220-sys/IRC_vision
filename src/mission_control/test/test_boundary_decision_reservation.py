"""Replay conditional continuations without a periodic decision tick or hardware."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecision
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info, goal_info, hurdle_info
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_goal_target_steering import target_sample
from test_motion_command_bridge import FakeBridge
from test_pickup_continuous_walk import ball, start, forward_status
from test_second_pickup_fine_observation import finish_observation_pause


class Timer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class BoundaryHarness(LiveInputHarness):
    # The older harness drives publication manually and does not call Node's
    # constructor. Enable the production event loop for these boundary replays.
    boundary_decision_enabled = True
    LINE_TURN_PRE_MOTION_SETTLE_SEC = .5

    def create_timer(self, period, callback, **kwargs):
        return Timer(period, callback)

    def destroy_timer(self, timer):
        assert timer.cancelled


def executable(node):
    return [message for message in node.publisher.messages
            if message['valid'] and message['action'] not in {'WAIT', 'STOP'}]


def fixed_candidate(node, source, action):
    # Isolate the scheduling gates from steering geometry. The normal publish
    # path still owns freshness, locking, settle, command IDs and interlocks.
    decision = MotionDecision(node.mission_phase, source, action, True, 'test', False, False, {})
    node._select_mission_decision = lambda observations, dt: (
        decision if observations.get(source) is not None else
        MotionDecision(node.mission_phase, source, 'WAIT', False, 'no_input', False, False, {})
    )


@pytest.mark.parametrize('field,source,action,sample', [
    ('correction_post_motion_dwell_until', 'line', 'STRAIGHT_1', line_info()),
    ('hurdle_post_motion_dwell_until', 'hurdle', 'STRAIGHT', hurdle_info()),
    ('post_shot_dwell_until', 'line', 'STRAIGHT_1', line_info()),
    ('goal_post_motion_dwell_until', 'goal', 'GOAL_CAMERA_90_FORWARD_1', goal_info()),
    ('post_ball_line_dwell_until', 'line', 'STRAIGHT_1', line_info()),
    ('ball_post_motion_dwell_until', 'ball', 'STRAIGHT', ball_info()),
])
def test_each_post_dwell_keeps_half_second_and_releases_on_new_image(
        clock, field, source, action, sample):
    node = BoundaryHarness(phase={
        'line': 'LINE_TRACK', 'ball': 'BALL_APPROACH',
        'goal': 'GOAL_APPROACH', 'hurdle': 'HURDLE_POSITIONING',
    }[source])
    fixed_candidate(node, source, action)
    node.boundary_decision_pending = True
    setattr(node, field, clock[0] + .5)
    node.correction_post_motion_source = source
    Node._publish_decision(node)
    timer = node.boundary_decision_timer
    assert timer.period == pytest.approx(.5)
    clock[0] = 10.48
    fresh(node, clock, source, sample, plan=False)
    assert not executable(node)
    clock[0] = 10.5
    timer.callback()
    assert timer.cancelled
    assert not executable(node)
    assert node.latest_info[source] is None
    fresh(node, clock, source, sample, plan=False)
    assert [message['action'] for message in executable(node)] == [action]


@pytest.mark.parametrize('action', [
    'LINE_HEADING_TURN_LEFT_3', 'LINE_OFFSET_TURN_LEFT_2',
    'POST_BALL_LINE_TURN_LEFT_2',
    'POST_SHOT_LINE_TURN_LEFT_2',
])
def test_turn_continuation_keeps_pre_motion_half_second(clock, action):
    node = BoundaryHarness(phase='LINE_TRACK')
    fixed_candidate(node, 'line', action)
    node.boundary_decision_pending = True
    fresh(node, clock, 'line', line_info(), plan=False)
    timer = node.boundary_decision_timer
    assert timer.period == pytest.approx(.5)
    started = clock[0]
    clock[0] += .47
    fresh(node, clock, 'line', line_info(), plan=False)
    assert not executable(node)
    clock[0] = started + .5
    timer.callback()
    assert [message['action'] for message in executable(node)] == [action]


def test_status_releases_next_decision_without_periodic_tick(clock):
    node = BoundaryHarness(phase='LINE_TRACK')
    fixed_candidate(node, 'line', 'STRAIGHT_1')
    fresh(node, clock, 'line', line_info())
    first = executable(node)[-1]
    assert node.boundary_decision_pending
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    fresh(node, clock, 'line', line_info(), plan=False)
    assert len(executable(node)) == 2
    assert executable(node)[-1]['command_id'] > first['command_id']


def test_existing_sdk_queue_has_priority(clock):
    node = BoundaryHarness(phase='LINE_TRACK')
    fixed_candidate(node, 'line', 'STRAIGHT_1')
    node.boundary_decision_pending = True
    node.queued_general_command_id = 2
    fresh(node, clock, 'line', line_info(), plan=False)
    assert not executable(node)


@pytest.mark.parametrize('blocked', ['locked', 'safety', 'stale'])
def test_deadline_does_not_bypass_execution_or_freshness(clock, blocked):
    node = BoundaryHarness(phase='LINE_TRACK')
    fixed_candidate(node, 'line', 'LINE_HEADING_TURN_LEFT_3')
    node.boundary_decision_pending = True
    fresh(node, clock, 'line', line_info(), plan=False)
    timer = node.boundary_decision_timer
    if blocked == 'locked':
        node.general_motion_gate.on_command_published('STRAIGHT_1', 90)
    elif blocked == 'safety':
        node.safety_interlock.observe_critical_fault(
            error_code='SDK_HARDWARE_NOT_READY', message='test', source='test',
            action=None, command_id=None, event_id=None,
        )
    else:
        node.latest_time['line'] = clock[0] - node.timeouts['line'] - 1
    clock[0] += .5
    timer.callback()
    assert not executable(node)


def test_goal_region_observation_keeps_full_second(clock):
    node = BoundaryHarness(phase='GOAL_APPROACH')
    node.boundary_decision_pending = True
    node.goal_shot_observe_until = clock[0] + 1.
    Node._schedule_boundary_decision(node)
    assert node.boundary_decision_timer.period == pytest.approx(1.)


def test_foreign_status_cannot_release_or_cancel_continuation(clock, monkeypatch):
    node = BoundaryHarness(phase='LINE_TRACK')
    node.boundary_decision_pending = True
    node.general_motion_gate.on_command_published('STRAIGHT_1', 1)
    calls = []
    monkeypatch.setattr(Node, '_handle_motion_status', lambda self, message: None)
    monkeypatch.setattr(Node, '_resume_boundary_decision', lambda self: calls.append(True))
    for payload in [
        {'command_id': 9, 'action': 'STRAIGHT_1', 'status': 'FAILED'},
        {'command_id': 1, 'action': 'STRAIGHT_2', 'status': 'SUCCEEDED'},
    ]:
        Node._motion_status_callback(node, String(data=json.dumps(payload)))
    assert node.boundary_decision_pending
    assert not calls


def test_goal_turn_rechecks_direction_after_pause_without_polling(clock):
    node = BoundaryHarness(phase='GOAL_APPROACH')
    fresh(node, clock, 'goal', target_sample(60))
    turn = executable(node)[-1]
    assert turn['action'] == 'GOAL_CAMERA90_TURN_RIGHT_6'
    node.send_status(turn['action'], turn['command_id'], 'RUNNING')
    fresh(node, clock, 'goal', target_sample(-60), plan=False)
    assert len(executable(node)) == 1
    node.send_status(turn['action'], turn['command_id'], 'SUCCEEDED')
    deadline = node.goal_post_motion_dwell_until
    timer = node.boundary_decision_timer
    assert deadline == pytest.approx(clock[0] + .5)
    clock[0] = deadline
    timer.callback()
    assert len(executable(node)) == 1
    # The earlier left-turn observation must not be committed after the pause.
    fresh(node, clock, 'goal', target_sample(0), plan=False)
    assert executable(node)[-1]['action'] == 'GOAL_CAMERA_90_FORWARD_2'
    assert len(executable(node)) == 2


def test_cancelled_deadline_cannot_consume_a_new_reservation(clock):
    node = BoundaryHarness(phase='LINE_TRACK')
    node.boundary_decision_pending = True
    node.goal_post_motion_dwell_until = clock[0] + .5
    Node._schedule_boundary_decision(node)
    old = node.boundary_decision_timer
    node.goal_post_motion_dwell_until = clock[0] + 1.
    Node._schedule_boundary_decision(node)
    new = node.boundary_decision_timer
    old.callback()
    assert node.boundary_decision_timer is new
    assert not new.cancelled


def test_correlated_failure_cancels_deadline(clock, monkeypatch):
    node = BoundaryHarness(phase='LINE_TRACK')
    node.boundary_decision_pending = True
    node.goal_post_motion_dwell_until = clock[0] + .5
    Node._schedule_boundary_decision(node)
    timer = node.boundary_decision_timer
    node.general_motion_gate.on_command_published('STRAIGHT_1', 1)
    monkeypatch.setattr(Node, '_handle_motion_status', lambda self, message: None)
    Node._motion_status_callback(node, String(data=json.dumps({
        'command_id': 1, 'action': 'STRAIGHT_1', 'status': 'FAILED',
    })))
    assert timer.cancelled
    assert not node.boundary_decision_pending


@pytest.mark.parametrize('bottom,expected', [
    (230, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'), (231, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_second_pickup_fine_continues_on_first_post_pause_frame(clock, bottom, expected):
    node = BoundaryHarness(phase='BALL_APPROACH')
    node.phase_manager.active_ball_mode_index = 2
    node.phase_manager.ball_modes_confirmed = 2
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    parent, request = start(node, bridge, clock)
    clock[0] += 2.
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert bridge.active_dwell_until == pytest.approx(clock[0] + .5)
    count = len(executable(node))
    fresh(node, clock, 'ball', ball(bottom_distance_px=bottom), plan=False)
    assert len(executable(node)) == count
    finish_observation_pause(node, bridge, clock)
    assert len(executable(node)) == count
    fresh(node, clock, 'ball', ball(bottom_distance_px=bottom), plan=False)
    assert executable(node)[-1]['action'] == expected
    assert len(executable(node)) == count + 1
    assert node.active_special_command_id == parent['command_id']


@pytest.mark.parametrize('offset,direction', [(-250, 'left'), (250, 'right')])
def test_two_cycle_pickup_crab_releases_fine_checkpoint_without_polling(clock, offset, direction):
    node = BoundaryHarness(phase='BALL_APPROACH')
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = f'pickup_crab_{direction}_0_2'
    _, request = start(node, bridge, clock, ball(offset_x_px=offset))
    assert request['motion_id'] == f'pickup_crab_{direction}_0_2'
    clock[0] += 2.
    forward_status(node, bridge, request, 'SUCCEEDED')
    count = len(executable(node))
    fresh(node, clock, 'ball', ball(), plan=False)
    assert len(executable(node)) == count + 1
    assert executable(node)[-1]['action'] in {
        'BALL_PICKUP_INITIAL_ALIGN_CONTINUE', 'BALL_PICKUP_FINE_FORWARD',
    }
