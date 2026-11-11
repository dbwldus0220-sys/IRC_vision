"""Bound head-up bottom-loss recovery without changing normal Ball control."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_bottom_loss import BallBottomLoss
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_interior_backward import clock
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message
from test_wait_refresh_and_fine_settle import LiveInputHarness


def camera(up=True):
    return {'ball_head_override_active': not up, 'goal_head_override_active': False,
            'hurdle_head_override_active': False}


def sample(stamp=10., y=600, **changes):
    return ball_info(rgb_stamp_ns=round(stamp * 1e9), raw_detected=True,
                     confirmation_confirmed=True, image_width=1280, image_height=720,
                     bbox=[570, y - 40, 650, min(720, y + 40)], depth_m=.58,
                     head_down_requested=False, offset_x_px=-30, **changes)


def miss(stamp=11.):
    return {'rgb_stamp_ns': round(stamp * 1e9), 'detected': False,
            'raw_detected': False, 'ball_loss_confirmed': True}


def history():
    r = BallBottomLoss()
    r.heartbeat(camera(), 10.)
    r.observe(sample(), active=True, min_confidence=.35)
    r.observe(sample(10.1, 665), active=True, min_confidence=.35)
    r.observe(miss(10.2), active=True, min_confidence=.35)
    r.heartbeat(camera(), 11.)
    return r


def test_downward_exit_is_remembered_until_confirmed_loss_only_once():
    r = history()
    assert r.eligible(miss(), 11., .5)
    assert not r.eligible({**miss(), 'ball_loss_confirmed': False}, 11., .5)
    r.state = 'PENDING'
    assert not r.eligible(miss(), 11., .5)
    r.complete(12_000_000_000, True)
    r.observe(sample(11.9, 665), active=True, min_confidence=.35)
    assert r.state == 'COMPLETE'
    r.observe({**sample(12.1), 'detected': False}, active=True, min_confidence=.35)
    assert r.state == 'COMPLETE'
    r.observe(sample(12.2), active=True, min_confidence=.35)
    assert r.state == 'IDLE'


@pytest.mark.parametrize('case', ['head_down', 'unknown_camera', 'low_confidence',
                                'no_depth', 'at_550', 'below_550', 'stale_depth',
                                'upward', 'sideways', 'middle', 'stale_frame',
                                'inactive', 'duplicate', 'stale_heartbeat', 'stale_loss'])
def test_other_situations_do_not_activate_new_retreat(case):
    r = BallBottomLoss()
    if case != 'unknown_camera':
        r.heartbeat(camera(case != 'head_down'), 11.)
    a, b = sample(), sample(10.1, 665)
    if case in {'low_confidence', 'no_depth', 'at_550', 'below_550', 'stale_depth'}:
        values = {'low_confidence': {'confidence': .2}, 'no_depth': {'depth_valid': False},
                  'at_550': {'distance_m': .55}, 'below_550': {'distance_m': .54},
                  'stale_depth': {'depth_age_sec': .8}}[case]
        a.update(values)
        b.update(values)
    if case == 'upward':
        a, b = sample(y=690), sample(10.1, 665)
    if case == 'sideways':
        b['bbox'] = [1000, 625, 1080, 705]
    if case == 'middle':
        a, b = sample(y=400), sample(10.1, 450)
    if case == 'stale_frame':
        b['rgb_stamp_ns'] = 10_500_000_000
    if case == 'duplicate':
        b['rgb_stamp_ns'] = a['rgb_stamp_ns']
    for info in (a, b, miss(10.2)):
        r.observe(info, active=case != 'inactive', min_confidence=.35)
    now = 12. if case == 'stale_heartbeat' else 11.
    assert not r.eligible(miss(13. if case == 'stale_loss' else 11.), now, .5)


def test_candidate_at_bottom_keeps_recent_confirmed_depth_without_promoting_detection():
    r = BallBottomLoss()
    r.heartbeat(camera(), 11.)
    r.observe(sample(), active=True, min_confidence=.35)
    candidate = {**sample(10.1, 665), 'detected': False, 'confirmation_confirmed': False,
                 'distance_m': None, 'depth_valid': False}
    r.observe(candidate, active=True, min_confidence=.35)
    r.observe(miss(10.2), active=True, min_confidence=.35)
    assert r.eligible(miss(), 11., .5)
    assert candidate['detected'] is False


def receive(node, clock, info):
    Node._info_callback(node, 'ball')(String(data=json.dumps({**info, 'rgb_stamp_ns': round(clock[0] * 1e9)})))
    Node._publish_decision(node)


def test_live_callbacks_remember_candidate_exit_without_changing_ball_mode(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.ball_bottom_loss = BallBottomLoss()
    node.ball_bottom_loss.heartbeat(camera(), clock[0])
    Node._info_callback(node, 'ball')(String(data=json.dumps(sample())))
    assert node.planner.ball_tracking_active
    clock[0] = 10.1
    candidate = {**sample(10.1, 665), 'detected': False,
                 'confirmation_confirmed': False, 'distance_m': None,
                 'depth_valid': False, 'approach_candidate': {
                     'distance_m': .57, 'depth_valid': True, 'depth_age_sec': .05,
                     'head_down_requested': False}}
    Node._info_callback(node, 'ball')(String(data=json.dumps(candidate)))
    clock[0] = 10.2
    Node._info_callback(node, 'ball')(String(data=json.dumps(miss(10.2))))
    node.ball_bottom_loss.heartbeat(camera(), 11.)
    assert node.ball_bottom_loss.eligible(miss(), 11., .5)
    assert node.planner.ball_tracking_active


@pytest.mark.parametrize('depth', [.58, .550])
def test_fresh_confirmed_reacquisition_returns_to_existing_planner(clock, depth):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.ball_bottom_loss = history()
    node.planner._update_ball_tracking(sample(), 0.)
    node.ball_bottom_loss.complete(11_000_000_000, True)
    clock[0] = 12.
    info = sample(12.)
    info.update(distance_m=depth, head_down_requested=depth <= .550)
    Node._info_callback(node, 'ball')(String(data=json.dumps(info)))
    assert node.ball_bottom_loss.state == 'IDLE'
    assert Node._bottom_ball_loss_decision(node, info, 'BALL_APPROACH') is None
    assert node.planner.ball_tracking_active


def test_recovery_finishes_current_motion_reobserves_and_never_chains_forward(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.ball_bottom_loss = history()
    node.planner._update_ball_tracking(sample(), 0.)
    node.ball_approach_alignment_pending = True
    clock[0] = 11.
    node.executor_active = True
    decision = Node._bottom_ball_loss_decision(node, miss(), 'BALL_APPROACH')
    assert decision.action == 'WAIT'
    node.executor_active = False
    for _ in range(9):
        clock[0] += .13
        node.ball_bottom_loss.heartbeat(camera(), clock[0])
        receive(node, clock, miss())
    commands = [m for m in node.publisher.messages if m['action'] == BallBottomLoss.ACTION]
    assert len(commands) == 1
    command = commands[0]
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    clock[0] += 2.
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.ball_bottom_loss.state == 'COMPLETE'
    assert not node.ball_loss_confirmation.confirmed
    assert node.latest_info['ball'] is None
    for _ in range(9):
        clock[0] += .13
        node.ball_bottom_loss.heartbeat(camera(), clock[0])
        receive(node, clock, miss())
    actions = [m['action'] for m in node.publisher.messages]
    assert actions.count(BallBottomLoss.ACTION) == 1
    assert 'BALL_LOST_BACKWARD_2' not in actions
    assert 'BALL_LOST_FORWARD_4' not in actions
    assert any(action.startswith('BALL_APPROACH_TURN_') for action in actions)


@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'TIMEOUT', 'REJECTED'])
def test_failed_retreat_waits_without_retry_or_forward(clock, status):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.ball_bottom_loss = history()
    node.planner._update_ball_tracking(sample(), 0.)
    node.ball_approach_alignment_pending = True
    node.executor_active = False
    clock[0] = 11.
    for _ in range(9):
        clock[0] += .13
        node.ball_bottom_loss.heartbeat(camera(), clock[0])
        receive(node, clock, miss())
    command = next(m for m in node.publisher.messages if m['action'] == BallBottomLoss.ACTION)
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], status)
    assert node.ball_bottom_loss.state == 'FAILED'
    decision = Node._bottom_ball_loss_decision(node, miss(), 'BALL_APPROACH')
    assert decision.action == 'WAIT' and not decision.valid


def test_bridge_uses_existing_one_cycle_backward_and_existing_settle():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action=BallBottomLoss.ACTION, source='ball', command_id=9100,
    ))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'pickup_lost_ball_backward_1'
    assert request['action'] == BallBottomLoss.ACTION


def test_bridge_preserves_half_second_holds_without_extra_decision_dwell(clock):
    from test_motion_command_bridge import executor_status

    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(
        action=BallBottomLoss.ACTION, source='ball', command_id=9100,
    ))
    assert bridge.transition_dwell_until == 10.5
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    clock[0] = 12.
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert bridge.transition_dwell_until == 12.5
    assert bridge.motion_in_progress
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'
    assert not Node._needs_correction_dwell(BallBottomLoss.ACTION)
