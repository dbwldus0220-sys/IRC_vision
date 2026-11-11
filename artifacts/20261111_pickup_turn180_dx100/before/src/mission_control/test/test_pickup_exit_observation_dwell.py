"""Reuse the final pickup pause without delaying a later usable Line frame."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_mission_phase_flow import line_info
from test_sparse_line_recovery import sparse_info
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda self: round(now[0] * 1e9))
    return now


def status(node, *, motion_id, status='RUNNING', command_id=50, **extra):
    Node._motion_status_callback(node, String(data=json.dumps({
        'action': 'PICKUP_NOW', 'command_id': command_id, 'event_id': None,
        'status': status, 'motion_id': motion_id, **extra,
    })))


def begin_exit(clock, section=1):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    node.phase_manager.pickups_completed = section - 1
    assert node.phase_manager.start_special_action('PICKUP_NOW', 50)
    node.phase_manager.ball_grasp_results[section] = 'GRABBED'
    motion_id = ('pickup_first_backward_turn_right' if section == 1
                 else 'pickup_second_backward_turn_left')
    status(node, motion_id=motion_id)
    status(node, motion_id=Node.PICKUP_DWELL_MARKER, completed_motion_id=motion_id)
    return node, motion_id


def frame(node, clock, info=None, *, stamp=None):
    payload = {**(line_info(heading=-65.) if info is None else info),
               'rgb_stamp_ns': round(clock[0] * 1e9) if stamp is None else stamp}
    Node._info_callback(node, 'line')(String(data=json.dumps(payload)))


def motions(node):
    return [message for message in node.publisher.messages if message.get('valid')]


def stable_frames(node, clock, end, info=None):
    for moment in (end - .02, end - .01, end):
        clock[0] = moment
        frame(node, clock, info)


@pytest.mark.parametrize('section', [1, 2])
def test_ready_during_pause_dispatches_at_completion_without_second_pause(clock, section):
    node, motion_id = begin_exit(clock, section)
    for moment in (10.1, 10.2, 10.4, 10.499):
        clock[0] = moment
        frame(node, clock)
        Node._publish_decision(node)
        assert not motions(node)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert len(motions(node)) == 1
    assert motions(node)[0]['action'].startswith('LINE_HEADING_TURN_LEFT_')
    assert node.pre_motion_settle_started_at is None
    assert node.pickup_exit_dwell_command_id is None
    Node._publish_decision(node)
    assert len(motions(node)) == 1


@pytest.mark.parametrize('late_time', [10.51, 10.73, 12.17])
def test_unusable_input_waits_only_until_next_usable_observation(clock, late_time):
    node, motion_id = begin_exit(clock)
    bad = {**line_info(), 'ground_projection_valid': False,
           'ground_heading_error_deg': None, 'ground_fit_reason': 'too_few_inliers'}
    clock[0] = 10.4
    frame(node, clock, bad)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    assert node.pickup_exit_dwell_complete
    for moment in (late_time, late_time + .01):
        clock[0] = moment
        frame(node, clock)
        assert not motions(node)
    clock[0] = late_time + .02
    frame(node, clock)
    # Dispatch happens on confirmation, with no extra timer tick or pause.
    assert len(motions(node)) == 1
    assert motions(node)[0]['action'].startswith('LINE_HEADING_TURN_LEFT_')


def test_two_point_confirmation_accumulates_during_bridge_pause(clock):
    node, motion_id = begin_exit(clock)
    for moment in (10.1, 10.2, 10.3):
        clock[0] = moment
        frame(node, clock, sparse_info(heading=0., distance=.6))
        Node._publish_decision(node)
        assert not motions(node)
    assert node.sparse_line_recovery.stable
    assert not node.latest_info['line']['ground_projection_valid']
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert len(motions(node)) == 1
    assert motions(node)[0]['action'] == 'LINE_SPARSE_FORWARD'
    assert motions(node)[0]['source_command']['ground_fit_mode'] == 'TWO_POINT_CONFIRMED'


def test_late_third_distinct_capture_dispatches_without_new_pause(clock):
    node, motion_id = begin_exit(clock)
    for moment in (10.2, 10.4):
        clock[0] = moment
        frame(node, clock, sparse_info(heading=0., distance=.6))
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    for moment in (10.51, 10.52):
        clock[0] = moment
        frame(node, clock, sparse_info(heading=0., distance=.6), stamp=10_400_000_000)
        assert not motions(node)
    clock[0] = 10.53
    frame(node, clock, sparse_info(heading=0., distance=.6))
    assert len(motions(node)) == 1
    assert motions(node)[0]['action'] == 'LINE_SPARSE_FORWARD'


@pytest.mark.parametrize('stamp', [9_900_000_000, 10_000_000_000, 11_000_000_000, True])
def test_pre_motion_future_or_invalid_capture_cannot_dispatch(clock, stamp):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.4
    frame(node, clock, stamp=stamp)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    stable_frames(node, clock, 10.6)
    assert len(motions(node)) == 1


def test_stale_or_missing_frames_wait_for_fresh_information(clock):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.01
    frame(node, clock)
    clock[0] = 10.7
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    clock[0] = 11.3
    Node._publish_decision(node)
    assert not motions(node)
    stable_frames(node, clock, 11.32)
    assert len(motions(node)) == 1


def test_duplicate_dwell_notice_does_not_restart_pause_or_drop_samples(clock):
    node, motion_id = begin_exit(clock)
    stable_frames(node, clock, 10.3)
    status(node, motion_id=Node.PICKUP_DWELL_MARKER, completed_motion_id=motion_id)
    assert node.pickup_exit_dwell_started_at == 10.
    assert node.latest_info['line'] is not None
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert len(motions(node)) == 1


@pytest.mark.parametrize('outcome', ['FAILED', 'TIMEOUT', 'REJECTED'])
def test_failed_pickup_does_not_keep_pause_credit(clock, outcome):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.3
    frame(node, clock)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status=outcome)
    assert not motions(node)
    assert node.pickup_exit_dwell_command_id is None


@pytest.mark.parametrize('block', ['safety', 'executor', 'subscriber'])
def test_existing_dispatch_guards_still_apply(clock, block):
    node, motion_id = begin_exit(clock)
    stable_frames(node, clock, 10.4)
    if block == 'safety':
        node.safety_interlock.observe_executor_status(
            error_code='SDK_HARDWARE_NOT_READY', message='test',
            action='PICKUP_NOW', command_id=50, event_id=None)
        assert node.safety_interlock.latched
    elif block == 'executor':
        node.executor_active = True
    else:
        node.publisher.subscription_count = 0
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)


def test_next_turn_keeps_normal_pause(clock):
    node, motion_id = begin_exit(clock)
    stable_frames(node, clock, 10.4)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    first = motions(node)[0]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] = 11.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    clock[0] = 11.5
    Node._publish_decision(node)
    clock[0] = 11.51
    frame(node, clock)
    Node._publish_decision(node)
    assert len(motions(node)) == 1
    assert node.pre_motion_settle_started_at == 11.51
    clock[0] = 12.02
    frame(node, clock)
    Node._publish_decision(node)
    assert len(motions(node)) == 2


def test_early_terminal_status_cannot_shorten_existing_half_second(clock):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.2
    frame(node, clock)
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    clock[0] = 10.3
    frame(node, clock)
    clock[0] = 10.499
    frame(node, clock)
    assert not motions(node)
    clock[0] = 10.5
    Node._publish_decision(node)
    assert len(motions(node)) == 1


def test_unstable_two_point_input_cannot_use_elapsed_pause_as_confirmation(clock):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    for moment, heading in [(10.6, 0.), (10.7, 12.), (10.8, 0.), (10.9, 12.)]:
        clock[0] = moment
        frame(node, clock, sparse_info(heading=heading, distance=.6))
        assert not motions(node)
    for moment in (11., 11.1):
        clock[0] = moment
        frame(node, clock, sparse_info(heading=0., distance=.6))
        assert not motions(node)
    clock[0] = 11.2
    frame(node, clock, sparse_info(heading=0., distance=.6))
    assert len(motions(node)) == 1


def test_future_capture_is_not_saved_for_later_execution(clock):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.4
    frame(node, clock, stamp=10_500_000_000)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    assert node.latest_info['line'] is None


def test_phase_override_discards_single_use_pause_credit(clock):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    node.send_phase('LINE_TRACK')
    assert node.pickup_exit_dwell_command_id is None
    clock[0] = 10.6
    frame(node, clock)
    Node._publish_decision(node)
    assert not motions(node)
    assert node.pre_motion_settle_started_at == 10.6


def test_missing_dwell_notice_keeps_existing_fresh_input_and_settle_policy(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    assert node.phase_manager.start_special_action('PICKUP_NOW', 50)
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    status(node, motion_id='pickup_first_backward_turn_right')
    clock[0] = 10.4
    frame(node, clock)
    clock[0] = 10.5
    status(node, motion_id='pickup_first_backward_turn_right', status='SUCCEEDED')
    assert node.latest_info['line'] is None
    clock[0] = 10.6
    frame(node, clock)
    Node._publish_decision(node)
    assert not motions(node)
    assert node.pre_motion_settle_started_at == 10.6


@pytest.mark.parametrize('section', [1, 2])
def test_bridge_final_dwell_status_drives_observation_and_exactly_once_dispatch(clock, section):
    from test_motion_command_bridge import FakeBridge

    node = LiveInputHarness(phase='BALL_APPROACH')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    node.phase_manager.pickups_completed = section - 1
    assert node.phase_manager.start_special_action('PICKUP_NOW', 50)
    node.phase_manager.ball_grasp_results[section] = 'GRABBED'
    motion_id = ('pickup_first_backward_turn_right' if section == 1
                 else 'pickup_second_backward_turn_left')
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 50
    bridge.active_motion_id = motion_id
    bridge.active_pickup_sequence = (motion_id, bridge.DWELL_MARKER)
    bridge.active_sequence_index = 0
    assert bridge._start_next_pickup_motion()
    Node._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    stable_frames(node, clock, 10.4)
    bridge._check_atomic_dwell()
    assert not motions(node)
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    completion = bridge.motion_status_publisher.messages[-1]
    assert json.loads(completion.data)['status'] == 'SUCCEEDED'
    Node._motion_status_callback(node, completion)
    assert len(motions(node)) == 1
    Node._motion_status_callback(node, completion)
    assert len(motions(node)) == 1
    assert not bridge.executor_request_publisher.messages


@pytest.mark.parametrize('kind', ['image', 'offset', 'lost'])
def test_other_line_turns_reuse_same_pause_with_their_own_direction_measure(clock, kind):
    from test_line_ground_fit_recovery import unconfirmed_sparse_info
    from test_line_offset_alignment import sample

    node, motion_id = begin_exit(clock)
    if kind == 'image':
        info = unconfirmed_sparse_info()
        expected = 'LINE_OFFSET_TURN_RIGHT_'
    elif kind == 'offset':
        info = sample(offset=250., target_angle=25., ground_heading_error_deg=0.,
                      ground_fit_point_count=3, ground_projection_enabled=True,
                      corner_preview_confirmed=False, corner_preview_raw_detected=False)
        expected = 'LINE_OFFSET_TURN_RIGHT_'
    else:
        info = {'detected': False, 'raw_detected': False}
        expected = 'POST_BALL_LINE_TURN_RIGHT_'
    stable_frames(node, clock, 10.4, info)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert len(motions(node)) == 1
    assert motions(node)[0]['action'].startswith(expected)


def test_single_good_heading_and_repeated_capture_do_not_authorize_turn(clock):
    node, motion_id = begin_exit(clock)
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    for moment, heading in [(10.6, -65.), (10.7, 65.), (10.8, -65.)]:
        clock[0] = moment
        frame(node, clock, line_info(heading=heading))
        assert not motions(node)
    clock[0] = 10.85
    frame(node, clock, stamp=10_800_000_000)
    assert not motions(node)
    clock[0] = 10.9
    frame(node, clock)
    assert not motions(node)
    clock[0] = 10.95
    frame(node, clock)
    assert len(motions(node)) == 1


def test_lower_quality_latest_input_prevents_using_earlier_stable_direction(clock):
    node, motion_id = begin_exit(clock)
    stable_frames(node, clock, 10.3)
    clock[0] = 10.4
    frame(node, clock, {**line_info(heading=-65.), 'heading_quality': .1})
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert not motions(node)
    stable_frames(node, clock, 10.6)
    assert len(motions(node)) == 1
