"""Fresh direction-first alignment and continuous camera-45 support changes."""

import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.ball_fine_approach import duration_sec, motion_id
from test_ball_entry_line_alignment import ground_ball_info, node_with_entry, send, start_line, status
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_continuous_approach import clock, fresh


def ball(angle=0., distance=.55, **updates):
    return ground_ball_info(depth_m=distance, ground_steering_angle_deg=angle,
                            bottom_distance_px=300., **updates)


@pytest.mark.parametrize('distance', [.500001, .55, .6])
@pytest.mark.parametrize('angle', [-35., 0., 35.])
def test_near_band_aligns_by_stationary_turn_before_forward(distance, angle):
    command = MotionDecisionPlanner()._plan_source('ball', ball(angle, distance), .1)
    assert command['valid'] and command['ball_fine_alignment']
    if angle:
        assert command['motion'].startswith('BALL_APPROACH_TURN_LEFT_' if angle < 0 else 'BALL_APPROACH_TURN_RIGHT_')
        assert command['linear_speed_mps'] == command['lateral_speed_mps'] == 0
    else:
        assert command['motion'] == 'BALL_APPROACH_FINE_FORWARD_5'


@pytest.mark.parametrize('updates', [
    {'depth_valid': False}, {'depth_age_sec': .701}, {'depth_age_sec': None},
    {'confidence': .1}, {'ground_projection_valid': False}, {'detected': False},
])
def test_bad_observations_never_authorize_near_forward_or_turn(updates):
    command = MotionDecisionPlanner()._plan_source('ball', ball(-35., **updates), .1)
    assert not command['valid']


# Ground-projected Ball angles must be strictly below 90 degrees.
@pytest.mark.parametrize('index,angle', [(1, -35.), (2, 35.), (1, -89.), (2, -89.)])
@pytest.mark.parametrize('turn_first', [True, False])
def test_forward_turn_prepare_fine_prepare_turn_keeps_correction_dwells(clock, index, angle, turn_first):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    _, running = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball(angle, 1.2), plan=False)
    # Each successor is selected during its predecessor, including the 600 mm crossing.
    successors = [(angle, 'BALL_APPROACH_TURN_')] if turn_first else []
    successors += [
        (0., 'BALL_APPROACH_PREPARE_FINE'),
        (0., 'BALL_APPROACH_FINE_FORWARD_5'),
        (-angle, 'BALL_APPROACH_PREPARE_TURN'),
        (-angle, 'BALL_APPROACH_TURN_'),
    ]
    for next_angle, expected in successors:
        scoped = running['motion_id'].startswith('ball_align_')
        start = node.ball_fine_started_at if scoped else node.active_line_motion_started_at
        duration = duration_sec(running['motion_id']) if scoped else node.active_line_motion_duration_sec
        prepare = running['action'].startswith('BALL_APPROACH_PREPARE_')
        clock[0] = start + duration * (.1 if prepare else .82)
        for _ in range(1 if prepare else 3):
            fresh(node, clock, 'ball', ball(next_angle), plan=False)
        command = node.publisher.messages[-1]
        assert command['action'].startswith(expected), command
        assert command['command_id'] == node.queued_general_command_id
        previous_count = len(decoded_messages(bridge.executor_request_publisher))
        send(bridge, command)
        correction_boundary = (running['motion_id'].startswith(('ball_align_turn_', 'ball_align_fine_'))
                               or command['action'].startswith(('BALL_APPROACH_TURN_', 'BALL_APPROACH_FINE_'))
                               or command['action'] == 'BALL_APPROACH_PREPARE_FINE')
        assert bridge.queued_request_deferred == correction_boundary
        assert clock[0] < start + duration
        assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
        if correction_boundary:
            assert len(decoded_messages(bridge.executor_request_publisher)) == previous_count
        else:
            status(node, bridge, decoded_messages(bridge.executor_request_publisher)[-1], 'QUEUED')
        clock[0] = start + duration
        status(node, bridge, running, 'SUCCEEDED')
        if correction_boundary:
            assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
            clock[0] += .499
            bridge._check_atomic_dwell()
            assert len(decoded_messages(bridge.executor_request_publisher)) == previous_count
            clock[0] += .001
            before = len(bridge.motion_status_publisher.messages)
            bridge._check_atomic_dwell()
            for message in bridge.motion_status_publisher.messages[before:]:
                Node._motion_status_callback(node, message)
        following = decoded_messages(bridge.executor_request_publisher)[-1]
        assert following['motion_id'] == motion_id(command['action'])
        status(node, bridge, following, 'RUNNING')
        assert node.general_motion_gate.active_command_id == command['command_id']
        assert node.ball_post_motion_dwell_until is None
        assert getattr(node, 'correction_post_motion_dwell_until', None) is None
        running = following


def test_scoped_duration_matches_actual_catalog():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((root/'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for identifier, name in aliases.items():
        if identifier.startswith('ball_align_'):
            m = catalog[name]
            expected = max(f['start_ms']+f['time_ms'] for f in m['frames']) / 1000 * m['repeat_count'] / m['playback_speed']
            assert duration_sec(identifier) == pytest.approx(expected)


def start_fine_prepare(node, bridge, clock):
    fresh(node, clock, 'ball', ball())
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_APPROACH_PREPARE_FINE'
    request = send(bridge, command)
    status(node, bridge, request, 'RUNNING')
    return request


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('bad', ['lost', 'raw_only', 'stale_depth', 'invalid_geometry', 'old_capture', 'future_capture', 'pickup_boundary'])
def test_no_blind_forward_reserved_during_preparation(clock, index, bad):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    request = start_fine_prepare(node, bridge, clock)
    started = node.ball_fine_started_at
    clock[0] = started + .04
    info = ball()
    if bad == 'lost': info.update(detected=False, raw_detected=False)
    if bad == 'raw_only': info.update(detected=False, raw_detected=True, confirmation_confirmed=False)
    if bad == 'stale_depth': info['depth_age_sec'] = 1.
    if bad == 'invalid_geometry': info['ground_projection_valid'] = False
    if bad == 'pickup_boundary': info = ball(distance=.500)
    if bad in {'old_capture', 'future_capture'}:
        from test_wait_refresh_and_fine_settle import observe
        info['rgb_stamp_ns'] = round((started - .1 if bad == 'old_capture' else clock[0] + 1.) * 1e9)
        observe(node, clock, 'ball', info, plan=False)
    else:
        fresh(node, clock, 'ball', info, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    # The bridge does not keep an old forward request behind this preparation.
    clock[0] = started + duration_sec(request['motion_id'])
    status(node, bridge, request, 'SUCCEEDED')
    assert len(decoded_messages(bridge.executor_request_publisher)) == 1
    assert bridge.pending_turn_request is None


def test_failed_preparation_blocks_forward_even_after_fresh_detection(clock):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    request = start_fine_prepare(node, bridge, clock)
    status(node, bridge, request, 'FAILED')
    before = len(node.publisher.messages)
    for _ in range(4):
        fresh(node, clock, 'ball', ball())
    assert node.ball_fine_faulted
    assert not any(m['valid'] for m in node.publisher.messages[before:])


def test_direction_change_during_preparation_selects_turn_support_instead_of_forward(clock):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    request = start_fine_prepare(node, bridge, clock)
    clock[0] = node.ball_fine_started_at + .04
    fresh(node, clock, 'ball', ball(-35.), plan=False)
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_APPROACH_PREPARE_TURN'
    queued = send(bridge, command)
    assert queued['motion_id'] == 'ball_align_prepare_turn_45'
    assert not bridge.queued_request_deferred


def test_distinct_recent_frames_required_before_reserving_turn(clock):
    from test_wait_refresh_and_fine_settle import observe
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball(-35., 1.2), plan=False)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .8
    info = {**ball(-35.), 'rgb_stamp_ns': round(clock[0] * 1e9)}
    for _ in range(5):
        observe(node, clock, 'ball', info, plan=False)
    assert node.queued_general_command_id is None
    for _ in range(2):
        fresh(node, clock, 'ball', ball(-35.), plan=False)
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('failure', ['FAILED', 'CANCELLED', 'TIMEOUT', 'REJECTED'])
def test_failed_forward_discards_deferred_support_change(clock, index, failure):
    from test_motion_command_bridge import executor_status
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    _, running = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball(0., 1.2), plan=False)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .8
    for _ in range(3): fresh(node, clock, 'ball', ball(), plan=False)
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_APPROACH_PREPARE_FINE'
    send(bridge, command)
    assert bridge.queued_request_deferred
    assert bridge.queued_command_id == command['command_id']
    assert len(bridge.executor_request_publisher.messages) == 1
    status(node, bridge, running, failure)
    assert not bridge.motion_in_progress
    assert bridge.queued_request_id is None
    assert bridge.transition_pending_request is None
    assert bridge.transition_dwell_until is None
    # A late success from the failed walk cannot revive the discarded reservation.
    clock[0] += 1.
    bridge.executor_status_callback(executor_status(**running, status='SUCCEEDED'))
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('distance', [.49, .5, .600001, .9])
def test_other_distance_bands_do_not_gain_alignment_override(distance):
    command = MotionDecisionPlanner()._plan_source('ball', ball(-35., distance), .1)
    assert command.get('ball_fine_alignment') is not True
