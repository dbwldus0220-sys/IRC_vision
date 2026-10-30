"""Replay Ball-owned Line approach until 550 mm without motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


def node_with_entry(enabled=True):
    node = LiveInputHarness(phase='AUTO')
    node.ball_entry_line_alignment = BallEntryLineAlignment(enabled=enabled)
    node.queued_general_action = None
    node.queued_general_command_id = None
    node.queued_general_source = None
    return node


def ground_ball_info(**overrides):
    return ball_info(**{
        'ground_projection_enabled': True, 'ground_projection_valid': True,
        'ground_projection_scope': 'ball_approach_only',
        'ground_coordinate_frame': 'robot_x_right_z_forward',
        'ground_steering_angle_deg': 0., 'ground_forward_distance_m': .4,
        **overrides,
    })


def send(bridge, command):
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    return decoded_messages(bridge.executor_request_publisher)[-1]


def status(node, bridge, request, value):
    bridge.executor_status_callback(executor_status(**request, status=value))
    Node._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])


def start_line(node, bridge, clock):
    fresh(node, clock, 'line', line_info())
    command = node.publisher.messages[-1]
    request = send(bridge, command)
    status(node, bridge, request, 'RUNNING')
    return command, request


def late_line(node, clock, info):
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    for _ in range(16):
        fresh(node, clock, 'line', info, plan=False)


@pytest.mark.parametrize('heading,offset,action', [
    (0., 0., 'STRAIGHT_2'), (-15., -.3, 'STRAIGHT_2'), (15., .3, 'STRAIGHT_2'),
    (-30., .3, 'RECOVER_RIGHT_TURN_LEFT_4'), (30., -.3, 'RECOVER_LEFT_TURN_RIGHT_4'),
])
def test_normal_line_and_ball_line_steps_reserve_walks_without_waits(clock, heading, offset, action):
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_line(node, bridge, clock)
    assert request['motion_id'] == 'line_forward_6'
    fresh(node, clock, 'ball', ball_info(depth_m=1.2, bearing_deg=30), plan=False)
    assert node.mission_phase == 'BALL_APPROACH'
    late_line(node, clock, line_info(heading, offset))
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    entry = node.publisher.messages[-1]
    assert node.queued_general_command_id == entry['command_id']
    assert node.general_motion_gate.active_command_id != entry['command_id']
    assert entry['valid'] and entry['action'] == action
    assert entry['source'] == 'ball' and entry['source_command']['ball_entry_line_alignment']
    entry_request = send(bridge, entry)
    assert entry_request['motion_id'] == ('line_forward_4' if action == 'STRAIGHT_2'
                                        else 'line_recovery_left_4' if 'TURN_LEFT' in action
                                        else 'line_recovery_right_4')
    assert not bridge.queued_request_deferred
    status(node, bridge, request, 'SUCCEEDED')
    status(node, bridge, entry_request, 'RUNNING')
    if action == 'STRAIGHT_2':
        assert node.active_line_motion_duration_sec == pytest.approx(.517 * 4 / .95)
    assert Node._ball_entry_line_running(node)
    late_line(node, clock, line_info(heading, offset))
    # Recovery-to-recovery reserves only near its end, as in ordinary Line.
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec - .2
    fresh(node, clock, 'line', line_info(heading, offset), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=-60.), plan=False)
    follow = node.publisher.messages[-1]
    assert follow['valid'] and follow['action'] == action
    assert follow['source_command']['ball_entry_line_alignment']
    assert follow['command_id'] != entry['command_id']
    assert node.queued_general_command_id == follow['command_id']
    assert node.ball_entry_line_alignment.command_id == entry['command_id']
    assert node.ball_entry_line_alignment.queued_command_id == follow['command_id']
    follow_request = send(bridge, follow)
    assert follow_request['motion_id'] == entry_request['motion_id']
    assert not bridge.queued_request_deferred
    status(node, bridge, entry_request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.state == 'QUEUED'
    status(node, bridge, follow_request, 'RUNNING')
    assert Node._ball_entry_line_running(node)
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert getattr(bridge, 'transition_dwell_until', None) is None


@pytest.mark.parametrize('enabled', [True, False])
def test_option_restores_direct_ball_entry_at_current_motion_completion(clock, enabled):
    node, bridge = node_with_entry(enabled), FakeBridge()
    _, request = start_line(node, bridge, clock)
    clock[0] += .1
    fresh(node, clock, 'ball', ball_info(depth_m=1.2, bearing_deg=30), plan=False)
    fresh(node, clock, 'line', line_info(), plan=False)
    status(node, bridge, request, 'SUCCEEDED')
    next_command = node.publisher.messages[-1]
    assert next_command['source'] == 'ball'
    assert next_command['action'] == ('STRAIGHT_2' if enabled else 'BALL_APPROACH_TURN_RIGHT_3')
    assert bool(next_command['source_command'].get('ball_entry_line_alignment')) is enabled


def test_already_accepted_line_reservation_is_adopted_then_line_control_repeats(clock):
    node, bridge = node_with_entry(), FakeBridge()
    _, first_request = start_line(node, bridge, clock)
    late_line(node, clock, line_info())
    queued = node.publisher.messages[-1]
    queued_request = send(bridge, queued)
    assert queued_request['motion_id'] == 'line_forward_6'
    assert node.queued_general_command_id == queued['command_id']
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    assert node.ball_entry_line_alignment.command_id == queued['command_id']
    assert node.mission_phase == 'BALL_APPROACH'
    ownership = node.publisher.messages[-1]
    assert ownership['source'] == 'ball' and ownership['phase'] == 'BALL_APPROACH'
    assert ownership['valid'] is False and ownership['action'] == queued['action']
    assert ownership['source_command']['adopted_command_id'] == queued['command_id']
    count = len(bridge.executor_request_publisher.messages)
    send(bridge, ownership)
    assert len(bridge.executor_request_publisher.messages) == count
    assert node.queued_general_command_id == queued['command_id']
    status(node, bridge, first_request, 'SUCCEEDED')
    status(node, bridge, queued_request, 'RUNNING')
    assert Node._ball_entry_line_running(node)
    clock[0] += .2
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=-30), plan=False)
    status(node, bridge, queued_request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_2'
    assert node.publisher.messages[-1]['source_command']['ball_entry_line_alignment']
    assert node.ball_entry_line_alignment.state == 'QUEUED'
    assert len([m for m in node.publisher.messages if m['valid']]) == 3


@pytest.mark.parametrize('changes', [
    {'detected': False}, {'ground_projection_valid': False}, {'geometry_quality': .1},
])
def test_invalid_line_at_570mm_uses_ground_ball_forward(clock, changes):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', {**line_info(), **changes}, plan=False)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.570))
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == 'STRAIGHT_2'
    assert command['source_command']['ball_entry_ground_tracking']
    assert command['source_command']['heading_error_deg'] == 0.
    request = send(bridge, command)
    assert request['motion_id'] == 'line_forward_4'
    status(node, bridge, request, 'RUNNING')
    fresh(node, clock, 'ball', ball_info(depth_m=.550), plan=False)
    assert node.ball_pickup_entry_pending
    status(node, bridge, request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('value', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_entry_does_not_start_tracking_or_another_line_motion(clock, value):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    entry = node.publisher.messages[-1]
    request = send(bridge, entry)
    status(node, bridge, request, 'RUNNING')
    status(node, bridge, request, value)
    assert node.ball_entry_line_alignment.state == 'FAILED'
    before = len(node.publisher.messages)
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    assert not any(m['valid'] for m in node.publisher.messages[before:])


def test_completed_step_rearms_once_but_finished_approach_requires_next_ball_reset():
    entry = BallEntryLineAlignment()
    assert entry.arm()
    entry.published(10)
    entry.on_status(9, 'SUCCEEDED')
    assert entry.state == 'QUEUED'
    entry.on_status(10, 'RUNNING')
    entry.finish()
    assert entry.state == 'RUNNING'
    entry.on_status(10, 'SUCCEEDED')
    assert entry.pending
    entry.published(11)
    entry.on_status(10, 'RUNNING')
    entry.on_status(10, 'SUCCEEDED')
    assert entry.state == 'QUEUED' and entry.command_id == 11
    entry.on_status(11, 'SUCCEEDED')
    entry.finish()
    assert entry.state == 'DONE' and not entry.arm()
    entry.reset()
    assert entry.arm() and entry.pending


def test_first_close_ball_skips_line_step_and_enters_existing_pickup(clock):
    node = node_with_entry()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.5))
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
    assert node.ball_entry_line_alignment.state == 'DONE'
    assert not any(m['source_command'].get('ball_entry_line_alignment') for m in node.publisher.messages)


@pytest.mark.parametrize('first_heading,next_heading', [(0., -30.), (-30., 0.), (-30., 30.)])
def test_ball_line_walk_transitions_are_sent_before_current_completion(clock, first_heading, next_heading):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(first_heading), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    first = node.publisher.messages[-1]
    request = send(bridge, first)
    status(node, bridge, request, 'RUNNING')
    late_line(node, clock, line_info(next_heading))
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec - .2
    fresh(node, clock, 'line', line_info(next_heading), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] == node.queued_general_command_id
    assert queued['command_id'] != first['command_id']
    next_request = send(bridge, queued)
    expected = ('line_forward_4' if next_heading == 0. else
                'line_recovery_left_4' if next_heading < 0. else 'line_recovery_right_4')
    assert next_request['motion_id'] == expected
    assert not bridge.queued_request_deferred
    assert bridge.active_command_id == first['command_id']


def test_close_after_reservation_finishes_accepted_walk_then_enters_pickup_without_dwell(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    first_request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, first_request, 'RUNNING')
    late_line(node, clock, line_info())
    fresh(node, clock, 'ball', ball_info(depth_m=.8), plan=False)
    queued = node.publisher.messages[-1]
    queued_request = send(bridge, queued)
    assert node.ball_entry_line_alignment.queued_command_id == queued['command_id']
    fresh(node, clock, 'ball', ball_info(depth_m=.55), plan=False)
    assert node.ball_pickup_entry_pending
    status(node, bridge, first_request, 'SUCCEEDED')
    status(node, bridge, queued_request, 'RUNNING')
    assert Node._ball_entry_line_running(node)
    late_line(node, clock, line_info())
    # The close crossing stays latched even if the next depth is slightly larger.
    fresh(node, clock, 'ball', ball_info(depth_m=.57), plan=False)
    assert node.queued_general_command_id is None
    completed_at = clock[0]
    status(node, bridge, queued_request, 'SUCCEEDED')
    pickup = node.publisher.messages[-1]
    assert pickup['action'] == 'PICKUP_NOW' and pickup['valid']
    assert node.ball_entry_line_alignment.state == 'DONE'
    bridge.navigation_command_callback(String(data=json.dumps(pickup)))
    assert clock[0] == completed_at
    assert bridge.pickup_initial_align_waiting
    assert bridge.pickup_initial_align_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert len(bridge.executor_request_publisher.messages) == 2


@pytest.mark.parametrize('value', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_running_entry_discards_its_reserved_successor(clock, value):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    late_line(node, clock, line_info())
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    send(bridge, node.publisher.messages[-1])
    assert node.ball_entry_line_alignment.queued_command_id is not None
    status(node, bridge, request, value)
    assert node.ball_entry_line_alignment.state == 'FAILED'
    assert node.ball_entry_line_alignment.queued_command_id is None
    assert node.queued_general_command_id is None
    before = len([m for m in node.publisher.messages if m['valid']])
    fresh(node, clock, 'ball', ball_info(depth_m=.5))
    assert len([m for m in node.publisher.messages if m['valid']]) == before


def test_rejected_successor_does_not_release_running_entry(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    late_line(node, clock, line_info())
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    queued_request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, queued_request, 'REJECTED')
    assert node.ball_entry_line_alignment.state == 'FAILED'
    assert node.general_motion_gate.locked
    assert node.general_motion_gate.active_command_id == request['request_id']
    assert node.queued_general_command_id is None
    status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.state == 'FAILED'


@pytest.mark.parametrize('change', ['expired', 'future', 'lost', 'turn', 'pickup', 'invalid_depth'])
def test_entry_reservation_requires_current_far_ball_and_walking_line_action(clock, change):
    from test_wait_refresh_and_fine_settle import observe
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    late_line(node, clock, line_info(heading=60. if change == 'turn' else 0.))
    updates = {'depth_m': .55 if change == 'pickup' else .9}
    if change == 'lost':
        updates.update(detected=False, raw_detected=False)
    elif change == 'invalid_depth':
        updates.update(depth_valid=False)
    info = ball_info(**updates)
    if change in {'expired', 'future'}:
        stamp = clock[0] + (1. if change == 'future' else -1.)
        observe(node, clock, 'ball', {**info, 'rgb_stamp_ns': round(stamp * 1e9)}, plan=False)
    else:
        fresh(node, clock, 'ball', info, plan=False)
    assert node.queued_general_command_id is None


def test_stationary_entry_turn_keeps_pre_and_post_stops(clock):
    node, bridge = node_with_entry(), FakeBridge()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    _, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    late_line(node, clock, line_info(heading=60.))
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    assert node.queued_general_command_id is None
    status(node, bridge, request, 'SUCCEEDED')
    assert len([m for m in node.publisher.messages if m['valid']]) == 1
    assert node.pre_motion_settle_started_at is not None
    clock[0] = node.pre_motion_settle_started_at + node.LINE_TURN_PRE_MOTION_SETTLE_SEC + .01
    fresh(node, clock, 'line', line_info(heading=60.), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    turn = node.publisher.messages[-1]
    assert turn['action'] == 'LINE_HEADING_TURN_RIGHT_6'
    turn_request = send(bridge, turn)
    status(node, bridge, turn_request, 'RUNNING')
    fresh(node, clock, 'ball', ball_info(depth_m=.55), plan=False)
    status(node, bridge, turn_request, 'SUCCEEDED')
    assert node.correction_post_motion_dwell_until > clock[0]
    before = len([m for m in node.publisher.messages if m['valid']])
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.55))
    assert len([m for m in node.publisher.messages if m['valid']]) == before


@pytest.mark.parametrize('value', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failure_of_preceding_line_blocks_pending_ball_line_approach(clock, value):
    node, bridge = node_with_entry(), FakeBridge()
    _, first_request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    late_line(node, clock, line_info())
    assert node.ball_entry_line_alignment.pending
    assert getattr(node, 'queued_general_command_id', None) is None
    status(node, bridge, first_request, value)
    assert node.ball_entry_line_alignment.state == 'FAILED'
    before = len(node.publisher.messages)
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    assert not any(m['valid'] for m in node.publisher.messages[before:])


def test_ball_entry_uses_own_bins_instead_of_normal_line_offset_override(clock):
    from test_line_offset_alignment import sample
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', sample(target_angle=0., ground_heading_error_deg=-30.), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    entry = node.publisher.messages[-1]
    assert entry['action'].endswith('TURN_LEFT_4')
    assert entry['source'] == 'ball' and entry['source_command']['ball_entry_line_alignment']
    request = send(bridge, entry)
    assert request['motion_id'] == 'line_recovery_left_4'


def test_stationary_ball_line_alignment_repeats_after_settle_and_fresh_frames(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(heading=60.), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    entry = node.publisher.messages[-1]
    assert entry['action'] == 'LINE_HEADING_TURN_RIGHT_6'
    request = send(bridge, entry)
    status(node, bridge, request, 'RUNNING')
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.pending
    assert node.correction_post_motion_dwell_until > clock[0]
    clock[0] = node.correction_post_motion_dwell_until + .01
    Node._publish_decision(node)
    fresh(node, clock, 'line', line_info(heading=-45.), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=0.))
    assert node.publisher.messages[-1]['action'] == 'LINE_HEADING_TURN_LEFT_3'
    assert node.publisher.messages[-1]['source_command']['ball_entry_line_alignment']
    assert len([m for m in node.publisher.messages if m['valid']]) == 2


@pytest.mark.parametrize('changes', [
    {'depth_m': 1.501}, {'confidence': .1}, {'depth_valid': False},
    {'detected': False, 'raw_detected': True},
])
def test_entry_retains_existing_ball_confirmation_and_range_checks(clock, changes):
    node = node_with_entry()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(**{'depth_m': 1.2, **changes}))
    assert node.ball_entry_line_alignment.state == 'IDLE'
    assert not any(m['source_command'].get('ball_entry_line_alignment') for m in node.publisher.messages)


@pytest.mark.parametrize('change', ['expired', 'future', 'lost', 'turn', 'pickup'])
def test_entry_never_reserves_ball_forward_for_unusable_or_nonwalking_next_decision(clock, change):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    clock[0] += node.active_line_motion_duration_sec * .75
    for _ in range(4):
        updates = {'detected': False, 'raw_detected': False} if change == 'lost' else {}
        updates.update(depth_m=.5 if change == 'pickup' else .9, bearing_deg=30 if change == 'turn' else 0.)
        info = ball_info(**updates)
        if change in {'expired', 'future'}:
            from test_wait_refresh_and_fine_settle import observe
            stamp = clock[0] + (1. if change == 'future' else -1.)
            observe(node, clock, 'ball', {**info, 'rgb_stamp_ns': round(stamp * 1e9)}, plan=False)
        else:
            fresh(node, clock, 'ball', info, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'BALL_SEARCH', 'BALL_APPROACH'])
def test_entry_is_ball_mode_from_all_supported_initial_line_or_ball_phases(clock, phase):
    node = node_with_entry()
    node.phase_manager.set_phase(phase)
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.5))
    command = node.publisher.messages[-1]
    assert command['phase'] == 'BALL_APPROACH' and command['source'] == 'ball'
    assert command['source_command']['ball_entry_line_alignment']


@pytest.mark.parametrize('angle,kind,count', [
    (0., 'forward', None), (14.999, 'forward', None), (15., 'forward', None),
    (15.001, 'recovery', None), (20., 'recovery', None), (30., 'recovery', None), (44.999, 'recovery', None),
    (45., 'stationary', 5), (59.999, 'stationary', 5), (60., 'stationary', 6),
    (75., 'stationary', 7), (89.999, 'stationary', 7),
])
@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('offset', [-.4, 0., .4])
def test_ball_only_heading_bins_and_existing_recovery_direction(angle, kind, count, sign, offset):
    planner = MotionDecisionPlanner()
    config = planner.line_planner.config
    heading = sign * angle
    decision = BallEntryLineAlignment().select(
        planner, line_info(heading=heading, offset=offset), .1, ball_info(depth_m=1.2))
    direction = 'RIGHT' if heading > 0. else 'LEFT'
    if kind == 'forward':
        assert decision.action == 'STRAIGHT_2'
    elif kind == 'recovery':
        original = planner.line_planner._classify_recovery(offset, heading, False)
        if original is not None:
            assert decision.action == original
        # The new bin requires recovery even where the normal policy skips it.
        # Heading still chooses the same physical left/right recovery catalog.
        assert FakeBridge.ACTION_TO_MOTION_ID[decision.action] == f'line_recovery_{direction.lower()}_4'
    else:
        expected = count if sign > 0 else int(angle // 15)
        assert decision.action == f'LINE_HEADING_TURN_{direction}_{expected}'
        assert decision.source_command['turn_count'] == expected
    assert decision.valid and planner.line_planner.config is config


def test_line_then_ground_ball_approach_until_550mm_waits_for_current_motion(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    for distance in (1.0, .8, .550001, .550):
        command = node.publisher.messages[-1]
        assert command['action'] == 'STRAIGHT_2'
        assert command['source_command']['ball_entry_line_alignment']
        request = send(bridge, command)
        status(node, bridge, request, 'RUNNING')
        before = len([m for m in node.publisher.messages if m['valid']])
        fresh(node, clock, 'line', line_info(), plan=False)
        fresh(node, clock, 'ball', ground_ball_info(depth_m=distance, bearing_deg=60.))
        assert len([m for m in node.publisher.messages if m['valid']]) == before
        assert node.ball_entry_line_alignment.state == 'RUNNING'
        assert getattr(node, 'queued_general_command_id', None) is None
        status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.state == 'DONE'
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
    assert not node.publisher.messages[-1]['source_command'].get('ball_entry_line_alignment')
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == ['line_forward_4'] * 4


@pytest.mark.parametrize('distance', [.549, .550])
def test_close_crossing_during_original_line_motion_skips_entry_step(clock, distance):
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=distance))
    assert node.ball_pickup_entry_pending
    assert len([m for m in node.publisher.messages if m['valid']]) == 1
    status(node, bridge, request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
    assert node.ball_entry_line_alignment.state == 'DONE'


@pytest.mark.parametrize('change', [
    {'confidence': .1}, {'depth_valid': False}, {'depth_age_sec': 10.},
    {'confirmation_confirmed': False}, {'distance_m': float('nan')},
])
def test_invalid_close_distance_cannot_end_ball_line_approach(clock, change):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    fresh(node, clock, 'ball', ball_info(**{'depth_m': .5, **change}), plan=False)
    assert not node.ball_pickup_entry_pending
    status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.pending
    assert node.active_special_command_id is None


def test_missing_ball_waits_then_fresh_ball_allows_forward_without_line(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    clock[0] += max(node.timeouts['ball'], node.timeouts['line']) + .1
    status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.pending
    count = len([m for m in node.publisher.messages if m['valid']])
    fresh(node, clock, 'ball', ball_info(depth_m=.8))
    assert len([m for m in node.publisher.messages if m['valid']]) == count + 1
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_2'
    assert node.publisher.messages[-1]['source_command']['ball_entry_line_fallback']
    assert node.publisher.messages[-1]['source_command']['ball_entry_line_alignment']


def test_fallback_reobserves_ball_without_new_line_and_never_prequeues(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', {**line_info(), 'ground_projection_valid': False}, plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.8))
    first = node.publisher.messages[-1]
    request = send(bridge, first)
    status(node, bridge, request, 'RUNNING')
    status(node, bridge, request, 'SUCCEEDED')
    # Repeated invalid Line input must not reuse the Ball frame that started the gait.
    fresh(node, clock, 'line', {**line_info(), 'ground_projection_valid': False})
    assert len([m for m in node.publisher.messages if m['valid']]) == 1
    fresh(node, clock, 'ball', ball_info(depth_m=.7))
    second = node.publisher.messages[-1]
    assert second['command_id'] != first['command_id']
    assert second['source_command']['ball_entry_line_fallback']
    request = send(bridge, second)
    status(node, bridge, request, 'RUNNING')
    clock[0] += node.active_line_motion_duration_sec * .9
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.570), plan=False)
    assert node.queued_general_command_id is None
    fallback = node.ball_entry_line_alignment.select(
        node.planner, None, .1, ground_ball_info(depth_m=.570))
    assert not Node._walking_prequeue_allowed(node, fallback, {}, clock[0])
    status(node, bridge, request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_2'


@pytest.mark.parametrize('change', [
    {'detected': False}, {'confidence': .1}, {'depth_valid': False},
    {'depth_age_sec': 10.}, {'confirmation_confirmed': False},
    {'distance_m': float('nan')}, {'distance_m': .550}, {'distance_m': .549},
])
def test_missing_line_does_not_bypass_ball_validity_or_pickup_boundary(change):
    decision = BallEntryLineAlignment().select(
        MotionDecisionPlanner(), None, .1, ball_info(**{'depth_m': .570, **change}))
    assert not decision.valid and decision.action == 'STOP'


def test_ball_entry_bins_are_not_replaced_by_normal_corner_policy(clock):
    from test_line_corner_memory import corner_info
    node = node_with_entry()
    fresh(node, clock, 'line', corner_info(heading=10., distance=.1), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == 'STRAIGHT_2'
    assert command['source_command']['ball_entry_line_alignment']


def test_ball_policy_does_not_change_normal_line_offset_selection():
    from test_line_offset_alignment import sample
    info = sample(target_angle=0., ground_heading_error_deg=-30.)
    planner = MotionDecisionPlanner()
    before = planner._plan_source('line', info, .1)
    entry = BallEntryLineAlignment().select(planner, info, .1, ball_info(depth_m=1.2))
    after = planner._plan_source('line', info, .1)
    assert before['motion'] == after['motion'] == 'STRAIGHT_1'
    assert entry.action.endswith('TURN_LEFT_4')


@pytest.mark.parametrize('launch_name', ['full_system', 'full_system_robot'])
@pytest.mark.parametrize('enabled', [None, 'false'])
def test_launch_wires_enabled_default_and_rollback_to_decision_node(monkeypatch, tmp_path, launch_name, enabled):
    import importlib
    from launch_ros.actions import Node as LaunchNode
    from test_full_system_launch import launch_context, node_parameters

    module = importlib.import_module('test_' + launch_name + '_launch')
    description = module.launch_description(monkeypatch, tmp_path)
    overrides = {} if enabled is None else {'ball_entry_line_alignment_enabled': enabled}
    context = launch_context(description, overrides)
    decision_node = next(entity for entity in description.entities
                         if isinstance(entity, LaunchNode)
                         and entity.node_executable == 'motion_decision_node')
    parameters = node_parameters(decision_node, context)
    assert parameters['ball_entry_line_alignment_enabled'] is (enabled is None)
