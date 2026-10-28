"""Replay Line -> one Ball-owned Line motion -> Ball tracking without motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


def node_with_entry(enabled=True):
    node = LiveInputHarness(phase='AUTO')
    node.ball_entry_line_alignment = BallEntryLineAlignment(enabled=enabled)
    return node


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
    (0., 0., 'STRAIGHT'), (-20., -.3, 'RECOVER_LEFT_TURN_LEFT_4'),
    (20., .3, 'RECOVER_RIGHT_TURN_RIGHT_4'),
])
def test_both_walking_boundaries_prequeue_one_command_without_transition_dwell(clock, heading, offset, action):
    node, bridge = node_with_entry(), FakeBridge()
    first, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2, bearing_deg=30), plan=False)
    assert node.mission_phase == 'BALL_APPROACH'
    assert node.ball_entry_line_alignment.pending
    late_line(node, clock, line_info(heading, offset))
    entry = node.publisher.messages[-1]
    assert entry['valid'] and entry['action'] == action
    assert entry['source'] == 'ball' and entry['phase'] == 'BALL_APPROACH'
    assert entry['source_command']['ball_entry_line_alignment']
    assert node.queued_general_command_id == entry['command_id']
    entry_request = send(bridge, entry)
    assert len(bridge.executor_request_publisher.messages) == 2
    if action == 'STRAIGHT':
        assert entry_request['motion_id'] == 'line_forward_6'
    assert not bridge.queued_request_deferred
    status(node, bridge, request, 'SUCCEEDED')
    status(node, bridge, entry_request, 'RUNNING')
    assert Node._ball_entry_line_running(node)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    # The latest Ball angle has changed while the entry motion is executing.
    for _ in range(4):
        fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=0.), plan=False)
    follow = node.publisher.messages[-1]
    assert follow['valid'] and follow['source'] == 'ball' and follow['action'] == 'STRAIGHT'
    assert not follow['source_command'].get('ball_entry_line_alignment')
    assert follow['command_id'] != entry['command_id']
    follow_request = send(bridge, follow)
    assert follow_request['motion_id'] == 'line_forward_4'
    assert len(bridge.executor_request_publisher.messages) == 3
    assert not bridge.queued_request_deferred
    status(node, bridge, entry_request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.state == 'DONE'
    assert bridge.transition_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    status(node, bridge, follow_request, 'RUNNING')
    assert node.general_motion_gate.active_command_id == follow['command_id']
    assert len([m for m in node.publisher.messages if m['valid']]) == 3


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
    assert next_command['action'] == ('STRAIGHT' if enabled else 'BALL_APPROACH_TURN_RIGHT_3')
    assert bool(next_command['source_command'].get('ball_entry_line_alignment')) is enabled


def test_already_accepted_line_reservation_is_adopted_without_extra_line_motion(clock):
    node, bridge = node_with_entry(), FakeBridge()
    _, first_request = start_line(node, bridge, clock)
    late_line(node, clock, line_info())
    queued = node.publisher.messages[-1]
    queued_request = send(bridge, queued)
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
    fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=-30), plan=False)
    status(node, bridge, queued_request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'BALL_APPROACH_TURN_LEFT_2'
    assert node.ball_entry_line_alignment.state == 'DONE'
    assert len([m for m in node.publisher.messages if m['valid']]) == 3


@pytest.mark.parametrize('changes', [{'detected': False}, {'ground_projection_valid': False}])
def test_invalid_line_waits_without_consuming_the_single_alignment(clock, changes):
    node = node_with_entry()
    fresh(node, clock, 'line', {**line_info(), **changes}, plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    assert node.ball_entry_line_alignment.pending
    assert not any(m['valid'] for m in node.publisher.messages)
    fresh(node, clock, 'line', line_info())
    assert node.publisher.messages[-1]['source_command']['ball_entry_line_alignment']


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


def test_completed_entry_is_not_rearmed_by_ball_loss_or_duplicates_but_next_ball_can_arm():
    entry = BallEntryLineAlignment()
    assert entry.arm()
    entry.published(10)
    entry.on_status(9, 'SUCCEEDED')
    assert entry.state == 'QUEUED'
    entry.on_status(10, 'RUNNING')
    entry.on_status(10, 'SUCCEEDED')
    entry.on_status(10, 'RUNNING')
    assert entry.state == 'DONE' and not entry.arm()
    entry.reset()
    assert entry.arm() and entry.pending


def test_close_ball_still_gets_one_line_motion_then_enters_existing_pickup(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.5))
    entry = node.publisher.messages[-1]
    assert entry['action'] == 'STRAIGHT' and entry['source_command']['ball_entry_line_alignment']
    request = send(bridge, entry)
    status(node, bridge, request, 'RUNNING')
    fresh(node, clock, 'ball', ball_info(depth_m=.5), plan=False)
    status(node, bridge, request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'


@pytest.mark.parametrize('value', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failure_of_preceding_line_discards_reserved_entry_without_skipping_it(clock, value):
    node, bridge = node_with_entry(), FakeBridge()
    _, first_request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    late_line(node, clock, line_info())
    entry = node.publisher.messages[-1]
    send(bridge, entry)
    assert node.ball_entry_line_alignment.state == 'QUEUED'
    status(node, bridge, first_request, value)
    assert node.queued_general_command_id is None
    assert node.ball_entry_line_alignment.state == 'FAILED'
    before = len(node.publisher.messages)
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    assert not any(m['valid'] for m in node.publisher.messages[before:])


def test_line_offset_policy_retains_short_forward_count_in_ball_mode(clock):
    from test_line_offset_alignment import sample
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', sample(target_angle=0., ground_heading_error_deg=-30.), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    entry = node.publisher.messages[-1]
    assert entry['action'] == 'STRAIGHT_1'
    assert entry['source'] == 'ball' and entry['source_command']['line_offset_alignment_active']
    request = send(bridge, entry)
    assert request['motion_id'] == 'line_forward_2'
    status(node, bridge, request, 'RUNNING')
    assert node.active_line_motion_duration_sec == Node.LINE_MOTION_CAPTURE_CONFIG['STRAIGHT_1'][0]


def test_stationary_line_alignment_executes_once_and_preserves_existing_settle(clock):
    from test_line_offset_alignment import sample
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', sample(target_angle=25., ground_heading_error_deg=-20.), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2))
    entry = node.publisher.messages[-1]
    assert entry['action'] == 'LINE_OFFSET_TURN_RIGHT_2'
    assert entry['source'] == 'ball' and entry['source_command']['ball_entry_line_alignment']
    request = send(bridge, entry)
    status(node, bridge, request, 'RUNNING')
    for _ in range(4):
        fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.state == 'DONE'
    assert node.correction_post_motion_dwell_until > clock[0]
    clock[0] = node.correction_post_motion_dwell_until + .01
    # The existing settle gate clears old input on its first expired tick.
    Node._publish_decision(node)
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=-30.))
    assert node.publisher.messages[-1]['action'] == 'BALL_APPROACH_TURN_LEFT_2'
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
