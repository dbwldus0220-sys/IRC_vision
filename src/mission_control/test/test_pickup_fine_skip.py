"""Skip pickup fine advance only from successful gait history and fresh pixels."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, complete_active_motion,
    continue_pickup_after_backward, executor_status,
)
from test_pickup_continuous_walk import ball


@pytest.mark.parametrize('bottom,action', [
    (229.999, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (230, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (230.001, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_default_checkpoint_includes_230_boundary(bottom, action):
    assert MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball(bottom_distance_px=bottom)).action == action


@pytest.mark.parametrize('robot', [False, True])
def test_both_launch_defaults_use_230_and_accept_override(monkeypatch, tmp_path, robot):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from test_full_system_launch import launch_description
    from test_full_system_robot_launch import launch_description as robot_description
    description = (robot_description if robot else launch_description)(monkeypatch, tmp_path)
    for value in (None, '240'):
        context = LaunchContext()
        if value is not None:
            context.launch_configurations['pickup_fine_align_bottom_distance_px'] = value
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        assert context.launch_configurations['pickup_fine_align_bottom_distance_px'] == (value or '230')


@pytest.mark.parametrize('motion,limit', [
    ('pickup_fine_forward_0', 230), ('ball_general_fine_forward_8', 230),
    ('ball_approach_fine_forward_45_5', 230),
    ('pickup_crab_left_0', 270), ('pickup_crab_right_0', 270),
    ('pickup_crab_left_0_2', 270), ('pickup_crab_right_0_2', 270),
])
@pytest.mark.parametrize('delta,action', [
    (-.001, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (0, 'BALL_PICKUP_FINE_FORWARD'), (.001, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_gait_limits_override_old_latched_distance(motion, limit, delta, action):
    if limit == 230 and delta == 0:
        action = 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    planner = MotionDecisionPlanner()
    planner.pickup_last_completed_motion_id = motion
    planner.pickup_fine_approach_complete = True
    assert planner.plan_ball_pickup_fine_alignment(
        ball(bottom_distance_px=limit + delta)).action == action


@pytest.mark.parametrize('motion,bottom', [
    ('pickup_fine_forward_0', 219), ('pickup_crab_right_0', 269),
])
@pytest.mark.parametrize('changes', [
    {'detected': False, 'raw_detected': False}, {'confidence': .1},
    {'bottom_distance_px': None}, {'bottom_distance_px': -1},
    {'bottom_distance_px': float('nan')}, {'bottom_distance_px': True},
    {'offset_x_px': None}, {'offset_x_px': -31}, {'offset_x_px': 56},
])
def test_unaligned_or_invalid_ball_never_starts_grasp_tail(motion, bottom, changes):
    planner = MotionDecisionPlanner()
    planner.pickup_last_completed_motion_id = motion
    decision = planner.plan_ball_pickup_fine_alignment(
        ball(**{'bottom_distance_px': bottom, **changes}))
    assert decision.action != 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


def send_command(node, bridge):
    before = len(bridge.motion_status_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)


def initial_checkpoint(node, bridge, clock, sample):
    fresh(node, clock, 'ball', sample)
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
    send_command(node, bridge)
    assert node.planner.pickup_last_completed_motion_id == bridge.last_physical_motion_id
    fresh(node, clock, 'ball', sample)
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
    send_command(node, bridge)


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('motion,bottom,preparation', [
    ('pickup_fine_forward_0', 230, None),
    ('pickup_crab_left_0', 269, 'pickup_crab_left_to_fine_0'),
    ('pickup_crab_right_0_2', 269, 'pickup_crab_right_to_fine_0'),
])
def test_both_modes_skip_forward_but_keep_retreat_and_grasp(clock, index, motion, bottom, preparation):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge.last_physical_motion_id = motion
    sample = ball(bottom_distance_px=bottom)
    initial_checkpoint(node, bridge, clock, sample)
    assert bridge.pickup_fine_align_waiting
    assert not bridge.executor_request_publisher.messages
    # Checkpoint requires a new observation; the previous aligned frame is cleared.
    assert node.latest_info['ball'] is None
    fresh(node, clock, 'ball', sample)
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    send_command(node, bridge)
    if preparation:
        assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == preparation
        bridge.executor_status_callback(executor_status(
            status='SUCCEEDED', motion_id=preparation,
            command_id=bridge.active_command_id, event_id=bridge.active_event_id,
            request_id=bridge.active_request_id,
        ))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r['motion_id'] for r in requests] == (
        ([preparation] if preparation else []) + ['pickup_pre_backward_camera_down'])
    assert bridge.pickup_fixed_sequence_started
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(float('inf'))
    assert bridge.pickup_post_backward_align_waiting
    continue_pickup_after_backward(bridge, bottom_distance_px=400)
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_400'
    exit_motion = ('pickup_first_backward_turn_right' if index == 1
                   else 'pickup_second_backward_turn_left')
    assert exit_motion in bridge.active_pickup_sequence


@pytest.mark.parametrize('motion,bottom', [
    ('pickup_fine_forward_0', 230.001), ('pickup_crab_right_0', 270),
    ('line_forward_4', 200), (None, 200),
])
def test_initial_step_remains_when_skip_condition_is_not_met(clock, motion, bottom):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = motion
    initial_checkpoint(node, bridge, clock, ball(bottom_distance_px=bottom))
    assert not bridge.pickup_fine_align_waiting
    assert bridge.executor_request_publisher.messages


def test_skip_candidate_rechecks_alignment_before_retreat(clock):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_crab_right_0'
    initial_checkpoint(node, bridge, clock, ball(bottom_distance_px=250))
    fresh(node, clock, 'ball', ball(bottom_distance_px=250, offset_x_px=-100))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_CRAB_LEFT'
    assert not bridge.pickup_fixed_sequence_started


def test_failed_crab_support_change_cannot_start_retreat_or_grasp(clock):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_crab_right_0'
    sample = ball(bottom_distance_px=250)
    initial_checkpoint(node, bridge, clock, sample)
    fresh(node, clock, 'ball', sample)
    send_command(node, bridge)
    complete_active_motion(bridge, 'FAILED', 'SDK_POSITION_TIMEOUT')
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r['motion_id'] for r in requests] == ['pickup_crab_right_to_fine_0']
    assert not bridge.motion_in_progress
    assert bridge.last_physical_motion_id is None


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('offset,bottom', [(0, 230), (100, 269)])
def test_completed_motion_status_drives_skip_in_both_modes(clock, index, offset, bottom):
    from test_pickup_continuous_walk import start, forward_status
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge.last_physical_motion_id = ('pickup_crab_right_0' if offset else 'pickup_fine_forward_0')
    _, request = start(node, bridge, clock, ball(offset_x_px=offset))
    forward_status(node, bridge, request, 'SUCCEEDED')
    if bridge.active_dwell_until is not None:
        clock[0] = bridge.active_dwell_until
        before = len(bridge.motion_status_publisher.messages)
        bridge._check_atomic_dwell(clock[0])
        for message in bridge.motion_status_publisher.messages[before:]:
            Node._motion_status_callback(node, message)
    assert node.planner.pickup_last_completed_motion_id == request['motion_id']
    fresh(node, clock, 'ball', ball(bottom_distance_px=bottom))
    if bridge.pickup_initial_align_waiting:
        assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
        count = len(bridge.executor_request_publisher.messages)
        send_command(node, bridge)
        assert len(bridge.executor_request_publisher.messages) == count
        fresh(node, clock, 'ball', ball(bottom_distance_px=bottom))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


def test_new_checkpoint_clears_history_when_last_motion_failed(clock):
    from test_pickup_continuous_walk import start, forward_status
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_crab_right_0'
    _, request = start(node, bridge, clock, ball(offset_x_px=100))
    forward_status(node, bridge, request, 'FAILED', error_code='SDK_POSITION_TIMEOUT')
    assert bridge.last_physical_motion_id is None
    assert node.planner.pickup_last_completed_motion_id is None
    fresh(node, clock, 'ball', ball(bottom_distance_px=250))
    send_command(node, bridge)
    assert not bridge.pickup_fine_align_waiting
    assert not bridge.pickup_fixed_sequence_started


@pytest.mark.parametrize('bottom,reserved', [(229, False), (230, False), (230.001, True)])
def test_reservation_uses_current_gait_without_mutating_completed_history(bottom, reserved):
    from mission_control.pickup_walk_reservation import PickupWalkReservation
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    planner.pickup_last_completed_motion_id = 'pickup_crab_right_0'
    planner.pickup_fine_approach_complete = True
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=1,
                               motion_id='pickup_fine_forward_0',
                               pickup_alignment_stage='fine'), 10., 10_000_000_000)
    for stamp in (11_500_000_000, 11_550_000_000, 11_600_000_000):
        reservation.observe(planner, ball(bottom_distance_px=bottom, rgb_stamp_ns=stamp),
                            stamp / 1e9, stamp, .5)
    assert (reservation.ready is not None) is reserved
    assert planner.pickup_last_completed_motion_id == 'pickup_crab_right_0'
