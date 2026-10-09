"""Pickup aligns before advancing, including the recorded October 9 geometry."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_ball_ground_alignment import ball_sample
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe
from test_motion_command_bridge import (
    FakeBridge, enter_pickup_fine_alignment, fine_alignment_message,
    complete_active_motion, complete_pickup_motion_and_dwell, decoded_messages,
)


def pickup_ball(angle=0., offset=0., bottom=321., **changes):
    return ball_sample(angle, **{
        'ground_forward_distance_m': .536, 'depth_m': .43, 'distance_m': .43,
        'offset_x_px': offset, 'offset_x_norm': offset / 640.,
        'bottom_distance_px': bottom, **changes,
    })


@pytest.mark.parametrize('fine', [False, True])
@pytest.mark.parametrize('sign', [-1, 1])
def test_heading_then_lateral_then_approach(fine, sign):
    planner = MotionDecisionPlanner()
    plan = planner.plan_ball_pickup_fine_alignment if fine else planner.plan_ball_pickup_initial_alignment
    direction = 'LEFT' if sign < 0 else 'RIGHT'
    count = 1 if sign < 0 else 2
    turn_prefix = 'BALL_PICKUP_FINE_TURN' if fine else 'BALL_PICKUP_CAMERA_DOWN_TURN'
    crab_prefix = 'BALL_PICKUP_CRAB' if fine else 'BALL_PICKUP_INITIAL_CRAB'
    assert plan(pickup_ball(sign * 20, sign * 185)).action == f'{turn_prefix}_{direction}_{count}'
    assert plan(pickup_ball(sign * 5, sign * 100)).action == f'{crab_prefix}_{direction}'
    result = plan(pickup_ball(sign * 5, 0))
    assert result.action == ('BALL_PICKUP_FINE_FORWARD' if fine else 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE')


def test_recorded_ball_is_corrected_before_another_fine_forward():
    planner = MotionDecisionPlanner()
    # At the decision checkpoint: 321 px from the bottom, far to screen-left.
    result = planner.plan_ball_pickup_fine_alignment(pickup_ball(-14.05, -185, 321))
    assert result.action == 'BALL_PICKUP_CRAB_LEFT'
    # At the user's screenshot: close enough, but yaw and lateral error remain.
    result = planner.plan_ball_pickup_fine_alignment(pickup_ball(-16.26, -210, 260))
    assert result.action == 'BALL_PICKUP_FINE_TURN_LEFT_1'
    assert planner.pickup_fine_approach_complete
    assert planner.plan_ball_pickup_fine_alignment(pickup_ball(0, -100, 280)).action == 'BALL_PICKUP_CRAB_LEFT'
    assert planner.plan_ball_pickup_fine_alignment(pickup_ball(0, 0, 280)).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('fine', [False, True])
@pytest.mark.parametrize('changes', [
    {'depth_valid': False}, {'depth_age_sec': 1.}, {'ground_projection_valid': False},
])
def test_unusable_heading_or_turn_distance_never_permits_forward(fine, changes):
    planner = MotionDecisionPlanner()
    plan = planner.plan_ball_pickup_fine_alignment if fine else planner.plan_ball_pickup_initial_alignment
    result = plan(pickup_ball(20., 0, **changes))
    assert not result.valid and result.action == 'WAIT'


@pytest.mark.parametrize('fine', [False, True])
def test_near_foot_ball_uses_lateral_correction_instead_of_turning(fine):
    planner = MotionDecisionPlanner()
    plan = planner.plan_ball_pickup_fine_alignment if fine else planner.plan_ball_pickup_initial_alignment
    result = plan(pickup_ball(30., 100, 120))
    assert result.action == ('BALL_PICKUP_CRAB_RIGHT' if fine else 'BALL_PICKUP_INITIAL_CRAB_RIGHT')


@pytest.mark.parametrize('direction,count', [('LEFT', 1), ('RIGHT', 2)])
def test_fine_turn_returns_to_same_checkpoint_without_automatic_forward(direction, count):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    action = f'BALL_PICKUP_FINE_TURN_{direction}_{count}'
    assert action in MotionDecisionNode.PICKUP_FINE_ALIGN_ACTIONS
    bridge.navigation_command_callback(fine_alignment_message(action))
    # Fine-to-turn posture transition must still finish before the physical turn.
    assert bridge.active_action == 'PICKUP_NOW'
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'fine_to_turn_ready_0'
    complete_active_motion(bridge)
    motion = f'pickup_camera_down_turn_{direction.lower()}_{count}'
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == motion
    before = len(bridge.executor_request_publisher.messages)
    complete_pickup_motion_and_dwell(bridge, motion)
    assert bridge.pickup_fine_align_waiting
    assert not bridge.pickup_fixed_sequence_started
    assert len(bridge.executor_request_publisher.messages) == before


def test_node_publishes_fine_turn_inside_pickup_lock_and_waits_for_completion(monkeypatch):
    clock = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    node = LiveInputHarness(phase='BALL_APPROACH')
    assert node.phase_manager.start_special_action('PICKUP_NOW', 1)
    node.pickup_fine_align_waiting = True
    observe(node, clock, 'ball', pickup_ball(-20., -185))
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_TURN_LEFT_1'
    assert command['active_special_command_id'] == 1
    assert command['sdk_motion_requested']
    assert not node.pickup_fine_align_waiting
    before = len([m for m in node.publisher.messages if m['valid']])
    observe(node, clock, 'ball', pickup_ball(0, 0))
    assert len([m for m in node.publisher.messages if m['valid']]) == before
