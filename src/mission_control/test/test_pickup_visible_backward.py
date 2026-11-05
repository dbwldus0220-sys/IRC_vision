"""Visible near-ball distance correction takes priority over pickup alignment."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PickupWalkReservation
from test_motion_decision_planner import ball_info
from test_motion_decision_node import (
    FreshMockInputNode, arm_special_command, send_status, select_decision,
)
from test_pickup_continuous_walk import ball


def plan(planner, stage, **changes):
    return getattr(planner, f'plan_ball_pickup_{stage.lower()}_alignment')(
        ball_info(depth_m=.4, **changes))


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
@pytest.mark.parametrize('bottom', [0, 50, 99.999])
@pytest.mark.parametrize('offset', [-220, 0, 220])
def test_visible_ball_retreats_before_crab_or_continuation(stage, bottom, offset):
    decision = plan(MotionDecisionPlanner(), stage,
                    bottom_distance_px=bottom, offset_x_px=offset)
    assert decision.valid and decision.action == f'BALL_PICKUP_{stage}_BACKWARD'
    assert decision.source_command['backward_count'] == 1
    assert not decision.source_command.get('lost_ball_alignment_from_memory')


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_each_checkpoint_uses_current_pixels_and_stops_retreat_at_100(stage):
    planner = MotionDecisionPlanner()
    for bottom in (50, 99.999):
        assert plan(planner, stage, bottom_distance_px=bottom).action.endswith('_BACKWARD')
    decision = plan(planner, stage, bottom_distance_px=100, offset_x_px=220)
    assert decision.valid and decision.action.endswith('CRAB_RIGHT')
    assert plan(planner, stage, bottom_distance_px=99).action.endswith('_BACKWARD')


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_recorded_146px_no_longer_requests_backward(stage):
    decision = plan(MotionDecisionPlanner(), stage, bottom_distance_px=146, offset_x_px=43)
    assert decision.valid and decision.action.endswith('_ALIGN_CONTINUE')


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
@pytest.mark.parametrize('changes', [
    {'detected': False}, {'detected': False, 'raw_detected': True},
    {'confidence': .1}, {'bottom_distance_px': None},
    {'bottom_distance_px': -1}, {'bottom_distance_px': float('nan')},
    {'bottom_distance_px': float('inf')}, {'bottom_distance_px': True},
])
def test_invalid_observation_cannot_authorize_visible_backward(stage, changes):
    sample = {'bottom_distance_px': 100, **changes}
    decision = plan(MotionDecisionPlanner(), stage, **sample)
    assert not decision.valid


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_valid_image_pixels_can_retreat_without_ground_depth(stage):
    decision = plan(MotionDecisionPlanner(), stage, bottom_distance_px=99,
                    depth_valid=False, depth_age_sec=None, distance_m=None)
    assert decision.valid and decision.action == f'BALL_PICKUP_{stage}_BACKWARD'


@pytest.mark.parametrize('bottom,action', [
    (270.001, 'BALL_PICKUP_FINE_FORWARD'),
    (270, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (150, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (99.999, 'BALL_PICKUP_FINE_BACKWARD'),
])
def test_fine_distance_threshold_remains_270(bottom, action):
    assert plan(MotionDecisionPlanner(), 'FINE', bottom_distance_px=bottom).action == action


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_backward_checkpoint_clears_old_ball_before_next_decision(stage):
    node = FreshMockInputNode()
    node.phase_manager.set_phase('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    close = ball_info(depth_m=.4, offset_x_px=220, bottom_distance_px=99)
    node.latest_info['ball'] = close
    send_status(node, status='RUNNING', action='PICKUP_NOW', command_id=10,
                event_id=1, dynamics_command=None, motion_id='pickup_lost_ball_backward_1')
    assert select_decision(node, ball=close).action == 'WAIT'
    marker = (Node.PICKUP_INITIAL_ALIGN_MARKER if stage == 'INITIAL'
              else Node.PICKUP_FINE_ALIGN_MARKER)
    send_status(node, status='RUNNING', action='PICKUP_NOW', command_id=10,
                event_id=1, dynamics_command=None, motion_id=marker)
    assert node.latest_info['ball'] is None
    assert select_decision(node, ball=None).action == 'WAIT'
    assert select_decision(node, ball=close).action == f'BALL_PICKUP_{stage}_BACKWARD'
    assert select_decision(node, ball={**close, 'bottom_distance_px': 100}).action.endswith('CRAB_RIGHT')
    assert not node.pickup_fixed_sequence_started


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('motion,offset', [
    ('pickup_crab_left_0', -100), ('pickup_crab_right_0', 100),
    ('pickup_fine_forward_0', 0),
])
def test_too_close_ball_cannot_reserve_another_crab_or_fine_step(stage, motion, offset):
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=1,
                               motion_id=motion, pickup_alignment_stage=stage), 10., 10_000_000_000)
    for stamp in (11_500_000_000, 11_550_000_000, 11_600_000_000):
        reservation.observe(planner, ball(bottom_distance_px=99, offset_x_px=offset,
                                          rgb_stamp_ns=stamp), stamp / 1e9, stamp, .5)
    assert reservation.ready is None and reservation.count == 0
