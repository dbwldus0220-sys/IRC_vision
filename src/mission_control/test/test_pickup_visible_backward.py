"""Visible-ball proximity does not request retreat or block alignment."""

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


@pytest.mark.parametrize('bottom', [0, 50, 99.999, 100])
@pytest.mark.parametrize('proximity', [None, 'PENDING', 'RETREAT', 'HOLD', 'CLEAR'])
@pytest.mark.parametrize('offset', [-220, 0, 220])
def test_close_visible_ball_keeps_crab_or_continuation(bottom, offset, proximity):
    decision = plan(MotionDecisionPlanner(), 'FINE', pickup_head_ready=True,
                    pickup_proximity=proximity, bottom_distance_px=bottom, offset_x_px=offset)
    expected = ('BALL_PICKUP_CRAB_LEFT' if offset < 0 else
                'BALL_PICKUP_CRAB_RIGHT' if offset > 0 else 'BALL_PICKUP_FINE_ALIGN_CONTINUE')
    assert decision.valid and decision.action == expected
    assert 'backward_count' not in decision.source_command
    assert not decision.source_command.get('lost_ball_alignment_from_memory')


def test_unconfirmed_close_frames_do_not_authorize_backward():
    planner = MotionDecisionPlanner()
    for bottom in (50, 99.999):
        assert plan(planner, 'FINE', bottom_distance_px=bottom).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
        assert not plan(planner, 'INITIAL', bottom_distance_px=bottom).action.endswith('_BACKWARD')
    decision = plan(planner, 'FINE', bottom_distance_px=100, offset_x_px=220)
    assert decision.valid and decision.action.endswith('CRAB_RIGHT')


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


def test_close_visible_pixels_continue_alignment_without_ground_depth():
    decision = plan(MotionDecisionPlanner(), 'FINE', bottom_distance_px=99,
                    pickup_head_ready=True, pickup_proximity='RETREAT',
                    depth_valid=False, depth_age_sec=None, distance_m=None)
    assert decision.valid and decision.action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('bottom,action', [
    (230.001, 'BALL_PICKUP_FINE_FORWARD'),
    (230, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (150, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (99.999, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
])
def test_fine_distance_threshold_remains_230(bottom, action):
    assert plan(MotionDecisionPlanner(), 'FINE', bottom_distance_px=bottom).action == action


def test_fine_backward_checkpoint_clears_old_ball_before_next_decision():
    node = FreshMockInputNode()
    node.phase_manager.set_phase('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    close = ball_info(depth_m=.4, offset_x_px=220, bottom_distance_px=99,
                      pickup_head_ready=True, pickup_proximity='RETREAT')
    node.latest_info['ball'] = close
    send_status(node, status='RUNNING', action='PICKUP_NOW', command_id=10,
                event_id=1, dynamics_command=None, motion_id='pickup_lost_ball_backward_1')
    assert select_decision(node, ball=close).action == 'WAIT'
    marker = Node.PICKUP_FINE_ALIGN_MARKER
    send_status(node, status='RUNNING', action='PICKUP_NOW', command_id=10,
                event_id=1, dynamics_command=None, motion_id=marker)
    assert node.latest_info['ball'] is None
    assert select_decision(node, ball=None).action == 'WAIT'
    assert select_decision(node, ball=close).action == 'BALL_PICKUP_CRAB_RIGHT'
    assert select_decision(node, ball={**close, 'bottom_distance_px': 100}).action.endswith('CRAB_RIGHT')
    assert not node.pickup_fixed_sequence_started


@pytest.mark.parametrize('motion,offset', [
    ('pickup_crab_left_0', -100), ('pickup_crab_right_0', 100),
    ('pickup_fine_forward_0', 0),
])
def test_close_visible_ball_can_reserve_crab_but_not_more_forward(motion, offset):
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=1,
                               motion_id=motion, pickup_alignment_stage='fine'), 10., 10_000_000_000)
    for stamp in (11_500_000_000, 11_550_000_000, 11_600_000_000):
        reservation.observe(planner, ball(bottom_distance_px=99, offset_x_px=offset,
                                          rgb_stamp_ns=stamp), stamp / 1e9, stamp, .5)
    if offset:
        assert reservation.ready is not None and reservation.count == 3
        assert reservation.ready.action == ('BALL_PICKUP_CRAB_LEFT' if offset < 0
                                            else 'BALL_PICKUP_CRAB_RIGHT')
    else:
        assert reservation.ready is None and reservation.count == 0
