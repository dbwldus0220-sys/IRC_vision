"""Replay head readiness and proximity metadata without visible-ball retreat."""

import json

import pytest
from std_msgs.msg import String

from mission_control.pickup_proximity import PickupProximity
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock, fresh
from test_pickup_continuous_walk import ball
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_decision_node import arm_special_command, send_status


def heartbeat(complete=True, head_id=1, stamp=10_000_000_000):
    return dict(sequence=head_id, ball_head_override_active=True,
                ball_head_transition_complete=complete,
                ball_head_transition_id=head_id,
                ball_head_transition_complete_stamp_ns=stamp if complete else None)


def ready():
    guard = PickupProximity()
    guard.heartbeat(heartbeat(), 10., 10_000_000_000)
    return guard


def frame(guard, t, bottom=99, **changes):
    sample = ball(bottom_distance_px=bottom, **changes)
    stamp = round(t * 1e9)
    guard.observe(sample, stamp, stamp, t, max_age=.5,
                  heartbeat_timeout=2., min_confidence=.5)
    return sample


def fine(sample):
    return MotionDecisionPlanner().plan_ball_pickup_fine_alignment(sample)


def test_completion_requires_capture_after_head_boundary():
    guard = PickupProximity()
    guard.heartbeat(heartbeat(False), 9.8, 9_800_000_000)
    assert frame(guard, 9.9)['pickup_head_ready'] is False
    guard.heartbeat(heartbeat(), 10.1, 10_100_000_000)
    for stamp in (9_990_000_000, 10_000_000_000):
        sample = ball(bottom_distance_px=33)
        guard.observe(sample, stamp, 10_100_000_000, 10.1, max_age=.5,
                      heartbeat_timeout=2., min_confidence=.5)
        assert not sample['pickup_head_ready']
        assert fine(sample).action == 'WAIT'
    assert frame(guard, 10.11)['pickup_head_ready']


def test_duplicates_and_out_of_order_frames_do_not_confirm_close():
    guard = ready()
    for t in (10.02, 10.02, 10.01, 10.02, 10.03):
        assert frame(guard, t)['pickup_proximity'] == 'PENDING'
    assert guard.close_count == 2
    sample = frame(guard, 10.04)
    assert sample['pickup_proximity'] == 'RETREAT'
    assert fine(sample).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('changes', [
    {'detected': False}, {'confidence': .1}, {'confirmation_confirmed': False},
    {'bottom_distance_px': None}, {'bottom_distance_px': float('nan')},
])
def test_invalid_image_breaks_consecutive_confirmation(changes):
    guard = ready()
    frame(guard, 10.01)
    frame(guard, 10.02)
    frame(guard, 10.03, **({'bottom': changes['bottom_distance_px']}
                         if 'bottom_distance_px' in changes else changes))
    assert frame(guard, 10.04)['pickup_proximity'] == 'PENDING'
    assert guard.close_count == 1


def test_gap_and_head_change_discard_previous_confirmation():
    guard = ready()
    frame(guard, 10.01)
    frame(guard, 10.02)
    assert frame(guard, 10.6)['pickup_proximity'] == 'PENDING'
    assert guard.close_count == 1
    guard.heartbeat(heartbeat(False, head_id=2), 10.61, 10_610_000_000)
    assert not frame(guard, 10.62)['pickup_head_ready']
    assert guard.close_count == 0
    assert not frame(ready(), 12.1)['pickup_head_ready']  # Heartbeat expired.


def test_unconfirmed_clearance_cannot_release_retreat_latch():
    guard = ready()
    guard.mark_retreat_sent()
    for t in (10.01, 10.02, 10.03):
        sample = frame(guard, t, 150, confirmation_confirmed=False)
        assert sample['pickup_proximity'] == 'HOLD'
    assert guard.retreat_sent


def test_legacy_proximity_metadata_does_not_block_alignment():
    guard = ready()
    for t in (10.01, 10.02, 10.03):
        sample = frame(guard, t)
    assert fine(sample).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    guard.mark_retreat_sent()
    for i, bottom in enumerate((99, 100, 119, 99, 120, 119, 120, 120)):
        action = fine(frame(guard, 10.04 + .01 * i, bottom)).action
        assert action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
        assert guard.retreat_sent
    assert fine(frame(guard, 10.12, 120)).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert not guard.retreat_sent
    for t in (10.13, 10.14):
        assert frame(guard, t)['pickup_proximity'] == 'PENDING'
    assert fine(frame(guard, 10.15)).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('bottom,offset,action', [
    (33, -16, 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'),
    (96, -92, 'BALL_PICKUP_INITIAL_CRAB_LEFT'),
])
def test_recorded_initial_frames_no_longer_retreat(bottom, offset, action):
    planner = MotionDecisionPlanner()
    sample = ball(bottom_distance_px=bottom, offset_x_px=offset, pickup_head_ready=False)
    assert planner.plan_ball_pickup_initial_alignment(sample).action == 'WAIT'
    sample['pickup_head_ready'] = True
    assert planner.plan_ball_pickup_initial_alignment(sample).action == action


def test_live_callback_continues_without_visible_ball_backward(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.pickup_proximity = PickupProximity()
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    send_status(node, status='RUNNING', action='PICKUP_NOW', command_id=10,
                event_id=1, dynamics_command=None, motion_id=Node.PICKUP_FINE_ALIGN_MARKER)
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat())))
    fresh(node, clock, 'ball', ball(bottom_distance_px=33))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert not node.pickup_proximity.retreat_sent
    samples, _ = Node._fresh_observations(node, clock[0])
    assert fine(samples['ball']).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat(False, 2))))
    samples, _ = Node._fresh_observations(node, clock[0])
    assert samples['ball']['pickup_head_ready'] is False
    assert fine(samples['ball']).action == 'WAIT'


def test_rejected_capture_breaks_confirmation_without_rearming_retreat(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.pickup_proximity = PickupProximity()
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat())))
    for _ in range(2):
        fresh(node, clock, 'ball', ball(bottom_distance_px=33), plan=False)
    assert node.pickup_proximity.close_count == 2
    node.pickup_proximity.mark_retreat_sent()
    Node._info_callback(node, 'ball')(String(data=json.dumps(
        ball(bottom_distance_px=150, rgb_stamp_ns=round((clock[0] + 1.) * 1e9)))))
    assert node.latest_info['ball'] is None
    assert node.pickup_proximity.close_count == node.pickup_proximity.release_count == 0
    assert node.pickup_proximity.retreat_sent
