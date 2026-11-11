"""Normal walking may skip fine walking only at a fresh centered 90px checkpoint."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, continue_pickup_after_backward,
)
from test_pickup_fine_skip import initial_checkpoint, send_command
from test_pickup_continuous_walk import ball


WALKS = ['line_forward_2', 'line_forward_4', 'line_forward_6', 'line_forward_8',
         'ball_camera_down_forward_2', 'ball_camera_down_forward_4',
         'line_recovery_left_4', 'line_recovery_right_4',
         'line_recovery_ball_first_left_4', 'line_recovery_ball_second_right_4']


@pytest.mark.parametrize('previous', WALKS)
@pytest.mark.parametrize('bottom,action', [(0, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (89.999, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'), (90, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (90.001, 'BALL_PICKUP_FINE_FORWARD')])
def test_normal_gait_threshold_overrides_old_distance_latch(previous, bottom, action):
    planner = MotionDecisionPlanner()
    planner.pickup_last_completed_motion_id = previous
    planner.pickup_fine_approach_complete = True
    result = planner.plan_ball_pickup_fine_alignment(ball(bottom_distance_px=bottom))
    assert result.action == action
    assert result.source_command['pickup_fine_skip_limit_px'] == 90


@pytest.mark.parametrize('updates', [
    {'detected': False, 'raw_detected': False}, {'confidence': .1},
    {'bottom_distance_px': None}, {'bottom_distance_px': -1},
    {'bottom_distance_px': float('nan')}, {'bottom_distance_px': float('inf')},
    {'pickup_head_ready': False}, {'offset_x_px': -100}, {'offset_x_px': 100},
])
def test_invalid_or_unaligned_sample_cannot_skip(updates):
    planner = MotionDecisionPlanner()
    planner.pickup_last_completed_motion_id = 'line_forward_4'
    result = planner.plan_ball_pickup_fine_alignment(ball(**{'bottom_distance_px': 90, **updates}))
    assert result.action != 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


def prepare_skip(clock, previous, index):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge.last_physical_motion_id = previous
    sample = ball(bottom_distance_px=90)
    initial_checkpoint(node, bridge, clock, sample)
    assert bridge.pickup_fine_align_waiting
    assert not bridge.executor_request_publisher.messages
    fresh(node, clock, 'ball', sample)
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    send_command(node, bridge)
    assert bridge.pickup_fixed_sequence_started and node.pickup_fixed_sequence_started
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .2)
    clock[0] += .199
    bridge._check_atomic_dwell()
    assert not bridge.executor_request_publisher.messages
    clock[0] += .001
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'pickup_forward_to_backward_prepare'
    return node, bridge, request


@pytest.mark.parametrize('previous', WALKS)
@pytest.mark.parametrize('index', [1, 2])
def test_skip_preparation_half_second_then_retreat_and_grasp(clock, previous, index):
    node, bridge, request = prepare_skip(clock, previous, index)
    clock[0] += .754
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    deadline = clock[0] + .5
    assert bridge.transition_dwell_until == pytest.approx(deadline)
    clock[0] += .2
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert bridge.transition_dwell_until == pytest.approx(deadline)
    clock[0] = deadline - .001
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = deadline
    bridge._check_atomic_dwell()
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r['motion_id'] for r in requests] == [
        'pickup_forward_to_backward_prepare', 'pickup_pre_backward_camera_down']
    assert bridge.transition_dwell_until is None
    # Complete the fixed tail, crossing the fresh post-retreat grasp checkpoint.
    for _ in range(25):
        if not bridge.motion_in_progress:
            break
        if bridge.pickup_post_backward_align_waiting:
            continue_pickup_after_backward(bridge, bottom_distance_px=400)
        elif bridge.transition_dwell_until is not None or bridge.active_dwell_until is not None:
            clock[0] = bridge.transition_dwell_until or bridge.active_dwell_until
            bridge._check_atomic_dwell()
        else:
            current = decoded_messages(bridge.executor_request_publisher)[-1]
            clock[0] += 1.
            bridge.executor_status_callback(executor_status(**current, status='SUCCEEDED'))
    assert not bridge.motion_in_progress
    ids = [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)]
    assert 'pickup_fine_forward_0' not in ids and 'ball_general_fine_forward_8' not in ids
    assert ids.count('pickup_forward_to_backward_prepare') == 1
    assert 'pickup_500' in ids
    assert ids[-1] == ('pickup_first_backward_turn_right' if index == 1 else 'pickup_second_backward_turn_left')


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_preparation_cannot_advance_tail(clock, status):
    _, bridge, request = prepare_skip(clock, 'line_recovery_left_4', 1)
    bridge.executor_status_callback(executor_status(**request, status=status, error_code='SDK_POSITION_TIMEOUT'))
    clock[0] += 5.
    bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    assert bridge.last_physical_motion_id is None
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('bottom', [0, 89.999, 90])
def test_ninety_pixel_observations_cannot_reserve_another_normal_walk(stage, bottom):
    from mission_control.pickup_walk_reservation import PickupWalkReservation
    reservation, planner = PickupWalkReservation(), MotionDecisionPlanner()
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=1,
                               motion_id='ball_camera_down_forward_2',
                               pickup_alignment_stage=stage), 10., 10_000_000_000)
    planner.pickup_last_completed_motion_id = 'pickup_crab_right_0'
    for now in (10.75, 10.85, 10.95):
        stamp = round(now * 1e9)
        reservation.observe(planner, ball(bottom_distance_px=bottom, pickup_head_ready=True,
                                          rgb_stamp_ns=stamp), now, stamp, .5)
        assert reservation.ready is None and not reservation.pending
    assert planner.pickup_last_completed_motion_id == 'pickup_crab_right_0'
