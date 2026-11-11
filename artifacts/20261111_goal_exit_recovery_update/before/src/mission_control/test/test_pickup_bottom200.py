"""Replay 180px/100px alignment and the independent 220px completion boundary without driving robot hardware."""

import json
import math

import pytest
from std_msgs.msg import String

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.pickup_walk_reservation import PickupWalkReservation, PICKUP_WALK_DURATIONS
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import ball, start, forward_status, finish_rest, late_frames, reserved_request
from test_pickup_crab_observation import advance_dwell
from test_wait_refresh_and_fine_settle import LiveInputHarness


def aligned_decision(planner, stage, sample):
    return getattr(planner, f'plan_ball_pickup_{stage}_alignment')(sample)


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('bottom', [179.999, 180., 180.001, 185., 287., 360.])
def test_latest_bottom_selects_crab_or_turn(stage, side, bottom):
    sample = ball(bottom_distance_px=bottom, offset_x_px=side * 190,
                  steering_angle_deg=side * 33.505, offset_x_norm=side * .3, depth_m=.458,
                  pickup_head_ready=True)
    decision = aligned_decision(MotionDecisionPlanner(), stage, sample)
    direction = 'LEFT' if side < 0 else 'RIGHT'
    prefix = ('BALL_PICKUP_INITIAL_CRAB' if stage == 'initial' else 'BALL_PICKUP_CRAB')
    if bottom > 180:
        prefix = 'BALL_PICKUP_CAMERA_DOWN_TURN' if stage == 'initial' else 'BALL_PICKUP_FINE_TURN'
    assert decision.valid and decision.action.startswith(f'{prefix}_{direction}')
    assert decision.source_command['pickup_crab_max_bottom_distance_px'] == 180
    assert decision.source_command['steering_source'] == 'camera_down_image_angle'


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('bottom', [180., 180.001, 185., 287.])
@pytest.mark.parametrize('offset', [99.999, 100., 100.001])
def test_turn_requires_both_strict_pixel_boundaries(stage, side, bottom, offset):
    dx = side * offset
    sample = ball(bottom_distance_px=bottom, offset_x_px=dx, offset_x_norm=dx / 640,
                  steering_angle_deg=math.degrees(math.atan2(dx, bottom)), pickup_head_ready=True)
    decision = aligned_decision(MotionDecisionPlanner(), stage, sample)
    assert decision.valid
    if bottom <= 180:
        assert 'CRAB_' in decision.action
        assert decision.source_command['pickup_alignment_mode'] == 'CRAB'
    elif offset > 100:
        assert 'TURN_' in decision.action
        assert decision.source_command['pickup_alignment_mode'] == 'TURN'
    else:
        expected = 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE' if stage == 'initial' else 'BALL_PICKUP_FINE_FORWARD'
        assert decision.action == expected
        assert decision.source_command['pickup_alignment_mode'] == 'APPROACH'


@pytest.mark.parametrize('stage', ['initial', 'fine'])
def test_recorded_near_ball_now_turns_left(stage):
    sample = ball(bottom_distance_px=185, offset_x_px=-187, offset_x_norm=-187 / 640,
                  steering_angle_deg=-45.3, depth_m=.46, distance_m=.46,
                  pickup_head_ready=True)
    decision = aligned_decision(MotionDecisionPlanner(), stage, sample)
    assert decision.valid and decision.action.endswith('TURN_LEFT_3')


@pytest.mark.parametrize('bottom', [180., 180.001, 185., 200., 211., 219.999, 220., 220.001])
@pytest.mark.parametrize('offset', [-100., -50.001, -50., -48., -30., 0., 55., 55.001, 100.])
@pytest.mark.parametrize('motion', [None, 'pickup_fine_forward_0', 'pickup_crab_left_0'])
def test_fine_completion_keeps_220px_and_centering_independent_of_crab_range(bottom, offset, motion):
    planner = MotionDecisionPlanner()
    planner.pickup_last_completed_motion_id = motion
    # Even an accumulated completion credit must not bypass either final guard.
    planner.pickup_fine_progress_complete = True
    decision = planner.plan_ball_pickup_fine_alignment(ball(
        bottom_distance_px=bottom, offset_x_px=offset, offset_x_norm=offset / 640,
        steering_angle_deg=0.))
    centered = -50 <= offset <= 55
    if bottom <= 180 and not centered:
        assert 'CRAB_' in decision.action
    elif bottom <= 220 and centered:
        assert decision.action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    else:
        assert decision.action == 'BALL_PICKUP_FINE_FORWARD'


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('side', [-1, 1])
def test_existing_angle_threshold_still_applies_after_pixel_gates(stage, side):
    sample = ball(bottom_distance_px=250, offset_x_px=side * 101, offset_x_norm=side * .2,
                  steering_angle_deg=side * (7.999 if side < 0 else 14.999))
    decision = aligned_decision(MotionDecisionPlanner(), stage, sample)
    assert decision.valid and 'TURN_' not in decision.action


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('offset', [-100., 0., 100.])
@pytest.mark.parametrize('invalid', [
    {'depth_valid': False}, {'depth_age_sec': .701}, {'depth_age_sec': None},
    {'distance_m': None}, {'distance_m': 0.},
    {'ground_projection_enabled': True, 'ground_projection_valid': False,
     'head_down_requested': False},
])
def test_skipping_yaw_does_not_bypass_forward_input_validity(stage, offset, invalid):
    planner = MotionDecisionPlanner()
    sample = ball(bottom_distance_px=250, offset_x_px=offset, offset_x_norm=offset / 640,
                  steering_angle_deg=30., pickup_head_ready=True)
    decision = aligned_decision(planner, stage, {**sample, **invalid})
    assert decision.action == 'WAIT' and not decision.valid
    recovered = aligned_decision(planner, stage, sample)
    assert recovered.valid and 'TURN_' not in recovered.action
    assert recovered.action == ('BALL_PICKUP_INITIAL_ALIGN_CONTINUE' if stage == 'initial'
                                else 'BALL_PICKUP_FINE_FORWARD')


@pytest.mark.parametrize('stage,motion,bottom', [
    ('initial', 'ball_camera_down_forward_2', 340),
    ('fine', 'pickup_fine_forward_0', 287),
])
@pytest.mark.parametrize('side', [-1, 1])
def test_unpublished_forward_reservation_clears_when_dx_crosses_100(stage, motion, bottom, side):
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=2,
                               motion_id=motion, pickup_alignment_stage=stage), 10., 10_000_000_000)
    now = 10. + PICKUP_WALK_DURATIONS[motion] * .75
    sample = ball(bottom_distance_px=bottom, offset_x_px=side * 100, offset_x_norm=side * .2,
                  steering_angle_deg=side * 35., pickup_head_ready=True)
    for _ in range(3):
        now += .01
        stamp = round(now * 1e9)
        reservation.observe(planner, {**sample, 'rgb_stamp_ns': stamp}, now, stamp, .5)
    assert reservation.decision(now, .5) is not None
    now += .01
    sample.update(offset_x_px=side * 100.001, rgb_stamp_ns=round(now * 1e9))
    assert 'TURN_' in aligned_decision(planner, stage, sample).action
    reservation.observe(planner, sample, now, round(now * 1e9), .5)
    assert reservation.decision(now, .5) is None and reservation.count == 0


def test_unpublished_fine_reservation_clears_at_recorded_211_minus48_sample():
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    motion = 'pickup_fine_forward_0'
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=2,
                               motion_id=motion, pickup_alignment_stage='fine'), 10., 10_000_000_000)
    now = 10. + PICKUP_WALK_DURATIONS[motion] * .75
    for _ in range(3):
        now += .01
        stamp = round(now * 1e9)
        reservation.observe(planner, ball(bottom_distance_px=300, rgb_stamp_ns=stamp), now, stamp, .5)
    assert reservation.decision(now, .5) is not None
    now += .01
    stamp = round(now * 1e9)
    sample = ball(bottom_distance_px=211, offset_x_px=-48, offset_x_norm=-48 / 640,
                  steering_angle_deg=-12.816, rgb_stamp_ns=stamp)
    reservation.observe(planner, sample, now, stamp, .5)
    assert reservation.decision(now, .5) is None
    assert reservation.count == 0
    assert planner.plan_ball_pickup_fine_alignment(sample).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('invalid', [
    {'depth_valid': False}, {'depth_age_sec': .701}, {'depth_m': .601},
    {'pickup_head_ready': False}, {'bottom_distance_px': None},
])
def test_invalid_turn_input_waits_then_recovers(stage, invalid):
    planner = MotionDecisionPlanner()
    sample = ball(bottom_distance_px=287, steering_angle_deg=-33.505,
                  offset_x_px=-190, offset_x_norm=-.3, pickup_head_ready=True)
    decision = aligned_decision(planner, stage, {**sample, **invalid})
    assert decision.action == 'WAIT' and not decision.valid
    recovered = aligned_decision(planner, stage, sample)
    assert recovered.valid and 'TURN_LEFT' in recovered.action


@pytest.mark.parametrize('stage', ['initial', 'fine'])
def test_close_memory_does_not_block_return_to_turn_range(stage):
    planner = MotionDecisionPlanner()
    sample = ball(bottom_distance_px=180, steering_angle_deg=-33.505, offset_x_px=-190, offset_x_norm=-.3)
    assert 'CRAB_LEFT' in aligned_decision(planner, stage, sample).action
    decision = aligned_decision(planner, stage, {**sample, 'bottom_distance_px': 180.001})
    assert 'TURN_LEFT' in decision.action
    assert not decision.source_command['pickup_close_alignment_active']


@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('side', [-1, 1])
def test_uncommitted_crab_reservation_clears_when_latest_frame_requires_turn(stage, side):
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    motion = f'pickup_crab_{"left" if side < 0 else "right"}_0'
    reservation.on_status(dict(status='RUNNING', command_id=1, request_id=2,
                               motion_id=motion, pickup_alignment_stage=stage), 10., 10_000_000_000)
    now = 10. + PICKUP_WALK_DURATIONS[motion] * .75
    for _ in range(3):
        now += .01
        reservation.observe(planner, ball(bottom_distance_px=180, offset_x_px=side * 100,
                                         steering_angle_deg=side * 33., offset_x_norm=side * .2, rgb_stamp_ns=round(now * 1e9)),
                            now, round(now * 1e9), .5)
    assert reservation.decision(now, .5) is not None
    now += .01
    reservation.observe(planner, ball(bottom_distance_px=180.001, offset_x_px=side * 110,
                                     steering_angle_deg=side * 33., offset_x_norm=side * .2, rgb_stamp_ns=round(now * 1e9)),
                        now, round(now * 1e9), .5)
    assert reservation.decision(now, .5) is None and reservation.count == 0


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('bottom,offset', [(180.001, 100), (185., -100), (200., 100),
                                          (220.001, 0), (225., 0), (230., 0)])
@pytest.mark.parametrize('expire_sample', [False, True])
def test_no_prequeue_at_old_completion_band_resumes_at_checkpoint(clock, index, bottom, offset, expire_sample):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    parent, first = start(node, bridge, clock, ball(bottom_distance_px=250))
    assert first['motion_id'] == 'ball_general_fine_forward_8'
    late_frames(node, clock, ball(bottom_distance_px=bottom, offset_x_px=offset))
    assert not node.pickup_walk_reservation.pending
    assert bridge.pickup_queued_request is None
    if expire_sample:
        clock[0] += node.timeouts['ball'] + .01
    forward_status(node, bridge, first, 'SUCCEEDED')
    if bridge.active_dwell_until is not None:
        advance_dwell(node, bridge, clock)
    if expire_sample:
        assert node.pickup_fine_align_waiting
        fresh(node, clock, 'ball', ball(bottom_distance_px=bottom, offset_x_px=offset))
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_FORWARD' and command['valid'], json.dumps(command)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    finish_rest(bridge, clock)
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    assert second['motion_id'] == 'pickup_fine_forward_0'
    forward_status(node, bridge, second, 'RUNNING')
    forward_status(node, bridge, second, 'SUCCEEDED')
    if bridge.active_dwell_until is not None:
        advance_dwell(node, bridge, clock)
    fresh(node, clock, 'ball', ball(bottom_distance_px=200))
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE' and command['valid']
    assert node.active_special_command_id == parent['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    finish_rest(bridge, clock)
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_pre_backward_camera_down'


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED', 'TIMEOUT', 'CANCELLED'])
def test_near_crab_queue_releases_only_after_success_and_clears_on_failure(clock, index, side, status):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    motion = f'pickup_crab_{"left" if side < 0 else "right"}_0'
    bridge.last_physical_motion_id = motion
    sample = ball(bottom_distance_px=180, offset_x_px=side * 100)
    parent, first = start(node, bridge, clock, sample)
    assert first['motion_id'] == motion
    late_frames(node, clock, sample)
    assert node.pickup_walk_reservation.pending
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    queued = reserved_request(bridge)
    before = len(bridge.executor_request_publisher.messages)
    assert queued['command_id'] == parent['command_id']
    forward_status(node, bridge, first, status)
    if status != 'SUCCEEDED':
        assert bridge.pickup_queued_request is None and not node.pickup_walk_reservation.pending
        assert not bridge.motion_in_progress
        assert len(bridge.executor_request_publisher.messages) == before
        return
    assert len(bridge.executor_request_publisher.messages) == before
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    finish_rest(bridge, clock)
    assert bridge.active_request_id == queued['request_id']
    forward_status(node, bridge, queued, 'RUNNING')
    forward_status(node, bridge, queued, 'SUCCEEDED')
    fresh(node, clock, 'ball', ball(bottom_distance_px=200))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
    assert node.active_special_command_id == parent['command_id']


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('bottom', [185, 287])
def test_turn_finishes_before_fresh_180px_crab(clock, index, side, bottom):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    parent, turn = start(node, bridge, clock, ball(
        bottom_distance_px=bottom, depth_m=.458, distance_m=.458,
        steering_angle_deg=side * 33.505, offset_x_norm=side * .3, offset_x_px=side * 190))
    direction = 'left' if side < 0 else 'right'
    assert turn['motion_id'].startswith(f'pickup_camera_down_turn_{direction}_')
    assert node.pickup_walk_reservation.key is None
    forward_status(node, bridge, turn, 'SUCCEEDED')
    assert bridge.active_dwell_until == pytest.approx(clock[0] + Bridge.PICKUP_TURN_DWELL_SEC)
    advance_dwell(node, bridge, clock)
    assert node.pickup_initial_align_waiting
    fresh(node, clock, 'ball', ball(bottom_distance_px=180, offset_x_px=side * 100))
    command = node.publisher.messages[-1]
    assert command['action'] == f'BALL_PICKUP_INITIAL_CRAB_{direction.upper()}'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    for _ in range(6):
        if bridge.transition_dwell_until is not None or bridge.active_dwell_until is not None:
            advance_dwell(node, bridge, clock)
        current = decoded_messages(bridge.executor_request_publisher)[-1]
        forward_status(node, bridge, current, 'RUNNING')
        if current['motion_id'] == f'pickup_crab_{direction}_0':
            break
        forward_status(node, bridge, current, 'SUCCEEDED')
    else:
        pytest.fail('Turn-to-crab transition did not reach the requested gait')
    forward_status(node, bridge, current, 'SUCCEEDED')
    fresh(node, clock, 'ball', ball(bottom_distance_px=200))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
    assert node.active_special_command_id == parent['command_id']


@pytest.mark.parametrize('side', [-1, 1])
def test_rejected_near_crab_reservation_returns_to_fresh_checkpoint(clock, side):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    sample = ball(bottom_distance_px=180, offset_x_px=side * 100)
    bridge.last_physical_motion_id = f'pickup_crab_{"left" if side < 0 else "right"}_0'
    _, first = start(node, bridge, clock, sample)
    late_frames(node, clock, sample)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    queued = reserved_request(bridge)
    forward_status(node, bridge, queued, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert bridge.pickup_queued_request is None
    assert bridge.active_request_id == first['request_id']
    clock[0] += node.timeouts['ball'] + .01
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert node.pickup_initial_align_waiting and not node.pickup_walk_reservation.pending
    fresh(node, clock, 'ball', ball(bottom_distance_px=200))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
