"""Replay the camera-down pixel band without driving robot hardware."""

import pytest

from mission_control.ball_fine_approach import (
    duration_sec, motion_id, physical_motion_id, with_preparation,
)
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.pickup_proximity import PickupProximity
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS
from test_ball_fine_alignment_queue import ball as approach_ball
from test_ball_entry_line_alignment import node_with_entry, send, status
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, enter_pickup_fine_alignment,
    fine_alignment_message, complete_active_motion,
)
from test_pickup_continuous_walk import (
    ball, start, late_frames, forward_status, reserved_request, finish_rest,
)
from test_wait_refresh_and_fine_settle import LiveInputHarness


def down_ball(**changes):
    return ball(pickup_head_ready=True, **changes)


@pytest.mark.parametrize('bottom,two', [
    (299.999, False), (300., True), (340., True), (360., True),
    (400., True), (500., True), (500.001, False),
])
@pytest.mark.parametrize('stage', ['initial', 'fine', 'approach'])
def test_inclusive_band_only_replaces_fine_forward(bottom, two, stage):
    planner = MotionDecisionPlanner()
    sample = down_ball(bottom_distance_px=bottom)
    if stage == 'initial':
        decision = planner.plan_ball_pickup_initial_alignment(sample)
        assert decision.valid
        assert decision.source_command['pickup_approach_motion'] == ('STRAIGHT_1' if two else 'STRAIGHT_0')
    elif stage == 'fine':
        decision = planner.plan_ball_pickup_fine_alignment(sample)
        assert decision.valid
        assert decision.action == ('BALL_PICKUP_FORWARD_2' if two else 'BALL_PICKUP_FINE_FORWARD')
    else:
        sample = {**approach_ball(), 'pickup_head_ready': True, 'bottom_distance_px': bottom}
        command = planner._plan_source('ball', sample, .1)
        assert command['valid']
        assert command['motion'] == 'BALL_APPROACH_FORWARD_2'


@pytest.mark.parametrize('head_ready', [False, None])
@pytest.mark.parametrize('stage', ['initial', 'fine', 'approach'])
def test_head_down_request_alone_does_not_authorize_regular_forward(head_ready, stage):
    planner = MotionDecisionPlanner()
    sample = ball(pickup_head_ready=head_ready)
    if stage == 'initial':
        decision = planner.plan_ball_pickup_initial_alignment(sample)
        assert decision.source_command.get('pickup_approach_motion') != 'STRAIGHT_1'
    elif stage == 'fine':
        assert planner.plan_ball_pickup_fine_alignment(sample).action != 'BALL_PICKUP_FORWARD_2'
    else:
        command = planner._plan_source('ball', approach_ball(pickup_head_ready=head_ready), .1)
        assert command['motion'] == 'BALL_APPROACH_FORWARD_2'


@pytest.mark.parametrize('fine', [False, True])
def test_alignment_still_precedes_forward(fine):
    planner = MotionDecisionPlanner()
    plan = planner.plan_ball_pickup_fine_alignment if fine else planner.plan_ball_pickup_initial_alignment
    assert 'TURN_RIGHT_' in plan(down_ball(bottom_distance_px=400, steering_angle_deg=35, offset_x_norm=.3, offset_x_px=192)).action
    assert 'CRAB_RIGHT' in plan(down_ball(bottom_distance_px=180, offset_x_px=100)).action


@pytest.mark.parametrize('index', [1, 2])
def test_regular_two_reserves_once_and_returns_to_initial_checkpoint(clock, index):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    sample = down_ball()
    parent, first = start(node, bridge, clock, sample)
    assert first['motion_id'] == 'ball_camera_down_forward_2'
    assert not bridge.pickup_fine_positioning_complete
    assert node.pickup_walk_reservation.stage == 'initial'
    late_frames(node, clock, sample)
    queued = node.publisher.messages[-1]
    assert queued['source_command']['pickup_walk_prequeue']
    send(bridge, queued)
    second = reserved_request(bridge)
    assert second['motion_id'] == first['motion_id']
    assert len(bridge.executor_request_publisher.messages) == 1
    forward_status(node, bridge, first, 'SUCCEEDED')
    finish_rest(bridge, clock)
    forward_status(node, bridge, second, 'RUNNING')
    assert bridge.active_request_id == second['request_id']
    assert node.active_special_command_id == parent['command_id']
    forward_status(node, bridge, second, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting
    assert node.pickup_initial_align_waiting
    assert not bridge.pickup_fine_positioning_complete


@pytest.mark.parametrize('changes', [
    {'bottom_distance_px': 299}, {'bottom_distance_px': 501},
    {'pickup_head_ready': False}, {'depth_valid': False}, {'depth_age_sec': .701},
    {'offset_x_px': 100}, {'bottom_distance_px': 400, 'steering_angle_deg': 35, 'offset_x_norm': .3, 'offset_x_px': 192},
])
def test_changed_observation_does_not_reserve_old_regular_forward(clock, changes):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    start(node, bridge, clock, down_ball())
    late_frames(node, clock, ball(**{'pickup_head_ready': True, **changes}))
    assert not node.pickup_walk_reservation.pending
    assert not any(m.get('source_command', {}).get('pickup_walk_prequeue') for m in node.publisher.messages)


@pytest.mark.parametrize('failure', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_regular_forward_discards_reserved_successor(clock, failure):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    _, first = start(node, bridge, clock, down_ball())
    late_frames(node, clock, down_ball())
    send(bridge, node.publisher.messages[-1])
    second = reserved_request(bridge)
    forward_status(node, bridge, first, failure, error_code='SDK_COMMUNICATION_ERROR')
    assert bridge.pickup_queued_request is None
    assert not bridge.motion_in_progress
    before = len(bridge.executor_request_publisher.messages)
    forward_status(node, bridge, second, 'RUNNING')
    assert len(bridge.executor_request_publisher.messages) == before


@pytest.mark.parametrize('index', [1, 2])
def test_approach_regular_forward_stays_regular_outside_pickup_pixel_band(clock, index):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    fresh(node, clock, 'ball', approach_ball(pickup_head_ready=True))
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_APPROACH_FORWARD_2'
    request = send(bridge, command)
    assert request['motion_id'] == 'ball_align_forward_2'
    status(node, bridge, request, 'RUNNING')
    assert node.ball_fine_duration_sec == pytest.approx(PICKUP_WALK_DURATIONS['ball_camera_down_forward_2'])
    clock[0] = node.ball_fine_started_at + duration_sec(request['motion_id']) * .75
    for _ in range(3):
        fresh(node, clock, 'ball', {**approach_ball(pickup_head_ready=True), 'bottom_distance_px': 299}, plan=False)
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_APPROACH_FORWARD_2'
    following = send(bridge, command)
    assert following['motion_id'] == 'ball_align_forward_2'
    assert not bridge.queued_request_deferred
    status(node, bridge, following, 'QUEUED')
    status(node, bridge, request, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert bridge.transition_dwell_until is None


def test_fine_to_regular_forward_keeps_support_preparation():
    planner = MotionDecisionPlanner()
    command = planner._plan_source('ball', approach_ball(pickup_head_ready=True), .1)
    assert with_preparation(command, 'fine')['motion'] == 'BALL_APPROACH_PREPARE_TURN'
    assert with_preparation(command, 'standard')['motion'] == 'BALL_APPROACH_FORWARD_2'
    assert physical_motion_id(motion_id('BALL_APPROACH_FORWARD_2')) == 'ball_camera_down_forward_2'
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    assert bridge._transition_prepare_motion('ball_camera_down_forward_2').startswith('fine_to_turn_ready_')


def test_fine_checkpoint_regular_forward_returns_to_same_checkpoint():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(fine_alignment_message('BALL_PICKUP_FORWARD_2'))
    assert bridge.pending_turn_request['motion_id'] == 'ball_camera_down_forward_2'
    complete_active_motion(bridge)
    assert any(m['motion_id'] == 'ball_camera_down_forward_2'
               for m in decoded_messages(bridge.executor_request_publisher))
    assert bridge.pickup_fine_align_waiting
    assert bridge.active_action == 'PICKUP_NOW'


def test_completed_regular_forward_does_not_count_as_completed_fine():
    planner = MotionDecisionPlanner()
    planner.pickup_last_completed_motion_id = 'ball_camera_down_forward_2'
    assert planner.plan_ball_pickup_fine_alignment(down_ball(bottom_distance_px=280)).action == 'BALL_PICKUP_FINE_FORWARD'


@pytest.mark.parametrize('complete', [False, True])
def test_actual_head_completion_controls_pixel_rule(complete):
    proximity = PickupProximity()
    proximity.heartbeat({
        'ball_head_override_active': True,
        'ball_head_transition_complete': complete,
        'ball_head_transition_id': 1,
        'ball_head_transition_complete_stamp_ns': 10_000_000_000,
    }, 10., 10_000_000_000)
    sample = ball(head_down_requested=False)
    proximity.observe(sample, 10_100_000_000, 10_100_000_000, 10.1,
                      max_age=.5, heartbeat_timeout=1., min_confidence=.35)
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(sample)
    if complete:
        assert decision.source_command['pickup_approach_motion'] == 'STRAIGHT_1'
    else:
        assert not decision.valid and decision.action == 'WAIT'
