"""Both Ball entries acquire ground steering without a distance switch or pause."""

import pytest

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_ball_entry_line_alignment import (
    node_with_entry, ground_ball_info, start_line, send, status, late_line,
)
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import FakeBridge, decoded_messages


@pytest.mark.parametrize('distance', [.610, .799, .800, .801, .829, 1.2, 1.49])
@pytest.mark.parametrize('angle,action', [
    (-27.19, 'RECOVER_LEFT_TURN_LEFT_4'),
    (19.46, 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (0., 'STRAIGHT_2'),
])
def test_available_ground_overrides_opposite_line_at_any_approach_distance(distance, angle, action):
    entry = BallEntryLineAlignment()
    decision = entry.select(MotionDecisionPlanner(), line_info(heading=-30. if angle >= 0 else 30.),
                            .1, ground_ball_info(depth_m=distance, ground_steering_angle_deg=angle))
    assert decision.valid and decision.action == action
    assert decision.source_command['heading_source'] == 'ball_ground'
    assert entry.ground_tracking_active


def test_ground_acquisition_is_retained_until_entry_reset():
    entry, planner = BallEntryLineAlignment(), MotionDecisionPlanner()
    line = line_info(heading=30., offset=-.3)
    before = entry.select(planner, line, .1, ball_info(depth_m=1.2))
    assert before.action == 'RECOVER_LEFT_TURN_RIGHT_4'
    info = ground_ball_info(depth_m=1.2, ground_steering_angle_deg=-27.19)
    assert entry.select(planner, line, .1, info).action == 'RECOVER_LEFT_TURN_LEFT_4'
    dropped = entry.select(planner, line, .1, {**info, 'ground_projection_valid': False})
    assert not dropped.valid and dropped.reason == 'ball_entry_waiting_for_valid_ground_ball'
    assert entry.select(planner, line, .1, info).action == 'RECOVER_LEFT_TURN_LEFT_4'
    entry.reset()
    assert not entry.ground_tracking_active
    assert entry.select(planner, line, .1, ball_info(depth_m=1.2)).action == before.action


@pytest.mark.parametrize('changes', [
    {'ground_projection_valid': False}, {'ground_steering_angle_deg': float('nan')},
    {'ground_forward_distance_m': 0.}, {'ground_coordinate_frame': 'camera'},
    {'depth_valid': False}, {'depth_age_sec': 1.}, {'confirmation_confirmed': False},
])
def test_invalid_input_cannot_latch_ground_tracking(changes):
    entry = BallEntryLineAlignment()
    entry.select(MotionDecisionPlanner(), line_info(), .1,
                 ground_ball_info(depth_m=1.2, **changes))
    assert not entry.ground_tracking_active


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('angle', [-27.19, 19.46, 0.])
def test_ground_arriving_during_previous_walk_reserves_next_walk_without_dwell(clock, index, angle):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    first, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ball_info(depth_m=1.2), plan=False)
    assert node.ball_entry_line_alignment.active
    assert not node.ball_entry_line_alignment.ground_tracking_active
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    for _ in range(3):
        # No Line frames are needed to hand the next gait to Ball steering.
        fresh(node, clock, 'ball', ground_ball_info(depth_m=1.2, ground_steering_angle_deg=angle), plan=False)
    queued = node.publisher.messages[-1]
    expected = ('STRAIGHT_2' if angle == 0 else 'RECOVER_LEFT_TURN_LEFT_4'
                if angle < 0 else 'RECOVER_RIGHT_TURN_RIGHT_4')
    assert queued['action'] == expected
    assert queued['source_command']['ball_entry_ground_tracking']
    assert node.general_motion_gate.active_command_id == first['command_id']
    assert node.queued_general_command_id == queued['command_id']
    following = send(bridge, queued)
    assert not bridge.queued_request_deferred
    assert following['request_id'] != request['request_id']
    completed_at = clock[0]
    status(node, bridge, request, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert clock[0] == completed_at
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert node.phase_manager.active_ball_mode_index == index
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2


@pytest.mark.parametrize('index', [1, 2])
def test_ground_does_not_replace_an_already_accepted_line_reservation(clock, index):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    _, first = start_line(node, bridge, clock)
    late_line(node, clock, line_info())
    accepted = node.publisher.messages[-1]
    queued_line = send(bridge, accepted)
    ground = ground_ball_info(depth_m=1.2, ground_steering_angle_deg=-27.19)
    fresh(node, clock, 'ball', ground, plan=False)
    assert node.queued_general_command_id == accepted['command_id']
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2
    status(node, bridge, first, 'SUCCEEDED')
    status(node, bridge, queued_line, 'RUNNING')
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    for _ in range(3):
        fresh(node, clock, 'ball', ground, plan=False)
    following = node.publisher.messages[-1]
    assert following['action'] == 'RECOVER_LEFT_TURN_LEFT_4'
    assert following['source_command']['ball_entry_ground_tracking']
    request = send(bridge, following)
    assert not bridge.queued_request_deferred
    status(node, bridge, queued_line, 'SUCCEEDED')
    status(node, bridge, request, 'RUNNING')
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
