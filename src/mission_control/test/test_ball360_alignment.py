"""Current pickup pixels exclusively select yaw or crab; approach only uses yaw."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from test_ball_ground_alignment import ball_sample
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_decision_planner import ball_info


@pytest.mark.parametrize('method,turn,crab', [
    ('plan_ball_pickup_initial_alignment', 'BALL_PICKUP_CAMERA_DOWN_TURN', 'BALL_PICKUP_INITIAL_CRAB'),
    ('plan_ball_pickup_fine_alignment', 'BALL_PICKUP_FINE_TURN', 'BALL_PICKUP_CRAB'),
])
@pytest.mark.parametrize('bottom', [150, 359, 360, 360.001, 450, 600])
@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
def test_pickup_360_boundary_uses_exactly_one_alignment_family(method, turn, crab, bottom, side):
    sign = -1 if side == 'LEFT' else 1
    decision = getattr(MotionDecisionPlanner(), method)(ball_sample(
        sign * 30, depth_m=.5, distance_m=.5, offset_x_px=sign * 200,
        bottom_distance_px=bottom,
    ))
    expected = f'{crab}_{side}' if bottom <= 360 else f'{turn}_{side}_{2 if side == "LEFT" else 3}'
    assert decision.valid and decision.action == expected
    assert decision.source_command['pickup_alignment_mode'] == ('CRAB' if bottom <= 360 else 'TURN')


@pytest.mark.parametrize('method', ['plan_ball_pickup_initial_alignment', 'plan_ball_pickup_fine_alignment'])
def test_current_pixels_override_previous_close_state_in_both_directions(method):
    planner = MotionDecisionPlanner()
    for bottom, family in [(150, 'CRAB'), (450, 'TURN'), (360, 'CRAB'), (361, 'TURN')]:
        decision = getattr(planner, method)(ball_sample(
            30, depth_m=.5, distance_m=.5, offset_x_px=-200, bottom_distance_px=bottom))
        assert family in decision.action


@pytest.mark.parametrize('method', ['plan_ball_pickup_initial_alignment', 'plan_ball_pickup_fine_alignment'])
def test_far_pickup_aligned_heading_advances_without_crab_even_with_pixel_offset(method):
    decision = getattr(MotionDecisionPlanner(), method)(ball_sample(
        10, depth_m=.5, distance_m=.5, offset_x_px=200, bottom_distance_px=450))
    assert decision.valid
    assert 'CRAB' not in decision.action and 'TURN' not in decision.action


@pytest.mark.parametrize('method', ['plan_ball_pickup_initial_alignment', 'plan_ball_pickup_fine_alignment'])
def test_close_crab_can_use_pixels_without_valid_ground_angle(method):
    decision = getattr(MotionDecisionPlanner(), method)(ball_sample(
        30, ground_projection_valid=False, depth_m=.5, distance_m=.5,
        offset_x_px=-200, bottom_distance_px=360))
    assert decision.valid and decision.action.endswith('CRAB_LEFT')


@pytest.mark.parametrize('bottom', [None, -1, float('nan'), float('inf'), True])
@pytest.mark.parametrize('method', ['plan_ball_pickup_initial_alignment', 'plan_ball_pickup_fine_alignment'])
def test_invalid_bottom_cannot_authorize_either_alignment(method, bottom):
    decision = getattr(MotionDecisionPlanner(), method)(ball_sample(
        30, depth_m=.5, distance_m=.5, offset_x_px=200, bottom_distance_px=bottom))
    assert not decision.valid


@pytest.mark.parametrize('method', ['plan_ball_pickup_initial_alignment', 'plan_ball_pickup_fine_alignment'])
@pytest.mark.parametrize('missing', [
    None,
    {'detected': False, 'raw_detected': False},
    {'detected': False, 'raw_detected': False, 'ball_loss_confirmed': False},
    {'detected': False, 'raw_detected': True, 'ball_loss_confirmed': True},
])
def test_close_ball_cannot_start_search_before_confirmed_loss(method, missing):
    planner = MotionDecisionPlanner()
    info = ball_sample(30, depth_m=.5, distance_m=.5, offset_x_px=200,
                       camera_center_offset_x_px=200, bottom_distance_px=360)
    planner._update_ball_tracking(info, 0.)
    getattr(planner, method)(info)
    decision = getattr(planner, method)(missing)
    assert not decision.valid
    assert decision.reason != 'turn_toward_last_seen_ball_side'


@pytest.mark.parametrize('angle,action', [(-30, 'BALL_APPROACH_TURN_LEFT_2'),
                                         (30, 'BALL_APPROACH_TURN_RIGHT_3')])
def test_visible_approach_turns_without_translation_and_resolves_existing_mapping(angle, action):
    decision = MotionDecisionPlanner().plan('BALL_APPROACH', {'ball': ball_sample(angle)}, .1)
    assert decision.valid and decision.action == action
    assert decision.source_command['travel_distance_m'] == 0
    assert decision.source_command['linear_speed_mps'] == 0
    assert MotionCommandBridgeNode.motion_id_for_action(action) == (
        'post_ball_line_turn_left_2' if angle < 0 else 'post_ball_line_turn_right_3')
    assert not MotionDecisionNode._is_continuous_ball_action(action)


def test_forward_does_not_prequeue_stationary_turn_and_turn_requires_new_frame_after_dwell(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.BALL_POST_MOTION_DWELL_SEC = 1.
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING', 'line_forward_4')
    clock[0] += 1.8
    for _ in range(4):
        fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=30), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    turn = node.publisher.messages[-1]
    assert turn['action'] == 'BALL_APPROACH_TURN_RIGHT_3'
    node.send_status(turn['action'], turn['command_id'], 'RUNNING', 'post_ball_line_turn_right_3')
    node.send_status(turn['action'], turn['command_id'], 'SUCCEEDED')
    assert node.ball_post_motion_dwell_until == pytest.approx(clock[0] + 1.)
    before = len(node.publisher.messages)
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    assert len(node.publisher.messages) == before
    clock[0] += 1.
    MotionDecisionNode._publish_decision(node)
    fresh(node, clock, 'ball', ball_info(depth_m=.9))
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT'
