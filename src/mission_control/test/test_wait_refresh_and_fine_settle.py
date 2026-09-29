"""Exercise live input refresh during WAIT and the production fine-step pause."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_planner import MotionDecision
from test_mission_phase_flow import MissionFlowHarness, line_info
from test_motion_decision_planner import ball_info, goal_info, hurdle_info
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status,
    fine_alignment_message, enter_pickup_fine_alignment,
)


class LiveInputHarness(MissionFlowHarness):
    FINE_FORWARD_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.FINE_FORWARD_PRE_MOTION_SETTLE_SEC

    def _fresh_observations(self, now):
        return MotionDecisionNode._fresh_observations(self, now)


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    return now


def observe(node, clock, source, info, *, plan=True):
    clock[0] += .01
    MotionDecisionNode._info_callback(node, source)(String(data=json.dumps(info)))
    if plan:
        MotionDecisionNode._publish_decision(node)


def ground_ball(valid=True, angle=25.):
    return ball_info(
        raw_detected=True, confirmation_confirmed=True,
        ground_projection_enabled=True, ground_projection_valid=valid,
        ground_projection_scope='ball_approach_only',
        ground_coordinate_frame='robot_x_right_z_forward',
        ground_steering_angle_deg=angle,
    )


def test_ball_wait_uses_new_ground_angle_instead_of_invalid_old_sample(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    observe(node, clock, 'ball', ground_ball(False))
    assert not node.last_selected_decision.valid
    observe(node, clock, 'ball', ground_ball(True, -25.))
    command = node.publisher.messages[-1]
    assert command['valid']
    assert command['action'] == 'BALL_APPROACH_RECOVER_LEFT_4'
    assert command['source_command']['steering_error_deg'] == -25.


def test_line_wait_uses_new_heading(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    observe(node, clock, 'line', {**line_info(), 'ground_projection_valid': False})
    assert not node.last_selected_decision.valid
    observe(node, clock, 'line', line_info(heading=30., offset=.3))
    command = node.publisher.messages[-1]
    assert command['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'
    assert command['source_command']['heading_error_deg'] == 30.


def test_wait_does_not_reuse_expired_input(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    observe(node, clock, 'ball', ground_ball(), plan=False)
    clock[0] += node.timeouts['ball'] + .01
    MotionDecisionNode._publish_decision(node)
    assert not node.last_selected_decision.valid
    assert not any(message['valid'] for message in node.publisher.messages)


def test_goal_fine_wait_replans_changed_geometry_and_restarts_pause(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    fine = goal_info(depth_m=.6, score_now=False, confirmation_confirmed=True)
    observe(node, clock, 'goal', fine)
    assert node.last_selected_decision.action.startswith('GOAL_CAMERA90_FINE_FORWARD_')
    assert not node.publisher.messages
    first_start = node.pre_motion_settle_started_at
    clock[0] += .4
    observe(node, clock, 'goal', {**fine, 'depth_valid': False})
    assert not node.last_selected_decision.valid
    assert node.pre_motion_settle_started_at is None
    observe(node, clock, 'goal', fine)
    assert node.pre_motion_settle_started_at > first_start
    restart = node.pre_motion_settle_started_at
    for elapsed in (.4, .8):
        clock[0] = restart + elapsed
        observe(node, clock, 'goal', fine)
        assert not any(message['valid'] for message in node.publisher.messages)
    clock[0] = restart + 1.
    observe(node, clock, 'goal', fine)
    assert node.publisher.messages[-1]['action'].startswith('GOAL_CAMERA90_FINE_FORWARD_')


def test_post_motion_wait_stores_frames_but_requires_one_after_pause(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    node.correction_post_motion_dwell_until = clock[0] + 1.
    node.correction_post_motion_source = 'line'
    observe(node, clock, 'line', line_info(heading=-30., offset=-.3))
    assert node.latest_info['line']['ground_heading_error_deg'] == -30.
    assert not node.publisher.messages
    clock[0] += 1.
    MotionDecisionNode._publish_decision(node)
    assert node.latest_info['line'] is None
    observe(node, clock, 'line', line_info(heading=30., offset=.3))
    assert node.publisher.messages[-1]['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'


@pytest.mark.parametrize('action,source,extra', [
    ('STRAIGHT_0', 'ball', {}), ('STRAIGHT_0', 'hurdle', {}),
    ('BALL_FINE_FORWARD_8', 'ball', {}),
    ('GOAL_CAMERA90_FINE_FORWARD_1', 'goal', {}),
    ('GOAL_CAMERA90_FINE_FORWARD_4', 'goal', {}),
    ('GO', 'hurdle', {'fine_sequence_requested': True}),
    ('GO', 'hurdle', {'depth_fallback_requested': True}),
])
def test_each_fine_action_requires_full_second(clock, action, source, extra):
    node = LiveInputHarness()
    decision = MotionDecision('AUTO', source, action, True, 'test', False, False, extra)
    assert not node._pre_motion_settle_ready(decision, 10.)
    assert not node._pre_motion_settle_ready(decision, 10.999)
    assert node._pre_motion_settle_ready(decision, 11.)


@pytest.mark.parametrize('action,flag,extra', [
    ('BALL_PICKUP_FINE_FORWARD', 'pickup_fine_align_waiting', {}),
    ('BALL_PICKUP_INITIAL_ALIGN_CONTINUE', 'pickup_initial_align_waiting', {'pickup_approach_motion': 'STRAIGHT_0'}),
])
def test_fine_checkpoint_waits_inside_pickup_lock_without_deadlock(clock, action, flag, extra):
    node = LiveInputHarness()
    assert node.phase_manager.start_special_action('PICKUP_NOW', 1)
    setattr(node, flag, True)
    decision = MotionDecision('BALL_PICKUP', 'ball', action, True, 'test', False, False, extra)
    assert not node._pre_motion_settle_ready(decision, 10.)
    assert not node._pre_motion_settle_ready(decision, 10.999)
    assert node._pre_motion_settle_ready(decision, 11.)
    setattr(node, flag, False)
    assert not node._pre_motion_settle_ready(decision, 12.)


def test_hurdle_go_waits_for_fresh_fine_sequence_decision(clock):
    node = LiveInputHarness(phase='HURDLE_POSITIONING')
    info = hurdle_info(depth_m=.55, distance_m=.55, bottom_distance_px=200,
                       camera_center_offset_x_px=0, go_now=False)
    observe(node, clock, 'hurdle', info)
    assert node.last_selected_decision.action == 'GO'
    assert node.last_selected_decision.source_command['fine_sequence_requested']
    assert not node.publisher.messages
    start = node.pre_motion_settle_started_at
    for elapsed in (.4, .8, 1.):
        clock[0] = start + elapsed
        observe(node, clock, 'hurdle', info)
    assert node.publisher.messages[-1]['action'] == 'GO'
    assert node.active_special_action == 'GO'


def test_pickup_preparation_is_followed_by_full_second_before_fine(clock):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(fine_alignment_message('BALL_PICKUP_FINE_FORWARD'))
    bridge.executor_status_callback(executor_status(status='SUCCEEDED', motion_id='pickup_fine_prepare'))
    count = len(bridge.executor_request_publisher.messages)
    assert bridge.active_dwell_until == pytest.approx(clock[0] + 1.)
    deadline = bridge.active_dwell_until
    bridge._check_atomic_dwell(deadline - .001)
    assert len(bridge.executor_request_publisher.messages) == count
    # A repeated preparation status cannot restart the timer or skip it.
    clock[0] += .3
    bridge.executor_status_callback(executor_status(status='SUCCEEDED', motion_id='pickup_fine_prepare'))
    assert bridge.active_dwell_until == deadline
    bridge._check_atomic_dwell(deadline)
    assert len(bridge.executor_request_publisher.messages) == count + 1
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_fine_forward_0'
    bridge._check_atomic_dwell(deadline + 1.)
    assert len(bridge.executor_request_publisher.messages) == count + 1


def test_pickup_loss_pending_during_preparation_skips_fine_after_pause(clock):
    from test_motion_command_bridge import pickup_positioning_loss_message
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(fine_alignment_message('BALL_PICKUP_FINE_FORWARD'))
    bridge.executor_status_callback(executor_status(status='SUCCEEDED', motion_id='pickup_fine_prepare'))
    bridge.navigation_command_callback(pickup_positioning_loss_message(bridge, 'pickup_fine_prepare', command_id=9000))
    assert bridge.pickup_positioning_loss_pending
    count = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert len(bridge.executor_request_publisher.messages) == count
    assert bridge.pickup_fine_align_waiting
