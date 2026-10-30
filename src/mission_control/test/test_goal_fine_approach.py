"""850 mm handoff and Goal walking transitions without fixed dwell timers."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.goal_navigation_planner import GoalNavigationPlanner
from test_goal_target_steering import target_sample
from test_continuous_approach import clock, fresh, start_goal
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import FakeBridge, navigation_message
from test_gait_transitions import finish_request, requests


@pytest.mark.parametrize('depth', [.570, .600, .849999, .850])
@pytest.mark.parametrize('bearing,action', [
    (0, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (14.999, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (15, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (29.999, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (30, 'GOAL_CAMERA90_TURN_RIGHT_3'),
    (30.001, 'GOAL_CAMERA90_TURN_RIGHT_3'),
    (45, 'GOAL_CAMERA90_TURN_RIGHT_5'),
    (-7.999, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (-8, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (-29.999, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (-30, 'GOAL_CAMERA90_TURN_LEFT_2'),
    (-30.001, 'GOAL_CAMERA90_TURN_LEFT_2'),
    (-45, 'GOAL_CAMERA90_TURN_LEFT_3'),
])
def test_fine_approach_turns_by_backboard_bearing_before_advancing(depth, bearing, action):
    sample = target_sample(0, depth)
    sample.update(bearing_deg=bearing, offset_x_px=200 if bearing > 0 else -200 if bearing < 0 else 0)
    command = GoalNavigationPlanner().plan(sample)
    decision = MotionDecisionPlanner().plan('GOAL_APPROACH', {'goal': sample}, .1)
    assert command.action == decision.action == action
    assert command.target_steering_angle_deg is None
    if 'TURN' in action:
        assert decision.source_command['turn_count'] == int(action.rsplit('_', 1)[1])


@pytest.mark.parametrize('changes', [
    {'bearing_deg': None}, {'bearing_deg': float('nan')},
    {'bearing_deg': float('inf')}, {'bearing_deg': 90},
    {'bearing_deg': -15, 'offset_x_px': 200},
])
def test_fine_approach_waits_for_valid_consistent_bearing(changes):
    command = GoalNavigationPlanner().plan({**target_sample(0, .700), **changes})
    assert command.action == 'WAIT'
    assert not command.valid


@pytest.mark.parametrize('bearing,offset,expected_turn', [
    (30, 200, 'GOAL_CAMERA90_TURN_RIGHT_3'),
    (-30, -200, 'GOAL_CAMERA90_TURN_LEFT_2'),
])
def test_fine_step_rechecks_turn_then_requires_fresh_view_before_advancing(
    clock, bearing, offset, expected_turn,
):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, .700)
    clock[0] += MotionDecisionNode.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    misaligned = {**target_sample(0, .700), 'bearing_deg': bearing, 'offset_x_px': offset}
    for _ in range(3):
        fresh(node, clock, 'goal', misaligned, plan=False)
        clock[0] += .01
    # A stationary correction must wait for the active walking chunk to finish.
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    fresh(node, clock, 'goal', misaligned)
    turn = node.publisher.messages[-1]
    assert turn['action'] == expected_turn
    node.send_status(turn['action'], turn['command_id'], 'RUNNING')
    node.send_status(turn['action'], turn['command_id'], 'SUCCEEDED')
    deadline = node.goal_post_motion_dwell_until
    assert deadline == pytest.approx(clock[0] + 1.0)
    published = len(node.publisher.messages)
    clock[0] = deadline - .001
    fresh(node, clock, 'goal', target_sample(0, .700))
    assert len(node.publisher.messages) == published
    clock[0] = deadline
    MotionDecisionNode._publish_decision(node)
    assert node.latest_info['goal'] is None
    clock[0] += .01
    fresh(node, clock, 'goal', target_sample(0, .700))
    assert node.publisher.messages[-1]['action'] == 'GOAL_CAMERA90_FINE_FORWARD_3'


@pytest.mark.parametrize('angle,action', [
    (0, 'GOAL_CAMERA_90_FORWARD_2'), (30, 'GOAL_CAMERA90_RECOVER_RIGHT_4'),
    (-30, 'GOAL_CAMERA90_RECOVER_LEFT_4'), (45, 'GOAL_CAMERA90_TURN_RIGHT_5'),
])
def test_just_above_850mm_still_uses_target_steering(angle, action):
    assert GoalNavigationPlanner().plan(target_sample(angle, .850001)).action == action


@pytest.mark.parametrize('previous,action,prepare', [
    ('goal_camera_90_forward_4', 'GOAL_CAMERA90_FINE_FORWARD_3', 'pickup_fine_prepare'),
    ('line_recovery_left_4', 'GOAL_CAMERA90_FINE_FORWARD_3', 'pickup_fine_prepare'),
    ('line_recovery_right_4', 'GOAL_CAMERA90_FINE_FORWARD_3', 'pickup_fine_prepare'),
    ('goal_camera_90_fine_forward_3', 'GOAL_CAMERA_90_FORWARD_2', 'fine_to_turn_ready_90'),
    ('goal_camera_90_fine_forward_3', 'GOAL_CAMERA90_RECOVER_LEFT_4', 'fine_to_turn_ready_90'),
    ('goal_camera_90_fine_forward_3', 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'fine_to_turn_ready_90'),
])
def test_goal_walks_keep_posture_transition_without_pre_or_post_pause(clock, previous, action, prepare):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(source='goal', action=action))
    assert [r['motion_id'] for r in requests(bridge)] == [prepare]
    assert bridge.transition_dwell_until is None
    assert bridge.active_dwell_until is None
    # Completing the real posture interpolation immediately sends the next walk.
    finish_request(bridge)
    assert [r['motion_id'] for r in requests(bridge)] == [prepare, bridge.motion_id_for_action(action)]
    assert bridge.transition_dwell_until is None
    assert bridge.active_dwell_until is None
    assert clock[0] == 10.0
    finish_request(bridge)
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED', 'CANCELLED'])
def test_failed_goal_posture_transition_never_starts_next_walk(clock, status):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'goal_camera_90_forward_4'
    bridge.navigation_command_callback(navigation_message(source='goal', action='GOAL_CAMERA90_FINE_FORWARD_3'))
    finish_request(bridge, status, 'TEST_FAILURE')
    assert [r['motion_id'] for r in requests(bridge)] == ['pickup_fine_prepare']
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('initial_angle', [0, -30, 30])
def test_850mm_handoff_reserves_fine_step_and_executes_without_extra_pause(clock, initial_angle):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, 1.0, initial_angle)
    clock[0] += MotionDecisionNode.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(3):
        fresh(node, clock, 'goal', target_sample(30, .850), plan=False)
        clock[0] += .01
    second = node.publisher.messages[-1]
    assert second['action'] == 'GOAL_CAMERA90_FINE_FORWARD_3'
    assert node.queued_general_command_id == second['command_id']

    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action=first['action'], source='goal', command_id=1))
    bridge.navigation_command_callback(navigation_message(action=second['action'], source='goal', command_id=2))
    assert len(requests(bridge)) == 1
    finish_request(bridge)
    assert requests(bridge)[-1]['motion_id'] == 'pickup_fine_prepare'
    assert bridge.transition_dwell_until is None
    finish_request(bridge)
    assert requests(bridge)[-1]['motion_id'] == 'goal_camera_90_fine_forward_3'
    assert bridge.transition_dwell_until is None


def test_fine_step_to_fine_step_is_queued_without_a_dwell(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    for command_id in (1, 2):
        bridge.navigation_command_callback(navigation_message(
            action='GOAL_CAMERA90_FINE_FORWARD_3', source='goal', command_id=command_id))
    assert len(requests(bridge)) == 2
    assert bridge.queued_request_id == 2
    assert bridge.transition_dwell_until is None
