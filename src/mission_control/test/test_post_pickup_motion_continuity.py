"""Exercise pickup exit, Goal reservations and fine-to-backward timer boundaries."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh, start_goal
from test_goal_target_steering import target_sample
from test_gait_transitions import finish_request, requests
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages, executor_status
from test_second_ball_line_resume import finish_pickup
from test_post_ball_goal_handoff import receive
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.mark.parametrize('ball_index', [1, 2])
def test_pickup_exit_turn_keeps_half_second_timer(clock, ball_index):
    bridge = FakeBridge(advance_transition_dwells=False)
    sequence = Bridge._with_pickup_motion_dwells(Bridge._pickup_motion_sequence({
        'source_command': {},
        'mission_progress': {'pickups_completed': ball_index - 1},
    }))
    motion = ('pickup_first_backward_turn_right' if ball_index == 1
              else 'pickup_second_backward_turn_left')
    assert sequence[-2:] == (motion, Bridge.DWELL_MARKER)
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_motion_id = motion
    bridge.active_pickup_sequence = sequence
    bridge.active_sequence_index = len(sequence) - 2
    bridge.executor_status_callback(executor_status(
        command_id=10, request_id=10, motion_id=motion, status='SUCCEEDED'))
    assert bridge.motion_in_progress
    assert bridge.active_dwell_until == 10.5
    clock[0] = 10.499
    bridge._check_atomic_dwell()
    assert bridge.motion_in_progress
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    assert bridge.active_dwell_until is None
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('ball_index', [1, 2])
@pytest.mark.parametrize('heading', [0.])
def test_exit_uses_first_fresh_line_without_restarting_settle(clock, ball_index, heading):
    node = finish_pickup(ball_index)
    receive(node, clock, 'line', line_info(heading=heading))
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['valid']
    assert node.pre_motion_settle_started_at is None


@pytest.mark.parametrize('first_angle,next_angle', [(0, -30), (0, 30), (-30, 0), (30, 0), (-30, 30), (0, 0)])
def test_goal_reservation_reaches_executor_before_current_walk_finishes(clock, first_angle, next_angle):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    bridge = FakeBridge(advance_transition_dwells=False)
    first = start_goal(node, clock, 1.5, first_angle)
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    first_request = requests(bridge)[-1]
    clock[0] += Node.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(3):
        fresh(node, clock, 'goal', target_sample(next_angle, 1.5), plan=False)
    second = node.publisher.messages[-1]
    assert second['command_id'] != first['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    assert len(requests(bridge)) == 2
    assert not bridge.queued_request_deferred
    assert bridge.transition_dwell_until is None
    bridge.executor_status_callback(executor_status(**first_request, status='SUCCEEDED'))
    assert bridge.active_command_id == second['command_id']
    assert len(requests(bridge)) == 2


@pytest.mark.parametrize('previous,action,prepare', [
    ('goal_camera_90_forward_4', 'GOAL_CAMERA90_FINE_FORWARD_3', 'goal_fine_prepare_90'),
    ('line_recovery_left_4', 'GOAL_CAMERA90_FINE_FORWARD_3', 'goal_fine_prepare_90'),
    ('goal_camera_90_fine_forward_3', 'GOAL_CAMERA_90_FORWARD_2', 'fine_to_turn_ready_90'),
    ('goal_camera_90_fine_forward_3', 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'fine_to_turn_ready_90'),
])
@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED', 'CANCELLED', 'TIMEOUT', 'REJECTED'])
def test_goal_posture_interpolation_keeps_half_second_holds(clock, previous, action, prepare, status):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(action=action, source='goal'))
    assert not requests(bridge)
    assert bridge.transition_dwell_until == clock[0] + .5
    clock[0] += .5
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in requests(bridge)] == [prepare]
    finish_request(bridge, status, '' if status == 'SUCCEEDED' else 'TEST_FAILURE')
    if status == 'SUCCEEDED':
        assert bridge.transition_dwell_until == clock[0] + .5
        assert [r['motion_id'] for r in requests(bridge)] == [prepare]
        clock[0] += .5
        bridge._check_atomic_dwell()
    expected = [prepare, bridge.motion_id_for_action(action)] if status == 'SUCCEEDED' else [prepare]
    assert [r['motion_id'] for r in requests(bridge)] == expected
    assert bridge.transition_dwell_until is None



@pytest.mark.parametrize('previous', sorted(Bridge.FINE_FORWARD_MOTION_IDS))
@pytest.mark.parametrize('backward', sorted(Bridge.BACKWARD_MOTION_IDS))
def test_fine_to_backward_holds_exactly_one_second_once(clock, previous, backward):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_motion_id = backward
    bridge._publish_executor_request(action='PICKUP_NOW', command_id=10, event_id=None,
                                    request_id=10, motion_id=backward, timeout_ms=12000)
    assert not requests(bridge)
    assert bridge.transition_dwell_until == 11.
    clock[0] = 10.999
    bridge._check_atomic_dwell()
    assert not requests(bridge)
    clock[0] = 11.
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in requests(bridge)] == [backward]
    clock[0] = 13.
    finish_request(bridge)
    assert bridge.transition_dwell_until == 13.5


@pytest.mark.parametrize('phase,action', [
    ('POST_BALL_LINE_ALIGN', 'POST_BALL_LINE_TURN_RIGHT_5'),
    ('LINE_TRACK_AFTER_PICKUP', 'LINE_HEADING_TURN_RIGHT_5'),
])
def test_successful_turn_reuses_fresh_in_motion_line_without_dwell(clock, phase, action):
    node = finish_pickup(1)
    node.phase_manager.current_phase = phase
    node.active_general_source = 'line'
    node.general_motion_gate.on_command_published(action, command_id=99)
    node.send_status(action, 99, 'RUNNING')
    receive(node, clock, 'line', line_info())
    clock[0] += .2
    node.send_status(action, 99, 'SUCCEEDED')
    assert node.latest_info['line'] is not None
    assert node.post_ball_line_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert node.publisher.messages[-1]['valid']
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT'


def test_queued_backward_one_second_starts_after_fine_motion_completion(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(
        action='GOAL_CAMERA90_FINE_FORWARD_3', source='goal', command_id=1))
    bridge.navigation_command_callback(navigation_message(
        action='BALL_LOST_BACKWARD_2', source='ball', command_id=2))
    assert bridge.queued_request_deferred
    assert bridge.transition_dwell_until is None
    clock[0] = 12.
    finish_request(bridge)
    assert bridge.transition_dwell_until == 13.
    assert len(requests(bridge)) == 1
    clock[0] = 13.
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in requests(bridge)] == [
        'goal_camera_90_fine_forward_3', 'ball_lost_camera_down_backward_2',
    ]


def test_confirmed_two_point_forward_starts_without_extra_settle_after_pickup(clock):
    from test_sparse_line_recovery import sparse_info

    node = LiveInputHarness(phase='LINE_TRACK_AFTER_PICKUP')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    info = sparse_info(heading=0., distance=.6)
    info.update(filtered_lateral_offset_norm=0., heading_error_deg=0., turn_angle_deg=0.)
    for index in range(3):
        clock[0] = 10. + index * .1
        receive(node, clock, 'line', info)
        Node._publish_decision(node)
        if index < 2:
            assert not any(m['valid'] for m in node.publisher.messages)
    valid = [m for m in node.publisher.messages if m['valid']]
    assert len(valid) == 1
    assert valid[0]['action'] == 'LINE_SPARSE_FORWARD'
    assert node.pre_motion_settle_started_at is None
