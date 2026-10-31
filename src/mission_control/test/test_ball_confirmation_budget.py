"""Two confirmed Ball modes are allowed, even if neither reaches the grasp."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_wait_refresh_and_fine_settle import LiveInputHarness, ground_ball
from test_post_ball_goal_handoff import receive
from test_mission_phase_flow import line_info


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def observe(node, clock, info):
    clock[0] += .1
    receive(node, clock, 'ball', info)


def confirmed():
    return ground_ball(angle=0.)


def fail_before_grasp(node, command_id):
    assert node.phase_manager.start_special_action('PICKUP_NOW', command_id)
    node.send_status('PICKUP_NOW', command_id, 'RUNNING', motion_id='pickup_camera_down')
    node.send_status('PICKUP_NOW', command_id, 'FAILED', motion_id='pickup_camera_down')


def test_confirmation_consumes_once_before_any_motion_and_loss_does_not_refund(clock):
    node = LiveInputHarness(phase='AUTO')
    observe(node, clock, {**confirmed(), 'detected': False, 'confirmation_confirmed': False})
    assert node.phase_manager.ball_modes_confirmed == 0
    observe(node, clock, confirmed())
    assert node.phase_manager.ball_modes_confirmed == 1
    assert node.phase_manager.snapshot()['ball_modes_remaining'] == 1
    assert node.phase_manager.pickups_executed == 0
    assert not node.publisher.messages
    for _ in range(20):
        observe(node, clock, confirmed())
    observe(node, clock, {'detected': False, 'raw_detected': False})
    observe(node, clock, confirmed())
    assert node.phase_manager.ball_modes_confirmed == 1
    assert node.phase_manager.active_ball_mode_index == 1


def test_two_confirmations_exhaust_modes_even_when_both_fail_before_grasp(clock):
    node = LiveInputHarness(phase='AUTO')
    for index in (1, 2):
        observe(node, clock, confirmed())
        assert node.phase_manager.active_ball_mode_index == index
        assert node.phase_manager.ball_modes_confirmed == index
        assert not Node._ball_navigation_blocked(node)
        progress = Node._mission_progress(node)
        sequence = Bridge._pickup_motion_sequence({'mission_progress': progress, 'source_command': {}})
        assert sequence[-2] == (
            'pickup_first_backward_turn_right' if index == 1 else 'pickup_second_backward_turn_left'
        )
        fail_before_grasp(node, index)
        assert node.phase_manager.pickups_executed == 0
        assert node.phase_manager.pickups_completed == 0
        assert node.phase_manager.ball_modes_confirmed == index
        assert node.phase_manager.active_ball_mode_index is None
    observe(node, clock, {'detected': False, 'raw_detected': False})
    for _ in range(5):
        observe(node, clock, confirmed())
        decision = Node._select_mission_decision(node, {'ball': confirmed(), 'line': line_info()}, .1)
        assert decision.source != 'ball'
    assert node.phase_manager.ball_modes_confirmed == 2
    assert node.phase_manager.snapshot()['ball_modes_remaining'] == 0
    assert not node.phase_manager.start_special_action('PICKUP_NOW', 3)


def test_failed_approach_consumes_mode_without_ever_starting_pickup(clock):
    node = LiveInputHarness(phase='AUTO')
    observe(node, clock, confirmed())
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['valid'] and command['source'] == 'ball'
    assert command['action'] != 'PICKUP_NOW'
    original_phase = node.mission_phase
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], 'FAILED')
    assert node.phase_manager.ball_modes_confirmed == 1
    assert node.phase_manager.pickups_executed == 0
    assert node.phase_manager.active_ball_mode_index is None
    assert node.phase_manager.ball_sections_processed == 0
    assert node.mission_phase == original_phase
    observe(node, clock, confirmed())
    assert node.phase_manager.active_ball_mode_index == 2


@pytest.mark.parametrize('index', [None, True, 0, 3, '2'])
def test_bridge_rejects_invalid_confirmed_mode_index(index):
    assert Bridge._pickup_motion_sequence({
        'mission_progress': {'pickups_completed': 0, 'active_ball_mode_index': index},
        'source_command': {},
    }) is None


@pytest.mark.parametrize('grasp', ['GRABBED', 'NOT_GRABBED'])
def test_second_mode_finishes_with_zero_remaining_and_keeps_its_own_result(clock, grasp):
    node = LiveInputHarness(phase='AUTO')
    observe(node, clock, confirmed())
    fail_before_grasp(node, 1)
    observe(node, clock, confirmed())
    assert node.phase_manager.snapshot()['ball_modes_remaining'] == 0
    assert node.phase_manager.start_special_action('PICKUP_NOW', 2)
    node.send_status('PICKUP_NOW', 2, 'RUNNING', motion_id='pickup_400')
    assert node.phase_manager.record_active_pickup_grasp_result(grasp)
    node.send_status('PICKUP_NOW', 2, 'SUCCEEDED', motion_id='pickup_second_backward_turn_left')
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'
    assert node.planner.post_ball_line_search_direction == 'LEFT'
    assert node.phase_manager.completed_ball_index == 2
    assert node.phase_manager.grasp_result_for_ball(2) == grasp
    assert node.phase_manager.pickups_completed == 1
    assert node.phase_manager.complete_post_ball_line_align()
    if grasp == 'GRABBED':
        assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
        assert node.phase_manager.complete_post_ball_line_run()
        assert node.mission_phase == 'POST_BALL_GOAL_TRANSITION'
        assert node.phase_manager.grasp_result_for_next_shot() == 'GRABBED'
    else:
        assert node.phase_manager.ball_sections_processed == 2
        assert node.mission_phase == 'AUTO'


@pytest.mark.parametrize('phase', ['GOAL_APPROACH', 'POST_BALL_LINE_ALIGN', 'POST_SHOT_TURN', 'HURDLE_POSITIONING'])
def test_other_mission_detections_do_not_start_an_extra_ball_mode(clock, phase):
    node = LiveInputHarness(phase=phase)
    observe(node, clock, confirmed())
    assert node.phase_manager.ball_modes_confirmed == 0
