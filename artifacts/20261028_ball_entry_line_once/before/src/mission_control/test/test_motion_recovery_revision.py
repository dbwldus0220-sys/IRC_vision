"""Regression coverage for one-second corrections and the second-ball route."""

from pathlib import Path
import json
import time

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.mission_phase_manager import MissionPhaseManager
from test_motion_decision_planner import ball_info
from test_motion_decision_node import (
    FreshMockInputNode, ReadinessPublishNode, general_decision, send_status,
)
from test_motion_command_bridge import (
    FakeBridge, navigation_message, fine_alignment_message,
    enter_pickup_fine_alignment, complete_active_motion,
    hurdle_fine_sequence_message, executor_status,
)


@pytest.mark.parametrize('stage', ['approach', 'INITIAL', 'FINE', 'normal'])
def test_interior_loss_searches_once_and_rearms_on_reacquisition(stage):
    planner = MotionDecisionPlanner()
    visible = ball_info(image_width=1280, image_height=720, bbox=[500,300,600,400])
    planner._update_ball_tracking(visible, 0.0)
    missing = {'detected': False, 'raw_detected': False, 'ball_loss_confirmed': True}

    def plan(info):
        if stage == 'approach':
            return planner.plan_lost_ball_approach_alignment(info).action
        if stage == 'normal':
            planner.ball_lost_elapsed_sec = 1.0
            return planner._plan_source('ball', info, 0.1)['motion']
        fn = (planner.plan_ball_pickup_initial_alignment if stage == 'INITIAL'
              else planner.plan_ball_pickup_fine_alignment)
        return fn(info).action

    expected = ('BALL_LOST_FORWARD_4' if stage in {'approach', 'normal'}
                else f'BALL_PICKUP_{stage}_SEARCH_FORWARD_4')
    assert plan(missing) == expected
    assert plan(missing) == expected  # Planning alone cannot consume a request.
    planner.ball_interior_loss_forward_sent = True
    assert plan(missing) != expected
    planner._update_ball_tracking(visible, 0.0)
    assert plan(missing) == expected


@pytest.mark.parametrize('missing', [None, {'detected': False},
    {'detected': False, 'raw_detected': True, 'ball_loss_confirmed': True},
    {'detected': True, 'ball_loss_confirmed': True},
])
def test_interior_recovery_needs_confirmed_loss(missing):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(
        image_width=1280, image_height=720, bbox=[500,300,600,400]), 0.0)
    assert planner.plan_lost_ball_approach_alignment(missing).action != 'BALL_LOST_FORWARD_4'


@pytest.mark.parametrize('bbox', [None, [0,300,70,400], [1210,300,1280,400],
    [500,0,600,100], [500,650,600,720]])
def test_edge_or_unknown_loss_does_not_use_interior_forward(bbox):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(image_width=1280, image_height=720, bbox=bbox), 0.0)
    decision = planner.plan_lost_ball_approach_alignment({
        'detected': False, 'raw_detected': False, 'ball_loss_confirmed': True})
    assert decision.action != 'BALL_LOST_FORWARD_4'


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_pickup_interior_recovery_keeps_parent_checkpoint(stage):
    bridge = FakeBridge()
    if stage == 'FINE':
        enter_pickup_fine_alignment(bridge)
    else:
        bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    sequence = bridge.active_pickup_sequence
    parent = bridge.active_command_id
    bridge.navigation_command_callback(fine_alignment_message(
        f'BALL_PICKUP_{stage}_SEARCH_FORWARD_4', command_id=8500))
    assert bridge.active_motion_id == 'ball_camera_down_forward_4'
    assert bridge.active_command_id == parent
    complete_active_motion(bridge)
    assert bridge.active_pickup_sequence == sequence
    assert bridge.pickup_fine_align_waiting == (stage == 'FINE')
    assert bridge.pickup_initial_align_waiting == (stage == 'INITIAL')
    assert not bridge.pickup_fixed_sequence_started


@pytest.mark.parametrize('action,source', [
    ('LEFT','line'), ('RIGHT','line'), ('LINE_LOST_TURN_LEFT','line'),
    ('BALL_FINE_FORWARD_8','ball'), ('GOAL_CAMERA90_FINE_FORWARD_2','goal'),
    ('GOAL_CAMERA90_CRAB_LEFT','goal'), ('GOAL_CAMERA90_BACKWARD_1','goal'),
])
def test_correction_success_reserves_one_second_and_needs_fresh_vision(monkeypatch, action, source):
    clock = [10.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    node = ReadinessPublishNode(general_decision("STRAIGHT"))
    node.active_general_source = source
    node.general_motion_gate.on_command_published(action, 31)
    send_status(node, status='RUNNING', action=action, command_id=31, event_id=None, dynamics_command=None)
    send_status(node, status='SUCCEEDED', action=action, command_id=31, event_id=None, dynamics_command=None)
    assert node.correction_post_motion_dwell_until == 11.0
    node.publisher.messages.clear()
    clock[0] = 10.999
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages
    clock[0] = 11.0
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages
    assert node.latest_info[source] is None
    assert not node.general_motion_gate.has_required_fresh_vision()
    # A duplicate completion cannot add another pause.
    send_status(node, status='SUCCEEDED', action=action, command_id=31, event_id=None, dynamics_command=None)
    assert node.correction_post_motion_dwell_until is None


def test_turn_cannot_prequeue_straight_before_its_pause():
    node = ReadinessPublishNode(general_decision('STRAIGHT'))
    node.general_motion_gate.on_command_published('LEFT', 1)
    assert not MotionDecisionNode._line_only_prequeue_allowed(node, {})


def test_duplicate_hurdle_fine_completion_cannot_skip_pause_or_count_twice(monkeypatch):
    monkeypatch.setattr(time, 'monotonic', lambda: 10.0)
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    first_id = bridge.active_request_id
    complete_active_motion(bridge)
    for _ in range(2):
        bridge.executor_status_callback(executor_status(
            status='SUCCEEDED', command_id=8100, request_id=first_id,
            motion_id='hurdle_fine_forward_10'))
    assert bridge.hurdle_sequence_fine_completed == 1
    assert bridge.active_dwell_until == 11.0
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('direction,korean', [('LEFT','좌'), ('RIGHT','우')])
def test_empty_second_pickup_returns_to_normal_line_and_corner_catalog(direction, korean):
    manager = MissionPhaseManager(initial_phase='BALL_APPROACH')
    manager.pickups_completed = 1
    manager.ball_sections_processed = 1
    assert manager.start_special_action('PICKUP_NOW', 7)
    assert manager.record_active_pickup_grasp_result('NOT_GRABBED')
    manager.handle_motion_status('PICKUP_NOW', 7, 'RUNNING')
    manager.handle_motion_status('PICKUP_NOW', 7, 'SUCCEEDED')
    assert manager.complete_post_ball_line_align()
    assert manager.current_phase == 'AUTO'
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motion_id = MotionCommandBridgeNode.motion_id_for_action(direction)
    assert aliases[motion_id] == f'찐라인복귀{korean}회전45도(4회)'
    catalog = json.loads((root/'artifacts/robot_motions_runtime.json').read_text())['motions']
    assert aliases[motion_id] in {entry['name'] for entry in catalog}
