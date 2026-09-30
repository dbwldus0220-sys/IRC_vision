"""Exercise final hurdle gating, bounded goal search, and line handoff."""

import json
from pathlib import Path
import time

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from test_motion_decision_planner import hurdle_info, goal_info
from test_mission_phase_flow import MissionFlowHarness, line_info, release_general
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages, complete_active_motion
from test_motion_decision_node import FakeDecisionNode, arm_special_command, send_grasp_detections


@pytest.mark.parametrize('depth,action', [
    (.700001, 'STRAIGHT_0'), (.7, 'STRAIGHT_0'), (.550001, 'STRAIGHT_0'), (.55, 'STRAIGHT_0'),
    (.540001, 'STRAIGHT_0'), (.54, 'GO'), (.43, 'GO'),
    (.42, 'GO'), (.200001, 'GO'), (.2, 'GO'), (.19, 'GO'),
])
def test_hurdle_approach_is_separate_from_final_atomic_tail(depth, action):
    planner = MotionDecisionPlanner()
    decision = planner.plan('HURDLE_POSITIONING', {'hurdle': hurdle_info(
        depth_m=depth, camera_center_offset_x_px=12, bottom_distance_px=92,
        hurdle_angle_deg=24.4, go_now=False,
    )}, .1)
    assert decision.action == action
    assert decision.requires_ack == (action == 'GO')
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action=decision.action, source_command=decision.source_command,
    ))
    if action != 'GO':
        complete_active_motion(bridge)
        bridge._check_atomic_dwell(1e12)
        assert not bridge.motion_in_progress
        assert not any(r['motion_id'] == 'hurdle' for r in decoded_messages(bridge.executor_request_publisher))
        assert bridge.hurdle_sequence_fine_completed == 0
    else:
        assert bridge.active_pickup_sequence == (
            'pickup_fine_forward_0', bridge.DWELL_MARKER,
            'pickup_fine_forward_0', bridge.HURDLE_PRE_GO_DWELL_MARKER, 'hurdle',
        )


@pytest.mark.parametrize('direction,count,offset', [('LEFT', 1, -.3), ('RIGHT', 2, .3)])
def test_goal_loss_dispatches_requested_catalog_motion_and_remains_bounded(direction, count, offset):
    planner = MotionDecisionPlanner()
    planner.plan('GOAL_APPROACH', {'goal': goal_info(depth_m=1., offset_x_norm=offset)}, .1)
    missing = {'goal': {'detected': False}}
    limiter = LostSearchTurnLimiter()
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((root/'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for index in range(3):
        # Search must not silently lose its remembered side after a slow turn.
        decision = limiter.filter(planner.plan('GOAL_APPROACH', missing, 10.), missing)
        assert decision.valid and decision.action == f'GOAL_CAMERA90_TURN_{direction}_{count}'
        assert decision.source_command['turn_angle_deg'] == 15.
        bridge = FakeBridge()
        bridge.navigation_command_callback(navigation_message(source='goal', action=decision.action, source_command=decision.source_command))
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        name = f'찐제자리{"좌" if direction == "LEFT" else "우"}회전90도-1({count}회)'
        assert aliases[request['motion_id']] == name
        assert name in catalog
        limiter.record_published(decision, missing)
    blocked = limiter.filter(planner.plan('GOAL_APPROACH', missing, 1.), missing)
    assert not blocked.valid and blocked.reason == 'lost_search_turn_limit_reached'
    assert planner.plan('GOAL_APPROACH', {'goal': None}, .1).action == 'WAIT'
    reacquired = planner.plan('GOAL_APPROACH', {'goal': goal_info(depth_m=1., offset_x_norm=0)}, .1)
    assert reacquired.reason != 'turn_toward_last_seen_goal_side'


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('direction,offset', [('LEFT', -.3), ('RIGHT', .3)])
def test_post_shot_forward_completion_preserves_recent_line_side(monkeypatch, section, direction, offset):
    monkeypatch.setattr(time, 'monotonic', lambda: 10.)
    node = MissionFlowHarness(phase='POST_SHOT_FORWARD')
    node.phase_manager.ball_sections_processed = section
    node.planner.observe_line_for_search(line_info(offset=offset))
    node.planner.observe_line_for_search(line_info(offset=0))
    node.active_general_source = 'line'
    node.general_motion_gate.on_command_published('POST_SHOT_FORWARD', 42)
    release_general(node, {'action': 'POST_SHOT_FORWARD', 'command_id': 42})
    assert node.planner.last_line_seen_direction == direction
    assert node.latest_info['line'] is None
    decision = node._select_mission_decision({'line': {'detected': False}}, .1)
    assert decision.valid and decision.action == f'LINE_LOST_TURN_{direction}'


def test_goal_direction_updates_during_executing_motion(monkeypatch):
    monkeypatch.setattr(time, 'monotonic', lambda: 10.)
    node = MissionFlowHarness(phase='GOAL_APPROACH')
    node.planner._update_goal_tracking(goal_info(offset_x_norm=-.3), 0)
    node.general_motion_gate.on_command_published('GOAL_CAMERA_90_FORWARD', 42)
    MotionDecisionNode._info_callback(node, 'goal')(String(data=json.dumps(goal_info(offset_x_norm=.3))))
    assert node.planner.last_goal_turn_direction == 'RIGHT'


@pytest.mark.parametrize('frames,expected', [(0, 'UNKNOWN'), (14, 'NOT_GRABBED'), (15, 'GRABBED')])
def test_grasp_display_is_armed_only_when_window_closes(monkeypatch, frames, expected):
    monkeypatch.setattr(time, 'monotonic', lambda: 10.)
    node = FakeDecisionNode('BALL_APPROACH')
    arm_special_command(node, 'PICKUP_NOW', 10, 1)
    MotionDecisionNode._start_grasp_verification_window(node)
    for stamp in range(1, frames + 1):
        send_grasp_detections(node, stamp, [{'class_name': 'grab', 'confidence': .9}])
    assert node.grasp_result_display_until is None
    MotionDecisionNode._finish_grasp_verification_window(node)
    assert node.grasp_verification_result == expected
    assert node.grasp_result_display_until == (None if frames == 0 else 13.)
    MotionDecisionNode._start_grasp_verification_window(node)
    assert node.grasp_result_display_until is None


@pytest.mark.parametrize('bbox,direction', [([50, 20, 150, 100], 'LEFT'), ([1100, 20, 1200, 100], 'RIGHT')])
def test_goal_search_direction_uses_image_side_over_calibrated_bearing(bbox, direction):
    planner = MotionDecisionPlanner()
    planner._update_goal_tracking(goal_info(
        image_width=1280, bbox=bbox, bearing_deg=50., offset_x_norm=.5,
    ), 0)
    assert planner.last_goal_turn_direction == direction
