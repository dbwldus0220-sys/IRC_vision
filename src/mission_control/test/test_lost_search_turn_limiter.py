"""Regression tests for bounded loss recovery and the post-shot handoff."""

from dataclasses import replace
import json
from pathlib import Path

import pytest
import yaml

from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from mission_control.motion_decision_planner import MotionDecision, MotionDecisionPlanner
from test_motion_decision_planner import line_info


def turn(action='LINE_LOST_TURN_RIGHT', source='line', reason='search', **command):
    return MotionDecision('AUTO', source, action, True, reason, False, False, command)


def test_candidates_do_not_spend_budget_and_failed_requests_are_not_refunded():
    limiter = LostSearchTurnLimiter()
    lost = {'line': {'detected': False}}
    for _ in range(100):
        assert limiter.filter(turn(), lost).valid
    for _ in range(2):
        decision = limiter.filter(turn(), lost)
        assert decision.valid
        limiter.record_published(decision, lost)
    blocked = limiter.filter(turn(), lost)
    assert not blocked.valid
    assert blocked.action == 'WAIT'
    assert blocked.reason == 'lost_search_turn_limit_reached'
    assert limiter.counts['line'] == 2
    assert limiter.angles['line'] == 90.0
    assert not limiter.filter(turn(), {}).valid


def test_small_turns_stop_at_count_limit_and_direction_changes_do_not_cancel():
    limiter = LostSearchTurnLimiter()
    lost = {'line': {'detected': False}}
    for action in ['POST_SHOT_LINE_TURN_RIGHT_2', 'POST_SHOT_LINE_TURN_LEFT_1',
                   'POST_SHOT_LINE_TURN_RIGHT_2']:
        decision = limiter.filter(turn(action), lost)
        assert decision.valid
        limiter.record_published(decision, lost)
    assert limiter.angles['line'] == 45.0
    assert not limiter.filter(turn('POST_SHOT_LINE_TURN_LEFT_1'), lost).valid


@pytest.mark.parametrize('source,action', [
    ('line', 'LINE_LOST_TURN_LEFT'),
    ('ball', 'BALL_PICKUP_FINE_SEARCH_LEFT'),
    ('goal', 'GOAL_CAMERA90_TURN_LEFT_2'),
    ('hurdle', 'ALIGN_LEFT'),
])
def test_limits_cover_each_source_and_require_published_tracking_to_rearm(source, action):
    limiter = LostSearchTurnLimiter(max_turns=1)
    lost = {source: {'detected': False}}
    search = turn(action, source, turn_angle_deg=30.0)
    limiter.record_published(limiter.filter(search, lost), lost)
    assert not limiter.filter(search, lost).valid
    assert limiter.counts[next(s for s in limiter.counts if s != source)] == 0
    tracking = turn('STRAIGHT', source)
    visible = {source: {'detected': True, 'confirmation_confirmed': True}}
    limiter.filter(tracking, visible)
    assert not limiter.filter(search, lost).valid
    limiter.record_published(replace(tracking, valid=False), visible)
    assert not limiter.filter(search, lost).valid
    limiter.record_published(tracking, {source: {'detected': True, 'confirmation_confirmed': False}})
    assert not limiter.filter(search, lost).valid
    limiter.record_published(tracking, visible)
    assert limiter.filter(search, lost).valid


def test_fixed_exit_and_visible_alignment_are_outside_search_budget():
    limiter = LostSearchTurnLimiter(max_turns=0)
    assert limiter.filter(turn('POST_SHOT_TURN_RIGHT_9'), {}).valid
    assert limiter.filter(turn('POST_SHOT_TURN_LEFT_4'), {}).valid
    assert limiter.filter(turn('POST_SHOT_LINE_TURN_RIGHT_5'), {'line': line_info()}).valid
    assert not limiter.filter(turn('POST_SHOT_LINE_TURN_RIGHT_5'), {}).valid


@pytest.mark.parametrize('heading', [-20.0, -19.999, 0.0, 19.999, 20.0])
def test_post_shot_heading_accepts_twenty_degrees_inclusive(heading):
    decision = MotionDecisionPlanner().plan('POST_SHOT_LINE_ALIGN', {
        'line': line_info(ground_heading_error_deg=heading)}, 0.1)
    assert decision.action == 'POST_SHOT_LINE_ALIGNED'
    assert decision.source_command['heading_tolerance_deg'] == 20.0


@pytest.mark.parametrize('heading,direction,count', [(-20.001, 'LEFT', 1), (20.001, 'RIGHT', 2)])
def test_post_shot_heading_outside_tolerance_still_turns(heading, direction, count):
    decision = MotionDecisionPlanner().plan('POST_SHOT_LINE_ALIGN', {
        'line': line_info(ground_heading_error_deg=heading)}, 0.1)
    assert decision.action == f'POST_SHOT_LINE_TURN_{direction}_{count}'


def test_goal_exit_composites_already_contain_nine_and_six_turn_cycles():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for direction, frame_name, expected in [('right', '제우오들25', 9), ('left', '제좌왼들25', 6)]:
        motion = motions[aliases[f'post_shot_default_turn_{direction}']]
        assert motion['repeat_count'] == 1
        assert sum(frame['name'] == frame_name for frame in motion['frames']) == expected


def test_node_charges_only_published_turns_and_blocks_next_request():
    from mission_control.motion_decision_node import MotionDecisionNode
    from test_motion_decision_node import ReadinessPublishNode

    node = ReadinessPublishNode(turn())
    node.latest_info['line'] = {'detected': False}
    node.lost_search_turn_limiter = LostSearchTurnLimiter(max_turns=1)
    node.publisher.subscription_count = 0
    MotionDecisionNode._publish_decision(node)
    assert node.lost_search_turn_limiter.counts['line'] == 0
    node.publisher.subscription_count = 1
    MotionDecisionNode._publish_decision(node)
    assert node.lost_search_turn_limiter.counts['line'] == 1
    node.general_motion_gate.on_motion_status('LINE_LOST_TURN_RIGHT', 'RUNNING', node.command_id)
    node.general_motion_gate.on_motion_status('LINE_LOST_TURN_RIGHT', 'SUCCEEDED', node.command_id)
    MotionDecisionNode._publish_decision(node)
    assert node.lost_search_turn_limiter.counts['line'] == 1
    assert json.loads(node.publisher.messages[-1].data)['reason'] == 'lost_search_turn_limit_reached'


@pytest.mark.parametrize('max_turns,max_angle', [(-1, 90.0), (3, -1.0), (3, float('inf')), (3, float('nan'))])
def test_invalid_limits_fail_at_startup(max_turns, max_angle):
    with pytest.raises(ValueError):
        LostSearchTurnLimiter(max_turns, max_angle)


def test_uncalibrated_lost_turn_is_held_with_explicit_reason():
    decision = LostSearchTurnLimiter().filter(turn('RECOVER_GOAL_TURN_RIGHT', 'goal'), {})
    assert not decision.valid
    assert decision.reason == 'lost_search_turn_angle_unavailable'


@pytest.mark.parametrize('robot', [False, True])
def test_launch_search_limits_reach_decision_node(monkeypatch, tmp_path, robot):
    from launch_ros.actions import Node
    from test_full_system_launch import node_parameters
    if robot:
        from test_full_system_robot_launch import launch_description, default_context
    else:
        from test_full_system_launch import launch_description, launch_context as default_context
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    node = next(entity for entity in description.entities
                if isinstance(entity, Node) and entity.node_executable == 'motion_decision_node')
    params = node_parameters(node, context)
    assert params['lost_search_max_turns'] == 3
    assert params['lost_search_max_angle_deg'] == 90.0


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('heading,action', [
    (-20.0, 'POST_SHOT_FORWARD'), (20.0, 'POST_SHOT_FORWARD'),
    (-20.001, 'POST_SHOT_LINE_TURN_LEFT_1'),
    (20.001, 'POST_SHOT_LINE_TURN_RIGHT_2'),
])
def test_real_exit_route_reaches_heading_gate(monkeypatch, section, heading, action):
    from test_mission_phase_flow import post_shot_line_search_ready, line_info

    harness, _clock = post_shot_line_search_ready(monkeypatch, section)
    assert harness.mission_phase == 'POST_SHOT_LINE_ALIGN'
    command = harness.publish_vision(line=line_info(heading=heading))[-1]
    assert command['action'] == action
    assert command['source_command']['heading_tolerance_deg'] == 20.0


@pytest.mark.parametrize('section,heading,search', [
    (1, -30.0, 'POST_SHOT_LINE_TURN_RIGHT_2'),
    (2, 30.0, 'POST_SHOT_LINE_TURN_LEFT_1'),
])
def test_post_shot_search_keeps_course_direction_after_opposite_correction(
    monkeypatch, section, heading, search,
):
    from test_mission_phase_flow import post_shot_line_search_ready, line_info, release_general

    harness, clock = post_shot_line_search_ready(monkeypatch, section)
    correction = harness.publish_vision(line=line_info(heading=heading))[-1]
    release_general(harness, correction)
    clock[0] = harness.post_shot_dwell_until
    assert harness.publish_vision(line={'detected': False}) == []
    assert harness.publish_vision(line={'detected': False})[-1]['action'] == search
