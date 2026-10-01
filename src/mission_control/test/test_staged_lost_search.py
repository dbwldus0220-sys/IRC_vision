"""Exercise complete loss searches and their actual camera-specific motion routes."""

from dataclasses import replace
import json
from pathlib import Path

import pytest
import yaml

from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_command_gate import normalize_general_action
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecision


FAMILIES = [
    ('line', 'LINE_LOST_TURN_', ''),
    ('line', 'POST_BALL_LINE_TURN_', 'post_ball_line_search'),
    ('line', 'POST_SHOT_LINE_TURN_', 'post_shot_line_search'),
    ('ball', 'BALL_APPROACH_TURN_', 'turn_toward_last_seen_ball_side'),
    ('ball', 'BALL_PICKUP_CAMERA_DOWN_TURN_', 'turn_toward_last_seen_ball_side'),
    ('ball', 'BALL_PICKUP_FINE_SEARCH_', 'turn_toward_last_seen_ball_side'),
    ('goal', 'GOAL_CAMERA90_TURN_', 'turn_toward_last_seen_goal_side'),
]


def limiter(**kwargs):
    return LostSearchTurnLimiter(staged_sources=('line', 'ball', 'goal'), **kwargs)


def candidate(source, prefix, reason, side='RIGHT'):
    return MotionDecision('AUTO', source, f'{prefix}{side}_2', True, reason, True, False, {})


def sequence(side):
    opposite = 'LEFT' if side == 'RIGHT' else 'RIGHT'
    return ([(side, 3 if side == 'LEFT' else 5, 'INITIAL')]
            + [(side, 1 if side == 'LEFT' else 2, 'EXTEND')] * 3
            + [(opposite, 1 if opposite == 'LEFT' else 2, 'REVERSE')] * 6)


@pytest.mark.parametrize('source,prefix,reason', FAMILIES)
@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
def test_complete_sequence_has_90_degrees_each_way_and_stops(source, prefix, reason, side):
    policy = limiter()
    lost = {source: {'detected': False, 'raw_detected': False}}
    request = candidate(source, prefix, reason, side)
    directions = []
    for index, (direction, count, stage) in enumerate(sequence(side)):
        result = policy.filter(request, lost)
        assert result.valid and result.action == f'{prefix}{direction}_{count}'
        assert result.source_command['lost_search_stage'] == stage
        assert result.source_command['turn_count'] == count
        assert result.source_command['turn_angle_deg'] == (45. if index == 0 else 15.)
        # Repeated planning and changes to a provisional remembered side spend nothing.
        for _ in range(5):
            assert policy.filter(request, lost) == result
        directions.append(result.source_command['turn_angle_deg'] * (1 if direction == side else -1))
        policy.record_published(result, lost)
        request = candidate(source, prefix, reason, 'LEFT' if side == 'RIGHT' else 'RIGHT')
    assert sum(directions[:4]) == 90.
    assert sum(directions[4:]) == -90.
    assert sum(directions) == 0.
    assert policy.counts[source] == 10 and policy.angles[source] == 180.
    blocked = policy.filter(request, lost)
    assert not blocked.valid and blocked.reason == 'lost_search_turn_limit_reached'
    assert blocked.source_command['lost_search_stage'] == 'EXHAUSTED'


@pytest.mark.parametrize('info,reason', [
    (None, 'lost_search_waiting_for_fresh_vision'),
    ({'detected': False, 'raw_detected': True}, 'lost_search_waiting_for_confirmation'),
])
def test_missing_input_or_raw_candidate_does_not_spend_or_rearm(info, reason):
    policy = limiter()
    search = candidate('line', 'LINE_LOST_TURN_', '')
    lost = {'line': {'detected': False}}
    first = policy.filter(search, lost)
    policy.record_published(first, lost)
    for _ in range(20):
        held = policy.filter(search, {'line': info})
        assert not held.valid and held.reason == reason
        policy.record_published(held, {'line': info})
    assert policy.counts['line'] == 1
    assert policy.filter(search, lost).action == 'LINE_LOST_TURN_RIGHT_2'


def test_only_published_confirmed_tracking_rearms_and_other_targets_are_independent():
    policy = limiter()
    search = candidate('goal', 'GOAL_CAMERA90_TURN_', 'turn_toward_last_seen_goal_side')
    lost = {'goal': {'detected': False}}
    for _ in range(10):
        policy.record_published(policy.filter(search, lost), lost)
    tracking = replace(search, action='GOAL_CAMERA_90_FORWARD_2', reason='approach')
    visible = {'goal': {'detected': True, 'confirmation_confirmed': True}}
    policy.filter(tracking, visible)
    policy.record_published(replace(tracking, valid=False), visible)
    policy.record_published(tracking, {'goal': {'detected': True, 'confirmation_confirmed': False}})
    assert not policy.filter(search, lost).valid
    assert policy.counts['line'] == policy.counts['ball'] == 0
    policy.record_published(tracking, visible)
    assert policy.filter(search, lost).source_command['lost_search_stage'] == 'INITIAL'


@pytest.mark.parametrize('source,prefix,reason', FAMILIES)
def test_failed_search_cannot_continue_with_a_new_action(source, prefix, reason):
    policy = limiter()
    lost = {source: {'detected': False}}
    request = candidate(source, prefix, reason)
    first = policy.filter(request, lost)
    policy.record_published(first, lost)
    policy.record_failure(source, first.action)
    blocked = policy.filter(request, lost)
    assert not blocked.valid and blocked.reason == 'lost_search_motion_failed'
    assert policy.counts[source] == 1


def test_hurdle_keeps_existing_limit_and_fixed_exit_is_not_search():
    policy = limiter()
    lost = {'hurdle': {'detected': False}}
    request = MotionDecision('HURDLE_APPROACH', 'hurdle', 'ALIGN_LEFT', True,
                             'search', True, False, {'turn_angle_deg': 30.})
    for _ in range(3):
        result = policy.filter(request, lost)
        assert result.action == 'ALIGN_LEFT' and 'lost_search_stage' not in result.source_command
        policy.record_published(result, lost)
    assert not policy.filter(request, lost).valid
    exit_turn = replace(request, source='line', action='POST_SHOT_TURN_RIGHT_9')
    assert policy.filter(exit_turn, {}) == exit_turn


@pytest.mark.parametrize('source,prefix,reason', FAMILIES)
@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
def test_each_stage_resolves_to_existing_motion_with_correct_camera_and_repeat_count(source, prefix, reason, side):
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    names = {m['name'] for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    policy = limiter()
    lost = {source: {'detected': False}}
    request = candidate(source, prefix, reason, side)
    for direction, count, _ in sequence(side):
        result = policy.filter(request, lost)
        if prefix == 'BALL_PICKUP_FINE_SEARCH_':
            assert result.action in MotionDecisionNode.PICKUP_FINE_ALIGN_ACTIONS
            motion = MotionCommandBridgeNode.PICKUP_FINE_ALIGN_MOTION_IDS[result.action]
        elif prefix == 'BALL_PICKUP_CAMERA_DOWN_TURN_':
            assert result.action in MotionDecisionNode.PICKUP_INITIAL_ALIGN_ACTIONS
            motion = MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS[result.action]
        else:
            assert normalize_general_action(result.action) == result.action
            motion = MotionCommandBridgeNode.ACTION_TO_MOTION_ID[result.action]
        camera = 90 if source == 'goal' else 0 if prefix.startswith('BALL_PICKUP_') else 45
        label = aliases[motion]
        assert f'{camera}도' in label and f'({count}회)' in label
        assert ('좌회전' if direction == 'LEFT' else '우회전') in label
        assert label in names
        policy.record_published(result, lost)


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_pickup_runs_entire_search_through_checkpoint_and_dwell(side, stage):
    from test_motion_command_bridge import (
        FakeBridge, decoded_messages, enter_pickup_fine_alignment,
        complete_active_motion, executor_status, fine_alignment_message, navigation_message,
    )
    bridge = FakeBridge()
    if stage == 'FINE':
        enter_pickup_fine_alignment(bridge)
    else:
        bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    policy = limiter()
    prefix = 'BALL_PICKUP_FINE_SEARCH_' if stage == 'FINE' else 'BALL_PICKUP_CAMERA_DOWN_TURN_'
    request = candidate('ball', prefix, 'turn_toward_last_seen_ball_side', side)
    lost = {'ball': {'detected': False}}
    for index, (direction, count, _) in enumerate(sequence(side)):
        decision = policy.filter(request, lost)
        bridge.navigation_command_callback(fine_alignment_message(decision.action, 8100 + index))
        policy.record_published(decision, lost)
        if bridge.pending_turn_request is not None:
            bridge.executor_status_callback(executor_status(
                status='SUCCEEDED', command_id=bridge.active_command_id,
                event_id=bridge.active_event_id, request_id=bridge.active_request_id,
                motion_id=bridge.turn_prepare_motion_id))
        motion = f'pickup_camera_down_turn_{direction.lower()}_{count}'
        assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == motion
        complete_active_motion(bridge)
        deadline = bridge.active_dwell_until
        assert deadline is not None
        bridge._check_atomic_dwell(deadline - .001)
        assert not bridge.pickup_initial_align_waiting and not bridge.pickup_fine_align_waiting
        bridge._check_atomic_dwell(deadline)
        assert bridge.pickup_initial_align_waiting == (stage == 'INITIAL')
        assert bridge.pickup_fine_align_waiting == (stage == 'FINE')
        assert not bridge.pickup_fixed_sequence_started
    assert not policy.filter(request, lost).valid


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
def test_post_pickup_missing_input_exception_obeys_all_stages_and_stops(monkeypatch, side):
    from test_mission_phase_flow import MissionFlowHarness
    clock = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    node = MissionFlowHarness(phase='POST_BALL_LINE_ALIGN')
    node.planner.post_ball_line_search_direction = side
    node.general_motion_gate.required_vision_generation = node.general_motion_gate.vision_generation + 1
    for direction, count, stage in sequence(side):
        before = len(node.publisher.messages)
        MotionDecisionNode._publish_decision(node)
        request = node.publisher.messages[-1]
        assert len(node.publisher.messages) == before + 1
        assert request['valid'] and request['action'] == f'POST_BALL_LINE_TURN_{direction}_{count}'
        assert request['source_command']['lost_search_stage'] == stage
        node.send_status(request['action'], request['command_id'], 'RUNNING')
        node.send_status(request['action'], request['command_id'], 'SUCCEEDED')
        clock[0] += 1.0
        MotionDecisionNode._publish_decision(node)
    MotionDecisionNode._publish_decision(node)
    assert node.last_selected_decision.reason == 'lost_search_turn_limit_reached'
    assert node.lost_search_turn_limiter.counts['line'] == 10
