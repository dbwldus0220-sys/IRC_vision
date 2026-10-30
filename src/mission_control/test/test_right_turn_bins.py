"""Check right-turn boundaries through planning, gating, aliases and R motion data."""

import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_command_gate import GENERAL_ACTIONS
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.sparse_line_recovery import SparseLineRecovery
from test_motion_decision_planner import ball_info, goal_info, line_info, observations

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_right_turn_bins'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
COUNTS = (1, 2, 3, 5, 6, 7, 9)
BOUNDARIES = [
    (0.0, 0), (14.999, 0), (15.0, 1), (29.999, 1),
    (30.0, 3), (44.999, 3), (45.0, 5), (59.999, 5),
    (60.0, 6), (74.999, 6), (75.0, 7), (89.999, 7),
    (90.0, 9), (180.0, 9),
]

LEFT_BOUNDARIES = [
    (0.0, 0), (7.999, 0), (8.0, 1), (29.999, 1),
    (30.0, 2), (44.999, 2), (45.0, 3), (59.999, 3),
    (60.0, 4), (74.999, 4), (75.0, 5), (89.999, 5),
    (90.0, 6), (180.0, 6),
]


@pytest.mark.parametrize('direction,sign,boundaries', [
    ('LEFT', -1, LEFT_BOUNDARIES), ('RIGHT', 1, BOUNDARIES),
])
@pytest.mark.parametrize('depth', [.43, 1.2])
def test_goal_close_and_far_alignment_share_all_boundaries(direction, sign, boundaries, depth):
    for angle, count in boundaries:
        decision = MotionDecisionPlanner().plan('GOAL_APPROACH', observations(goal=goal_info(
            depth_m=depth, offset_x_px=sign * 384, offset_x_norm=sign * .6,
            bearing_deg=sign * angle, score_now=False,
        )), .1)
        if count:
            assert decision.action == f'GOAL_CAMERA90_TURN_{direction}_{count}'
            assert decision.action in GENERAL_ACTIONS
            assert MotionCommandBridgeNode.motion_id_for_action(decision.action) == (
                f'goal_camera_90_turn_{direction.lower()}_{count}')
        else:
            assert 'TURN_' not in decision.action


@pytest.mark.parametrize('angle,count', LEFT_BOUNDARIES)
def test_left_boundaries_reach_pickup_ball_hurdle_sparse_and_post_shot(monkeypatch, angle, count):
    planner = MotionDecisionPlanner()
    info = ball_info(steering_angle_deg=-angle, bearing_deg=-angle,
                     depth_m=.8, distance_m=.8, bottom_distance_px=500,
                     offset_x_px=0, offset_x_norm=.06)
    pickup = {**info, 'depth_m': .5, 'distance_m': .5, 'ground_distance_m': .5}
    decisions = [
        (planner.plan('BALL_APPROACH', observations(ball=info), .1), 'BALL_APPROACH'),
        (planner.plan_ball_pickup_initial_alignment(pickup), 'BALL_PICKUP_CAMERA_DOWN'),
        (planner.plan_ball_pickup_fine_alignment(pickup), 'BALL_PICKUP_FINE'),
        (planner.plan('POST_SHOT_LINE_ALIGN', observations(line=line_info(
            ground_heading_error_deg=-angle)), .1), 'POST_SHOT_LINE'),
    ]
    for decision, prefix in decisions:
        if count:
            assert decision.action == f'{prefix}_TURN_LEFT_{count}'
        else:
            assert 'TURN_LEFT' not in decision.action
    command = replace(planner.hurdle_planner.wait('alignment'), valid=True,
                      action='ALIGN_LEFT', hurdle_angle_deg=angle)
    monkeypatch.setattr(planner.hurdle_planner, 'plan', lambda *a, **k: command)
    result = planner._plan_source('hurdle', {}, .1)
    assert result.get('turn_count', 0) == count
    policy = SparseLineRecovery()
    policy.samples.append((-angle,))
    result = policy._turn(decisions[-1][0], 'LEFT', 'confirmed_heading')
    if count:
        assert result.action == f'LINE_SPARSE_TURN_LEFT_{count}'
        assert result.action in GENERAL_ACTIONS
        assert MotionCommandBridgeNode.motion_id_for_action(result.action) == f'post_ball_line_turn_left_{count}'
    else:
        assert not result.valid


@pytest.mark.parametrize('camera', ['', '0도', '90도'])
def test_one_repeat_variants_preserve_source60_gait_and_runtime_policy(camera):
    source = json.loads((ROOT / 'artifacts/20261029_catalog60_full/source60.json').read_text())
    original = next(m for m in source['motions'] if m['name'] == f'R제자리우회전{camera}(2회)')
    expected = copy.deepcopy(original)
    expected.update(name=f'R제자리우회전{camera}(1회)', repeat_count=1)
    for suffix in ('pc', 'runtime'):
        catalog = json.loads((ROOT / f'artifacts/robot_motions_{suffix}.json').read_text())
        actual = next(m for m in catalog['motions'] if m['name'] == expected['name'])
        if suffix == 'runtime':
            expected['playback_speed'] = .855
            expected['completion']['position_tolerance_deg'] = 5.0
        assert actual == expected


@pytest.mark.parametrize('angle,count', BOUNDARIES)
def test_right_boundaries_reach_ball_pickup_goal_and_line_routes(angle, count):
    planner = MotionDecisionPlanner()
    for selector in (planner._turn_repeat_count, planner._ball_turn_repeat_count,
                     planner._goal_turn_repeat_count):
        assert selector(angle, 'RIGHT') == count
    ball = ball_info(steering_angle_deg=angle, bearing_deg=angle,
                     depth_m=.8, distance_m=.8, bottom_distance_px=500,
                     offset_x_px=0, offset_x_norm=.06)
    pickup = {**ball, 'depth_m': .5, 'distance_m': .5, 'ground_distance_m': .5}
    decisions = [
        (planner.plan('BALL_APPROACH', observations(ball=ball), .1), 'BALL_APPROACH'),
        (planner.plan_ball_pickup_initial_alignment(pickup), 'BALL_PICKUP_CAMERA_DOWN'),
        (planner.plan_ball_pickup_fine_alignment(pickup), 'BALL_PICKUP_FINE'),
        (MotionDecisionPlanner().plan('GOAL_APPROACH', observations(goal=goal_info(
            depth_m=1.2, bearing_deg=angle)), .1), 'GOAL_CAMERA90'),
        (MotionDecisionPlanner().plan('POST_SHOT_LINE_ALIGN', observations(line=line_info(
            ground_heading_error_deg=angle)), .1), 'POST_SHOT_LINE'),
    ]
    for decision, prefix in decisions:
        if count == 0:
            assert 'TURN_RIGHT' not in decision.action
        else:
            assert decision.action == f'{prefix}_TURN_RIGHT_{count}'
            if prefix.startswith('BALL_PICKUP'):
                assert decision.action in (MotionDecisionNode.PICKUP_INITIAL_ALIGN_ACTIONS
                                           | MotionDecisionNode.PICKUP_FINE_ALIGN_ACTIONS)
            else:
                assert decision.action in GENERAL_ACTIONS


@pytest.mark.parametrize('angle,count', BOUNDARIES)
def test_hurdle_stationary_alignment_uses_same_bins(monkeypatch, angle, count):
    planner = MotionDecisionPlanner()
    command = replace(planner.hurdle_planner.wait('test_alignment'),
                      valid=True, action='ALIGN_RIGHT', center_steering_deg=angle)
    monkeypatch.setattr(planner.hurdle_planner, 'plan', lambda *a, **k: command)
    result = planner._plan_source('hurdle', {}, .1)
    if count == 0:
        assert not result['valid'] and result['action'] == 'WAIT'
    else:
        assert result['turn_count'] == count
        assert MotionCommandBridgeNode.motion_id_for_action(
            f'POST_BALL_LINE_TURN_RIGHT_{count}') == f'post_ball_line_turn_right_{count}'


@pytest.mark.parametrize('angle,count', BOUNDARIES)
def test_sparse_confirmed_line_does_not_clamp_right_turn_to_two(angle, count):
    planner = MotionDecisionPlanner()
    decision = planner.plan('POST_SHOT_LINE_ALIGN', observations(line=line_info(
        ground_heading_error_deg=angle)), .1)
    policy = SparseLineRecovery()
    policy.samples.append((angle,))
    result = policy._turn(decision, 'RIGHT', 'test_confirmed_heading')
    if count == 0:
        assert not result.valid
    else:
        assert result.action == f'LINE_SPARSE_TURN_RIGHT_{count}'
        assert result.action in GENERAL_ACTIONS
        assert MotionCommandBridgeNode.motion_id_for_action(result.action) == f'post_ball_line_turn_right_{count}'


@pytest.mark.parametrize('count', COUNTS)
def test_every_right_turn_route_resolves_to_matching_r_repeat_count(count):
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads(
        (ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    routes = [
        (f'{prefix}_TURN_RIGHT_{count}', MotionCommandBridgeNode.ACTION_TO_MOTION_ID)
        for prefix in ('LINE_HEADING', 'LINE_OFFSET', 'LINE_SPARSE', 'BALL_APPROACH',
                       'POST_BALL_LINE', 'POST_SHOT_LINE', 'GOAL_CAMERA90')
    ] + [
        (f'BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_{count}', MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS),
        (f'BALL_PICKUP_FINE_TURN_RIGHT_{count}', MotionCommandBridgeNode.PICKUP_FINE_ALIGN_MOTION_IDS),
    ]
    for action, mapping in routes:
        motion = catalog[aliases[mapping[action]]]
        assert motion['name'].startswith('R제자리우회전')
        assert motion['repeat_count'] == count
    assert f'BALL_PICKUP_POST_BACKWARD_TURN_RIGHT_{count}' in MotionCommandBridgeNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    assert f'BALL_PICKUP_POST_BACKWARD_TURN_RIGHT_{count}' in MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_three_six_repeat_r_variants_added_and_source_data_preserved(filename):
    source_path = ROOT / MANIFEST['source_file']
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(source_path.read_text())['motions']}
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    # Verify this import before the later source59 fine-gait replacement.
    actual = json.loads((ROOT / 'artifacts/20261029_ball_search_catalog59/before/artifacts' / filename).read_text())
    expected = copy.deepcopy(before)
    for name, base in MANIFEST['derived'].items():
        motion = copy.deepcopy(source[base])
        motion.update(name=name, repeat_count=6)
        if 'runtime' in filename:
            motion['completion']['position_tolerance_deg'] = 5.0
        expected['motions'].append(motion)
    assert actual == expected
    # Every selected R variant shares its source-camera gait; only repeats differ.
    for motion in actual['motions']:
        if not motion['name'].startswith('R제자리우회전'):
            continue
        camera = motion['name'].split('(')[0].removeprefix('R제자리우회전')
        reference = copy.deepcopy(source[f'R제자리우회전{camera}(7회)'])
        reference.update(name=motion['name'], repeat_count=motion['repeat_count'])
        if 'runtime' in filename:
            reference['completion']['position_tolerance_deg'] = 5.0
        assert motion == reference


def test_only_six_repeat_aliases_added():
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    # Later source59 Goal aliases are covered by test_catalog59_fine_motion.
    snapshot = ROOT / 'artifacts/20261029_ball_search_catalog59/before'
    actual = yaml.safe_load((snapshot / ALIAS_PATH).read_text())['motion_aliases']
    additions = {
        'line_turn_right_6': 'R제자리우회전(6회)',
        'post_ball_line_turn_right_6': 'R제자리우회전(6회)',
        'pickup_camera_down_turn_right_6': 'R제자리우회전0도(6회)',
        'goal_camera_90_turn_right_6': 'R제자리우회전90도(6회)',
    }
    assert actual == {**before, **additions}
