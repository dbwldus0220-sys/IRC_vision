"""Verify source62 fidelity, mission isolation and real runtime gait durations."""

import copy
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_ball_entry_line_alignment import ground_ball_info, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261030_catalog62_ball500'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    assert len(motions) == len({m['name'] for m in motions})
    return {m['name']: m for m in motions}


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_requested_gaits_added_and_all_source_fields_preserved(filename):
    raw = (UPDATE / 'source62.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(raw)['motions']}
    actual = catalog(ROOT / 'artifacts' / filename)
    before = catalog(UPDATE / 'before/artifacts' / filename)
    # The later Goal turn update adds one source62 support transition.
    assert actual.keys() == before.keys() | MANIFEST['motions'].keys() | {'r미세오뒤에서 오뒤90도'}
    for name, original in before.items():
        assert actual[name] == original, name
    for name, spec in MANIFEST['motions'].items():
        expected = copy.deepcopy(source[spec['source']])
        expected.update(name=name, repeat_count=spec['repeat_count'])
        if 'runtime' in filename:
            expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
            expected['completion']['position_tolerance_deg'] = 5.0
        # Includes every joint target (also IDs 4/5), torque and frame timestamp.
        assert actual[name] == expected, name


def test_aliases_change_only_requested_fine_gaits():
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    assert aliases == {**before, **MANIFEST['aliases'],
                       'fine_to_turn_ready_90': 'r미세오뒤에서 오뒤90도'}
    runtime = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(aliases.values()) <= runtime.keys()
    expected = {
        'ball_approach_fine_forward_45_5': ('r미세45도', 5, .9975, -34.),
        'pickup_fine_forward_0': ('r미세0도 공잡기용', 5, .9975, -64.),
        'hurdle_fine_forward_5': ('r미세 0도 허들용', 5, .9975, -64.),
        'hurdle_fine_forward_10': ('r미세 0도 허들용(10회)', 10, .9975, -64.),
        **{f'goal_camera_90_fine_forward_{n}': (f'r미세 골대전용({n}회)', n, .95, 1.)
           for n in range(1, 5)},
    }
    for alias, (name, count, speed, head) in expected.items():
        assert aliases[alias] == name
        motion = runtime[name]
        assert motion['repeat_count'] == count
        assert motion['playback_speed'] == speed
        assert all(f['angles']['0'] == head for f in motion['frames'])


@pytest.mark.parametrize('action', ['BALL_APPROACH_FINE_FORWARD_5', *[
    f'GOAL_CAMERA90_FINE_FORWARD_{n}' for n in range(1, 5)
]])
def test_capture_duration_matches_scaled_runtime(action):
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    motion = catalog(ROOT / 'artifacts/robot_motions_runtime.json')[aliases[Bridge.ACTION_TO_MOTION_ID[action]]]
    cycle_ms = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    duration = cycle_ms / 1000 * motion['repeat_count'] / motion['playback_speed']
    table = (MotionDecisionNode.APPROACH_WALK_DURATION_SEC if action.startswith('BALL')
             else MotionDecisionNode.GOAL_FORWARD_DURATION_SEC)
    assert table[action] == pytest.approx(duration)


@pytest.mark.parametrize('distance', [.600001, .8, .800001, 1.5])
@pytest.mark.parametrize('heading', [-50., -30., 0., 15., 30., 50.])
def test_far_ball_line_and_ground_policy_is_unchanged(distance, heading):
    planner = MotionDecisionPlanner()
    ball = ground_ball_info(depth_m=distance)
    ball['ground_steering_angle_deg'] = heading
    # Deliberately disagree: the boundary must switch the selected reference.
    line = line_info(heading=-heading)
    command = BallEntryLineAlignment().select(planner, line, .1, ball).source_command
    angle = heading if distance <= .8 else -heading
    assert command['heading_error_deg'] == angle
    if abs(angle) <= 15:
        assert command['motion'] == 'STRAIGHT_2'
    elif abs(angle) < 45:
        assert command['motion'].startswith('RECOVER_')
    else:
        assert command['motion'].startswith('LINE_HEADING_TURN_')


@pytest.mark.parametrize('source,expected', [
    ('ball', 'ball_general_fine_forward_8'), ('hurdle', 'hurdle_fine_forward_5'),
])
def test_generic_fine_action_routes_to_its_own_mission(source, expected):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(source=source, action='STRAIGHT_0'))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == expected


def test_hurdle_new_fine_id_retains_posture_transition_handling():
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'line_forward_4'
    assert bridge._transition_prepare_motion('hurdle_fine_forward_5') == 'pickup_fine_prepare'
    bridge.last_physical_motion_id = 'hurdle_fine_forward_5'
    assert bridge._transition_prepare_motion('line_turn_right_1') == 'fine_to_turn_ready_0'
    assert Bridge.HURDLE_FINAL_FINE_MOTION_ID == 'hurdle_fine_forward_10'
