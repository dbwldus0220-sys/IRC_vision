"""Validate complete source60 import, derived routes, and once-only runtime speed."""

import copy
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261029_catalog60_full'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
DERIVED = {**MANIFEST['derived_motions'], **{
    f'R제자리우회전{camera}(1회)': {
        'source': f'R제자리우회전{camera}(2회)', 'repeat_count': 1,
    } for camera in ('', '0도', '90도')
}}


def catalogs():
    raw = (UPDATE / 'source60.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST['source_sha256']
    return [{m['name']: m for m in payload['motions']} for payload in (
        json.loads(raw),
        json.loads((ROOT / 'artifacts/robot_motions_pc.json').read_text()),
        json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text()),
    )]


def test_every_original_and_derived_motion_uses_latest_source_data():
    source, pc, runtime = catalogs()
    added62 = json.loads((ROOT / 'artifacts/20261030_catalog62_ball500/manifest.json').read_text())['motions']
    assert len(source) == 74 and len(pc) == len(runtime) == 103 + len(added62)
    assert pc.keys() == runtime.keys() == source.keys() | DERIVED.keys() | added62.keys() | {f'R골넣기{n}번구역' for n in range(1, 5)}
    for name, original in source.items():
        assert pc[name] == original
    for name, spec in DERIVED.items():
        expected = copy.deepcopy(source[spec['source']])
        expected.update(name=name, repeat_count=spec['repeat_count'])
        if 'head_angle_deg' in spec:
            names = {f['name']: f['name'] + '(회전준비90도)' for f in expected['frames']}
            for frame in expected['frames']:
                frame['angles']['0'] = spec['head_angle_deg']
                frame['name'] = names[frame['name']]
            for field in ('start_pose', 'end_pose'):
                expected[field] = names.get(expected[field], expected[field])
        assert pc[name] == expected
    for name, motion in runtime.items():
        expected = copy.deepcopy(pc[name])
        expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
        expected['completion']['position_tolerance_deg'] = 5.0
        if name in {'r공잡기 400', 'r공잡기500'}:
            renames = {pose: pose + '(건공잡기)' for pose in ('미세후진오뒤(0도)', '미세오뒤4111(45도)')}
            renames['건허리돌리고숙임'] = '건허리돌리고숙임(r공잡기)'
            for frame in expected['frames']:
                frame['name'] = renames.get(frame['name'], frame['name'])
            for field in ('start_pose', 'end_pose'):
                expected[field] = renames.get(expected[field], expected[field])
        assert motion == expected


def test_routes_use_new_crab_shot_and_fine_gaits_and_startup_stays_unambiguous():
    source, pc, runtime = catalogs()
    catalog62 = json.loads((ROOT / 'artifacts/20261030_catalog62_ball500/source62.json').read_text())
    source62 = {m['name']: m for m in catalog62['motions']}
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    assert set(aliases.values()) <= runtime.keys()
    assert aliases['goal_shot'] == 'R골넣기1번구역'
    expected = {'goal_camera_90_crab_right': '유미세오옆꽃게90도',
                'pickup_crab_right_0': '유미세오옆꽃게0도', 'pickup_fine_forward_0': 'r미세0도 공잡기용'}
    for alias, name in expected.items():
        assert aliases[alias] == name
        original = source62[name] if alias == 'pickup_fine_forward_0' else source[name]
        assert runtime[name]['frames'] == original['frames']
    for n in range(1, 5):
        assert aliases[f'goal_camera_90_fine_forward_{n}'] == f'r미세 골대전용({n}회)'
    matches = [f['angles'] for m in runtime.values() for f in m['frames'] if f['name'] == '오뒤무게중심앞']
    assert matches and all(pose == matches[0] for pose in matches)
    assert matches[0] == source['R오뒤45도']['frames'][0]['angles']


@pytest.mark.parametrize('table', ['LINE_MOTION_CAPTURE_CONFIG', 'APPROACH_WALK_DURATION_SEC', 'GOAL_FORWARD_DURATION_SEC'])
def test_prequeue_and_capture_estimates_match_scaled_runtime(table):
    _, _, runtime = catalogs()
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    for action, value in getattr(MotionDecisionNode, table).items():
        duration = value[0] if isinstance(value, tuple) else value
        motion_id = MotionCommandBridgeNode.ACTION_TO_MOTION_ID.get(action)
        if action == 'STRAIGHT':
            motion_id = 'line_forward_4' if table == 'APPROACH_WALK_DURATION_SEC' else 'line_forward_6'
        if motion_id is None:
            # Recovery suffixes share the four-repeat aliases.
            motion_id = 'line_recovery_left_4' if 'TURN_LEFT_' in action else 'line_recovery_right_4'
        motion = runtime[aliases[motion_id]]
        expected = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']
        assert duration == pytest.approx(expected), action
