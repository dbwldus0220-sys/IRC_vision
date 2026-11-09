"""Verify source80 fidelity, stable dispatch and actual reservation durations."""

import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_wait_refresh_and_fine_settle import LiveInputHarness

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261109_catalog80_pickup_hurdle_forward'
SNAPSHOT = ROOT / 'artifacts/20261109_goal_regions_5_8/before'
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_motions_change_and_source_data_is_preserved(kind):
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(SNAPSHOT / f'artifacts/robot_motions_{kind}.json')
    assert after.keys() == before.keys()
    for name, old in before.items():
        if name not in MANIFEST['imports']:
            assert after[name] == old, name
            continue
        source_name, count = MANIFEST['derived'].get(name, (name, SOURCE.get(name, {}).get('repeat_count')))
        expected = copy.deepcopy(SOURCE[source_name])
        expected.update(name=name, repeat_count=count)
        poses = dict(MANIFEST['pose_renames'])
        if kind == 'runtime':
            expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
            expected['completion']['position_tolerance_deg'] = 5.0
            if name in {'r공잡기 400', 'r공잡기500'}:
                poses.update({'미세후진오뒤(0도)': '미세후진오뒤(0도)(건공잡기)',
                              '미세오뒤4111(45도)': '미세오뒤4111(45도)(건공잡기)',
                              '건허리돌리고숙임': '건허리돌리고숙임(r공잡기)'})
        for frame in expected['frames']:
            frame['name'] = poses.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = poses.get(expected[key], expected[key])
        assert after[name] == expected, name


def test_names_aliases_and_bridge_dispatch_remain_valid():
    assert (SNAPSHOT / ALIAS_PATH).read_bytes() == (UPDATE / 'before' / ALIAS_PATH).read_bytes()
    assert set(ALIASES.values()) <= RUNTIME.keys()
    assert set(Bridge.ACTION_TO_MOTION_ID.values()) <= ALIASES.keys()
    assert set(MANIFEST['derived']) == {'p전진45(2회)', 'p전진45(8회)', 'p전진90도(2회)'}
    assert len(MANIFEST['imports']) == 12


@pytest.mark.parametrize('name', MANIFEST['imports'])
def test_source80_motions_fit_timeout_and_keep_full_end_support(name):
    # Source82 intentionally replaces pickup end poses; check source80's snapshot here.
    motion = catalog(SNAPSHOT / 'artifacts/robot_motions_runtime.json')[name]
    before = catalog(UPDATE / 'before/artifacts/robot_motions_runtime.json')[name]
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    assert duration * motion['repeat_count'] / motion['playback_speed'] + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    for field in ('angles', 'torques'):
        assert motion['frames'][-1][field] == before['frames'][-1][field]


def test_imported_and_startup_pose_names_are_unambiguous():
    names = {f['name'] for name in MANIFEST['imports'] for f in RUNTIME[name]['frames']}
    names.add('오뒤무게중심앞')
    for name in names:
        frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == name]
        assert frames and all((f['angles'], f['torques']) == (frames[0]['angles'], frames[0]['torques']) for f in frames), name


@pytest.mark.parametrize('phase,action,alias', [
    ('LINE_TRACK', 'STRAIGHT_1', 'line_forward_2'),
    ('LINE_TRACK', 'STRAIGHT_2', 'line_forward_4'),
    ('LINE_TRACK', 'STRAIGHT_3', 'line_forward_6'),
    ('LINE_TRACK', 'STRAIGHT_4', 'line_forward_8'),
    ('LINE_TRACK_AFTER_PICKUP', 'LINE_SPARSE_FORWARD', 'line_forward_2'),
    ('POST_SHOT_LINE_ALIGN', 'POST_SHOT_FORWARD', 'line_forward_6'),
])
def test_capture_window_uses_scaled_runtime_including_special_forward_routes(phase, action, alias):
    motion = RUNTIME[ALIASES[alias]]
    seconds = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
    node = LiveInputHarness(phase=phase)
    Node._start_line_motion_capture(node, action, alias)
    assert node.active_line_motion_duration_sec == pytest.approx(seconds)
