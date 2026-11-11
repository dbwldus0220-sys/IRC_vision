"""Verify source83 left turns, real catalog timings and reservation handoffs."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog83_left_turns'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_twenty_motions_change_and_keep_source_speed(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / relative)
    after = catalog(ROOT / 'artifacts/20261110_catalog84_crab_thresholds/before' / relative)
    assert before.keys() == after.keys()
    assert {n for n in after if after[n] != before[n]} == set(MANIFEST['imports'])
    for name in MANIFEST['imports']:
        expected = copy.deepcopy(SOURCE[name])
        for frame in expected['frames']:
            frame['name'] = MANIFEST['pose_renames'].get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = MANIFEST['pose_renames'].get(expected[key], expected[key])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
        assert after[name]['playback_speed'] == 1.05


def test_source_snapshot_aliases_and_pose_lookup_are_preserved():
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == MANIFEST['source_sha256']
    snapshot = ROOT / 'artifacts/20261110_catalog84_crab_thresholds/before'
    assert (snapshot / ALIAS_PATH).read_bytes() == (UPDATE / 'before' / ALIAS_PATH).read_bytes()
    assert set(ALIASES.values()) <= RUNTIME.keys()
    poses = {f['name'] for n in MANIFEST['imports'] for f in RUNTIME[n]['frames']}
    for name in poses:
        frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == name]
        assert all((f['angles'], f['torques']) ==
                   (frames[0]['angles'], frames[0]['torques']) for f in frames), name


@pytest.mark.parametrize('name', MANIFEST['imports'])
def test_changed_trajectory_fits_timeout_and_keeps_end_support(name):
    motion = RUNTIME[name]
    before = catalog(UPDATE / 'before/artifacts/robot_motions_runtime.json')[name]
    actual_ms = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    actual_ms *= motion['repeat_count'] / motion['playback_speed']
    assert actual_ms == pytest.approx(MANIFEST['durations_ms'][name]['after'])
    assert actual_ms + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    assert motion['completion']['position_tolerance_deg'] == 5.0
    assert motion['frames'][-1]['angles'] == before['frames'][-1]['angles']
    assert motion['frames'][-1]['torques'] == before['frames'][-1]['torques']
    assert any(f['name'] == motion['end_pose'] for f in motion['frames'])
    for frame in motion['frames']:
        assert set(frame['angles']) == set(frame['torques']) == {str(i) for i in range(23)}


@pytest.mark.parametrize('count', range(1, 7))
def test_actual_speed_drives_all_camera45_reservation_windows(count):
    alias = f'ball_align_turn_left_{count}'
    expected = MANIFEST['durations_ms'][ALIASES[alias]]['after'] / 1000
    assert duration_sec(alias) == pytest.approx(expected)
    assert Node.LINE_TURN_CAPTURE_CONFIG[f'LINE_OFFSET_TURN_LEFT_{count}'][0] == pytest.approx(expected)
    for action, config in Node.LINE_TURN_CAPTURE_CONFIG.items():
        if action.endswith(f'TURN_LEFT_{count}'):
            assert config[0] == pytest.approx(expected)
    assert Node.POST_SHOT_ALIGN_CAPTURE_CONFIG[f'POST_SHOT_LINE_TURN_LEFT_{count}'][0] == pytest.approx(expected)


@pytest.mark.parametrize('count', range(1, 7))
def test_every_left_turn_reobserves_and_releases_its_lock(clock, count):
    from test_line_turn_prequeue import test_turn_waits_for_completion_and_reobserves
    from test_mission_phase_flow import line_info

    test_turn_waits_for_completion_and_reobserves(
        clock, 'LINE_TRACK', f'LINE_OFFSET_TURN_LEFT_{count}', line_info(), 'STRAIGHT')


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('angle', [-15., -30., -45., -60., -75.])
def test_faster_ball_turns_keep_prepare_and_fine_handoffs(clock, index, angle):
    from test_ball_fine_alignment_queue import test_forward_turn_prepare_fine_prepare_turn_keeps_correction_dwells

    test_forward_turn_prepare_fine_prepare_turn_keeps_correction_dwells(clock, index, angle, True)
