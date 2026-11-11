"""Verify the requested shot data and its runtime handoff constraints."""

import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog90_goal_shots'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
@pytest.mark.parametrize('update,after_root', [
    # Source91 supersedes two shots; retain the source90 import check on its snapshot.
    (UPDATE, ROOT / 'artifacts/20261110_catalog91_goal_shots/before'),
    (ROOT / 'artifacts/20261110_catalog91_goal_shots', ROOT),
])
def test_only_requested_shots_use_selected_source(kind, update, after_root):
    manifest = json.loads((update / 'manifest.json').read_text())
    source = catalog(update / 'source_robot_motions.json')
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(update / 'before' / relative)
    after = catalog(after_root / relative)
    assert before.keys() == after.keys()
    for name, motion in after.items():
        if name not in manifest['imports']:
            assert motion == before[name]
            continue
        expected = copy.deepcopy(source[name])
        renames = manifest['pose_renames'].get(name, {})
        for frame in expected['frames']:
            frame['name'] = renames.get(frame['name'], frame['name'])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
            assert runtime_motion_from_source(expected) == expected
        assert motion == expected


@pytest.mark.parametrize('region', [1, 2, 8, 5, 6, 7])
def test_shot_alias_timing_and_waist_frames(region):
    name = ALIASES[f'goal_shot_region_{region}']
    assert name in MANIFEST['imports']
    motion = RUNTIME[name]
    previous = catalog(UPDATE / 'before/artifacts/robot_motions_runtime.json')[name]
    duration = lambda m: max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed']
    assert duration(motion) == duration(previous)
    assert duration(motion) + motion['completion']['settle_timeout_ms'] < MotionCommandBridgeNode.DEFAULT_TIMEOUT_MS
    assert motion['start_pose'] == previous['start_pose']
    assert motion['end_pose'] == previous['end_pose']
    assert motion['repeat_count'] == 1
    assert motion['playback_speed'] == 1.0
    assert motion['completion']['position_tolerance_deg'] == 5.0
    assert all(f['angles']['11'] == .47265625 and f['torques']['11'] for f in motion['frames'][:-1])
    assert motion['frames'][-1]['angles']['11'] == -.615234375


def test_requested_pose_names_are_unambiguous_and_aliases_unchanged():
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    assert (ROOT / relative).read_bytes() == (UPDATE / 'before' / relative).read_bytes()
    poses = {f['name'] for n in MANIFEST['imports'] for f in RUNTIME[n]['frames']}
    for pose in poses:
        frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == pose]
        assert all((f['angles'], f['torques']) == (frames[0]['angles'], frames[0]['torques']) for f in frames), pose
