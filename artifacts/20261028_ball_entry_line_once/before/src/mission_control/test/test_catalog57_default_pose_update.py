"""Preserve the supplied default-posture transitions and all unrelated motion data."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_catalog57_default_pose'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_default_transitions_are_replaced_with_source57(filename):
    source_path = ROOT / MANIFEST['source_file']
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']:m for m in json.loads(source_path.read_text())['motions']}
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    actual = json.loads((ROOT / 'artifacts' / filename).read_text())
    expected = copy.deepcopy(before)
    for index, old in enumerate(before['motions']):
        name = MANIFEST['renames'].get(old['name'])
        if name is None:
            continue
        motion = copy.deepcopy(source[name])
        if 'runtime' in filename:
            motion['completion']['position_tolerance_deg'] = 5.
        expected['motions'][index] = motion
        assert motion['playback_speed'] == .95
        assert motion['repeat_count'] == 1
        assert [(f['start_ms'], f['time_ms']) for f in motion['frames']] == [(65, 60), (125, 110)]
        assert motion['start_pose'] == motion['frames'][0]['name']
        assert motion['end_pose'] == motion['frames'][-1]['name']
    assert actual == expected


def test_stable_aliases_resolve_new_default_postures():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    actual = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert actual == {k: MANIFEST['renames'].get(v, v) for k, v in before.items()}
    catalog = {m['name']:m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    assert set(actual.values()) <= catalog.keys()
    assert not catalog.keys() & MANIFEST['renames'].keys()
    assert catalog[actual['pickup_forward_to_crab_0']]['end_pose'] == 'R기본자세0도'
    assert catalog[actual['goal_forward_to_crab_right_90']]['end_pose'] == 'R기본자세45도'
    assert actual['goal_forward_to_default'] == actual['goal_forward_to_crab_right_90']
