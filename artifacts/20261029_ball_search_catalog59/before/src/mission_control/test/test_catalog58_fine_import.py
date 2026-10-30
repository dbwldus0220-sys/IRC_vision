"""Protect source58 joint targets and the user's existing execution speeds."""

import hashlib
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
IMPORT = ROOT / 'artifacts/20261029_catalog58_fine_import'
MANIFEST = json.loads((IMPORT / 'manifest.json').read_text())


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_source58_fine_targets_and_existing_execution_settings(filename):
    raw = (IMPORT / 'source58.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(raw)['motions']}
    before = {m['name']: m for m in json.loads((IMPORT / ('before_' + filename)).read_text())['motions']}
    actual = {m['name']: m for m in json.loads((ROOT / 'artifacts' / filename).read_text())['motions']}
    assert actual.keys() == before.keys()
    assert len(MANIFEST['motions']) == 27
    for name, motion in actual.items():
        assert motion['playback_speed'] == before[name]['playback_speed']
        assert motion['repeat_count'] == before[name]['repeat_count']
        if name not in MANIFEST['motions']:
            assert motion == before[name]
            continue
        spec = MANIFEST['motions'][name]
        original = source[spec['source']]
        assert motion['max_seq_ms'] == original['max_seq_ms']
        assert len(motion['frames']) == len(original['frames'])
        for frame, supplied in zip(motion['frames'], original['frames']):
            expected_angles = dict(supplied['angles'])
            if 'head_angle_deg' in spec:
                expected_angles['0'] = spec['head_angle_deg']
            assert frame['angles'] == expected_angles, (name, frame['name'])
            assert {k: v for k, v in frame.items() if k not in {'angles', 'name'}} == {
                k: v for k, v in supplied.items() if k not in {'angles', 'name'}
            }
        if filename == 'robot_motions_runtime.json':
            assert motion['completion']['position_tolerance_deg'] == 5.0
    if filename == 'robot_motions_runtime.json':
        for name in ('r공잡기 400', 'r공잡기500'):
            assert actual[name]['end_pose'].endswith('(건공잡기)')
            assert actual[name]['frames'][-1]['name'].endswith('(건공잡기)')
