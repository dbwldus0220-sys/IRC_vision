"""Check source99 import fidelity, derived counts, and preserved goal exit hold."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_catalog99_forward_recovery'
DEPLOYED = ROOT / 'artifacts/20261111_catalog100_pickup90/before'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, EXIT_RECOVERIES


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_import_preserves_source_and_only_replaces_requested_motions(kind):
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    source_path = UPDATE / 'source_robot_motions.json'
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == manifest['source_sha256']
    source = catalog(source_path)
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    actual = catalog(DEPLOYED / f'artifacts/robot_motions_{kind}.json')
    assert actual.keys() == before.keys()
    for name in manifest['imports']:
        src, count = manifest['derived'].get(name, (name, source.get(name, {}).get('repeat_count')))
        expected = copy.deepcopy(source[src])
        expected.update(name=name, repeat_count=count)
        renames = manifest['pose_renames']
        for frame in expected['frames']:
            frame['name'] = renames.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = renames.get(expected[key], expected[key])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert actual[name] == expected, name
    pc = catalog(DEPLOYED / 'artifacts/robot_motions_pc.json')
    for turn, recovery in EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert actual[expected['name']] == expected
        turn_end = max(f['start_ms'] + f['time_ms'] for f in pc[turn]['frames'])
        start = expected['frames'][len(pc[turn]['frames'])]['start_ms']
        assert (start - turn_end) / expected['playback_speed'] == 500
    changed = set(manifest['imports'] + manifest['composites'])
    for name in actual.keys() - changed:
        assert actual[name] == before[name], name
    aliases = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    assert (DEPLOYED / aliases).read_bytes() == (UPDATE / 'before' / aliases).read_bytes()
