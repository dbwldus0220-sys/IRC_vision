"""Verify the partial catalog51 import and preserve unrelated walking motions."""

import hashlib
import json
from pathlib import Path
import runpy

import pytest


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261009_catalog51_pickup_startup_update'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())


@pytest.mark.parametrize('filename', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
def test_requested_motions_match_source_and_other_motions_are_preserved(filename):
    source_path = ROOT / MANIFEST['source_file']
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(source_path.read_text())['motions']}
    before = {m['name']: m for m in json.loads(
        (UPDATE / filename.replace('.json', '.before.json')).read_text())['motions']}
    # The later forward import renames eight entries; retain this import boundary.
    snapshot = ROOT / 'artifacts/20261009_catalog51_forward_update' / filename.replace('.json', '.before.json')
    motions = json.loads(snapshot.read_text())['motions']
    live = {m['name']: m for m in json.loads((ROOT / 'artifacts' / filename).read_text())['motions']}
    current = {m['name']: m for m in motions}
    assert len(motions) == len(current) == len(before) + 1
    assert current.keys() == before.keys() | {'오뒤무게중심앞'}
    normalize = runpy.run_path(str(ROOT / 'tools/upsert_motion_catalog.py'))['_with_runtime_policy']
    for name in MANIFEST['updated_motions']:
        expected = normalize(source[name]) if 'runtime' in filename else source[name]
        assert current[name] == expected
        assert live[name] == expected
    for name in before.keys() - {'건공잡기'}:
        assert current[name] == before[name]
    pickup = current['건공잡기']
    assert len(pickup['frames']) == 11
    for original, actual in zip(source['건공잡기']['frames'], pickup['frames'], strict=True):
        assert actual['angles'] == original['angles']
    changed = pickup['frames'][3]['angles']
    assert {k: changed[k] for k in ('3', '5', '7', '9')} == {
        '3': -42.890625, '5': -43.76953125, '7': -12.919921875, '9': 46.93359375,
    }
