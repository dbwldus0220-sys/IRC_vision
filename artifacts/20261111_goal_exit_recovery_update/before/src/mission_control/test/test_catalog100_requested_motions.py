"""Source100 mapping, unchanged motions, and source-speed command counts."""

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_catalog100_pickup90'
spec = importlib.util.spec_from_file_location('catalog100_import', UPDATE / 'import_motions.py')
importer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(importer)


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_catalog100_source_fidelity_and_unchanged_entries(kind):
    variants, mapping, overrides = importer.replacements()
    before = {m['name']: m for m in importer.read(UPDATE / f'before/artifacts/robot_motions_{kind}.json')['motions']}
    data = importer.read(ROOT / f'artifacts/robot_motions_{kind}.json')['motions']
    actual = {m['name']: m for m in data}
    assert len(actual) == len(data) == len(before) + 1
    assert len(mapping) == 19 and len(variants) == 21
    for name, motion in variants.items():
        expected = importer.runtime_motion_from_source(motion) if kind == 'runtime' else motion
        assert actual[name] == expected
    for name in before.keys() - variants.keys():
        assert actual[name] == before[name]
    assert overrides == {'n전진45도(8회)': {'source': 6, 'command': 8},
                         'n전진0도(2회)': {'source': 3, 'command': 2}}
    manifest = importer.read(UPDATE / 'manifest.json')
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == manifest['source_sha256']


def test_only_new_alias_added_and_prepare_pose_is_consistent():
    path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    before = yaml.safe_load((UPDATE / 'before' / path).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / path).read_text())['motion_aliases']
    assert after == {**before, 'pickup_forward_to_backward_prepare': 'n공에서 전진하고 후진준비자세'}
    motions = importer.read(ROOT / 'artifacts/robot_motions_runtime.json')['motions']
    assert set(after.values()) <= {m['name'] for m in motions}
    prepare = next(m for m in motions if m['name'] == after['pickup_forward_to_backward_prepare'])
    for f in prepare['frames']:
        for motion in motions:
            for other in motion['frames']:
                if other['name'] == f['name']:
                    assert (f['angles'], f['torques']) == (other['angles'], other['torques'])
    assert max(f['start_ms'] + f['time_ms'] for f in prepare['frames']) == 754
    assert prepare['repeat_count'] == 1 and prepare['playback_speed'] == 1.
