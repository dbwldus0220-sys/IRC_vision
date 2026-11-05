"""Verify source71 crab data and the source72 camera correction without logic changes."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261105_catalog71_crab'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
SOURCE = {m['name']: m for m in json.loads(
    (UPDATE / 'source_requested.json').read_text())['motions']}
SOURCE.update({m['name']: m for m in json.loads((ROOT /
    'artifacts/20261105_catalog72_crab90/source_requested.json').read_text())['motions']})


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_exact_source_data_and_all_other_motions_preserved(kind):
    motions = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    catalog = {m['name']: m for m in motions}
    assert len(catalog) == len(motions) == 143
    for name, original in SOURCE.items():
        expected = copy.deepcopy(original)
        if kind == 'runtime':
            expected['completion']['position_tolerance_deg'] = 5.0
        assert catalog[name] == expected
    for name, digest in MANIFEST['catalogs'][kind]['unchanged_motion_sha256'].items():
        encoded = json.dumps(catalog[name], ensure_ascii=False, sort_keys=True).encode()
        assert hashlib.sha256(encoded).hexdigest() == digest, name


def test_existing_aliases_still_select_correct_camera_and_repeat_count():
    aliases = yaml.safe_load((ROOT /
        'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    for alias, name in {
        'pickup_crab_right_0': 't미세오옆꽃게(0도)-1회',
        'pickup_crab_right_0_2': 't미세오옆꽃게(0도)-2회',
        'goal_camera_90_crab_right': 't미세오옆꽃게90도(1회)',
        'goal_camera_90_crab_right_2': 't미세오옆꽃게90도(2회)',
    }.items():
        assert aliases[alias] == name


@pytest.mark.parametrize('name', list(SOURCE))
def test_reservation_metadata_and_duration_unchanged(name):
    before = {m['name']: m for m in json.loads(
        (UPDATE / 'before_pc_requested.json').read_text())['motions']}[name]
    after = SOURCE[name]
    for key in ('name', 'repeatable', 'repeat_count', 'playback_speed', 'start_pose', 'end_pose'):
        assert after[key] == before[key]
    assert [(f['start_ms'], f['time_ms']) for f in after['frames']] == [
        (f['start_ms'], f['time_ms']) for f in before['frames']]
