"""Verify source57 fine gait data, navigation aliases, and unrelated data."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_catalog57_fine_forward'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_source57_fine_gaits_preserve_targets_timing_torques_and_source_speed(filename):
    source_path = ROOT / MANIFEST['source_file']
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(source_path.read_text())['motions']}
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    # Later right-turn additions are checked against this import's completed snapshot.
    actual = json.loads((ROOT / 'artifacts/20261028_right_turn_bins/before/artifacts' / filename).read_text())
    expected = copy.deepcopy(before)
    for i, old in enumerate(before['motions']):
        spec = MANIFEST['motions'].get(old['name'])
        if spec is None:
            continue
        motion = copy.deepcopy(source[spec['source']])
        motion['name'] = old['name']
        motion['repeat_count'] = spec['repeat_count']
        if 'runtime' in filename:
            motion['completion']['position_tolerance_deg'] = 5.0
        expected['motions'][i] = motion
    # Full-catalog equality also protects every unrelated motion and metadata field.
    assert actual == expected


def test_all_active_fine_aliases_select_source57_camera_specific_gaits():
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    assert {key: aliases[key] for key in before} == {**before, **MANIFEST['alias_changes']}
    motions = json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    catalog = {m['name']: m for m in motions}
    assert len(catalog) == len(motions)
    assert set(aliases.values()) <= catalog.keys()
    expected = {
        'ball_general_fine_forward_8': ('r미세0도', 5, -64.0),
        'pickup_fine_forward_0': ('r미세0도', 5, -64.0),
        'hurdle_fine_forward_10': ('r미세0도(허들10회)', 10, -64.0),
        'ball_top_loss_fine_forward_45_3': ('r미세0도(상단탐색3회)', 3, -64.0),
        'pickup_top_loss_fine_forward_0_3': ('r미세0도(상단탐색3회)', 3, -64.0),
        **{f'goal_camera_90_fine_forward_{n}': (f'r마세90도({n}회)', n, 1.0)
           for n in range(1, 5)},
    }
    for alias, (name, count, head_angle) in expected.items():
        assert aliases[alias] == name
        motion = catalog[name]
        assert motion['repeat_count'] == count
        assert motion['playback_speed'] == 1.05
        assert all(f['angles']['0'] == head_angle for f in motion['frames'])
