"""Import exact supplied left turns, including derived counts, without other edits."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_catalog56_ball360'
DEPLOYED = ROOT / 'artifacts/20261028_catalog57_default_pose/before'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_catalog56_preserves_source_frames_repeats_speeds_and_unrelated_motions(filename):
    source_path = ROOT / MANIFEST['source_file']
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(source_path.read_text())['motions']}
    before = {m['name']: m for m in json.loads((UPDATE / 'before/artifacts' / filename).read_text())['motions']}
    data = json.loads((DEPLOYED / 'artifacts' / filename).read_text())['motions']
    actual = {m['name']: m for m in data}
    assert len(actual) == len(data) == len(before)
    expected = {n: copy.deepcopy(source[n]) for n in MANIFEST['source_left_motions']}
    for name, spec in MANIFEST['derived'].items():
        expected[name] = {**copy.deepcopy(source[spec['source']]), 'name': name,
                          'repeat_count': spec['repeat_count']}
    for name, old in before.items():
        wanted = expected.get(name, old)
        if name in expected and 'runtime' in filename:
            wanted['completion']['position_tolerance_deg'] = 5.
            wanted['playback_speed'] = old['playback_speed']
        assert actual[name] == wanted
    for count in range(1, 7):
        motion = actual[f'R제자리좌회전90도({count}회)']
        assert [f['name'] for f in motion['frames']] == ['R제좌왼들90도', 'R김오뒤90도']
        assert motion['end_pose'] == 'R김오뒤90도'
        assert motion['repeat_count'] == count


def test_existing_aliases_resolve_to_updated_turns():
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    assert (DEPLOYED / relative).read_bytes() == (UPDATE / 'before' / relative).read_bytes()
    aliases = yaml.safe_load((DEPLOYED / relative).read_text())['motion_aliases']
    catalog = {m['name']:m for m in json.loads((DEPLOYED / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    assert set(aliases.values()) <= catalog.keys()
    for count in range(1, 7):
        motion = catalog[aliases[f'goal_camera_90_turn_left_{count}']]
        assert motion['end_pose'] == 'R김오뒤90도'
    assert catalog[aliases['post_shot_default_turn_left']]['end_pose'] == '오뒤무게중심앞'
