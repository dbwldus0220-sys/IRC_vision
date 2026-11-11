"""Verify the three source93 shot angle changes without changing timing or routing."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog93_goal_shots'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    items = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in items}
    assert len(result) == len(items)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_three_requested_shots_change_and_match_source(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / relative)
    actual = catalog(ROOT / relative)
    source = catalog(UPDATE / 'source_robot_motions.json')
    assert len(actual) == len(before) == 160
    assert {n for n in before if actual[n] != before[n]} == set(MANIFEST['imports'])
    for name in MANIFEST['imports']:
        expected = source[name] if kind == 'pc' else runtime_motion_from_source(source[name])
        assert actual[name] == expected
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == MANIFEST['source_sha256']


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
@pytest.mark.parametrize('name,angle', [
    ('r골넣기 5번구역', -170.), ('r골넣기 6번구역', -172.), ('r골넣기7번구역', -173.),
])
def test_exact_second_frame_motor_three_change_preserves_every_other_field(kind, name, angle):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / relative)[name]
    actual = catalog(ROOT / relative)[name]
    expected = copy.deepcopy(before)
    expected['frames'][1]['angles']['3'] = angle
    assert actual == expected
    assert actual['playback_speed'] == 1.0
    assert actual['repeat_count'] == before['repeat_count']
    if kind == 'runtime':
        assert actual == runtime_motion_from_source(actual)


def test_aliases_and_runtime_catalog_resolve_updated_shots():
    assert (ROOT / ALIAS).read_bytes() == (UPDATE / 'before' / ALIAS).read_bytes()
    aliases = yaml.safe_load((ROOT / ALIAS).read_text())['motion_aliases']
    runtime = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    for region, name in zip((5, 6, 7), MANIFEST['imports']):
        assert aliases[f'goal_shot_region_{region}'] == name
    assert set(aliases.values()) <= runtime.keys()
