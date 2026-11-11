"""Source100 names must preserve motion data and every deployed command target."""

import copy
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_catalog100_name_mapping'
RENAMES = json.loads((UPDATE / 'manifest.json').read_text())['renames']


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_rename_changes_only_motion_names(kind):
    relative = f'artifacts/robot_motions_{kind}.json'
    before = json.loads((UPDATE / 'before' / relative).read_text())
    actual = json.loads((ROOT / relative).read_text())
    expected = copy.deepcopy(before)
    for motion in expected['motions']:
        motion['name'] = RENAMES.get(motion['name'], motion['name'])
    # The later forward-two exit request changes only these two composite bodies.
    revised_exits = {
        't골대후제자리우회전(6회)' + '+라인복귀좌회전45도(4회)' * 2,
        'p골대후제자리좌회전(4회)' + '+라인복귀우회전45도(4회)' * 2,
    }
    assert {k: v for k, v in actual.items() if k != 'motions'} == {k: v for k, v in expected.items() if k != 'motions'}
    assert {m['name']: m for m in actual['motions'] if m['name'] not in revised_exits} == {
        m['name']: m for m in expected['motions'] if m['name'] not in revised_exits}
    motions = {m['name']: m for m in actual['motions']}
    assert len(motions) == len(actual['motions'])
    assert not motions.keys() & RENAMES.keys()
    assert motions['p전진45(8회)']['repeat_count'] == 8
    assert motions['p전진0도(2회)']['repeat_count'] == 2


def test_every_command_keeps_its_original_data_after_rename():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    actual = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert actual == {alias: RENAMES.get(name, name) for alias, name in before.items()}
    motions = json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    assert set(actual.values()) <= {m['name'] for m in motions}
