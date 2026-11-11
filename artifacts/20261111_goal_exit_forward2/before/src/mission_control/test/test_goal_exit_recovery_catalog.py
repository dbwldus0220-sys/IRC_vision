"""Verify the requested goal exits without changing unrelated motion data."""

import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
BEFORE = ROOT / 'artifacts/20261111_goal_exit_turn6_4/before'
RENAMES = json.loads((ROOT / 'artifacts/20261111_catalog100_name_mapping/manifest.json').read_text())['renames']
sys.path.insert(0, str(ROOT / 'tools'))
from build_post_shot_forward_exits import build_exit, build_turn_variant, GOAL_EXIT_RECOVERIES
from upsert_motion_catalog import runtime_motion_from_source


EXPECTED_ALIASES = {
    'post_shot_exit_right_9_recovery_right_4': 't골대후제자리우회전+라인복귀우회전45도(4회)',
    'post_shot_exit_left_4_recovery_left_4': 'p골대후제자리좌회전+라인복귀좌회전45도(4회)',
    'post_shot_exit_right_5_forward_4': 't골대후제자리우회전(6회)' + '+라인복귀좌회전45도(4회)' * 2,
    'post_shot_exit_left_3_forward_4': 'p골대후제자리좌회전(4회)' + '+라인복귀우회전45도(4회)' * 2,
}
OLD_COMPOSITES = {
    't골대후제자리우회전(5회)' + '+라인복귀우회전45도(4회)' * 2,
    'p골대후제자리좌회전(3회)' + '+라인복귀좌회전45도(4회)' * 2,
}
NEW_TURNS = {'t골대후제자리우회전(6회)', 'p골대후제자리좌회전(4회)'}


def catalog(path):
    data = json.loads(path.read_text())['motions']
    result = {RENAMES.get(motion['name'], motion['name']):
              {**motion, 'name': RENAMES.get(motion['name'], motion['name'])}
              for motion in data}
    assert len(result) == len(data)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_two_exit_composites_change_with_two_new_turn_variants(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before, after = catalog(BEFORE / relative), catalog(ROOT / relative)
    assert after.keys() == before.keys() - OLD_COMPOSITES | set(EXPECTED_ALIASES.values()) | NEW_TURNS
    for name in before.keys() - OLD_COMPOSITES:
        assert after[name] == before[name], name
    source = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    for name, count in [('t골대후제자리우회전', 6), ('p골대후제자리좌회전', 4)]:
        expected = build_turn_variant(source[name], count)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[expected['name']] == expected
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(source[turn], source[recovery], blocks)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[expected['name']] == expected
        # The join is at the existing support pose on both sides.
        for field in ('angles', 'torques'):
            assert source[turn]['frames'][-1][field] == source[recovery]['frames'][-1][field]


def test_only_requested_aliases_change_and_all_targets_exist():
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    before = yaml.safe_load((BEFORE / relative).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    expected = {**before, **EXPECTED_ALIASES}
    # The deployed ring policy already removed the unsupported region-7 alias.
    expected.pop('goal_shot_region_7', None)
    assert after == {alias: RENAMES.get(name, name) for alias, name in expected.items()}
    assert set(after.values()) <= catalog(ROOT / 'artifacts/robot_motions_runtime.json').keys()
