"""Verify catalog52 imports, repeat compatibility, and untouched motion data."""
import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT/'artifacts/20261027_catalog52_r_update'
MANIFEST = json.loads((UPDATE/'manifest.json').read_text())
SOURCE_PATH = ROOT/MANIFEST['source_file']
SOURCE = {m['name']: m for m in json.loads(SOURCE_PATH.read_text())['motions']}
TURN_UPDATE = json.loads((ROOT/'artifacts/20261028_catalog53_turn_queue/manifest.json').read_text())
SPEED1_MOTIONS = set(json.loads((ROOT/'artifacts/20261028_forward_speed1_boundary/speed_update.json').read_text())['motions'])
SUPERSEDED_TURNS = set(TURN_UPDATE['source_turn_motions']) | set(TURN_UPDATE['derived'])


@pytest.mark.parametrize('filename', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
def test_r_source_and_derived_frames_preserved(filename):
    values = json.loads((ROOT/'artifacts'/filename).read_text())['motions']
    actual = {m['name']: m for m in values}
    assert len(actual) == len(values) == 86
    expected = {name: copy.deepcopy(SOURCE[name]) for name in MANIFEST['source_r_motions']}
    assert len(expected) == 29
    for name, spec in MANIFEST['derived'].items():
        if name == 'R전진라인복귀좌(6회)':
            continue
        motion = copy.deepcopy(SOURCE[spec['source']])
        motion.update(name=name, repeat_count=spec['repeat_count'])
        expected[name] = motion
    for name, motion in expected.items():
        if name in SUPERSEDED_TURNS:
            continue  # Exact replacements are checked in test_catalog53_turn_update.py.
        if name.startswith(('R전진0도(', 'R전진45도(', 'R전진90도(')):
            motion['playback_speed'] = 1.0
        if '라인복귀' in name:
            motion['playback_speed'] = 1.0  # Subsequent approved four-cycle recovery.
        if 'runtime' in filename:
            motion['completion']['position_tolerance_deg'] = 5.0
        assert actual[name] == motion
    before = {m['name']: m for m in json.loads(
        (UPDATE/filename.replace('.json', '.before.json')).read_text())['motions']}
    for name in before.keys() - MANIFEST['renames'].keys():
        if name in TURN_UPDATE['renames'] or name in SUPERSEDED_TURNS:
            continue
        if name == '건라인복귀좌회전45도(1회)':
            continue
        unchanged = copy.deepcopy(before[name])
        if name in SPEED1_MOTIONS:
            unchanged['playback_speed'] = 1.0
        assert actual[name] == unchanged
    assert not (MANIFEST['renames'].keys() & actual.keys())
    if 'runtime' in filename:
        assert all(m['completion']['position_tolerance_deg'] == 5.0 for m in values)


def test_mapping_keeps_old_command_repeat_counts_and_exposes_all_source_r_motions():
    old_aliases = yaml.safe_load((UPDATE/'motion_aliases.before.yaml').read_text())['motion_aliases']
    aliases = yaml.safe_load((ROOT/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    before = {m['name']: m for m in json.loads((UPDATE/'robot_motions_runtime.before.json').read_text())['motions']}
    actual = {m['name']: m for m in json.loads((ROOT/'artifacts/robot_motions_runtime.json').read_text())['motions']}
    assert old_aliases.keys() <= aliases.keys()
    for key, old in old_aliases.items():
        if key == 'line_recovery_left_6':
            assert aliases[key] == 'R전진라인복귀좌'
            assert actual[aliases[key]]['repeat_count'] == 4
        else:
            previous = MANIFEST['renames'].get(old, old)
            assert aliases[key] == TURN_UPDATE['renames'].get(previous, previous)
            assert actual[aliases[key]]['repeat_count'] == before[old]['repeat_count']
    assert set(aliases.values()) <= actual.keys()
    assert set(MANIFEST['source_r_motions']) <= set(aliases.values())
    assert aliases['line_turn_right_3'] == 'R제자리우회전(3회)'
    assert aliases['line_turn_right_5'] == 'R제자리우회전(5회('
    assert aliases['line_forward_2'] == 'R전진45도(2회)'
    assert aliases['pickup_first_backward_turn_right'] == 'R후진에서제자리우회전(공)'


def test_source_and_initial_posture_preserved():
    assert hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    assert SOURCE['R오뒤45도']['completion']['position_tolerance_deg'] == 2.0
    before = next(m for m in json.loads((UPDATE/'robot_motions_runtime.before.json').read_text())['motions']
                  if m['name'] == '오뒤무게중심앞')['frames'][0]['angles']
    for filename in ('robot_motions_runtime.json', 'robot_motions_pc.json'):
        matches = [f['angles'] for m in json.loads((ROOT/'artifacts'/filename).read_text())['motions']
                   for f in m['frames'] if f['name'] == '오뒤무게중심앞']
        assert matches and all(angles == before for angles in matches)


def test_recovery_is_four_cycles_and_forward_uses_approved_speed():
    aliases = yaml.safe_load((ROOT/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    for filename in ('robot_motions_runtime.json', 'robot_motions_pc.json'):
        motions = json.loads((ROOT/'artifacts'/filename).read_text())['motions']
        recoveries = {m['name']: m for m in motions if '라인복귀' in m['name']}
        assert set(recoveries) == {'R라인복귀우회전', 'R전진라인복귀좌'}
        assert all(m['repeat_count'] == 4 and m['playback_speed'] == 1.0 for m in recoveries.values())
        forwards = [m for m in motions if m['name'].startswith(('R전진45도(', 'R전진90도(', 'R전진0도('))]
        assert len(forwards) == 10
        assert all(m['playback_speed'] == 1.0 for m in forwards)
        assert all(target in recoveries for key, target in aliases.items() if key.startswith('line_recovery_'))
