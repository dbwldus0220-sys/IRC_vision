"""Check the requested t-turn restore against its pre-source104 deployment."""

import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node

ROOT = Path(__file__).resolve().parents[3]
ORIGINAL = ROOT / 'artifacts/20261112_catalog104_right_turns/before'
UPDATE = ROOT / 'artifacts/20261112_restore_t_right_turns'
DEPLOYED = ROOT / 'artifacts/20261112_forward_recovery_speed105/before'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from build_post_shot_forward_exits import build_exit


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {motion['name']: motion for motion in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_exact_restore_and_unrelated_data_preserved(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    actual = catalog(DEPLOYED / relative)
    original = catalog(ORIGINAL / relative)
    before = catalog(UPDATE / 'before' / relative)
    assert actual == original
    assert not actual.keys() & MANIFEST['renames'].keys()
    for name in before.keys() - MANIFEST['renames'].keys():
        assert actual[name] == before[name]


def test_aliases_durations_and_timeout_budget_match_runtime():
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    actual = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    original = yaml.safe_load((ORIGINAL / relative).read_text())['motion_aliases']
    assert actual == original
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(actual.values()) <= motions.keys()
    for name, seconds in MANIFEST['duration_seconds'].items():
        motion = motions[name]
        if '+' not in name:
            assert seconds == pytest.approx(max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
                                            * motion['repeat_count'] / motion['playback_speed'] / 1000)
        assert seconds * 1000 + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    for action, (seconds, _) in Node.POST_SHOT_EXIT_CAPTURE_CONFIG.items():
        motion = motions[actual[Bridge.ACTION_TO_MOTION_ID[action]]]
        assert seconds == pytest.approx(motion['max_seq_ms'] / motion['playback_speed'] / 1000)


@pytest.mark.parametrize('field,motor,value', [('angles', '0', -64.0), ('angles', '4', 999.0),
                                             ('torques', '0', False)])
def test_join_rejects_camera_body_and_torque_mismatch(field, motor, value):
    motions = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    turn, recovery = motions['t골대후제자리우회전'], motions['m라인복귀우회전45도']
    assert build_exit(turn, recovery) == motions[turn['name'] + '+라인복귀우회전45도(4회)']
    changed = copy.deepcopy(turn)
    assert changed['frames'][-1][field][motor] != value
    changed['frames'][-1][field][motor] = value
    with pytest.raises(AssertionError):
        build_exit(changed, recovery)
