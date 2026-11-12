"""Check deployed right-turn source data, aliases, pose identities and time budgets."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import camera45_turn_duration_sec
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261112_stationary_right103'
SOURCE = {m['name']: m for m in json.loads(
    (UPDATE / 'source_robot_motions.json').read_text())['motions']}
ALIASES = yaml.safe_load((ROOT /
    'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
COUNTS = (1, 2, 3, 4, 5, 6, 7, 9)


def catalog(kind):
    motions = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
@pytest.mark.parametrize('camera', [0, 45, 90])
@pytest.mark.parametrize('count', COUNTS)
def test_all_standalone_turns_match_source(kind, camera, count):
    name = f't제자리우회전{camera}도({count}회)'
    expected = copy.deepcopy(SOURCE.get(name, SOURCE[f't제자리우회전{camera}도(2회)']))
    expected.update(name=name, repeat_count=count)
    if camera == 45:
        expected['end_pose'] = '오뒤무게중심앞(catalog70)'
        expected['frames'][-1]['name'] = expected['end_pose']
    if kind == 'runtime':
        expected['completion']['position_tolerance_deg'] = 5.
    actual = catalog(kind)[name]
    assert actual == expected
    duration = max(f['start_ms'] + f['time_ms'] for f in actual['frames'])
    duration *= count / actual['playback_speed'] / 1000
    assert duration == pytest.approx(camera45_turn_duration_sec('RIGHT', count))
    assert duration * 1000 + actual['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS


def test_every_right_turn_alias_and_action_resolves():
    motions = catalog('runtime')
    assert set(ALIASES.values()) <= motions.keys()
    targets = {name for name in ALIASES.values() if '제자리우회전' in name}
    assert len(targets) == 29  # The six-cycle composite prefix is internal only.
    for camera in (0, 45, 90):
        for count in COUNTS:
            assert f't제자리우회전{camera}도({count}회)' in targets
    for action, alias in Bridge.ACTION_TO_MOTION_ID.items():
        if alias in ALIASES and ALIASES[alias] in targets:
            assert Bridge.motion_id_for_action(action) == alias
    assert all(name.startswith(('t제자리우회전', 't공후 제자리우회전', 't골대후제자리우회전'))
               for name in targets)


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_right_turn_pose_does_not_collide_with_startup(kind):
    motions = catalog(kind)
    poses = {}
    for motion in motions.values():
        for frame in motion['frames']:
            if frame['name'] in {'오뒤무게중심앞', '오뒤무게중심앞(catalog70)'}:
                pose = (frame['angles'], frame['torques'])
                assert poses.setdefault(frame['name'], pose) == pose
    assert len(poses) == 2
    assert poses['오뒤무게중심앞'] != poses['오뒤무게중심앞(catalog70)']
