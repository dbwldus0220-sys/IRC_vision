"""Check the approved 전. replacement against the supplied motion data."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261027_catalog40_forward_pickup_update'
SOURCE_PATH = ROOT / 'motion_imports/robot_motions(40)(1).json'
SOURCE = {m['name']: m for m in json.loads(SOURCE_PATH.read_text())['motions']}
ALIASES = yaml.safe_load(
    (ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text()
)['motion_aliases']


@pytest.mark.parametrize('filename', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
def test_forward_and_pickup_match_source_and_other_motions_are_unchanged(filename):
    before = {m['name']: m for m in json.loads(
        (UPDATE / filename.replace('.json', '.before.json')).read_text()
    )['motions']}
    motions = json.loads((ROOT / 'artifacts' / filename).read_text())['motions']
    current = {m['name']: m for m in motions}
    runtime = 'runtime' in filename
    old_names = set()
    new_names = set()
    for camera in (45, 90):
        for count in (2, 4, 6, 8):
            old = f'건전진{camera}도({count}회)'
            name = f'전{camera}도({count}회)'
            old_names.add(old)
            new_names.add(name)
            expected = copy.deepcopy(SOURCE['전.'])
            expected.update(name=name, repeat_count=count, playback_speed=0.9)
            for frame, camera_frame in zip(expected['frames'], SOURCE[old]['frames'], strict=True):
                frame['angles']['0'] = camera_frame['angles']['0']
                if camera == 90:
                    frame['name'] += '(90도)'
            expected['start_pose'] = expected['frames'][0]['name']
            expected['end_pose'] = expected['frames'][-1]['name']
            if runtime:
                expected['completion']['position_tolerance_deg'] = 5.0
            assert current[name] == expected
            assert max(f['start_ms'] + f['time_ms'] for f in current[name]['frames']) == 506

    pickup = copy.deepcopy(SOURCE['건공잡기'])
    if runtime:
        pickup['completion']['position_tolerance_deg'] = 5.0
        for frame in pickup['frames']:
            if frame['name'] in ('미세후진오뒤(0도)', '미세오뒤4111(45도)'):
                frame['name'] += '(건공잡기)'
        pickup['end_pose'] += '(건공잡기)'
    assert current['건공잡기'] == pickup
    assert len(motions) == len(current) == len(before)
    assert current.keys() == before.keys() - old_names | new_names
    for name in before.keys() - old_names - {'건공잡기'}:
        # The subsequent shoulder-policy rollback changes only joints 4 and 5.
        previous = copy.deepcopy(before[name])
        for frame, updated in zip(previous['frames'], current[name]['frames'], strict=True):
            for joint in ('4', '5'):
                frame['angles'][joint] = updated['angles'][joint]
        assert current[name] == previous
    if runtime:
        assert all(m['completion']['position_tolerance_deg'] == 5.0 for m in motions)
    # New frame names must not resolve to conflicting poses in another motion.
    for name in new_names:
        for frame in current[name]['frames']:
            assert all(other['angles'] == frame['angles']
                       for motion in motions for other in motion['frames']
                       if other['name'] == frame['name'])


def test_all_forward_aliases_and_pickup_resolve_to_requested_variants():
    before = yaml.safe_load((UPDATE / 'motion_aliases.before.yaml').read_text())['motion_aliases']
    expected = {
        alias: target.replace('건전진', '전') if target.startswith('건전진') else target
        for alias, target in before.items()
    }
    assert ALIASES == expected
    assert sum(name.startswith('전') for name in ALIASES.values()) == 14
    assert ALIASES['pickup'] == ALIASES['sdk_pickup'] == '건공잡기'
    for filename in ('robot_motions_runtime.json', 'robot_motions_pc.json'):
        names = {m['name'] for m in json.loads((ROOT / 'artifacts' / filename).read_text())['motions']}
        assert set(ALIASES.values()) <= names
    assert ALIASES[Bridge.motion_id_for_action('STRAIGHT')] == '전45도(6회)'
    assert ALIASES[Bridge.motion_id_for_action('GOAL_CAMERA_90_FORWARD')] == '전90도(6회)'


def test_original_source_is_preserved():
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    assert hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() == manifest['source_sha256']
    assert SOURCE['건전진45도(6회)']['repeat_count'] == 4
    assert SOURCE['전.']['repeat_count'] == 4
    assert SOURCE['건공잡기']['completion']['position_tolerance_deg'] == 2.0
