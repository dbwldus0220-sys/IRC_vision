"""Check requested imports, derived repeats, aliases, and unchanged robot data."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_catalog54_55_pickup'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())


@pytest.mark.parametrize('filename', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
def test_import_preserves_source_targets_timing_and_unrelated_motions(filename):
    sources = {}
    for number, spec in MANIFEST['sources'].items():
        path = ROOT / spec['path']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == spec['sha256']
        sources[int(number)] = {m['name']: m for m in json.loads(path.read_text())['motions']}
    data = json.loads((ROOT / 'artifacts' / filename).read_text())['motions']
    actual = {m['name']: m for m in data}
    assert len(actual) == len(data) == 90
    expected = {name: copy.deepcopy(sources[spec['catalog']][spec['source']])
                for name, spec in MANIFEST['imports'].items()}
    for name, spec in MANIFEST['derived'].items():
        motion = copy.deepcopy(sources[55][spec['source']])
        motion['name'] = name
        if 'repeat_count' in spec:
            motion['repeat_count'] = spec['repeat_count']
        if 'head_angle_deg' in spec:
            poses = {f['name']: f['name'] + '(회전준비90도)' for f in motion['frames']}
            for frame in motion['frames']:
                frame['name'] = poses[frame['name']]
                frame['angles']['0'] = spec['head_angle_deg']
            motion['start_pose'] = poses[motion['start_pose']]
            motion['end_pose'] = poses[motion['end_pose']]
        expected[name] = motion
    for name, motion in expected.items():
        if 'runtime' in filename:
            # Follow-up request scales all catalog54/55 imports and derivatives.
            motion['playback_speed'] = round(motion['playback_speed'] * .95, 12)
            motion['completion']['position_tolerance_deg'] = 5.0
            poses = MANIFEST['runtime_pose_renames'].get(name, {})
            for frame in motion['frames']:
                frame['name'] = poses.get(frame['name'], frame['name'])
            for key in ('start_pose', 'end_pose'):
                motion[key] = poses.get(motion[key], motion[key])
        assert actual[name] == motion
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())['motions']
    for motion in before:
        if motion['name'] not in MANIFEST['renames'] and motion['name'] not in expected:
            assert actual[motion['name']] == motion
    assert not actual.keys() & MANIFEST['renames'].keys()


def test_aliases_resolve_and_retain_requested_repeat_counts():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    aliases = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    expected = {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    expected.update(pickup='r공잡기 400', sdk_pickup='r공잡기 400',
                    pickup_400='r공잡기 400', pickup_500='r공잡기500')
    assert aliases == expected
    motions = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    assert set(aliases.values()) <= motions.keys()
    for alias, count in [('pickup_pre_backward_camera_down', 2), ('pickup_400', 1),
                         ('pickup_500', 1), ('ball_general_fine_forward_8', 5),
                         ('pickup_fine_forward_0', 5), ('hurdle_fine_forward_10', 10),
                         ('ball_top_loss_fine_forward_45_3', 3), ('pickup_top_loss_fine_forward_0_3', 3),
                         *[(f'goal_camera_90_fine_forward_{i}', i) for i in range(1, 5)]]:
        assert motions[aliases[alias]]['repeat_count'] == count


def test_reimport_keeps_runtime_pickup_pose_names_and_source_angles():
    import runpy
    normalize = runpy.run_path(str(ROOT / 'tools/upsert_motion_catalog.py'))['_with_runtime_policy']
    source = {m['name']: m for m in json.loads((ROOT / MANIFEST['sources']['54']['path']).read_text())['motions']}
    runtime = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for name in ('r공잡기 400', 'r공잡기500'):
        expected = normalize(source[name])
        expected['playback_speed'] = round(expected['playback_speed'] * .95, 12)
        assert expected == runtime[name]
        assert normalize(runtime[name]) == runtime[name]
