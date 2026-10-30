"""Check source59 motion data through deployed aliases and capture timing."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261029_ball_search_catalog59'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_import_changes_only_requested_fine_gaits_and_preserves_source_data(filename):
    raw = (UPDATE / 'source59.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST['source_sha256']
    source = {m['name']: m for m in json.loads(raw)['motions']}
    expected = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    positions = {m['name']: i for i, m in enumerate(expected['motions'])}
    for name, spec in MANIFEST['motions'].items():
        motion = copy.deepcopy(source[spec['source']])
        motion.update(name=name, repeat_count=spec['repeat_count'])
        if 'runtime' in filename:
            motion['completion']['position_tolerance_deg'] = 5.0
        if name in positions:
            expected['motions'][positions[name]] = motion
        else:
            expected['motions'].append(motion)
    actual = json.loads((ROOT / 'artifacts' / filename).read_text())
    assert actual == expected


def test_goal_ball_aliases_resolve_exactly_and_goal_capture_matches_saved_timing():
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    aliases = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    expected = {**before, **{f'goal_camera_90_fine_forward_{n}': f'R미세90도({n}회)'
                            for n in range(1, 5)}}
    assert aliases == expected
    catalog = {m['name']: m for m in json.loads(
        (ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    assert set(aliases.values()) <= catalog.keys()
    for count in range(1, 5):
        action = f'GOAL_CAMERA90_FINE_FORWARD_{count}'
        motion = catalog[aliases[MotionCommandBridgeNode.ACTION_TO_MOTION_ID[action]]]
        assert motion['name'] == f'R미세90도({count}회)'
        assert motion['repeat_count'] == count
        cycle_ms = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
        assert MotionDecisionNode.GOAL_FORWARD_DURATION_SEC[action] == pytest.approx(
            cycle_ms / 1000 * count / motion['playback_speed'])
    for alias in ('ball_general_fine_forward_8', 'pickup_fine_forward_0'):
        assert aliases[alias] == 'r미세0도'
    for alias in ('ball_top_loss_fine_forward_45_3', 'pickup_top_loss_fine_forward_0_3'):
        assert aliases[alias] == 'r미세0도(상단탐색3회)'
