"""Check the speed restoration against the exact pre-edit production catalog."""

import json
from pathlib import Path

import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge


ROOT = Path(__file__).resolve().parents[3]
BEFORE = ROOT / 'artifacts/20261110_video_review_speed1/before'
AFTER = ROOT / 'artifacts/20261110_catalog87_check_pose/before'


def test_only_fifteen_runtime_speeds_change():
    path = Path('artifacts/robot_motions_runtime.json')
    expected = json.loads((BEFORE / path).read_text())
    names = []
    for motion in expected['motions']:
        if '전진' in motion['name'] or '라인복귀' in motion['name']:
            assert motion['playback_speed'] == 1.05
            motion['playback_speed'] = 1.0
            names.append(motion['name'])
            duration = max(frame['start_ms'] + frame['time_ms'] for frame in motion['frames'])
            duration *= motion['repeat_count'] / motion['playback_speed']
            assert duration + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    assert len(names) == 15
    assert json.loads((AFTER / path).read_text()) == expected
    pc = Path('artifacts/robot_motions_pc.json')
    assert (AFTER / pc).read_bytes() == (BEFORE / pc).read_bytes()


def test_grasp_check_still_uses_source86_pose():
    aliases = yaml.safe_load((AFTER / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    assert aliases['pickup_grasp_check_pose'] == 'p공확인자세'
    source_path = ROOT / 'artifacts/20261110_catalog86_crab_entry/source_robot_motions.json'
    source = {motion['name']: motion for motion in json.loads(source_path.read_text())['motions']}
    runtime = {motion['name']: motion for motion in json.loads(
        (AFTER / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    expected = source['p공확인자세']
    expected['completion']['position_tolerance_deg'] = 5.0
    assert runtime['p공확인자세'] == expected
