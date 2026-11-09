"""Verify the check-pose replacement and speed-only walking update offline."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_line_turn_prequeue import start


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog85_check_speed'
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {motion['name']: motion for motion in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_check_pose_and_requested_runtime_speeds_change(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = json.loads((UPDATE / 'before' / relative).read_text())
    after = json.loads((ROOT / 'artifacts/20261110_catalog86_crab_entry/before' / relative).read_text())
    source = json.loads((UPDATE / 'source_check_pose.json').read_text())
    expected = copy.deepcopy(before)
    changed_speeds = []
    for index, motion in enumerate(expected['motions']):
        if motion['name'] == '찐공확인자세':
            expected['motions'][index] = copy.deepcopy(source)
            if kind == 'runtime':
                expected['motions'][index]['completion']['position_tolerance_deg'] = 5.0
        elif kind == 'runtime' and ('전진' in motion['name'] or '라인복귀' in motion['name']):
            changed_speeds.append(motion['name'])
            motion['playback_speed'] = 1.05
    assert after == expected
    assert len(changed_speeds) == (15 if kind == 'runtime' else 0)


def test_alias_pose_lookup_and_completion_budget():
    snapshot = ROOT / 'artifacts/20261110_catalog87_check_pose/before'
    aliases = yaml.safe_load((snapshot / ALIAS_PATH).read_text())['motion_aliases']
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    assert aliases == {**before, 'pickup_grasp_check_pose': 'p공확인자세'}
    runtime = catalog(snapshot / 'artifacts/robot_motions_runtime.json')
    assert set(aliases.values()) <= runtime.keys()
    assert set(Bridge.ACTION_TO_MOTION_ID.values()) <= aliases.keys()
    assert '찐공확인자세' not in runtime
    check = runtime['p공확인자세']
    assert check['playback_speed'] == 1.0
    assert check['start_pose'] == check['end_pose'] == 'tt공확인자세'
    poses = [frame for motion in runtime.values() for frame in motion['frames']
             if frame['name'] == check['end_pose']]
    assert poses
    assert all((frame['angles'], frame['torques']) ==
               (poses[0]['angles'], poses[0]['torques']) for frame in poses)
    for motion in runtime.values():
        if motion['name'] != 'p공확인자세' and not any(
                word in motion['name'] for word in ('전진', '라인복귀')):
            continue
        duration_ms = max(frame['start_ms'] + frame['time_ms'] for frame in motion['frames'])
        duration_ms *= motion['repeat_count'] / motion['playback_speed']
        assert duration_ms + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS


@pytest.mark.parametrize('action', [action for action in Node.LINE_MOTION_CAPTURE_CONFIG
                                    if action in Bridge.ACTION_TO_MOTION_ID])
def test_line_capture_duration_uses_actual_catalog_speed(action):
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    runtime = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    motion = runtime[aliases[Bridge.motion_id_for_action(action)]]
    duration = max(frame['start_ms'] + frame['time_ms'] for frame in motion['frames'])
    duration *= motion['repeat_count'] / motion['playback_speed'] / 1000
    assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(duration)


@pytest.mark.parametrize('action,phase,count', [
    ('LINE_SPARSE_FORWARD', 'LINE_TRACK_AFTER_PICKUP', 2),
    ('POST_SHOT_FORWARD', 'POST_SHOT_LINE_ALIGN', 6),
])
def test_post_pickup_and_post_shot_capture_windows(clock, action, phase, count):
    node, bridge, request = start(clock, action, phase)
    assert node.active_line_motion_duration_sec == pytest.approx(.514 * count / 1.05)
