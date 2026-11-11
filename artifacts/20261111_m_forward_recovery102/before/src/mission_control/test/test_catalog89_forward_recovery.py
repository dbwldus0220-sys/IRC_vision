"""Source89 fidelity, original speeds and late-motion reservation timing."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_line_continuous_missions import clock, start_forward, capture
from test_wait_refresh_and_fine_settle import LiveInputHarness


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog89_forward_recovery'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    values = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in values}
    assert len(values) == len(result)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')


def seconds(alias):
    motion = RUNTIME[ALIASES[alias]]
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_fifteen_motions_are_imported_from_source89(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / relative)
    actual = catalog(ROOT / relative)
    expected = {}
    for old_name, original in before.items():
        name = MANIFEST['renames'].get(old_name, old_name)
        if name not in MANIFEST['imports']:
            expected[name] = original
            continue
        source_name, count = MANIFEST['derived'].get(name, (name, original['repeat_count']))
        motion = copy.deepcopy(SOURCE[source_name])
        motion.update(name=name, repeat_count=count)
        poses = MANIFEST['pose_renames'][name]
        for frame in motion['frames']:
            frame['name'] = poses.get(frame['name'], frame['name'])
        for field in ('start_pose', 'end_pose'):
            motion[field] = poses.get(motion[field], motion[field])
        expected[name] = runtime_motion_from_source(motion) if kind == 'runtime' else motion
    assert actual == expected
    assert len(actual) == 156 and len(MANIFEST['imports']) == 15
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == MANIFEST['source_sha256']


@pytest.mark.parametrize('name', MANIFEST['imports'])
def test_original_speed_completion_budget_and_idempotent_import(name):
    runtime = RUNTIME[name]
    pc = catalog(ROOT / 'artifacts/robot_motions_pc.json')[name]
    assert runtime['playback_speed'] == pc['playback_speed'] == 1.0
    assert runtime == runtime_motion_from_source(pc)
    assert runtime == runtime_motion_from_source(runtime)
    duration_ms = max(f['start_ms'] + f['time_ms'] for f in runtime['frames']) * runtime['repeat_count'] / runtime['playback_speed']
    assert duration_ms + runtime['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    assert runtime['completion']['position_tolerance_deg'] == 5.0


def test_names_startup_poses_and_all_bridge_routes_resolve():
    assert (ROOT / ALIAS_PATH).read_bytes() == (UPDATE / 'before' / ALIAS_PATH).read_bytes()
    assert set(Bridge.ACTION_TO_MOTION_ID.values()) <= ALIASES.keys()
    assert set(ALIASES.values()) <= RUNTIME.keys()
    assert 'R라인복귀우회전90도' not in RUNTIME
    assert 't라인복귀우실전90도' in RUNTIME
    assert ALIASES['pickup_grasp_check_pose'] == 'n공확인자세'
    for kind in ('pc', 'runtime'):
        motions = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
        touched = {'오뒤무게중심앞'} | {f['name'] for n in MANIFEST['imports'] for f in motions[n]['frames']}
        for pose in touched:
            frames = [f for m in motions.values() for f in m['frames'] if f['name'] == pose]
            assert frames and all((f['angles'], f['torques']) == (frames[0]['angles'], frames[0]['torques']) for f in frames), pose


@pytest.mark.parametrize('action', [action for action in Node.LINE_MOTION_CAPTURE_CONFIG
                                    if action in Bridge.ACTION_TO_MOTION_ID])
def test_line_capture_duration_equals_actual_motion_playback(action):
    assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(seconds(Bridge.motion_id_for_action(action)))


@pytest.mark.parametrize('action,alias', [
    ('STRAIGHT', 'line_forward_4'), ('RECOVER_LEFT_TURN_LEFT_4', 'line_recovery_left_4'),
    ('RECOVER_RIGHT_TURN_RIGHT_4', 'line_recovery_right_4'),
])
def test_ball_and_hurdle_approach_duration_equals_playback(action, alias):
    assert Node.APPROACH_WALK_DURATION_SEC[action] == pytest.approx(seconds(alias))


@pytest.mark.parametrize('action,alias', [
    ('GOAL_CAMERA_90_FORWARD', 'goal_camera_90_forward_6'),
    ('GOAL_CAMERA_90_FORWARD_1', 'goal_camera_90_forward_2'),
    ('GOAL_CAMERA_90_FORWARD_2', 'goal_camera_90_forward_4'),
    ('GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    ('GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
])
def test_goal_duration_equals_playback(action, alias):
    assert Node.GOAL_FORWARD_DURATION_SEC[action] == pytest.approx(seconds(alias))


@pytest.mark.parametrize('alias', list(Node.BALL_RECOVERY_DURATION_SEC))
def test_ball_recovery_duration_equals_playback(alias):
    assert Node.BALL_RECOVERY_DURATION_SEC[alias] == pytest.approx(seconds(alias))


@pytest.mark.parametrize('phase,action,alias', [
    ('LINE_TRACK_AFTER_PICKUP', 'LINE_SPARSE_FORWARD', 'line_forward_2'),
    ('POST_SHOT_LINE_ALIGN', 'POST_SHOT_FORWARD', 'line_forward_6'),
])
def test_special_forward_capture_uses_original_speed(phase, action, alias):
    node = LiveInputHarness(phase=phase)
    Node._start_line_motion_capture(node, action, alias)
    assert node.active_line_motion_duration_sec == pytest.approx(seconds(alias))


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
def test_forward_reserves_once_and_unlocks_at_next_actual_boundary(clock, phase):
    node, first = start_forward(phase)
    assert node.active_line_motion_duration_sec == pytest.approx(seconds('line_forward_6'))
    started = node.active_line_motion_started_at
    capture(node, clock)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] != first['command_id']
    assert clock[0] < started + seconds('line_forward_6')
    count = len(node.publisher.messages)
    capture(node, clock)
    assert len(node.publisher.messages) == count
    clock[0] = started + seconds('line_forward_6')
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert node.queued_general_command_id is None
