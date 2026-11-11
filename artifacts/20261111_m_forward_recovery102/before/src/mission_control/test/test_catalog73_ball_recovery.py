"""Keep Ball-specific recovery data, routing and reservations isolated."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import (
    ground_ball_info, node_with_entry, send, start_line, status,
)
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261106_catalog73_ball_recovery'
NEW_IDS = {1: 'line_recovery_ball_first_left_4', 2: 'line_recovery_ball_second_right_4'}


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_two_motions_added_and_source_preserved(kind):
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    source = json.loads((UPDATE / 'source_requested.json').read_text())['motions']
    entries = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    catalog = {m['name']: m for m in entries}
    unchanged = manifest['catalogs'][kind]['unchanged_motion_sha256']
    assert len(entries) == len(catalog) == len(unchanged) + 2
    for name, digest in unchanged.items():
        assert hashlib.sha256(json.dumps(catalog[name], ensure_ascii=False, sort_keys=True).encode()).hexdigest() == digest
    for motion in source:
        expected = copy.deepcopy(motion)
        # Source73 reused the startup pose name for different joint targets.
        renames = manifest['imported_pose_renames']
        for frame in expected['frames']:
            frame['name'] = renames.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = renames.get(expected[key], expected[key])
        if kind == 'runtime':
            expected['playback_speed'] *= .95
            expected['completion']['position_tolerance_deg'] = 5.0
        assert catalog[motion['name']] == expected


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_ball_recovery_end_pose_does_not_ambiguate_startup(kind):
    motions = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    startup = [frame['angles'] for motion in motions for frame in motion['frames']
               if frame['name'] == '오뒤무게중심앞']
    assert startup and all(angles == startup[0] for angles in startup)
    for motion in motions:
        if motion['name'] in {'t라인복귀좌공용', 't라인복귀우공용'}:
            assert motion['end_pose'] == motion['frames'][-1]['name'] == '오뒤무게중심앞(공용복귀)'
            assert motion['frames'][-1]['angles'] != startup[0]


def test_existing_aliases_unchanged_and_capture_duration_matches_runtime():
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    before = manifest['unchanged_aliases']
    after = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert {key: after[key] for key in before} == before
    alignment_ids = {key for key in after if key.startswith('ball_align_')}
    assert set(after) - set(before) == set(NEW_IDS.values()) | alignment_ids
    catalog = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for motion_id in NEW_IDS.values():
        motion = catalog[after[motion_id]]
        duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000
        assert Node.BALL_RECOVERY_DURATION_SEC[motion_id] == pytest.approx(
            duration * motion['repeat_count'] / motion['playback_speed'])


@pytest.mark.parametrize('index', [1, 2, None, 3, True])
@pytest.mark.parametrize('phase,source', [
    ('BALL_APPROACH', 'ball'), ('BALL_APPROACH', 'line'), ('LINE_TRACK', 'line'),
    ('POST_BALL_LINE_ALIGN', 'ball'), ('LINE_TRACK_AFTER_PICKUP', 'line'),
    ('GOAL_APPROACH', 'goal'), ('HURDLE_APPROACH', 'hurdle'),
])
@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
def test_only_requested_ball_mode_and_direction_remapped(index, phase, source, direction):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        phase=phase, source=source, action=f'RECOVER_LEFT_TURN_{direction}_4',
        mission_progress={'active_ball_mode_index': index, 'pickups_completed': 0}))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    expected = f'line_recovery_{direction.lower()}_4'
    if phase == 'BALL_APPROACH' and source == 'ball' and type(index) is int:
        if (index, direction) in {(1, 'LEFT'), (2, 'RIGHT')}:
            expected = NEW_IDS[index]
    assert request['motion_id'] == expected


@pytest.mark.parametrize('index,angle', [(1, -27.19), (2, 19.46)])
def test_forward_recovery_repeat_forward_reservations_run_without_dwell(clock, index, angle):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    _, running = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=1.2, ground_steering_angle_deg=angle), plan=False)
    for target_angle, expected_id in [(angle, NEW_IDS[index]), (angle, NEW_IDS[index]), (0., 'line_forward_4')]:
        start = node.active_line_motion_started_at
        duration = node.active_line_motion_duration_sec
        clock[0] = start + duration - .25
        for _ in range(6):
            fresh(node, clock, 'ball', ground_ball_info(depth_m=1.2, ground_steering_angle_deg=target_angle), plan=False)
        command = node.publisher.messages[-1]
        assert node.queued_general_command_id == command['command_id']
        queued = send(bridge, command)
        assert queued['motion_id'] == expected_id
        assert not bridge.queued_request_deferred
        assert clock[0] < start + duration
        status(node, bridge, queued, 'QUEUED')
        status(node, bridge, running, 'SUCCEEDED')
        status(node, bridge, queued, 'RUNNING')
        assert node.general_motion_gate.active_command_id == command['command_id']
        assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
        assert getattr(node, 'correction_post_motion_dwell_until', None) is None
        if expected_id in NEW_IDS.values():
            assert node.active_line_motion_duration_sec == pytest.approx(Node.BALL_RECOVERY_DURATION_SEC[expected_id])
            assert node.approach_walk_duration_sec == pytest.approx(Node.BALL_RECOVERY_DURATION_SEC[expected_id])
        running = queued
