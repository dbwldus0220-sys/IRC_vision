"""Source106 replacement, timing, and completion-driven mixed reservations."""

import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages, navigation_message,
)
from test_motion_decision_node import FakeDecisionNode

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261113_final_forward106'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_six_cycle_forward_is_replaced_with_exact_source(kind):
    path = Path(f'artifacts/robot_motions_{kind}.json')
    before, after = catalog(UPDATE / 'before' / path), catalog(ROOT / path)
    source_path = UPDATE / 'source_robot_motions.json'
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == manifest['source_sha256']
    source = catalog(source_path)['최종전진']
    assert before.keys() - after.keys() == {'m전진45(6회)'}
    assert after.keys() - before.keys() == {'최종전진'}
    for name in before.keys() & after.keys():
        assert after[name] == before[name], name
    assert after['최종전진'] == (runtime_motion_from_source(source) if kind == 'runtime' else source)
    assert source['repeat_count'] == 6 and source['playback_speed'] == 1.0
    assert all(frame['angles']['0'] == -64.0 for frame in source['frames'])


def test_aliases_and_all_six_cycle_capture_estimates_match_runtime():
    path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    before = yaml.safe_load((UPDATE / 'before' / path).read_text())['motion_aliases']
    aliases = yaml.safe_load((ROOT / path).read_text())['motion_aliases']
    expected = dict(before, line_forward_6='최종전진', post_ball_forward_6='최종전진')
    assert aliases == expected
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(aliases.values()) <= motions.keys()
    motion = motions['최종전진']
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * 6 / 1000
    assert duration == pytest.approx(3.864)
    for action in ('STRAIGHT', 'STRAIGHT_3'):
        assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(duration)
    assert Node.APPROACH_WALK_DURATION_SEC['STRAIGHT_3'] == pytest.approx(duration)
    node = FakeDecisionNode()
    Node._start_line_motion_capture(node, 'POST_SHOT_FORWARD')
    assert node.active_line_motion_duration_sec == pytest.approx(duration)


@pytest.mark.parametrize('first,second', [
    ('STRAIGHT_2', 'STRAIGHT_3'), ('STRAIGHT_3', 'STRAIGHT_1'),
    ('LEFT', 'STRAIGHT_3'), ('STRAIGHT_3', 'RIGHT'),
])
@pytest.mark.parametrize('terminal', ['SUCCEEDED', 'FAILED', 'TIMEOUT', 'CANCELLED'])
def test_mixed_reservation_starts_only_after_success(first, second, terminal):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=first))
    bridge.navigation_command_callback(navigation_message(action=second, command_id=8001))
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.queued_request_deferred
    complete_active_motion(bridge, status=terminal)
    requests = decoded_messages(bridge.executor_request_publisher)
    if terminal == 'SUCCEEDED':
        assert len(requests) == 2
        assert requests[-1]['command_id'] == 8001
        assert bridge.active_command_id == 8001
    else:
        assert len(requests) == 1
        assert bridge.queued_request_id is None


def test_final_forward_repeat_keeps_sdk_queue_and_single_pending_request():
    bridge = FakeBridge()
    for command_id in (8000, 8001, 8002):
        bridge.navigation_command_callback(navigation_message(action='STRAIGHT_3', command_id=command_id))
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [
        'line_forward_6', 'line_forward_6',
    ]
    assert not bridge.queued_request_deferred
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'QUEUE_FULL'
