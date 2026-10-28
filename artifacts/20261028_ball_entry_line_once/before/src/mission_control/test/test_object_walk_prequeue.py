"""Replay object-owned walking through real input, decision and bridge callbacks."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_decision_planner import ball_info
from test_hurdle_positioning import hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


def sample(source, angle=0., **updates):
    if source == 'ball':
        return ball_info(depth_m=.9, bearing_deg=angle,
                         confirmation_confirmed=True, **updates)
    return hurdle(.9, **updates)


def feed(node, clock, source, angle=0., **updates):
    if source == 'hurdle':
        # Moving the line/hurdle intersection requests ordinary walking recovery.
        fresh(node, clock, 'line', line_info(x=710 + round(angle * 10)), plan=False)
    fresh(node, clock, source, sample(source, angle, **updates), plan=False)


def start(node, clock, source, angle=0.):
    feed(node, clock, source, angle)
    MotionDecisionNode._publish_decision(node)
    first = node.publisher.messages[-1]
    assert first['valid'] and first['source'] == source, first
    motion = ('line_forward_4' if first['action'] == 'STRAIGHT'
              else MotionCommandBridgeNode.motion_id_for_action(first['action']))
    node.send_status(first['action'], first['command_id'], 'RUNNING', motion)
    assert node.approach_walk_started_at == clock[0]
    return first


@pytest.mark.parametrize('source,phase,angles', [
    ('ball', 'BALL_APPROACH', (0., 0.)),
    *[('hurdle', 'HURDLE_APPROACH', angles) for angles in
      [(0., 0.), (0., 25.), (25., 0.), (25., -25.)]],
])
def test_object_walk_queues_once_and_bridge_promotes_without_dwell(clock, source, phase, angles):
    node, bridge = LiveInputHarness(phase=phase), FakeBridge()
    first = start(node, clock, source, angles[0])
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    for _ in range(3):
        feed(node, clock, source, angles[1])
    assert getattr(node, 'queued_general_command_id', None) is None
    clock[0] = node.approach_walk_started_at + MotionDecisionNode.APPROACH_WALK_DURATION_SEC[first['action']] - .25
    for _ in range(4):
        feed(node, clock, source, angles[1])
    second = node.publisher.messages[-1]
    assert second['command_id'] != first['command_id']
    assert second['action'] == ('STRAIGHT' if angles[1] == 0 else
        f"{'BALL_APPROACH_' if source == 'ball' else ''}RECOVER_{'RIGHT' if angles[1] > 0 else 'LEFT'}"
        + (f"_TURN_{'RIGHT' if angles[1] > 0 else 'LEFT'}" if source == 'hurdle' else '') + '_4')
    assert len([m for m in node.publisher.messages if m['valid']]) == 2
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    assert not bridge.queued_request_deferred
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 2
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    assert bridge.transition_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    bridge.executor_status_callback(executor_status(**requests[-1], status='RUNNING'))
    MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    assert node.general_motion_gate.active_command_id == second['command_id']
    assert node.approach_walk_started_at == clock[0]


@pytest.mark.parametrize('source,phase', [('ball', 'BALL_APPROACH'), ('hurdle', 'HURDLE_APPROACH')])
@pytest.mark.parametrize('blocker', ['near', 'lost', 'depth', 'confidence', 'unconfirmed', 'old', 'duplicate'])
def test_object_queue_requires_fresh_confirmed_far_target(clock, source, phase, blocker):
    node = LiveInputHarness(phase=phase)
    start(node, clock, source)
    clock[0] += 1.8
    updates = {
        'lost': {'detected': False, 'raw_detected': False},
        'depth': {'depth_valid': False}, 'confidence': {'confidence': .1},
        'unconfirmed': {'confirmation_confirmed': False},
    }.get(blocker, {})
    if blocker == 'near':
        updates = {'depth_m': .5, 'distance_m': .5}
    stamp = round(clock[0] * 1e9)
    for _ in range(4):
        info = sample(source)
        info.update(updates)
        if source == 'hurdle':
            fresh(node, clock, 'line', line_info(), plan=False)
        if blocker in {'old', 'duplicate'}:
            from test_wait_refresh_and_fine_settle import observe
            observe(node, clock, source, {**info, 'rgb_stamp_ns': 10_010_000_000 if blocker == 'old' else stamp}, plan=False)
        else:
            fresh(node, clock, source, info, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert not node.planner.ball_terminal_requested
    assert not node.planner.hurdle_go_requested


@pytest.mark.parametrize('source,phase', [('ball', 'BALL_APPROACH'), ('hurdle', 'HURDLE_APPROACH')])
@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'TIMEOUT'])
def test_object_queue_is_discarded_on_failure(clock, source, phase, status):
    node = LiveInputHarness(phase=phase)
    first = start(node, clock, source)
    clock[0] += 1.8
    for _ in range(3):
        feed(node, clock, source)
    assert node.queued_general_command_id is not None
    node.send_status(first['action'], first['command_id'], status)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.approach_walk_started_at is None
    assert not node.general_motion_gate.has_required_fresh_vision()


@pytest.mark.parametrize('source,phase', [('ball', 'BALL_APPROACH'), ('hurdle', 'HURDLE_APPROACH')])
def test_without_reservation_completion_dispatches_recent_walk_immediately(clock, source, phase):
    node = LiveInputHarness(phase=phase)
    first = start(node, clock, source)
    clock[0] += .1
    feed(node, clock, source)
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    second = node.publisher.messages[-1]
    assert second['valid'] and second['action'] == 'STRAIGHT'
    assert second['command_id'] != first['command_id']


@pytest.mark.parametrize('source,phase', [('hurdle', 'HURDLE_APPROACH')])
def test_recovery_to_recovery_waits_until_last_point_three_seconds(clock, source, phase):
    node = LiveInputHarness(phase=phase)
    first = start(node, clock, source, 25.)
    started = node.approach_walk_started_at
    duration = MotionDecisionNode.APPROACH_WALK_DURATION_SEC[first['action']]
    clock[0] = started + duration * .72
    for _ in range(3):
        feed(node, clock, source, -25.)
    assert getattr(node, 'queued_general_command_id', None) is None
    clock[0] = started + duration - .25
    for _ in range(3):
        feed(node, clock, source, -25.)
    assert node.queued_general_command_id is not None


def test_goal_and_object_capture_timing_matches_runtime_motion_data():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for action, duration in {**MotionDecisionNode.APPROACH_WALK_DURATION_SEC,
                             **MotionDecisionNode.GOAL_FORWARD_DURATION_SEC}.items():
        motion_id = 'line_forward_4' if action == 'STRAIGHT' else MotionCommandBridgeNode.motion_id_for_action(action)
        motion = catalog[aliases[motion_id]]
        actual = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert duration == pytest.approx(actual), (action, duration, actual)
