"""Retry unreserved Line walking on fresh input without a timer tick."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def finish_without_queue(clock, *, phase='LINE_TRACK', status='SUCCEEDED'):
    node = LiveInputHarness(phase=phase)
    messages = []
    node.get_logger().info = messages.append
    fresh(node, clock, 'line', line_info(-16., -.2))
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += 2.5  # The last image is now too old for completion dispatch.
    node.send_status(first['action'], first['command_id'], status)
    return node, messages


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
def test_fresh_line_dispatches_once_without_timer(clock, phase):
    node, messages = finish_without_queue(clock, phase=phase)
    before = len(node.publisher.messages)
    assert node.line_boundary_dispatch_pending
    assert any('Line completed without prequeue:' in m and 'fresh_line=False' in m
               for m in messages)
    fresh(node, clock, 'line', line_info(), plan=False)
    assert len(node.publisher.messages) == before + 1
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT'
    assert not node.line_boundary_dispatch_pending
    fresh(node, clock, 'line', line_info(), plan=False)
    Node._publish_decision(node)
    assert len(node.publisher.messages) == before + 1


@pytest.mark.parametrize('bad', ['stale', 'future', 'missing', 'bool_stamp', 'lost', 'quality'])
def test_unusable_image_does_not_dispatch_but_next_good_image_does(clock, bad):
    node, _ = finish_without_queue(clock)
    before = len(node.publisher.messages)
    info = {**line_info(), 'rgb_stamp_ns': round((clock[0] + .01) * 1e9)}
    if bad == 'stale':
        info['rgb_stamp_ns'] -= 1_000_000_000
    elif bad == 'future':
        info['rgb_stamp_ns'] += 1_000_000_000
    elif bad == 'missing':
        del info['rgb_stamp_ns']
    elif bad == 'bool_stamp':
        info['rgb_stamp_ns'] = True
    elif bad == 'lost':
        info['detected'] = False
    else:
        info['heading_quality'] = .1
    observe(node, clock, 'line', info, plan=False)
    assert len(node.publisher.messages) == before
    fresh(node, clock, 'line', line_info(), plan=False)
    assert len(node.publisher.messages) == before + 1


@pytest.mark.parametrize('blocked', ['safety', 'executor', 'special', 'running', 'queued', 'dwell'])
def test_fresh_dispatch_preserves_existing_guards(clock, blocked):
    node, _ = finish_without_queue(clock)
    before = len(node.publisher.messages)
    if blocked == 'safety':
        node.safety_interlock.observe_executor_status(
            error_code='SDK_INTERNAL_ERROR', message='test fault',
            action='STRAIGHT', command_id=1, event_id=None)
    elif blocked == 'executor':
        node.executor_auto_ready = False
    elif blocked == 'special':
        node.phase_manager.start_special_action('PICKUP_NOW', 99)
    elif blocked == 'running':
        node.general_motion_gate.on_command_published('STRAIGHT', 99)
    elif blocked == 'queued':
        node.queued_general_command_id = 99
    else:
        node.correction_post_motion_dwell_until = clock[0] + .5
    fresh(node, clock, 'line', line_info(), plan=False)
    assert len(node.publisher.messages) == before


@pytest.mark.parametrize('status', ['FAILED', 'REJECTED', 'TIMEOUT'])
def test_failed_motion_does_not_enable_fresh_dispatch(clock, status):
    node, _ = finish_without_queue(clock, status=status)
    before = len(node.publisher.messages)
    fresh(node, clock, 'line', line_info(), plan=False)
    assert len(node.publisher.messages) == before
    assert not node.line_boundary_dispatch_pending


def test_existing_prequeue_still_owns_next_motion(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    fresh(node, clock, 'line', line_info())
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .71
    for _ in range(10):
        fresh(node, clock, 'line', line_info(), plan=False)
    queued = node.queued_general_command_id
    assert queued is not None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    before = len(node.publisher.messages)
    fresh(node, clock, 'line', line_info(), plan=False)
    assert len(node.publisher.messages) == before
    assert node.queued_general_command_id == queued
    assert not node.line_boundary_dispatch_pending


def test_frame_shortage_is_logged_once_at_completion(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    messages = []
    node.get_logger().info = messages.append
    fresh(node, clock, 'line', line_info(-16., -.2))
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .8
    fresh(node, clock, 'line', line_info(), plan=False)
    assert not any('without prequeue' in m for m in messages)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    logs = [m for m in messages if 'without prequeue' in m]
    assert len(logs) == 1
    assert 'reason=insufficient_usable_frames' in logs[0]
    assert 'usable_frames=1/15' in logs[0]
