"""Use later observations for another recovery without adding a stopped dwell."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def start_recovery(node, clock, sign):
    info = line_info(heading=sign * 30., offset=sign * .3)
    fresh(node, clock, 'line', info)
    first = node.publisher.messages[-1]
    assert first['action'].startswith('RECOVER_')
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    # Populate the rolling window, without reserving at the old 70% boundary.
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .70
    for _ in range(16):
        fresh(node, clock, 'line', info, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    return first


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
@pytest.mark.parametrize('aligned', [False, True])
def test_late_frame_selects_recovery_or_forward_before_completion(clock, sign, phase, aligned):
    node = LiveInputHarness(phase=phase)
    node.post_ball_line_run_until = clock[0] + 30.
    first = start_recovery(node, clock, sign)
    end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    clock[0] = end - Node.LINE_RECOVERY_PREQUEUE_LEAD_SEC
    info = line_info(heading=sign * (7.78 if aligned else 22.03), offset=sign * .19)
    fresh(node, clock, 'line', info, plan=False)
    queued = node.publisher.messages[-1]
    assert clock[0] < end
    assert queued['command_id'] != first['command_id']
    assert queued['action'] == ('STRAIGHT' if aligned else first['action'])
    assert node.queued_general_command_id == queued['command_id']
    # The accepted queue remains single-slot and promotes without a new pause.
    fresh(node, clock, 'line', info, plan=False)
    assert node.queued_general_command_id == queued['command_id']
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert node.pre_motion_settle_started_at is None


@pytest.mark.parametrize('bad', ['old_capture', 'future_capture', 'missing_capture', 'low_quality', 'lost', 'competing_mission'])
def test_late_queue_does_not_reuse_early_or_unusable_input(clock, bad):
    node = LiveInputHarness(phase='AUTO')
    start_recovery(node, clock, -1)
    start = node.active_line_motion_started_at
    duration = node.active_line_motion_duration_sec
    clock[0] = start + duration - Node.LINE_RECOVERY_PREQUEUE_LEAD_SEC
    info = line_info(heading=-30., offset=-.3)
    stamp = round((clock[0] + .01) * 1e9)
    if bad == 'old_capture':
        # Still inside the general 0.5s freshness limit, but before the late window.
        stamp = round((start + duration * .69) * 1e9)
    elif bad == 'future_capture':
        stamp += 1_000_000_000
    elif bad == 'missing_capture':
        stamp = None
    elif bad == 'low_quality':
        info['heading_quality'] = .1
    elif bad == 'lost':
        info['detected'] = False
    elif bad == 'competing_mission':
        fresh(node, clock, 'finish', {'detected': True}, plan=False)
    observe(node, clock, 'line', {**info, 'rgb_stamp_ns': stamp}, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None


def test_recovery_to_forward_can_still_queue_in_original_window(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    start_recovery(node, clock, -1)
    fresh(node, clock, 'line', line_info(), plan=False)
    end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    assert clock[0] < end - Node.LINE_RECOVERY_PREQUEUE_LEAD_SEC
    assert node.queued_general_action == 'STRAIGHT'
