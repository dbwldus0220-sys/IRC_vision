"""Replay two-cycle hurdle walking, fresh-input gates, and the final reservation."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_hurdle_positioning import HurdleHarness, hurdle
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status
from test_pickup_continuous_walk import forward_status

ROOT = Path(__file__).resolve().parents[3]
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
MOTIONS = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}


def duration(motion_id):
    motion = MOTIONS[ALIASES[motion_id]]
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']


def observe(node, clock, depth=.60, delay=0., **updates):
    node.receive({**hurdle(depth), 'rgb_stamp_ns': round((clock[0] - delay) * 1e9), **updates})


def start(clock):
    node = HurdleHarness()
    node.pending_hurdle_final_sequence = None
    node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC = Node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC
    observe(node, clock)
    command = node.publish()[-1]
    assert command['action'] == 'STRAIGHT_1' and command['source'] == 'hurdle'
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'line_forward_4'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'line_forward_2'
    forward_status(node, bridge, request, 'RUNNING')
    return node, bridge, request, clock[0]


def finish_walk(node, bridge, request, clock, started):
    clock[0] = started + duration(request['motion_id'])
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    clock[0] += .499
    assert not any(c['valid'] for c in node.publish())
    clock[0] += .001


@pytest.mark.parametrize('delay', [0., .10, .25])
@pytest.mark.parametrize('depth', [.450001, .47, .482, .50, .53])
def test_forward2_reserves_once_and_chains_without_settle_or_input_reset(clock, delay, depth):
    node, bridge, request, started = start(clock)
    for fraction in (.51, .61, .71):
        clock[0] = started + duration('line_forward_2') * fraction + delay
        observe(node, clock, depth, delay)
        assert node.pending_hurdle_final_sequence is None
        assert node.publish() == []
        if fraction < .71:
            assert getattr(node, 'queued_general_command_id', None) is None
    command = node.publisher.messages[-1]
    assert command['source_command']['hurdle_forward_prequeue']
    assert node.queued_general_command_id == command['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    assert second['motion_id'] == 'line_forward_2'
    assert not bridge.queued_request_deferred
    count = len(node.publisher.messages)
    observe(node, clock, depth, delay)
    assert len(node.publisher.messages) == count
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.latest_info['hurdle'] is not None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert bridge.transition_dwell_until is None
    forward_status(node, bridge, second, 'RUNNING')
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert node.approach_walk_started_at == clock[0]
    assert node.approach_walk_capture_count == 0


def test_insufficient_late_frames_use_latest_valid_input_at_completion(clock):
    node, bridge, request, started = start(clock)
    for fraction in (.82, .92):
        clock[0] = started + duration('line_forward_2') * fraction
        observe(node, clock, .53, .35)
        assert getattr(node, 'queued_general_command_id', None) is None
    count = len(node.publisher.messages)
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.latest_info['hurdle'] is not None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert len(node.publisher.messages) == count + 1
    command = node.publisher.messages[-1]
    assert command['action'] == 'STRAIGHT_1' and command['valid']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == ['line_forward_2'] * 2


@pytest.mark.parametrize('depth', [.60, .50, .47, .450001])
def test_camera_down_frame_resumes_without_half_second_or_periodic_tick(clock, depth):
    from step.hurdle_camera_pose import HurdleCameraPose
    from test_boundary_decision_reservation import BoundaryHarness

    node, bridge, request, started = start(clock)
    node.boundary_decision_enabled = True
    node.create_timer = BoundaryHarness.create_timer.__get__(node)
    node.destroy_timer = BoundaryHarness.destroy_timer.__get__(node)
    node.hurdle_camera_pose = HurdleCameraPose()
    heartbeat = {'sequence': 1, 'auto_ready': True, 'active': True,
                 'hurdle_head_override_active': True,
                 'hurdle_head_override_current_deg': -50.}
    clock[0] = started + .8
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))

    def frame(delay=0.):
        stamp = round((clock[0] - delay) * 1e9)
        observe(node, clock, depth, delay,
                stamp={'sec': stamp // 1_000_000_000, 'nanosec': stamp % 1_000_000_000},
                hurdle_angle_45_deg=30., hurdle_angle_0_deg=0.)

    frame()
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    count = sum(c['valid'] for c in node.publisher.messages)
    heartbeat.update(sequence=2, active=False, hurdle_head_override_current_deg=-64.)
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))
    clock[0] += .01
    frame(.1)  # Arrived now, but was photographed before the head settled.
    assert sum(c['valid'] for c in node.publisher.messages) == count
    clock[0] += .01
    frame()  # The image callback itself must dispatch; do not call publish().
    assert sum(c['valid'] for c in node.publisher.messages) == count + 1
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == 'STRAIGHT_1'
    assert node.general_motion_gate.active_command_id == command['command_id']


@pytest.mark.parametrize('invalid', ['expired', 'future', 'unconfirmed', 'depth', 'camera'])
def test_completion_cannot_repeat_forward_from_invalid_latest_input(clock, invalid):
    node, bridge, request, started = start(clock)
    clock[0] = started + .9
    updates = {'confirmation_confirmed': False} if invalid == 'unconfirmed' else (
        {'depth_valid': False} if invalid == 'depth' else (
            {'hurdle_camera_ready': False} if invalid == 'camera' else {}))
    observe(node, clock, .53, **updates)
    if invalid == 'future':
        node.latest_info['hurdle']['rgb_stamp_ns'] = round((clock[0] + 2) * 1e9)
    clock[0] = started + duration('line_forward_2') + (1 if invalid == 'expired' else 0)
    count = len(node.publisher.messages)
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not any(c['valid'] for c in node.publisher.messages[count:])
    assert not node.general_motion_gate.locked


@pytest.mark.parametrize('changes', [
    {'detected': False, 'raw_detected': False}, {'confirmation_confirmed': False},
    {'confidence': .1}, {'depth_valid': False}, {'depth_m': .45},
    {'hurdle_angle_45_deg': 30.}, {'hurdle_camera_ready': False},
    {'hurdle_angle_45_deg': None}, {'depth_age_sec': 1.},
])
def test_late_invalid_turn_camera_or_final_checkpoint_does_not_reserve_forward(clock, changes):
    node, bridge, request, started = start(clock)
    for fraction in (.51, .61, .71, .81):
        clock[0] = started + duration('line_forward_2') * fraction
        observe(node, clock, **changes)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('bad', ['duplicate', 'before_capture_window'])
def test_three_distinct_frames_from_current_motion_are_required(clock, bad):
    node, _, _, started = start(clock)
    for fraction in (.8, .85, .9):
        clock[0] = started + duration('line_forward_2') * fraction
        stamp = started + duration('line_forward_2') * (
            .75 if bad == 'duplicate' else -.01)
        observe(node, clock, delay=clock[0] - stamp)
    assert getattr(node, 'queued_general_command_id', None) is None


def reserve_walk(node, bridge, clock, started):
    for fraction in (.51, .61, .71):
        clock[0] = started + duration('line_forward_2') * fraction
        observe(node, clock)
    command = node.publisher.messages[-1]
    assert command['source_command']['hurdle_forward_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    return decoded_messages(bridge.executor_request_publisher)[-1]


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_running_walk_clears_next_walk_and_latched_final_sequence(clock, status):
    node, bridge, first, started = start(clock)
    reserve_walk(node, bridge, clock, started)
    clock[0] += .05
    observe(node, clock, .44)
    assert node.pending_hurdle_final_sequence is not None
    forward_status(node, bridge, first, status)
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert node.pending_hurdle_final_sequence is None


@pytest.mark.parametrize('depth', [.45, .449999, .44])
def test_rejected_next_walk_keeps_final_checkpoint_and_waits_for_current_walk(clock, depth):
    node, bridge, first, started = start(clock)
    second = reserve_walk(node, bridge, clock, started)
    clock[0] += .05
    observe(node, clock, depth)
    forward_status(node, bridge, second, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert node.queued_general_command_id is None
    assert node.pending_hurdle_final_sequence is not None
    assert node.general_motion_gate.locked
    finish_walk(node, bridge, first, clock, started)
    assert node.publish()[-1]['action'] == 'GO'


@pytest.mark.parametrize('depth', [.45, .449999, .44])
def test_final_crossing_after_commit_waits_for_accepted_walk_and_cannot_queue_a_third(clock, depth):
    node, bridge, first, started = start(clock)
    second = reserve_walk(node, bridge, clock, started)
    clock[0] += .05
    observe(node, clock, depth)
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert node.publish() == []  # Wait for the reserved RUNNING status, not GO.
    forward_status(node, bridge, second, 'RUNNING')
    second_started = clock[0]
    for fraction in (.51, .61, .71):
        clock[0] = second_started + duration('line_forward_2') * fraction
        observe(node, clock, .6)  # A rebound does not undo the latched checkpoint.
        assert node.publish() == []
    assert node.queued_general_command_id is None
    assert len(bridge.executor_request_publisher.messages) == 2
    finish_walk(node, bridge, second, clock, second_started)
    assert node.publish()[-1]['action'] == 'GO'


def test_walking_reservation_uses_posture_matched_captures_after_head_transition(clock):
    from step.hurdle_camera_pose import HurdleCameraPose

    node, bridge, _, started = start(clock)
    node.hurdle_camera_pose = HurdleCameraPose()
    heartbeat = {'sequence': 1, 'auto_ready': True, 'active': True,
                 'hurdle_head_override_active': True,
                 'hurdle_head_override_current_deg': -50.}
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))

    def frame(fraction, delay=0.):
        clock[0] = started + duration('line_forward_2') * fraction
        stamp = round((clock[0] - delay) * 1e9)
        observe(node, clock, delay=delay,
                stamp={'sec': stamp // 1_000_000_000, 'nanosec': stamp % 1_000_000_000},
                hurdle_angle_45_deg=30., hurdle_angle_0_deg=0.)

    for fraction in (.51, .56, .61):
        frame(fraction)
    assert getattr(node, 'queued_general_command_id', None) is None
    heartbeat.update(sequence=2, hurdle_head_override_current_deg=-64.)
    Node._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))
    frame(.65, delay=.1)  # Received after settling but captured during neck movement.
    assert node.approach_walk_capture_count == 0
    for fraction in (.71, .81, .91):
        frame(fraction)
    command = node.publisher.messages[-1]
    assert command['source_command']['hurdle_camera_pose'] == '0'
    assert command['source_command']['hurdle_forward_prequeue']
    assert node.queued_general_command_id == command['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert not bridge.queued_request_deferred


def test_three_successive_gaits_refresh_evidence_and_ignore_duplicate_completion(clock):
    node, bridge, current, started = start(clock)
    for _ in range(2):
        next_request = reserve_walk(node, bridge, clock, started)
        clock[0] = started + duration('line_forward_2')
        forward_status(node, bridge, current, 'SUCCEEDED')
        forward_status(node, bridge, next_request, 'RUNNING')
        forward_status(node, bridge, current, 'SUCCEEDED')
        assert node.general_motion_gate.active_command_id == next_request['command_id']
        assert node.approach_walk_capture_count == 0
        assert getattr(node, 'correction_post_motion_dwell_until', None) is None
        current, started = next_request, clock[0]
    clock[0] = started + .8
    observe(node, clock, .44)
    assert node.pending_hurdle_final_sequence is not None
    assert node.queued_general_command_id is None
    finish_walk(node, bridge, current, clock, started)
    assert node.publish()[-1]['action'] == 'GO'
    assert len(bridge.executor_request_publisher.messages) == 3


def test_two_up_frames_cannot_combine_with_one_down_frame_to_commit(clock):
    node, _, _, started = start(clock)
    for fraction in (.51, .61):
        clock[0] = started + duration('line_forward_2') * fraction
        observe(node, clock)
    assert node.approach_walk_capture_count == 2
    for fraction in (.71, .81, .91):
        clock[0] = started + duration('line_forward_2') * fraction
        observe(node, clock, hurdle_camera_pose='0')
        if fraction < .91:
            assert getattr(node, 'queued_general_command_id', None) is None
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('depth', [.450, .449999, .449, .40])
@pytest.mark.parametrize('delay', [0., .20, .35])
def test_crossing_reserves_final_sequence_once_even_when_hurdle_disappears(clock, depth, delay):
    node, bridge, request, started = start(clock)
    clock[0] = started + duration('line_forward_2') * .8
    observe(node, clock, depth, delay)
    assert node.pending_hurdle_final_sequence['observed_during_action'] == 'STRAIGHT_1'
    observe(node, clock, detected=False, raw_detected=False)
    assert node.publish() == []
    assert getattr(node, 'queued_general_command_id', None) is None
    finish_walk(node, bridge, request, clock, started)
    go = node.publish()[-1]
    assert go['action'] == 'GO' and go['source_command']['hurdle_final_sequence_latched']
    bridge.navigation_command_callback(String(data=json.dumps(go)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_fine_prepare'
    # Preparation is required when the final fine gait follows ordinary walking.
    forwarded = len(bridge.motion_status_publisher.messages)
    for _ in range(20):
        if not bridge.motion_in_progress:
            break
        deadline = bridge.transition_dwell_until or bridge.active_dwell_until
        if deadline is not None:
            clock[0] = max(clock[0], deadline)
            bridge._check_atomic_dwell()
        else:
            current = decoded_messages(bridge.executor_request_publisher)[-1]
            bridge.executor_status_callback(executor_status(**current, status='RUNNING'))
            clock[0] += duration(current['motion_id'])
            bridge.executor_status_callback(executor_status(**current, status='SUCCEEDED'))
        for message in bridge.motion_status_publisher.messages[forwarded:]:
            Node._motion_status_callback(node, message)
        forwarded = len(bridge.motion_status_publisher.messages)
    assert not bridge.motion_in_progress
    assert node.mission_phase == 'LINE_TRACK' and node.phase_manager.hurdles_completed == 1
    ids = [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)]
    assert ids == ['line_forward_2', 'pickup_fine_prepare', 'hurdle_fine_forward_10', 'hurdle']
    assert node.pending_hurdle_final_sequence is None
    assert node.active_special_command_id is None


@pytest.mark.parametrize('updates', [
    {'detected': False, 'raw_detected': True, 'confirmation_confirmed': False},
    {'detected': False, 'raw_detected': False},
    {'depth_valid': False}, {'depth_m': None}, {'depth_m': float('nan')},
])
def test_missing_invalid_or_unconfirmed_input_holds_then_recovers(clock, updates):
    node, bridge, request, started = start(clock)
    clock[0] = started + .8
    observe(node, clock, .44, **updates)
    assert node.pending_hurdle_final_sequence is None
    finish_walk(node, bridge, request, clock, started)
    observe(node, clock, .44, **updates)
    assert not any(command['valid'] for command in node.publish())
    observe(node, clock, .53)
    assert node.publish()[-1]['action'] == 'STRAIGHT_1'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_forward_discards_final_reservation_and_never_jumps_on_old_input(clock, status):
    node, bridge, request, started = start(clock)
    clock[0] = started + .8
    observe(node, clock, .44)
    assert node.pending_hurdle_final_sequence is not None
    forward_status(node, bridge, request, status, error_code='TEST_FAILURE')
    assert node.pending_hurdle_final_sequence is None
    observe(node, clock, detected=False, raw_detected=False)
    clock[0] += 1.
    assert not any(c['valid'] and c['action'] == 'GO' for c in node.publish())
    assert node.phase_manager.hurdles_completed == 0


def test_forward2_catalog_matches_existing_normal_walk():
    assert ALIASES['line_forward_2'] == 'p전진45도(2회)'
    motion = MOTIONS[ALIASES['line_forward_2']]
    assert motion['repeat_count'] == 2
    assert motion['playback_speed'] == 1.0
    assert duration('line_forward_2') == pytest.approx(1.028)
