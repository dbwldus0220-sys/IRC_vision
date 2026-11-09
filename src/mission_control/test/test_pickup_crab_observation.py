"""Replay crab completion through the real decision and bridge state machines."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_CRAB_DURATIONS
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import ball, start, forward_status


def advance_dwell(node, bridge, clock):
    before = len(bridge.motion_status_publisher.messages)
    deadline = bridge.transition_dwell_until or bridge.active_dwell_until
    assert deadline is not None
    clock[0] = deadline
    bridge._check_transition_dwell(clock[0])
    bridge._check_atomic_dwell(clock[0])
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)


def start_fine_crab(clock, index=1, offset=-100):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    parent, first = start(node, bridge, clock)
    clock[0] += 2.
    forward_status(node, bridge, first, 'SUCCEEDED')
    if bridge.active_dwell_until is not None:
        advance_dwell(node, bridge, clock)
    fresh(node, clock, 'ball', ball(offset_x_px=offset, bottom_distance_px=250))
    correction = node.publisher.messages[-1]
    assert correction['action'] in {'BALL_PICKUP_CRAB_LEFT', 'BALL_PICKUP_CRAB_RIGHT'}
    bridge.navigation_command_callback(String(data=json.dumps(correction)))
    if bridge.transition_dwell_until is not None:
        advance_dwell(node, bridge, clock)
    prepare = decoded_messages(bridge.executor_request_publisher)[-1]
    assert prepare['motion_id'].startswith('pickup_fine_to_crab_')
    forward_status(node, bridge, prepare, 'RUNNING')
    clock[0] += .4
    forward_status(node, bridge, prepare, 'SUCCEEDED')
    advance_dwell(node, bridge, clock)
    crab = decoded_messages(bridge.executor_request_publisher)[-1]
    assert crab['motion_id'] in PICKUP_CRAB_DURATIONS
    forward_status(node, bridge, crab, 'RUNNING')
    assert node.pickup_checkpoint_observation.key[-1] == crab['motion_id']
    return node, bridge, parent, crab


def late_centered_frames(node, clock, count=3):
    observer = node.pickup_checkpoint_observation
    clock[0] = observer.started_at + PICKUP_CRAB_DURATIONS[observer.key[-1]] * .75
    for _ in range(count):
        fresh(node, clock, 'ball', ball(bottom_distance_px=250), plan=False)


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('offset', [-200, -100, 100, 200])
def test_late_crab_frames_continue_only_after_success_and_keep_support_dwell(clock, index, offset):
    node, bridge, parent, crab = start_fine_crab(clock, index, offset)
    before = len(node.publisher.messages)
    late_centered_frames(node, clock)
    sample = node.pickup_checkpoint_observation.sample
    assert sample is not None
    received_at = node.pickup_checkpoint_observation.received_at
    assert len(node.publisher.messages) == before
    assert not node.pickup_walk_reservation.pending and bridge.pickup_queued_request is None
    clock[0] = node.pickup_checkpoint_observation.started_at + PICKUP_CRAB_DURATIONS[crab['motion_id']]
    forward_status(node, bridge, crab, 'SUCCEEDED')
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE' and command['valid']
    assert node.latest_time['ball'] == received_at
    assert node.latest_info['ball']['rgb_stamp_ns'] == sample['rgb_stamp_ns']
    assert node.active_special_command_id == parent['command_id']
    assert node.pickup_checkpoint_observation.key is None
    assert not node.pickup_fine_align_waiting
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    # The cached result selects the existing sequence, including preparation.
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    advance_dwell(node, bridge, clock)
    prepare = decoded_messages(bridge.executor_request_publisher)[-1]
    side = 'left' if offset < 0 else 'right'
    assert prepare['motion_id'] == f'pickup_crab_{side}_to_fine_0'
    forward_status(node, bridge, prepare, 'RUNNING')
    clock[0] += .4
    forward_status(node, bridge, prepare, 'SUCCEEDED')
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    advance_dwell(node, bridge, clock)
    backward = decoded_messages(bridge.executor_request_publisher)[-1]
    assert backward['motion_id'] == 'pickup_pre_backward_camera_down'
    assert bridge.transition_dwell_until is None
    assert bridge.pickup_queued_request is None


@pytest.mark.parametrize('case', ['too_few', 'expired', 'lost', 'changed', 'future', 'duplicate'])
def test_invalid_cached_observations_fall_back_and_fresh_frame_releases_checkpoint(clock, case):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock, 2 if case == 'too_few' else 3)
    if case == 'expired':
        clock[0] += node.timeouts['ball'] + .01
    elif case in {'lost', 'changed'}:
        changes = {'detected': False, 'raw_detected': False} if case == 'lost' else {'offset_x_px': -100}
        fresh(node, clock, 'ball', ball(bottom_distance_px=250, **changes), plan=False)
    elif case in {'future', 'duplicate'}:
        info = dict(node.pickup_checkpoint_observation.sample)
        if case == 'future':
            info['rgb_stamp_ns'] += 10_000_000_000
        Node._info_callback(node, 'ball')(String(data=json.dumps(info)))
    count = len(node.publisher.messages)
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert len(node.publisher.messages) == count
    assert node.latest_info['ball'] is None
    assert node.pickup_fine_align_waiting and bridge.pickup_fine_align_waiting
    assert node.pickup_checkpoint_observation.key is None
    fresh(node, clock, 'ball', ball(bottom_distance_px=250))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert not node.pickup_fine_align_waiting


@pytest.mark.parametrize('field', ['command_id', 'event_id', 'request_id'])
def test_unmatched_success_cannot_release_cached_crab(clock, field):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    count = len(node.publisher.messages)
    if field == 'request_id':
        forward_status(node, bridge, {**crab, field: crab[field] + 100}, 'SUCCEEDED')
    else:
        # Decision ownership is checked on the bridge's checkpoint status.
        status = decoded_messages(bridge.motion_status_publisher)[-1]
        Node._motion_status_callback(node, String(data=json.dumps({
            **status, field: crab[field] + 100, 'motion_id': Node.PICKUP_FINE_ALIGN_MARKER,
            'completed_motion_id': crab['motion_id'],
        })))
    assert len(node.publisher.messages) == count
    assert bridge.active_motion_id == crab['motion_id']
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('status,error', [('FAILED', 'SDK_COMMUNICATION_ERROR'),
                                         ('TIMEOUT', 'TIMEOUT'), ('CANCELLED', '')])
def test_failure_discards_cached_crab(clock, status, error):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    forward_status(node, bridge, crab, status, error_code=error)
    assert node.pickup_checkpoint_observation.key is None
    assert not any(m['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE' for m in node.publisher.messages)
    if status == 'FAILED':
        # Existing recoverable-fault policy returns to a fresh checkpoint.
        assert bridge.pickup_fine_align_waiting
        assert node.latest_info['ball'] is None
        fresh(node, clock, 'ball', ball(bottom_distance_px=230))
        assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    else:
        assert not bridge.motion_in_progress


@pytest.mark.parametrize('field', ['command_id', 'event_id'])
def test_unmatched_raw_completion_cannot_authorize_reuse(clock, field):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    before = len(node.publisher.messages)
    forward_status(node, bridge, {**crab, field: crab[field] + 100}, 'SUCCEEDED')
    assert len(node.publisher.messages) == before
    assert node.latest_info['ball'] is None
    assert node.pickup_checkpoint_observation.key is None


def test_cached_observation_does_not_replace_same_gait_reservation(clock):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    for _ in range(3):
        fresh(node, clock, 'ball', ball(offset_x_px=-100, bottom_distance_px=250), plan=False)
    command = node.publisher.messages[-1]
    assert command['source_command']['pickup_walk_prequeue']
    assert node.pickup_checkpoint_observation.key is None
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    queued = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert bridge.active_request_id == queued['request_id']
    assert not bridge.pickup_fine_align_waiting
    forward_status(node, bridge, queued, 'RUNNING')
    late_centered_frames(node, clock)
    forward_status(node, bridge, queued, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('change', [
    {'depth_valid': False}, {'depth_age_sec': .701}, {'distance_m': .501},
    {'confidence': .1}, {'confirmation_confirmed': False},
    {'bottom_distance_px': 300.001}, {'bottom_distance_px': 99},
])
def test_latest_invalid_or_different_decision_discards_ready_sample(clock, change):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    fresh(node, clock, 'ball', ball(**{'bottom_distance_px': 250, **change}), plan=False)
    assert node.pickup_checkpoint_observation.sample is None
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert node.pickup_fine_align_waiting and node.latest_info['ball'] is None
    fresh(node, clock, 'ball', ball(bottom_distance_px=250))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


def test_early_frames_and_duplicate_running_do_not_create_or_restart_preview(clock):
    node, bridge, _, crab = start_fine_crab(clock)
    for _ in range(3):
        fresh(node, clock, 'ball', ball(bottom_distance_px=250), plan=False)
    assert node.pickup_checkpoint_observation.sample is None
    late_centered_frames(node, clock)
    started_at = node.pickup_checkpoint_observation.started_at
    forward_status(node, bridge, crab, 'RUNNING')
    assert node.pickup_checkpoint_observation.started_at == started_at
    assert node.pickup_checkpoint_observation.sample is not None
    forward_status(node, bridge, crab, 'SUCCEEDED')
    before = len(node.publisher.messages)
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert len(node.publisher.messages) == before


def test_rejected_same_gait_queue_uses_fresh_checkpoint_and_can_resume(clock):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    for _ in range(3):
        fresh(node, clock, 'ball', ball(offset_x_px=-100, bottom_distance_px=250), plan=False)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    queued = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, queued, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert bridge.pickup_queued_request is None
    assert not node.pickup_walk_reservation.pending
    assert node.pickup_fine_align_waiting
    fresh(node, clock, 'ball', ball(bottom_distance_px=250))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


def test_executor_not_ready_does_not_dispatch_cached_sample_and_expired_sample_is_not_reused(clock):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    before = len(node.publisher.messages)
    node.executor_auto_ready = False
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert len(node.publisher.messages) == before
    clock[0] += node.timeouts['ball'] + .01
    node.executor_auto_ready = True
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['action'] != 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    fresh(node, clock, 'ball', ball(bottom_distance_px=250))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


def test_phase_reset_clears_crab_preview(clock):
    node, bridge, _, crab = start_fine_crab(clock)
    late_centered_frames(node, clock)
    Node._clear_ball_navigation_state(node)
    assert node.pickup_checkpoint_observation.key is None
    before = len(node.publisher.messages)
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert len(node.publisher.messages) == before


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('offset', [-250, -100, 100, 250])
def test_initial_crab_reuses_observation_across_both_checkpoints(clock, index, offset):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_crab_left_0' if offset < 0 else 'pickup_crab_right_0'
    _, crab = start(node, bridge, clock, ball(offset_x_px=offset))
    assert node.pickup_checkpoint_observation.stage == 'initial'
    clock[0] += PICKUP_CRAB_DURATIONS[crab['motion_id']] * .75
    for _ in range(3):
        fresh(node, clock, 'ball', ball(bottom_distance_px=250), plan=False)
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
    before = len(bridge.motion_status_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert node.pickup_checkpoint_observation.key is None


def test_checkpoint_observation_durations_match_runtime():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for motion_id, duration in PICKUP_CRAB_DURATIONS.items():
        motion = motions[aliases[motion_id]]
        assert duration == pytest.approx(max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
                                         / 1000 * motion['repeat_count'] / motion['playback_speed'])
