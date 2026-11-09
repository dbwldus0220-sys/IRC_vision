"""Replay expanded capture windows and newly covered boundaries without motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS
from mission_control.ball_fine_approach import duration_sec
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info
from test_post_ball_goal_handoff import receive
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import ball, start, forward_status, reserved_request, finish_rest
from test_pickup_crab_observation import start_fine_crab
from test_ball_entry_line_alignment import node_with_entry, start_line
from test_ball_fine_alignment_queue import ball as approach_ball


def deliver(node, bridge, command):
    before = len(bridge.motion_status_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)


def frames(node, clock, started, duration, sample, fractions=(.61, .66, .76), source='ball'):
    for fraction in fractions:
        clock[0] = started + duration * fraction
        receive(node, clock, source, sample)


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('offset', [-250, 250])
def test_two_cycle_crab_repeats_rest_and_preserve_parent(clock, index, offset):
    node, bridge, parent, crab = start_fine_crab(clock, index, offset)
    assert crab['motion_id'].endswith('_2')
    reservation = node.pickup_walk_reservation
    started = reservation.started_at
    duration = PICKUP_WALK_DURATIONS[crab['motion_id']]
    frames(node, clock, started, duration, ball(offset_x_px=offset, bottom_distance_px=250), (.61, .65, .69))
    assert reservation.count == 3 and not reservation.pending
    frames(node, clock, started, duration, ball(offset_x_px=offset, bottom_distance_px=250), (.76,))
    command = node.publisher.messages[-1]
    assert command['source_command']['pickup_walk_prequeue']
    assert command['source_command']['pickup_queue_motion_id'] == crab['motion_id']
    deliver(node, bridge, command)
    queued = reserved_request(bridge)
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    finish_rest(bridge, clock)
    assert bridge.active_request_id == queued['request_id']
    assert node.active_special_command_id == parent['command_id']
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('offset', [-250, 250])
def test_two_cycle_to_one_cycle_uses_checkpoint_not_wrong_two_cycle_queue(clock, offset):
    node, bridge, _, crab = start_fine_crab(clock, offset=offset)
    observer = node.pickup_checkpoint_observation
    frames(node, clock, observer.started_at, observer.duration,
           ball(offset_x_px=offset / 2, bottom_distance_px=250))
    assert not node.pickup_walk_reservation.pending
    forward_status(node, bridge, crab, 'SUCCEEDED')
    command = node.publisher.messages[-1]
    assert command['action'] == ('BALL_PICKUP_CRAB_LEFT' if offset < 0 else 'BALL_PICKUP_CRAB_RIGHT')
    deliver(node, bridge, command)
    finish_rest(bridge, clock)
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == crab['motion_id'][:-2]


def test_changed_latest_sample_cancels_early_same_gait_candidate(clock):
    node, bridge, _, crab = start_fine_crab(clock)
    observer = node.pickup_checkpoint_observation
    started, duration = observer.started_at, observer.duration
    frames(node, clock, started, duration, ball(offset_x_px=-100, bottom_distance_px=250), (.61, .65, .69))
    assert not node.pickup_walk_reservation.pending
    frames(node, clock, started, duration, ball(bottom_distance_px=250), (.76,))
    assert not node.pickup_walk_reservation.pending
    forward_status(node, bridge, crab, 'SUCCEEDED')
    assert node.pickup_fine_align_waiting  # One changed sample cannot authorize a new action.
    fresh(node, clock, 'ball', ball(bottom_distance_px=250))
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


@pytest.mark.parametrize('bottom,offset,action', [
    (250, -100, 'BALL_PICKUP_CRAB_LEFT'), (250, 100, 'BALL_PICKUP_CRAB_RIGHT'),
    (230, 0, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
])
def test_first_fine_prepares_changed_action_before_completion(clock, bottom, offset, action):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    _, fine = start(node, bridge, clock)
    observer = node.pickup_checkpoint_observation
    frames(node, clock, observer.started_at, observer.duration, ball(bottom_distance_px=bottom, offset_x_px=offset), (.61, .66, .9))
    assert observer.sample is not None and not node.pickup_walk_reservation.pending
    assert node.publisher.messages[-1]['action'] != action
    forward_status(node, bridge, fine, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == action
    deliver(node, bridge, node.publisher.messages[-1])
    assert (bridge.active_dwell_until or bridge.transition_dwell_until) is not None


def test_short_turn_collects_early_but_commits_only_with_late_image(clock):
    node = node_with_entry()
    node.general_motion_gate.on_command_published('BALL_APPROACH_TURN_LEFT_1', 1)
    node.active_general_source = 'ball'
    node.ball_fine_running_id = 1
    node.ball_fine_started_at = clock[0]
    node.ball_fine_duration_sec = duration_sec('ball_align_turn_left_1')
    node.ball_fine_support = 'standard'
    started = clock[0]
    frames(node, clock, started, node.ball_fine_duration_sec, approach_ball(), (.61, .71, .79))
    assert node.queued_general_command_id is None
    frames(node, clock, started, node.ball_fine_duration_sec, approach_ball(), (.89,))
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('fps,lag,expected', [(30, 0., True), (30, .1, False), (15, 0., False), (15, .1, False)])
def test_short_turn_with_realistic_image_period_and_delivery_delay(clock, fps, lag, expected):
    node = node_with_entry()
    node.general_motion_gate.on_command_published('BALL_APPROACH_TURN_LEFT_1', 1)
    node.active_general_source = 'ball'
    node.ball_fine_running_id = 1
    node.ball_fine_started_at = clock[0]
    node.ball_fine_duration_sec = duration_sec('ball_align_turn_left_1')
    node.ball_fine_support = 'standard'
    started, duration = clock[0], node.ball_fine_duration_sec
    for i in range(1, int(duration * fps) + 1):
        captured = started + i / fps
        clock[0] = captured + lag
        if clock[0] >= started + duration:
            break
        Node._info_callback(node, 'ball')(String(data=json.dumps({**approach_ball(), 'rgb_stamp_ns': round(captured * 1e9)})))
    assert (node.queued_general_command_id is not None) is expected


def test_post_shot_conditional_turn_retains_half_second_settle(clock):
    from test_line_offset_alignment import sample
    node, bridge, forward = start_post_shot_forward(clock, 1)
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    sample = sample(target_angle=30., ground_heading_error_deg=-1.43,
                    filtered_lateral_offset_norm=.262, ground_lateral_offset_m=.1294,
                    ground_lookahead_distance_m=.211, ground_line_points_m=[[.13,z] for z in (.15,.25,.4)])
    frames(node, clock, node.active_line_motion_started_at, node.active_line_motion_duration_sec,
           sample, (.8, .84, .9), source='line')
    assert node.queued_general_command_id is None
    preview = Node._post_shot_exit_preview(node, clock[0])
    assert preview is not None and preview.action.startswith('LINE_OFFSET_TURN_')
    forward_status(node, bridge, forward, 'SUCCEEDED')
    assert node.pre_motion_settle_deadline == pytest.approx(clock[0] + .5)
    completed = clock[0]
    clock[0] = completed + .48
    fresh(node, clock, 'line', sample)
    assert node.publisher.messages[-1]['action'] != preview.action
    clock[0] = completed + .5
    fresh(node, clock, 'line', sample)
    assert node.publisher.messages[-1]['action'] == preview.action


@pytest.mark.parametrize('index', [1, 2])
def test_approach_prepares_pickup_and_initial_alignment_without_new_image(clock, index):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    _, walking = start_line(node, bridge, clock)
    observer = node.pickup_entry_observation
    sample = ball(bottom_distance_px=340)
    frames(node, clock, observer.started_at, observer.duration, sample, (.8, .84, .9))
    assert observer.sample is not None and node.ball_pickup_entry_pending
    assert node.active_special_command_id is None
    forward_status(node, bridge, walking, 'SUCCEEDED')
    parent = node.publisher.messages[-1]
    assert parent['action'] == 'PICKUP_NOW'
    deliver(node, bridge, parent)
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'
    assert not node.pickup_initial_align_waiting


@pytest.mark.parametrize('index', [1, 2])
def test_camera45_fine_approach_hands_off_to_pickup_without_new_image(clock, index):
    from test_ball_fine_alignment_queue import start_fine_prepare
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    prepare = start_fine_prepare(node, bridge, clock)
    clock[0] += .04
    fresh(node, clock, 'ball', approach_ball(), plan=False)
    deliver(node, bridge, node.publisher.messages[-1])
    assert bridge.queued_request_deferred
    forward_status(node, bridge, prepare, 'SUCCEEDED')
    finish_rest(bridge, clock)
    fine = decoded_messages(bridge.executor_request_publisher)[-1]
    assert fine['motion_id'] == 'ball_align_fine_forward_45_5'
    forward_status(node, bridge, fine, 'RUNNING')
    observer = node.pickup_entry_observation
    frames(node, clock, observer.started_at, observer.duration,
           {**approach_ball(distance=.49), 'bottom_distance_px': 340}, (.8, .84, .9))
    assert observer.sample is not None
    forward_status(node, bridge, fine, 'SUCCEEDED')
    parent = node.publisher.messages[-1]
    assert parent['action'] == 'PICKUP_NOW'
    deliver(node, bridge, parent)
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'


def test_phase_override_discards_prepared_entry(clock):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    _, walking = start_line(node, bridge, clock)
    observer = node.pickup_entry_observation
    frames(node, clock, observer.started_at, observer.duration, ball(bottom_distance_px=340), (.8, .84, .9))
    Node._phase_callback(node, String(data='HURDLE_APPROACH'))
    assert observer.key is None
    assert node.pickup_entry_ready_sample is None
    assert node.pickup_initial_entry_sample is None


@pytest.mark.parametrize('bad', ['lost', 'changed', 'expired', 'failed'])
def test_pickup_entry_candidate_is_not_used_after_invalid_input_or_failure(clock, bad):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    _, walking = start_line(node, bridge, clock)
    observer = node.pickup_entry_observation
    frames(node, clock, observer.started_at, observer.duration, ball(bottom_distance_px=340), (.8, .84, .9))
    if bad == 'lost':
        fresh(node, clock, 'ball', ball(detected=False, raw_detected=False), plan=False)
    elif bad == 'changed':
        fresh(node, clock, 'ball', ball(distance_m=.8, depth_m=.8), plan=False)
    elif bad == 'expired':
        clock[0] += node.timeouts['ball'] + .01
    forward_status(node, bridge, walking, 'FAILED' if bad == 'failed' else 'SUCCEEDED',
                   error_code='SDK_COMMUNICATION_ERROR' if bad == 'failed' else '')
    assert getattr(node, 'pickup_initial_entry_sample', None) is None
    assert node.pickup_entry_observation.key is None


def start_post_shot_forward(clock, section):
    node = LiveInputHarness(phase='POST_SHOT_FORWARD')
    node.queued_general_command_id = None
    node.queued_general_action = None
    node.queued_general_source = None
    node.phase_manager.ball_sections_processed = section
    bridge = FakeBridge(advance_transition_dwells=False)
    fresh(node, clock, 'line', line_info())
    command = node.publisher.messages[-1]
    assert command['action'] == 'POST_SHOT_FORWARD'
    deliver(node, bridge, command)
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    return node, bridge, request


@pytest.mark.parametrize('section', [1, 2])
def test_post_shot_forward_queues_normal_line_and_promotes(clock, section):
    node, bridge, forward = start_post_shot_forward(clock, section)
    frames(node, clock, node.active_line_motion_started_at, node.active_line_motion_duration_sec,
           line_info(), (.8, .84, .9), source='line')
    command = node.publisher.messages[-1]
    assert command['source_command']['post_shot_exit_prequeue']
    assert command['action'] in {'STRAIGHT', 'STRAIGHT_1', 'STRAIGHT_2'}
    assert node.mission_phase == 'POST_SHOT_FORWARD'
    deliver(node, bridge, command)
    queued = decoded_messages(bridge.executor_request_publisher)[-1]
    assert not bridge.queued_request_deferred
    forward_status(node, bridge, queued, 'QUEUED')
    forward_status(node, bridge, forward, 'SUCCEEDED')
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')
    forward_status(node, bridge, queued, 'RUNNING')
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert node.active_general_source == 'line'


@pytest.mark.parametrize('bad', ['lost', 'duplicate', 'early', 'stale', 'failure', 'rejected'])
def test_post_shot_exit_failure_and_fresh_fallback(clock, bad):
    node, bridge, forward = start_post_shot_forward(clock, 1)
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    if bad in {'lost', 'duplicate', 'early'}:
        fractions = (.8, .8, .8) if bad == 'duplicate' else (.61, .63, .65) if bad == 'early' else (.8, .84, .9)
        sample = {**line_info(), 'detected': False} if bad == 'lost' else line_info()
        frames(node, clock, started, duration, sample, fractions, source='line')
        assert node.queued_general_command_id is None
    elif bad == 'stale':
        clock[0] = started + duration * .9
        for i in range(3):
            Node._info_callback(node, 'line')(String(data=json.dumps({**line_info(), 'rgb_stamp_ns': round((clock[0] - 1 + i*.01)*1e9)})))
        assert node.queued_general_command_id is None
    else:
        frames(node, clock, started, duration, line_info(), (.8, .84, .9), source='line')
        deliver(node, bridge, node.publisher.messages[-1])
        queued = decoded_messages(bridge.executor_request_publisher)[-1]
        if bad == 'rejected':
            forward_status(node, bridge, queued, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
            assert node.queued_general_command_id is None
    forward_status(node, bridge, forward, 'FAILED' if bad == 'failure' else 'SUCCEEDED',
                   error_code='SDK_COMMUNICATION_ERROR' if bad == 'failure' else '')
    if bad == 'failure':
        assert node.queued_general_command_id is None
        assert node.phase_manager.post_shot_failed
    else:
        fresh(node, clock, 'line', line_info())
        assert node.publisher.messages[-1]['action'] != 'POST_SHOT_FORWARD'
