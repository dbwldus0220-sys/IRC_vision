"""Replay transient Ball gaps through planning, reservation and completion."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_approach_observation import BallApproachObservation
from mission_control.motion_decision_node import MotionDecisionNode as Node
from step.ball_navigation_planner import BallNavigationConfig
from test_ball_approach_latency import sample
from test_ball_entry_line_alignment import node_with_entry, start_line, send, status
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge, decoded_messages


def observation(clock, *, confirmed=False, angle=0., distance=1.2):
    info = sample(clock[0], detected=confirmed, distance=distance)
    info['confirmation_confirmed'] = confirmed
    geometry = dict(ground_projection_enabled=True, ground_projection_valid=True,
                    ground_projection_scope='ball_approach_only',
                    ground_coordinate_frame='robot_x_right_z_forward',
                    ground_steering_angle_deg=angle, ground_forward_distance_m=distance)
    if confirmed:
        info.update(geometry)
    else:
        info['approach_candidate'].update(geometry)
        info['ground_projection_valid'] = False
    return info


def prepare(clock, *, angle=0., distance=1.2):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    _, running = start_line(node, bridge, clock)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .69
    fresh(node, clock, 'ball', observation(clock, confirmed=True, angle=angle, distance=distance), plan=False)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .72
    assert node.queued_general_command_id is None
    return node, bridge, running


def miss(node, clock):
    fresh(node, clock, 'ball', {'detected': False, 'raw_detected': False}, plan=False)


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('distance', [.610, .975, 1.2])
@pytest.mark.parametrize('gap', [False, True])
@pytest.mark.parametrize('angle,expected', [(0., 'STRAIGHT_2'), (-25., 'RECOVER_LEFT_TURN_LEFT_4'),
                                            (25., 'RECOVER_RIGHT_TURN_RIGHT_4')])
def test_third_candidate_reserves_without_second_confirmation_or_dwell(clock, index, distance, gap, angle, expected):
    node, bridge, running = prepare(clock, angle=angle, distance=distance)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock, angle=angle, distance=distance), plan=False)
        assert node.queued_general_command_id is None
    if gap:
        miss(node, clock)
        assert node.approach_walk_capture_count == 2
        assert node.ball_approach_observation.hits == 2
        assert node.queued_general_command_id is None
    fresh(node, clock, 'ball', observation(clock, angle=angle, distance=distance), plan=False)
    queued = node.publisher.messages[-1]
    assert queued['action'] == expected
    assert node.queued_general_command_id == queued['command_id']
    assert node.latest_info['ball']['confirmation_confirmed'] is False
    assert not node.ball_confirmation_pending_latched
    following = send(bridge, queued)
    assert not bridge.queued_request_deferred
    boundary = clock[0]
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert clock[0] == boundary
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2


def test_confirmed_frames_resume_count_after_short_miss(clock):
    node, _, _ = prepare(clock)
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock, confirmed=True), plan=False)
    miss(node, clock)
    assert node.approach_walk_capture_count == 2
    fresh(node, clock, 'ball', observation(clock, confirmed=True), plan=False)
    assert node.queued_general_action == 'STRAIGHT_2'


def test_changed_direction_needs_three_matching_current_decisions(clock):
    node, _, _ = prepare(clock)
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock), plan=False)
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock, angle=25.), plan=False)
        assert node.queued_general_command_id is None
    fresh(node, clock, 'ball', observation(clock, angle=25.), plan=False)
    assert node.queued_general_action == 'RECOVER_RIGHT_TURN_RIGHT_4'


@pytest.mark.parametrize('invalid', ['position_jump', 'depth_jump', 'old_anchor', 'bad_depth',
                                    'bad_ground', 'close_fine', 'close_pickup', 'low_confidence'])
def test_invalid_candidates_never_reuse_old_confirmation(clock, invalid):
    node, _, _ = prepare(clock)
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock), plan=False)
    if invalid == 'old_anchor':
        clock[0] += .51
    for _ in range(3):
        info = observation(clock)
        if invalid == 'position_jump':
            info['bbox'] = [850, 240, 900, 290]
        elif invalid == 'depth_jump':
            info['approach_candidate']['distance_m'] = .9
        elif invalid == 'bad_depth':
            info['approach_candidate']['depth_valid'] = False
        elif invalid == 'bad_ground':
            info['approach_candidate']['ground_projection_valid'] = False
        elif invalid in {'close_fine', 'close_pickup'}:
            info['approach_candidate']['distance_m'] = .6 if invalid == 'close_fine' else .5
        elif invalid == 'low_confidence':
            info['confidence'] = .1
        fresh(node, clock, 'ball', info, plan=False)
    assert node.queued_general_command_id is None
    assert not node.ball_pickup_entry_pending


def test_continuous_missing_does_not_publish_from_stored_geometry(clock):
    node, bridge, running = prepare(clock)
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock), plan=False)
    for _ in range(6):
        miss(node, clock)
        assert node.queued_general_command_id is None
    status(node, bridge, running, 'SUCCEEDED')
    assert not any(m['valid'] for m in node.publisher.messages[1:])


@pytest.mark.parametrize('guard', ['safety', 'executor', 'pickup', 'special'])
def test_reobservation_keeps_publication_guards(clock, guard):
    node, _, _ = prepare(clock)
    if guard == 'safety':
        node.safety_interlock.observe_executor_status(error_code='SDK_INTERNAL_ERROR',
            message='test', action='STRAIGHT', command_id=1, event_id=None)
    elif guard == 'executor':
        node.executor_auto_ready = False
    elif guard == 'pickup':
        node.ball_pickup_entry_pending = True
    else:
        node.phase_manager.start_special_action('PICKUP_NOW', 99)
    for _ in range(3):
        fresh(node, clock, 'ball', observation(clock), plan=False)
    assert node.queued_general_command_id is None


def test_duplicate_ball_and_line_callbacks_do_not_add_confirmation_hits(clock):
    node, _, _ = prepare(clock)
    fresh(node, clock, 'ball', observation(clock), plan=False)
    packet = String(data=json.dumps(node.latest_info['ball']))
    for _ in range(5):
        Node._info_callback(node, 'ball')(packet)
        Node._prepare_pending_approach_walk(node, clock[0])
    assert node.ball_approach_observation.hits == 1
    assert node.approach_walk_capture_count == 1
    assert node.queued_general_command_id is None


def test_line_steered_entry_uses_current_candidates_without_waiting_for_confirmed_flag(clock):
    node, bridge = node_with_entry(), FakeBridge()
    start_line(node, bridge, clock)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .69
    fresh(node, clock, 'ball', sample(clock[0]), plan=False)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .72
    fresh(node, clock, 'ball', sample(clock[0], detected=False), plan=False)
    for _ in range(12):
        fresh(node, clock, 'line', line_info(), plan=False)
    assert node.queued_general_command_id is None
    fresh(node, clock, 'ball', sample(clock[0], detected=False), plan=False)
    miss(node, clock)
    fresh(node, clock, 'ball', sample(clock[0], detected=False), plan=False)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.latest_info['ball']['detected'] is False


@pytest.mark.parametrize('preserve', [False, True])
def test_gap_retention_is_opt_in_and_does_not_extend_anchor_lifetime(preserve):
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10.), 10., config, preserve_gaps=preserve)
    for stamp in (10.05, 10.1):
        observer.observe(sample(stamp, detected=False), stamp, config, preserve_gaps=preserve)
    observer.observe(dict(detected=False, raw_detected=False, rgb_stamp_ns=10_150_000_000),
                     10.15, config, preserve_gaps=preserve)
    assert observer.observation is None and observer.candidate is None
    observer.observe(sample(10.2, detected=False), 10.2, config, preserve_gaps=preserve)
    assert observer.is_current(10.2) is preserve
    observer.observe(sample(10.51, detected=False), 10.51, config, preserve_gaps=preserve)
    assert not observer.is_current(10.51)
    assert observer.anchor_received_at == 10.


@pytest.mark.parametrize('result', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_motion_discards_reobserved_reservation(clock, result):
    node, bridge, running = prepare(clock)
    for _ in range(3):
        fresh(node, clock, 'ball', observation(clock), plan=False)
    send(bridge, node.publisher.messages[-1])
    status(node, bridge, running, result)
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None


def test_old_valid_frames_do_not_survive_a_long_gap(clock):
    node, _, _ = prepare(clock)
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock, confirmed=True), plan=False)
    clock[0] += .51
    miss(node, clock)
    fresh(node, clock, 'ball', observation(clock, confirmed=True), plan=False)
    assert node.approach_walk_capture_count == 1
    assert node.queued_general_command_id is None


def test_unconfirmed_preview_does_not_latch_ground_tracking(clock):
    node, bridge = node_with_entry(), FakeBridge()
    start_line(node, bridge, clock)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .69
    fresh(node, clock, 'ball', sample(clock[0]), plan=False)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .72
    for _ in range(2):
        fresh(node, clock, 'ball', observation(clock), plan=False)
        assert not node.ball_entry_line_alignment.ground_tracking_active
        assert node.queued_general_command_id is None
    fresh(node, clock, 'ball', observation(clock), plan=False)
    assert node.ball_entry_line_alignment.ground_tracking_active
    assert node.queued_general_action == 'STRAIGHT_2'
