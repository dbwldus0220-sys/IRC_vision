"""Near-horizontal corner replay through real observation, motion and queue gates."""

import copy
import json
import math

import numpy as np
import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from mission_control.sparse_line_recovery import TwoPointRotationObserver
from step.yolo_line_analyzer import (
    GROUND_PROJECTION_DEFAULTS, LinePoint, analyze_ground_line, two_point_ground_candidate,
)
from test_line_ground_fit_recovery import clock, node_for, frame, finish_turn, unconfirmed_sparse_info
from test_mission_phase_flow import line_info
from test_corner_exit_continuity import exit_info
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_exit_observation_dwell import begin_exit, status, motions
from test_pickup_exit_observation_dwell import frame as dwell_frame


def rotation_info(pixels=None, **updates):
    # Hand-read video markers, not the missing original line_info packet.
    pixels = pixels if pixels is not None else [[861., 712.], [730., 694.]]
    points = [LinePoint(x, y, 1.) for x, y in pixels]
    return {
        **unconfirmed_sparse_info(),
        **analyze_ground_line(points, 1280, 720, corner_start_index=1),
        'center_points_px': pixels,
        'ground_two_point_candidate': two_point_ground_candidate(
            points, 1280, 720, {}, corner_start_index=1),
        'ground_two_point_rotation_candidate': two_point_ground_candidate(
            points, 1280, 720, {}, corner_start_index=1, rotation_only=True),
        **updates,
    }


def heading_info(angle, **updates):
    # Synthetic post-motion observation; this does not model actual body yaw.
    inverse = np.linalg.inv(np.array(GROUND_PROJECTION_DEFAULTS['ground_homography']).reshape(3, 3))
    pixels = []
    for x, z in ((.08, .36), (.08 + .075 * math.sin(math.radians(angle)),
                            .36 + .075 * math.cos(math.radians(angle)))):
        p = inverse @ [x, z, 1.]
        pixels.append((p[:2] / p[2]).tolist())
    return rotation_info(pixels, **updates)


def next_motion(node, clock, info, duration=1.2):
    start = clock[0]
    for i in range(round(duration / .05) + 1):
        clock[0] = start + i * .05
        overrides = {'rgb_stamp_ns': info['rgb_stamp_ns']} if 'rgb_stamp_ns' in info else {}
        valid = [c for c in frame(node, clock, info, **overrides) if c['valid']]
        if valid:
            return valid[-1]
    return None


def test_rotation_candidate_preserves_walking_geometry_and_source():
    info = rotation_info()
    assert info['ground_two_point_candidate'] is None
    assert not info['ground_projection_valid']
    assert info['ground_heading_error_deg'] is None
    assert info['ground_two_point_rotation_candidate']['heading_deg'] == pytest.approx(-83.46, abs=.02)
    assert info['ground_fit_input_point_count'] == 2


@pytest.mark.parametrize('angle', [-83., 83.])
@pytest.mark.parametrize('phase', ['POST_BALL_LINE_ALIGN', 'LINE_TRACK_AFTER_PICKUP', 'LINE_TRACK'])
def test_confirmed_heading_owns_one_turn_before_image_target(clock, angle, phase):
    node = node_for(phase)
    info = heading_info(angle, corner_start_depth_valid=False, corner_start_distance_m=None)
    original = copy.deepcopy(info)
    assert not [c for c in frame(node, clock, info) if c['valid']]
    clock[0] += .05
    assert not [c for c in frame(node, clock, info) if c['valid']]
    command = next_motion(node, clock, info)
    side = 'LEFT' if angle < 0 else 'RIGHT'
    assert command['action'] == f'LINE_HEADING_TURN_{side}_1'
    assert command['source_command']['two_point_ground_alignment']
    assert not command['source_command'].get('image_line_alignment')
    assert command['source_command']['travel_distance_m'] == 0.
    assert not node.latest_info['line']['ground_projection_valid']
    assert info == original


@pytest.mark.parametrize('section', [1, 2])
def test_pickup_pause_collects_rotation_frames_without_second_pause(clock, section):
    node, motion_id = begin_exit(clock, section)
    for moment in (10.1, 10.2, 10.4):
        clock[0] = moment
        dwell_frame(node, clock, rotation_info())
        Node._publish_decision(node)
        assert not motions(node)
    assert node.line_rotation_observer.stable
    clock[0] = 10.5
    status(node, motion_id=motion_id, status='SUCCEEDED')
    assert [c['action'] for c in motions(node)] == ['LINE_HEADING_TURN_LEFT_1']
    assert node.pre_motion_settle_started_at is None
    assert getattr(node, 'post_ball_line_started_count', 0) == 0


def test_real_bridge_gets_one_turn_and_active_motion_never_prequeues(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    command = next_motion(node, clock, rotation_info())
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'post_ball_line_turn_right_1'
    bridge.stationary_since = clock[0] - .5
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 1 and requests[0]['motion_id'] == 'post_ball_line_turn_left_1'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert len(decoded_messages(bridge.executor_request_publisher)) == 1
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    for _ in range(8):
        clock[0] += .05
        assert not [c for c in frame(node, clock, rotation_info()) if c['valid']]
    assert getattr(node, 'queued_general_command_id', None) is None
    assert bridge.queued_request_id is None
    finish_turn(node, clock, command)
    assert next_motion(node, clock, rotation_info()) is None
    assert node.last_selected_decision.reason == 'line_two_point_alignment_no_progress'


def test_reference_change_keeps_shared_budget_and_uses_ground_improvement(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    limiter = node.lost_search_turn_limiter = LostSearchTurnLimiter(staged_sources=('line',))
    limiter.image_alignment_count = 1
    limiter.image_alignment_angle = 15.
    limiter.image_alignment_error = 60.
    limiter.image_alignment_direction = 'RIGHT'
    for count, angle in ((2, -83.), (3, -68.)):
        command = next_motion(node, clock, heading_info(angle))
        assert command and command['action'] == 'LINE_HEADING_TURN_LEFT_1'
        assert limiter.image_alignment_count == count
        assert limiter.image_alignment_error == 60.
        assert limiter.two_point_alignment_error == pytest.approx(abs(angle))
        finish_turn(node, clock, command)
    assert limiter.image_alignment_angle == 45.
    assert next_motion(node, clock, heading_info(-53.)) is None
    assert node.last_selected_decision.reason == 'line_image_alignment_limit_reached'


def test_three_bounded_turns_release_to_confirmed_two_point_walking(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    for angle in (-83., -68., -53.):
        command = next_motion(node, clock, heading_info(angle))
        assert command['action'] == 'LINE_HEADING_TURN_LEFT_1'
        finish_turn(node, clock, command)
        assert getattr(node, 'post_ball_line_started_count', 0) == 0
    assert node.lost_search_turn_limiter.image_alignment_count == 3
    assert node.lost_search_turn_limiter.image_alignment_angle == 45.
    command = next_motion(node, clock, heading_info(-38., corner_preview_confirmed=True))
    assert command['action'] == 'LINE_SPARSE_FORWARD'
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    assert command['source_command']['post_pickup_line_index'] == 1
    assert 'goal_head_prepare_id' not in command['source_command']


@pytest.mark.parametrize('near_exit', [False, True])
def test_waiting_for_existing_two_point_fit_releases_phase_and_corner(clock, near_exit):
    node = node_for('POST_BALL_LINE_ALIGN')
    first = next_motion(node, clock, rotation_info())
    finish_turn(node, clock, first)
    info = heading_info(-10., corner_preview_confirmed=True)
    if near_exit:
        info.update({k: v for k, v in exit_info(30., distance=.14).items() if k.startswith('corner_')})
    for _ in range(2):
        clock[0] += .05
        assert not [c for c in frame(node, clock, info) if c['valid']]
    command = next_motion(node, clock, info)
    assert command and node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    assert command['reason'] == ('line_corner_exit_alignment' if near_exit else 'two_point_short_forward')
    assert command['source_command'].get('goal_head_prepare_id') is None
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    assert node.post_ball_line_started_count == 1


@pytest.mark.parametrize('updates,reason', [
    ({'corner_start_depth_valid': False}, 'line_image_corner_distance_unavailable'),
    ({'corner_start_distance_m': None}, 'line_image_corner_distance_unavailable'),
    ({'corner_start_distance_m': .14}, 'line_image_corner_exit_required'),
])
def test_image_translation_does_not_cross_unverified_near_corner(clock, updates, reason):
    node = node_for('POST_BALL_LINE_ALIGN')
    assert next_motion(node, clock, heading_info(-10., **updates)) is None
    assert node.last_selected_decision.reason == reason


def test_normal_ground_releases_rotation_budget(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    first = next_motion(node, clock, rotation_info())
    finish_turn(node, clock, first)
    info = {**line_info(), 'ground_projection_enabled': True, 'ground_fit_point_count': 3,
            'ground_fit_reason': 'ok', 'corner_preview_raw_detected': False}
    command = next_motion(node, clock, info)
    assert command['action'].startswith('STRAIGHT')
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    assert node.lost_search_turn_limiter.image_alignment_count == 0
    assert node.lost_search_turn_limiter.two_point_alignment_error is None


def test_three_line_starts_and_camera_preparation_follow_reacquisition(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    first = next_motion(node, clock, rotation_info())
    finish_turn(node, clock, first)
    assert getattr(node, 'post_ball_line_started_count', 0) == 0
    info = {**line_info(), 'ground_projection_enabled': True,
            'ground_fit_point_count': 3, 'ground_fit_reason': 'ok',
            'corner_preview_raw_detected': False}
    for ordinal in (1, 2, 3):
        command = next_motion(node, clock, info)
        assert command['source_command']['post_pickup_line_index'] == ordinal
        assert ('goal_head_prepare_id' in command['source_command']) is (ordinal == 3)
        node.send_status(command['action'], command['command_id'], 'RUNNING')
        assert node.post_ball_line_started_count == ordinal
        clock[0] += .7
        node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
        clock[0] += .01
    assert node.mission_phase == 'GOAL_APPROACH'


@pytest.mark.parametrize('changes', [
    {'corner_start_index': 0}, {'corner_start_index': True},
    {'parameters': {'ground_projection_enabled': False}},
    {'parameters': {'ground_homography': [0.] * 9}},
])
def test_rotation_candidate_never_bypasses_calibration_or_segment(changes):
    args = dict(points=[LinePoint(861, 712, 1.), LinePoint(730, 694, 1.)],
                image_width=1280, image_height=720, parameters={},
                corner_start_index=1, rotation_only=True)
    args.update(changes)
    assert two_point_ground_candidate(**args) is None


def test_direction_reversals_share_budget_across_error_references(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    limiter = node.lost_search_turn_limiter = LostSearchTurnLimiter(staged_sources=('line',))
    limiter.image_alignment_count = 1
    limiter.image_alignment_angle = 15.
    limiter.image_alignment_error = 60.
    limiter.image_alignment_direction = 'RIGHT'
    first = next_motion(node, clock, rotation_info())
    assert first and limiter.image_alignment_reversals == 1
    finish_turn(node, clock, first)
    assert next_motion(node, clock, heading_info(68.)) is None
    assert node.last_selected_decision.reason == 'line_image_alignment_oscillating'


@pytest.mark.parametrize('change', [
    {'heading_quality': .1}, {'ground_two_point_rotation_candidate': None},
    {'corner_preview_held': True}, {'corner_preview_raw_detected': False},
])
def test_invalid_rotation_evidence_never_authorizes_rotation(clock, change):
    node = node_for('POST_BALL_LINE_ALIGN')
    next_motion(node, clock, rotation_info(**change))
    assert not any(c['source_command'].get('two_point_ground_alignment') for c in node.publisher.messages if c['valid'])


def test_unstable_rotations_cannot_escape_to_image_motion(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    for angle in [-83., -65.] * 15:
        clock[0] += .05
        assert not [c for c in frame(node, clock, heading_info(angle)) if c['valid']]
    assert node.last_selected_decision.reason == 'rotation_heading_unstable'


@pytest.mark.parametrize('options', [
    {'stationary': False}, {'stamp': 10_000_000_000, 'minimum_stamp': 10_000_000_000},
    {'stamp': 9_000_000_000}, {'stamp': 11_000_000_000}, {'stamp': True},
])
def test_observer_rejects_motion_stale_duplicate_and_pre_boundary_samples(options):
    observer = TwoPointRotationObserver()
    info = rotation_info()
    for i in range(5):
        args = dict(stamp=10_000_000_000 + i * 10_000_000,
                    ros_now=10_000_000_000 + i * 10_000_000,
                    stationary=True, minimum_stamp=0, max_age=.5, min_quality=.35)
        args.update(options)
        observer.observe(info, **args)
        assert not observer.stable


def test_duplicate_frame_does_not_confirm_and_bad_frame_clears_evidence():
    observer = TwoPointRotationObserver()
    args = dict(stamp=10_000_000_000, ros_now=10_000_000_000,
                stationary=True, minimum_stamp=0, max_age=.5, min_quality=.35)
    for _ in range(5):
        observer.observe(rotation_info(), **args)
    assert len(observer.samples) == 1 and not observer.stable
    observer.observe(rotation_info(heading_quality=.1), **args)
    assert not observer.samples


@pytest.mark.parametrize('outcome', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_rotation_never_restarts_on_better_geometry(clock, outcome):
    node = node_for('POST_BALL_LINE_ALIGN')
    first = next_motion(node, clock, rotation_info())
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += .6
    node.send_status(first['action'], first['command_id'], outcome)
    clock[0] += .6
    assert next_motion(node, clock, heading_info(-68.)) is None
