"""Replay corner ownership across observations, motion boundaries, and recovery."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_line_corner_memory import CornerHarness, corner_info
from test_mission_phase_flow import line_info, release_general
from test_sparse_line_recovery import sparse_info
from test_line_ground_fit_recovery import unconfirmed_sparse_info


@pytest.fixture
def clock(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: int(now[0] * 1e9))
    return now


def node_for():
    node = CornerHarness(phase='LINE_TRACK')
    node.line_corner_turn_distance_m = .4
    return node


def observe(node, clock, info):
    clock[0] += .1
    info = {**info, 'rgb_stamp_ns': int(clock[0] * 1e9)}
    Node._info_callback(node, 'line')(String(data=json.dumps(info)))
    return info


def publish(node, clock, info):
    observe(node, clock, info)
    Node._publish_decision(node)
    return node.publisher.messages[-1]


@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('reason,count', [
    ('too_few_segment_points', 2), ('too_few_projected_points', 3),
    ('too_few_inliers', 4), ('degenerate_forward_span', 3),
    ('fit_did_not_converge', 4),
])
def test_measured_near_corner_cannot_authorize_turn_without_tracking_geometry(clock, direction, reason, count):
    node = node_for()
    info = {**sparse_info(direction=direction, distance=.35),
            'ground_fit_reason': reason, 'ground_fit_input_point_count': count,
            'center_points_px': [[500 if direction == 'RIGHT' else 800, 700]], 'image_width': 1280}
    result = publish(node, clock, info)
    assert not result['valid'] and result['action'] == 'WAIT'
    assert result['action'] not in {'LEFT', 'RIGHT', 'STRAIGHT_1'}


@pytest.mark.parametrize('updates', [
    {'ground_fit_reason': 'invalid_parameters'}, {'ground_fit_segment': 'FULL_PATH'},
    {'corner_start_depth_valid': False}, {'corner_preview_raw_detected': False},
    {'corner_preview_held': True}, {'geometry_quality': .1},
])
def test_memory_does_not_authorize_turn_from_bad_corner_evidence(clock, updates):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    result = publish(node, clock, {**sparse_info(distance=.35), **updates})
    assert not result['valid'] and result['action'] == 'WAIT'


def test_fresh_loss_can_search_without_last_near_point_side(clock):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    node.planner.last_line_seen_direction = None
    result = publish(node, clock, {'detected': False, 'raw_detected': False})
    assert result['action'] == 'LINE_LOST_TURN_RIGHT_2'
    assert node.pending_line_corner['search_started']


@pytest.mark.parametrize('next_info', [
    {'detected': False},
    corner_info(corner_preview_held=True),
    {**sparse_info(), 'corner_start_depth_valid': False},
])
def test_expired_memory_blocks_both_search_and_ground_recovery(clock, next_info):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    last_confirmed = node.pending_line_corner['last_confirmed_at']
    clock[0] += Node.LINE_CORNER_MEMORY_TIMEOUT_SEC + .1
    result = publish(node, clock, next_info)
    assert result['action'] == 'WAIT' and result['reason'] == 'line_corner_memory_expired'
    assert node.pending_line_corner['last_confirmed_at'] == last_confirmed
    # Only a real new confirmation refreshes the expired memory.
    result = publish(node, clock, corner_info(corner_start_distance_m=.35))
    assert result['action'] == 'RIGHT'


def test_cached_frame_neither_refreshes_memory_nor_confirms_new_direction(clock):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    opposite = observe(node, clock, corner_info('LEFT', corner_start_distance_m=.7))
    for _ in range(10):
        Node._remember_line_corner(node, opposite)
    assert node.pending_line_corner['direction_candidate_hits'] == 1
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'
    observe(node, clock, corner_info('LEFT', corner_start_distance_m=.7))
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'
    observe(node, clock, corner_info('LEFT', corner_start_distance_m=.7))
    assert node.pending_line_corner['corner_direction'] == 'LEFT'


def test_direction_conflict_after_search_does_not_reverse_or_refund(clock):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    command = publish(node, clock, {'detected': False})
    release_general(node, command)
    for _ in range(4):
        result = publish(node, clock, corner_info('LEFT', corner_start_distance_m=.35))
        assert not result['valid'] and result['reason'] == 'line_corner_direction_conflict'
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'
    assert node.lost_search_turn_limiter.angles['line'] == 15.


def test_visible_alignment_does_not_spend_corner_loss_budget(clock):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    visible = {**unconfirmed_sparse_info(distance=.7), 'corner_preview_confirmed': True,
               'ground_two_point_candidate': None}
    for index, info in enumerate(({'detected': False}, visible, {'detected': False})):
        command = publish(node, clock, info)
        expected = 'LINE_OFFSET_TURN_RIGHT_2' if info.get('detected') else 'LINE_LOST_TURN_RIGHT_2'
        assert command['action'] == expected
        assert node.lost_search_turn_limiter.angles['line'] == (15. if index < 2 else 30.)
        release_general(node, command)
        if info.get('detected'):
            assert node.lost_search_turn_limiter.image_alignment_angle == 15.
            clock[0] += 1.01
            Node._publish_decision(node)
    command = publish(node, clock, {'detected': False})
    assert command['valid'] and command['action'] == 'LINE_LOST_TURN_RIGHT_2'
    release_general(node, command)
    blocked = publish(node, clock, {'detected': False})
    assert not blocked['valid'] and blocked['reason'] == 'lost_search_turn_limit_reached'
    corner = publish(node, clock, corner_info(corner_start_distance_m=.35))
    assert corner['action'] == 'RIGHT'
    release_general(node, corner)
    assert node.lost_search_turn_limiter.angles['line'] == 0.
    assert node.lost_search_turn_limiter.image_alignment_angle == 0.


def test_freshness_and_failure_still_outrank_corner_selection(clock):
    node = node_for()
    observe(node, clock, corner_info(corner_start_distance_m=.7))
    command = publish(node, clock, {'detected': False})
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], 'FAILED')
    result = publish(node, clock, {'detected': False})
    assert not result['valid']
    assert node.lost_search_turn_limiter.angles['line'] == 15.
    result = publish(node, clock, corner_info(corner_start_distance_m=.35))
    assert not result['valid'] and result['reason'] == 'lost_search_motion_failed'


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
def test_completed_corner_cannot_repeat_via_normal_planner(clock, side):
    node = node_for()
    info = corner_info(side, corner_start_distance_m=.35)
    command = publish(node, clock, info)
    assert command['action'] == side
    release_general(node, command)
    for _ in range(3):
        result = publish(node, clock, info)
        assert not result['valid'] and result['reason'] == 'line_corner_waiting_for_clear'
    forward = publish(node, clock, {**line_info(), 'corner_preview_confirmed': False})
    assert forward['action'] == 'STRAIGHT'
    release_general(node, forward)
    next_corner = publish(node, clock, info)
    assert next_corner['valid'] and next_corner['action'] == side
