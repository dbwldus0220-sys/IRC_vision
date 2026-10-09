"""Current image geometry owns alignment; memory owns only actual line loss."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_line_ground_fit_recovery import (
    clock, node_for, frame, start_turn, finish_turn, unconfirmed_sparse_info,
)
from test_line_offset_alignment import sample


def test_target_uses_robot_center_and_stops_before_corner():
    info = unconfirmed_sparse_info()
    info['center_points_px'] = [[1010, 650], [960, 550], [100, 430]]
    x, y, angle = Node._line_image_alignment_target(info)
    assert (x, y) == (960, 550)
    assert angle > 0
    # The post-corner point is closer to the target row, but belongs to another leg.
    info.update(corner_preview_raw_detected=False, corner_preview_confirmed=False)
    assert Node._line_image_alignment_target(info)[0] == 100


@pytest.mark.parametrize('updates', [
    {'center_points_px': []}, {'center_points_px': [[800, 500]]},
    {'center_points_px': [[800, 500], [800, 510]]},
    {'center_points_px': [[800, 500], [800, 490]]},
    {'center_points_px': [[800, 500], [float('nan'), 400]]},
    {'center_points_px': [[800, 500], [-1, 400]]},
    {'center_points_px': [[800, 500], [1280, 400]]},
    {'center_points_px': [[800, 720], [800, 400]]},
    {'center_points_px': [[True, 500], [800, 400]]},
    {'corner_start_index': None}, {'corner_start_index': True}, {'corner_start_index': 0},
    {'robot_center_x_px': None}, {'robot_center_x_px': 1280},
    {'image_width': None}, {'image_height': 0},
])
def test_invalid_image_geometry_cannot_authorize_alignment(updates):
    assert Node._line_image_alignment_target({**unconfirmed_sparse_info(), **updates}) is None


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
def test_current_image_alignment_can_oppose_remembered_corner(clock, phase):
    node = node_for(phase)
    info = {**unconfirmed_sparse_info(), 'corner_preview_confirmed': True,
            'ground_two_point_candidate': None,
            'center_points_px': [[410, 650], [460, 450], [900, 250]]}
    command = start_turn(node, clock, info)
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'
    assert command['action'] == 'LINE_OFFSET_TURN_LEFT_1'
    assert command['source_command']['direction_source'] == 'current_image_line'
    assert not command['source_command'].get('corner_search_pending')


def test_each_completed_turn_uses_new_image_direction(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    right = start_turn(node, clock, unconfirmed_sparse_info())
    finish_turn(node, clock, right)
    left = start_turn(node, clock, {**unconfirmed_sparse_info(),
                                  'center_points_px': [[540, 650], [560, 450]]})
    assert left['action'] == 'LINE_OFFSET_TURN_LEFT_1'
    assert node.lost_search_turn_limiter.image_alignment_angle == 30.
    assert node.lost_search_turn_limiter.angles['line'] == 0.


def test_ground_fit_flicker_does_not_restart_equivalent_alignment_pause(clock):
    node = node_for()
    good = sample(offset=250., target_angle=25., ground_heading_error_deg=-10., ground_fit_point_count=3,
                  ground_projection_enabled=True, corner_preview_confirmed=False,
                  corner_preview_raw_detected=False)
    start = clock[0]
    for elapsed, info in [(0., good), (.2, unconfirmed_sparse_info()), (.4, good),
                          (.6, unconfirmed_sparse_info()), (.8, good)]:
        clock[0] = start + elapsed
        assert not any(m['valid'] for m in frame(node, clock, info))
        assert node.pre_motion_settle_started_at == start
    clock[0] = start + 1.01
    command = frame(node, clock, unconfirmed_sparse_info())[-1]
    assert command['valid'] and command['action'] == 'LINE_OFFSET_TURN_RIGHT_2'
    assert command['source_command']['image_line_alignment']


def test_corner_capture_boundary_also_blocks_image_alignment(clock):
    node = node_for()
    info = {**unconfirmed_sparse_info(), 'corner_preview_confirmed': True,
            'ground_two_point_candidate': None,
            'rgb_stamp_ns': round(clock[0] * 1e9)}
    node.latest_time['line'] = clock[0]
    Node._remember_line_corner(node, info)
    node.pending_line_corner['minimum_rgb_stamp_ns'] = info['rgb_stamp_ns']
    candidate = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    gated = Node._apply_pending_line_corner(node, candidate, info)
    result = Node._recover_invalid_line_ground(node, gated, info, clock[0])
    assert not result.valid


def test_visible_line_cannot_use_legacy_corner_search_override(clock):
    from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
    node = node_for()
    candidate = Node._corner_search_decision(node, {'corner_direction': 'RIGHT'},
                                           {'corner_ground_reacquire': True})
    result = LostSearchTurnLimiter(staged_sources=('line',)).filter(
        candidate, {'line': {**unconfirmed_sparse_info(), 'corner_preview_confirmed': True}})
    assert not result.valid
    assert result.reason == 'lost_search_waiting_for_fresh_vision'
