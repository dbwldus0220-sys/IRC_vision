"""Direction-only hurdle alignment and the 450mm atomic checkpoint."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from test_hurdle_positioning import hurdle, line_info, HurdleHarness, clock
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages


@pytest.mark.parametrize('depth', [.8, .7, .54, .450001])
@pytest.mark.parametrize('offset', [None, -.9, -.01, 0., .01, .9])
@pytest.mark.parametrize('angle,action', [
    (-50., 'ALIGN_LEFT'), (-15., 'ALIGN_LEFT'), (-14.999, 'STRAIGHT'),
    (0., 'STRAIGHT'), (14.999, 'STRAIGHT'), (15., 'ALIGN_RIGHT'),
    (50., 'ALIGN_RIGHT'), (50.001, 'ALIGN_RIGHT'),
])
def test_angle_alone_selects_direction_even_inside_old_pixel_deadband(depth, offset, angle, action):
    command = HurdleNavigationPlanner().plan({
        **hurdle(depth), 'path_reference_valid': True,
        'path_offset_x_norm': offset, 'path_target_bearing_deg': -89.,
        'line_heading_valid': True, 'line_heading_deg': angle,
        'line_heading_reference_source': 'fresh',
    }, positioning=True)
    expected = 'STRAIGHT_1' if action == 'STRAIGHT' and depth <= .7 else action
    assert command.valid and command.action == expected
    assert command.center_steering_deg == angle


@pytest.mark.parametrize('angle,count', [(-15., 1), (-30., 2), (-45., 3),
                                       (15., 1), (30., 3), (45., 5)])
def test_actual_fusion_and_bridge_use_line_heading_not_intersection_or_parallel_angle(angle, count):
    # The diagnostic intersection is on the opposite side of the requested heading.
    observations = {'hurdle': hurdle(.6, bbox=[400, 300, 1000, 719],
                                    hurdle_angle_deg=-80 if angle > 0 else 80),
                    'line': {**line_info(x=500 if angle > 0 else 900, heading=angle),
                             'center_points_px': [[500 if angle > 0 else 900, y]
                                                  for y in (250, 180, 100)]}}
    planner = MotionDecisionPlanner()
    decision = planner.plan('AUTO', observations, .1)
    direction = 'LEFT' if angle < 0 else 'RIGHT'
    assert decision.action == f'ALIGN_{direction}'
    assert decision.source_command['path_reference_valid']
    assert decision.source_command['path_target_bearing_deg'] * angle < 0
    assert decision.source_command['turn_count'] == count
    assert not decision.requires_ack
    limiter = LostSearchTurnLimiter()
    assert limiter.filter(decision, observations).valid
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action=decision.action, source_command=decision.source_command,
    ))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == (
        f'post_ball_line_turn_{direction.lower()}_{count}'
    )
    assert bridge.queued_request_id is None


@pytest.mark.parametrize('depth,latched', [(.54, False), (.52, False), (.450001, False),
                                         (.45, True), (.449999, True)])
def test_old_checkpoint_cannot_reserve_final_tail(clock, depth, latched):
    node = HurdleHarness()
    node.receive_line()
    current = node.publish()[-1]
    node.receive(hurdle(depth))
    assert (getattr(node, 'pending_hurdle_final_sequence', None) is not None) is latched
    assert node.publish() == []
    assert node.general_motion_gate.active_command_id == current['command_id']


@pytest.mark.parametrize('angle', [None, float('nan'), float('inf')])
def test_fresh_heading_marked_valid_without_numeric_angle_does_not_authorize_motion(angle):
    command = HurdleNavigationPlanner().plan({
        **hurdle(.6), 'path_reference_valid': True, 'path_offset_x_norm': 0,
        'line_heading_valid': True, 'line_heading_deg': angle,
        'line_heading_reference_source': 'fresh',
    }, positioning=True)
    assert not command.valid and command.action == 'WAIT'


@pytest.mark.parametrize('bottom', [None, -1, 100, 101])
@pytest.mark.parametrize('depth', [.8, .6, .45])
def test_proximity_guards_and_final_priority_remain_explicit(depth, bottom):
    command = HurdleNavigationPlanner().plan({
        **hurdle(depth, bottom_distance_px=bottom), 'path_reference_valid': True,
        'line_heading_valid': True, 'line_heading_deg': 30.,
        'line_heading_reference_source': 'fresh', 'path_offset_x_norm': .01,
    }, positioning=True)
    if depth <= .45:
        assert command.action == 'GO'
    elif bottom in (None, -1):
        assert command.action == 'WAIT' and not command.valid
    elif bottom <= 100:
        assert command.action == ('STRAIGHT_1' if depth <= .7 else 'WAIT')
    else:
        assert command.action == 'ALIGN_RIGHT'


@pytest.mark.parametrize('heading', [1, 5, 13])
def test_small_line_slope_does_not_turn_toward_large_intersection_or_parallel_angle(heading):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(.6, bbox=[400, 300, 1000, 719], hurdle_angle_deg=20.2),
        'line': {**line_info(x=500, heading=heading),
                 'center_points_px': [[500, 250], [500, 180], [500, 100]]},
    }, .1)
    assert abs(decision.source_command['path_target_bearing_deg']) > 89
    assert decision.valid and decision.action == 'STRAIGHT_1'
    assert decision.source_command['center_steering_deg'] == heading
    assert 'turn_count' not in decision.source_command


def test_reacquired_hurdle_waits_for_line_then_releases_final_sequence():
    planner = MotionDecisionPlanner()
    planner.plan('AUTO', {'hurdle': hurdle(.6, camera_center_offset_x_px=400)}, .1)
    planner.plan('AUTO', {'hurdle': {'detected': False}}, .1)
    near = hurdle(.43, hurdle_angle_deg=40)
    waiting = planner.plan('AUTO', {'hurdle': near}, .1)
    assert waiting.reason == 'hurdle_recovery_waiting_for_line_heading'
    assert not waiting.valid and not planner.hurdle_go_requested
    ready = planner.plan('AUTO', {'hurdle': near, 'line': line_info(heading=5)}, .1)
    assert ready.valid and ready.action == 'GO'
    assert not planner.hurdle_planner.recovery_active
