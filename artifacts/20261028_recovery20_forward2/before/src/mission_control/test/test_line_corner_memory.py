"""Replay confirmed corners through the real callbacks and motion boundaries."""

import json
from dataclasses import replace

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from test_mission_phase_flow import MissionFlowHarness, line_info, release_general


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    return now


class CornerHarness(MissionFlowHarness):
    _fresh_observations = MotionDecisionNode._fresh_observations

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.line_corner_turn_distance_m = 0.15
        # These tests isolate memory/freshness gates; three-hit confirmation is tested separately.
        self.planner.line_planner.config = replace(
            self.planner.line_planner.config, direction_confirmation_frames=1)


    def publish_vision(self, **observations):
        for source, info in observations.items():
            if info is None:
                self.latest_info[source] = None
                self.latest_time[source] = None
            else:
                MotionDecisionNode._info_callback(self, source)(
                    String(data=json.dumps(info)),
                )
        before = len(self.publisher.messages)
        MotionDecisionNode._publish_decision(self)
        return self.publisher.messages[before:]


def receive_line(node, clock, info):
    clock[0] += .01
    MotionDecisionNode._info_callback(node, 'line')(String(data=json.dumps(info)))


def corner_info(direction='RIGHT', **overrides):
    info = {
        **line_info(heading=10. if direction == 'RIGHT' else -10.),
        'turn_angle_deg': 60. if direction == 'RIGHT' else -60.,
        'corner_preview_confirmed': True,
        'corner_direction': direction, 'corner_start_distance_m': .14,
        'corner_start_depth_valid': True, 'corner_preview_held': False,
        'corner_start_lateral_offset_m': 0.,
        # A right corner can have its nearest visible point on the left.
        'center_points_px': [[550, 700], [950, 400]], 'image_width': 1280,
    }
    info.update(overrides)
    return info


def start_line(node):
    command = node.publish_vision(line=line_info())[-1]
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    return command


@pytest.mark.parametrize('capture_age_sec,held,ground_valid,expected', [
    (.1, False, True, 'RECOVER_LEFT_TURN_LEFT_4'),
    (.1, True, True, 'RECOVER_LEFT_TURN_LEFT_4'),
    (-.1, False, True, 'WAIT'),
    (.6, False, True, 'WAIT'),
    (.1, True, False, 'WAIT'),
])
def test_visible_corner_separates_clock_rejection_from_occlusion(
    clock, monkeypatch, capture_age_sec, held, ground_valid, expected,
):
    node = CornerHarness(phase='LINE_TRACK')
    ros_now = 100_000_000_000
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns',
                        staticmethod(lambda _: ros_now))
    stamp = ros_now - round(capture_age_sec * 1e9)
    info = corner_info(
        ground_heading_error_deg=-14.26, heading_error_deg=-10.6,
        filtered_lateral_offset_norm=-.055, turn_angle_deg=51.,
        corner_start_distance_m=1.15, corner_preview_held=held,
        ground_projection_valid=ground_valid,
        stamp={'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
    )
    node.latest_time['line'] = clock[0]
    node.pending_line_corner = {
        'corner_direction': 'RIGHT', 'motion_completed_at': clock[0] - 2,
        'last_confirmed_at': clock[0] - 2,
        'minimum_rgb_stamp_ns': ros_now - 2_000_000_000,
    }
    decision = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    result = MotionDecisionNode._apply_pending_line_corner(node, decision, info)
    assert result.action == expected
    if expected == 'WAIT':
        assert result.reason == 'line_corner_waiting_for_usable_line'
        checks = result.source_command['line_input_checks']
        assert checks['capture_age_sec'] == pytest.approx(capture_age_sec)
        assert checks['received_age_sec'] == 0.
        assert checks['geometry_usable'] is ground_valid
        assert checks['captured_after_motion'] is True


@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
def test_far_selected_corner_turn_uses_short_forward_without_lateral_target(clock, direction):
    node = CornerHarness(phase='LINE_TRACK')
    heading = 30. if direction == 'RIGHT' else -30.
    info = corner_info(direction, ground_heading_error_deg=heading,
                       corner_start_distance_m=.4, corner_start_lateral_offset_m=None)
    receive_line(node, clock, info)
    decision = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    assert decision.action == direction
    gated = MotionDecisionNode._apply_pending_line_corner(node, decision, info)
    assert gated.valid and gated.action == 'STRAIGHT_1'
    assert gated.reason == 'line_corner_turn_too_far'


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK'])
@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('after_motion', [{'detected': False}, None])
def test_corner_survives_loss_but_waits_for_fresh_distance_after_completion(clock, phase, direction, after_motion):
    node = CornerHarness(phase=phase)
    current = start_line(node)
    receive_line(node, clock, corner_info(direction, ground_projection_valid=False))
    receive_line(node, clock, {'detected': False})
    count = len(node.publisher.messages)
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == count
    node.send_status('STRAIGHT', current['command_id'] + 1, 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == count
    node.send_status('STRAIGHT', current['command_id'], 'SUCCEEDED')
    command = node.publish_vision(line=after_motion)[-1]
    assert command['action'] == 'WAIT' and not command['valid']
    assert node.pending_line_corner['corner_direction'] == direction
    receive_line(node, clock, corner_info(direction))
    command = node.publish_vision()[-1]
    assert command['action'] == direction
    assert command['source'] == 'line'
    assert command['reason'] == 'line_tracking'
    assert command['source_command']['corner_from_memory'] is True
    assert node.pending_line_corner is None
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID[direction] == f'line_recovery_{direction.lower()}_4'
    assert not any(c['valid'] for c in node.publish_vision(line={'detected': False}))


def test_corner_is_kept_through_recovery_and_fresh_negative_frame(clock):
    node = CornerHarness(phase='AUTO')
    current = node.publish_vision(line=line_info(heading=-20., offset=-.3))[-1]
    assert current['action'] == 'RECOVER_LEFT_TURN_LEFT_4'
    node.send_status(current['action'], current['command_id'], 'RUNNING')
    receive_line(node, clock, corner_info('RIGHT'))
    release_general(node, current)
    # A fresh negative observation still overrides the earlier usable frame.
    receive_line(node, clock, {'detected': False})
    assert node.publish_vision()[-1]['action'] == 'WAIT'
    receive_line(node, clock, corner_info())
    assert node.publish_vision()[-1]['action'] == 'RIGHT'


@pytest.mark.parametrize('overrides', [
    {'corner_preview_confirmed': False}, {'corner_preview_confirmed': 'true'},
    {'detected': False}, {'corner_direction': 'UNKNOWN'},
    {'heading_quality': .1},
])
def test_unconfirmed_or_low_quality_corner_does_not_override_search(clock, overrides):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info(**overrides))
    assert getattr(node, 'pending_line_corner', None) is None
    release_general(node, current)
    command = node.publish_vision(line={'detected': False})[-1]
    assert command['action'] not in {'LEFT', 'RIGHT'}


def test_first_confirmed_direction_survives_conflicting_preview(clock):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info('RIGHT'))
    receive_line(node, clock, corner_info('LEFT'))
    release_general(node, current)
    assert node.publish_vision(line={'detected': False})[-1]['action'] == 'WAIT'
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'
    receive_line(node, clock, corner_info())
    assert node.publish_vision()[-1]['action'] == 'RIGHT'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_unsuccessful_current_motion_discards_corner(clock, status):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    node.send_status(current['action'], current['command_id'], status)
    assert node.pending_line_corner is None
    assert node.line_corner_rearm_required


def test_other_mission_takes_priority_and_clears_corner(clock):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    ball = dict(detected=True, confidence=.9, depth_valid=True, depth_age_sec=.05,
                distance_m=1., depth_m=1., bearing_deg=0., offset_x_norm=0.)
    command = node.publish_vision(ball=ball, line={'detected': False})[-1]
    assert command['source'] == 'ball'
    assert node.pending_line_corner is None


def test_phase_override_discards_corner(clock):
    node = CornerHarness()
    start_line(node)
    receive_line(node, clock, corner_info())
    node.send_phase('BALL_APPROACH')
    assert node.pending_line_corner is None


def test_memory_rechecks_corner_before_queueing_walking_correction(clock):
    node = CornerHarness()
    start_line(node)
    assert MotionDecisionNode._line_only_prequeue_allowed(node, {})
    receive_line(node, clock, corner_info())
    assert MotionDecisionNode._line_only_prequeue_allowed(node, {})
    count = len(node.publisher.messages)
    decision = node.planner.plan('AUTO', {'line': node.latest_info['line']}, .1)
    MotionDecisionNode._publish_decision(node, decision, queue_while_locked=True)
    assert len(node.publisher.messages) == count + 1
    assert node.publisher.messages[-1]['action'] == 'RIGHT'
    assert node.publisher.messages[-1]['reason'] == 'line_tracking'


def test_already_published_straight_is_finished_before_remembered_corner(clock):
    node = CornerHarness()
    first = start_line(node)
    receive_line(node, clock, line_info())
    MotionDecisionNode._publish_decision(node, node.last_selected_decision, queue_while_locked=True)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] != first['command_id']
    receive_line(node, clock, corner_info())
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    count = len(node.publisher.messages)
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == count
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    receive_line(node, clock, {'detected': False})
    node.send_status(queued['action'], queued['command_id'], 'SUCCEEDED')
    assert node.publish_vision(line={'detected': False})[-1]['action'] == 'WAIT'
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'
    receive_line(node, clock, corner_info())
    assert node.publish_vision()[-1]['action'] == 'RIGHT'


def test_same_corner_is_not_remembered_again_until_visible_clear(clock):
    node = CornerHarness()
    first = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, first)
    receive_line(node, clock, corner_info())
    turn = node.publish_vision()[-1]
    node.send_status(turn['action'], turn['command_id'], 'RUNNING')
    receive_line(node, clock, corner_info())
    assert node.pending_line_corner is None
    release_general(node, turn)
    assert node.planner.last_line_seen_direction is None
    receive_line(node, clock, corner_info())
    repeated_corner = node.publish_vision()[-1]
    assert repeated_corner['action'] == 'WAIT'
    assert repeated_corner['reason'] == 'line_corner_waiting_for_clear'
    receive_line(node, clock, {'detected': False})
    receive_line(node, clock, corner_info())
    assert node.pending_line_corner is None
    receive_line(node, clock, {**line_info(), 'corner_preview_confirmed': False})
    receive_line(node, clock, corner_info('LEFT'))
    assert node.pending_line_corner['corner_direction'] == 'LEFT'


def test_remembered_walking_corner_has_no_stationary_turn_settle(clock):
    node = CornerHarness()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    receive_line(node, clock, corner_info())
    assert node.publish_vision()[-1]['action'] == 'RIGHT'
    assert node.pre_motion_settle_started_at is None
    assert node.pending_line_corner is None


def test_remembered_corner_cannot_bypass_executor_fault(clock):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    node.safety_interlock.observe_critical_fault(
        error_code='SDK_HARDWARE_NOT_READY', message='test fault', source='executor',
        action='STRAIGHT', command_id=current['command_id'], event_id=None,
    )
    assert node.publish_vision(line={'detected': False}) == []


@pytest.mark.parametrize('phase', ['POST_BALL_LINE_ALIGN', 'POST_SHOT_LINE_ALIGN'])
def test_route_specific_alignment_does_not_remember_normal_corners(clock, phase):
    node = CornerHarness()
    start_line(node)
    node.phase_manager.set_phase(phase)
    receive_line(node, clock, corner_info())
    assert getattr(node, 'pending_line_corner', None) is None


@pytest.mark.parametrize('distance,action', [
    (.14, 'RIGHT'), (.15, 'RIGHT'), (.1501, 'STRAIGHT_1'),
    (.263, 'STRAIGHT_1'), (.264, 'STRAIGHT_1'),
    (.413, 'STRAIGHT_1'), (.414, 'STRAIGHT_1'),
    (.87, 'STRAIGHT_1'), (.9777, 'STRAIGHT_1'), (2., 'STRAIGHT_1'),
])
@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK'])
def test_stationary_corner_uses_distance_before_turn(clock, distance, action, phase):
    node = CornerHarness(phase=phase)
    result = node.publish_vision(line=corner_info(corner_start_distance_m=distance))[-1]
    assert result['action'] == action
    assert result['valid']
    if action.startswith('STRAIGHT'):
        assert result['reason'] == 'line_corner_turn_too_far'
        assert node.pending_line_corner is not None


def test_video_recovery_then_distant_corner_approaches_and_reobserves(clock):
    node = CornerHarness()
    current = node.publish_vision(line=line_info(heading=20., offset=.3))[-1]
    assert current['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'
    node.send_status(current['action'], current['command_id'], 'RUNNING')
    receive_line(node, clock, corner_info(corner_start_distance_m=.9777))
    release_general(node, current)
    # A recent walking frame remains usable at the completion boundary.
    forward = node.publish_vision(hurdle={'detected': False})[-1]
    assert forward['action'] == 'STRAIGHT_1'
    assert node.pending_line_corner['corner_start_distance_m'] == .9777
    clock[0] += 1.
    release_general(node, forward)
    assert node.publish_vision(hurdle={'detected': False})[-1]['action'] == 'WAIT'
    receive_line(node, clock, corner_info(corner_start_distance_m=.24))
    short = node.publish_vision()[-1]
    assert short['action'] == 'STRAIGHT_1'
    release_general(node, short)
    receive_line(node, clock, corner_info(corner_start_distance_m=.14))
    assert node.publish_vision()[-1]['action'] == 'RIGHT'
    assert node.pending_line_corner is None


@pytest.mark.parametrize('updates', [
    {'corner_start_distance_m': None}, {'corner_start_distance_m': -1.},
    {'corner_start_distance_m': 0.}, {'corner_start_distance_m': True},
    {'corner_start_distance_m': float('nan')}, {'corner_start_distance_m': float('inf')},
    {'corner_start_depth_valid': False}, {'corner_preview_held': True},
    {'corner_preview_confirmed': False}, {'ground_heading_error_deg': None},
    {'detected': False}, {'heading_quality': .1}, {'corner_direction': 'LEFT'},
])
def test_bad_or_conflicting_current_geometry_does_not_authorize_corner(clock, updates):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    receive_line(node, clock, corner_info(**updates))
    result = node.publish_vision()[-1]
    assert result['action'] == 'WAIT' and not result['valid']
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'


def test_corner_distance_expires_even_when_other_vision_is_fresh(clock):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    receive_line(node, clock, corner_info())
    clock[0] += node.timeouts['line'] + .01
    result = node.publish_vision(hurdle={'detected': False})[-1]
    assert result['action'] == 'WAIT' and not result['valid']


def test_late_arriving_frame_captured_before_completion_cannot_authorize_turn(clock):
    node = CornerHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    node.pending_line_corner['minimum_rgb_stamp_ns'] = 100
    receive_line(node, clock, corner_info(rgb_stamp_ns=99))
    assert node.publish_vision()[-1]['action'] == 'WAIT'
    receive_line(node, clock, corner_info(rgb_stamp_ns=101))
    assert node.publish_vision()[-1]['action'] == 'RIGHT'


def test_lost_corner_waits_for_observation_then_walks_without_extra_settle(clock):
    node = CornerHarness()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    assert node.publish_vision(line={'detected': False})[-1]['action'] == 'WAIT'
    receive_line(node, clock, corner_info())
    assert node.publish_vision()[-1]['action'] == 'RIGHT'


def test_distant_corner_preserves_local_recovery(clock):
    node = CornerHarness()
    info = corner_info(corner_start_distance_m=.87, ground_heading_error_deg=-20.,
                       filtered_lateral_offset_norm=-.3)
    result = node.publish_vision(line=info)[-1]
    assert result['action'] == 'RECOVER_LEFT_TURN_LEFT_4'
    assert node.pending_line_corner is not None


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('updates,ordinary_action', [
    ({'ground_heading_error_deg': 0., 'turn_angle_deg': 0.}, 'STRAIGHT'),
    ({'ground_heading_error_deg': -20., 'filtered_lateral_offset_norm': -.3,
      'turn_angle_deg': 0.},
     'RECOVER_LEFT_TURN_LEFT_4'),
    ({'ground_heading_error_deg': 75.83}, 'LINE_HEADING_TURN_RIGHT_7'),
    ({'ground_heading_error_deg': 31.99, 'filtered_lateral_offset_norm': -.795,
      'lateral_offset_px': -507.9, 'offset_reference_valid': True,
      'offset_reference_steering_deg': -75.8, 'turn_angle_deg': 3.6},
     'STOP'),
    ({'ground_heading_error_deg': 0., 'filtered_heading_error_deg': 0.,
      'heading_error_deg': 55., 'turn_angle_deg': 0.}, 'STRAIGHT'),
])
def test_near_confirmed_corner_preserves_normal_non_corner_decision(
    clock, side, updates, ordinary_action,
):
    node = CornerHarness(phase='LINE_TRACK')
    info = corner_info(side, **updates)
    ordinary = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    assert ordinary.action == ordinary_action
    receive_line(node, clock, info)
    result = MotionDecisionNode._apply_pending_line_corner(node, ordinary, info)
    assert result.action == ordinary.action
    assert result.valid == ordinary.valid
    assert result.reason == ordinary.reason
    assert result.source_command == ordinary.source_command


@pytest.mark.parametrize('updates', [
    {'corner_start_distance_m': .1501}, {'corner_start_distance_m': None},
    {'corner_start_depth_valid': False}, {'corner_preview_confirmed': False},
    {'corner_preview_held': True}, {'corner_direction': 'LEFT'},
])
def test_remembered_direction_alone_cannot_replace_normal_forward(clock, updates):
    node = CornerHarness(phase='LINE_TRACK')
    remembered = corner_info(corner_start_distance_m=.8)
    receive_line(node, clock, remembered)
    info = corner_info(ground_heading_error_deg=0., turn_angle_deg=0., **updates)
    result = node.publish_vision(line=info)[-1]
    assert result['valid'] and result['action'] == 'STRAIGHT'
    assert node.pending_line_corner['corner_direction'] == 'RIGHT'


def test_pixel_reference_missing_does_not_block_valid_normal_forward(clock):
    node = CornerHarness(phase='LINE_TRACK')
    node.planner.config = replace(node.planner.config, line_offset_align_enter_px=100.)
    info = corner_info(ground_heading_error_deg=0., turn_angle_deg=0.,
                       lateral_offset_px=None, offset_reference_valid=False)
    result = node.publish_vision(line=info)[-1]
    assert result['valid'] and result['action'] == 'STRAIGHT'
    assert result['reason'] == 'line_tracking'


def test_selected_corner_switch_waits_for_motion_then_resumes_without_settle(clock):
    node = CornerHarness(phase='LINE_TRACK')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    current = start_line(node)
    near = corner_info()
    receive_line(node, clock, near)
    assert node.publish_vision() == []
    release_general(node, current)
    turn = node.publish_vision()[-1]
    assert turn['action'] == 'RIGHT' and turn['reason'] == 'line_tracking'
    assert node.pending_line_corner is None
    release_general(node, turn)
    receive_line(node, clock, corner_info(ground_heading_error_deg=0., turn_angle_deg=0.))
    assert node.publish_vision()[-1]['action'] == 'STRAIGHT'


def test_corner_short_approach_alias_resolves_to_existing_camera45_gait():
    import json
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    by_name = {motion['name']: motion for motion in catalog}
    for action, equivalent in [('STRAIGHT_1', 'ball_camera_down_forward_2'),
                               ('STRAIGHT_2', 'line_forward_4')]:
        alias = MotionCommandBridgeNode.ACTION_TO_MOTION_ID[action]
        assert aliases[alias] == aliases[equivalent]
        assert aliases[alias] in by_name


@pytest.mark.parametrize('stamp,allowed', [
    ({'sec': 10, 'nanosec': 0}, True),
    ({'sec': 9, 'nanosec': 600000000}, True),
    ({'sec': 9, 'nanosec': 0}, False),
    ({'sec': 11, 'nanosec': 0}, False),
    ({'sec': True, 'nanosec': 0}, False), (None, False),
])
def test_production_clock_requires_recent_capture_stamp(clock, monkeypatch, stamp, allowed):
    node = CornerHarness()
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: 10_000_000_000)
    result = node.publish_vision(line=corner_info(stamp=stamp))[-1]
    assert result['valid'] is allowed
    assert result['action'] == ('RIGHT' if allowed else 'WAIT')


def test_first_corner_seen_after_discrete_turn_rejects_delayed_capture(clock, monkeypatch):
    node = CornerHarness()
    action = 'LINE_HEADING_TURN_RIGHT_2'
    node.general_motion_gate.on_command_published(action, 100)
    node.active_general_source = 'line'
    node.send_status(action, 100, 'RUNNING')
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: 10_000_000_000)
    node.send_status(action, 100, 'SUCCEEDED')
    receive_line(node, clock, corner_info(stamp={'sec': 9, 'nanosec': 900000000}))
    info = node.latest_info['line']
    decision = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    result = MotionDecisionNode._apply_pending_line_corner(node, decision, info)
    assert result.action == 'WAIT'
    assert result.source_command['line_input_checks']['captured_after_motion'] is False
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: 10_100_000_000)
    receive_line(node, clock, corner_info(stamp={'sec': 10, 'nanosec': 50000000}))
    info = node.latest_info['line']
    decision = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    assert MotionDecisionNode._apply_pending_line_corner(node, decision, info).action == 'RIGHT'


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
def test_corner_allows_fresh_loss_search_but_keeps_budget_and_memory(clock, side):
    node = CornerHarness(phase='LINE_TRACK')
    current = start_line(node)
    # The nearest point is on the opposite side of the actual bend.
    x = 800 if side == 'LEFT' else 500
    receive_line(node, clock, corner_info(side, center_points_px=[[x, 700], [640, 400]]))
    release_general(node, current)
    small = 1 if side == 'LEFT' else 2
    sequence = [(side, small)] * 3
    for direction, repeats in sequence:
        expected = f'LINE_LOST_TURN_{direction}_{repeats}'
        clock[0] += .1
        command = node.publish_vision(line={'detected': False, 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
        assert command['action'] == expected
        assert command['source_command']['corner_search_pending']
        assert node.pending_line_corner['corner_direction'] == side
        assert command['source_command']['turn_angle_deg'] == 15.
        assert command['source_command']['lost_search_max_angle_deg'] == 45.
        count = len(node.publisher.messages)
        assert node.publish_vision(line={'detected': False, 'rgb_stamp_ns': round((clock[0] + .01) * 1e9)}) == []
        assert len(node.publisher.messages) == count
        release_general(node, command)
        # The just-consumed observation must not authorize the next turn.
        assert not any(c['valid'] for c in node.publish_vision())
    clock[0] += .1
    blocked = node.publish_vision(line={'detected': False, 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
    assert blocked['reason'] == 'lost_search_turn_limit_reached'
    assert not blocked['valid']
    assert node.lost_search_turn_limiter.angles['line'] == 45.
    # Reacquiring Line alone cannot authorize advancing toward the remembered bend.
    receive_line(node, clock, {**line_info(), 'corner_preview_confirmed': False})
    forward = node.publish_vision()[-1]
    assert forward['action'] == 'STRAIGHT'
    release_general(node, forward)
    receive_line(node, clock, corner_info(side))
    assert node.publish_vision()[-1]['action'] == side


@pytest.mark.parametrize('stamp', [None, True, 10_000_000_000, 10_010_000_000, 99_000_000_000])
def test_corner_loss_search_requires_current_post_motion_capture(clock, monkeypatch, stamp):
    node = CornerHarness(phase='LINE_TRACK')
    current = start_line(node)
    receive_line(node, clock, corner_info())
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', staticmethod(lambda node: round(clock[0] * 1e9)))
    release_general(node, current)
    clock[0] += 1.
    command = node.publish_vision(line={'detected': False, 'rgb_stamp_ns': stamp})[-1]
    assert command['action'] == 'WAIT' and not command['valid']
    assert node.pending_line_corner is not None
