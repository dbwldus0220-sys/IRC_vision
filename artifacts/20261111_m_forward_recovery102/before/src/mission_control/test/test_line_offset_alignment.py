"""Approach a visible ground target when the line is outside the pixel corridor."""

import math

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import line_info
from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages, navigation_message,
)
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def sample(angle=40.0, offset=200.0, *, target_angle=20.0, **changes):
    data = {
        **line_info(heading=20.0, offset=0.7),
        "lateral_offset_px": offset, "offset_reference_valid": True,
        "offset_reference_x_px": 710.0 + offset,
        "offset_reference_y_px": 590.4, "offset_reference_steering_deg": angle,
        **changes,
    }
    heading = math.radians(data.get("ground_heading_error_deg") or 0.0)
    slope = math.tan(heading)
    intercept = math.tan(math.radians(target_angle)) * .5 - slope * .5
    data.setdefault("ground_lateral_offset_m", intercept * math.cos(heading))
    data.setdefault("ground_lookahead_distance_m", .5)
    data.setdefault("ground_line_points_m", [[slope*z+intercept, z] for z in (.35, .5, .7)])
    data.setdefault("ground_fit_segment", "PRE_CORNER" if data.get("corner_preview_confirmed") else "FULL_PATH")
    return data


def planner(threshold=100.0):
    return MotionDecisionPlanner(MotionDecisionConfig(line_offset_align_enter_px=threshold))


@pytest.mark.parametrize("offset", [-600., -100., 100., 600.])
def test_pixel_offset_does_not_replace_forward(offset):
    info = sample(offset=offset, ground_heading_error_deg=0.,
                  filtered_lateral_offset_norm=0., target_angle=0.)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid and decision.action == "STRAIGHT"
    assert not decision.source_command["line_offset_alignment_active"]


@pytest.mark.parametrize("sign", [-1, 1])
def test_parallel_far_line_checks_target_even_when_heading_selects_forward(sign):
    info = sample(offset=sign * 181.7, target_angle=sign * 30.,
                  ground_heading_error_deg=-sign * 1.43,
                  filtered_lateral_offset_norm=sign * .262,
                  ground_lateral_offset_m=sign * .1594,
                  ground_lookahead_distance_m=.211,
                  ground_line_points_m=[[sign * .13, z] for z in (.15, .25, .4)])
    baseline = planner(-1.).plan("LINE_TRACK", {"line": info}, .1)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert baseline.action == "STRAIGHT"
    assert decision.valid
    assert decision.action == ("LINE_OFFSET_TURN_RIGHT_3" if sign > 0 else "LINE_OFFSET_TURN_LEFT_2")
    assert decision.reason == "line_offset_target_turn"


@pytest.mark.parametrize("offset", [99.9, 100., 182.])
@pytest.mark.parametrize("distance", [.10, .1316, .1499, .15, .1501])
def test_forward_guard_requires_both_pixel_and_ground_distance(offset, distance):
    info = sample(offset=offset, ground_heading_error_deg=0.,
                  ground_lateral_offset_m=distance)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    guarded = offset >= 100. and distance > .15
    assert decision.source_command["line_offset_alignment_active"] is guarded
    assert decision.valid
    assert decision.action == ("LINE_OFFSET_TURN_RIGHT_1" if guarded else "STRAIGHT")
    if guarded:
        assert decision.reason == "line_offset_target_turn"


@pytest.mark.parametrize('base_motion', ['STRAIGHT', 'RECOVER_RIGHT_TURN_RIGHT_4'])
@pytest.mark.parametrize('angle,expected', [
    (-15., 'LINE_OFFSET_TURN_LEFT_1'), (-14.999, 'STRAIGHT_1'),
    (-10.21, 'STRAIGHT_1'), (-8., 'STRAIGHT_1'),
    (-7.999, 'STRAIGHT_1'), (0., 'STRAIGHT_1'), (10., 'STRAIGHT_1'),
    (10.001, 'STRAIGHT_1'), (10.21, 'STRAIGHT_1'), (14.999, 'STRAIGHT_1'),
    (15., 'LINE_OFFSET_TURN_RIGHT_1'), (15.001, 'LINE_OFFSET_TURN_RIGHT_1'),
])
def test_offset_forward_and_turn_ranges_have_no_gap(monkeypatch, base_motion, angle, expected):
    controller = planner()
    monkeypatch.setattr(controller, '_line_offset_ground_target',
                        lambda info: (.17, .5, angle, .1672))
    result = controller._line_offset_alignment(
        {'lateral_offset_px': 149.6}, {'valid': True, 'motion': base_motion})
    assert result['valid']
    assert result['motion'] == expected
    direction = 'RIGHT' if angle > 0 else 'LEFT'
    assert result['target_heading_tolerance_deg'] == 15.0


def test_recorded_near_right_target_preserves_forward_and_releases_bridge():
    info = sample(offset=149.6, ground_heading_error_deg=-6.63,
                  ground_lateral_offset_m=.1472)
    decision = planner().plan('LINE_TRACK', {'line': info}, .1)
    assert decision.valid and decision.action == 'STRAIGHT'
    assert not decision.source_command['line_offset_alignment_active']
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action=decision.action, source='line', source_command=decision.source_command))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'line_forward_6'
    complete_active_motion(bridge)
    assert not bridge.motion_in_progress


def test_far_parallel_line_uses_short_forward_if_target_already_ahead():
    info = sample(ground_heading_error_deg=0., ground_lateral_offset_m=.16,
                  ground_lookahead_distance_m=1., ground_line_points_m=[[.16, z] for z in (.8, 1., 1.2)])
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.action == "STRAIGHT_1"
    assert decision.reason == "line_offset_target_forward"


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('distance', [.1499, .15, .1501])
@pytest.mark.parametrize('heading', [0., 20.])
def test_ground_distance_boundary_preserves_normal_forward_or_recovery(sign, distance, heading):
    info = sample(offset=sign * 270., ground_heading_error_deg=sign * heading,
                  ground_lateral_offset_m=sign * distance,
                  filtered_lateral_offset_norm=sign * .4)
    decision = planner().plan('LINE_TRACK', {'line': info}, .1)
    baseline = planner(-1.).plan('LINE_TRACK', {'line': info}, .1)
    assert decision.valid
    assert decision.source_command['line_offset_alignment_active'] is (distance > .15)
    if distance <= .15:
        assert decision.action == baseline.action
        assert decision.action == ('STRAIGHT' if heading == 0. else
                                   f'RECOVER_{"RIGHT" if sign > 0 else "LEFT"}_TURN_{"RIGHT" if sign > 0 else "LEFT"}_4')


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('heading', [0., 20.])
def test_fifteen_cm_candidate_reserves_once_and_promotes_without_extra_dwell(monkeypatch, sign, heading):
    from test_continuous_approach import fresh
    clock = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
    node = LiveInputHarness(phase='LINE_TRACK')
    bridge = FakeBridge(advance_transition_dwells=False)
    fresh(node, clock, 'line', line_info())
    first = node.publisher.messages[-1]
    bridge.navigation_command_callback(navigation_message(**first))
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    start = node.active_line_motion_started_at
    duration = node.active_line_motion_duration_sec
    clock[0] = start + duration * .70
    info = sample(offset=sign * 270., ground_heading_error_deg=sign * heading,
                  ground_lateral_offset_m=sign * .15,
                  filtered_lateral_offset_norm=sign * .4)
    for _ in range(12):
        fresh(node, clock, 'line', info, plan=False)
    queued = node.publisher.messages[-1]
    assert clock[0] < start + duration
    assert node.queued_general_command_id == queued['command_id'] != first['command_id']
    assert queued['action'] == ('STRAIGHT' if heading == 0. else
                                f'RECOVER_{"RIGHT" if sign > 0 else "LEFT"}_TURN_{"RIGHT" if sign > 0 else "LEFT"}_4')
    bridge.navigation_command_callback(navigation_message(**queued))
    assert not bridge.queued_request_deferred
    assert bridge.transition_dwell_until is None
    fresh(node, clock, 'line', info, plan=False)
    assert node.queued_general_command_id == queued['command_id']
    complete_active_motion(bridge)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    assert bridge.active_command_id == queued['command_id']
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize("flag", ["corner_preview_confirmed", "corner_preview_raw_detected"])
def test_new_forward_guard_preserves_corner_candidate(flag):
    info = sample(ground_heading_error_deg=0., target_angle=30., **{flag: True})
    result = {"valid": True, "motion": "STRAIGHT", "corner_prepare": False}
    guarded = planner()._line_offset_alignment(info, result)
    assert guarded["motion"] == "STRAIGHT"
    assert not guarded["line_offset_alignment_active"]


def test_far_forward_with_missing_target_does_not_blindly_advance():
    info = sample(ground_heading_error_deg=0., ground_line_points_m=None)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert not decision.valid
    assert decision.reason == "line_offset_target_invalid"


@pytest.mark.parametrize("sign", [-1, 1])
def test_recorded_near_line_now_continues_straight(sign):
    info = sample(offset=sign * 242.199, target_angle=sign * 18.8804,
                  ground_heading_error_deg=sign * 9.03,
                  filtered_lateral_offset_norm=sign * .378,
                  heading_error_deg=-sign * 8.3, turn_angle_deg=3.3)
    baseline = planner(-1.).plan("LINE_TRACK", {"line": info}, .1)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid and decision.action == baseline.action == "STRAIGHT"
    assert not decision.source_command["line_offset_alignment_active"]


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("offset", [99.9, 100., 100.1])
def test_far_recovery_guard_keeps_inclusive_pixel_threshold(sign, offset):
    info = sample(offset=sign*offset, target_angle=sign*25.,
                  ground_heading_error_deg=-sign*20., filtered_lateral_offset_norm=sign*.7)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid
    if offset < 100.:
        assert decision.action.startswith("RECOVER_")
    else:
        assert decision.action == ("LINE_OFFSET_TURN_RIGHT_1" if sign > 0 else "LINE_OFFSET_TURN_LEFT_1")
        assert abs(decision.source_command["ground_lateral_offset_m"]) > .15


@pytest.mark.parametrize("target", [-7.999, 0., 10.])
def test_far_recovery_guard_allows_short_forward_when_target_aligned(target):
    info = sample(target_angle=target, ground_heading_error_deg=-35., heading_error_deg=55.)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid and decision.action == "STRAIGHT_1"
    assert decision.reason == "line_offset_target_forward"


@pytest.mark.parametrize("heading", [-60., 60.])
def test_large_ground_heading_is_not_reversed_by_offset_target(heading):
    info = sample(ground_heading_error_deg=heading, target_angle=-heading)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid
    assert decision.action.startswith("LINE_HEADING_TURN_RIGHT" if heading > 0 else "LINE_HEADING_TURN_LEFT")


def test_guard_does_not_latch_when_next_frame_selects_normal_recovery():
    p = planner()
    first = p.plan("LINE_TRACK", {"line": sample(target_angle=25., ground_heading_error_deg=-20.)}, .1)
    assert first.action == "LINE_OFFSET_TURN_RIGHT_1"
    result = p.plan("LINE_TRACK", {"line": sample(target_angle=24., ground_heading_error_deg=24.,
                      filtered_lateral_offset_norm=.378)}, .1)
    assert result.action == "RECOVER_RIGHT_TURN_RIGHT_4"
    assert not p.line_offset_alignment_active


@pytest.mark.parametrize("changes", [
    {"ground_projection_valid": False}, {"ground_heading_error_deg": None},
    {"ground_heading_error_deg": float("nan")}, {"heading_quality": .1},
    {"ground_line_points_m": None}, {"ground_line_points_m": [[0., .5]]},
    {"ground_line_points_m": [[0., .5], [0., .5], [0., .5]]},
    {"ground_lateral_offset_m": None}, {"ground_lookahead_distance_m": -1.},
    {"corner_preview_confirmed": True, "ground_fit_segment": "FULL_PATH"},
])
def test_unusable_ground_cannot_authorize_far_recovery(changes):
    info = sample(target_angle=25., ground_heading_error_deg=-20.)
    info.update(changes)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert not decision.valid


def test_target_is_clamped_to_observed_segment_before_corner():
    info = sample(target_angle=20., ground_heading_error_deg=-20.,
                  ground_line_points_m=[[.18, .35], [.18, .4], [.18, .45]],
                  ground_lookahead_distance_m=.5,
                  corner_preview_confirmed=True, ground_fit_segment="PRE_CORNER")
    result = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert result.source_command["target_forward_m"] == .45


@pytest.mark.parametrize('held,raw,ground_valid,expected', [
    (True, False, True, True), (False, True, True, False),
    (True, True, True, False), (True, False, False, False),
])
def test_far_guard_uses_full_path_only_when_corner_is_held(held, raw, ground_valid, expected):
    info = sample(target_angle=0., ground_heading_error_deg=-35.,
                  corner_preview_confirmed=True, corner_preview_held=held,
                  corner_preview_raw_detected=raw, ground_fit_segment='FULL_PATH',
                  ground_projection_valid=ground_valid)
    result = planner().plan('LINE_TRACK', {'line': info}, .1)
    assert result.valid is expected
    if expected:
        assert result.action == 'STRAIGHT_1'


def test_failed_guard_cannot_be_bypassed_by_normal_forward():
    p = planner()
    p.line_offset_alignment_failed = True
    decision = p.plan('LINE_TRACK', {'line': sample(ground_heading_error_deg=0.,
                       filtered_lateral_offset_norm=0., target_angle=0.)}, .1)
    assert not decision.valid and decision.reason == 'line_offset_motion_failed'


@pytest.mark.parametrize("stamp", [None, True, 9_000_000_000, 10_000_000_000])
def test_alignment_refresh_rejects_frames_captured_before_boundary(monkeypatch, stamp):
    clock = [10.1]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    node = LiveInputHarness(phase="LINE_TRACK")
    node.line_offset_min_rgb_stamp_ns = 10_000_000_000
    observe(node, clock, "line", sample(rgb_stamp_ns=stamp), plan=False)
    assert node.latest_info["line"] is None
    observe(node, clock, "line", sample(rgb_stamp_ns=10_100_000_000), plan=False)
    assert node.latest_info["line"]["rgb_stamp_ns"] == 10_100_000_000


def test_alignment_dwell_sets_capture_boundary(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda self: 12_000_000_000)
    node = LiveInputHarness(phase="LINE_TRACK")
    node.correction_post_motion_dwell_until = 11.
    node.correction_post_motion_source = "line"
    node.line_offset_alignment_refresh_pending = True
    clock[0] = 12.
    MotionDecisionNode._publish_decision(node)
    assert node.line_offset_min_rgb_stamp_ns == 12_000_000_000
    assert not node.line_offset_alignment_refresh_pending


@pytest.mark.parametrize("robot", [False, True])
@pytest.mark.parametrize("override", [None, "-1.0", "180.0"])
def test_launch_exposes_pixel_threshold(monkeypatch, tmp_path, robot, override):
    from launch_ros.actions import Node
    from test_full_system_launch import node_parameters
    if robot:
        from test_full_system_robot_launch import launch_description, default_context
    else:
        from test_full_system_launch import launch_description, launch_context as default_context
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    assert context.launch_configurations["line_offset_align_enter_px"] == "100.0"
    if override is not None:
        context.launch_configurations["line_offset_align_enter_px"] = override
    node = next(e for e in description.entities if isinstance(e, Node)
                and e.node_executable == "motion_decision_node")
    expected = 100. if override is None else float(override)
    assert node_parameters(node, context)["line_offset_align_enter_px"] == expected



def test_alignment_failure_status_latches_until_other_mission(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda _: round(clock[0] * 1e9))
    node = LiveInputHarness(phase="LINE_TRACK")
    info = sample(target_angle=0., ground_heading_error_deg=-30.)
    observe(node, clock, "line", {**info, "rgb_stamp_ns": 10_010_000_000})
    command = node.publisher.messages[-1]
    assert command["action"] == "STRAIGHT_1"
    node.send_status(command["action"], command["command_id"], "RUNNING")
    node.send_status(command["action"], command["command_id"], "FAILED")
    assert node.planner.line_offset_alignment_failed
    valid_count = sum(m["valid"] for m in node.publisher.messages)
    for _ in range(3):
        observe(node, clock, "line", {**info, "rgb_stamp_ns": round((clock[0] + .01)*1e9)})
    assert sum(m["valid"] for m in node.publisher.messages) == valid_count
    assert node.planner.line_offset_alignment_failed
