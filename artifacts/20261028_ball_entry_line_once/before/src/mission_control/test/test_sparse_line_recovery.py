"""Verify sparse-geometry recovery cannot bypass freshness, motion or retry gates."""

import copy
import json
import math

import pytest

from mission_control.sparse_line_recovery import SparseLineRecovery
from mission_control.motion_decision_planner import MotionDecision, MotionDecisionPlanner
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_command_gate import normalize_general_action
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def sparse_info(heading=8., direction="RIGHT", distance=.4):
    dz = .08
    points = [[.08, .4], [.08 + dz * math.tan(math.radians(heading)), .48]]
    return dict(
        detected=True, ground_projection_enabled=True, ground_projection_valid=False,
        ground_heading_error_deg=None, ground_fit_reason="too_few_segment_points",
        ground_fit_segment="PRE_CORNER", ground_fit_input_point_count=2,
        ground_two_point_candidate={"heading_deg": heading, "points_m": points},
        corner_preview_raw_detected=True, corner_preview_confirmed=True,
        corner_preview_held=False, corner_start_depth_valid=True,
        corner_direction=direction, corner_start_distance_m=distance,
        heading_quality=.8, geometry_quality=.8, detection_quality=.8,
        filtered_lateral_offset_norm=.29, heading_error_deg=12.9,
        lateral_offset_px=0.,
        filtered_heading_error_deg=12.9, turn_angle_deg=78.2, turn_consistency=.9,
    )


def prepare(policy, info, time, **overrides):
    args = dict(stamp=int(time * 1e9), ros_now=int(time * 1e9), now=time,
                max_age=.5, stationary=True)
    args.update(overrides)
    return policy.prepare(info, **args)


def wait_decision(reason="line_corner_waiting_for_usable_line"):
    return MotionDecision(phase="LINE_TRACK", source="line", action="WAIT", valid=False,
                          reason=reason, sdk_motion_requested=False, requires_ack=False,
                          source_command={})


def test_two_point_heading_requires_three_distinct_captures_and_preserves_raw_input():
    policy, info = SparseLineRecovery(), sparse_info()
    original = copy.deepcopy(info)
    assert not prepare(policy, info, 10.)["ground_projection_valid"]
    for _ in range(8):
        assert not prepare(policy, info, 10.)["ground_projection_valid"]
    assert not prepare(policy, info, 10.1)["ground_projection_valid"]
    promoted = prepare(policy, info, 10.2)
    assert promoted["ground_projection_valid"]
    assert promoted["ground_fit_point_count"] == 2
    assert promoted["ground_heading_error_deg"] == 8.
    assert info == original


@pytest.mark.parametrize("updates", [
    {"stationary": False}, {"stamp": None}, {"stamp": True},
    {"ros_now": None}, {"stamp": 9_000_000_000}, {"stamp": 12_000_000_000},
])
def test_invalid_capture_cannot_confirm_or_escape(updates):
    policy, info = SparseLineRecovery(), sparse_info()
    for t in (10., 10.1, 10.2, 11.5):
        assert not prepare(policy, info, t, **updates)["ground_projection_valid"]
    assert not policy.constrain(wait_decision(), info, 12.).valid


@pytest.mark.parametrize("updates", [
    {"heading_quality": .2}, {"ground_two_point_candidate": None},
    {"corner_preview_held": True}, {"corner_preview_confirmed": False},
    {"corner_preview_raw_detected": False}, {"corner_start_depth_valid": False},
    {"corner_direction": None}, {"detected": False},
    {"ground_projection_enabled": False}, {"ground_projection_valid": "false"},
    {"ground_fit_reason": "too_few_projected_points"},
])
def test_bad_geometry_and_unconfirmed_corners_never_escape(updates):
    policy, info = SparseLineRecovery(), {**sparse_info(), **updates}
    for t in (10., 10.2, 10.4, 10.6, 10.8, 11.1):
        prepare(policy, info, t)
    assert not policy.constrain(wait_decision(), info, 11.1).valid


@pytest.mark.parametrize("direction", ["RIGHT", "LEFT"])
def test_unstable_two_point_heading_never_escapes_into_motion(direction):
    policy = SparseLineRecovery()
    for i, t in enumerate((10., 10.2, 10.4, 10.6, 10.8, 11.1)):
        info = sparse_info(heading=(2. if i % 2 else 12.) * (1 if direction == "RIGHT" else -1),
                           direction=direction)
        prepare(policy, info, t)
        assert not policy.constrain(wait_decision(), info, t).valid
    assert not policy.stable
    assert policy.motions_used == 0


def test_normal_fit_three_new_frames_rearms_budget_but_invalid_frames_do_not():
    policy = SparseLineRecovery()
    policy.motions_used = 2
    info = sparse_info()
    for t in (10., 10.1, 10.2):
        prepare(policy, info, t)
    assert not policy.constrain(wait_decision(), info, 10.2).valid
    assert policy.motions_used == 2
    normal = {**info, "ground_projection_valid": True, "ground_fit_point_count": 3,
              "ground_heading_error_deg": 0.}
    for t in (10.3, 10.4):
        prepare(policy, normal, t)
        assert policy.motions_used == 2
    prepare(policy, normal, 10.5)
    assert policy.motions_used == 0


@pytest.mark.parametrize("reason", ["invalid_vision_boolean_type", "low_line_quality",
                                   "line_corner_waiting_for_fresh_distance"])
def test_wait_escape_cannot_override_other_failure_reasons(reason):
    policy, info = SparseLineRecovery(), sparse_info()
    for t in (10., 10.2, 10.4, 10.6, 10.8, 11.1):
        prepare(policy, info, t)
    assert not policy.constrain(wait_decision(reason), info, 11.1).valid


def test_two_point_straight_is_short_and_requires_corner_clearance():
    policy, info = SparseLineRecovery(), sparse_info(heading=0., distance=.6)
    info.update(filtered_lateral_offset_norm=0., heading_error_deg=0., turn_angle_deg=0.)
    for t in (10., 10.1, 10.2):
        promoted = prepare(policy, info, t)
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": promoted}, .1)
    assert decision.action == "STRAIGHT"
    result = policy.constrain(decision, promoted, 10.2)
    assert result.action == "LINE_SPARSE_FORWARD"
    assert MotionCommandBridgeNode.motion_id_for_action(result.action) == "line_forward_2"
    assert MotionDecisionNode._needs_correction_dwell(result.action)
    assert not policy.constrain(decision, {**promoted, "corner_start_distance_m": .15}, 10.2).valid


def test_stationary_confirmation_and_motion_completion_require_new_captures(monkeypatch):
    from test_line_ground_fit_recovery import unconfirmed_sparse_info
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda self: int(clock[0] * 1e9))
    node = LiveInputHarness(phase="LINE_TRACK")
    # Keep this recovery-boundary test outside the calibrated corner entry.
    node.line_corner_turn_distance_m = .15
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    # Current image geometry requests a small turn without promoting a ground fit.
    info = {**unconfirmed_sparse_info(), "turn_angle_deg": 0.}
    def frame():
        observe(node, clock, "line", {**info, "rgb_stamp_ns": int((clock[0] + .005) * 1e9)})
    frame()
    frame()
    assert not any(m["valid"] for m in node.publisher.messages)
    frame()
    assert node.last_selected_decision.action == "LINE_OFFSET_TURN_RIGHT_2"
    assert node.sparse_line_recovery.motions_used == 0
    for _ in range(6):
        clock[0] += .2
        frame()
    command = next(m for m in node.publisher.messages if m["valid"])
    assert command["action"] == "LINE_OFFSET_TURN_RIGHT_2"
    assert node.sparse_line_recovery.motions_used == 0
    node.send_status(command["action"], command["command_id"], "RUNNING")
    node.send_status(command["action"], command["command_id"], "SUCCEEDED")
    assert node.latest_info["line"] is None
    assert node.correction_post_motion_dwell_until > clock[0]
    clock[0] = node.correction_post_motion_dwell_until + .01
    MotionDecisionNode._publish_decision(node)
    assert node.line_offset_min_rgb_stamp_ns == int(clock[0] * 1e9)
    frame()
    assert not node.sparse_line_recovery.samples
    assert node.latest_info["line"]["ground_projection_valid"] is False


def test_failed_motion_and_exhausted_budget_never_authorize_another_peek():
    for failed, count in ((True, 1), (False, 2)):
        policy, info = SparseLineRecovery(), sparse_info()
        policy.failed, policy.motions_used = failed, count
        for t in (10., 10.2, 10.4, 10.6, 10.8, 11.1):
            assert not prepare(policy, info, t)["ground_projection_valid"]
        result = policy.constrain(wait_decision(), info, 11.1)
        assert not result.valid
        assert result.reason == ("sparse_line_motion_failed" if failed else "sparse_line_motion_limit_reached")


def test_position_jitter_cannot_confirm_even_with_identical_heading():
    policy = SparseLineRecovery()
    for i, t in enumerate((10., 10.1, 10.2)):
        info = sparse_info()
        for p in info["ground_two_point_candidate"]["points_m"]:
            p[0] += i * .02
        assert not prepare(policy, info, t)["ground_projection_valid"]


def test_gap_and_out_of_order_capture_discard_confirmation():
    policy, info = SparseLineRecovery(), sparse_info()
    prepare(policy, info, 10.)
    prepare(policy, info, 10.1)
    assert not prepare(policy, info, 11.)["ground_projection_valid"]
    assert len(policy.samples) == 1
    prepare(policy, info, 10.9)
    assert not policy.samples


def test_normal_commands_can_rearm_budget_across_motion_boundaries():
    policy = SparseLineRecovery()
    policy.motions_used = 2
    normal = {**sparse_info(), "ground_projection_valid": True,
              "ground_fit_point_count": 3, "ground_heading_error_deg": 0.}
    for t in (10., 12., 14.):
        prepared = prepare(policy, normal, t)
        decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": prepared}, .1)
        assert decision.valid
        policy.published(decision, int(t * 1e9))
    assert policy.motions_used == 0


def test_configured_quality_threshold_applies_to_sparse_recovery():
    policy, info = SparseLineRecovery(), sparse_info()
    for t in (10., 10.1, 10.2):
        assert not prepare(policy, info, t, min_quality=.9)["ground_projection_valid"]
    assert not policy.eligible


def test_recorded_two_point_corner_uses_confirmed_short_forward(monkeypatch):
    import rclpy
    from std_msgs.msg import String
    from step.yolo_line_analyzer import YoloLineAnalyzer

    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda self: int(clock[0] * 1e9))
    node = LiveInputHarness(phase="LINE_TRACK")
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    rclpy.init()
    analyzer = None
    try:
        analyzer = YoloLineAnalyzer()
        outputs = []
        monkeypatch.setattr(analyzer.publisher, "publish", lambda msg: outputs.append(json.loads(msg.data)))
        # Screen recording markers and displayed corner distance, not original RGB-D samples.
        xy = [(1375, 827), (1374, 746), (1398, 671), (1458, 614),
              (1560, 570), (1638, 552), (1732, 533), (1800, 522)]
        points = [(x - 532, y - 115) for x, y in xy]
        monkeypatch.setattr(analyzer, "_line_point_distance", lambda point: dict(
            point_px=[point.x, point.y], depth_m=.57, lateral_offset_m=.08,
            ground_forward_distance_m=.4, horizontal_distance_m=.4, depth_valid=True))
        for _ in range(25):
            clock[0] += .1
            stamp = int(clock[0] * 1e9)
            analyzer._detections_callback(String(data=json.dumps(dict(
                image_width=1280, image_height=720,
                stamp={"sec": stamp // 1_000_000_000, "nanosec": stamp % 1_000_000_000},
                detections=[dict(class_name="line", confidence=.95, center=[x,y],
                                 bbox=[x-5,y-5,x+5,y+5]) for x,y in points],
            ))))
            MotionDecisionNode._info_callback(node, "line")(String(data=json.dumps(outputs[-1])))
            MotionDecisionNode._publish_decision(node)
            if any(m["valid"] for m in node.publisher.messages):
                break
        assert outputs[-1]["ground_fit_reason"] == "too_few_segment_points"
        assert outputs[-1]["ground_fit_input_point_count"] == 2
        assert outputs[-1]["ground_two_point_candidate"] is not None
        motions = [m for m in node.publisher.messages if m["valid"]]
        assert len(motions) == 1
        assert motions[0]["action"] == "LINE_SPARSE_FORWARD"
        assert motions[0]["source_command"]["ground_fit_mode"] == "TWO_POINT_CONFIRMED"
        assert motions[0]["source_command"]["sparse_line_motion"]
    finally:
        if analyzer is not None:
            analyzer.destroy_node()
        rclpy.shutdown()
