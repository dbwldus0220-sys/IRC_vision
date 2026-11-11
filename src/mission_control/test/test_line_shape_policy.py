"""Replay sparse confirmation and corner ownership through the live decision path."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecision
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.visible_sparse_line import VisibleSparseLineObserver
from test_line_ground_fit_recovery import clock, frame, node_for
from test_corner_exit_continuity import exit_info


def visible_info(side="RIGHT", count=2, **updates):
    x = 980 if side == "RIGHT" else 440
    points = [[x, 550], [x - 10, 420]][:count]
    result = dict(
        detected=False, raw_detected=False, ground_projection_enabled=True,
        ground_projection_valid=False, ground_fit_reason="too_few_segment_points",
        image_width=1280, image_height=720, robot_center_x_px=710,
        visible_line_points_px=points, visible_line_confidences=[.9] * count,
        corner_visible_outgoing_fraction=None, center_points_px=[],
        heading_quality=0., geometry_quality=0., detection_quality=0.,
        corner_preview_confirmed=False, corner_preview_raw_detected=False,
    )
    result.update(updates)
    return result


def observe(observer, info, t, **changes):
    args = dict(stamp=int(t * 1e9), ros_now=int(t * 1e9), now=t,
                minimum_stamp=0, max_age=.5, stationary=True)
    args.update(changes)
    observer.observe(info, **args)


@pytest.mark.parametrize("count", [1, 2])
@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
def test_sparse_wait_and_three_new_captures(count, side):
    observer = VisibleSparseLineObserver()
    info = visible_info(side, count)
    for t in (10., 10.2, 10.4):
        observe(observer, info, t)
        assert not observer.ready
    observe(observer, info, 10.6)
    assert observer.ready and observer.direction == side


def test_duplicate_capture_never_confirms_and_motion_discards_evidence():
    observer = VisibleSparseLineObserver()
    for t in (10., 10.1, 10.2, 10.4):
        observe(observer, visible_info(), t, stamp=10_000_000_000)
    assert not observer.ready and len(observer.samples) == 1
    observe(observer, visible_info(), 10.5, stationary=False)
    assert not observer.samples


@pytest.mark.parametrize("changes", [
    {"stamp": 1}, {"stamp": True}, {"stamp": 11_000_000_000},
    {"minimum_stamp": 11_000_000_000}, {"stationary": False},
])
def test_stale_future_pre_motion_or_moving_frames_never_authorize_view_turn(changes):
    observer = VisibleSparseLineObserver()
    for t in (10., 10.2, 10.4, 10.6):
        observe(observer, visible_info(), t, **changes)
    assert not observer.ready


@pytest.mark.parametrize("updates", [
    {"visible_line_confidences": [.2, .9]},
    {"visible_line_points_px": [[True, 400], [900, 300]]},
    {"visible_line_points_px": [[710, 550], [710, 420]]},
    {"visible_line_points_px": [[float('nan'), 550], [900, 420]]},
])
def test_invalid_or_centered_sparse_points_do_not_guess_direction(updates):
    observer = VisibleSparseLineObserver()
    for t in (10., 10.2, 10.4, 10.6):
        observe(observer, visible_info(**updates), t)
    assert not observer.ready


def test_jitter_and_direction_change_restart_wait():
    observer = VisibleSparseLineObserver()
    for i, t in enumerate((10., 10.2, 10.4, 10.6)):
        observe(observer, visible_info("RIGHT" if i % 2 else "LEFT"), t)
    assert not observer.ready and len(observer.samples) == 1


@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("count", [1, 2])
def test_sparse_replay_selects_stationary_turn_never_recovery_or_corner(clock, side, count):
    node = node_for()
    info = visible_info(side, count)
    start = clock[0]
    commands = []
    for elapsed in (0., .2, .4, .6, .8, 1., 1.2, 1.4, 1.6):
        clock[0] = start + elapsed
        commands += frame(node, clock, info)
        valid = [c for c in commands if c["valid"]]
        if valid:
            break
    assert valid, node.last_selected_decision
    command = valid[-1]
    assert command["action"] == f"LINE_OFFSET_TURN_{side}_1"
    assert command["source_command"]["sparse_view_turn"]
    assert Bridge.motion_id_for_action(command["action"]) == f"post_ball_line_turn_{side.lower()}_1"
    assert commands[0]["action"] == "WAIT"
    assert getattr(node, "pending_line_corner", None) is None
    node.send_status(command["action"], command["command_id"], "RUNNING")
    clock[0] += .2
    assert not any(c["valid"] for c in frame(node, clock, info))
    node.send_status(command["action"], command["command_id"], "SUCCEEDED")
    assert node.latest_info["line"] is None


def shape_info(angle=60., **changes):
    result = exit_info(angle, distance=changes.pop("distance", .14))
    result.update(
        visible_line_points_px=[[710, 650], [710, 600], [900, 550], [1100, 500]],
        visible_line_confidences=[.9] * 4, corner_visible_outgoing_fraction=.8,
        image_height=720)
    result.update(changes)
    return result


def apply(node, clock, info, action="RECOVER_LEFT_TURN_LEFT_4"):
    clock[0] += .1
    info = {**info, "rgb_stamp_ns": int(clock[0] * 1e9)}
    node.latest_time["line"] = clock[0]
    decision = MotionDecision("LINE_TRACK", "line", action, True, "line_center_recovery", False, False, {})
    return Node._apply_pending_line_corner(node, decision, info)


@pytest.mark.parametrize("side,angle", [("LEFT", -60.), ("RIGHT", 60.)])
@pytest.mark.parametrize("ordinary", ["STRAIGHT", "RECOVER_LEFT_TURN_LEFT_4", "LINE_HEADING_TURN_RIGHT_7"])
def test_dominant_corner_owns_motion_after_three_distinct_observations(clock, side, angle, ordinary):
    node = node_for()
    for _ in range(2):
        result = apply(node, clock, shape_info(angle), ordinary)
        assert not result.valid and result.reason == "line_corner_confirming_dominance"
    result = apply(node, clock, shape_info(angle), ordinary)
    assert result.valid and result.action == side
    assert result.source_command["corner_shape_turn"]
    assert Bridge.motion_id_for_action(side) == f"line_recovery_{side.lower()}_4"


@pytest.mark.parametrize("updates", [
    {"corner_visible_outgoing_fraction": .59}, {"distance": .151},
    {"corner_start_lateral_offset_m": .151},
])
def test_incoming_segment_owns_alignment_before_entry(clock, updates):
    node = node_for()
    for _ in range(4):
        result = apply(node, clock, shape_info(**updates))
        assert result.action == "RECOVER_LEFT_TURN_LEFT_4"
        assert not result.source_command.get("corner_shape_turn")


def test_image_exit_direction_supports_three_point_corner(clock):
    node = node_for()
    info = shape_info(corner_exit_ground_valid=False, corner_exit_ground_heading_deg=None)
    for _ in range(3):
        result = apply(node, clock, info)
    assert result.action == "RIGHT"
    assert result.source_command["corner_heading_reference"] == "image_exit"


def test_dominant_corner_still_shortens_forward_approach(clock):
    result = apply(node_for(), clock, shape_info(distance=.4), "STRAIGHT")
    assert result.action == "STRAIGHT_1" and result.reason == "line_corner_continuous_approach"


def test_stale_capture_does_not_confirm_corner(clock):
    node = node_for()
    apply(node, clock, shape_info())
    node.pending_line_corner["minimum_rgb_stamp_ns"] = int((clock[0] + 1) * 1e9)
    assert not apply(node, clock, shape_info()).valid


def test_active_corner_does_not_repeat_without_measured_progress(clock):
    node = node_for()
    apply(node, clock, shape_info())
    node.pending_line_corner.update(turning=True, turns_used=1,
        last_turn_heading_deg=60., heading_reference="ground_exit")
    for _ in range(4):
        result = apply(node, clock, shape_info(59.))
        assert not result.valid and result.reason == "line_corner_turn_limit_or_no_progress"
    for _ in range(3):
        result = apply(node, clock, shape_info(40.))
    assert result.valid and result.action == "RIGHT"


def test_sparse_visibility_blocks_old_frame_prequeue(clock):
    node = node_for()
    assert Node._line_prequeue_observation(node, visible_info(), clock[0]) is None
    assert not Node._line_frame_is_usable(node, visible_info(detected=True, ground_projection_valid=True))


@pytest.mark.parametrize("side,used_yaw", [("LEFT", 120.), ("RIGHT", 120.)])
def test_corner_yaw_budget_still_blocks_improving_exit(clock, side, used_yaw):
    node = node_for()
    sign = 1 if side == "RIGHT" else -1
    apply(node, clock, shape_info(sign * 60.))
    node.pending_line_corner.update(turning=True, turns_used=4, yaw_used_deg=used_yaw,
                                   last_turn_heading_deg=sign * 80., heading_reference="ground_exit")
    assert not apply(node, clock, shape_info(sign * 60.)).valid


def test_normal_line_after_sparse_wait_cancels_view_turn(clock):
    node = node_for()
    for _ in range(2):
        assert not any(c["valid"] for c in frame(node, clock, visible_info()))
        clock[0] += .2
    straight = shape_info(corner_preview_confirmed=False, corner_preview_raw_detected=False,
                          turn_angle_deg=0., ground_heading_error_deg=0.)
    commands = frame(node, clock, straight)
    assert commands[-1]["valid"] and commands[-1]["action"].startswith("STRAIGHT")
    assert not commands[-1]["source_command"].get("sparse_view_turn")


def test_live_corner_retains_direction_until_three_exit_frames(clock):
    node = node_for()
    start = clock[0]
    valid = []
    for elapsed in (0., .1, .2, .4, .6, .8, 1.):
        clock[0] = start + elapsed
        valid += [c for c in frame(node, clock, shape_info()) if c["valid"]]
        if valid:
            break
    assert valid, node.last_selected_decision
    command = valid[-1]
    assert command["action"] == "RIGHT"
    assert node.pending_line_corner["turning"]
    node.send_status("RIGHT", command["command_id"], "RUNNING")
    clock[0] += 1.
    node.send_status("RIGHT", command["command_id"], "SUCCEEDED")
    assert node.pending_line_corner is not None
    assert node.latest_info["line"] is None
    # Exercise exit confirmation directly after the completion capture boundary.
    aligned = shape_info(corner_preview_raw_detected=False, corner_preview_confirmed=False,
                         ground_heading_error_deg=0., ground_fit_point_count=4)
    for _ in range(2):
        result = apply(node, clock, aligned, "STRAIGHT")
        assert not result.valid and node.pending_line_corner is not None
    result = apply(node, clock, aligned, "STRAIGHT")
    assert result.valid and result.action == "STRAIGHT"
    assert node.pending_line_corner is None
