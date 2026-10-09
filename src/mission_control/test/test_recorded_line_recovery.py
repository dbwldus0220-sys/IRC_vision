"""Regress October 6 recovery stalls without commanding robot hardware."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode
from test_line_corner_memory import CornerHarness, corner_info, receive_line
from test_line_offset_alignment import planner, sample
from test_mission_phase_flow import approaching_goal, line_info, release_general


@pytest.fixture
def clock(monkeypatch):
    now = [100.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda _: round(now[0] * 1e9))
    return now


def stamped(info, clock):
    return {**info, "rgb_stamp_ns": round(clock[0] * 1e9)}


def short_line(**updates):
    return {
        **line_info(), "ground_projection_enabled": True,
        "ground_projection_valid": False, "ground_heading_error_deg": None,
        "ground_fit_reason": "too_few_segment_points", "ground_fit_segment": "FULL_PATH",
        "ground_fit_input_point_count": 2, "ground_fit_point_count": 0,
        "heading_quality": 0., "geometry_quality": .50, "detection_quality": .50,
        "corner_preview_confirmed": False, "corner_preview_raw_detected": False,
        **updates,
    }


def remember(node, clock, side):
    receive_line(node, clock, stamped(corner_info(side, corner_start_distance_m=.46), clock))
    assert node.pending_line_corner["corner_direction"] == side


@pytest.mark.parametrize("phase", ["AUTO", "LINE_TRACK", "LINE_TRACK_AFTER_PICKUP"])
def test_first_screenshot_preserves_original_recovery(phase):
    info = sample(65.5, 282., ground_heading_error_deg=20.09,
                  heading_error_deg=-4.9, filtered_heading_error_deg=-4.9,
                  filtered_lateral_offset_norm=.386, turn_angle_deg=2.6)
    actual = planner().plan(phase, {"line": info}, .1)
    assert actual.valid and actual.action == "RECOVER_RIGHT_TURN_RIGHT_4"
    assert actual.action == planner(-1.).plan(phase, {"line": info}, .1).action


def test_recovery_waits_for_forward_completion_and_rechecks_line(clock):
    node = CornerHarness(phase="AUTO")
    forward = node.publish_vision(line=stamped(line_info(), clock))[-1]
    assert forward["action"] == "STRAIGHT"
    node.send_status(forward["action"], forward["command_id"], "RUNNING")
    info = sample(65.5, 282., ground_heading_error_deg=20.09,
                  filtered_lateral_offset_norm=.386, turn_angle_deg=2.6)
    clock[0] += 2.2
    assert not node.publish_vision(line=stamped(info, clock))
    release_general(node, forward)
    clock[0] += .1
    recovery = node.publish_vision(line=stamped(info, clock))[-1]
    assert recovery["action"] == "RECOVER_RIGHT_TURN_RIGHT_4"
    assert recovery["command_id"] != forward["command_id"]


@pytest.mark.parametrize("side,count", [("LEFT", 1), ("RIGHT", 2)])
def test_failed_pickup_corner_loss_and_zero_heading_fit_share_settle(clock, side, count):
    node = CornerHarness(phase="AUTO")
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_sections_processed = 1
    node.phase_manager.ball_grasp_results[1] = "NOT_GRABBED"
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    remember(node, clock, side)
    start = clock[0]
    for index in range(16):
        clock[0] = start + .1 * index
        info = short_line() if index % 2 else {"detected": False}
        node.publish_vision(line=stamped(info, clock), goal=approaching_goal())
        if index < 10:
            assert not any(m["valid"] for m in node.publisher.messages)
    motions = [m for m in node.publisher.messages if m["valid"]]
    assert len(motions) == 1
    assert motions[0]["action"] == f"LINE_LOST_TURN_{side}_{count}"
    assert motions[0]["source_command"]["turn_angle_deg"] == 15.
    assert node.lost_search_turn_limiter.counts["line"] == 1
    assert node.mission_phase == "AUTO"
    assert not node.planner.goal_lock_active
    assert all(m["source"] != "goal" for m in motions)


@pytest.mark.parametrize("side,count", [("LEFT", 1), ("RIGHT", 2)])
def test_short_fit_and_loss_share_three_turn_budget_and_motion_gate(clock, side, count):
    node = CornerHarness(phase="AUTO")
    remember(node, clock, side)
    for index in range(3):
        clock[0] += .1
        info = short_line() if index % 2 == 0 else {"detected": False}
        command = node.publish_vision(line=stamped(info, clock))[-1]
        assert command["action"] == f"LINE_LOST_TURN_{side}_{count}"
        assert command["source_command"]["turn_angle_deg"] == 15.
        assert not node.publish_vision(line=stamped(short_line(), clock))
        release_general(node, command)
        assert not any(m["valid"] for m in node.publish_vision())
        clock[0] += 1.1
    command = node.publish_vision(line=stamped(short_line(), clock))[-1]
    assert not command["valid"]
    assert command["reason"] == "lost_search_turn_limit_reached"
    assert node.lost_search_turn_limiter.counts["line"] == 3
    assert node.lost_search_turn_limiter.angles["line"] == 45.


@pytest.mark.parametrize("updates", [
    {"geometry_quality": .1}, {"detection_quality": .1},
    {"ground_projection_enabled": False}, {"ground_fit_reason": "invalid_calibration"},
    {"ground_fit_input_point_count": 1}, {"ground_fit_input_point_count": 3},
    {"ground_fit_segment": "POST_CORNER"}, {"detected": "true"},
    {"corner_preview_raw_detected": True, "corner_direction": "LEFT"},
])
def test_unusable_or_conflicting_geometry_cannot_start_corner_turn(clock, updates):
    node = CornerHarness(phase="AUTO")
    remember(node, clock, "RIGHT")
    clock[0] += .1
    result = node.publish_vision(line=stamped(short_line(**updates), clock))[-1]
    assert not result["valid"]
    assert not any(m["valid"] for m in node.publisher.messages)


@pytest.mark.parametrize("stamp", [None, True, 99_000_000_000, 102_000_000_000])
def test_short_fit_rejects_invalid_capture(clock, stamp):
    node = CornerHarness(phase="AUTO")
    remember(node, clock, "RIGHT")
    result = node.publish_vision(line={**short_line(), "rgb_stamp_ns": stamp})[-1]
    assert not result["valid"]


@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
def test_new_confirmed_short_corner_can_be_remembered_with_zero_heading(clock, side):
    node = CornerHarness(phase="AUTO")
    info = {**short_line(), **corner_info(side), "heading_quality": 0.,
            "ground_projection_valid": False, "corner_preview_raw_detected": True,
            "ground_fit_segment": "PRE_CORNER"}
    result = node.publish_vision(line=stamped(info, clock))[-1]
    assert result["valid"] and result["action"].startswith(f"LINE_LOST_TURN_{side}_")
    assert node.pending_line_corner["corner_direction"] == side
    assert result["source_command"]["turn_angle_deg"] == 15.


def test_corner_search_failure_prevents_more_short_fit_turns(clock):
    node = CornerHarness(phase="AUTO")
    remember(node, clock, "RIGHT")
    command = node.publish_vision(line=stamped(short_line(), clock))[-1]
    node.send_status(command["action"], command["command_id"], "RUNNING")
    node.send_status(command["action"], command["command_id"], "FAILED")
    clock[0] += 2.
    result = node.publish_vision(line=stamped(short_line(), clock))[-1]
    assert not result["valid"]
    assert "line" in node.lost_search_turn_limiter.failed_sources
    remember(node, clock, "RIGHT")
    result = node.publish_vision(line=stamped(short_line(), clock))[-1]
    assert not result["valid"] and result["reason"] == "lost_search_motion_failed"


def test_short_fit_without_confirmed_corner_does_not_guess_direction(clock):
    node = CornerHarness(phase="AUTO")
    result = node.publish_vision(line=stamped(short_line(), clock))[-1]
    assert not result["valid"]
    assert getattr(node, "pending_line_corner", None) is None


@pytest.mark.parametrize("elapsed", [16., 75.])
def test_expired_corner_releases_fresh_usable_non_corner_line(clock, elapsed):
    node = CornerHarness(phase="AUTO")
    remember(node, clock, "RIGHT")
    clock[0] += elapsed
    info = sample(-78.3, -621.1, ground_heading_error_deg=-21.35,
                  heading_error_deg=24.7, filtered_heading_error_deg=24.7,
                  filtered_lateral_offset_norm=-.97, corner_preview_confirmed=False)
    result = node.publish_vision(line=stamped(info, clock))[-1]
    assert node.pending_line_corner is None
    assert result["valid"] and result["action"] == "RECOVER_LEFT_TURN_LEFT_4"


@pytest.mark.parametrize("info", [short_line(), {"detected": False},
                                  {**line_info(), "corner_preview_confirmed": True}])
def test_expired_corner_never_authorizes_blind_turn(clock, info):
    node = CornerHarness(phase="AUTO")
    remember(node, clock, "RIGHT")
    clock[0] += 16.
    result = node.publish_vision(line=stamped(info, clock))[-1]
    assert not result["valid"] and result["reason"] == "line_corner_memory_expired"
    assert node.pending_line_corner is not None
