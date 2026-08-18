"""Unit tests for unified mission command selection."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig
from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.line_navigation_planner import LineNavigationPlanner
from step.line_navigation_planner import NavigationConfig


def line_info(**overrides):
    sample = {
        "detected": True,
        "filtered_heading_error_deg": 0.0,
        "filtered_lateral_offset_norm": 0.0,
        "heading_quality": 0.9,
        "geometry_quality": 0.9,
        "detection_quality": 0.9,
        "turn_angle_deg": 0.0,
        "turn_consistency": 1.0,
    }
    sample.update(overrides)
    return sample


def ball_info(**overrides):
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 1.2,
        "distance_m": 1.2,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    return sample


def goal_info(**overrides):
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.25,
        "distance_m": 0.25,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
    }
    sample.update(overrides)
    return sample


def hurdle_info(**overrides):
    sample = {
        "detected": True,
        "raw_detected": True,
        "confirmation_confirmed": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.8,
        "distance_m": 0.8,
        "ground_gap_m": 0.1,
        "camera_bottom_gap_m": 0.02,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "hurdle_angle_deg": 0.0,
        "go_now": True,
        "bbox": [300, 300, 980, 500],
        "image_width": 1280,
    }
    sample.update(overrides)
    return sample


def vision_line_payload(**overrides):
    """Match the fields emitted by YoloLineAnalyzer."""
    sample = line_info(
        center_points_px=[[640, 700], [640, 500]],
        lateral_offset_norm=0.0,
        heading_error_deg=0.0,
        mean_confidence=0.95,
        filter_ready=True,
    )
    sample.update(overrides)
    return sample


def vision_ball_payload(**overrides):
    """Match the mission fields emitted by BallAnalyzer."""
    sample = ball_info(
        raw_detected=True,
        confirmation_confirmed=True,
        state="PICKUP_READY",
        depth_m=0.8,
        distance_m=0.8,
        pickup_ready=True,
        pickup_now=True,
    )
    sample.update(overrides)
    return sample


def vision_goal_payload(**overrides):
    """Match the mission fields emitted by GoalAnalyzer."""
    sample = goal_info(
        raw_detected=True,
        confirmation_confirmed=True,
        state="SCORE_READY",
        score_now=True,
    )
    sample.update(overrides)
    return sample


def vision_hurdle_payload(**overrides):
    """Match the mission fields emitted by HurdleAnalyzer."""
    sample = hurdle_info(
        state="GO_READY",
        go_now=True,
    )
    sample.update(overrides)
    return sample


def observations(**overrides):
    samples = {
        "line": None,
        "ball": None,
        "goal": None,
        "hurdle": None,
    }
    samples.update(overrides)
    return samples


def recovery_planner():
    """Create a planner with the head-scan recovery policy enabled."""
    return MotionDecisionPlanner(
        MotionDecisionConfig(enable_ball_lost_recovery=True)
    )


def test_actual_line_publisher_payload_contract():
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(line=vision_line_payload()),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"


def test_actual_ball_publisher_payload_contract():
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(ball=vision_ball_payload()),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "PICKUP_NOW"


def test_actual_hurdle_publisher_payload_contract():
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(
            ball=vision_ball_payload(),
            hurdle=vision_hurdle_payload(),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "GO"


def test_actual_goal_publisher_payload_contract():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            ball=vision_ball_payload(),
            goal=vision_goal_payload(),
        ),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.action == "SHOT"


def test_actual_unconfirmed_hurdle_payload_holds_motion():
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(
            ball=vision_ball_payload(),
            hurdle=vision_hurdle_payload(
                detected=False,
                raw_detected=True,
                confirmation_confirmed=False,
                depth_valid=False,
                depth_m=None,
                go_now=False,
            ),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "WAIT"
    assert decision.reason == "hurdle_confirmation_pending"


def test_persistent_unconfirmed_hurdle_enters_verification_timeout():
    planner = MotionDecisionPlanner(
        MotionDecisionConfig(
            hurdle_verification_timeout_sec=0.25,
        )
    )

    pending = observations(
        line=line_info(),
        hurdle=hurdle_info(
            confirmation_confirmed=False,
        ),
    )

    first = planner.plan("AUTO", pending, 0.1)
    second = planner.plan("AUTO", pending, 0.1)
    third = planner.plan("AUTO", pending, 0.1)
    timed_out = planner.plan("AUTO", pending, 0.1)
    assert third.reason == "hurdle_confirmation_pending"

    assert first.source == "hurdle"
    assert first.action == "WAIT"
    assert first.reason == "hurdle_confirmation_pending"

    assert second.source == "hurdle"
    assert second.action == "WAIT"
    assert second.reason == "hurdle_confirmation_pending"

    assert timed_out.source == "hurdle"
    assert timed_out.action == "WAIT"
    assert timed_out.reason == "hurdle_verification_timeout"


def test_hurdle_verification_timer_resets_after_disappearance():
    planner = MotionDecisionPlanner(
        MotionDecisionConfig(
            hurdle_verification_timeout_sec=0.25,
        )
    )

    pending = observations(
        line=line_info(),
        hurdle=hurdle_info(
            confirmation_confirmed=False,
        ),
    )

    planner.plan("AUTO", pending, 0.2)

    disappeared = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            hurdle=None,
        ),
        0.1,
    )

    assert disappeared.source == "line"
    assert disappeared.action == "STRAIGHT"
    assert planner.hurdle_verification_elapsed_sec == 0.0

    pending_again = planner.plan("AUTO", pending, 0.2)

    assert pending_again.source == "hurdle"
    assert pending_again.action == "WAIT"
    assert pending_again.reason == "hurdle_confirmation_pending"


def test_hurdle_verification_timer_resets_when_hurdle_becomes_safe():
    planner = MotionDecisionPlanner(
        MotionDecisionConfig(
            hurdle_verification_timeout_sec=0.5,
        )
    )

    pending = observations(
        line=line_info(),
        hurdle=hurdle_info(
            confirmation_confirmed=False,
        ),
    )

    planner.plan("AUTO", pending, 0.2)

    confirmed = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            hurdle=hurdle_info(
                confirmation_confirmed=True,
                depth_valid=True,
            ),
        ),
        0.1,
    )

    assert confirmed.source == "hurdle"
    assert planner.hurdle_verification_elapsed_sec == 0.0


@pytest.mark.parametrize(
    ("source", "payload"),
    [
        ("ball", vision_ball_payload(pickup_now="false")),
        ("goal", vision_goal_payload(score_now=1)),
        ("hurdle", vision_hurdle_payload(depth_valid="false")),
        ("line", vision_line_payload(detected="true")),
    ],
)
def test_invalid_publisher_boolean_type_holds_motion(source, payload):
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(**{source: payload}),
        0.1,
    )

    assert decision.source == source
    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "invalid_vision_boolean_type"


def test_optional_ball_alignment_field_missing_stops_safely():
    payload = vision_ball_payload()
    payload.pop("bearing_deg")
    payload.pop("offset_x_norm")

    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(ball=payload),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "STOP"
    assert decision.valid is False
    assert decision.reason == "invalid_ball_alignment"


def test_goal_approach_keeps_goal_ahead_of_fresh_ball():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "GOAL_APPROACH",
        observations(
            ball=ball_info(depth_m=0.80, distance_m=0.80),
            goal=goal_info(),
        ),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.action == "SHOT"
    assert decision.requires_ack is True


def test_go_ready_confirmed_hurdle_has_priority_over_close_ball():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=0.85, distance_m=0.86),
            goal=goal_info(),
            hurdle=hurdle_info(),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "GO"


def test_confirmed_hurdle_priority_does_not_depend_on_lateral_offset():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=0.85, distance_m=0.86),
            hurdle=hurdle_info(
                bbox=[1050, 300, 1250, 500],
                offset_x_norm=0.80,
            ),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "GO"


@pytest.mark.parametrize("phase", ["BALL_APPROACH", "LINE_TRACK"])
def test_confirmed_hurdle_preempts_ball_in_non_goal_phase(phase):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        phase,
        observations(
            line=line_info(),
            ball=ball_info(),
            hurdle=hurdle_info(
                go_now=False,
                ground_gap_m=0.35,
                camera_bottom_gap_m=0.20,
            ),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "APPROACH_HURDLE"


@pytest.mark.parametrize("include_ball", [False, True])
def test_goal_approach_confirmed_hurdle_preempts_goal(include_ball):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "GOAL_APPROACH",
        observations(
            ball=ball_info() if include_ball else None,
            goal=goal_info(),
            hurdle=hurdle_info(
                go_now=False,
                ground_gap_m=0.35,
                camera_bottom_gap_m=0.20,
            ),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "APPROACH_HURDLE"


def test_unconfirmed_hurdle_cannot_reenter_through_auto_fallback():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            hurdle=hurdle_info(
                confirmation_confirmed=False,
                go_now=True,
            ),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "hurdle_confirmation_pending"


@pytest.mark.parametrize(
    ("phase", "target"),
    [
        ("AUTO", {"line": line_info(), "ball": ball_info()}),
        ("LINE_TRACK", {"line": line_info()}),
        ("BALL_APPROACH", {"line": line_info(), "ball": ball_info()}),
        ("GOAL_APPROACH", {"goal": goal_info()}),
    ],
)
def test_detected_unconfirmed_hurdle_holds_each_phase(phase, target):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        phase,
        observations(
            **target,
            hurdle=hurdle_info(confirmation_confirmed=False),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "hurdle_confirmation_pending"
    assert decision.requires_ack is False
    assert decision.sdk_motion_requested is False


def test_confirmed_hurdle_with_invalid_depth_holds_motion():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            ball=ball_info(),
            hurdle=hurdle_info(depth_valid=False),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "hurdle_depth_invalid_wait"


@pytest.mark.parametrize("hurdle", [None, {"detected": False}])
def test_absent_or_not_detected_hurdle_does_not_hold_ball(hurdle):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            ball=ball_info(depth_m=0.8, distance_m=0.8),
            hurdle=hurdle,
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action != "WAIT"


def test_ball_between_90cm_and_3m_keeps_line_with_tracking_memory():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=2.5, distance_m=2.5),
        ),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert planner.ball_tracking_active is True


def test_ball_search_keeps_line_until_ball_is_inside_90cm():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "BALL_SEARCH",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=2.0, distance_m=2.0),
        ),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"


def test_ball_search_approach_phase_requires_controllable_observation():
    planner = MotionDecisionPlanner()

    assert planner.approach_phase_for_search(
        "BALL_SEARCH",
        observations(ball=ball_info(depth_m=0.9)),
    ) == "BALL_APPROACH"
    assert planner.approach_phase_for_search(
        "BALL_SEARCH",
        observations(ball=ball_info(depth_m=0.91)),
    ) is None
    assert planner.approach_phase_for_search(
        "BALL_SEARCH",
        observations(ball=None),
    ) is None


def test_ball_approach_phase_coarsely_approaches_far_aligned_ball():
    decision = MotionDecisionPlanner().plan(
        "BALL_APPROACH",
        observations(ball=ball_info(depth_m=2.0, distance_m=2.0)),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.valid is True
    assert decision.action == "APPROACH"


def test_ball_beyond_3m_does_not_start_tracking_memory():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=3.01, distance_m=3.01),
        ),
        0.1,
    )

    assert decision.source == "line"
    assert planner.ball_tracking_active is False


def test_untracked_missing_ball_keeps_line_without_head_scan():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert planner.ball_tracking_active is False
    assert planner.ball_scan_active is False


def test_90cm_takeover_uses_depth_not_hypotenuse_distance():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=0.89,
                distance_m=1.70,
                horizontal_distance_m=1.60,
            ),
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "FINE_FORWARD_STEP"


def test_lost_tracked_ball_runs_timed_head_scan_with_body_stopped():
    planner = recovery_planner()
    visible_right = ball_info(
        depth_m=2.5,
        distance_m=2.5,
        bearing_deg=12.0,
        offset_x_norm=0.25,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball=visible_right),
        0.1,
    )

    stopped = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )
    scan_left = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.5,
    )
    scan_right = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.6,
    )
    centered = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.9,
    )

    assert stopped.source == "ball"
    assert stopped.action == "BALL_LOST_STOP"
    assert stopped.valid is False
    assert stopped.source_command["linear_speed_mps"] == 0.0
    assert stopped.reason == "ball_lost_stop_before_head_scan"
    assert scan_left.action == "HEAD_SCAN_LEFT"
    assert scan_left.valid is False
    assert scan_left.reason == "scan_head_left_for_ball"
    assert scan_right.action == "HEAD_SCAN_RIGHT"
    assert scan_right.valid is False
    assert scan_right.reason == "scan_head_right_for_ball"
    assert centered.action == "HEAD_CENTER"
    assert centered.valid is False
    assert centered.reason == "return_head_to_center"
    for decision in (stopped, scan_left, scan_right, centered):
        assert decision.requires_ack is False
        assert decision.sdk_motion_requested is False
        assert decision.source_command["linear_speed_mps"] == 0.0
        assert decision.source_command["angular_speed_rad_s"] == 0.0
        assert decision.source_command["command_duration_sec"] > 0.0


def test_confirmed_hurdle_interrupts_active_ball_head_scan():
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=2.5,
                distance_m=2.5,
                bearing_deg=12.0,
            ),
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.4,
    )

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball={"detected": False},
            hurdle=hurdle_info(),
        ),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "GO"


def test_reacquired_ball_requires_three_frames_before_control_resumes():
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=2.5, distance_m=2.5),
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.4,
    )

    first = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=0.88, distance_m=0.89),
        ),
        0.1,
    )
    second = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=0.88, distance_m=0.89),
        ),
        0.1,
    )
    resumed = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=0.88, distance_m=0.89),
        ),
        0.1,
    )

    assert first.action == "BALL_LOST_STOP"
    assert first.source_command["angular_speed_rad_s"] == 0.0
    assert second.action == "HEAD_SCAN_LEFT"
    assert resumed.source == "ball"
    assert resumed.action == "FINE_FORWARD_STEP"
    assert planner.ball_lost_elapsed_sec == 0.0
    assert planner.ball_scan_active is False


def test_reacquire_count_resets_when_detection_breaks():
    """Reset consecutive recovery confirmation after a missing frame."""
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=0.8, distance_m=0.8),
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.6,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball=ball_info(depth_m=0.8)),
        0.1,
    )
    assert planner.ball_reacquire_count == 1

    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )

    assert planner.ball_reacquire_count == 0
    assert planner.ball_scan_active is True


def test_ball_tracking_status_exposes_head_scan_progress():
    """Expose scan stage, confirmation count, and configured limits."""
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(ball=ball_info(depth_m=0.8)),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(ball={"detected": False}),
        0.6,
    )

    status = planner.ball_tracking_status()

    assert status["scan_active"] is True
    assert status["reacquire_count"] == 0
    assert status["recovery_timeout_sec"] == 2.5
    assert status["recovery_max_distance_m"] == 1.0
    assert status["recovery_stage"] == "HEAD_SCAN_LEFT"


def test_far_ball_does_not_reacquire_and_timeout_returns_to_line():
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=2.5,
                distance_m=2.5,
                bearing_deg=-15.0,
            ),
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.4,
    )

    scanning = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=2.4,
                distance_m=2.4,
                bearing_deg=-10.0,
            ),
        ),
        0.1,
    )
    abandoned = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=2.4,
                distance_m=2.4,
                bearing_deg=2.0,
            ),
        ),
        2.1,
    )

    assert scanning.source == "ball"
    assert scanning.action == "BALL_LOST_STOP"
    assert scanning.valid is False
    assert planner.ball_reacquire_count == 0
    assert abandoned.source == "line"
    assert abandoned.action == "STRAIGHT"
    assert planner.ball_tracking_active is False
    assert planner.ball_scan_active is False


def test_goal_between_50cm_and_3m_is_remembered_while_line_continues():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(depth_m=1.5, distance_m=1.5),
        ),
        0.1,
    )

    assert decision.source == "line"
    assert planner.goal_tracking_active is True


def test_goal_inside_50cm_takes_priority_and_approaches():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(depth_m=0.49, distance_m=0.49),
        ),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.action == "APPROACH_GOAL"


def test_goal_search_keeps_line_until_goal_is_inside_50cm():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "GOAL_SEARCH",
        observations(
            line=line_info(),
            goal=goal_info(depth_m=1.0, distance_m=1.0),
        ),
        0.1,
    )

    assert decision.source == "line"


def test_goal_search_approach_phase_requires_controllable_observation():
    planner = MotionDecisionPlanner()

    assert planner.approach_phase_for_search(
        "GOAL_SEARCH",
        observations(goal=goal_info(depth_m=0.5)),
    ) == "GOAL_APPROACH"
    assert planner.approach_phase_for_search(
        "GOAL_SEARCH",
        observations(goal=goal_info(depth_m=0.51)),
    ) is None
    assert planner.approach_phase_for_search(
        "GOAL_SEARCH",
        observations(goal=None),
    ) is None


def test_goal_approach_phase_coarsely_approaches_far_aligned_goal():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(depth_m=1.0, distance_m=1.0)),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.valid is True
    assert decision.action == "APPROACH_GOAL"


def test_lost_goal_stops_then_turns_toward_last_seen_side():
    planner = MotionDecisionPlanner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=1.5,
                distance_m=1.5,
                bearing_deg=-12.0,
                offset_x_norm=-0.25,
            ),
        ),
        0.1,
    )

    stopped = planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        0.3,
    )
    turning = planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        0.1,
    )

    assert stopped.source == "goal"
    assert stopped.action == "GOAL_LOST_STOP"
    assert stopped.valid is False
    assert stopped.source_command["linear_speed_mps"] == 0.0
    assert turning.action == "LEFT"
    assert turning.valid is True
    assert turning.source_command["angular_speed_rad_s"] < 0.0
    assert turning.source_command["target_heading_change_deg"] < 0.0
    assert planner.goal_tracking_active is True
    assert planner.goal_lost_elapsed_sec > 0.0


def test_goal_recovery_timeout_is_strict_and_clears_tracking_state():
    planner = MotionDecisionPlanner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=1.5,
                distance_m=1.5,
                bearing_deg=-12.0,
                offset_x_norm=-0.25,
            ),
        ),
        0.1,
    )

    before_timeout = planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        planner.config.goal_lost_stop_sec + 0.01,
    )
    remaining = (
        planner.config.goal_recovery_timeout_sec
        - planner.goal_lost_elapsed_sec
    )
    at_timeout = planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        remaining,
    )

    assert before_timeout.action == "LEFT"
    assert at_timeout.action == "LEFT"
    assert planner.goal_tracking_active is True
    assert planner.goal_recovery_centering is True
    assert planner.goal_lost_elapsed_sec == pytest.approx(
        planner.config.goal_recovery_timeout_sec
    )
    assert planner.last_goal_bearing_deg == -12.0
    assert planner.last_goal_offset_x_norm == -0.25
    assert planner.last_goal_turn_direction == "LEFT"

    timed_out = planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        0.001,
    )

    assert timed_out.source == "line"
    assert timed_out.action == "STRAIGHT"
    assert planner.goal_tracking_active is False
    assert planner.goal_recovery_centering is False
    assert planner.goal_lost_elapsed_sec == 0.0
    assert planner.last_goal_bearing_deg is None
    assert planner.last_goal_offset_x_norm is None
    assert planner.last_goal_turn_direction == "LEFT"


def test_reacquired_goal_is_centered_before_line_or_goal_control():
    planner = MotionDecisionPlanner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=1.5,
                distance_m=1.5,
                bearing_deg=15.0,
            ),
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), goal={"detected": False}),
        0.4,
    )

    centering = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=1.0,
                distance_m=1.0,
                bearing_deg=10.0,
            ),
        ),
        0.1,
    )
    resumed = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=1.0,
                distance_m=1.0,
                bearing_deg=2.0,
            ),
        ),
        0.1,
    )

    assert centering.source == "goal"
    assert centering.action == "RIGHT"
    assert centering.source_command["angular_speed_rad_s"] > 0.0
    assert centering.source_command["target_heading_change_deg"] > 0.0
    assert resumed.source == "line"
    assert planner.goal_recovery_centering is False


def test_confirmation_waits_are_non_executable():
    planner = MotionDecisionPlanner()

    score_wait = planner.plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(score_now=False)),
        0.1,
    )
    go_wait = planner.plan(
        "HURDLE_APPROACH",
        observations(hurdle=hurdle_info(go_now=False)),
        0.1,
    )

    for decision in (score_wait, go_wait):
        assert decision.valid is False
        assert decision.sdk_motion_requested is False
        assert decision.requires_ack is False
    assert score_wait.action == "WAIT_SCORE_CONFIRMATION"
    assert go_wait.action == "WAIT_GO_CONFIRMATION"


def test_hurdle_go_is_normalized_as_acknowledged_sdk_event():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "HURDLE_APPROACH",
        observations(hurdle=hurdle_info()),
        0.1,
    )

    assert decision.source == "hurdle"
    assert decision.action == "GO"
    assert decision.sdk_motion_requested is True
    assert decision.requires_ack is True


def test_line_phase_reuses_existing_line_planner():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "LINE_TRACK",
        observations(line=line_info()),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert decision.valid is True


def test_unknown_phase_fails_safe():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "NOT_A_PHASE",
        observations(ball=ball_info()),
        0.1,
    )

    assert decision.source == "none"
    assert decision.action == "WAIT"
    assert decision.valid is False


def test_search_phase_tracks_line_until_target_appears():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "BALL_SEARCH",
        observations(line=line_info()),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"


def test_lock_phase_waits_for_cpp_motion_status():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "HURDLE_LOCK",
        observations(line=line_info(), hurdle=hurdle_info()),
        0.1,
    )

    assert decision.source == "none"
    assert decision.action == "WAIT"
    assert decision.reason == "mission_locked_waiting_for_motion_status"


def test_line_offset_policy_uses_tolerance_and_strong_correction():
    planner = MotionDecisionPlanner()

    centered = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=0.05)),
        0.1,
    )
    left = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=-0.24)),
        0.1,
    )
    right = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=0.24)),
        0.1,
    )
    far_left = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=-0.35)),
        0.1,
    )
    far_right = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=0.35)),
        0.1,
    )

    assert centered.action == "STRAIGHT"
    assert left.action == "FINE_LEFT"
    assert right.action == "FINE_RIGHT"
    assert left.valid is True
    assert right.valid is True
    assert left.reason == "line_tracking"
    assert right.reason == "line_tracking"
    assert far_left.action == "LEFT"
    assert far_right.action == "RIGHT"
    assert far_left.action != "STRAIGHT"
    assert far_right.action != "STRAIGHT"


def test_large_heading_error_selects_matching_correction():
    planner = MotionDecisionPlanner()

    left = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_heading_error_deg=-20.0)),
        0.1,
    )
    right = planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_heading_error_deg=20.0)),
        0.1,
    )

    assert left.action == "LEFT"
    assert right.action == "RIGHT"


def test_one_missing_line_frame_does_not_start_strong_recovery():
    planner = MotionDecisionPlanner()
    planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=-0.24)),
        0.1,
    )

    missing = planner.plan(
        "LINE_TRACK",
        observations(line=None),
        0.1,
    )

    assert missing.action == "STOP"
    assert missing.reason == "line_not_detected"
    assert planner.line_planner.line_lost_frames == 1
    assert planner.line_planner.line_recovery_attempts == 0


@pytest.mark.parametrize(
    ("offset", "expected"),
    [(-0.24, "LEFT"), (0.24, "RIGHT")],
)
def test_complete_line_loss_recovers_from_last_valid_offset(
    offset,
    expected,
):
    planner = MotionDecisionPlanner()
    planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=offset)),
        0.1,
    )
    planner.plan("LINE_TRACK", observations(line=None), 0.1)

    recovery = planner.plan(
        "LINE_TRACK",
        observations(line=None),
        0.1,
    )

    assert recovery.action == expected
    assert recovery.reason == "line_lost_recovery"
    assert planner.line_planner.recovering_line is True
    assert planner.line_planner.line_recovery_attempts == 1


def test_line_loss_uses_heading_when_remembered_offset_is_ambiguous():
    planner = MotionDecisionPlanner()
    planner.plan(
        "LINE_TRACK",
        observations(
            line=line_info(
                filtered_lateral_offset_norm=0.02,
                filtered_heading_error_deg=-10.0,
            )
        ),
        0.1,
    )
    planner.plan("LINE_TRACK", observations(line=None), 0.1)

    recovery = planner.plan(
        "LINE_TRACK",
        observations(line=None),
        0.1,
    )

    assert recovery.action == "LEFT"


def test_line_loss_without_history_stops_safely():
    planner = MotionDecisionPlanner()

    planner.plan("LINE_TRACK", observations(line=None), 0.1)
    lost = planner.plan("LINE_TRACK", observations(line=None), 0.1)

    assert lost.action == "STOP"
    assert lost.reason == "line_lost_without_history"
    assert planner.line_planner.recovering_line is False


def test_reacquired_line_returns_directly_to_straight():
    planner = MotionDecisionPlanner()
    planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=-0.24)),
        0.1,
    )
    planner.plan("LINE_TRACK", observations(line=None), 0.1)
    planner.plan("LINE_TRACK", observations(line=None), 0.1)

    reacquired = planner.plan(
        "LINE_TRACK",
        observations(line=line_info()),
        0.1,
    )

    assert reacquired.action == "STRAIGHT"
    assert planner.line_planner.recovering_line is False
    assert planner.line_planner.line_lost_frames == 0
    assert planner.line_planner.line_recovery_attempts == 0


def test_line_recovery_attempt_limit_stops_repeated_commands():
    planner = MotionDecisionPlanner()
    planner.line_planner = LineNavigationPlanner(
        NavigationConfig(
            line_lost_frame_threshold=1,
            line_max_recovery_attempts=2,
        )
    )
    planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=0.24)),
        0.1,
    )

    first = planner.plan("LINE_TRACK", observations(line=None), 0.1)
    second = planner.plan("LINE_TRACK", observations(line=None), 0.1)
    exhausted = planner.plan("LINE_TRACK", observations(line=None), 0.1)

    assert first.action == "RIGHT"
    assert second.action == "RIGHT"
    assert exhausted.action == "STOP"
    assert exhausted.reason == "line_recovery_attempts_exhausted"
    assert planner.line_planner.line_recovery_attempts == 2


@pytest.mark.parametrize("phase", ["AUTO", "LINE_TRACK"])
def test_auto_and_line_track_share_line_correction_policy(phase):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        phase,
        observations(line=line_info(filtered_lateral_offset_norm=0.24)),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "FINE_RIGHT"
    assert decision.valid is True
    assert decision.reason == "line_tracking"


def test_special_motion_lock_does_not_advance_line_recovery():
    planner = MotionDecisionPlanner()
    planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=0.24)),
        0.1,
    )
    before = planner.line_planner.line_recovery_attempts

    locked = planner.plan(
        "GOAL_APPROACH_LOCK",
        observations(line=None),
        0.1,
    )

    assert locked.action == "WAIT"
    assert locked.reason == "mission_locked_waiting_for_motion_status"
    assert planner.line_planner.line_lost_frames == 0
    assert planner.line_planner.line_recovery_attempts == before
