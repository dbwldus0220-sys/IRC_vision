"""Unit tests for unified mission command selection."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig
from mission_control.motion_decision_planner import MotionDecisionPlanner


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
        observations(
            ball=vision_ball_payload(depth_m=0.07, distance_m=0.07)
        ),
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


def test_actual_unconfirmed_hurdle_does_not_preempt_ball():
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

    assert decision.source == "ball"
    assert decision.action == "STRAIGHT"


def test_persistent_unconfirmed_hurdle_does_not_take_mission_lock():
    planner = MotionDecisionPlanner()

    pending = observations(
        line=line_info(),
        hurdle=hurdle_info(
            confirmation_confirmed=False,
        ),
    )

    decisions = [planner.plan("AUTO", pending, 0.1) for _ in range(4)]
    assert all(decision.source == "line" for decision in decisions)
    assert all(decision.action == "STRAIGHT" for decision in decisions)
    assert planner.hurdle_lock_active is False


def test_unconfirmed_hurdle_disappearance_keeps_line_available():
    planner = MotionDecisionPlanner()

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
    assert planner.hurdle_lock_active is False


def test_confirmed_hurdle_acquires_mission_lock():
    planner = MotionDecisionPlanner()

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
    assert planner.hurdle_lock_active is True


@pytest.mark.parametrize(
    ("source", "field", "payload_factory"),
    [
        ("ball", "pickup_now", vision_ball_payload),
        ("goal", "score_now", vision_goal_payload),
        ("hurdle", "depth_valid", vision_hurdle_payload),
        ("line", "detected", vision_line_payload),
    ],
)
@pytest.mark.parametrize(
    "invalid_value",
    [1, 0, "true", "false", None, [], {}],
)
def test_invalid_publisher_boolean_type_holds_motion(
    source,
    field,
    payload_factory,
    invalid_value,
):
    payload = payload_factory(**{field: invalid_value})
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(**{source: payload}),
        0.1,
    )

    assert decision.source == source
    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "invalid_vision_boolean_type"


@pytest.mark.parametrize(
    ("source", "field", "payload_factory", "value", "expected_action"),
    [
        ("ball", "pickup_now", vision_ball_payload, True, "STRAIGHT"),
        ("ball", "pickup_now", vision_ball_payload, False, "STRAIGHT"),
        ("goal", "score_now", vision_goal_payload, True, "SHOT"),
        (
            "goal",
            "score_now",
            vision_goal_payload,
            False,
            "WAIT_SCORE_CONFIRMATION",
        ),
        ("hurdle", "depth_valid", vision_hurdle_payload, True, "GO"),
        ("hurdle", "depth_valid", vision_hurdle_payload, False, "WAIT"),
        ("line", "detected", vision_line_payload, True, "STRAIGHT"),
        ("line", "detected", vision_line_payload, False, "WAIT"),
    ],
)
def test_valid_publisher_boolean_type_preserves_existing_action(
    source,
    field,
    payload_factory,
    value,
    expected_action,
):
    payload = payload_factory(**{field: value})
    decision = MotionDecisionPlanner().plan(
        "AUTO",
        observations(**{source: payload}),
        0.1,
    )

    assert decision.action == expected_action
    assert decision.reason != "invalid_vision_boolean_type"


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
    assert decision.action == "STRAIGHT_2"


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
    assert decision.action == "STRAIGHT_2"


def test_unconfirmed_hurdle_does_not_enter_auto_priority():
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

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert planner.hurdle_lock_active is False


@pytest.mark.parametrize(
    ("phase", "target"),
    [
        ("AUTO", {"line": line_info(), "ball": ball_info()}),
        ("LINE_TRACK", {"line": line_info()}),
        ("BALL_APPROACH", {"line": line_info(), "ball": ball_info()}),
        ("GOAL_APPROACH", {"goal": goal_info()}),
    ],
)
def test_detected_unconfirmed_hurdle_does_not_own_each_phase(phase, target):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        phase,
        observations(
            **target,
            hurdle=hurdle_info(confirmation_confirmed=False),
        ),
        0.1,
    )

    assert decision.source != "hurdle"
    assert planner.hurdle_lock_active is False


def test_confirmed_hurdle_with_invalid_depth_does_not_take_lock():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            ball=ball_info(),
            hurdle=hurdle_info(depth_valid=False),
        ),
        0.1,
    )

    assert decision.source == "none"
    assert decision.action == "WAIT"
    assert decision.valid is False
    assert planner.hurdle_lock_active is False


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


def test_ball_between_control_and_tracking_range_keeps_line():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=1.2, distance_m=1.2),
        ),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert planner.ball_tracking_active is False


def test_ball_search_keeps_line_until_ball_is_inside_90cm():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "BALL_SEARCH",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=1.2, distance_m=1.2),
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


def test_ball_approach_phase_rejects_ball_outside_control_range():
    decision = MotionDecisionPlanner().plan(
        "BALL_APPROACH",
        observations(ball=ball_info(depth_m=1.2, distance_m=1.2)),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.valid is False
    assert decision.action == "STOP"
    assert decision.reason == "ball_outside_control_range"


def test_ball_beyond_1_5m_does_not_start_tracking_memory():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=1.501, distance_m=1.501),
        ),
        0.1,
    )

    assert decision.source == "line"
    assert planner.ball_tracking_active is False


def test_untracked_missing_ball_keeps_line_without_recovery():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert planner.ball_tracking_active is False
    assert planner.ball_recovery_centering is False


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
    assert decision.action == "STRAIGHT"


def test_lost_tracked_ball_stops_then_turns_toward_last_seen_side():
    planner = recovery_planner()
    visible_right = ball_info(
        depth_m=0.85,
        distance_m=0.85,
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
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )
    turning = planner.plan(
        "AUTO",
        observations(line=line_info(), ball={"detected": False}),
        0.1,
    )

    assert stopped.source == "ball"
    assert stopped.action == "BALL_LOST_STOP"
    assert stopped.source_command["linear_speed_mps"] == 0.0
    assert stopped.reason == "ball_lost_stop_before_search"
    assert turning.action == "RECOVER_TURN_RIGHT"
    assert turning.source_command["linear_speed_mps"] == 0.0
    assert turning.source_command["angular_speed_rad_s"] > 0.0


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


def test_reacquired_ball_inside_90cm_resumes_ball_control():
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=1.2, distance_m=1.2),
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
            ball=ball_info(depth_m=0.88, distance_m=0.89),
        ),
        0.1,
    )
    assert decision.source == "ball"
    assert decision.action == "STRAIGHT"
    assert planner.ball_lost_elapsed_sec == 0.0


def test_ball_tracking_status_exposes_current_recovery_state():
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
    status = planner.ball_tracking_status()

    assert status["active"] is True
    assert status["recovery_centering"] is True
    assert status["tracking_range_m"] == 1.5
    assert status["control_range_m"] == 0.9
    assert status["lost_elapsed_sec"] == 0.6


def test_reacquired_far_ball_cannot_fall_back_to_line_after_mission_entry():
    planner = recovery_planner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=0.85,
                distance_m=0.85,
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

    centering = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=1.2,
                distance_m=1.2,
                bearing_deg=-10.0,
            ),
        ),
        0.1,
    )
    resumed = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=1.2,
                distance_m=1.2,
                bearing_deg=2.0,
            ),
        ),
        0.1,
    )

    assert centering.source == "ball"
    assert centering.action == "RECOVER_TURN_LEFT"
    assert centering.source_command["linear_speed_mps"] == 0.0
    assert resumed.source == "ball"
    assert resumed.action == "STOP"
    assert resumed.reason == "ball_outside_control_range"
    assert planner.ball_recovery_centering is False


def test_goal_between_control_and_tracking_range_is_remembered():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(depth_m=0.8, distance_m=0.8),
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
    assert decision.action == "STRAIGHT_3"


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


def test_goal_approach_phase_rejects_goal_outside_control_range():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(depth_m=1.0, distance_m=1.0)),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.valid is False
    assert decision.action == "WAIT"
    assert decision.reason == "goal_outside_control_range"


def test_lost_goal_stops_then_turns_toward_last_seen_side():
    planner = MotionDecisionPlanner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=0.49,
                distance_m=0.49,
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
    assert turning.action == "RECOVER_GOAL_TURN_LEFT"
    assert turning.valid is True
    assert turning.source_command["angular_speed_rad_s"] < 0.0
    assert turning.source_command["target_heading_change_deg"] < 0.0
    assert planner.goal_tracking_active is True
    assert planner.goal_lost_elapsed_sec > 0.0


def test_reacquired_far_goal_cannot_fall_back_to_line_after_mission_entry():
    planner = MotionDecisionPlanner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=0.49,
                distance_m=0.49,
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
    assert centering.action == "RECOVER_GOAL_TURN_RIGHT"
    assert resumed.source == "goal"
    assert resumed.action == "WAIT"
    assert resumed.reason == "goal_outside_control_range"
    assert planner.goal_recovery_centering is False


def test_reacquired_goal_inside_tracking_range_is_centered_first():
    planner = MotionDecisionPlanner()
    planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(
                depth_m=0.49,
                distance_m=0.49,
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
    assert centering.action == "RECOVER_GOAL_TURN_RIGHT"
    assert centering.source_command["angular_speed_rad_s"] > 0.0
    assert centering.source_command["target_heading_change_deg"] > 0.0
    assert resumed.source == "goal"
    assert resumed.action == "WAIT"
    assert resumed.reason == "goal_outside_control_range"
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


def test_line_offset_policy_keeps_straight_without_heading_error():
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
    assert centered.action == "STRAIGHT"
    assert left.action == "STRAIGHT"
    assert right.action == "STRAIGHT"
    assert left.valid is True
    assert right.valid is True
    assert left.reason == "line_tracking"
    assert right.reason == "line_tracking"


def test_large_heading_error_selects_matching_correction():
    left_planner = MotionDecisionPlanner()
    right_planner = MotionDecisionPlanner()

    for _ in range(5):
        left = left_planner.plan(
            "LINE_TRACK",
            observations(line=line_info(filtered_heading_error_deg=-20.0)),
            0.1,
        )
        right = right_planner.plan(
            "LINE_TRACK",
            observations(line=line_info(filtered_heading_error_deg=20.0)),
            0.1,
        )

    assert left.action == "RECOVER_LEFT_TURN_LEFT_2"
    assert right.action == "RECOVER_RIGHT_TURN_RIGHT_4"


def test_composite_line_recovery_exposes_turn_metadata():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "LINE_TRACK",
        observations(
            line=line_info(
                filtered_heading_error_deg=-45.0,
                filtered_lateral_offset_norm=0.25,
            )
        ),
        0.1,
    )

    assert decision.action == "RECOVER_RIGHT_TURN_LEFT_6"
    assert decision.source_command["recovery_side"] == "RIGHT"
    assert decision.source_command["turn_motion"] == "TURN_LEFT_6"
    assert decision.source_command["turn_angle_deg"] == -45.0


def test_one_missing_line_frame_stops_safely():
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
    assert missing.reason == "waiting_for_line_info"
    assert missing.valid is False


@pytest.mark.parametrize("offset", [-0.24, 0.24])
def test_complete_line_loss_remains_stopped(offset):
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

    assert recovery.action == "STOP"
    assert recovery.reason == "waiting_for_line_info"
    assert recovery.valid is False


def test_line_loss_does_not_replay_remembered_heading():
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

    assert recovery.action == "STOP"
    assert recovery.reason == "waiting_for_line_info"


def test_line_loss_without_history_stops_safely():
    planner = MotionDecisionPlanner()

    planner.plan("LINE_TRACK", observations(line=None), 0.1)
    lost = planner.plan("LINE_TRACK", observations(line=None), 0.1)

    assert lost.action == "STOP"
    assert lost.reason == "waiting_for_line_info"


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
    assert reacquired.valid is True


def test_repeated_line_loss_never_emits_stale_motion():
    planner = MotionDecisionPlanner()
    planner.plan(
        "LINE_TRACK",
        observations(line=line_info(filtered_lateral_offset_norm=0.24)),
        0.1,
    )

    decisions = [
        planner.plan("LINE_TRACK", observations(line=None), 0.1)
        for _ in range(3)
    ]

    assert all(decision.action == "STOP" for decision in decisions)
    assert all(not decision.valid for decision in decisions)


@pytest.mark.parametrize("phase", ["AUTO", "LINE_TRACK"])
def test_auto_and_line_track_share_line_correction_policy(phase):
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        phase,
        observations(line=line_info(filtered_lateral_offset_norm=0.24)),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert decision.valid is True
    assert decision.reason == "line_tracking"


def test_line_lock_keeps_publishing_continuous_line_guidance():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "LINE_LOCK",
        observations(line=line_info()),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert decision.valid is True
    assert decision.requires_ack is False
