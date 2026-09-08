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
        "depth_age_sec": 0.05,
        "depth_m": 1.2,
        "ground_distance_m": 1.2,
        "distance_m": 1.2,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
    return sample


def goal_info(**overrides):
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.25,
        "ground_distance_m": 0.25,
        "distance_m": 0.25,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
    }
    sample.update(overrides)
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
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
        state="APPROACH",
        depth_m=0.8,
        distance_m=0.8,
        pickup_ready=False,
        pickup_now=False,
    )
    sample.update(overrides)
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
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
            ball=vision_ball_payload(
                depth_m=0.40,
                ground_distance_m=0.15,
                distance_m=0.62,
                pickup_ready=True,
                pickup_now=True,
            )
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "PICKUP_NOW"
    assert decision.source_command["pickup_approach_motion"] == "STRAIGHT_2"


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
    assert decision.action == "STRAIGHT_3"


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
        ("ball", "pickup_ready", vision_ball_payload),
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
        ("ball", "pickup_now", vision_ball_payload, True, "STRAIGHT_3"),
        ("ball", "pickup_now", vision_ball_payload, False, "STRAIGHT_3"),
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

    assert decision.source == "ball"
    assert decision.action == "STRAIGHT_3"
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


def test_ball_inside_1_5m_preempts_line():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=1.2, distance_m=1.2),
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "STRAIGHT_3"
    assert planner.ball_lock_active is True


def test_confirmed_ball_without_depth_stops_without_raw_turn():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_valid=False,
                depth_m=None,
                distance_m=None,
                steering_angle_deg=30.0,
            ),
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "STOP"
    assert decision.reason == "missing_valid_ball_ground_distance"
    assert decision.source_command["linear_speed_mps"] == 0.0
    assert decision.source_command["depth_valid"] is False
    assert planner.ball_lock_active is True


def test_centered_ball_without_depth_preempts_line_but_cannot_advance():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_valid=False,
                depth_m=None,
                distance_m=None,
                steering_angle_deg=0.0,
            ),
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "STOP"
    assert decision.reason == "missing_valid_ball_ground_distance"
    assert decision.source_command["linear_speed_mps"] == 0.0


def test_ball_search_switches_to_ball_inside_1_5m():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "BALL_SEARCH",
        observations(
            line=line_info(),
            ball=ball_info(depth_m=1.2, distance_m=1.2),
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "STRAIGHT_3"


def test_ball_search_approach_phase_requires_controllable_observation():
    planner = MotionDecisionPlanner()

    assert planner.approach_phase_for_search(
        "BALL_SEARCH",
        observations(ball=ball_info(depth_m=1.5)),
    ) == "BALL_APPROACH"
    assert planner.approach_phase_for_search(
        "BALL_SEARCH",
        observations(ball=ball_info(depth_m=1.501)),
    ) is None
    assert planner.approach_phase_for_search(
        "BALL_SEARCH",
        observations(ball=None),
    ) is None


def test_explicit_ball_approach_accepts_valid_distance_above_1_5m():
    decision = MotionDecisionPlanner().plan(
        "BALL_APPROACH",
        observations(ball=ball_info(depth_m=1.501, distance_m=1.501)),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.valid is True
    assert decision.action == "STRAIGHT_3"
    assert decision.reason == "ball_aligned_discrete_approach"


def test_ball_straight_0_becomes_context_specific_fine_forward():
    decision = MotionDecisionPlanner().plan(
        "BALL_APPROACH",
        observations(
            ball=ball_info(
                depth_m=0.50,
                ground_distance_m=0.12,
                distance_m=0.50,
            )
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "BALL_FINE_FORWARD_8"
    assert decision.valid is True
    assert decision.source_command["semantic_motion"] == "STRAIGHT_0"


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


def test_takeover_uses_ground_distance_not_depth_or_camera_distance():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            ball=ball_info(
                depth_m=0.89,
                ground_distance_m=1.60,
                distance_m=1.70,
                horizontal_distance_m=1.60,
            ),
        ),
        0.1,
    )

    assert decision.source == "line"
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
    assert decision.action == "STRAIGHT_3"
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
    assert status["control_range_m"] == 1.5
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
                steering_angle_deg=-45.0,
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
    assert centering.action == "STRAIGHT_3"
    assert centering.reason == "ball_aligned_discrete_approach"
    assert resumed.source == "ball"
    assert resumed.action == "STRAIGHT_3"
    assert resumed.reason == "ball_aligned_discrete_approach"
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
    assert decision.action == "GOAL_CAMERA_90_FORWARD"


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


def test_goal_approach_uses_camera_90_forward_inside_tracking_range():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(depth_m=1.0, distance_m=1.0)),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.valid is True
    assert decision.action == "GOAL_CAMERA_90_FORWARD"
    assert decision.reason == "goal_camera90_depth_bucket_approach"


@pytest.mark.parametrize(
    ("depth", "expected_action", "semantic_motion", "valid"),
    [
        (0.80, "GOAL_CAMERA_90_FORWARD", "STRAIGHT_3", True),
        (0.65, "GOAL_CAMERA_90_FORWARD_4", "STRAIGHT_4", True),
        (0.50, "GOAL_CAMERA_90_FORWARD", "STRAIGHT_3", True),
        (0.35, "GOAL_CAMERA_90_FORWARD_2", "STRAIGHT_2", True),
        (0.20, "GOAL_CAMERA_90_FORWARD_1", "STRAIGHT_1", True),
        (0.12, "STRAIGHT_0", "STRAIGHT_0", False),
    ],
)
def test_centered_goal_uses_depth_bucketed_camera90_forward(
    depth,
    expected_action,
    semantic_motion,
    valid,
):
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=depth,
                distance_m=depth,
                score_now=False,
                depth_in_score_range=False,
            )
        ),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.action == expected_action
    assert decision.valid is valid
    assert decision.source_command["approach_motion"] == semantic_motion


def test_post_ball_line_aligned_signals_transition_without_turn():
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(
            line=line_info(
                filtered_heading_error_deg=11.9,
                filtered_lateral_offset_norm=0.20,
            )
        ),
        0.1,
    )

    assert decision.source == "line"
    assert decision.action == "POST_BALL_LINE_ALIGNED"
    assert decision.valid is True
    assert decision.source_command["heading_tolerance_deg"] == 12.0
    assert decision.source_command["offset_in_tolerance"] is False


@pytest.mark.parametrize(
    ("heading", "expected_action", "expected_count"),
    [
        (14.0, "POST_BALL_LINE_TURN_RIGHT_1", 1),
        (34.0, "POST_BALL_LINE_TURN_RIGHT_3", 3),
        (88.0, "POST_BALL_LINE_TURN_RIGHT_9", 9),
        (-31.0, "POST_BALL_LINE_TURN_LEFT_3", 3),
    ],
)
def test_post_ball_line_heading_selects_ten_degree_repeat_count(
    heading,
    expected_action,
    expected_count,
):
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(
            line=line_info(filtered_heading_error_deg=heading)
        ),
        0.1,
    )

    assert decision.action == expected_action
    assert decision.valid is True
    assert decision.source_command["turn_count"] == expected_count


def test_post_ball_line_missing_left_count_has_no_fallback():
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(
            line=line_info(filtered_heading_error_deg=-51.0)
        ),
        0.1,
    )

    assert decision.action == "POST_BALL_LINE_TURN_LEFT_5"
    assert decision.valid is False
    assert decision.reason == "post_ball_line_left_turn_not_available"
    assert decision.source_command["catalog_motion_available"] is False


def test_post_ball_line_missing_detection_waits_without_forward():
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(line={"detected": False}),
        0.1,
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "post_ball_line_not_detected"


def test_goal_camera90_right_turn_count_uses_bearing_error():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.49,
                bearing_deg=34.0,
                offset_x_norm=0.3,
            )
        ),
        0.1,
    )

    assert decision.action == "GOAL_CAMERA90_TURN_RIGHT_3"
    assert decision.source_command["turn_count"] == 3
    assert decision.source_command["catalog_motion_available"] is True


def test_goal_camera90_left_turn_uses_available_counted_motion():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.49,
                bearing_deg=-26.0,
                offset_x_norm=-0.3,
            )
        ),
        0.1,
    )

    assert decision.action == "GOAL_CAMERA90_TURN_LEFT_3"
    assert decision.valid is True
    assert decision.source_command["turn_count"] == 3
    assert decision.source_command["catalog_motion_available"] is True


@pytest.mark.parametrize(
    ("bearing_deg", "expected_action", "expected_count"),
    [
        (-14.0, "GOAL_CAMERA90_TURN_LEFT_1", 1),
        (-34.0, "GOAL_CAMERA90_TURN_LEFT_3", 3),
        (-58.0, "GOAL_CAMERA90_TURN_LEFT_6", 6),
        (-70.0, "GOAL_CAMERA90_TURN_LEFT_6", 6),
    ],
)
def test_goal_camera90_left_turn_is_clamped_to_available_six_repeats(
    bearing_deg,
    expected_action,
    expected_count,
):
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.49,
                bearing_deg=bearing_deg,
                offset_x_norm=-0.3,
            )
        ),
        0.1,
    )

    assert decision.action == expected_action
    assert decision.source_command["turn_count"] == expected_count
    assert "GOAL_CAMERA90_TURN_LEFT_7" != decision.action


@pytest.mark.parametrize(
    ("offset_x_norm", "expected_action", "direction"),
    [
        (0.11, "GOAL_CAMERA90_CRAB_RIGHT", "RIGHT"),
        (-0.11, "GOAL_CAMERA90_CRAB_LEFT", "LEFT"),
    ],
)
def test_goal_camera90_lateral_uses_offset_after_yaw_is_aligned(
    offset_x_norm,
    expected_action,
    direction,
):
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.49,
                bearing_deg=0.0,
                offset_x_norm=offset_x_norm,
            )
        ),
        0.1,
    )

    assert decision.action == expected_action
    assert decision.reason == "align_goal_lateral_camera90"
    assert decision.source_command["lateral_direction"] == direction
    assert decision.source_command["center_tolerance_norm"] == 0.10


def test_goal_camera90_yaw_has_priority_over_lateral_offset():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.49,
                bearing_deg=34.0,
                offset_x_norm=0.3,
            )
        ),
        0.1,
    )

    assert decision.action == "GOAL_CAMERA90_TURN_RIGHT_3"


def test_goal_tracking_range_does_not_apply_precision_yaw_or_lateral():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.8,
                distance_m=0.8,
                bearing_deg=34.0,
                offset_x_norm=0.3,
            )
        ),
        0.1,
    )

    assert decision.action == "GOAL_CAMERA_90_FORWARD"


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
    assert centering.action == "GOAL_CAMERA_90_FORWARD"
    assert resumed.source == "goal"
    assert resumed.action == "GOAL_CAMERA_90_FORWARD"
    assert resumed.reason == "goal_camera90_depth_bucket_approach"
    assert planner.goal_recovery_centering is False


def test_reacquired_goal_inside_tracking_range_resumes_forward_only():
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
    assert centering.action == "GOAL_CAMERA_90_FORWARD"
    assert resumed.source == "goal"
    assert resumed.action == "GOAL_CAMERA_90_FORWARD"
    assert resumed.reason == "goal_camera90_depth_bucket_approach"
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
                filtered_heading_error_deg=45.0,
                filtered_lateral_offset_norm=0.25,
            )
        ),
        0.1,
    )

    assert decision.action == "RECOVER_RIGHT_TURN_RIGHT_8"
    assert decision.source_command["recovery_side"] == "RIGHT"
    assert decision.source_command["turn_motion"] == "TURN_RIGHT_8"
    assert decision.source_command["turn_angle_deg"] == 45.0


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


@pytest.mark.parametrize(
    ("offset", "pickup_ready", "expected_action", "available"),
    [
        (0.0, True, "BALL_PICKUP_FINE_ALIGN_CONTINUE", True),
        (0.08, True, "BALL_PICKUP_FINE_ALIGN_CONTINUE", True),
        (0.081, False, "BALL_PICKUP_CRAB_RIGHT", True),
        (-0.081, False, "BALL_PICKUP_CRAB_LEFT", True),
    ],
)
def test_pickup_fine_alignment_uses_ready_gate_and_lateral_offset(
    offset,
    pickup_ready,
    expected_action,
    available,
):
    planner = MotionDecisionPlanner()

    decision = planner.plan_ball_pickup_fine_alignment(
        ball_info(
            offset_x_norm=offset,
            pickup_x_tolerance_norm=0.08,
            pickup_ready=pickup_ready,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == expected_action
    assert decision.valid is True
    assert decision.source == "ball"
    assert decision.source_command["offset_x_norm"] == offset
    assert decision.source_command["pickup_x_tolerance_norm"] == 0.08
    assert decision.source_command["catalog_motion_available"] is available


def test_centered_ball_not_ready_runs_fine_forward_then_rechecks():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            ground_distance_m=0.20,
            pickup_ready=False,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == "BALL_PICKUP_FINE_FORWARD"
    assert decision.valid is True
    assert decision.sdk_motion_requested is True


def test_centered_ball_outside_pickup_window_waits():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            pickup_ready=False,
            is_in_pickup_window=False,
        )
    )

    assert decision.action == "WAIT"
    assert decision.valid is False


@pytest.mark.parametrize(
    "overrides",
    [
        {"depth_valid": False, "depth_age_sec": 0.05},
        {"depth_valid": True, "depth_age_sec": None},
        {"depth_valid": True, "depth_age_sec": 0.701},
    ],
)
def test_centered_ball_never_moves_forward_with_invalid_or_stale_depth(
    overrides,
):
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            ground_distance_m=0.20,
            pickup_ready=False,
            is_in_pickup_window=True,
            **overrides,
        )
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.sdk_motion_requested is False
    assert decision.reason == (
        "ball_pickup_fine_forward_waiting_for_fresh_depth"
    )


def test_lateral_correction_remains_available_during_depth_dropout():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            offset_x_norm=0.20,
            pickup_x_tolerance_norm=0.08,
            depth_valid=False,
            depth_age_sec=None,
            pickup_ready=False,
            is_in_pickup_window=False,
        )
    )

    assert decision.action == "BALL_PICKUP_CRAB_RIGHT"
    assert decision.valid is True


def test_inconsistent_pickup_ready_cannot_start_grasp_without_fresh_depth():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            depth_valid=False,
            depth_age_sec=0.05,
            pickup_ready=True,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "ball_pickup_ready_waiting_for_fresh_depth"


@pytest.mark.parametrize(
    "info",
    [
        None,
        ball_info(detected=False, pickup_x_tolerance_norm=0.08),
        ball_info(confidence=0.1, pickup_x_tolerance_norm=0.08),
        ball_info(offset_x_norm=None, pickup_x_tolerance_norm=0.08),
        ball_info(offset_x_norm=0.0),
    ],
)
def test_pickup_fine_alignment_missing_or_invalid_ball_waits(info):
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(info)

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.sdk_motion_requested is False
