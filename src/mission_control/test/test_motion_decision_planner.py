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
        "offset_x_px": 0,
        "offset_x_norm": 0.0,
        "camera_center_offset_x_px": 0,
        "bottom_distance_px": 600,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    if "camera_center_offset_x_px" not in overrides and "offset_x_px" in overrides:
        sample["camera_center_offset_x_px"] = overrides["offset_x_px"]
    if "distance_m" not in overrides and "depth_m" in overrides:
        sample["distance_m"] = overrides["depth_m"]
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
    return sample


def goal_info(**overrides):
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.795,
        "ground_distance_m": 0.795,
        "distance_m": 0.795,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "offset_x_px": 0,
        "score_now": True,
    }
    sample.update(overrides)
    if "offset_x_px" not in overrides and "offset_x_norm" in overrides:
        sample["offset_x_px"] = overrides["offset_x_norm"] * 640
    if "distance_m" not in overrides and "depth_m" in overrides:
        sample["distance_m"] = overrides["depth_m"]
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
        "depth_m": 0.1,
        "distance_m": 0.1,
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
    if "distance_m" not in overrides and "depth_m" in overrides:
        sample["distance_m"] = overrides["depth_m"]
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
                distance_m=0.55,
                pickup_ready=True,
                pickup_now=True,
            )
        ),
        0.1,
    )

    assert decision.source == "ball"
    assert decision.action == "PICKUP_NOW"
    assert decision.source_command["pickup_approach_motion"] is None


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
    assert decision.action == "STRAIGHT_2"


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
        ("ball", "pickup_now", vision_ball_payload, True, "STRAIGHT_2"),
        ("ball", "pickup_now", vision_ball_payload, False, "STRAIGHT_2"),
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
                depth_m=0.35,
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
                depth_m=0.35,
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
    assert decision.action == "STRAIGHT_2"
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
    assert decision.action == "STRAIGHT_2"
    assert planner.ball_lock_active is True


def test_confirmed_ball_without_valid_distance_keeps_existing_line_flow():
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

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert decision.valid is True
    assert planner.ball_lock_active is False


def test_centered_ball_without_valid_distance_does_not_preempt_line():
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

    assert decision.source == "line"
    assert decision.action == "STRAIGHT"
    assert decision.valid is True


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
    assert decision.action == "STRAIGHT_2"


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
    assert decision.action == "STRAIGHT_2"
    assert decision.reason == "ball_aligned_discrete_approach"


def test_ball_pickup_entry_uses_distance_not_legacy_ground_distance():
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
    assert decision.action == "PICKUP_NOW"
    assert decision.valid is True
    assert decision.source_command["distance_m"] == 0.50


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


def test_takeover_uses_distance_field():
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
        offset_x_px=100,
    )
    planner.plan(
        "AUTO",
        observations(
            line=line_info(filtered_lateral_offset_norm=-0.2),
            ball=visible_right,
        ),
        0.1,
    )

    stopped = planner.plan(
        "AUTO",
        observations(
            line=line_info(filtered_lateral_offset_norm=-0.2),
            ball={"detected": False},
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(
            line=line_info(filtered_lateral_offset_norm=-0.2),
            ball={"detected": False},
        ),
        0.1,
    )
    planner.plan(
        "AUTO",
        observations(
            line=line_info(filtered_lateral_offset_norm=-0.2),
            ball={"detected": False},
        ),
        0.1,
    )
    turning = planner.plan(
        "AUTO",
        observations(
            line=line_info(filtered_lateral_offset_norm=-0.2),
            ball={"detected": False},
        ),
        0.1,
    )

    assert stopped.source == "ball"
    assert stopped.action == "BALL_LOST_STOP"
    assert stopped.source_command["linear_speed_mps"] == 0.0
    assert stopped.reason == "ball_lost_stop_before_ball_turn"
    assert turning.action == "BALL_APPROACH_TURN_RIGHT_5"
    assert turning.source_command["linear_speed_mps"] == 0.0
    assert turning.source_command["last_seen_ball_side"] == "RIGHT"
    assert turning.source_command["turn_count"] == 5


def test_lost_ball_side_turn_has_no_recovery_timeout():
    planner = MotionDecisionPlanner(
        MotionDecisionConfig(
            enable_ball_lost_recovery=True,
            ball_recovery_timeout_sec=0.1,
        )
    )
    visible_ball = ball_info(
        depth_m=0.85,
        distance_m=0.85,
        offset_x_px=-100,
    )
    left_line = line_info(filtered_lateral_offset_norm=-0.2)
    planner.plan(
        "AUTO",
        observations(line=left_line, ball=visible_ball),
        0.1,
    )

    decision = planner.plan(
        "AUTO",
        observations(line=left_line, ball={"detected": False}),
        10.0,
    )

    assert planner.ball_tracking_active is True
    assert decision.action == "BALL_APPROACH_TURN_LEFT_2"


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
    assert decision.action == "STRAIGHT_2"
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
    assert centering.action == "STRAIGHT_2"
    assert centering.reason == "ball_aligned_discrete_approach"
    assert resumed.source == "ball"
    assert resumed.action == "STRAIGHT_2"
    assert resumed.reason == "ball_aligned_discrete_approach"
    assert planner.ball_recovery_centering is False


def test_goal_at_scoring_range_is_remembered_and_selected():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "AUTO",
        observations(
            line=line_info(),
            goal=goal_info(depth_m=0.8, distance_m=0.8),
        ),
        0.1,
    )

    assert decision.source == "goal"
    assert planner.goal_tracking_active is True


def test_goal_inside_50cm_takes_priority_but_does_not_advance():
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
    assert decision.action == "GOAL_CAMERA90_BACKWARD_1"
    assert decision.reason == "retreat_goal_to_scoring_depth"


def test_goal_search_keeps_line_outside_tracking_range():
    planner = MotionDecisionPlanner()

    decision = planner.plan(
        "GOAL_SEARCH",
        observations(
            line=line_info(),
            goal=goal_info(depth_m=2.01, distance_m=2.01),
        ),
        0.1,
    )

    assert decision.source == "line"


@pytest.mark.parametrize("depth_m", [1.5, 2.0])
def test_far_goal_uses_depth_selected_forward(depth_m):
    planner = MotionDecisionPlanner()
    decision = planner.plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(depth_m=depth_m, distance_m=depth_m)),
        0.1,
    )

    assert planner.goal_tracking_active is True
    assert decision.valid is True
    assert decision.action == "GOAL_CAMERA_90_FORWARD"
    assert decision.reason == "approach_goal_by_depth"


def test_goal_search_approach_phase_requires_controllable_observation():
    planner = MotionDecisionPlanner()

    assert planner.approach_phase_for_search(
        "GOAL_SEARCH",
        observations(goal=goal_info(depth_m=0.5)),
    ) == "GOAL_APPROACH"
    assert planner.approach_phase_for_search(
        "GOAL_SEARCH",
        observations(goal=goal_info(depth_m=2.01)),
    ) is None
    assert planner.approach_phase_for_search(
        "GOAL_SEARCH",
        observations(goal=None),
    ) is None


def test_goal_approach_selects_four_fine_repeats_at_one_meter():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(depth_m=1.0, distance_m=1.0)),
        0.1,
    )

    assert decision.source == "goal"
    assert decision.valid is True
    assert decision.action == "GOAL_CAMERA90_FINE_FORWARD_4"
    assert decision.reason == "approach_goal_by_depth"


@pytest.mark.parametrize(
    ("depth", "expected_action", "reason"),
    [
        (0.90, "GOAL_CAMERA90_FINE_FORWARD_2", "approach_goal_by_depth"),
        (0.80, "WAIT_SCORE_CONFIRMATION", "waiting_for_stable_score_condition"),
        (0.65, "GOAL_CAMERA90_BACKWARD_1", "retreat_goal_to_scoring_depth"),
        (0.50, "GOAL_CAMERA90_BACKWARD_1", "retreat_goal_to_scoring_depth"),
        (0.35, "GOAL_CAMERA90_BACKWARD_1", "retreat_goal_to_scoring_depth"),
        (0.20, "GOAL_CAMERA90_BACKWARD_1", "retreat_goal_to_scoring_depth"),
        (0.12, "GOAL_CAMERA90_BACKWARD_1", "retreat_goal_to_scoring_depth"),
    ],
)
def test_goal_depth_approach_and_near_target_hold(
    depth,
    expected_action,
    reason,
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
    assert decision.valid is (depth > 0.82 or depth < 0.77)
    assert decision.reason == reason


def post_ball_return_line(**overrides):
    """Synthetic near-line geometry matching the reported left-side case."""
    return line_info(**{
        "filtered_heading_error_deg": 24.591,
        "filtered_lateral_offset_norm": -0.928512,
        "image_width": 1280,
        "image_height": 720,
        "robot_center_x_px": 710,
        "center_points_px": [[80, 680], [120, 590], [155, 510],
                             [230, 385], [620, 250]],
        "corner_start_index": 3,
        **overrides,
    })


@pytest.mark.parametrize("heading,direction,count", [
    (-35.0, "LEFT", 1), (24.591, "RIGHT", 2),
])
@pytest.mark.parametrize("offset", [-0.93, -0.451, -0.449, 0.449, 0.451, 0.93])
def test_post_ball_alignment_keeps_heading_reference_across_offset_boundary(
    heading, direction, count, offset,
):
    info = post_ball_return_line(
        filtered_heading_error_deg=heading,
        filtered_lateral_offset_norm=offset,
    )
    result = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN", observations(line=info), 0.1,
    )
    assert result.action == f"POST_BALL_LINE_TURN_{direction}_{count}"
    assert result.source_command["alignment_reference"] == "line_heading"
    assert result.source_command["steering_error_deg"] == heading
    assert result.source_command["turn_angle_deg"] == (15.0 if direction == "RIGHT" else 30.0)


@pytest.mark.parametrize("overrides", [
    {"center_points_px": None},
    {"center_points_px": [[80, 680]]},
    {"center_points_px": [[80, 680], [120, float("nan")]]},
    {"center_points_px": [[80, 680], [120, 700]]},
    {"center_points_px": [[80, 680], [120, 650]]},
    {"center_points_px": [[80, 680], [120, 300]]},
    {"center_points_px": [[80, 680], [-1, 590]]},
    {"robot_center_x_px": None},
    {"image_height": 0},
    {"corner_start_index": 1},
    {"corner_start_index": "invalid"},
])
def test_post_ball_heading_alignment_does_not_require_near_geometry(overrides):
    result = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(line=post_ball_return_line(**overrides)), 0.1,
    )
    assert result.action == "POST_BALL_LINE_TURN_RIGHT_2"
    assert result.valid is True
    assert result.reason == "post_ball_line_heading_correction"


def test_recorded_post_ball_alignment_finishes_when_heading_is_corrected():
    planner = MotionDecisionPlanner()
    before = post_ball_return_line(
        filtered_heading_error_deg=18.904,
        filtered_lateral_offset_norm=-0.351468,
    )
    assert planner.plan(
        "POST_BALL_LINE_ALIGN", observations(line=before), 0.1,
    ).action == "POST_BALL_LINE_TURN_RIGHT_2"
    after = post_ball_return_line(
        filtered_heading_error_deg=13.656,
        filtered_lateral_offset_norm=-0.756465,
        center_points_px=[[710, 700], [240, 537]],
        corner_start_index=None,
    )
    aligned = planner.plan("POST_BALL_LINE_ALIGN", observations(line=after), 0.1)
    assert aligned.action == "POST_BALL_LINE_ALIGNED"
    assert aligned.source_command["alignment_reference"] == "line_heading"
    assert not aligned.source_command["offset_in_tolerance"]


def test_post_ball_and_post_shot_use_same_heading_alignment():
    planner = MotionDecisionPlanner()
    info = post_ball_return_line()
    assert planner.plan(
        "POST_BALL_LINE_ALIGN", observations(line=info), 0.1,
    ).action == "POST_BALL_LINE_TURN_RIGHT_2"
    result = planner.plan("POST_SHOT_LINE_ALIGN", observations(line=info), 0.1)
    assert result.action == "POST_SHOT_LINE_TURN_RIGHT_2"


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
    assert decision.source_command["heading_tolerance_deg"] == 15.0
    assert decision.source_command["offset_in_tolerance"] is False


@pytest.mark.parametrize(
    ("heading", "expected_action", "expected_count"),
    [
        (15.0, "POST_BALL_LINE_TURN_RIGHT_2", 2),
        (34.0, "POST_BALL_LINE_TURN_RIGHT_3", 3),
        (88.0, "POST_BALL_LINE_TURN_RIGHT_7", 7),
        (-31.0, "POST_BALL_LINE_TURN_LEFT_1", 1),
    ],
)
def test_post_ball_line_heading_selects_direction_calibrated_repeat_count(
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


@pytest.mark.parametrize(
    ("angle", "count", "estimated_angle"),
    [
        (15.001, 2, 15.0), (15.0, 2, 15.0),
        (22.5, 2, 15.0), (29.999, 2, 15.0),
        (30.0, 3, 30.0), (37.5, 3, 30.0),
        (44.999, 3, 30.0), (45.0, 5, 45.0),
        (55.0, 5, 45.0), (64.999, 5, 45.0),
        (65.0, 7, 65.0), (80.0, 7, 65.0),
        (94.999, 7, 65.0), (95.0, 9, 95.0), (180.0, 9, 95.0),
    ],
)
def test_stationary_right_turn_calibration_in_line_and_goal_paths(
    angle, count, estimated_angle,
):
    planner = MotionDecisionPlanner()
    decisions = (
        (
            planner.plan(
                "POST_BALL_LINE_ALIGN",
                observations(line=line_info(filtered_heading_error_deg=angle)),
                0.1,
            ),
            "POST_BALL_LINE",
        ),
        (
            planner.plan(
                "GOAL_APPROACH",
                observations(goal=goal_info(depth_m=0.9, bearing_deg=angle)),
                0.1,
            ),
            "GOAL_CAMERA90",
        ),
    )
    for decision, prefix in decisions:
        assert decision.valid is True
        assert decision.action == f"{prefix}_TURN_RIGHT_{count}"
        assert decision.source_command["turn_count"] == count
        assert decision.source_command["turn_repeat_deg"] is None
        assert decision.source_command["turn_angle_deg"] == estimated_angle


def test_lost_ball_recovery_right_turn_is_incremental():
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(offset_x_px=100), 0.0)
    planner.ball_lost_elapsed_sec = 1.0
    command = planner._lost_ball_recovery_command()
    assert command["motion"] == "BALL_APPROACH_TURN_RIGHT_5"
    assert command["target_heading_change_deg"] == 45.0


@pytest.mark.parametrize("angle,count,expected_angle", [
    (30.001, 1, 30.0), (44.999, 1, 30.0), (30.0, 1, 30.0),
    (37.5, 1, 30.0), (37.6, 1, 30.0), (45.0, 2, 45.0),
    (52.5, 2, 45.0), (52.6, 2, 45.0), (60.0, 3, 60.0),
    (67.5, 3, 60.0), (67.6, 3, 60.0), (75.0, 4, 75.0),
    (82.5, 4, 75.0), (82.6, 4, 75.0), (90.0, 5, 90.0),
    (97.5, 5, 90.0), (97.6, 5, 90.0), (105.0, 5, 90.0),
    (180.0, 5, 90.0),
])
def test_line_left_turn_keeps_existing_calibration(angle, count, expected_angle):
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(line=line_info(filtered_heading_error_deg=-angle)), 0.1,
    )
    assert decision.valid is True
    assert decision.action.endswith(f"TURN_LEFT_{count}")
    assert decision.source_command["turn_angle_deg"] == expected_angle
    assert decision.source_command["turn_repeat_deg"] is None


def test_last_raw_image_half_reverses_search_without_confirming_new_target():
    planner = MotionDecisionPlanner()
    raw = {"detected": False, "raw_detected": True, "confidence": 0.9,
           "camera_center_offset_x_px": 420, "depth_valid": False}
    planner._update_ball_tracking(raw, 0.0)
    assert planner.ball_tracking_active is False
    assert planner.last_ball_turn_direction is None
    planner._update_ball_tracking(ball_info(camera_center_offset_x_px=-200), 0.0)
    assert planner.last_ball_turn_direction == "LEFT"
    planner._update_ball_tracking(raw, 0.0)
    assert planner.last_ball_turn_direction == "RIGHT"
    assert planner.plan_ball_pickup_initial_alignment(raw).action == "WAIT"
    lost = {"detected": False, "raw_detected": False}
    assert planner.plan_lost_ball_approach_alignment(lost).action == "BALL_APPROACH_TURN_RIGHT_5"
    assert planner.plan_ball_pickup_initial_alignment(lost).action == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_5"
    assert planner.plan_ball_pickup_fine_alignment(lost).action == "BALL_PICKUP_FINE_SEARCH_RIGHT"
    planner._update_ball_tracking({**raw, "camera_center_offset_x_px": -400}, 0.0)
    result = planner.plan_lost_ball_approach_alignment(lost)
    assert result.action == "BALL_APPROACH_TURN_LEFT_2"
    assert result.source_command["turn_angle_deg"] == 45.0


def test_centered_ball_loss_without_side_history_waits():
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(camera_center_offset_x_px=0), 0.0)
    assert planner.ball_tracking_active is True
    assert planner.last_ball_turn_direction is None
    decision = planner.plan_lost_ball_approach_alignment({"detected": False})
    assert decision.action == "WAIT"
    assert decision.reason == "lost_ball_alignment_has_no_remembered_ball_side"
    assert decision.valid is False


def test_image_half_memory_rejects_weak_raw_and_preserves_side_at_center():
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(camera_center_offset_x_px=-20), 0.0)
    for sample in (
        {"detected": False, "raw_detected": True, "confidence": 0.1,
         "camera_center_offset_x_px": 500},
        ball_info(camera_center_offset_x_px=0),
        ball_info(camera_center_offset_x_px=None, offset_x_px=300),
        None,
    ):
        planner._update_ball_tracking(sample, 0.0)
        assert planner.last_ball_turn_direction == "LEFT"
    planner._update_ball_tracking(ball_info(
        camera_center_offset_x_px=None, center_x=900, image_width=1280), 0.0)
    assert planner.last_ball_turn_direction == "RIGHT"


def test_post_ball_line_uses_calibrated_left_count():
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(
            line=line_info(filtered_heading_error_deg=-51.0)
        ),
        0.1,
    )

    assert decision.action == "POST_BALL_LINE_TURN_LEFT_2"
    assert decision.valid is True
    assert decision.source_command["turn_angle_deg"] == 45.0
    assert decision.source_command["catalog_motion_available"] is True


@pytest.mark.parametrize(
    "direction,count,angle", [("RIGHT", 2, 15.0), ("LEFT", 1, 30.0)]
)
def test_post_ball_line_missing_detection_searches_without_forward(
    direction, count, angle
):
    planner = MotionDecisionPlanner()
    planner.post_ball_line_search_direction = direction
    decision = planner.plan(
        "POST_BALL_LINE_ALIGN",
        observations(line={"detected": False}),
        0.1,
    )

    assert decision.action == f"POST_BALL_LINE_TURN_{direction}_{count}"
    assert decision.valid is True
    assert decision.reason == "post_ball_line_search"
    assert decision.source_command["turn_angle_deg"] == angle


@pytest.mark.parametrize("line", [None, {}, {"detected": None}])
def test_post_ball_line_without_valid_detection_status_waits(line):
    decision = MotionDecisionPlanner().plan(
        "POST_BALL_LINE_ALIGN",
        observations(line=line),
        0.1,
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == (
        "invalid_vision_boolean_type" if line == {"detected": None}
        else "post_ball_line_not_detected"
    )


def test_goal_camera90_right_turn_count_uses_bearing_error():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.9,
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
                depth_m=0.9,
                bearing_deg=-30.0,
                offset_x_norm=-0.3,
            )
        ),
        0.1,
    )

    assert decision.action == "GOAL_CAMERA90_TURN_LEFT_1"
    assert decision.valid is True
    assert decision.source_command["turn_count"] == 1
    assert decision.source_command["catalog_motion_available"] is True


@pytest.mark.parametrize(
    ("bearing_deg", "expected_action", "expected_count"),
    [
        (-15.001, "GOAL_CAMERA90_TURN_LEFT_1", 1),
        (-20.0, "GOAL_CAMERA90_TURN_LEFT_1", 1),
        (-30.0, "GOAL_CAMERA90_TURN_LEFT_1", 1),
        (-34.0, "GOAL_CAMERA90_TURN_LEFT_1", 1),
        (-45.0, "GOAL_CAMERA90_TURN_LEFT_1", 1),
        (-45.001, "GOAL_CAMERA90_TURN_LEFT_2", 2),
        (-58.0, "GOAL_CAMERA90_TURN_LEFT_2", 2),
        (-60.0, "GOAL_CAMERA90_TURN_LEFT_3", 3),
        (-70.0, "GOAL_CAMERA90_TURN_LEFT_3", 3),
        (-75.0, "GOAL_CAMERA90_TURN_LEFT_4", 4),
        (-89.999, "GOAL_CAMERA90_TURN_LEFT_4", 4),
        (-90.0, "GOAL_CAMERA90_TURN_LEFT_5", 5),
        (-105.0, "GOAL_CAMERA90_TURN_LEFT_5", 5),
    ],
)
def test_goal_camera90_left_turn_uses_goal_only_thresholds(
    bearing_deg,
    expected_action,
    expected_count,
):
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.9,
                bearing_deg=bearing_deg,
                offset_x_norm=-0.3,
            )
        ),
        0.1,
    )

    assert decision.action == expected_action
    assert decision.source_command["turn_count"] == expected_count
    assert decision.source_command["turn_angle_deg"] == {
        1: 30.0, 2: 45.0, 3: 60.0, 4: 75.0, 5: 90.0,
    }[expected_count]
    assert "GOAL_CAMERA90_TURN_LEFT_7" != decision.action


@pytest.mark.parametrize("bearing", [-15.0, -14.999, -5.0, 0.0])
def test_goal_left_deadband_keeps_distance_based_forward(bearing):
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(
            depth_m=1.2, bearing_deg=bearing, offset_x_norm=-0.3,
        )), 0.1,
    )
    assert decision.action == "GOAL_CAMERA_90_FORWARD_2"


@pytest.mark.parametrize("angle,count", [(20.0, 0), (30.0, 1), (45.0, 2)])
def test_goal_left_thresholds_do_not_change_ball_or_line_turns(angle, count):
    planner = MotionDecisionPlanner()
    ball = planner.plan_ball_approach_alignment(
        ball_info(steering_angle_deg=-angle, bearing_deg=-angle),
    )
    assert ball.action == (
        f"BALL_APPROACH_TURN_LEFT_{count}" if count
        else "BALL_APPROACH_ALIGNED"
    )
    line = planner.plan(
        "POST_BALL_LINE_ALIGN",
        observations(line=line_info(filtered_heading_error_deg=-angle)), 0.1,
    )
    assert line.action == (
        f"POST_BALL_LINE_TURN_LEFT_{count}" if count
        else "POST_BALL_LINE_ALIGNED"
    )


@pytest.mark.parametrize("bearing,count", [
    (14.999, 0), (15.0, 2), (29.999, 2), (30.0, 3), (44.999, 3),
    (45.0, 5), (64.999, 5), (65.0, 7), (94.999, 7), (95.0, 9),
])
def test_goal_right_turn_thresholds_are_unchanged(bearing, count):
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(goal=goal_info(
            depth_m=1.2, bearing_deg=bearing, offset_x_norm=0.3,
        )), 0.1,
    )
    assert decision.action == (
        f"GOAL_CAMERA90_TURN_RIGHT_{count}" if count
        else "GOAL_CAMERA_90_FORWARD_2"
    )


@pytest.mark.parametrize(
    ("offset_x_norm", "expected_action", "direction"),
    [
        (0.20, "GOAL_CAMERA90_CRAB_RIGHT", "RIGHT"),
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
                depth_m=0.795,
                bearing_deg=0.0,
                offset_x_norm=offset_x_norm,
            )
        ),
        0.1,
    )

    assert decision.action == expected_action
    assert decision.reason == "align_goal_lateral_camera90"
    assert decision.action.endswith(direction)


def test_goal_camera90_yaw_has_priority_over_lateral_offset():
    decision = MotionDecisionPlanner().plan(
        "GOAL_APPROACH",
        observations(
            goal=goal_info(
                depth_m=0.9,
                bearing_deg=34.0,
                offset_x_norm=0.3,
            )
        ),
        0.1,
    )

    assert decision.action == "GOAL_CAMERA90_TURN_RIGHT_3"


def test_goal_scoring_range_uses_crab_even_with_large_bearing():
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

    assert decision.action == "GOAL_CAMERA90_CRAB_RIGHT"


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
                bearing_deg=15.0,
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
    assert centering.action == "GOAL_CAMERA90_TURN_RIGHT_2"
    assert resumed.source == "goal"
    assert resumed.action == "GOAL_CAMERA90_FINE_FORWARD_4"
    assert resumed.reason == "approach_goal_by_depth"
    assert planner.goal_recovery_centering is False


def test_reacquired_goal_inside_tracking_range_aligns_then_approaches():
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
                bearing_deg=15.0,
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
    assert centering.action == "GOAL_CAMERA90_TURN_RIGHT_2"
    assert resumed.source == "goal"
    assert resumed.action == "GOAL_CAMERA90_FINE_FORWARD_4"
    assert resumed.reason == "approach_goal_by_depth"
    assert planner.goal_recovery_centering is False


@pytest.mark.parametrize('depth', [0.77, 0.795, 0.82])
@pytest.mark.parametrize('offset_px', [-40, 0, 100])
def test_goal_shot_rectangle_takes_priority_over_bearing(depth, offset_px):
    decision = MotionDecisionPlanner().plan(
        'GOAL_APPROACH',
        observations(goal=goal_info(
            depth_m=depth, offset_x_px=offset_px, bearing_deg=20.0,
        )), 0.1,
    )
    assert decision.action == 'SHOT'
    assert decision.requires_ack is True


@pytest.mark.parametrize('offset_px,direction', [(-41, 'LEFT'), (101, 'RIGHT')])
@pytest.mark.parametrize('bearing', [-20.0, 20.0])
def test_goal_at_scoring_depth_uses_crab_not_yaw(offset_px, direction, bearing):
    decision = MotionDecisionPlanner().plan(
        'GOAL_APPROACH',
        observations(goal=goal_info(offset_x_px=offset_px, bearing_deg=bearing)), 0.1,
    )
    assert decision.action == f'GOAL_CAMERA90_CRAB_{direction}'


@pytest.mark.parametrize('depth,forward', [
    (2.0, 'GOAL_CAMERA_90_FORWARD'),
    (1.280, 'GOAL_CAMERA_90_FORWARD_2'),
    (1.025, 'GOAL_CAMERA90_FINE_FORWARD_4'),
    (0.985, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.950, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (0.875, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.821, 'GOAL_CAMERA90_FINE_FORWARD_1'),
])
@pytest.mark.parametrize('bearing,offset_px,turn', [
    (4.0, 200, None),
    (14.0, 200, None),
    (-15.0, -200, None),
    (-29.0, -200, 'GOAL_CAMERA90_TURN_LEFT_1'),
    (15.0, 200, 'GOAL_CAMERA90_TURN_RIGHT_2'),
    (34.0, 200, 'GOAL_CAMERA90_TURN_RIGHT_3'),
    (-30.0, -200, 'GOAL_CAMERA90_TURN_LEFT_1'),
    (-58.0, -200, 'GOAL_CAMERA90_TURN_LEFT_2'),
])
def test_goal_approach_only_turns_or_advances_before_scoring_depth(
    depth, forward, bearing, offset_px, turn,
):
    decision = MotionDecisionPlanner().plan(
        'GOAL_APPROACH',
        observations(goal=goal_info(
            depth_m=depth, bearing_deg=bearing,
            offset_x_px=offset_px, offset_x_norm=offset_px / 640,
            score_now=False,
        )), 0.1,
    )
    assert decision.action == (turn or forward)
    assert decision.valid is True


def test_successful_latched_pickup_releases_ball_owner_for_goal():
    planner = MotionDecisionPlanner()
    planner.ball_lock_active = True
    planner.ball_terminal_requested = False
    planner.clear_collected_ball_tracking()
    decision = planner.plan('GOAL_APPROACH', observations(goal=goal_info()), 0.1)
    assert planner.ball_lock_active is False
    assert decision.source == 'goal'
    assert decision.action == 'SHOT'


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


@pytest.mark.parametrize("normal_phase", ["AUTO", "LINE_TRACK", "BALL_SEARCH"])
def test_corner_turns_are_disabled_only_during_post_pickup_line_run(normal_phase):
    planner = MotionDecisionPlanner()
    sample = observations(line=line_info(
        filtered_heading_error_deg=8.9,
        filtered_lateral_offset_norm=-0.256,
        turn_angle_deg=63.0,
        corner_preview_confirmed=True,
        corner_start_distance_m=0.74,
    ))
    for _ in range(3):
        normal = planner.plan(normal_phase, sample, 0.1)
    assert normal.action == "RIGHT"

    for _ in range(5):
        restricted = planner.plan("LINE_TRACK_AFTER_PICKUP", sample, 0.1)
        assert restricted.action == "STRAIGHT"
        assert restricted.source == "line"
        assert restricted.valid is True

    for _ in range(3):
        normal = planner.plan(normal_phase, sample, 0.1)
    assert normal.action == "RIGHT"


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


def test_pickup_initial_alignment_prefers_steering_and_selects_camera_turn():
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            depth_m=0.39,
            steering_angle_deg=26.0,
            bearing_deg=-18.0,
            offset_x_norm=-0.2,
        )
    )

    assert decision.action == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_2"
    assert decision.valid is True
    assert decision.sdk_motion_requested is True
    assert decision.source_command["steering_error_deg"] == 26.0


def test_pickup_initial_alignment_uses_fifteen_degree_turn_threshold():
    planner = MotionDecisionPlanner()

    aligned = planner.plan_ball_pickup_initial_alignment(
        ball_info(steering_angle_deg=14.999, bottom_distance_px=500)
    )
    turn = planner.plan_ball_pickup_initial_alignment(
        ball_info(steering_angle_deg=15.0, bottom_distance_px=500)
    )

    assert aligned.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert aligned.source_command["heading_tolerance_deg"] == 15.0
    assert turn.action == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_2"


@pytest.mark.parametrize(
    (
        "steering_error",
        "expected_action",
        "expected_count",
        "expected_turn_angle",
    ),
    [
        (26.0, "BALL_APPROACH_TURN_RIGHT_2", 2, 15.0),
        (88.0, "BALL_APPROACH_TURN_RIGHT_7", 7, 65.0),
        (-30.0, "BALL_APPROACH_TURN_LEFT_1", 1, 30.0),
        (-50.0, "BALL_APPROACH_TURN_LEFT_2", 2, 45.0),
        (-88.0, "BALL_APPROACH_TURN_LEFT_4", 4, 75.0),
    ],
)
def test_general_ball_alignment_uses_ordinary_stationary_turns(
    steering_error,
    expected_action,
    expected_count,
    expected_turn_angle,
):
    decision = MotionDecisionPlanner().plan_ball_approach_alignment(
        ball_info(steering_angle_deg=steering_error, distance_m=0.9)
    )

    assert decision.action == expected_action
    assert decision.source_command["turn_count"] == expected_count
    assert decision.source_command["turn_angle_deg"] == expected_turn_angle
    assert "CAMERA_DOWN" not in decision.action


def test_general_ball_alignment_skips_turn_inside_existing_tolerance():
    decision = MotionDecisionPlanner().plan_ball_approach_alignment(
        ball_info(steering_angle_deg=4.9, distance_m=0.9)
    )

    assert decision.action == "BALL_APPROACH_ALIGNED"
    assert decision.sdk_motion_requested is False


def test_pickup_initial_alignment_uses_bearing_then_offset_fallback():
    planner = MotionDecisionPlanner()

    bearing_decision = planner.plan_ball_pickup_initial_alignment(
        ball_info(steering_angle_deg=None, bearing_deg=-30.0)
    )
    offset_decision = planner.plan_ball_pickup_initial_alignment(
        ball_info(
            steering_angle_deg=None,
            bearing_deg=None,
            offset_x_norm=0.4,
        )
    )

    assert bearing_decision.action == "BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_1"
    assert offset_decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"


def test_pickup_initial_alignment_selects_fine_step_from_distance():
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            depth_m=0.30,
            bottom_distance_px=500,
            steering_angle_deg=2.0,
            pickup_ready=True,
        )
    )

    assert decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert decision.valid is True
    assert decision.source_command["depth_m"] == 0.30
    assert decision.source_command["pickup_approach_motion"] == "STRAIGHT_0"


@pytest.mark.parametrize("offset_px", [-70, 70])
def test_pickup_fine_forward_accepts_inclusive_robot_dx_window(offset_px):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            distance_m=0.570,
            offset_x_px=offset_px,
            bottom_distance_px=200,
            steering_angle_deg=0.0,
        )
    )

    assert decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert decision.source_command["pickup_approach_motion"] == "STRAIGHT_0"


@pytest.mark.parametrize(
    ("offset_px", "expected_action"),
    [
        (-71, "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"),
        (71, "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"),
    ],
)
def test_pickup_pixel_offset_does_not_force_turn_below_fifteen_degrees(
    offset_px,
    expected_action,
):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            offset_x_px=offset_px,
            bottom_distance_px=101,
            steering_angle_deg=0.0,
        )
    )

    assert decision.action == expected_action
    assert decision.sdk_motion_requested is True


@pytest.mark.parametrize(
    ("offset_px", "expected_action"),
    [
        (-71, "BALL_PICKUP_INITIAL_CRAB_LEFT"),
        (71, "BALL_PICKUP_INITIAL_CRAB_RIGHT"),
    ],
)
@pytest.mark.parametrize("bottom_distance", [0, 99, 100])
def test_close_pickup_alignment_uses_only_crab(offset_px, expected_action, bottom_distance):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            offset_x_px=offset_px,
            bottom_distance_px=bottom_distance,
            steering_angle_deg=35.0,
        )
    )

    assert decision.action == expected_action
    assert "TURN" not in decision.action


def test_close_centered_pickup_does_not_use_stationary_turn():
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            distance_m=0.570,
            offset_x_px=70,
            bottom_distance_px=100,
            steering_angle_deg=35.0,
        )
    )

    assert decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert decision.source_command["pickup_approach_motion"] == "STRAIGHT_0"


def test_pickup_initial_alignment_waits_when_ball_not_visible():
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        {"detected": False}
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "ball_pickup_initial_alignment_waiting_for_ball"


def test_post_backward_alignment_waits_until_ball_is_visible():
    decision = (
        MotionDecisionPlanner().plan_ball_pickup_post_backward_alignment(
            ball_info(detected=False, pickup_x_tolerance_norm=0.08)
        )
    )

    assert decision.action == "WAIT"
    assert decision.valid is False


def test_post_backward_alignment_corrects_heading_before_lateral_offset():
    decision = (
        MotionDecisionPlanner().plan_ball_pickup_post_backward_alignment(
            ball_info(
                steering_angle_deg=31.0,
                offset_x_norm=-0.20,
                pickup_x_tolerance_norm=0.08,
            )
        )
    )

    assert decision.action == "BALL_PICKUP_POST_BACKWARD_TURN_RIGHT_3"
    assert decision.valid is True


def test_post_backward_alignment_corrects_lateral_offset_after_heading():
    decision = (
        MotionDecisionPlanner().plan_ball_pickup_post_backward_alignment(
            ball_info(
                steering_angle_deg=0.0,
                offset_x_norm=-0.20,
                pickup_x_tolerance_norm=0.08,
            )
        )
    )

    assert decision.action == "BALL_PICKUP_POST_BACKWARD_CRAB_LEFT"
    assert decision.valid is True


def test_post_backward_alignment_continues_when_ball_is_centered():
    decision = (
        MotionDecisionPlanner().plan_ball_pickup_post_backward_alignment(
            ball_info(
                steering_angle_deg=0.0,
                offset_x_norm=0.02,
                pickup_x_tolerance_norm=0.08,
            )
        )
    )

    assert decision.action == "BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE"
    assert decision.valid is True


@pytest.mark.parametrize(
    ("robot_offset", "expected_action", "expected_count"),
    [(100, "BALL_APPROACH_TURN_RIGHT_5", 5),
     (-100, "BALL_APPROACH_TURN_LEFT_2", 2)],
)
def test_lost_ball_alignment_uses_image_half_not_robot_side(
    robot_offset, expected_action, expected_count,
):
    decision = MotionDecisionPlanner().plan_lost_ball_approach_alignment(
        ball_info(camera_center_offset_x_px=robot_offset,
                  offset_x_px=-robot_offset, bearing_deg=-robot_offset / 10)
    )
    assert decision.action == expected_action
    assert decision.valid is True
    assert decision.source_command["turn_count"] == expected_count
    assert decision.source_command["lost_ball_alignment_from_memory"] is True
    assert decision.source_command["alignment_reference"] == "ball_side"


@pytest.mark.parametrize("bottom_distance_px", [0, 500, 700])
@pytest.mark.parametrize(
    ("distance_m", "expected_motion"),
    [
        (0.670, "STRAIGHT_2"), (0.571, "STRAIGHT_2"),
        (0.570, "STRAIGHT_0"), (0.569, "STRAIGHT_0"),
        (0.470, "STRAIGHT_0"), (0.10, "STRAIGHT_0"),
    ],
)
def test_pickup_initial_alignment_uses_metric_distance_threshold(
    distance_m, expected_motion, bottom_distance_px,
):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            depth_m=0.80,
            distance_m=distance_m,
            ground_distance_m=0.05,
            bottom_distance_px=bottom_distance_px,
            steering_angle_deg=0.0,
        )
    )

    assert decision.action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert decision.source_command["pickup_approach_motion"] == expected_motion


def test_pickup_fine_step_distance_threshold_is_configurable():
    planner = MotionDecisionPlanner(
        MotionDecisionConfig(pickup_fine_step_distance_m=0.40)
    )
    decision = planner.plan_ball_pickup_initial_alignment(
        ball_info(distance_m=0.401, bottom_distance_px=80, steering_angle_deg=0.0)
    )

    assert decision.source_command["pickup_approach_motion"] == "STRAIGHT_2"
    assert decision.source_command["pickup_fine_step_distance_m"] == 0.40


def test_pickup_initial_alignment_waits_when_bottom_distance_is_missing():
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(
            depth_m=0.30,
            distance_m=None,
            ground_distance_m=0.10,
            bottom_distance_px=None,
            steering_angle_deg=0.0,
        )
    )

    assert decision.action == "WAIT"
    assert decision.reason == "ball_pickup_waiting_for_bottom_distance_px"


@pytest.mark.parametrize(
    "overrides",
    [
        {"depth_valid": False},
        {"depth_age_sec": None},
        {"depth_age_sec": 0.701},
        {"depth_age_sec": -0.01},
        {"distance_m": None},
        {"distance_m": 0.0},
        {"distance_m": -0.1},
        {"distance_m": float("nan")},
        {"distance_m": float("inf")},
    ],
)
def test_pickup_initial_alignment_requires_fresh_distance_when_aligned(
    overrides,
):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(
        ball_info(steering_angle_deg=0.0, **overrides)
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.reason == "ball_pickup_waiting_for_fresh_distance"
    assert decision.sdk_motion_requested is False


@pytest.mark.parametrize(
    ("offset_px", "expected_action"),
    [
        (-31, "BALL_PICKUP_CRAB_LEFT"),
        (-30, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
        (0, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
        (50, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
        (51, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
        (55, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
        (56, "BALL_PICKUP_CRAB_RIGHT"),
    ],
)
def test_pickup_fine_alignment_uses_asymmetric_pixel_window(
    offset_px,
    expected_action,
):
    planner = MotionDecisionPlanner()

    decision = planner.plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=200,
            offset_x_px=offset_px,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == expected_action
    assert decision.valid is True
    assert decision.source == "ball"
    assert decision.source_command["offset_x_px"] == offset_px
    assert decision.source_command["pickup_left_bound_px"] == -30.0
    assert decision.source_command["pickup_right_bound_px"] == 55.0
    assert decision.source_command["catalog_motion_available"] is True


@pytest.mark.parametrize("bottom_distance_px", [301, 400, 500, 600])
@pytest.mark.parametrize("offset_px", [-31, 0, 51])
def test_pickup_repeats_fine_approach_before_crab_or_backward(
    bottom_distance_px,
    offset_px,
):
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=bottom_distance_px,
            offset_x_px=offset_px,
            pickup_ready=True,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == "BALL_PICKUP_FINE_FORWARD"
    assert decision.valid is True
    assert decision.sdk_motion_requested is True
    assert decision.source_command["bottom_distance_px"] == bottom_distance_px


@pytest.mark.parametrize(
    "bottom_distance_px",
    [None, -1, float("nan"), float("inf"), "invalid", True],
)
def test_pickup_fine_alignment_waits_without_valid_pixel_distance(
    bottom_distance_px,
):
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(bottom_distance_px=bottom_distance_px)
    )

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.sdk_motion_requested is False


@pytest.mark.parametrize(
    ("bottom_distance_px", "expected_action"),
    [
        (81, "BALL_PICKUP_FINE_FORWARD"),
        (80, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
        (0, "BALL_PICKUP_FINE_ALIGN_CONTINUE"),
    ],
)
def test_pickup_fine_alignment_distance_is_separate_from_entry_threshold(
    bottom_distance_px,
    expected_action,
):
    planner = MotionDecisionPlanner(
        MotionDecisionConfig(
            pickup_fine_step_distance_m=0.570,
            pickup_fine_align_bottom_distance_px=80,
        )
    )
    info = ball_info(distance_m=0.570, bottom_distance_px=bottom_distance_px)

    initial = planner.plan_ball_pickup_initial_alignment(info)
    fine = planner.plan_ball_pickup_fine_alignment(info)

    assert initial.source_command["pickup_approach_motion"] == "STRAIGHT_0"
    assert fine.action == expected_action
    assert fine.source_command["pickup_fine_align_bottom_distance_px"] == 80


def test_centered_ball_not_ready_starts_pre_grab_sequence():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=200,
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            ground_distance_m=0.20,
            pickup_ready=False,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    assert decision.valid is True
    assert decision.sdk_motion_requested is False


def test_centered_ball_outside_depth_window_starts_pre_grab_sequence():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=200,
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            pickup_ready=False,
            is_in_pickup_window=False,
        )
    )

    assert decision.action == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    assert decision.valid is True


def test_centered_ball_does_not_require_depth_to_start_pre_grab_sequence():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=200,
            offset_x_norm=0.0,
            pickup_x_tolerance_norm=0.08,
            ground_distance_m=0.20,
            pickup_ready=False,
            is_in_pickup_window=True,
            depth_valid=False,
            depth_age_sec=None,
        )
    )

    assert decision.action == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    assert decision.valid is True
    assert decision.sdk_motion_requested is False


def test_lateral_correction_remains_available_during_depth_dropout():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=200,
            offset_x_norm=0.20,
            offset_x_px=56,
            depth_valid=False,
            depth_age_sec=None,
            pickup_ready=False,
            is_in_pickup_window=False,
        )
    )

    assert decision.action == "BALL_PICKUP_CRAB_RIGHT"
    assert decision.valid is True


def test_fixed_fine_motion_is_not_repeated_when_depth_drops_out():
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(
            bottom_distance_px=200,
            offset_x_norm=0.0,
            offset_x_px=0,
            depth_valid=False,
            depth_age_sec=0.05,
            pickup_ready=True,
            is_in_pickup_window=True,
        )
    )

    assert decision.action == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    assert decision.valid is True
    assert decision.sdk_motion_requested is False


@pytest.mark.parametrize(
    "info",
    [
        None,
        ball_info(detected=False),
        ball_info(confidence=0.1),
        ball_info(offset_x_px=None),
    ],
)
def test_pickup_fine_alignment_missing_or_invalid_ball_waits(info):
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(info)

    assert decision.action == "WAIT"
    assert decision.valid is False
    assert decision.sdk_motion_requested is False


@pytest.mark.parametrize("distance,expected", [(1.5, True), (1.51, False), (0.4, True)])
def test_ball_loss_recovery_requires_confirmed_control_range(distance, expected):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(distance_m=distance, offset_x_px=-80), 0.0)
    result = planner.plan_ball_pickup_initial_alignment({"detected": False})
    assert result.valid is expected
    assert (result.action == "BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_2") is expected


@pytest.mark.parametrize("fine", [False, True])
@pytest.mark.parametrize("offset", [-100, 100])
def test_close_pickup_loss_uses_existing_search_and_reacquires(fine, offset):
    planner = MotionDecisionPlanner()
    close = ball_info(distance_m=0.4, offset_x_px=offset, bottom_distance_px=100)
    planner._update_ball_tracking(close, 0.0)
    plan = (planner.plan_ball_pickup_fine_alignment if fine
            else planner.plan_ball_pickup_initial_alignment)
    plan(close)
    direction = "RIGHT" if offset > 0 else "LEFT"
    expected = (
        f"BALL_PICKUP_FINE_SEARCH_{direction}" if fine
        else f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{5 if offset > 0 else 2}"
    )
    for _ in range(3):
        decision = plan({"detected": False, "raw_detected": False})
        assert decision.action == expected
        assert decision.reason == "ball_lost_turn_toward_last_ball_side"
        assert decision.sdk_motion_requested
    centered = ball_info(distance_m=0.4, offset_x_px=0, bottom_distance_px=90)
    assert plan(centered).action.endswith("ALIGN_CONTINUE")
    planner.clear_collected_ball_tracking()
    assert not planner.pickup_close_alignment_active


def test_pickup_close_latch_survives_pixel_jitter_and_suppresses_fallback_turn():
    planner = MotionDecisionPlanner()
    close = ball_info(
        distance_m=0.4, bottom_distance_px=100, offset_x_px=150,
        offset_x_norm=0.25, steering_angle_deg=40, pickup_x_tolerance_norm=0.08,
    )
    assert planner.plan_ball_pickup_initial_alignment(close).action.endswith("CRAB_RIGHT")
    jitter = {**close, "bottom_distance_px": 101}
    assert planner.plan_ball_pickup_initial_alignment(jitter).action.endswith("CRAB_RIGHT")
    assert planner.plan_ball_pickup_post_backward_alignment(jitter).action.endswith("CRAB_RIGHT")
    planner.clear_collected_ball_tracking()
    assert planner.plan_ball_pickup_initial_alignment(jitter).action == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_3"


@pytest.mark.parametrize("overrides", [
    {"bottom_distance_px": None}, {"bottom_distance_px": -1},
    {"bottom_distance_px": float("nan")}, {"bottom_distance_px": 101},
    {"bottom_distance_px": 200},
    {"confidence": 0.1}, {"detected": False, "raw_detected": False},
])
def test_invalid_or_distant_ball_does_not_latch_close_pickup(overrides):
    planner = MotionDecisionPlanner()
    planner.plan_ball_pickup_initial_alignment(ball_info(
        **{"distance_m": 0.4, "bottom_distance_px": 100, **overrides},
    ))
    assert not planner.pickup_close_alignment_active


@pytest.mark.parametrize("missing", [None, {"detected": False, "raw_detected": True}])
def test_ball_loss_does_not_turn_on_stale_or_unconfirmed_visible_input(missing):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(offset_x_px=-80), 0.0)
    assert planner.plan_ball_pickup_initial_alignment(missing).action == "WAIT"
    assert planner.plan_ball_pickup_fine_alignment(missing).action == "WAIT"
    result = planner.plan("AUTO", observations(ball=missing), 1.0)
    assert not result.action.startswith("BALL_APPROACH_TURN_")


@pytest.mark.parametrize("fine", [False, True])
@pytest.mark.parametrize("offset,direction", [(-80, "LEFT"), (80, "RIGHT")])
def test_pickup_loss_search_repeats_until_ball_returns(fine, offset, direction):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(distance_m=0.4, offset_x_px=offset), 0.0)
    plan = (planner.plan_ball_pickup_fine_alignment if fine
            else planner.plan_ball_pickup_initial_alignment)
    expected = (f"BALL_PICKUP_FINE_SEARCH_{direction}" if fine
                else f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{5 if direction == 'RIGHT' else 2}")
    for _ in range(3):
        assert plan({"detected": False, "raw_detected": False}).action == expected
    reacquired = ball_info(distance_m=0.4, offset_x_px=0)
    planner._update_ball_tracking(reacquired, 0.0)
    assert plan(reacquired).action != expected
    assert planner.ball_recovery_centering is False
    planner.clear_collected_ball_tracking()
    assert plan({"detected": False}).action == "WAIT"


def test_no_depth_cannot_start_ball_loss_recovery_but_keeps_acquired_side():
    planner = MotionDecisionPlanner()
    no_depth = ball_info(depth_valid=False, distance_m=None, offset_x_px=-100)
    planner._update_ball_tracking(no_depth, 0.0)
    assert planner.ball_tracking_active is False
    planner._update_ball_tracking(ball_info(offset_x_px=100), 0.0)
    planner._update_ball_tracking(no_depth, 0.0)
    assert planner.last_ball_turn_direction == "LEFT"
    assert planner.ball_tracking_active is True
    planner._update_ball_tracking(ball_info(distance_m=1.6), 0.0)
    assert planner.ball_tracking_active is False
    assert planner.last_ball_turn_direction is None


def test_ball_loss_recovery_can_be_disabled():
    planner = MotionDecisionPlanner(MotionDecisionConfig(enable_ball_lost_recovery=False))
    planner._update_ball_tracking(ball_info(offset_x_px=-100), 0.0)
    assert planner.plan_ball_pickup_fine_alignment({"detected": False}).action == "WAIT"


@pytest.mark.parametrize("method,action", [
    ("plan_lost_ball_approach_alignment", "BALL_LOST_FORWARD_2"),
    ("plan_ball_pickup_initial_alignment", "BALL_PICKUP_INITIAL_SEARCH_FORWARD"),
    ("plan_ball_pickup_fine_alignment", "BALL_PICKUP_FINE_SEARCH_FORWARD"),
])
def test_top_edge_ball_loss_searches_forward_once_per_observation(method, action):
    planner = MotionDecisionPlanner()
    visible = ball_info(
        bbox=[610, 72, 670, 132], image_height=720,
        camera_center_offset_x_px=50,
    )
    planner._update_ball_tracking(visible, 0.0)
    lost = {"detected": False, "raw_detected": False}
    plan = getattr(planner, method)
    assert plan(lost).action == action
    assert plan(lost).action == action  # Planning alone does not consume it.
    planner.mark_ball_top_loss_forward_sent()
    assert plan(lost).action != action
    planner._update_ball_tracking(visible, 0.0)
    assert plan(lost).action == action
    planner.clear_collected_ball_tracking()
    assert plan(lost).action == "WAIT"


@pytest.mark.parametrize("bbox,height", [
    ([600, 73, 660, 133], 720),
    ([600, 360, 660, 420], 720),
    ([600, 650, 660, 710], 720),
    (None, 720), ([600, 0, 660, 60], None),
    ([600, -1, 660, 60], 720), ([600, float('nan'), 660, 60], 720),
    ([600, 0, 660, 60], 0), ([600, 10, 660, 5], 720),
])
def test_non_top_or_invalid_last_geometry_keeps_side_search(bbox, height):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(
        bbox=bbox, image_height=height, camera_center_offset_x_px=80,
    ), 0.0)
    assert planner.plan_ball_pickup_initial_alignment(
        {"detected": False}
    ).action == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_5"


def test_top_loss_general_recovery_stops_first_and_requires_fresh_loss():
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", observations(ball=ball_info(
        bbox=[600, 0, 660, 60], image_height=720,
    )), 0.1)
    lost = {"detected": False, "raw_detected": False}
    assert planner.plan("AUTO", observations(ball=lost), 0.1).action == "BALL_LOST_STOP"
    assert planner.plan("AUTO", observations(ball=lost), 0.4).action == "BALL_LOST_FORWARD_2"
    for info in (None, {"detected": False, "raw_detected": True}):
        assert planner.plan("AUTO", observations(ball=info), 1.0).action != "BALL_LOST_FORWARD_2"
        assert planner.plan_ball_pickup_initial_alignment(info).action == "WAIT"
        assert planner.plan_ball_pickup_fine_alignment(info).action == "WAIT"
        assert planner.plan_lost_ball_approach_alignment(info).action != "BALL_LOST_FORWARD_2"


def test_raw_latest_ball_position_updates_and_clears_top_edge_memory():
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(camera_center_offset_x_px=50), 0.0)
    raw = ball_info(detected=False, raw_detected=True, depth_valid=False,
                    bbox=[600, 1, 660, 61], image_height=720)
    planner._update_ball_tracking(raw, 0.0)
    assert planner.plan_ball_pickup_initial_alignment({"detected": False}).action == (
        "BALL_PICKUP_INITIAL_SEARCH_FORWARD"
    )
    planner._update_ball_tracking({**raw, "bbox": None}, 0.0)
    assert planner.last_ball_top_edge_ratio is None
    assert planner.plan_ball_pickup_initial_alignment({"detected": False}).action == (
        "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_5"
    )


def test_goal_control_range_override_reaches_goal_subplanner():
    planner = MotionDecisionPlanner(MotionDecisionConfig(goal_control_range_m=1.0))
    decision = planner.plan('GOAL_APPROACH', observations(
        goal=goal_info(depth_m=1.5)), 0.1)
    assert decision.valid is False
    assert decision.reason == 'goal_outside_control_range'


@pytest.mark.parametrize('bottom_distance_px,expected_action', [
    (299, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (300, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (301, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_pickup_fine_approach_stops_at_300_pixel_boundary(bottom_distance_px, expected_action):
    decision = MotionDecisionPlanner().plan_ball_pickup_fine_alignment(
        ball_info(bottom_distance_px=bottom_distance_px, offset_x_px=0),
    )
    assert decision.action == expected_action
    assert decision.source_command['pickup_fine_align_bottom_distance_px'] == 300


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'BALL_SEARCH', 'GOAL_SEARCH'])
@pytest.mark.parametrize('offset,direction,count', [(-0.8, 'LEFT', 1), (0.8, 'RIGHT', 3)])
def test_lost_line_searches_last_seen_side(phase, offset, direction, count):
    planner = MotionDecisionPlanner()
    planner.plan(phase, observations(line=line_info(filtered_lateral_offset_norm=offset)), 0.1)
    decision = planner.plan(phase, observations(line={'detected': False}), 0.1)
    assert decision.source == 'line'
    assert decision.action == f'LINE_LOST_TURN_{direction}'
    assert decision.valid is True
    assert decision.source_command['turn_count'] == count
    assert decision.requires_ack is False


def test_line_search_uses_latest_visible_point_before_loss():
    planner = MotionDecisionPlanner()
    for x in (20, 1240):
        planner.observe_line_for_search(line_info(
            center_points_px=[[x, 700], [x, 650]], image_width=1280,
            filtered_lateral_offset_norm=-0.9,
        ))
    lost = observations(line={'detected': False})
    assert planner.plan('AUTO', lost, 0.1).action == 'LINE_LOST_TURN_RIGHT'
    planner.observe_line_for_search(line_info(
        center_points_px=[[10, 700]], image_width=1280, geometry_quality=0.1,
    ))
    assert planner.plan('AUTO', lost, 0.1).action == 'LINE_LOST_TURN_RIGHT'


def test_line_search_does_not_turn_without_history_or_on_stale_input():
    planner = MotionDecisionPlanner()
    assert not planner.plan('AUTO', observations(line={'detected': False}), 0.1).valid
    planner.observe_line_for_search(line_info(filtered_lateral_offset_norm=-0.8))
    assert not planner.plan('AUTO', observations(line=None), 0.1).valid
    assert not planner.plan('LINE_TRACK', observations(line=None), 0.1).valid


def test_line_reacquisition_resumes_normal_decisions():
    planner = MotionDecisionPlanner()
    planner.observe_line_for_search(line_info(filtered_lateral_offset_norm=-0.8))
    assert planner.plan('AUTO', observations(line={'detected': False}), 0.1).action == 'LINE_LOST_TURN_LEFT'
    assert planner.plan('AUTO', observations(line=line_info()), 0.1).action == 'STRAIGHT'


def test_ball_priority_clears_old_line_search_direction():
    planner = MotionDecisionPlanner()
    planner.observe_line_for_search(line_info(filtered_lateral_offset_norm=-0.8))
    decision = planner.plan('AUTO', observations(ball=ball_info(), line={'detected': False}), 0.1)
    assert decision.source == 'ball'
    assert planner.last_line_seen_direction is None


@pytest.mark.parametrize("direction,sign", [("LEFT", -1), ("RIGHT", 1)])
@pytest.mark.parametrize("angle,count", [
    (0.0, 0), (5.0, 0), (7.1, 0), (14.999, 0),
    (15.0, 1), (29.999, 1), (30.0, 2), (44.999, 2),
    (45.0, 3), (59.999, 3), (60.0, 4), (64.999, 4), (65.0, 4), (74.999, 4),
    (75.0, 5), (89.999, 5), (90.0, 6), (94.999, 6), (95.0, 6), (104.999, 6),
    (105.0, 7), (119.999, 7), (120.0, 8), (134.999, 8),
    (135.0, 9), (180.0, 12),
])
def test_alignment_uses_shared_turn_table_in_ball_and_line_stages(
    direction, sign, angle, count,
):
    planner = MotionDecisionPlanner()
    info = ball_info(
        steering_angle_deg=sign * angle, bearing_deg=-sign * 80,
        distance_m=0.9, offset_x_px=0, offset_x_norm=0.0,
        bottom_distance_px=300, pickup_x_tolerance_norm=0.08,
    )
    expected_count = max(0, min(count - 1, 5))
    expected_angle = {0: 0.0, 1: 30.0, 2: 45.0, 3: 60.0, 4: 75.0, 5: 90.0}[expected_count]
    if direction == "RIGHT":
        if angle < 15.0:
            expected_count, expected_angle = 0, 0.0
        elif angle < 30.0:
            expected_count, expected_angle = 2, 15.0
        elif angle < 45.0:
            expected_count, expected_angle = 3, 30.0
        elif angle < 65.0:
            expected_count, expected_angle = 5, 45.0
        elif angle < 95.0:
            expected_count, expected_angle = 7, 65.0
        else:
            expected_count, expected_angle = 9, 95.0
    for decision, prefix, aligned_action in (
        (planner.plan_ball_approach_alignment(info), "BALL_APPROACH",
         "BALL_APPROACH_ALIGNED"),
        (planner.plan_ball_pickup_initial_alignment(info), "BALL_PICKUP_CAMERA_DOWN",
         "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"),
        (planner.plan_ball_pickup_post_backward_alignment(info),
         "BALL_PICKUP_POST_BACKWARD", "BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE"),
        (planner.plan("POST_BALL_LINE_ALIGN", observations(
            line=line_info(filtered_heading_error_deg=sign * angle)), 0.1),
         "POST_BALL_LINE", "POST_BALL_LINE_ALIGNED"),
    ):
        assert decision.valid
        if expected_count == 0:
            assert decision.action == aligned_action
        else:
            assert decision.action == f"{prefix}_TURN_{direction}_{expected_count}"
            assert decision.source_command["turn_count"] == expected_count
            assert decision.source_command["turn_repeat_deg"] is None
            assert decision.source_command["turn_angle_deg"] == expected_angle
            assert decision.source_command["turn_angle_deg"] <= angle
