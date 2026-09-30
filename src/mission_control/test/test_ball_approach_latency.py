"""Exercise short Ball motion boundaries without a ROS graph or hardware."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_approach_observation import BallApproachObservation
from mission_control.motion_decision_node import MotionDecisionNode
from step.ball_navigation_planner import BallNavigationConfig
from test_mission_phase_flow import MissionFlowHarness, line_info, release_general


def sample(stamp, *, detected=True, distance=1.2):
    info = {
        "detected": detected, "raw_detected": True, "confidence": 0.9,
        "rgb_stamp_ns": round(stamp * 1e9), "depth_valid": True,
        "depth_age_sec": 0.02, "distance_m": distance, "depth_m": distance,
        "ground_distance_m": distance, "steering_angle_deg": 0.0,
        "bearing_deg": 0.0, "offset_x_norm": 0.0,
        "bbox": [600, 240, 650, 290], "image_width": 1280, "image_height": 720,
        "center_x": 625, "center_y": 265, "camera_center_offset_x_px": -15,
        "bottom_distance_px": 454, "head_down_requested": False,
        "pickup_now": False, "pickup_ready": False,
    }
    if not detected:
        info["approach_candidate"] = {
            key: info[key] for key in (
                "depth_valid", "depth_age_sec", "distance_m", "depth_m",
                "ground_distance_m", "steering_angle_deg", "bearing_deg",
                "offset_x_norm", "bottom_distance_px", "head_down_requested",
            )
        }
        info.update(depth_valid=False, distance_m=None, depth_m=None)
    return info


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    monkeypatch.setattr(
        MotionDecisionNode, "_current_ros_time_ns", staticmethod(lambda _: round(now[0] * 1e9)),
    )
    return now


def receive(harness, clock, now, info):
    clock[0] = now
    MotionDecisionNode._info_callback(harness, "ball", dispatch_on_ball=True)(
        String(data=json.dumps(info)),
    )


def start_approach(harness, clock):
    receive(harness, clock, 10.0, sample(9.98))
    command = harness.publisher.messages[-1]
    assert command["source"] == "ball" and command["action"] == "STRAIGHT"
    receive(harness, clock, 10.15, sample(10.13))
    assert len(harness.publisher.messages) == 1
    clock[0] = 10.2
    release_general(harness, command)
    return command


def test_distant_raw_ball_keeps_line_moving_but_disables_prequeue(clock):
    harness = MissionFlowHarness()
    raw = sample(9.98, detected=False)
    receive(harness, clock, 10.0, raw)
    assert harness.ball_confirmation_pending_latched is False
    assert harness.planner.ball_lock_active is False
    decision = harness._select_mission_decision({"ball": raw, "line": line_info()}, 0.1)
    assert decision.source == "line" and decision.valid
    assert not MotionDecisionNode._line_only_prequeue_allowed(harness, {"ball": raw})
    assert harness.publisher.messages == []


@pytest.mark.parametrize("overrides", [
    {"distance_m": 0.9}, {"distance_m": 1.0}, {"distance_m": None},
    {"depth_valid": False}, {"depth_age_sec": 0.3},
])
def test_near_or_unknown_candidate_retains_boundary_hold(clock, overrides):
    harness = MissionFlowHarness()
    raw = sample(9.98, detected=False)
    raw["approach_candidate"].update(overrides)
    receive(harness, clock, 10.0, raw)
    decision = harness._select_mission_decision({"ball": raw, "line": line_info()}, 0.1)
    assert decision.action == "WAIT" and not decision.valid
    assert decision.reason == "ball_confirmation_pending_hold"


def test_confirmed_post_motion_frame_dispatches_without_timer_or_extra_pause(clock):
    harness = MissionFlowHarness()
    start_approach(harness, clock)
    # Delayed pre-completion RGB cannot unlock the next motion.
    receive(harness, clock, 10.23, sample(10.19))
    assert harness.latest_info["ball"] is None
    assert len(harness.publisher.messages) == 1
    receive(harness, clock, 10.25, sample(10.23))
    assert len(harness.publisher.messages) == 2
    assert harness.publisher.messages[-1]["action"] == "STRAIGHT"
    assert harness.ball_post_motion_dwell_until is None
    # A timer invocation cannot publish a duplicate while this motion runs.
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == 2


def test_three_consistent_post_motion_candidates_resume_only_existing_approach(clock):
    harness = MissionFlowHarness()
    start_approach(harness, clock)
    for now in (10.25, 10.30):
        receive(harness, clock, now, sample(now - 0.02, detected=False))
        assert len(harness.publisher.messages) == 1
    receive(harness, clock, 10.35, sample(10.33, detected=False))
    command = harness.publisher.messages[-1]
    assert len(harness.publisher.messages) == 2
    assert command["action"] == "STRAIGHT"
    assert command["source_command"]["approach_reobserved"] is True
    assert harness.latest_info["ball"]["detected"] is False
    assert harness.ball_pickup_entry_pending is False


@pytest.mark.parametrize("stamp_delta", [-0.6, 0.1])
def test_old_or_future_camera_frames_cannot_start_approach(clock, stamp_delta):
    harness = MissionFlowHarness()
    receive(harness, clock, 10.0, sample(10.0 + stamp_delta))
    assert not harness.publisher.messages
    assert harness.latest_info["ball"] is None


def test_stationary_turn_keeps_entire_one_second_dwell_and_fresh_frame_gate(clock):
    harness = MissionFlowHarness()
    harness.BALL_POST_MOTION_DWELL_SEC = 1.0
    harness.planner.ball_lock_active = True
    harness.general_motion_gate.on_new_vision_input()
    harness.general_motion_gate.on_command_published("BALL_APPROACH_TURN_RIGHT_5", 7)
    harness.active_general_source = "ball"
    harness.send_status("BALL_APPROACH_TURN_RIGHT_5", 7, "RUNNING")
    harness.send_status("BALL_APPROACH_TURN_RIGHT_5", 7, "SUCCEEDED")
    assert harness.ball_post_motion_dwell_until == 11.0
    receive(harness, clock, 10.99, sample(10.97))
    MotionDecisionNode._publish_decision(harness)
    assert harness.publisher.messages == []
    clock[0] = 11.0
    MotionDecisionNode._publish_decision(harness)
    assert harness.publisher.messages == []
    receive(harness, clock, 11.02, sample(10.99))
    assert harness.publisher.messages == []
    receive(harness, clock, 11.04, sample(11.02))
    assert harness.publisher.messages[-1]["action"] == "STRAIGHT"


def test_raw_ball_never_acquires_control_without_strong_initial_confirmation(clock):
    harness = MissionFlowHarness()
    for index in range(8):
        now = 10.0 + index * 0.05
        receive(harness, clock, now, sample(now - 0.02, detected=False))
    assert harness.planner.ball_lock_active is False
    assert harness.publisher.messages == []


@pytest.mark.parametrize("blocked", ["safety", "executor", "pickup", "invalid_geometry"])
def test_frame_dispatch_preserves_existing_execution_guards(clock, blocked):
    harness = MissionFlowHarness()
    start_approach(harness, clock)
    if blocked == "safety":
        from types import SimpleNamespace
        harness.safety_interlock = SimpleNamespace(latched=True)
    elif blocked == "executor":
        harness.executor_auto_ready = False
    elif blocked == "pickup":
        harness.phase_manager.start_special_action("PICKUP_NOW", 99)
    for now in (10.25, 10.30, 10.35):
        raw = sample(now - 0.02, detected=False)
        if blocked == "invalid_geometry":
            raw["approach_candidate"]["ground_projection_enabled"] = False
        receive(harness, clock, now, raw)
    assert not any(command["valid"] for command in harness.publisher.messages[1:])


def test_reobservation_does_not_reuse_geometry_after_camera_pause(clock):
    harness = MissionFlowHarness()
    start_approach(harness, clock)
    for now in (10.8, 10.85, 10.9):
        receive(harness, clock, now, sample(now - 0.02, detected=False))
    MotionDecisionNode._publish_decision(harness)
    assert harness.publisher.messages[-1]["action"] == "WAIT"
    assert not any(command["valid"] for command in harness.publisher.messages[1:])


def test_raw_observation_near_pickup_never_uses_fast_path(clock):
    harness = MissionFlowHarness()
    start_approach(harness, clock)
    for now in (10.25, 10.30, 10.35):
        receive(harness, clock, now, sample(now - 0.02, detected=False, distance=0.5))
    MotionDecisionNode._publish_decision(harness)
    assert harness.publisher.messages[-1]["action"] == "WAIT"
    assert harness.ball_pickup_entry_pending is False
    assert not any(command["action"] == "PICKUP_NOW" for command in harness.publisher.messages)


@pytest.mark.parametrize("change", [
    {"bbox": [950, 240, 1000, 290]}, {"bbox": [600, 240, 620, 260]},
    {"image_width": 640}, {"confidence": 0.3},
    {"distance_m": 0.5}, {"distance_m": 1.5},
    {"depth_age_sec": 0.25}, {"head_down_requested": True},
])
def test_fast_reobservation_rejects_changed_or_unsafe_target(change):
    observer = BallApproachObservation()
    config = BallNavigationConfig()
    observer.observe(sample(10.0), 10.02, config)
    for index in range(3):
        stamp = 10.05 + index * 0.05
        raw = sample(stamp, detected=False)
        for key, value in change.items():
            if key in raw["approach_candidate"]:
                raw["approach_candidate"][key] = value
            else:
                raw[key] = value
        observer.observe(raw, stamp + 0.02, config)
        assert observer.observation is None


def test_duplicate_frames_and_expired_anchor_do_not_fast_confirm():
    observer = BallApproachObservation()
    config = BallNavigationConfig()
    observer.observe(sample(10.0), 10.02, config)
    for _ in range(4):
        observer.observe(sample(10.05, detected=False), 10.07, config)
    assert observer.hits == 1 and observer.observation is None
    for stamp in (10.6, 10.65, 10.7):
        observer.observe(sample(stamp, detected=False), stamp + 0.02, config)
    assert observer.observation is None


def test_weak_reobservation_cannot_extend_its_own_confirmation_forever():
    observer = BallApproachObservation()
    config = BallNavigationConfig()
    observer.observe(sample(10.0), 10.02, config)
    for stamp in (10.05, 10.10, 10.15):
        observer.observe(sample(stamp, detected=False), stamp + 0.02, config)
    assert observer.is_current(10.17)
    assert not observer.is_current(10.4)
    observer.observe(sample(10.6, detected=False), 10.62, config)
    assert observer.observation is None
