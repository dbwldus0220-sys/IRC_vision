"""Reject wrong-posture, transitional and pre-transition hurdle geometry."""

import pytest

from step.hurdle_camera_pose import HurdleCameraPose
from step.hurdle_navigation_planner import HurdleNavigationPlanner


def heartbeat(sequence=1, **updates):
    return {
        "sequence": sequence, "auto_ready": True,
        "hurdle_head_override_active": False,
        "ball_head_override_active": False, "goal_head_override_active": False,
        **updates,
    }


def observation(seconds=10.1, **updates):
    stamp_ns = round(seconds * 1e9)
    return {
        "stamp": {"sec": stamp_ns // 1_000_000_000, "nanosec": stamp_ns % 1_000_000_000},
        "detected": True, "confirmation_confirmed": True, "confidence": .95,
        "depth_valid": True, "depth_m": .6, "depth_age_sec": .03,
        "bottom_distance_px": 200,
        "hurdle_angle_deg": -80,  # Deliberately conflicting legacy geometry.
        "hurdle_angle_45_deg": 30, "hurdle_angle_0_deg": -30,
        "hurdle_angle_fit_point_count": 5,
        "hurdle_angle_sample_source": "yellow_bar_centerline",
        **updates,
    }


def plan(tracker, info, now):
    return HurdleNavigationPlanner().plan(tracker.annotate(info, now), positioning=True)


def test_selects_both_postures_and_waits_for_new_capture_after_interpolation():
    tracker = HurdleCameraPose()
    tracker.heartbeat(heartbeat(), 10., 10_000_000_000)
    up = plan(tracker, observation(), 10.1)
    assert up.action == "RECOVER_LEFT_TURN_LEFT_4" and up.turn_count is None
    assert up.alignment_reference == "hurdle_angle_45"

    assert plan(tracker, observation(head_down_requested=True), 10.1).action == "WAIT"
    tracker.heartbeat(heartbeat(2, hurdle_head_override_active=True,
                                hurdle_head_override_current_deg=-50), 10.2, 10_200_000_000)
    assert plan(tracker, observation(10.3), 10.3).action == "WAIT"
    tracker.heartbeat(heartbeat(3, hurdle_head_override_active=True,
                                hurdle_head_override_current_deg=-64), 10.6, 10_600_000_000)
    assert plan(tracker, observation(10.59), 10.7).action == "WAIT"
    down = plan(tracker, observation(10.7, depth_m=.57, head_down_requested=True), 10.7)
    assert down.action == "ALIGN_RIGHT" and down.turn_count == 3
    assert down.hurdle_angle_deg == -30
    assert down.alignment_reference == "hurdle_angle_0"


@pytest.mark.parametrize("updates", [
    {"auto_ready": False}, {"ball_head_override_active": True},
    {"goal_head_override_active": True}, {"hurdle_head_override_active": None},
    {"hurdle_head_override_active": True},
    {"hurdle_head_override_active": True, "hurdle_head_override_current_deg": -60},
    {"hurdle_head_override_active": True, "hurdle_head_override_current_deg": -63},
    {"hurdle_head_override_active": True, "hurdle_head_override_current_deg": float("nan")},
])
def test_unknown_or_other_camera_posture_cannot_select_either_hypothesis(updates):
    tracker = HurdleCameraPose()
    tracker.heartbeat(heartbeat(**updates), 10., 10_000_000_000)
    assert plan(tracker, observation(), 10.1).action == "WAIT"


def test_missing_stale_duplicate_heartbeat_and_restart_require_fresh_camera_evidence():
    tracker = HurdleCameraPose()
    assert plan(tracker, observation(), 10.1).action == "WAIT"
    tracker.heartbeat(heartbeat(5), 10., 10_000_000_000)
    assert plan(tracker, observation(10.), 10.1).action == "WAIT"
    assert plan(tracker, observation(stamp=None), 10.1).action == "WAIT"
    tracker.heartbeat(heartbeat(5), 11., 11_000_000_000)
    assert plan(tracker, observation(11.1), 11.1).action == "WAIT"
    tracker.heartbeat(heartbeat(0), 11.2, 11_200_000_000)
    assert plan(tracker, observation(11.1), 11.3).action == "WAIT"
    assert plan(tracker, observation(11.3), 11.3).action == "RECOVER_LEFT_TURN_LEFT_4"


@pytest.mark.parametrize("pose", ["45", "0"])
@pytest.mark.parametrize("bad", [None, True, float("nan"), float("inf"), 90, -90])
def test_invalid_selected_angle_never_falls_back_to_other_pose_or_legacy(pose, bad):
    info = observation(**{f"hurdle_angle_{pose}_deg": bad},
                       hurdle_camera_pose=pose, hurdle_camera_ready=True)
    assert HurdleNavigationPlanner().plan(info, positioning=True).action == "WAIT"


def test_close_depth_cannot_bypass_camera_transition():
    command = plan(HurdleCameraPose(), observation(depth_m=.45), 10.1)
    assert command.action == "WAIT" and not command.fine_sequence_requested


def test_walking_heartbeats_keep_same_posture_images_usable_without_reobservation():
    tracker = HurdleCameraPose()
    tracker.heartbeat(heartbeat(), 10., 10_000_000_000)
    info = observation(10.2, hurdle_angle_45_deg=0.)
    for sequence, now in [(2, 10.25), (3, 10.3), (4, 10.4)]:
        tracker.heartbeat(heartbeat(sequence, active=True), now, round(now * 1e9))
        assert tracker.ready_stamp_ns == 10_000_000_000
        command = plan(tracker, info, now)
        assert command.action == 'STRAIGHT_3'
        assert command.alignment_reference == 'hurdle_angle_45'
