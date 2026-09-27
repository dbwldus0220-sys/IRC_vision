"""Near-hurdle entry and fresh-frame checkpoints using the real node callbacks."""

import json
import math

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_mission_phase_flow import MissionFlowHarness, line_info, release_general


def hurdle(depth=0.55, **updates):
    info = {
        "detected": True, "confirmation_confirmed": True, "confidence": 0.9,
        "depth_valid": True, "depth_m": depth, "hurdle_angle_deg": 0.0,
        "bottom_distance_px": 200, "camera_center_offset_x_px": 0, "go_now": False,
    }
    info.update(updates)
    return info


class HurdleHarness(MissionFlowHarness):
    _fresh_observations = MotionDecisionNode._fresh_observations

    def receive_line(self):
        MotionDecisionNode._info_callback(self, "line")(
            String(data=json.dumps(line_info()))
        )

    def receive(self, info):
        MotionDecisionNode._info_callback(self, "hurdle")(
            String(data=json.dumps(info))
        )

    def publish(self):
        before = len(self.publisher.messages)
        MotionDecisionNode._publish_decision(self)
        return self.publisher.messages[before:]


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    return now


def enter_positioning(node, clock):
    node.receive(hurdle())
    assert node.hurdle_positioning_entry_pending
    assert node.publish() == []
    clock[0] += 3.0
    assert node.publish() == []
    node.receive(hurdle())
    command = node.publish()[-1]
    assert command["phase"] == "HURDLE_POSITIONING"
    return command


@pytest.mark.parametrize("depth,accepted", [
    (0.551, False), (0.55, True), (0.549, True), (0.0, False),
    (-0.1, False), (None, False), (True, False), (float("nan"), False),
    (float("inf"), False),
])
def test_hurdle_positioning_depth_boundary(clock, depth, accepted):
    node = HurdleHarness()
    node.receive(hurdle(depth))
    assert getattr(node, "hurdle_positioning_entry_pending", False) is accepted


@pytest.mark.parametrize("update", [
    {"detected": False}, {"detected": 1}, {"depth_valid": False},
    {"confirmation_confirmed": False}, {"confirmation_confirmed": 1},
    {"confidence": 0.59}, {"confidence": None},
])
def test_hurdle_positioning_requires_confirmed_valid_depth(clock, update):
    node = HurdleHarness()
    node.receive(hurdle(**update))
    assert not getattr(node, "hurdle_positioning_entry_pending", False)


def test_crossing_during_motion_waits_for_completion_dwell_and_fresh_frame(clock):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(0.7))
    approach = node.publish()[-1]
    assert approach["source"] == "hurdle"
    assert approach["action"] == "STRAIGHT"
    node.receive(hurdle())
    assert node.hurdle_positioning_entry_pending
    assert node.mission_phase == "AUTO"
    assert node.publish() == []
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None
    clock[0] = 11.0
    release_general(node, approach)
    assert node.hurdle_post_motion_dwell_until == 14.0
    clock[0] = 13.99
    node.receive(hurdle(0.6))
    assert node.publish() == []
    clock[0] = 14.0
    assert node.publish() == []
    assert node.latest_info["hurdle"] is None
    # A Line frame cannot authorize motion using old Hurdle geometry.
    node.general_motion_gate.on_new_vision_input()
    waiting = node.publish()[-1]
    assert waiting["phase"] == "HURDLE_POSITIONING"
    assert waiting["action"] == "WAIT"
    node.receive(hurdle(0.6))
    fine = node.publish()[-1]
    assert fine["action"] == "STRAIGHT"
    assert fine["source_command"]["hurdle_positioning_active"]
    assert fine["source_command"]["approach_motion"] == "STRAIGHT"
    assert fine["source_command"]["approach_level"] is None
    assert node.phase_manager.hurdles_executed == 0


def test_positioning_rechecks_after_each_motion_and_preserves_mode_on_loss(clock):
    node = HurdleHarness()
    fine = enter_positioning(node, clock)
    release_general(node, fine)
    assert node.hurdle_post_motion_dwell_until == clock[0] + 3.0
    node.receive(hurdle(0.3))
    assert node.publish() == []
    clock[0] += 3.0
    assert node.publish() == []
    node.receive({"detected": False})
    waiting = node.publish()[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["phase"] == "HURDLE_POSITIONING"
    assert not waiting["valid"]
    clock[0] += 0.01
    node.receive(hurdle(0.4, hurdle_angle_deg=-45.0, camera_center_offset_x_px=200))
    turn = node.publish()[-1]
    assert turn["action"] == "ALIGN_RIGHT"
    assert turn["source_command"]["turn_count"] == 5


@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED", "CANCELLED"])
def test_positioning_go_terminal_lifecycle(clock, status):
    node = HurdleHarness()
    fine = enter_positioning(node, clock)
    release_general(node, fine)
    clock[0] += 3.0
    node.publish()
    node.receive(hurdle(0.15, go_now=True))
    go = node.publish()[-1]
    assert go["action"] == "GO"
    node.send_status("GO", go["command_id"], "RUNNING", motion_id="hurdle")
    node.send_status("GO", go["command_id"], status, motion_id="hurdle")
    assert node.mission_phase == (
        "LINE_TRACK" if status == "SUCCEEDED" else "HURDLE_POSITIONING"
    )
    assert node.phase_manager.hurdles_completed == int(status == "SUCCEEDED")
    assert not getattr(node, "hurdle_positioning_entry_pending", False)


@pytest.mark.parametrize("status", ["FAILED", "CANCELLED", "REJECTED"])
def test_failed_approach_discards_positioning_reservation(clock, status):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(0.7))
    approach = node.publish()[-1]
    node.receive(hurdle())
    node.send_status(approach["action"], approach["command_id"], "RUNNING")
    node.send_status(approach["action"], approach["command_id"], status)
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None
    assert node.mission_phase != "HURDLE_POSITIONING"


def test_near_crossing_waits_for_already_queued_line_motion(clock):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(1.1))
    approach = node.publish()[-1]
    node.queued_general_action = "STRAIGHT"
    node.queued_general_command_id = 99
    node.queued_general_source = "line"
    node.receive(hurdle())
    assert node.hurdle_positioning_entry_pending
    release_general(node, approach)
    assert node.publish() == []
    node.send_status("STRAIGHT", 99, "RUNNING")
    clock[0] += 4.0
    assert node.publish() == []
    node.send_status("STRAIGHT", 99, "SUCCEEDED")
    assert node.hurdle_post_motion_dwell_until == clock[0] + 3.0
    clock[0] += 3.0
    assert node.publish() == []
    node.receive(hurdle())
    fine = node.publish()[-1]
    assert fine["phase"] == "HURDLE_POSITIONING"
    assert fine["action"] == "STRAIGHT_0"


@pytest.mark.parametrize("blocked", ["pickup", "ball", "completed", "budget"])
def test_positioning_cannot_interrupt_other_motion_or_completed_mission(clock, blocked):
    node = HurdleHarness()
    if blocked == "pickup":
        node.phase_manager.start_special_action("PICKUP_NOW", 99)
    elif blocked == "ball":
        node.general_motion_gate.on_command_published("STRAIGHT", 99)
        node.active_general_source = "ball"
    elif blocked == "completed":
        node.phase_manager.hurdles_completed = node.phase_manager.required_hurdles
    else:
        node.phase_manager.hurdles_executed = node.phase_manager.required_hurdles
    node.receive(hurdle())
    assert not getattr(node, "hurdle_positioning_entry_pending", False)


def test_manual_phase_override_discards_positioning_reservation(clock):
    node = HurdleHarness()
    node.receive(hurdle())
    node.send_phase("LINE_TRACK")
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None


def test_far_approach_and_existing_near_go_conditions_unchanged():
    planner = MotionDecisionPlanner()
    far = planner.plan("HURDLE_APPROACH", {
        "hurdle": hurdle(0.7), "line": line_info(),
    }, 0.1)
    assert far.action == "STRAIGHT"
    assert far.source == "hurdle"
    assert not far.source_command.get("hurdle_positioning_active", False)
    aligned = planner.plan("HURDLE_POSITIONING", {"hurdle": hurdle(0.4)}, 0.1)
    assert aligned.action == "STRAIGHT_0"
    not_confirmed = planner.plan("HURDLE_POSITIONING", {"hurdle": hurdle(0.15)}, 0.1)
    assert not_confirmed.action == "WAIT_GO_CONFIRMATION"
    ready = planner.plan("HURDLE_POSITIONING", {"hurdle": hurdle(0.15, go_now=True)}, 0.1)
    assert ready.action == "GO"


@pytest.mark.parametrize("angle", [-34.0, 34.0])
def test_recognition_approach_has_no_fine_sequence_dwell(clock, angle):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(0.9))
    approach = node.publish()[-1]
    assert approach["source"] == "hurdle"
    assert approach["action"] == "STRAIGHT"
    node.receive(hurdle(0.7, hurdle_angle_deg=angle))
    assert node.publish() == []
    release_general(node, approach)
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None
    clock[0] += 0.01
    node.receive_line()
    node.receive(hurdle(0.7, hurdle_angle_deg=angle))
    forward = node.publish()[-1]
    assert forward["source"] == "hurdle"
    assert forward["action"] == "STRAIGHT"


def test_recognition_approach_works_without_line(clock):
    node = HurdleHarness()
    node.receive(hurdle(0.7, hurdle_angle_deg=-34.0))
    waiting = node.publish()[-1]
    assert waiting["valid"] and waiting["action"] == "STRAIGHT"
    assert waiting["source"] == "hurdle"
    assert not node.planner.hurdle_lock_active
    assert not getattr(node, "hurdle_positioning_entry_pending", False)


def test_idle_near_entry_does_not_add_second_turn_pause(clock):
    node = HurdleHarness()
    node.receive(hurdle(0.5, hurdle_angle_deg=-45.0, camera_center_offset_x_px=200))
    assert node.publish() == []
    clock[0] += 3.0
    assert node.publish() == []
    node.receive(hurdle(0.5, hurdle_angle_deg=-45.0, camera_center_offset_x_px=200))
    turn = node.publish()[-1]
    assert turn["phase"] == "HURDLE_POSITIONING"
    assert turn["action"] == "ALIGN_RIGHT"


@pytest.mark.parametrize("depth,allowed", [(1.001, True), (1.0, False), (0.7, False), (0.551, False), (0.55, False)])
def test_recognition_approach_blocks_new_line_prequeue(clock, depth, allowed):
    node = HurdleHarness()
    assert MotionDecisionNode._line_only_prequeue_allowed(
        node, {"hurdle": hurdle(depth)},
    ) is allowed


@pytest.mark.parametrize("depth", [1.0, 0.7, 0.550001])
@pytest.mark.parametrize("heading", [-34.0, 0.0, 34.0])
def test_recognition_approach_ignores_line_heading(depth, heading):
    line = line_info(heading=heading)
    planner = MotionDecisionPlanner()
    actual = planner.plan("AUTO", {
        "line": line, "hurdle": hurdle(depth, hurdle_angle_deg=-34.0),
    }, 0.1)
    assert actual.source == "hurdle" and actual.action == "STRAIGHT"
    assert actual.source_command["hurdle_stage"] == "RECOGNITION_APPROACH"
    assert not planner.hurdle_lock_active


@pytest.mark.parametrize("depth", [0.7, 0.4])
@pytest.mark.parametrize("bottom,action", [(101, "ALIGN_RIGHT"), (100, "STRAIGHT_0")])
def test_rotation_cutoff_is_image_distance_not_550mm(depth, bottom, action):
    planner = MotionDecisionPlanner()
    decision = planner.plan("HURDLE_POSITIONING", {
        "hurdle": hurdle(depth, hurdle_angle_deg=-45.0, camera_center_offset_x_px=200, bottom_distance_px=bottom),
    }, 0.1)
    expected = "STRAIGHT" if bottom == 100 and depth > 0.55 else action
    assert decision.action == expected
    if bottom == 100:
        later = planner.plan("HURDLE_POSITIONING", {
            "hurdle": hurdle(depth, hurdle_angle_deg=-45.0, camera_center_offset_x_px=200, bottom_distance_px=200),
        }, 0.1)
        assert later.action == expected


@pytest.mark.parametrize("angle", [-60.0, -15.01, -14.999, 0.0, 14.999, 15.01, 60.0])
@pytest.mark.parametrize("phase", ["AUTO", "HURDLE_POSITIONING"])
def test_turn_uses_center_angle_and_15_degree_minimum(angle, phase):
    decision = MotionDecisionPlanner().plan(phase, {
        "hurdle": hurdle(0.5, hurdle_angle_deg=0, bottom_distance_px=101, camera_center_offset_x_px=101 * math.tan(math.radians(angle))),
    }, 0.1)
    expected = (
        "ALIGN_RIGHT" if angle >= 15.0 else
        "ALIGN_LEFT" if angle <= -15.0 else "STRAIGHT_0"
    )
    assert decision.valid and decision.action == expected
    assert decision.source_command["center_steering_deg"] == pytest.approx(angle, abs=0.001)
    if abs(angle) < 15.0:
        assert "turn_count" not in decision.source_command


@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED", "CANCELLED", "REJECTED"])
def test_near_turn_pauses_three_seconds_before_and_after(clock, status):
    node = HurdleHarness()
    node.receive(hurdle(0.5, hurdle_angle_deg=45.0, camera_center_offset_x_px=-200))
    assert node.publish() == []
    clock[0] = 12.999
    node.receive(hurdle(0.5, hurdle_angle_deg=45.0, camera_center_offset_x_px=-200))
    assert node.publish() == []
    clock[0] = 13.0
    assert node.publish() == []
    node.receive(hurdle(0.5, hurdle_angle_deg=45.0, camera_center_offset_x_px=-200))
    turn = node.publish()[-1]
    assert turn["action"] == "ALIGN_LEFT"
    node.send_status(turn["action"], turn["command_id"], "RUNNING")
    clock[0] = 14.0
    node.send_status(turn["action"], turn["command_id"], status)
    assert node.hurdle_post_motion_dwell_until == 17.0
    clock[0] = 16.999
    node.receive(hurdle(0.4, hurdle_angle_deg=44.999))
    assert node.publish() == []
    clock[0] = 17.0
    assert node.publish() == []
    assert node.latest_info["hurdle"] is None
    node.receive(hurdle(0.4, hurdle_angle_deg=44.999))
    forward = node.publish()[-1]
    assert forward["action"] == "STRAIGHT_0"


def test_turn_without_recorded_stationary_time_still_waits_three_seconds(clock):
    node = HurdleHarness(phase="HURDLE_POSITIONING")
    node.receive(hurdle(0.5, hurdle_angle_deg=45.0, camera_center_offset_x_px=-200))
    assert node.publish() == []
    clock[0] = 12.999
    node.receive(hurdle(0.5, hurdle_angle_deg=45.0, camera_center_offset_x_px=-200))
    assert node.publish() == []
    clock[0] = 13.0
    node.receive(hurdle(0.5, hurdle_angle_deg=45.0, camera_center_offset_x_px=-200))
    assert node.publish()[-1]["action"] == "ALIGN_LEFT"


@pytest.mark.parametrize("depth,depth_valid", [
    (0.551, True), (0.66, True), (None, False), (0.0, True),
    (-0.1, True), (float("nan"), True), (0.5, False),
])
def test_bottom_trigger_cannot_reserve_hurdle_without_close_valid_depth(clock, depth, depth_valid):
    node = HurdleHarness()
    node.receive_line()
    observation = hurdle(
        depth, depth_valid=depth_valid, bottom_distance_px=120, head_down_requested=True,
    )
    node.receive(observation)
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    assert not node.planner._hurdle_positioning_ready(observation)
    command = node.publish()[-1]
    assert command["source"] == ("hurdle" if depth_valid and depth in (0.551, 0.66) else "line")
    assert not node.planner.hurdle_lock_active


def test_depth_entry_waits_for_line_then_stays_hurdle_on_lost_detection(clock):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(0.66))
    approach = node.publish()[-1]
    assert approach["source"] == "hurdle"
    node.receive(hurdle(0.55, bottom_distance_px=200, head_down_requested=False))
    assert node.hurdle_positioning_entry_pending
    assert node.publish() == []
    release_general(node, approach)
    clock[0] += 3.0
    assert node.publish() == []
    node.receive({"detected": False})
    waiting = node.publish()[-1]
    assert waiting["phase"] == "HURDLE_POSITIONING"
    assert waiting["source"] == "hurdle" and waiting["action"] == "WAIT"
    node.receive(hurdle(0.66, bottom_distance_px=120, head_down_requested=True))
    fine = node.publish()[-1]
    assert fine["source"] == "hurdle" and fine["action"] == "STRAIGHT"


def test_missing_depth_after_depth_entry_waits_in_hurdle_mode(clock):
    node = HurdleHarness()
    node.receive(hurdle(0.55, bottom_distance_px=200, head_down_requested=False))
    assert node.hurdle_positioning_entry_pending
    clock[0] += 3.0
    assert node.publish() == []
    node.receive(hurdle(None, depth_valid=False, bottom_distance_px=120, head_down_requested=True))
    waiting = node.publish()[-1]
    assert waiting["phase"] == "HURDLE_POSITIONING"
    assert waiting["action"] == "WAIT"
    assert waiting["reason"] == "missing_valid_hurdle_depth"


@pytest.mark.parametrize("depth", [0.550001, 0.7, 1.0])
def test_recognition_depth_never_enters_fine_sequence(clock, depth):
    node = HurdleHarness()
    node.receive(hurdle(depth, bottom_distance_px=120, head_down_requested=True))
    command = node.publish()[-1]
    assert command["source"] == "hurdle" and command["action"] == "STRAIGHT"
    assert command["phase"] == "AUTO"
    assert command["source_command"]["hurdle_stage"] == "RECOGNITION_APPROACH"
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    assert not node.planner.hurdle_lock_active
    release_general(node, command)
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None


def test_unconfirmed_or_out_of_range_hurdle_does_not_take_line_control():
    for info in [hurdle(1.001), hurdle(0.7, confirmation_confirmed=False)]:
        command = MotionDecisionPlanner().plan("AUTO", {"hurdle": info, "line": line_info()}, 0.1)
        assert command.source == "line"


@pytest.mark.parametrize("depth", [0.7, 0.5])
@pytest.mark.parametrize("center_dx,action", [(-200, "ALIGN_LEFT"), (200, "ALIGN_RIGHT")])
def test_both_approach_stages_steer_toward_hurdle_center(depth, center_dx, action):
    decision = MotionDecisionPlanner().plan("AUTO", {
        "hurdle": hurdle(depth, camera_center_offset_x_px=center_dx, hurdle_angle_deg=None),
        "line": line_info(heading=-34 if center_dx > 0 else 34),
    }, 0.1)
    assert decision.source == "hurdle" and decision.valid
    assert decision.action == action
    assert decision.source_command["hurdle_stage"] == (
        "RECOGNITION_APPROACH" if depth > 0.55 else "FINE_APPROACH"
    )
