"""Near-hurdle entry and fresh-frame checkpoints using the real node callbacks."""

import json
import math

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_mission_phase_flow import MissionFlowHarness, line_info as base_line_info, release_general


def line_info(heading=0.0, offset=0.0, x=710):
    return {
        **base_line_info(heading, offset),
        "robot_center_x_px": 710.0,
        "center_points_px": [[x, 650], [x, 580], [x, 250]],
    }


def hurdle(depth=0.20, **updates):
    info = {
        "detected": True, "confirmation_confirmed": True, "confidence": 0.9,
        "depth_valid": True, "depth_m": depth, "hurdle_angle_deg": 0.0,
        "bottom_distance_px": 200, "camera_center_offset_x_px": 0, "go_now": False,
        "bbox": [400, 300, 1000, 500], "image_width": 1280, "image_height": 720,
    }
    info.update(updates)
    return info


class HurdleHarness(MissionFlowHarness):
    _fresh_observations = MotionDecisionNode._fresh_observations

    def receive_line(self, x=710):
        MotionDecisionNode._info_callback(self, "line")(
            String(data=json.dumps(line_info(x=x)))
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
    command = node.publish()[-1]
    assert command["phase"] == "HURDLE_POSITIONING"
    assert command["action"] == "GO"
    assert command["source_command"]["fine_sequence_requested"]
    return command


@pytest.mark.parametrize("depth,accepted", [
    (0.700001, False), (0.7, True), (0.699, True),
    (0.551, True), (0.55, True), (0.0, False),
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


def test_crossing_during_motion_waits_for_completion_and_fresh_frame(clock):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(0.8))
    approach = node.publish()[-1]
    assert approach["action"] == "STRAIGHT"
    node.receive(hurdle())
    assert node.hurdle_positioning_entry_pending
    assert node.publish() == []
    release_general(node, approach)
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None
    node.receive(hurdle())
    go = node.publish()[-1]
    assert go["phase"] == "HURDLE_POSITIONING" and go["action"] == "GO"
    assert go["source_command"]["fine_sequence_requested"]
    assert node.phase_manager.hurdles_completed == 0


def test_positioning_sequence_keeps_mission_lock_on_detection_loss(clock):
    node = HurdleHarness()
    go = enter_positioning(node, clock)
    node.send_status("GO", go["command_id"], "RUNNING", motion_id="pickup_fine_forward_0")
    node.receive({"detected": False})
    waiting = node.publish()[-1]
    assert waiting["action"] == "WAIT" and not waiting["valid"]
    assert waiting["active_special_action"] == "GO"
    assert node.phase_manager.hurdles_completed == 0
    node.send_status("GO", go["command_id"], "SUCCEEDED", motion_id="hurdle")
    assert node.mission_phase == "LINE_TRACK"
    assert node.phase_manager.hurdles_completed == 1


@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED", "CANCELLED"])
def test_positioning_go_terminal_lifecycle(clock, status):
    node = HurdleHarness()
    go = enter_positioning(node, clock)
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
    node.receive(hurdle(0.8))
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
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None
    node.receive(hurdle())
    fine = node.publish()[-1]
    assert fine["phase"] == "HURDLE_POSITIONING"
    assert fine["action"] == "GO"


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


def test_far_approach_and_close_sequence_trigger():
    planner = MotionDecisionPlanner()
    far = planner.plan("HURDLE_APPROACH", {
        "hurdle": hurdle(0.8), "line": line_info(),
    }, 0.1)
    assert far.action == "STRAIGHT"
    assert far.source == "hurdle"
    assert not far.source_command.get("hurdle_positioning_active", False)
    aligned = planner.plan("HURDLE_POSITIONING", {"hurdle": hurdle(0.6)}, 0.1)
    assert aligned.action == "STRAIGHT_0" and not aligned.source_command["fine_sequence_requested"]
    not_confirmed = planner.plan("HURDLE_POSITIONING", {"hurdle": hurdle(0.15)}, 0.1)
    assert not_confirmed.action == "GO" and not_confirmed.requires_ack
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
    node.receive(hurdle(0.8, hurdle_angle_deg=angle))
    assert node.publish() == []
    release_general(node, approach)
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None
    clock[0] += 0.01
    node.receive_line()
    node.receive(hurdle(0.8, hurdle_angle_deg=angle))
    forward = node.publish()[-1]
    assert forward["source"] == "hurdle"
    assert forward["action"] == "STRAIGHT"


def test_recognition_approach_waits_without_line(clock):
    node = HurdleHarness()
    node.receive(hurdle(0.8, hurdle_angle_deg=-34.0))
    waiting = node.publish()[-1]
    assert not waiting["valid"] and waiting["action"] == "WAIT"
    assert waiting["reason"] == "waiting_for_fresh_hurdle_line_intersection"
    assert waiting["source"] == "hurdle"
    assert not node.planner.hurdle_lock_active


def test_idle_near_entry_starts_sequence_without_entry_dwell(clock):
    node = HurdleHarness()
    go = enter_positioning(node, clock)
    assert go["action"] == "GO"
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None


@pytest.mark.parametrize("depth,allowed", [(1.001, True), (1.0, False), (0.7, False), (0.551, False), (0.55, False)])
def test_recognition_approach_blocks_new_line_prequeue(clock, depth, allowed):
    node = HurdleHarness()
    assert MotionDecisionNode._line_only_prequeue_allowed(
        node, {"hurdle": hurdle(depth)},
    ) is allowed


@pytest.mark.parametrize("depth", [1.0, 0.8, 0.700001])
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


@pytest.mark.parametrize("depth", [0.8, 0.7, 0.4])
@pytest.mark.parametrize("bottom", [101, 100])
def test_positioning_never_rotates_even_if_depth_rebounds(depth, bottom):
    decision = MotionDecisionPlanner().plan("HURDLE_POSITIONING", {
        "hurdle": hurdle(depth, camera_center_offset_x_px=600, bottom_distance_px=bottom),
    }, 0.1)
    assert decision.action == ("GO" if depth <= 0.54 else "STRAIGHT_0")
    assert decision.source_command["close_rotation_blocked"]


@pytest.mark.parametrize("angle", [-75.0, -70.0, -45.0, 0.0, 45.0, 70.0, 75.0])
@pytest.mark.parametrize("phase", ["AUTO", "HURDLE_POSITIONING"])
def test_fine_approach_ignores_center_angle(angle, phase):
    decision = MotionDecisionPlanner().plan(phase, {
        "hurdle": hurdle(0.5, bottom_distance_px=101,
                          camera_center_offset_x_px=101 * math.tan(math.radians(angle))),
    }, 0.1)
    assert decision.valid and decision.action == "GO"
    assert decision.source_command["center_steering_deg"] == pytest.approx(angle, abs=0.001)
    assert "turn_count" not in decision.source_command


@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED", "CANCELLED", "REJECTED"])
def test_recovery_completion_reobserves_intersection(clock, status):
    node = HurdleHarness()
    node.receive_line(x=500)
    node.receive(hurdle(0.8))
    turn = node.publish()[-1]
    assert turn["action"] == "RECOVER_LEFT_TURN_LEFT_4"
    node.send_status(turn["action"], turn["command_id"], "RUNNING")
    node.send_status(turn["action"], turn["command_id"], status)
    clock[0] += 0.1
    node.receive_line()
    node.receive(hurdle(0.8))
    assert node.publish()[-1]["action"] == "STRAIGHT"



def test_positioning_forward_rechecks_fresh_hurdle_without_dwell(clock):
    node = HurdleHarness(phase="HURDLE_POSITIONING")
    node.receive(hurdle(0.8))
    forward = node.publish()[-1]
    assert forward["action"] == "STRAIGHT_0"
    release_general(node, forward)
    assert node.hurdle_post_motion_dwell_until is None
    assert node.hurdle_stationary_since == clock[0]
    assert node.latest_info["hurdle"] is None
    assert not node.general_motion_gate.has_required_fresh_vision()
    assert all(not command["sdk_motion_requested"] for command in node.publish())
    clock[0] += 0.01
    node.receive(hurdle(0.6))
    assert node.publish()[-1]["action"] == "STRAIGHT_0"


def test_crossing_during_recovery_finishes_turn_before_fine(clock):
    node = HurdleHarness()
    node.receive_line(x=500)
    node.receive(hurdle(0.8))
    turn = node.publish()[-1]
    assert turn["action"] == "RECOVER_LEFT_TURN_LEFT_4"
    node.receive(hurdle(0.69))
    assert node.hurdle_positioning_entry_pending
    assert node.publish() == []
    release_general(node, turn)
    clock[0] += 0.1
    node.receive(hurdle(0.69))
    result = node.publish()[-1]
    assert result["action"] == "STRAIGHT_0"
    assert result["phase"] == "HURDLE_POSITIONING"



def test_recovery_requires_current_intersection(clock):
    node = HurdleHarness()
    node.receive_line(x=500)
    node.receive(hurdle(0.8))
    turn = node.publish()[-1]
    release_general(node, turn)
    clock[0] += 2.0
    node.receive(hurdle(0.8))
    result = node.publish()[-1]
    assert result["action"] == "WAIT" and not result["valid"]



@pytest.mark.parametrize("depth,depth_valid", [
    (0.701, True), (0.8, True), (None, False), (0.0, True),
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
    assert command["source"] == ("hurdle" if depth_valid and depth in (0.701, 0.8) else "line")
    assert not node.planner.hurdle_lock_active


def test_depth_entry_waits_for_line_then_stays_hurdle_on_lost_detection(clock):
    node = HurdleHarness()
    node.receive_line()
    node.receive(hurdle(0.8))
    approach = node.publish()[-1]
    assert approach["source"] == "hurdle"
    node.receive(hurdle(0.55, bottom_distance_px=200, head_down_requested=False))
    assert node.hurdle_positioning_entry_pending
    assert node.publish() == []
    release_general(node, approach)
    node.receive({"detected": False})
    waiting = node.publish()[-1]
    assert waiting["phase"] == "HURDLE_POSITIONING"
    assert waiting["source"] == "hurdle" and waiting["action"] == "WAIT"
    clock[0] += 0.1
    node.receive(hurdle(0.8, bottom_distance_px=120, head_down_requested=True))
    fine = node.publish()[-1]
    assert fine["source"] == "hurdle" and fine["action"] == "STRAIGHT_0"


def test_missing_depth_after_depth_entry_waits_in_hurdle_mode(clock):
    node = HurdleHarness()
    node.receive(hurdle(0.55, bottom_distance_px=200, head_down_requested=False))
    assert node.hurdle_positioning_entry_pending
    node.receive(hurdle(None, depth_valid=False, bottom_distance_px=120, head_down_requested=True))
    waiting = node.publish()[-1]
    assert waiting["phase"] == "HURDLE_POSITIONING"
    assert waiting["action"] == "WAIT"
    assert waiting["reason"] == "missing_valid_hurdle_depth"


@pytest.mark.parametrize("depth", [0.700001, 0.8, 1.0])
def test_recognition_depth_never_enters_fine_sequence(clock, depth):
    node = HurdleHarness()
    node.receive_line()
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


@pytest.mark.parametrize("depth", [0.8, 0.7, 0.5])
@pytest.mark.parametrize("x,direction", [(500, "LEFT"), (900, "RIGHT")])
def test_only_coarse_approach_steers_toward_intersection(depth, x, direction):
    decision = MotionDecisionPlanner().plan("AUTO", {
        "hurdle": hurdle(depth, camera_center_offset_x_px=600 if x < 710 else -600,
                          hurdle_angle_deg=None),
        "line": line_info(x=x),
    }, 0.1)
    expected = (f"RECOVER_{direction}_TURN_{direction}_4" if depth > 0.7
                else "STRAIGHT_0" if depth > 0.54 else "GO")
    assert decision.source == "hurdle" and decision.valid
    assert decision.action == expected
    assert decision.source_command["hurdle_stage"] == (
        "RECOGNITION_APPROACH" if depth > 0.7 else "FINE_APPROACH"
    )


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED", "REJECTED"])
def test_fine_sequence_failure_releases_go_without_counting_hurdle(clock, status):
    node = HurdleHarness()
    go = enter_positioning(node, clock)
    node.send_status("GO", go["command_id"], "RUNNING", motion_id="pickup_fine_forward_0")
    assert node.phase_manager.hurdles_executed == 0
    node.send_status("GO", go["command_id"], status, motion_id="pickup_fine_forward_0")
    assert node.active_special_command_id is None
    assert node.mission_phase == "HURDLE_POSITIONING"
    assert node.phase_manager.hurdles_completed == 0


def test_video_search_during_positioning_waits_even_with_visible_line():
    planner = MotionDecisionPlanner()
    result = planner.plan('HURDLE_POSITIONING', {
        'line': line_info(), 'hurdle': {'detected': False, 'state': 'SEARCH'},
    }, .1)
    assert result.source == 'hurdle'
    assert result.action == 'WAIT' and not result.valid
    assert result.reason == 'hurdle_not_detected'
    assert result.source_command['hurdle_stage'] == 'FINE_APPROACH'
    restored = planner.plan('HURDLE_POSITIONING', {
        'line': line_info(), 'hurdle': hurdle(.2),
    }, .1)
    assert restored.action == 'GO' and restored.valid


@pytest.mark.parametrize('source', ['line', 'hurdle'])
@pytest.mark.parametrize('lost_info', [{'detected': False}, None])
def test_520mm_checkpoint_survives_loss_until_current_motion_and_settle_finish(clock, source, lost_info):
    node = HurdleHarness()
    node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC = 1.
    node.receive_line()
    if source == 'hurdle':
        node.receive(hurdle(.8))
    current = node.publish()[-1]
    assert current['source'] == source
    node.send_status(current['action'], current['command_id'], 'RUNNING')
    node.receive(hurdle(.694, bottom_distance_px=172))
    assert node.hurdle_positioning_entry_pending
    assert getattr(node, 'pending_hurdle_final_sequence', None) is None
    # Values read from the user's screenshot; the analyzer's go_now is false.
    node.receive(hurdle(.52, camera_center_offset_x_px=-286,
                        bottom_distance_px=195, hurdle_angle_deg=-3.5, go_now=False))
    checkpoint = node.pending_hurdle_final_sequence
    assert checkpoint['depth_m'] == .52
    assert checkpoint['center_steering_deg'] == pytest.approx(-55.714, abs=.01)
    if lost_info is None:
        clock[0] += 1.
    else:
        node.receive(lost_info)
    assert node.publish() == []
    node.send_status(current['action'], current['command_id'] + 1, 'SUCCEEDED')
    assert node.publish() == []
    release_general(node, current)
    assert node.publish() == []
    clock[0] += .99
    assert node.publish() == []
    clock[0] += .02
    result = node.publish()[-1]
    assert result['action'] == 'GO' and result['valid']
    assert result['reason'] == 'hurdle_final_sequence_latched'
    assert result['source_command']['fine_sequence_requested']
    assert result['source_command']['depth_m'] == .52
    assert node.pending_hurdle_final_sequence is None
    assert not any(c['action'] == 'GO' and c['valid'] for c in node.publish())
    node.send_status('GO', result['command_id'], 'RUNNING', motion_id='pickup_fine_forward_0')
    node.receive({'detected': False})
    assert not any(c['action'] == 'GO' and c['valid'] for c in node.publish())


@pytest.mark.parametrize('depth,latched', [(.52, True), (.54, True), (.540001, False), (.694, False)])
def test_final_sequence_reservation_uses_540mm_not_700mm(clock, depth, latched):
    node = HurdleHarness()
    node.receive(hurdle(depth))
    assert (getattr(node, 'pending_hurdle_final_sequence', None) is not None) is latched
    node.receive({'detected': False})
    result = node.publish()[-1]
    assert result['action'] == ('GO' if latched else 'WAIT')
    assert result['valid'] is latched


@pytest.mark.parametrize('updates', [
    {'detected': False}, {'confirmation_confirmed': False},
    {'confirmation_confirmed': 'true'}, {'confidence': .59},
    {'depth_valid': False}, {'depth_m': None}, {'depth_m': 0.},
    {'depth_m': -1.}, {'depth_m': True}, {'depth_m': float('nan')},
    {'depth_m': float('inf')},
])
def test_invalid_540mm_observation_cannot_reserve_sequence(clock, updates):
    node = HurdleHarness()
    node.receive(hurdle(.694))
    node.receive(hurdle(.52, **updates))
    assert getattr(node, 'pending_hurdle_final_sequence', None) is None
    node.receive({'detected': False})
    assert node.publish()[-1]['action'] == 'WAIT'


def test_fine_sequence_reservation_does_not_request_late_turn(clock):
    node = HurdleHarness()
    node.receive(hurdle(.52, bottom_distance_px=200, camera_center_offset_x_px=600))
    checkpoint = node.pending_hurdle_final_sequence
    assert checkpoint["fine_sequence_requested"]
    assert checkpoint["close_rotation_blocked"]


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_preceding_motion_cancels_final_sequence_reservation(clock, status):
    node = HurdleHarness()
    node.receive_line()
    current = node.publish()[-1]
    node.send_status(current['action'], current['command_id'], 'RUNNING')
    node.receive(hurdle(.52))
    assert node.pending_hurdle_final_sequence is not None
    node.receive({'detected': False})
    node.send_status(current['action'], current['command_id'], status)
    assert node.pending_hurdle_final_sequence is None
    assert not any(c['action'] == 'GO' and c['valid'] for c in node.publish())


def test_phase_override_cancels_final_sequence_reservation(clock):
    node = HurdleHarness()
    node.receive(hurdle(.52))
    node.send_phase('LINE_TRACK')
    assert node.pending_hurdle_final_sequence is None
    node.receive({'detected': False})
    assert not any(c['action'] == 'GO' and c['valid'] for c in node.publish())


def test_executor_startup_hold_cancels_final_sequence_reservation(clock):
    node = HurdleHarness()
    node.receive(hurdle(.52))
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 3, 'active': True, 'auto_ready': False,
    })))
    assert node.pending_hurdle_final_sequence is None
    node.receive(hurdle(.52))
    assert node.pending_hurdle_final_sequence is None
    assert node.publish() == []


def test_reserved_final_sequence_cannot_bypass_executor_fault(clock):
    node = HurdleHarness()
    node.receive(hurdle(.52))
    node.receive({'detected': False})
    node.safety_interlock.observe_critical_fault(
        error_code='SDK_HARDWARE_NOT_READY', message='test fault', source='executor',
        action='GO', command_id=None, event_id=None,
    )
    assert node.publish() == []


@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_final_sequence_is_consumed_once_and_not_replayed_after_terminal(clock, status):
    node = HurdleHarness()
    node.receive(hurdle(.52))
    node.receive({'detected': False})
    go = node.publish()[-1]
    assert go['action'] == 'GO'
    node.send_status('GO', go['command_id'], 'RUNNING', motion_id='pickup_fine_forward_0')
    node.receive(hurdle(.52))
    assert node.pending_hurdle_final_sequence is None
    node.send_status('GO', go['command_id'], status, motion_id='hurdle')
    node.receive({'detected': False})
    assert not any(c['action'] == 'GO' and c['valid'] for c in node.publish())


def test_520mm_seen_during_positioning_fine_step_survives_input_invalidation(clock):
    node = HurdleHarness()
    node.receive(hurdle(.6))
    forward = node.publish()[-1]
    assert forward['action'] == 'STRAIGHT_0'
    node.receive(hurdle(.52))
    node.receive({'detected': False})
    release_general(node, forward)
    assert node.latest_info['hurdle'] is None
    clock[0] += MotionDecisionNode.CORRECTION_POST_MOTION_DWELL_SEC
    result = node.publish()[-1]
    assert result['action'] == 'GO' and result['source_command']['depth_m'] == .52


@pytest.mark.parametrize('queued_status', ['SUCCEEDED', 'REJECTED'])
def test_final_checkpoint_waits_for_already_queued_motion_or_cancels_on_rejection(clock, queued_status):
    node = HurdleHarness()
    node.receive_line()
    current = node.publish()[-1]
    node.queued_general_action = 'STRAIGHT'
    node.queued_general_command_id = 99
    node.queued_general_source = 'line'
    node.receive(hurdle(.52))
    node.receive({'detected': False})
    release_general(node, current)
    assert node.publish() == []
    if queued_status == 'SUCCEEDED':
        node.send_status('STRAIGHT', 99, 'RUNNING')
        assert node.publish() == []
    node.send_status('STRAIGHT', 99, queued_status)
    result = node.publish()[-1]
    assert result['action'] == ('GO' if queued_status == 'SUCCEEDED' else 'WAIT')
    assert node.pending_hurdle_final_sequence is None
