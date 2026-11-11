"""Replay far walking reservations and the higher-priority near Hurdle handoff."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_hurdle_580_sequence import executable, near, observe, start_far
from test_hurdle_positioning import HurdleHarness, hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages


def start(clock, source="hurdle"):
    node = HurdleHarness()
    node.queued_general_command_id = None
    node.queued_general_action = None
    node.queued_general_source = None
    if source == "hurdle":
        first = start_far(node, clock)
        duration = node.approach_walk_duration_sec
    else:
        info = {**line_info(), "rgb_stamp_ns": round(clock[0] * 1e9)}
        Node._info_callback(node, "line")(String(data=json.dumps(info)))
        node.publish()
        first = executable(node)[-1]
        node.send_status(first["action"], first["command_id"], "RUNNING", "line_forward_6")
        duration = node.active_line_motion_duration_sec
    return node, first, clock[0], duration


def frame(node, clock, started, duration, ratio, heading=0., **changes):
    clock[0] = started + duration * ratio
    observe(node, clock, hurdle(.9, hurdle_angle_45_deg=-heading, **changes))


@pytest.mark.parametrize("source", ["line", "hurdle"])
@pytest.mark.parametrize("heading,action", [
    (0., "STRAIGHT_3"), (-30., "RECOVER_LEFT_TURN_LEFT_4"),
    (30., "RECOVER_RIGHT_TURN_RIGHT_4"),
])
def test_far_hurdle_reserves_one_gait_without_line_geometry(clock, source, heading, action):
    node, first, started, duration = start(clock, source)
    node.latest_info["line"] = None
    node.latest_time["line"] = None
    for ratio in (.72, .78):
        frame(node, clock, started, duration, ratio, heading)
        assert node.queued_general_command_id is None
    frame(node, clock, started, duration, .84, heading)
    queued = executable(node)[-1]
    assert queued["source"] == "hurdle" and queued["action"] == action
    assert node.queued_general_command_id == queued["command_id"] != first["command_id"]
    count = len(executable(node))
    frame(node, clock, started, duration, .90, heading)
    assert len(executable(node)) == count

    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 2 and not bridge.queued_request_deferred
    assert requests[-1]["motion_id"] == (
        "line_forward_6" if heading == 0. else
        "line_recovery_left_4" if heading < 0. else "line_recovery_right_4")
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", requests[0]["motion_id"])
    node.send_status(queued["action"], queued["command_id"], "RUNNING", requests[1]["motion_id"])
    assert node.active_general_source == "hurdle"
    assert node.general_motion_gate.active_command_id == queued["command_id"]
    assert node.approach_walk_started_at == clock[0]


@pytest.mark.parametrize("changes", [
    {"confirmation_confirmed": False}, {"detected": False, "raw_detected": False},
    {"depth_valid": False}, {"confidence": .1}, {"depth_m": 1.01},
    {"hurdle_camera_ready": False}, {"hurdle_camera_pose": "0"},
    {"hurdle_angle_45_deg": None}, {"depth_age_sec": .701},
])
def test_invalid_frame_breaks_the_candidate_streak(clock, changes):
    node, _, started, duration = start(clock)
    frame(node, clock, started, duration, .71)
    frame(node, clock, started, duration, .75)
    clock[0] = started + duration * .79
    observe(node, clock, {**hurdle(.9), **changes})
    assert node.queued_general_command_id is None
    frame(node, clock, started, duration, .83)
    frame(node, clock, started, duration, .87)
    assert node.queued_general_command_id is None
    frame(node, clock, started, duration, .91)
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize("kind", ["duplicate", "early_capture", "expired", "future"])
def test_three_arrivals_are_not_three_fresh_late_captures(clock, kind):
    node, _, started, duration = start(clock)
    for ratio in (.75, .80, .85):
        clock[0] = started + duration * ratio
        captured = (started + duration * .74 if kind == "duplicate" else
                    started + duration * .69 if kind == "early_capture" else
                    clock[0] - .6 if kind == "expired" else clock[0] + .1)
        observe(node, clock, hurdle(.9), delay=clock[0] - captured)
    assert node.queued_general_command_id is None


def test_changed_action_needs_three_new_captures(clock):
    node, _, started, duration = start(clock)
    for ratio, heading in [(.71, 0), (.75, 0), (.79, 30), (.83, 30)]:
        frame(node, clock, started, duration, ratio, heading)
        assert node.queued_general_command_id is None
    frame(node, clock, started, duration, .87, 30)
    assert node.queued_general_action == "RECOVER_RIGHT_TURN_RIGHT_4"


def test_recovery_chain_waits_until_last_300ms_before_reserving(clock):
    node, first, started, duration = start(clock)
    for ratio in (.72, .78, .84):
        frame(node, clock, started, duration, ratio, heading=30.)
    recovery = executable(node)[-1]
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    node.send_status(recovery["action"], recovery["command_id"], "RUNNING", "line_recovery_right_4")
    started, duration = clock[0], node.approach_walk_duration_sec
    for ratio in (.72, .78, .84):
        frame(node, clock, started, duration, ratio, heading=30.)
    assert node.hurdle_approach_capture[3] == 3
    assert node.queued_general_command_id is None
    frame(node, clock, started, duration, .90, heading=30.)
    assert node.queued_general_action == "RECOVER_RIGHT_TURN_RIGHT_4"


@pytest.mark.parametrize("source", ["line", "hurdle"])
@pytest.mark.parametrize("heading,action", [(0, "GO"), (30, "ALIGN_RIGHT"), (-30, "ALIGN_LEFT")])
def test_near_crossing_discards_far_candidate_and_replans_after_current_gait(clock, source, heading, action):
    node, first, started, duration = start(clock, source)
    for ratio in (.72, .78):
        frame(node, clock, started, duration, ratio)
    clock[0] = started + duration * .84
    observe(node, clock, hurdle(.57, hurdle_camera_ready=False))
    assert node.hurdle_positioning_entry_pending
    assert node.hurdle_approach_capture is None
    assert node.queued_general_command_id is None
    frame(node, clock, started, duration, .90)  # A depth rebound cannot undo near entry.
    assert node.queued_general_command_id is None
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    stopped = clock[0]
    count = len(executable(node))
    clock[0] = stopped + .8
    observe(node, clock, near(.8, heading))
    node.publish()
    assert len(executable(node)) == count
    clock[0] = stopped + 1.
    node.publish()
    assert executable(node)[-1]["action"] == action
    assert executable(node)[-1]["phase"] == "HURDLE_POSITIONING"


def test_near_crossing_after_accepted_reservation_cannot_add_another_far_gait(clock):
    node, first, started, duration = start(clock)
    for ratio in (.72, .78, .84):
        frame(node, clock, started, duration, ratio)
    queued = executable(node)[-1]
    node.send_status(queued["action"], queued["command_id"], "QUEUED", "line_forward_6")
    clock[0] = started + duration * .90
    observe(node, clock, hurdle(.57, hurdle_camera_ready=False))
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    node.send_status(queued["action"], queued["command_id"], "RUNNING", "line_forward_6")
    count = len(executable(node))
    second_started = clock[0]
    for ratio in (.72, .78, .84):
        frame(node, clock, second_started, duration, ratio)
    assert len(executable(node)) == count and node.queued_general_command_id is None
    clock[0] = second_started + duration
    node.send_status(queued["action"], queued["command_id"], "SUCCEEDED", "line_forward_6")
    stopped = clock[0]
    clock[0] += .8
    observe(node, clock, near(heading=30))
    clock[0] = stopped + 1.
    node.publish()
    assert executable(node)[-1]["action"] == "ALIGN_RIGHT"


def test_line_preview_cannot_count_the_same_hurdle_image_twice(clock):
    node, _, started, duration = start(clock, "line")
    frame(node, clock, started, duration, .72)
    info = {**line_info(), "rgb_stamp_ns": round(clock[0] * 1e9)}
    node.latest_info["line"] = info
    node.latest_time["line"] = clock[0]
    node.active_line_motion_frames = [info] * node.active_line_motion_target_frames
    for _ in range(10):
        Node._prepare_pending_line_decision(node, clock[0])
        assert node.pending_line_decision is None
    assert node.hurdle_approach_capture[3] == 1
    assert node.queued_general_command_id is None
    for ratio in (.78, .84):
        frame(node, clock, started, duration, ratio)
    assert node.queued_general_source == "hurdle"


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED"])
def test_running_failure_clears_queued_hurdle_and_does_not_start_go(clock, status):
    node, first, started, duration = start(clock)
    for ratio in (.72, .78, .84):
        frame(node, clock, started, duration, ratio)
    clock[0] = started + duration * .90
    observe(node, clock, hurdle(.57))
    count = len(executable(node))
    node.send_status(first["action"], first["command_id"], status, "line_forward_6")
    assert node.queued_general_command_id is None
    assert not node.hurdle_positioning_entry_pending
    node.publish()
    assert len(executable(node)) == count


def test_rejected_reservation_keeps_near_entry_and_replans_after_current_motion(clock):
    node, first, started, duration = start(clock)
    for ratio in (.72, .78, .84):
        frame(node, clock, started, duration, ratio)
    queued = executable(node)[-1]
    clock[0] = started + duration * .90
    observe(node, clock, hurdle(.57, hurdle_camera_ready=False))
    node.send_status(queued["action"], queued["command_id"], "REJECTED", "line_forward_6")
    assert node.hurdle_positioning_entry_pending
    assert node.queued_general_command_id is None and node.general_motion_gate.locked
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    stopped = clock[0]
    clock[0] += .8
    observe(node, clock, near(heading=-30))
    clock[0] = stopped + 1.
    node.publish()
    assert executable(node)[-1]["action"] == "ALIGN_LEFT"
