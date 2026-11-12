"""Replay representative observations from the November 12 hurdle video.

These are synthetic node inputs, not a replay of the original RGB/depth stream.
"""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_hurdle_580_sequence import executable, observe
from test_hurdle_approach_prequeue import start, frame
from test_hurdle_positioning import HurdleHarness, hurdle, line_info


def record(node, clock, event):
    latest = node.publisher.messages[-1] if node.publisher.messages else {}
    print(json.dumps({
        "event": event, "time": round(clock[0], 3),
        "lock": node.planner.hurdle_lock_active,
        "near": node.planner.hurdle_planner.fine_approach_active,
        "running": node.general_motion_gate.active_action,
        "queued": getattr(node, "queued_general_action", None),
        "last_published_source": latest.get("source"),
        "last_published_action": latest.get("action"),
        "reason": latest.get("reason"),
        "executable_count": len(executable(node)),
    }, sort_keys=True))


def line_frame(node, clock, depth):
    Node._info_callback(node, "line")(String(data=json.dumps({
        **line_info(), "nearest_line_depth_m": depth,
        "rgb_stamp_ns": round(clock[0] * 1e9),
    })))


@pytest.mark.parametrize("depth,line_depth,heading", [
    (.76, .563, 6.7), (.88, .596, 10.3),
], ids=["first_video_scene", "second_video_scene"])
@pytest.mark.parametrize("loss_kind", ["camera_unknown", "expired"])
def test_recorded_loss_without_camera_readiness_waits_then_resumes(
    clock, depth, line_depth, heading, loss_kind,
):
    node = HurdleHarness()
    observe(node, clock, hurdle(depth, hurdle_angle_45_deg=-heading))
    node.publish()
    first = executable(node)[-1]
    assert first["source"] == "hurdle" and first["action"] == "STRAIGHT_3"
    node.send_status(first["action"], first["command_id"], "RUNNING", "line_forward_6")
    started, duration = clock[0], node.approach_walk_duration_sec
    record(node, clock, "confirmed_approach")

    clock[0] = started + duration * .4
    line_frame(node, clock, line_depth)
    if loss_kind == "camera_unknown":
        observe(node, clock, {"detected": False, "raw_detected": False})
    else:
        observe(node, clock, hurdle(depth), delay=.6)
    node.publish()
    assert len(executable(node)) == 1
    assert node.general_motion_gate.active_command_id == first["command_id"]
    assert node.planner.hurdle_lock_active
    record(node, clock, "loss_during_current_motion")

    clock[0] = started + duration
    line_frame(node, clock, line_depth)
    if loss_kind == "camera_unknown":
        observe(node, clock, {"detected": False, "raw_detected": False})
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    node.publish()
    expected_reason = "hurdle_not_detected" if loss_kind == "camera_unknown" else "waiting_for_hurdle_info"
    for elapsed in (0., 1., 5., 20.):
        clock[0] = started + duration + elapsed
        line_frame(node, clock, line_depth)
        if loss_kind == "camera_unknown":
            observe(node, clock, {"detected": False, "raw_detected": False})
        node.publish()
        latest = node.publisher.messages[-1]
        assert latest["source"] == "hurdle" and latest["action"] == "WAIT"
        assert latest["reason"] == expected_reason
        assert len(executable(node)) == 1
        assert node.planner.hurdle_lock_active
        assert not node.planner.hurdle_planner.fine_approach_active
        record(node, clock, f"after_completion_plus_{elapsed}s")

    clock[0] += .1
    observe(node, clock, hurdle(.62))
    node.publish()
    resumed = executable(node)[-1]
    assert resumed["source"] == "hurdle" and resumed["action"] == "STRAIGHT_3"
    assert len(executable(node)) == 2
    assert not node.planner.hurdle_planner.fine_approach_active
    record(node, clock, "reacquired_at_620mm")


@pytest.mark.parametrize("depth", [.580, .564])
def test_recorded_near_crossing_then_loss_retreats_only_after_camera_ready(clock, depth):
    node, first, started, duration = start(clock)
    clock[0] = started + duration * .4
    observe(node, clock, hurdle(depth, head_down_requested=True, hurdle_camera_ready=False))
    assert node.hurdle_positioning_entry_pending
    assert node.planner.hurdle_planner.fine_approach_active
    record(node, clock, "valid_near_crossing")
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    missing = {"detected": False, "raw_detected": False,
               "hurdle_camera_pose": "0", "hurdle_camera_ready": False}
    clock[0] += 1.
    observe(node, clock, missing)
    node.publish()
    assert len(executable(node)) == 1
    assert node.publisher.messages[-1]["action"] == "WAIT"
    record(node, clock, "camera_not_ready")
    clock[0] += .1
    observe(node, clock, {**missing, "hurdle_camera_ready": True})
    node.publish()
    assert len(executable(node)) == 2
    assert executable(node)[-1]["action"] == "HURDLE_LOST_BACKWARD_2"
    record(node, clock, "camera_ready_missing_retreat")


def test_loss_without_camera_readiness_finishes_reserved_motion_then_waits(clock):
    node, first, started, duration = start(clock)
    for ratio in (.72, .78, .84):
        frame(node, clock, started, duration, ratio)
    queued = executable(node)[-1]
    node.send_status(queued["action"], queued["command_id"], "QUEUED", "line_forward_6")
    clock[0] = started + duration * .9
    observe(node, clock, {"detected": False, "raw_detected": False})
    node.publish()
    record(node, clock, "loss_after_accepted_queue")
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    node.send_status(queued["action"], queued["command_id"], "RUNNING", "line_forward_6")
    second_started = clock[0]
    for ratio in (.72, .78, .84, 1.):
        clock[0] = second_started + duration * ratio
        node.receive_line()
        observe(node, clock, {"detected": False, "raw_detected": False})
        node.publish()
        assert len(executable(node)) == 2
        assert node.queued_general_command_id is None
    node.send_status(queued["action"], queued["command_id"], "SUCCEEDED", "line_forward_6")
    node.publish()
    assert node.publisher.messages[-1]["source"] == "hurdle"
    assert node.publisher.messages[-1]["action"] == "WAIT"
    assert node.planner.hurdle_lock_active
    record(node, clock, "reserved_motion_finished_wait")
