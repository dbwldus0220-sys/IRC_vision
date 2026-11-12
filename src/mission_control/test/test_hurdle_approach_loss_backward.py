"""Retreat on fresh loss of an acquired hurdle before the 580 mm crossing."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock
from test_hurdle_580_sequence import executable, observe
from test_hurdle_positioning import HurdleHarness, hurdle
from test_motion_command_bridge import FakeBridge, decoded_messages


def missing(pose="45", ready=True):
    return {"detected": False, "raw_detected": False,
            "hurdle_camera_pose": pose, "hurdle_camera_ready": ready}


def start_and_lose(node, clock, depth):
    observe(node, clock, hurdle(depth))
    node.publish()
    first = executable(node)[-1]
    node.send_status(first["action"], first["command_id"], "RUNNING", "line_forward_6")
    started, duration = clock[0], node.approach_walk_duration_sec
    clock[0] = started + duration * .4
    observe(node, clock, missing())
    node.publish()
    assert len(executable(node)) == 1
    clock[0] = started + duration
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    observe(node, clock, missing())
    node.publish()
    assert len(executable(node)) == 1
    stopped = clock[0]
    clock[0] = stopped + .9
    observe(node, clock, missing())
    node.publish()
    assert len(executable(node)) == 1
    clock[0] = stopped + 1.
    node.publish()
    return executable(node)[-1], stopped


@pytest.mark.parametrize("depth", [.76, .88])
def test_recorded_approach_loss_requests_camera_down_and_backward(clock, depth):
    node = HurdleHarness()
    retreat, stopped = start_and_lose(node, clock, depth)
    assert len(executable(node)) == 2
    assert retreat["action"] == "HURDLE_LOST_BACKWARD_2"
    assert retreat["reason"] == "hurdle_approach_missing_backward_2"
    assert node.planner.hurdle_lock_active
    assert not node.planner.hurdle_planner.fine_approach_active
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    assert retreat["source_command"]["hurdle_camera_down_control"]

    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = "line_forward_6"
    bridge.stationary_since = stopped
    bridge.navigation_command_callback(String(data=json.dumps(retreat)))
    assert not bridge.executor_request_publisher.messages
    for current in (-34., -60., -63.8):
        bridge.executor_heartbeat_callback(String(data=json.dumps({
            "hurdle_head_override_active": True,
            "hurdle_head_override_current_deg": current,
        })))
        bridge._check_transition_dwell()
        assert not bridge.executor_request_publisher.messages
    bridge.executor_heartbeat_callback(String(data=json.dumps({
        "hurdle_head_override_active": True,
        "hurdle_head_override_current_deg": -64.,
    })))
    bridge._check_transition_dwell()
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == (
        "ball_lost_camera_down_backward_2"
    )


def test_repeated_loss_requires_completion_rest_and_new_capture(clock):
    node = HurdleHarness()
    retreat, _ = start_and_lose(node, clock, .76)
    motion = "ball_lost_camera_down_backward_2"
    node.send_status(retreat["action"], retreat["command_id"], "RUNNING", motion)
    for _ in range(5):
        clock[0] += .1
        observe(node, clock, missing("0"))
        node.publish()
        assert len(executable(node)) == 2
    node.send_status(retreat["action"], retreat["command_id"], "SUCCEEDED", motion)
    stopped = clock[0]
    clock[0] += 1.
    node.publish()
    assert len(executable(node)) == 2
    observe(node, clock, missing("0"), delay=1.1)
    node.publish()
    assert len(executable(node)) == 2
    clock[0] += .01
    observe(node, clock, missing("0"))
    node.publish()
    assert len(executable(node)) == 3
    assert executable(node)[-1]["action"] == "HURDLE_LOST_BACKWARD_2"
    assert clock[0] >= stopped + 1.


@pytest.mark.parametrize("info", [
    None, missing(ready=False), missing(pose=None),
    {**missing(), "raw_detected": True},
    hurdle(.76, confirmation_confirmed=False),
    hurdle(.76, depth_valid=False),
    hurdle(.76, hurdle_angle_45_deg=None),
])
def test_other_wait_reasons_do_not_authorize_retreat(info):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": hurdle(.76)}, .1)
    decision = planner.plan("AUTO", {"hurdle": info}, .1)
    assert not decision.valid
    assert decision.action == "WAIT"


def test_missing_hurdle_without_acquired_lock_does_not_retreat():
    decision = MotionDecisionPlanner().plan("HURDLE_APPROACH", {"hurdle": missing()}, .1)
    assert decision.action != "HURDLE_LOST_BACKWARD_2"


@pytest.mark.parametrize("depth,expected", [(.8, "ALIGN_RIGHT"), (.58, "ALIGN_RIGHT")])
def test_reacquired_hurdle_uses_lowered_camera_turn_without_false_near_entry(clock, depth, expected):
    node = HurdleHarness()
    retreat, _ = start_and_lose(node, clock, .76)
    motion = "ball_lost_camera_down_backward_2"
    node.send_status(retreat["action"], retreat["command_id"], "RUNNING", motion)
    clock[0] += 1.
    node.send_status(retreat["action"], retreat["command_id"], "SUCCEEDED", motion)
    clock[0] += .9
    observe(node, clock, hurdle(depth, hurdle_camera_pose="0", hurdle_angle_0_deg=-60.))
    # A newly observed near crossing starts its own stationary observation window.
    clock[0] = max(clock[0] + .1, node.hurdle_post_motion_dwell_until or 0.)
    observe(node, clock, hurdle(depth, hurdle_camera_pose="0", hurdle_angle_0_deg=-60.))
    node.publish()
    turn = executable(node)[-1]
    assert turn["action"] == expected
    assert node.planner.hurdle_planner.fine_approach_active == (depth <= .58)
    assert not node.planner.hurdle_go_requested
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(turn)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"].startswith(
        "pickup_camera_down_turn_right_"
    )


def test_reacquired_aligned_far_hurdle_cannot_start_go():
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": hurdle(.76)}, .1)
    assert planner.plan("AUTO", {"hurdle": missing()}, .1).action == "HURDLE_LOST_BACKWARD_2"
    decision = planner.plan("AUTO", {"hurdle": hurdle(.8, hurdle_camera_pose="0")}, .1)
    assert decision.action == "STRAIGHT_3"
    assert not planner.hurdle_planner.fine_approach_active
    assert not planner.hurdle_go_requested
