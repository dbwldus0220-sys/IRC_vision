"""Third-hurdle finish entry and seamless six-cycle requests, without hardware."""

import json

import pytest
from std_msgs.msg import String

from mission_control.finish_run import FinishRun
from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_hurdle_580_sequence import observe
from test_hurdle_positioning import HurdleHarness, hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


def ready(clock):
    node = HurdleHarness()
    node.phase_manager.required_hurdles = 3
    node.phase_manager.hurdles_completed = 2
    node.phase_manager.hurdles_executed = 2
    node.queued_general_action = None
    node.queued_general_command_id = None
    node.queued_general_source = None
    node.executor_heartbeat_watchdog.observe(sequence=1, observed_at=clock[0])
    observe(node, clock, {"detected": False, "raw_detected": False})
    return node


def start(clock):
    node = ready(clock)
    observe(node, clock, hurdle(.580))
    node.publish()
    return node


def status(node, command, value, motion=FinishRun.MOTION, **extra):
    Node._motion_status_callback(node, String(data=json.dumps({
        "command_id": command["command_id"], "request_id": command["command_id"],
        "event_id": None, "action": FinishRun.ACTION, "status": value,
        "motion_id": motion, **extra,
    })))


@pytest.mark.parametrize("depth,accepted", [(.580, True), (.579, True), (.580001, False),
    (.8, False), (0, False), (-1, False), (None, False), (True, False), (float("nan"), False)])
def test_finish_depth_boundary(clock, depth, accepted):
    node = ready(clock)
    observe(node, clock, hurdle(depth))
    assert node.finish_run.latched is accepted
    assert not node.planner.hurdle_lock_active
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    if accepted:
        node.publish()
        command = node.publisher.messages[-1]
        assert command["phase"] == FinishRun.PHASE
        assert command["action"] == FinishRun.ACTION
        assert command["source_command"]["head_down_requested"] is False


@pytest.mark.parametrize("changes", [{"detected": False}, {"confirmation_confirmed": False},
    {"depth_valid": False}, {"depth_age_sec": .701}, {"depth_age_sec": None}, {"confidence": .59}])
def test_finish_requires_confirmed_fresh_depth(clock, changes):
    node = ready(clock)
    observe(node, clock, hurdle(.57, **changes))
    assert not node.finish_run.latched


def test_old_frame_and_second_hurdle_cannot_start_finish(clock):
    node = ready(clock)
    node.finish_run.clear_seen = False
    observe(node, clock, hurdle(.57))
    assert not node.finish_run.latched
    observe(node, clock, {"detected": False, "raw_detected": False})
    observe(node, clock, hurdle(.57), delay=.6)
    assert not node.finish_run.latched
    observe(node, clock, hurdle(.57))
    assert node.finish_run.latched


def test_far_third_hurdle_keeps_line_control(clock):
    node = ready(clock)
    Node._info_callback(node, "line")(String(data=json.dumps({
        **line_info(), "rgb_stamp_ns": round(clock[0] * 1e9),
    })))
    observe(node, clock, hurdle(.8))
    node.publish()
    assert node.publisher.messages[-1]["source"] == "line"
    assert not node.finish_run.latched
    assert not node.planner.hurdle_lock_active


def test_finish_entry_waits_for_accepted_motion_without_near_dwell(clock):
    node = ready(clock)
    node.command_id = 10
    node.general_motion_gate.on_command_published("STRAIGHT_3", 10)
    node.active_general_source = "line"
    node.send_status("STRAIGHT_3", 10, "RUNNING", "line_forward_6")
    observe(node, clock, hurdle(.57))
    count = len(node.publisher.messages)
    node.publish()
    assert len(node.publisher.messages) == count
    node.send_status("STRAIGHT_3", 10, "SUCCEEDED", "line_forward_6")
    node.publish()
    assert node.publisher.messages[-1]["phase"] == FinishRun.PHASE
    assert node.publisher.messages[-1]["command_id"] == 11
    assert getattr(node, "hurdle_post_motion_dwell_until", None) is None


def test_unrelated_status_cannot_reserve_another_finish_motion(clock):
    node = start(clock)
    current = node.publisher.messages[-1]
    count = len(node.publisher.messages)
    status(node, {"command_id": current["command_id"] + 999}, "RUNNING")
    status(node, current, "RUNNING", event_id=42)
    node.publish()
    assert len(node.publisher.messages) == count


def test_lost_executor_heartbeat_blocks_finish_refill(clock):
    node = start(clock)
    current = node.publisher.messages[-1]
    count = len(node.publisher.messages)
    clock[0] += 3.
    status(node, current, "RUNNING")
    node.publish()
    assert len(node.publisher.messages) == count
    assert node.safety_interlock.latched


def test_first_two_near_entries_keep_original_path(clock):
    for completed in (0, 1):
        node = HurdleHarness()
        node.phase_manager.hurdles_completed = completed
        observe(node, clock, hurdle(.57))
        assert node.hurdle_positioning_entry_pending
        assert not getattr(getattr(node, "finish_run", None), "latched", False)


def test_second_hurdle_success_arms_finish_only_after_obstacle_clears(clock):
    node = ready(clock)
    node.phase_manager.hurdles_completed = 1
    node.phase_manager.hurdles_executed = 1
    node.finish_run.clear_seen = False
    assert node.phase_manager.start_special_action("GO", 50)
    node.send_status("GO", 50, "RUNNING", "hurdle")
    node.send_status("GO", 50, "SUCCEEDED", "hurdle")
    assert node.phase_manager.hurdles_completed == 2
    assert node.mission_phase == "LINE_TRACK"
    observe(node, clock, hurdle(.3))
    assert not node.finish_run.latched
    observe(node, clock, {"detected": False, "raw_detected": False})
    observe(node, clock, hurdle(.58))
    node.publish()
    assert node.publisher.messages[-1]["phase"] == FinishRun.PHASE
    assert node.publisher.messages[-1]["action"] == FinishRun.ACTION


def test_six_cycle_stream_queues_once_and_survives_all_vision_loss(clock):
    node = start(clock)
    current = node.publisher.messages[-1]
    node.latest_info = {key: None for key in node.SOURCES}
    node.latest_time = {key: None for key in node.SOURCES}
    node.executor_ready_requires_fresh_vision = True
    for _ in range(12):
        before = len(node.publisher.messages)
        status(node, current, "RUNNING")
        assert len(node.publisher.messages) == before + 1
        next_command = node.publisher.messages[-1]
        assert next_command["action"] == FinishRun.ACTION
        status(node, next_command, "QUEUED")
        status(node, current, "RUNNING")
        node.publish()
        assert len(node.publisher.messages) == before + 1
        status(node, current, "SUCCEEDED")
        status(node, current, "SUCCEEDED")
        assert len(node.publisher.messages) == before + 1
        current = next_command
    assert node.finish_run.completed_batches == 12
    assert node.phase_manager.hurdles_completed == 2
    assert node.mission_phase == FinishRun.PHASE


@pytest.mark.parametrize("failure", ["FAILED", "TIMEOUT", "CANCELLED", "REJECTED"])
def test_motion_fault_stops_refill_until_restart(clock, failure):
    node = start(clock)
    current = node.publisher.messages[-1]
    status(node, current, "RUNNING")
    queued = node.publisher.messages[-1]
    status(node, current, failure, error_code="SDK_HARDWARE_NOT_READY")
    count = len(node.publisher.messages)
    status(node, queued, "RUNNING")
    status(node, queued, "SUCCEEDED")
    node.publish()
    assert len(node.publisher.messages) == count
    assert node.finish_run.fault


def test_motor_off_then_ready_does_not_restart(clock):
    node = start(clock)
    Node._executor_heartbeat_callback(node, String(data=json.dumps({"sequence": 2, "auto_ready": False})))
    Node._executor_heartbeat_callback(node, String(data=json.dumps({"sequence": 3, "auto_ready": True})))
    count = len(node.publisher.messages)
    node.publish()
    assert len(node.publisher.messages) == count
    assert node.finish_run.fault == "executor_not_ready"


def test_bridge_prequeues_finish_forward_without_dwell(clock):
    node = start(clock)
    bridge = FakeBridge(advance_transition_dwells=False)
    current = node.publisher.messages[-1]
    bridge.navigation_command_callback(String(data=json.dumps(current)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == FinishRun.MOTION
    bridge.executor_status_callback(executor_status(**request, status="RUNNING"))
    status(node, current, "RUNNING")
    queued = node.publisher.messages[-1]
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 2
    assert all(r["motion_id"] == FinishRun.MOTION for r in requests)
    assert bridge.queued_request_id == queued["command_id"]
    assert not bridge.queued_request_deferred
    assert bridge.active_dwell_until is None and bridge.transition_dwell_until is None
    for _ in range(6):
        current_request, queued_request = requests[-2:]
        bridge.executor_status_callback(executor_status(**queued_request, status="QUEUED"))
        status(node, queued, "QUEUED")
        bridge.executor_status_callback(executor_status(**current_request, status="SUCCEEDED"))
        status(node, current, "SUCCEEDED")
        bridge.executor_status_callback(executor_status(**queued_request, status="RUNNING"))
        status(node, queued, "RUNNING")
        current, queued = queued, node.publisher.messages[-1]
        bridge.navigation_command_callback(String(data=json.dumps(queued)))
        requests = decoded_messages(bridge.executor_request_publisher)
        assert requests[-1]["motion_id"] == FinishRun.MOTION
        assert bridge.active_dwell_until is None and bridge.transition_dwell_until is None
        assert not bridge.queued_request_deferred
    assert len(requests) == 8


def test_finish_banner_distinguishes_running_and_stopped():
    from step.yolo26_detector import Yolo26Detector

    assert Yolo26Detector._finish_run_banner(None) is None
    assert "FORWARD 6 REPEAT" in Yolo26Detector._finish_run_banner({"finish_run": {"latched": True}})
    assert "STOPPED" in Yolo26Detector._finish_run_banner({
        "finish_run": {"latched": True, "fault": "CANCELLED"},
    })


def test_three_modes_allow_only_two_jump_executions():
    manager = MissionPhaseManager()
    assert manager.required_hurdles == 3
    manager.hurdles_executed = 2
    assert manager.execution_limit_reached("GO")
