"""Connect executor posture feedback to real mission observation callbacks."""

import json

from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from step.hurdle_camera_pose import HurdleCameraPose
from test_hurdle_positioning import HurdleHarness, clock, hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


def test_executor_feedback_selects_camera_angle_in_mission_observations(clock, monkeypatch):
    node = HurdleHarness()
    node.hurdle_camera_pose = HurdleCameraPose()
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda _: round(clock[0] * 1e9))
    heartbeat = {
        "sequence": 1, "auto_ready": True, "active": False,
        "hurdle_head_override_active": False,
        "ball_head_override_active": False, "goal_head_override_active": False,
    }
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))

    def decide():
        stamp_ns = round(clock[0] * 1e9)
        node.receive(hurdle(.6, stamp={"sec": stamp_ns // 1_000_000_000,
                                      "nanosec": stamp_ns % 1_000_000_000},
                            hurdle_angle_45_deg=30, hurdle_angle_0_deg=-30))
        observations, _ = node._fresh_observations(clock[0])
        observations["line"] = line_info(heading=-50)
        return node.planner.plan("HURDLE_POSITIONING", observations, .1)

    clock[0] += .1
    up = decide()
    assert up.action == "ALIGN_LEFT" and up.source_command["turn_count"] == 2
    heartbeat.update(sequence=2, hurdle_head_override_active=True,
                     hurdle_head_override_current_deg=-50)
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))
    clock[0] += .1
    assert decide().action == "WAIT"
    heartbeat.update(sequence=3, hurdle_head_override_current_deg=-64)
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps(heartbeat)))
    clock[0] += .1
    down = decide()
    assert down.action == "ALIGN_RIGHT" and down.source_command["turn_count"] == 3
    assert down.source_command["alignment_reference"] == "hurdle_angle_0"
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source="hurdle", action=down.action, source_command=down.source_command,
    ))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == "post_ball_line_turn_right_3"
