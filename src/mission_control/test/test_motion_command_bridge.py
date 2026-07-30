"""Tests for the navigation-to-Dynamics motion command bridge."""

import json
from types import MethodType

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from robot_msgs.msg import MotionEnd
from std_msgs.msg import String


class FakeLogger:
    """Provide no-op logger methods required by the bridge callbacks."""

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class CapturePublisher:
    """Store every ROS message published during a test."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeBridge:
    """Provide the state and methods used by bridge callbacks without ROS."""

    SPECIAL_ACTIONS = MotionCommandBridgeNode.SPECIAL_ACTIONS

    def __init__(self):
        self.last_sent_command_id = None
        self.motion_in_progress = False

        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_dynamics_command = None

        self.motion_command_publisher = CapturePublisher()
        self.motion_status_publisher = CapturePublisher()
        self.logger = FakeLogger()

        self.map_action_to_dynamics = MethodType(
            MotionCommandBridgeNode.map_action_to_dynamics,
            self,
        )
        self.publish_motion_status = MethodType(
            MotionCommandBridgeNode.publish_motion_status,
            self,
        )

    def get_logger(self):
        return self.logger


def status_payload(bridge):
    """Decode the most recently published /motion/status message."""
    return json.loads(bridge.motion_status_publisher.messages[-1].data)


def navigation_message(**overrides):
    """Create one valid navigation command message."""
    payload = {
        "command_id": 8000,
        "event_id": 8,
        "action": "STRAIGHT",
        "valid": True,
    }
    payload.update(overrides)

    message = String()
    message.data = json.dumps(payload)
    return message


def motion_end_message(command):
    """Create one completed Dynamics motion message."""
    message = MotionEnd()
    message.finished = True
    message.command = command
    message.motion_end_detect = True
    return message


def test_general_left_and_right_reuse_existing_turn_paths():
    bridge = FakeBridge()

    assert bridge.map_action_to_dynamics(
        {"action": "LEFT", "source_command": {"target_heading_change_deg": 8}}
    ) == (2, 8)
    assert bridge.map_action_to_dynamics(
        {
            "action": "RIGHT",
            "source_command": {"target_heading_change_deg": -9},
        }
    ) == (3, 9)


def test_fine_actions_have_no_dynamics_mapping():
    bridge = FakeBridge()

    for action in ("FINE_LEFT", "FINE_RIGHT"):
        assert bridge.map_action_to_dynamics({"action": action}) is None


def test_running_ignored_then_succeeded():
    """Keep an active motion until its matching completion arrives."""
    bridge = FakeBridge()

    MotionCommandBridgeNode.navigation_command_callback(
        bridge,
        navigation_message(),
    )

    assert len(bridge.motion_command_publisher.messages) == 1

    dynamics_message = bridge.motion_command_publisher.messages[0]
    assert dynamics_message.command == 1
    assert dynamics_message.angle == 0

    running = status_payload(bridge)
    assert running["status"] == "RUNNING"
    assert running["command_id"] == 8000
    assert running["event_id"] == 8
    assert running["action"] == "STRAIGHT"
    assert running["dynamics_command"] == 1
    assert running["motion_in_progress"] is True

    MotionCommandBridgeNode.motion_end_callback(
        bridge,
        motion_end_message(command=17),
    )

    ignored = status_payload(bridge)
    assert ignored["status"] == "IGNORED"
    assert ignored["command_id"] == 8000
    assert ignored["dynamics_command"] == 1
    assert ignored["motion_in_progress"] is True
    assert ignored["reason"] == (
        "motion_end_command_mismatch:received=17,expected=1"
    )

    assert bridge.motion_in_progress is True
    assert bridge.active_dynamics_command == 1

    MotionCommandBridgeNode.motion_end_callback(
        bridge,
        motion_end_message(command=1),
    )

    succeeded = status_payload(bridge)
    assert succeeded["status"] == "SUCCEEDED"
    assert succeeded["command_id"] == 8000
    assert succeeded["event_id"] == 8
    assert succeeded["action"] == "STRAIGHT"
    assert succeeded["dynamics_command"] == 1
    assert succeeded["motion_in_progress"] is False

    assert bridge.motion_in_progress is False
    assert bridge.active_command_id is None
    assert bridge.active_event_id is None
    assert bridge.active_action is None
    assert bridge.active_dynamics_command is None
