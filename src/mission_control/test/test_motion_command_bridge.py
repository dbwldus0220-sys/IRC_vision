"""Tests for the navigation-to-C++-executor motion bridge."""

import inspect
import json
from types import MethodType

import pytest

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from std_msgs.msg import String


class FakeLogger:
    """Provide logger methods required by callbacks."""

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class CapturePublisher:
    """Capture published ROS messages."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeBridge:
    """Provide bridge state without constructing a ROS node."""

    ACTION_TO_MOTION_ID = MotionCommandBridgeNode.ACTION_TO_MOTION_ID
    TERMINAL_STATUSES = MotionCommandBridgeNode.TERMINAL_STATUSES
    DEFAULT_TIMEOUT_MS = MotionCommandBridgeNode.DEFAULT_TIMEOUT_MS

    def __init__(self):
        self.last_sent_command_id = None
        self.motion_in_progress = False
        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_request_id = None
        self.active_motion_id = None
        self.queued_command_id = None
        self.queued_event_id = None
        self.queued_action = None
        self.queued_request_id = None
        self.queued_motion_id = None
        self.executor_request_publisher = CapturePublisher()
        self.motion_status_publisher = CapturePublisher()
        self.logger = FakeLogger()

        for name in (
            "publish_motion_status",
            "_publish_local_rejection",
            "navigation_command_callback",
            "_valid_executor_status",
            "_clear_active_request",
            "_promote_queued_request",
            "executor_status_callback",
        ):
            setattr(
                self,
                name,
                MethodType(getattr(MotionCommandBridgeNode, name), self),
            )

    @staticmethod
    def _is_integer(value):
        return MotionCommandBridgeNode._is_integer(value)

    def motion_id_for_action(self, action):
        return self.ACTION_TO_MOTION_ID.get(action)

    def timeout_ms_from_payload(self, payload):
        return MotionCommandBridgeNode.timeout_ms_from_payload(payload)

    def get_logger(self):
        return self.logger


def string_message(payload):
    """Create a String message containing JSON or raw text."""
    message = String()
    message.data = payload if isinstance(payload, str) else json.dumps(payload)
    return message


def navigation_message(**overrides):
    """Create one valid navigation command."""
    payload = {
        "command_id": 8000,
        "event_id": 8,
        "action": "STRAIGHT",
        "valid": True,
    }
    payload.update(overrides)
    return string_message(payload)


def executor_status(**overrides):
    """Create one valid executor status."""
    payload = {
        "status": "RUNNING",
        "command_id": 8000,
        "event_id": 8,
        "request_id": 8000,
        "motion_id": "forward",
        "error_code": "",
        "message": "",
    }
    payload.update(overrides)
    return string_message(payload)


def decoded_messages(publisher):
    """Decode all captured String messages."""
    return [json.loads(message.data) for message in publisher.messages]


EXPECTED_PRODUCTION_ACTIONS = {
    "STRAIGHT": "line_forward_6",
    "STRAIGHT_1": "line_forward_2",
    "STRAIGHT_2": "line_forward_4",
    "STRAIGHT_3": "line_forward_6",
    "STRAIGHT_4": "line_forward_8",
    "STRAIGHT_5": "line_forward_10",
    "APPROACH": "forward",
    "LEFT": "line_turn_left_15",
    "RIGHT": "line_turn_right_large",
    "PICKUP_NOW": "pickup",
    "GO": "hurdle",
    "TURN_LEFT": "stationary_turn_left",
    "TURN_RIGHT": "stationary_turn_right",
    "ALIGN_LEFT": "stationary_turn_left",
    "ALIGN_RIGHT": "stationary_turn_right",
}
for recovery_side in ("LEFT", "RIGHT"):
    for suffix, motion_id in {
        2: "line_turn_left_4",
        4: "line_recovery_left_4",
        5: "line_recovery_left_5",
        6: "line_recovery_left_6",
        7: "line_recovery_left_7",
        8: "line_recovery_left_8",
        10: "line_turn_left_12",
        13: "line_turn_left_15",
    }.items():
        EXPECTED_PRODUCTION_ACTIONS[
            f"RECOVER_{recovery_side}_TURN_LEFT_{suffix}"
        ] = motion_id
    for suffix, motion_id in {
        4: "line_recovery_right_4",
        5: "line_recovery_right_5",
        6: "line_recovery_right_6",
        7: "line_recovery_right_7",
        8: "line_recovery_right_8",
        10: "line_turn_right_8",
        12: "line_turn_right_10",
        15: "line_turn_right_large",
    }.items():
        EXPECTED_PRODUCTION_ACTIONS[
            f"RECOVER_{recovery_side}_TURN_RIGHT_{suffix}"
        ] = motion_id


def test_production_action_mapping_contains_only_approved_contract():
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID == (
        EXPECTED_PRODUCTION_ACTIONS
    )


@pytest.mark.parametrize(
    ("action", "motion_id"),
    sorted(EXPECTED_PRODUCTION_ACTIONS.items()),
)
def test_supported_action_builds_executor_request(action, motion_id):
    bridge = FakeBridge()

    bridge.navigation_command_callback(navigation_message(action=action))

    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests == [
        {
            "action": action,
            "command_id": 8000,
            "event_id": 8,
            "request_id": 8000,
            "motion_id": motion_id,
            "timeout_ms": 12000,
        }
    ]
    assert bridge.motion_in_progress is True
    assert bridge.active_command_id == 8000
    assert bridge.active_event_id == 8
    assert bridge.active_action == action
    assert bridge.active_request_id == 8000
    assert bridge.active_motion_id == motion_id


@pytest.mark.parametrize("timeout", [0, -1, "100", True, None])
def test_invalid_timeout_uses_default(timeout):
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(timeout_ms=timeout)
    )

    request = decoded_messages(bridge.executor_request_publisher)[0]
    assert request["timeout_ms"] == 12000


def test_positive_source_timeout_is_preserved():
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(source_command={"timeout_ms": 2500})
    )

    request = decoded_messages(bridge.executor_request_publisher)[0]
    assert request["timeout_ms"] == 2500


@pytest.mark.parametrize(
    "action",
    [
        "RETREAT_GOAL",
        "SLOW_APPROACH",
        "FINE_FORWARD_STEP",
        "APPROACH_GOAL",
        "APPROACH_HURDLE",
        "SHOT",
        "CROSS_FINISH",
        "STOP",
        "WAIT",
        "BALL_LOST_STOP",
        "GOAL_LOST_STOP",
        "HEAD_SCAN_LEFT",
        "HEAD_SCAN_RIGHT",
        "HEAD_CENTER",
        "RECOVER_GOAL_TURN_LEFT",
        "RECOVER_GOAL_TURN_RIGHT",
        "WAIT_SCORE_CONFIRMATION",
        "WAIT_GO_CONFIRMATION",
        "UNKNOWN",
    ],
)
def test_unsupported_action_does_not_publish_executor_request(action):
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(action=action, sdk_motion_requested=True)
    )

    assert bridge.executor_request_publisher.messages == []
    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == "UNSUPPORTED"
    assert status["error_code"] == "UNSUPPORTED_ACTION"
    assert status["action"] == action


def test_duplicate_command_id_is_rejected():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())
    bridge.motion_in_progress = False

    bridge.navigation_command_callback(navigation_message())

    assert len(bridge.executor_request_publisher.messages) == 1
    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == "REJECTED"
    assert status["error_code"] == "DUPLICATE_COMMAND_ID"


def test_new_command_while_running_is_queued():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.navigation_command_callback(
        navigation_message(command_id=8001, event_id=9)
    )

    assert len(bridge.executor_request_publisher.messages) == 2
    assert bridge.queued_request_id == 8001
    assert bridge.queued_action == "STRAIGHT"


def test_running_status_keeps_lock_and_preserves_fields():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(executor_status(message="executing"))

    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status == {
        "status": "RUNNING",
        "command_id": 8000,
        "event_id": 8,
        "request_id": 8000,
        "motion_id": "forward",
        "error_code": "",
        "message": "executing",
        "action": "STRAIGHT",
        "source_node": "motion_command_bridge_node",
        "motion_in_progress": True,
    }
    assert bridge.motion_in_progress is True
    assert bridge.active_request_id == 8000


@pytest.mark.parametrize(
    ("terminal_status", "error_code"),
    [
        ("SUCCEEDED", ""),
        ("FAILED", "COMMUNICATION_ERROR"),
        ("REJECTED", "INVALID_MOTION"),
        ("CANCELLED", ""),
    ],
)
def test_terminal_status_releases_lock(terminal_status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(
        executor_status(
            status=terminal_status,
            error_code=error_code,
            message="executor detail",
        )
    )

    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == terminal_status
    assert status["error_code"] == error_code
    assert status["message"] == "executor detail"
    assert status["action"] == "STRAIGHT"
    assert status["motion_in_progress"] is False
    assert bridge.motion_in_progress is False
    assert bridge.active_command_id is None
    assert bridge.active_event_id is None
    assert bridge.active_action is None
    assert bridge.active_request_id is None
    assert bridge.active_motion_id is None


def test_mismatched_request_id_is_ignored():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(executor_status(request_id=9000))

    assert bridge.motion_status_publisher.messages == []
    assert bridge.motion_in_progress is True


@pytest.mark.parametrize(
    "payload",
    [
        "not-json",
        [],
        {"status": "RUNNING"},
        {
            "status": "UNKNOWN",
            "command_id": 8000,
            "event_id": 8,
            "request_id": 8000,
            "motion_id": "forward",
            "error_code": "",
            "message": "",
        },
    ],
)
def test_malformed_executor_status_is_ignored(payload):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(string_message(payload))

    assert bridge.motion_status_publisher.messages == []
    assert bridge.motion_in_progress is True


def test_bridge_source_has_no_robot_msgs_or_dynamics_interface():
    source = inspect.getsource(MotionCommandBridgeNode)

    assert "robot_msgs" not in source
    assert '"/motion_command"' not in source
    assert "/motion_end" not in source
    assert "active_dynamics_command" not in source
