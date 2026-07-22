"""Tests for special-motion state handling in the motion decision node."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode


class FakeLogger:
    """Provide no-op logger methods used by the callback."""

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class FakeDecisionNode:
    """Provide only the state required by the motion-status callback."""

    SPECIAL_ACTIONS = MotionDecisionNode.SPECIAL_ACTIONS
    SPECIAL_COMPLETION_PHASES = (
        MotionDecisionNode.SPECIAL_COMPLETION_PHASES
    )

    def __init__(self, mission_phase="AUTO"):
        self.mission_phase = mission_phase
        self.terminal_latch = ("test", "terminal")

        self.special_motion_running = False
        self.active_special_action = None
        self.active_special_command_id = None
        self.active_special_event_id = None
        self.active_special_dynamics_command = None

        self.logger = FakeLogger()

    def get_logger(self):
        """Return the fake logger."""
        return self.logger


def status_message(
    *,
    status,
    action,
    command_id,
    event_id,
    dynamics_command,
):
    """Create one /motion/status JSON message."""
    payload = {
        "status": status,
        "action": action,
        "command_id": command_id,
        "event_id": event_id,
        "dynamics_command": dynamics_command,
    }

    message = String()
    message.data = json.dumps(payload)
    return message


def send_status(node, **kwargs):
    """Call the real motion-status callback using a fake node."""
    MotionDecisionNode._motion_status_callback(
        node,
        status_message(**kwargs),
    )


@pytest.mark.parametrize(
    (
        "initial_phase",
        "action",
        "dynamics_command",
        "expected_phase",
    ),
    [
        ("AUTO", "PICKUP_NOW", 9, "GOAL_APPROACH"),
        ("GOAL_APPROACH", "SHOT", 17, "AUTO"),
        ("HURDLE_APPROACH", "GO", 14, "AUTO"),
    ],
)
def test_special_motion_success_advances_phase(
    initial_phase,
    action,
    dynamics_command,
    expected_phase,
):
    """Advance to the configured phase after a matching success."""
    node = FakeDecisionNode(initial_phase)

    send_status(
        node,
        status="RUNNING",
        action=action,
        command_id=100,
        event_id=10,
        dynamics_command=dynamics_command,
    )

    assert node.special_motion_running is True
    assert node.active_special_action == action
    assert node.mission_phase == initial_phase

    send_status(
        node,
        status="SUCCEEDED",
        action=action,
        command_id=100,
        event_id=10,
        dynamics_command=dynamics_command,
    )

    assert node.special_motion_running is False
    assert node.mission_phase == expected_phase
    assert node.terminal_latch is None

    assert node.active_special_action is None
    assert node.active_special_command_id is None
    assert node.active_special_event_id is None
    assert node.active_special_dynamics_command is None


def test_ignored_status_keeps_special_motion_locked():
    """Keep the active phase and lock after an ignored status."""
    node = FakeDecisionNode("AUTO")

    send_status(
        node,
        status="RUNNING",
        action="PICKUP_NOW",
        command_id=200,
        event_id=20,
        dynamics_command=9,
    )

    send_status(
        node,
        status="IGNORED",
        action="PICKUP_NOW",
        command_id=200,
        event_id=20,
        dynamics_command=9,
    )

    assert node.special_motion_running is True
    assert node.active_special_action == "PICKUP_NOW"
    assert node.mission_phase == "AUTO"
    assert node.terminal_latch == ("test", "terminal")


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT"])
def test_failed_or_timed_out_special_motion_returns_to_auto(status):
    """Release the lock and return to AUTO after failure or timeout."""
    node = FakeDecisionNode("GOAL_APPROACH")

    send_status(
        node,
        status="RUNNING",
        action="SHOT",
        command_id=300,
        event_id=30,
        dynamics_command=17,
    )

    send_status(
        node,
        status=status,
        action="SHOT",
        command_id=300,
        event_id=30,
        dynamics_command=17,
    )

    assert node.special_motion_running is False
    assert node.mission_phase == "AUTO"
    assert node.terminal_latch is None