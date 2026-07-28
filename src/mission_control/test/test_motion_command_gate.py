import pytest

from mission_control.motion_command_gate import GeneralMotionCommandGate


def gate_with_vision():
    gate = GeneralMotionCommandGate()
    gate.on_new_vision_input()
    return gate


def test_first_left_publishes_once_and_repeated_left_is_blocked():
    gate = gate_with_vision()
    assert gate.can_publish("LEFT")
    gate.on_command_published("LEFT")
    gate.on_new_vision_input()
    assert not gate.can_publish("LEFT")


def test_right_is_blocked_while_left_is_running():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    gate.on_motion_status("TURN_LEFT", "RUNNING")
    assert not gate.can_publish("RIGHT")


def test_running_keeps_lock_with_left_alias():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    transition = gate.on_motion_status("TURN_LEFT", "RUNNING")
    assert transition.matched
    assert not transition.released
    assert gate.locked


@pytest.mark.parametrize(
    "status",
    ["SUCCEEDED", "FAILED", "TIMEOUT", "CANCELED", "CANCELLED"],
)
def test_terminal_requires_new_vision_before_republishing(status):
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    gate.on_motion_status("TURN_LEFT", "RUNNING")
    assert gate.on_motion_status("TURN_LEFT", status).released
    assert not gate.can_publish("LEFT")
    gate.on_new_vision_input()
    assert gate.can_publish("LEFT")


def test_rejected_does_not_retry_same_action_forever():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    assert gate.on_motion_status("TURN_LEFT", "REJECTED").released
    for _ in range(3):
        gate.on_new_vision_input()
        assert not gate.can_publish("LEFT")
    assert gate.can_publish("RIGHT")


def test_old_success_cannot_release_new_lock_without_running():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    transition = gate.on_motion_status("TURN_LEFT", "SUCCEEDED")
    assert not transition.matched
    assert not transition.released
    assert gate.locked


def test_mismatched_terminal_cannot_release_lock():
    gate = gate_with_vision()
    gate.on_command_published("RIGHT")
    gate.on_motion_status("TURN_RIGHT", "RUNNING")
    assert not gate.on_motion_status("TURN_LEFT", "SUCCEEDED").matched
    assert gate.locked


@pytest.mark.parametrize(
    ("command_action", "status_action"),
    [
        ("LEFT", "TURN_LEFT"),
        ("RIGHT", "TURN_RIGHT"),
        ("STRAIGHT", "STRAIGHT"),
    ],
)
def test_action_aliases_correlate(command_action, status_action):
    gate = gate_with_vision()
    assert gate.can_publish(command_action)
    gate.on_command_published(command_action)
    assert gate.on_motion_status(status_action, "RUNNING").matched


@pytest.mark.parametrize("action", ["WAIT", "STOP", "PICKUP_NOW"])
def test_non_general_action_is_not_managed_as_execution(action):
    assert not gate_with_vision().can_publish(action)
