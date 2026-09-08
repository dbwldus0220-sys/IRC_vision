"""Hardware-independent tests for the dual RealSense RGB selector."""

import pytest

from step.dual_realsense_selector_test import CameraSelectorState
from step.dual_realsense_selector_test import normalize_camera_name


def test_normalizes_camera_selection_command():
    """Accept whitespace and case without creating extra camera names."""
    assert normalize_camera_name(" FRONT ") == "front"
    assert normalize_camera_name("Down") == "down"


def test_rejects_unknown_camera_selection_command():
    """Reject invalid commands instead of silently switching sources."""
    with pytest.raises(ValueError, match="front, down"):
        normalize_camera_name("side")


def test_initial_camera_controls_forwarding():
    """Forward only frames belonging to the initial source."""
    state = CameraSelectorState("front")

    assert state.should_forward("front") is True
    assert state.should_forward("down") is False


def test_switch_changes_the_forwarded_source():
    """Switch sources without stopping or restarting either camera."""
    state = CameraSelectorState("front")

    assert state.select("down") is True
    assert state.active_camera == "down"
    assert state.should_forward("front") is False
    assert state.should_forward("down") is True
    assert state.select("down") is False


def test_invalid_switch_preserves_the_active_camera():
    """Keep the current source when a malformed command is received."""
    state = CameraSelectorState("front")

    with pytest.raises(ValueError):
        state.select("invalid")

    assert state.active_camera == "front"
