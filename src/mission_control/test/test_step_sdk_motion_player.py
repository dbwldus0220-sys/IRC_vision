"""Contract tests for the disconnected STEP SDK MotionPlayer skeleton."""

import builtins

from mission_control.mock_motion_player import (
    CancelResult,
    MotionError,
    MotionStatus,
    StartResult,
)
from mission_control.motion_player_factory import create_motion_player
from mission_control.motion_player_protocol import MotionPlayerProtocol
from mission_control.sdk_motion_player_placeholder import (
    SdkMotionPlayerPlaceholder,
)
from mission_control.step_sdk_motion_player import StepSdkMotionPlayer


def test_skeleton_implements_motion_player_protocol():
    player = StepSdkMotionPlayer()

    assert isinstance(player, MotionPlayerProtocol)


def test_factory_sdk_selection_remains_the_safe_placeholder():
    player = create_motion_player("sdk")

    assert isinstance(player, SdkMotionPlayerPlaceholder)
    assert not isinstance(player, StepSdkMotionPlayer)


def test_known_motion_is_rejected_as_hardware_not_ready():
    player = StepSdkMotionPlayer()

    assert player.hardwareReady() is False
    assert player.start("forward") is StartResult.HARDWARE_NOT_READY
    assert player.status() is MotionStatus.IDLE
    assert player.running() is False
    assert player.currentMotion() is None
    assert player.result() is MotionError.HARDWARE_NOT_READY


def test_unsupported_motion_is_explicitly_rejected():
    player = StepSdkMotionPlayer()

    assert player.start("not_a_motion") is StartResult.INVALID_MOTION
    assert player.status() is MotionStatus.IDLE
    assert player.succeeded() is False
    assert player.result() is MotionError.NONE
    assert "unsupported motion_id" in player.lastError()


def test_repeated_start_never_creates_an_active_motion():
    player = StepSdkMotionPlayer()

    assert player.start("shoot") is StartResult.HARDWARE_NOT_READY
    assert player.start("shoot") is StartResult.HARDWARE_NOT_READY
    assert player.running() is False
    assert player.currentMotion() is None


def test_cancel_is_not_stop_action_and_remains_unavailable():
    player = StepSdkMotionPlayer()

    assert player.cancel() is CancelResult.HARDWARE_NOT_READY
    assert player.running() is False


def test_skeleton_does_not_open_files_or_import_hardware(monkeypatch):
    def fail_open(*args, **kwargs):
        del args, kwargs
        raise AssertionError("SDK skeleton must not open files or devices")

    monkeypatch.setattr(builtins, "open", fail_open)
    player = StepSdkMotionPlayer()

    assert player.start("pick_ball") is StartResult.HARDWARE_NOT_READY
    player.update()
