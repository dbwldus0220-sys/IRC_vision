"""
Contract-only skeleton for a future STEP SDK MotionPlayer backend.

This module deliberately imports no hardware or vendor SDK. The class is not
registered in the player factory and cannot start a motion.
"""

from typing import Optional

try:
    from .mock_motion_player import (
        CancelResult,
        MotionError,
        MotionStatus,
        StartResult,
    )
except ImportError:  # Allows direct unit-test imports.
    from mock_motion_player import (
        CancelResult,
        MotionError,
        MotionStatus,
        StartResult,
    )


class StepSdkMotionPlayer:
    """Define the adapter surface without connecting to the STEP SDK."""

    CONTRACT_MOTION_IDS = frozenset(
        {
            "forward",
            "forward_short",
            "turn_left",
            "turn_right",
            "adjust_left",
            "adjust_right",
            "backward",
            "pick_ball",
            "shoot",
            "hurdle",
        }
    )
    ERROR_MESSAGE = "STEP SDK backend is not implemented"

    def __init__(self) -> None:
        """Initialize a permanently disconnected contract skeleton."""
        self._last_result = MotionError.HARDWARE_NOT_READY
        self._last_error = self.ERROR_MESSAGE

    def hardwareReady(self) -> bool:
        """Report that no SDK initialization has been implemented."""
        return False

    def start(self, motion_id: str) -> StartResult:
        """Reject every start without importing or calling a hardware SDK."""
        if motion_id not in self.CONTRACT_MOTION_IDS:
            self._last_result = MotionError.NONE
            self._last_error = f"unsupported motion_id: {motion_id}"
            return StartResult.INVALID_MOTION

        self._last_result = MotionError.HARDWARE_NOT_READY
        self._last_error = self.ERROR_MESSAGE
        return StartResult.HARDWARE_NOT_READY

    def update(self) -> None:
        """Perform no polling because no SDK is connected."""
        return None

    def running(self) -> bool:
        """Report that no motion can be active."""
        return False

    def status(self) -> MotionStatus:
        """Remain idle because motion start is unavailable."""
        return MotionStatus.IDLE

    def succeeded(self) -> bool:
        """Report that no motion has completed."""
        return False

    def result(self) -> MotionError:
        """Return the last contract-level rejection detail."""
        return self._last_result

    def lastError(self) -> str:
        """Return a human-readable rejection reason."""
        return self._last_error

    def cancel(self) -> CancelResult:
        """Reject cancellation because no SDK motion can be active."""
        self._last_result = MotionError.HARDWARE_NOT_READY
        self._last_error = self.ERROR_MESSAGE
        return CancelResult.HARDWARE_NOT_READY

    def currentMotion(self) -> Optional[str]:
        """Return no active motion."""
        return None
