"""ROS-independent mission phase and special-motion progress state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MotionStatusResult:
    """Describe how one motion status affected mission state."""

    handled: bool
    terminal: bool
    duplicate: bool
    reason: str
    previous_phase: str
    current_phase: str


class MissionPhaseManager:
    """Own mission progress and special-motion phase transitions."""

    ALLOWED_PHASES = frozenset(
        {
            "AUTO",
            "BALL_SEARCH",
            "BALL_APPROACH",
            "GOAL_SEARCH",
            "GOAL_APPROACH",
            "HURDLE_APPROACH",
            "LINE_TRACK",
            "FINISH",
            "WALK_TO_FINISH",
            "FINISHED",
        }
    )
    SPECIAL_ACTIONS = frozenset(
        {"PICKUP_NOW", "SHOT", "GO", "CROSS_FINISH"}
    )
    SUPPORTED_STATUSES = frozenset(
        {
            "RUNNING",
            "SUCCEEDED",
            "FAILED",
            "TIMEOUT",
            "CANCELLED",
            "REJECTED",
        }
    )
    TERMINAL_STATUSES = frozenset(
        {"SUCCEEDED", "FAILED", "TIMEOUT", "CANCELLED", "REJECTED"}
    )

    def __init__(
        self,
        *,
        required_pickups: int = 2,
        required_shots: int = 2,
        required_ball_sections: int = 2,
        initial_phase: str = "AUTO",
    ) -> None:
        """Initialize bounded progress counters and one validated phase."""
        self.required_pickups = self._nonnegative_count(required_pickups)
        self.required_shots = self._nonnegative_count(required_shots)
        self.required_ball_sections = self._nonnegative_count(
            required_ball_sections
        )

        normalized_phase = self._normalize_phase(initial_phase)
        if normalized_phase is None:
            raise ValueError("initial_phase must be a supported phase")

        self.current_phase = normalized_phase
        self.pickups_completed = 0
        self.shots_completed = 0
        self.ball_sections_processed = 0
        self.finish_enabled = self.required_ball_sections == 0
        self.mission_complete = False

        self.active_special_action: str | None = None
        self.active_special_command_id: int | None = None
        self.active_special_running = False
        self._active_special_origin_phase: str | None = None
        self._completed_command_ids: set[int] = set()

    @staticmethod
    def _nonnegative_count(value: Any) -> int:
        """Match the existing node policy by clamping integer counts to zero."""
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("required counts must be integers")
        return max(0, value)

    @classmethod
    def _normalize_phase(cls, phase: Any) -> str | None:
        if not isinstance(phase, str):
            return None
        normalized = phase.strip().upper()
        return normalized if normalized in cls.ALLOWED_PHASES else None

    @classmethod
    def _normalize_action(cls, action: Any) -> str | None:
        if not isinstance(action, str):
            return None
        normalized = action.strip().upper()
        return normalized if normalized in cls.SPECIAL_ACTIONS else None

    @staticmethod
    def _valid_command_id(command_id: Any) -> bool:
        return (
            isinstance(command_id, int)
            and not isinstance(command_id, bool)
            and command_id > 0
        )

    def set_phase(self, phase: str) -> bool:
        """Set one permanent supported phase; dynamic lock phases are rejected."""
        normalized = self._normalize_phase(phase)
        if normalized is None:
            return False
        self.current_phase = normalized
        return True

    def start_special_action(self, action: str, command_id: int) -> bool:
        """Register one special command before its RUNNING status arrives."""
        normalized_action = self._normalize_action(action)
        if normalized_action is None or not self._valid_command_id(command_id):
            return False
        if command_id in self._completed_command_ids:
            return False
        if self.active_special_command_id is not None:
            return (
                self.active_special_action == normalized_action
                and self.active_special_command_id == command_id
            )

        self.active_special_action = normalized_action
        self.active_special_command_id = command_id
        self.active_special_running = False
        self._active_special_origin_phase = self.current_phase
        return True

    def handle_motion_status(
        self,
        action: str,
        command_id: int,
        status: str,
    ) -> MotionStatusResult:
        """Apply one correlated special-motion status."""
        previous_phase = self.current_phase
        normalized_action = self._normalize_action(action)
        normalized_status = (
            status.strip().upper() if isinstance(status, str) else ""
        )

        if normalized_status not in self.SUPPORTED_STATUSES:
            return self._result(
                False, False, False, "unsupported_status", previous_phase
            )
        if not self._valid_command_id(command_id):
            return self._result(
                False, False, False, "invalid_command_id", previous_phase
            )
        if (
            normalized_status in self.TERMINAL_STATUSES
            and command_id in self._completed_command_ids
        ):
            return self._result(
                False, True, True, "duplicate_terminal", previous_phase
            )
        if (
            normalized_action is None
            or normalized_action != self.active_special_action
            or command_id != self.active_special_command_id
        ):
            return self._result(
                False, False, False, "active_command_mismatch", previous_phase
            )

        if normalized_status == "RUNNING":
            self.active_special_running = True
            return self._result(
                True, False, False, "running", previous_phase
            )

        if (
            normalized_status != "REJECTED"
            and not self.active_special_running
        ):
            return self._result(
                False,
                True,
                False,
                "terminal_before_running",
                previous_phase,
            )

        self._apply_terminal(normalized_action, normalized_status)
        self._completed_command_ids.add(command_id)
        self.active_special_action = None
        self.active_special_command_id = None
        self.active_special_running = False
        self._active_special_origin_phase = None
        return self._result(
            True, True, False, "terminal_applied", previous_phase
        )

    def _apply_terminal(self, action: str, status: str) -> None:
        succeeded = status == "SUCCEEDED"

        if action == "PICKUP_NOW":
            if succeeded:
                self.pickups_completed = min(
                    self.pickups_completed + 1,
                    self.required_pickups,
                )
                self.current_phase = "GOAL_APPROACH"
                return

            self.ball_sections_processed = min(
                self.ball_sections_processed + 1,
                self.required_ball_sections,
            )
            self._update_finish_enabled()
            self.current_phase = "AUTO"
            return

        if action == "SHOT":
            if succeeded:
                self.shots_completed = min(
                    self.shots_completed + 1,
                    self.required_shots,
                )
            self.ball_sections_processed = min(
                self.ball_sections_processed + 1,
                self.required_ball_sections,
            )
            self._update_finish_enabled()
            self.current_phase = "AUTO"
            return

        if action == "GO":
            self.current_phase = (
                "GOAL_APPROACH"
                if self._active_special_origin_phase == "GOAL_APPROACH"
                else "AUTO"
            )
            return

        if succeeded:
            self.mission_complete = True
            self.current_phase = "FINISHED"
        else:
            self.mission_complete = False
            self.current_phase = "WALK_TO_FINISH"

    def _update_finish_enabled(self) -> None:
        self.finish_enabled = (
            self.ball_sections_processed >= self.required_ball_sections
        )

    def _result(
        self,
        handled: bool,
        terminal: bool,
        duplicate: bool,
        reason: str,
        previous_phase: str,
    ) -> MotionStatusResult:
        return MotionStatusResult(
            handled=handled,
            terminal=terminal,
            duplicate=duplicate,
            reason=reason,
            previous_phase=previous_phase,
            current_phase=self.current_phase,
        )

    def snapshot(self) -> dict[str, int | bool | str | None]:
        """Return a new dictionary containing the externally visible state."""
        return {
            "current_phase": self.current_phase,
            "pickups_completed": self.pickups_completed,
            "shots_completed": self.shots_completed,
            "ball_sections_processed": self.ball_sections_processed,
            "finish_enabled": self.finish_enabled,
            "mission_complete": self.mission_complete,
            "required_pickups": self.required_pickups,
            "required_shots": self.required_shots,
            "required_ball_sections": self.required_ball_sections,
            "active_special_action": self.active_special_action,
            "active_special_command_id": self.active_special_command_id,
            "active_special_running": self.active_special_running,
        }
