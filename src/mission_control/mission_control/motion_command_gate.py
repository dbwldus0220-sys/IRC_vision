"""ROS-free state for suppressing duplicate general motion commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


GENERAL_ACTIONS = frozenset({"STRAIGHT", "LEFT", "RIGHT"})
UNSUPPORTED_GENERAL_ACTIONS = frozenset({"FINE_LEFT", "FINE_RIGHT"})
TERMINAL_STATUSES = frozenset(
    {
        "SUCCEEDED",
        "FAILED",
        "TIMEOUT",
        "REJECTED",
        "CANCELED",
        "CANCELLED",
    }
)
ACTION_ALIASES = {
    "STRAIGHT": "STRAIGHT",
    "LEFT": "LEFT",
    "TURN_LEFT": "LEFT",
    "RIGHT": "RIGHT",
    "TURN_RIGHT": "RIGHT",
}


def normalize_general_action(action: Any) -> str | None:
    """Normalize only action names used by the general-motion path."""
    if not isinstance(action, str):
        return None
    normalized = action.strip().upper()
    if normalized in UNSUPPORTED_GENERAL_ACTIONS:
        return None
    return ACTION_ALIASES.get(normalized)


@dataclass(frozen=True)
class GateTransition:
    """Describe whether one status matched and released the active lock."""

    matched: bool
    released: bool


class GeneralMotionCommandGate:
    """Allow one general command until its matching execution terminates."""

    def __init__(self) -> None:
        self.locked = False
        self.active_action: str | None = None
        self.active_command_id: int | None = None
        self.running_seen = False
        self.vision_generation = 0
        self.required_vision_generation = 0
        self.rejected_action: str | None = None

    def on_new_vision_input(self) -> None:
        """Record one valid Vision JSON message."""
        self.vision_generation += 1

    def can_publish(self, action: Any) -> bool:
        """Return whether a general action may be published now."""
        normalized = normalize_general_action(action)
        if normalized is None or self.locked:
            return False
        if self.vision_generation < self.required_vision_generation:
            return False
        if self.rejected_action == normalized:
            return False
        if (
            self.rejected_action is not None
            and self.rejected_action != normalized
        ):
            self.rejected_action = None
        return True

    def on_command_published(
        self, action: Any, command_id: int | None = None
    ) -> None:
        """Lock immediately after publishing one general command."""
        normalized = normalize_general_action(action)
        if normalized is None:
            raise ValueError("unsupported general motion action")
        if self.locked:
            raise RuntimeError("general motion command is already locked")
        self.locked = True
        self.active_action = normalized
        self.active_command_id = command_id
        self.running_seen = False

    def on_motion_status(
        self,
        action: Any,
        status: Any,
        command_id: Any = None,
    ) -> GateTransition:
        """Apply a matching status while protecting against stale terminals."""
        normalized_action = normalize_general_action(action)
        normalized_status = (
            status.strip().upper() if isinstance(status, str) else ""
        )
        if (
            not self.locked
            or normalized_action is None
            or normalized_action != self.active_action
        ):
            return GateTransition(False, False)

        # New messages correlate by mission command ID. If either side lacks
        # one, retain the legacy action/RUNNING compatibility path below.
        if (
            self.active_command_id is not None
            and command_id is not None
            and command_id != self.active_command_id
        ):
            return GateTransition(False, False)

        if normalized_status == "RUNNING":
            self.running_seen = True
            return GateTransition(True, False)

        if normalized_status not in TERMINAL_STATUSES:
            return GateTransition(False, False)

        # Accepted Executor requests publish RUNNING before their terminal
        # result. Requiring it prevents an old same-action terminal message
        # from releasing a newly-created lock. REJECTED has no RUNNING.
        if normalized_status != "REJECTED" and not self.running_seen:
            return GateTransition(False, False)

        completed_action = self.active_action
        self.locked = False
        self.active_action = None
        self.active_command_id = None
        self.running_seen = False
        self.required_vision_generation = self.vision_generation + 1

        # A busy rejection must not be retried by every repeated frame.
        if normalized_status == "REJECTED":
            self.rejected_action = completed_action

        return GateTransition(True, True)
