"""ROS-independent liveness watchdog for the production motion executor."""

from __future__ import annotations

from dataclasses import dataclass


EXECUTOR_HEARTBEAT_TIMEOUT = "EXECUTOR_HEARTBEAT_TIMEOUT"


@dataclass(frozen=True)
class HeartbeatTimeout:
    """Describe why the executor heartbeat watchdog expired."""

    error_code: str
    message: str
    executor_seen: bool


class ExecutorHeartbeatWatchdog:
    """Track startup grace and heartbeat loss using monotonic timestamps."""

    def __init__(
        self,
        *,
        started_at: float,
        startup_grace_sec: float,
        timeout_sec: float,
    ) -> None:
        self.started_at = float(started_at)
        self.startup_grace_sec = max(0.0, float(startup_grace_sec))
        self.timeout_sec = max(0.0, float(timeout_sec))
        self.executor_seen = False
        self.last_heartbeat_at: float | None = None
        self.last_sequence: int | None = None
        self._timeout_reported = False

    def observe(self, *, sequence: int, observed_at: float) -> None:
        """Record one valid heartbeat without clearing a reported timeout."""
        self.executor_seen = True
        self.last_heartbeat_at = float(observed_at)
        self.last_sequence = sequence

    def check(self, now: float) -> HeartbeatTimeout | None:
        """Return the first timeout event, distinguishing startup from loss."""
        if self._timeout_reported:
            return None

        current = float(now)
        if not self.executor_seen:
            if current - self.started_at <= self.startup_grace_sec:
                return None
            message = "executor heartbeat was not seen before startup grace expired"
        else:
            assert self.last_heartbeat_at is not None
            if current - self.last_heartbeat_at <= self.timeout_sec:
                return None
            message = "executor heartbeat stopped after initial liveness was observed"

        self._timeout_reported = True
        return HeartbeatTimeout(
            error_code=EXECUTOR_HEARTBEAT_TIMEOUT,
            message=message,
            executor_seen=self.executor_seen,
        )
