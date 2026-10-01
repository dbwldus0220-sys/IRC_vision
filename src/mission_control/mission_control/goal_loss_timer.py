"""Measure Goal loss from distinct camera frames, independently of motion time."""

from dataclasses import dataclass


@dataclass
class GoalLossTimer:
    max_gap_sec: float = 0.5
    started_at: float | None = None
    started_stamp_ns: int | None = None
    last_observed_at: float | None = None
    last_stamp_ns: int | None = None
    elapsed_sec: float = 0.0

    def reset(self) -> None:
        """Discard loss evidence while retaining the consumed-frame watermark."""
        self.started_at = None
        self.started_stamp_ns = None
        self.elapsed_sec = 0.0

    def observe(self, info: dict | None, now: float) -> None:
        """Count only continuous fresh misses in both capture and receive time."""
        if self.last_observed_at is not None and not (
            0.0 <= now - self.last_observed_at <= self.max_gap_sec
        ):
            self.reset()
        stamp = info.get("rgb_stamp_ns") if info is not None else None
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
            self.reset()
            return
        if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            return
        if self.last_stamp_ns is not None and (
            stamp - self.last_stamp_ns > self.max_gap_sec * 1e9
        ):
            self.reset()
        self.last_stamp_ns = stamp
        self.last_observed_at = now
        if info.get("detected") is not False or info.get("raw_detected") is True:
            self.reset()
            return
        if self.started_at is None:
            self.started_at = now
            self.started_stamp_ns = stamp
        self.elapsed_sec = min(
            now - self.started_at, (stamp - self.started_stamp_ns) / 1e9,
        )
