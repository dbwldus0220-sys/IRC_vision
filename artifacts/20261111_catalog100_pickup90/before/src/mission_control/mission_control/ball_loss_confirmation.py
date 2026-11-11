"""Confirm Ball loss using distinct camera frames and continuous observation."""

from dataclasses import dataclass
from typing import Any


@dataclass
class BallLossConfirmation:
    min_duration_sec: float = 0.5
    min_frames: int = 5
    last_stamp_ns: int | None = None
    last_received_at: float | None = None
    first_missing_stamp_ns: int | None = None
    first_missing_at: float | None = None
    missing_frames: int = 0
    confirmed: bool = False

    def reset_missing(self) -> None:
        """Keep the frame watermark while clearing the loss evidence."""
        self.first_missing_stamp_ns = None
        self.first_missing_at = None
        self.missing_frames = 0
        self.confirmed = False

    def update(
        self, info: dict[str, Any], received_at: float, max_gap_sec: float,
    ) -> bool:
        """Return False for duplicate/old frames; never count an unstamped miss."""
        stamp = info.get("rgb_stamp_ns")
        valid_stamp = isinstance(stamp, int) and not isinstance(stamp, bool) and stamp > 0
        if valid_stamp and self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            return False

        # Neither a camera outage nor a delayed burst proves continuous loss.
        if self.last_received_at is not None and (
            received_at - self.last_received_at > max_gap_sec
            or (
                valid_stamp and self.last_stamp_ns is not None
                and (stamp - self.last_stamp_ns) / 1e9 > max_gap_sec
            )
        ):
            self.reset_missing()
        self.last_received_at = received_at
        if valid_stamp:
            self.last_stamp_ns = stamp

        if info.get("detected") is True or info.get("raw_detected") is True:
            self.reset_missing()
        elif not valid_stamp or info.get("detected") is not False:
            self.reset_missing()
        else:
            if self.first_missing_at is None:
                self.first_missing_at = received_at
                self.first_missing_stamp_ns = stamp
            self.missing_frames += 1
            self.confirmed = (
                self.missing_frames >= self.min_frames
                and received_at - self.first_missing_at >= self.min_duration_sec
                and stamp - self.first_missing_stamp_ns >= round(self.min_duration_sec * 1e9)
            )
        return True
