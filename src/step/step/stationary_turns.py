"""Shared error-angle bins for stationary turns; bounds are inclusive."""

import math


STATIONARY_TURN_BINS = {
    "LEFT": ((8.0, 1), (30.0, 2), (45.0, 3), (60.0, 4), (75.0, 5), (90.0, 6)),
    "RIGHT": ((15.0, 1), (30.0, 3), (45.0, 5), (60.0, 6), (75.0, 7), (90.0, 9)),
}
STATIONARY_TURN_MIN_DEG = {side: bins[0][0] for side, bins in STATIONARY_TURN_BINS.items()}


def stationary_turn_repeat_count(angle_deg: float, direction: str) -> int:
    """Select repetitions from the observed error, not from estimated physical yaw."""
    if not math.isfinite(angle_deg):
        return 0
    for lower_bound, count in reversed(STATIONARY_TURN_BINS[direction]):
        if abs(angle_deg) >= lower_bound:
            return count
    return 0
