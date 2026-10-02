"""Shared distance bands for discrete straight walking motions."""

from __future__ import annotations

import math


# Measured distance bands for the retained STRAIGHT_0..4 actions.
# STRAIGHT_0 is deliberately left to the behavior layer: it may mean hold,
# one micro-step, or a small retreat depending on the active mission.
APPROACH_DISTANCE_LIMITS_M = (
    0.130,
    0.263,
    0.427,
    0.564,
    0.680,
)


def approach_motion_for_distance(distance_m: float | None) -> str:
    """Return STRAIGHT_0..4, or generic STRAIGHT outside known range."""
    if distance_m is None or isinstance(distance_m, bool):
        return "STRAIGHT"
    try:
        distance = float(distance_m)
    except (TypeError, ValueError):
        return "STRAIGHT"
    if not math.isfinite(distance) or distance < 0.0:
        return "STRAIGHT"
    for level, upper_bound in enumerate(APPROACH_DISTANCE_LIMITS_M):
        if distance <= upper_bound + 1e-9:
            return f"STRAIGHT_{level}"
    return "STRAIGHT"


BALL_HURDLE_FINE_DISTANCE_M = 0.550
HURDLE_FINE_DISTANCE_M = 0.700


def ball_hurdle_approach_motion(
    distance_m: float | None,
    *,
    fine_distance_m: float = BALL_HURDLE_FINE_DISTANCE_M,
) -> str:
    """Select camera-45 approach; callers must validate distance before moving."""
    if distance_m is None or isinstance(distance_m, bool):
        return "STRAIGHT"
    try:
        distance = float(distance_m)
    except (TypeError, ValueError):
        return "STRAIGHT"
    if not math.isfinite(distance) or distance < 0.0:
        return "STRAIGHT"
    return "STRAIGHT_0" if distance <= fine_distance_m else "STRAIGHT"


def approach_level_from_motion(motion: str) -> int | None:
    """Extract a valid approach level from a supported motion name."""
    normalized = motion.strip().upper()
    if not normalized.startswith("STRAIGHT_"):
        return None
    try:
        level = int(normalized.rsplit("_", 1)[1])
    except (TypeError, ValueError):
        return None
    return level if 0 <= level < len(APPROACH_DISTANCE_LIMITS_M) else None
