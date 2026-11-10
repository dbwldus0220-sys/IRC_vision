"""Calibrated scoring zones shared by goal analysis and motion planning."""

import math


GOAL_SCORE_MIN_DEPTH_M = 0.450
GOAL_SCORE_TARGET_DEPTH_M = 0.660
GOAL_SCORE_TARGET_OFFSET_PX = 0.0
GOAL_SCORE_SPLIT_DEPTH_M = 0.530
GOAL_SCORE_LEGACY_MAX_DEPTH_M = 0.590
GOAL_SCORE_MAX_DEPTH_M = 0.760
GOAL_SCORE_NEAR_LIMIT_PX = 120.0
GOAL_SCORE_FAR_LIMIT_PX = 120.0
GOAL_LONG_SHOT_BANDS = (
    (0.590, 0.630, -80.0, 8),
    (0.630, 0.660, -80.0, 5),
    (0.660, 0.700, -80.0, 6),
    (0.700, 0.760, -70.0, 7),
)


def goal_shot_pixel_bounds(depth_m: float) -> tuple[float, float]:
    return -GOAL_SCORE_NEAR_LIMIT_PX, GOAL_SCORE_NEAR_LIMIT_PX


def goal_shot_pixels_aligned(depth_m: float, offset_x_px: float) -> bool:
    lower, upper = goal_shot_pixel_bounds(depth_m)
    return lower <= offset_x_px <= upper


def goal_shot_pixel_limit(depth_m: float) -> float:
    return GOAL_SCORE_NEAR_LIMIT_PX


def goal_shot_region(depth_m: float, offset_x_px: float | None = None) -> int | None:
    """Select by ring Depth Z only; pixel alignment is checked separately."""
    if not math.isfinite(depth_m):
        return None
    if not GOAL_SCORE_MIN_DEPTH_M <= depth_m < GOAL_SCORE_MAX_DEPTH_M:
        return None
    if depth_m < GOAL_SCORE_SPLIT_DEPTH_M:
        return 1
    if depth_m < GOAL_SCORE_LEGACY_MAX_DEPTH_M:
        return 2
    for lower, upper, _legacy_left, region in GOAL_LONG_SHOT_BANDS:
        if lower <= depth_m < upper:
            return region
    return None
