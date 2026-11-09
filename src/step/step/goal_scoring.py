"""Calibrated scoring zones shared by goal analysis and motion planning."""

import math


GOAL_SCORE_MIN_DEPTH_M = 0.450
GOAL_SCORE_TARGET_DEPTH_M = 0.630
GOAL_SCORE_TARGET_OFFSET_PX = -40.0
GOAL_SCORE_SPLIT_DEPTH_M = 0.530
GOAL_SCORE_LEGACY_MAX_DEPTH_M = 0.590
GOAL_SCORE_MAX_DEPTH_M = 0.760
GOAL_SCORE_NEAR_LIMIT_PX = 120.0
GOAL_SCORE_FAR_LIMIT_PX = 100.0
GOAL_LONG_SHOT_BANDS = (
    (0.590, 0.630, -80.0, 8),
    (0.630, 0.660, -80.0, 5),
    (0.660, 0.700, -80.0, 6),
    (0.700, 0.760, -70.0, 7),
)


def goal_shot_pixel_bounds(depth_m: float) -> tuple[float, float]:
    if depth_m >= GOAL_SCORE_LEGACY_MAX_DEPTH_M:
        return (-70.0 if depth_m >= 0.700 else -80.0), 0.0
    limit = goal_shot_pixel_limit(depth_m)
    return -limit, limit


def goal_shot_pixels_aligned(depth_m: float, offset_x_px: float) -> bool:
    lower, upper = goal_shot_pixel_bounds(depth_m)
    return lower <= offset_x_px <= upper


def goal_shot_pixel_limit(depth_m: float) -> float:
    return (GOAL_SCORE_NEAR_LIMIT_PX if depth_m < GOAL_SCORE_SPLIT_DEPTH_M
            else GOAL_SCORE_FAR_LIMIT_PX)


def goal_shot_region(depth_m: float, offset_x_px: float) -> int | None:
    """Use camera Depth Z; include lower bounds and exclude upper bounds."""
    if not math.isfinite(depth_m) or not math.isfinite(offset_x_px):
        return None
    if not GOAL_SCORE_MIN_DEPTH_M <= depth_m < GOAL_SCORE_MAX_DEPTH_M:
        return None
    if depth_m >= GOAL_SCORE_LEGACY_MAX_DEPTH_M:
        for lower, upper, left, region in GOAL_LONG_SHOT_BANDS:
            if lower <= depth_m < upper and left <= offset_x_px <= 0.0:
                return region
        return None
    if not goal_shot_pixels_aligned(depth_m, offset_x_px):
        return None
    # Zero horizontal error belongs to the nonnegative side of each depth band.
    if depth_m < GOAL_SCORE_SPLIT_DEPTH_M:
        return 1 if offset_x_px >= 0.0 else 4
    return 2 if offset_x_px >= 0.0 else 3
