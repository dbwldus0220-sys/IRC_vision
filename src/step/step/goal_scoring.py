"""Calibrated scoring zones shared by goal analysis and motion planning."""

import math


GOAL_SCORE_MIN_DEPTH_M = 0.430
GOAL_SCORE_TARGET_DEPTH_M = 0.510
GOAL_SCORE_MAX_DEPTH_M = 0.570
GOAL_SCORE_NEAR_LIMIT_PX = 120.0
GOAL_SCORE_FAR_LIMIT_PX = 100.0


def goal_shot_pixel_limit(depth_m: float) -> float:
    return (GOAL_SCORE_NEAR_LIMIT_PX if depth_m < GOAL_SCORE_TARGET_DEPTH_M
            else GOAL_SCORE_FAR_LIMIT_PX)


def goal_shot_region(depth_m: float, offset_x_px: float) -> int | None:
    """Use camera Depth Z; include lower bounds and exclude upper bounds."""
    if not math.isfinite(depth_m) or not math.isfinite(offset_x_px):
        return None
    if not GOAL_SCORE_MIN_DEPTH_M <= depth_m < GOAL_SCORE_MAX_DEPTH_M:
        return None
    if abs(offset_x_px) > goal_shot_pixel_limit(depth_m):
        return None
    # Zero horizontal error belongs to the nonnegative side of each depth band.
    if depth_m < GOAL_SCORE_TARGET_DEPTH_M:
        return 1 if offset_x_px >= 0.0 else 4
    return 2 if offset_x_px >= 0.0 else 3
