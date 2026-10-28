"""Select the grasp only from a valid post-retreat pixel distance."""

import math


PICKUP_GRASP_SPLIT_BOTTOM_DISTANCE_PX = 450.0


def pickup_grasp_motion_id(bottom_distance_px: object) -> str | None:
    """Keep the 450-pixel boundary in the 400 grasp range."""
    if (isinstance(bottom_distance_px, bool)
            or not isinstance(bottom_distance_px, (int, float))
            or not math.isfinite(bottom_distance_px)
            or bottom_distance_px < 0):
        return None
    return ("pickup_400" if bottom_distance_px <= PICKUP_GRASP_SPLIT_BOTTOM_DISTANCE_PX
            else "pickup_500")
