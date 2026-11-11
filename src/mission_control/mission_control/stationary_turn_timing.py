"""Rest duration for standalone right turns, excluding walking/composite motions."""

RIGHT_TURN_DWELL_SEC = 0.3


def right_turn_dwell_sec(identifier: str | None, default: float) -> float:
    """Accept navigation actions and physical IDs without matching recovery turns."""
    name = (identifier or "").lower()
    if name in {
        "turn_right", "align_right", "stationary_turn_right", "line_lost_turn_right",
        "ball_pickup_fine_search_right", "post_shot_default_turn_right",
    } or name.startswith((
        "line_turn_right_", "line_search_right_", "line_lost_turn_right_",
        "line_offset_turn_right_", "line_heading_turn_right_", "line_sparse_turn_right_",
        "post_ball_line_turn_right_", "post_shot_line_turn_right_", "post_shot_turn_right_",
        "post_shot_default_turn_right_", "pickup_first_turn_right_", "pickup_second_turn_right_",
        "pickup_camera_down_turn_right_", "ball_approach_turn_right_", "ball_align_turn_right_",
        "ball_pickup_camera_down_turn_right_", "ball_pickup_fine_turn_right_",
        "ball_pickup_fine_search_right_", "ball_pickup_post_backward_turn_right_",
        "goal_camera90_turn_right_", "goal_camera_90_turn_right_",
    )):
        return min(default, RIGHT_TURN_DWELL_SEC)
    return default
