"""Bottom-distance limits for omitting a pickup fine-forward step."""

FINE_FORWARD_MOTIONS = frozenset({
    "ball_approach_fine_forward_45_5", "ball_general_fine_forward_8",
    "pickup_fine_forward_0", "ball_top_loss_fine_forward_45_3",
    "pickup_top_loss_fine_forward_0_3",
})

FORWARD_RECOVERY_SKIP_LIMIT_PX = 210.0


def is_pickup_forward_or_recovery(motion_id: str | None) -> bool:
    return bool(motion_id and (
        motion_id in {"forward", "sdk_forward_4", "ball_align_forward_2"}
        or motion_id.startswith(("line_forward_", "ball_camera_down_forward_",
                                 "post_ball_forward_", "line_recovery_"))
    ))


def pickup_fine_skip_limit(motion_id: str | None) -> float | None:
    if is_pickup_forward_or_recovery(motion_id):
        return FORWARD_RECOVERY_SKIP_LIMIT_PX
    if motion_id in FINE_FORWARD_MOTIONS:
        return 270.0
    if motion_id in {
        "pickup_crab_left_0", "pickup_crab_right_0",
        "pickup_crab_left_0_2", "pickup_crab_right_0_2",
    }:
        return 220.0
    return None


def pickup_fine_can_skip(motion_id: str | None, bottom: float) -> bool:
    limit = pickup_fine_skip_limit(motion_id)
    if limit is None or bottom < 0:
        return False
    return bottom <= limit
