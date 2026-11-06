"""Bottom-distance limits for omitting a pickup fine-forward step."""

FINE_FORWARD_MOTIONS = frozenset({
    "ball_approach_fine_forward_45_5", "ball_general_fine_forward_8",
    "pickup_fine_forward_0", "ball_top_loss_fine_forward_45_3",
    "pickup_top_loss_fine_forward_0_3",
})


def pickup_fine_skip_limit(motion_id: str | None) -> float | None:
    if motion_id in FINE_FORWARD_MOTIONS:
        return 230.0
    if motion_id in {
        "pickup_crab_left_0", "pickup_crab_right_0",
        "pickup_crab_left_0_2", "pickup_crab_right_0_2",
    }:
        return 270.0
    return None


def pickup_fine_can_skip(motion_id: str | None, bottom: float) -> bool:
    limit = pickup_fine_skip_limit(motion_id)
    if limit is None or bottom < 0:
        return False
    # Fine walking includes the boundary; the existing crab limit stays strict.
    return bottom <= limit if motion_id in FINE_FORWARD_MOTIONS else bottom < limit
