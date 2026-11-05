"""Bottom-distance limits for omitting a pickup fine-forward step."""


def pickup_fine_skip_limit(motion_id: str | None) -> float | None:
    if motion_id in {
        "ball_approach_fine_forward_45_5", "ball_general_fine_forward_8",
        "pickup_fine_forward_0", "ball_top_loss_fine_forward_45_3",
        "pickup_top_loss_fine_forward_0_3",
    }:
        return 220.0
    if motion_id in {
        "pickup_crab_left_0", "pickup_crab_right_0",
        "pickup_crab_left_0_2", "pickup_crab_right_0_2",
    }:
        return 270.0
    return None
