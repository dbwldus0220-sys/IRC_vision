"""Select crab repeats from signed robot-center pixels after direction selection."""

import math


TWO_REPEAT_THRESHOLDS_PX = {
    'pickup_crab_right_0': 200.0,
    'pickup_crab_left_0': -100.0,
    'goal_camera_90_crab_right': 150.0,
    'goal_camera_90_crab_left': -120.0,
}
GOAL_CRAB_DURATIONS = {
    'goal_camera_90_crab_left': .540 / .95,
    'goal_camera_90_crab_left_2': 2 * .540 / .95,
    'goal_camera_90_crab_right': .550,
    'goal_camera_90_crab_right_2': 2 * .550,
}


def crab_repeat_motion(motion_id, offset_px):
    threshold = TWO_REPEAT_THRESHOLDS_PX.get(motion_id)
    if (threshold is None or type(offset_px) not in {int, float}
            or not math.isfinite(offset_px)):
        return motion_id
    if (offset_px > threshold if threshold > 0 else offset_px < threshold):
        return motion_id + '_2'
    return motion_id


def goal_crab_motion_id(action, command):
    if action not in {'GOAL_CAMERA90_CRAB_LEFT', 'GOAL_CAMERA90_CRAB_RIGHT'}:
        return None
    base = 'goal_camera_90_crab_' + action.rsplit('_', 1)[1].lower()
    return crab_repeat_motion(base, (command or {}).get('offset_x_px'))


def crab_support_motion_id(motion_id):
    """One and two repeats finish in the same support posture."""
    if motion_id.endswith('_2') and motion_id[:-2] in TWO_REPEAT_THRESHOLDS_PX:
        return motion_id[:-2]
    return motion_id
