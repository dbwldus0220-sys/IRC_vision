"""Explicit support changes for near Ball alignment, including head-down walking."""

PREPARE_FINE = 'BALL_APPROACH_PREPARE_FINE'
PREPARE_TURN = 'BALL_APPROACH_PREPARE_TURN'
FINE_FORWARD = 'BALL_APPROACH_FINE_FORWARD_5'
FORWARD_TWO = 'BALL_APPROACH_FORWARD_2'
PREPARATIONS = {PREPARE_FINE, PREPARE_TURN}


def camera45_turn_duration_sec(direction, count):
    """Source83 left turns use a 355ms cycle at the original 1.05 speed."""
    if direction == 'LEFT':
        return .355 * count / 1.05
    return .300 * count / .9


def physical_motion_id(identifier):
    """Keep existing pickup/loss transition history in its original aliases."""
    return {
        'ball_align_prepare_fine_45': 'pickup_fine_prepare',
        'ball_align_prepare_turn_45': 'fine_to_turn_ready_45',
        'ball_align_fine_forward_45_5': 'ball_approach_fine_forward_45_5',
        'ball_align_forward_2': 'ball_camera_down_forward_2',
    }.get(identifier, identifier.replace('ball_align_turn_', 'post_ball_line_turn_'))


def motion_id(action):
    if action == PREPARE_FINE:
        return 'ball_align_prepare_fine_45'
    if action == PREPARE_TURN:
        return 'ball_align_prepare_turn_45'
    if action == FINE_FORWARD:
        return 'ball_align_fine_forward_45_5'
    if action == FORWARD_TWO:
        return 'ball_align_forward_2'
    if action.startswith('BALL_APPROACH_TURN_'):
        return 'ball_align_turn_' + action.removeprefix('BALL_APPROACH_TURN_').lower()
    return None


def duration_sec(identifier):
    if identifier == 'ball_align_prepare_fine_45':
        return .148
    if identifier == 'ball_align_prepare_turn_45':
        return .175 / 1.05
    if identifier == 'ball_align_fine_forward_45_5':
        return .394 * 5 / 1.05
    if identifier == 'ball_align_forward_2':
        return .514 * 2 / 1.05
    if identifier.startswith('ball_align_turn_'):
        count = int(identifier.rsplit('_', 1)[1])
        return camera45_turn_duration_sec('LEFT' if '_left_' in identifier else 'RIGHT', count)
    return None


def support_after(identifier):
    return ('fine' if identifier in {'ball_align_prepare_fine_45',
                                    'ball_align_fine_forward_45_5'} else 'standard')


def with_preparation(command, support):
    """Make preparation a separate decision so its successor rechecks Vision."""
    if not command.get('valid') or command.get('ball_fine_alignment') is not True:
        return command
    target = command.get('motion', '')
    if target in PREPARATIONS:
        return command
    prepare = None
    if target == FINE_FORWARD and support != 'fine':
        prepare = PREPARE_FINE
    elif (target.startswith('BALL_APPROACH_TURN_') or target == FORWARD_TWO) and support == 'fine':
        prepare = PREPARE_TURN
    if prepare is None:
        return command
    return {**command, 'motion': prepare, 'action': prepare,
            'reason': 'ball_fine_support_change_before_reobservation',
            'linear_speed_mps': 0.0, 'angular_speed_rad_s': 0.0,
            'travel_distance_m': 0.0, 'lateral_travel_distance_m': 0.0,
            'ball_fine_target_action': target}
