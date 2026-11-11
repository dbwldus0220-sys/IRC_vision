"""Camera-45 support changes rest before and after, crediting prior rest."""

import pytest

from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status, navigation_message
from test_transition_dwells import clock


def command(action, command_id):
    return navigation_message(
        action=action, command_id=command_id, event_id=None,
        source='ball', phase='BALL_APPROACH',
        source_command={'ball_fine_alignment': True},
    )


@pytest.mark.parametrize('previous,prepare,following', [
    ('line_forward_4', 'BALL_APPROACH_PREPARE_FINE', 'BALL_APPROACH_FINE_FORWARD_5'),
    ('post_ball_line_turn_right_1', 'BALL_APPROACH_PREPARE_FINE', 'BALL_APPROACH_FINE_FORWARD_5'),
    ('ball_approach_fine_forward_45_5', 'BALL_APPROACH_PREPARE_TURN', 'BALL_APPROACH_TURN_RIGHT_1'),
])
@pytest.mark.parametrize('rest', [0., .2, .5, .8])
def test_camera45_preparation_rests_follow_adjacent_gaits(clock, previous, prepare, following, rest):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    bridge.stationary_since = clock[0]
    clock[0] += rest
    bridge.navigation_command_callback(command(prepare, 1))
    remaining = max(0., .5 - rest)
    if remaining:
        assert not bridge.executor_request_publisher.messages
        assert bridge.transition_dwell_until == pytest.approx(clock[0] + remaining)
        clock[0] = bridge.transition_dwell_until
        bridge._check_atomic_dwell()
    else:
        assert bridge.transition_dwell_until is None
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'].startswith('ball_align_prepare_')
    clock[0] += .2
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert not bridge.motion_in_progress
    assert bridge.transition_dwell_until is None
    # The successor owns the post-preparation rest; the preparation has no own post timer.
    bridge.navigation_command_callback(command(following, 2))
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    clock[0] = bridge.transition_dwell_until
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 2
