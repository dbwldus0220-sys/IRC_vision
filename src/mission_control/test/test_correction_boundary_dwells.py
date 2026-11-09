"""Check correction rests and ordinary walking with the real bridge callbacks."""

import pytest

from test_continuous_approach import clock
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, navigation_message,
)


@pytest.mark.parametrize('source,phase,action,motion', [
    ('goal', 'GOAL_APPROACH', 'GOAL_CAMERA90_FINE_FORWARD_2', 'goal_camera_90_fine_forward_2'),
    ('hurdle', 'HURDLE_APPROACH', 'STRAIGHT_0', 'hurdle_fine_forward_5'),
    ('line', 'LINE_TRACK_AFTER_PICKUP', 'LINE_HEADING_TURN_LEFT_2', 'post_ball_line_turn_left_2'),
])
def test_reserved_correction_waits_from_real_completion(clock, source, phase, action, motion):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = motion
    bridge.stationary_since = clock[0] - .5
    bridge.navigation_command_callback(navigation_message(
        source=source, phase=phase, action=action, command_id=1, event_id=None))
    first = decoded_messages(bridge.executor_request_publisher)[-1]
    assert first['motion_id'] == motion
    bridge.executor_status_callback(executor_status(**first, status='RUNNING'))
    bridge.navigation_command_callback(navigation_message(
        source=source, phase=phase, action=action, command_id=2, event_id=None))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] += 2.
    bridge.executor_status_callback(executor_status(**first, status='SUCCEEDED'))
    deadline = clock[0] + .5
    assert bridge.transition_dwell_until == pytest.approx(deadline)
    clock[0] += .2
    bridge.executor_status_callback(executor_status(**first, status='SUCCEEDED'))
    assert bridge.transition_dwell_until == pytest.approx(deadline)
    clock[0] = deadline - .001
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = deadline
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher)[-1]['command_id'] == 2
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('source,phase,action', [
    ('line', 'LINE_TRACK', 'STRAIGHT_2'),
    ('goal', 'GOAL_APPROACH', 'GOAL_CAMERA_90_FORWARD_2'),
    ('hurdle', 'HURDLE_APPROACH', 'STRAIGHT_2'),
])
def test_ordinary_forward_keeps_continuous_queue(clock, source, phase, action):
    bridge = FakeBridge(advance_transition_dwells=False)
    for command_id in (1, 2):
        bridge.navigation_command_callback(navigation_message(
            source=source, phase=phase, action=action, command_id=command_id, event_id=None))
    assert len(bridge.executor_request_publisher.messages) == 2
    assert not bridge.queued_request_deferred
    assert bridge.transition_dwell_until is None
