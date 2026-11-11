"""Regressions for the original late-motion Ball reservation policy."""

import pytest

from test_ball_entry_recent_reservation import begin, emit
from test_continuous_approach import clock


def test_confirmed_ball_can_move_in_image_while_walking(clock):
    node, _, _ = begin(clock)
    for index, ratio in enumerate((.76, .80, .84)):
        emit(node, clock, ratio, track=54,
             bbox=[600, 240 + 40 * index, 650, 290 + 40 * index])
    assert node.queued_general_action == 'STRAIGHT_2'


@pytest.mark.parametrize('capture_age,expected', [(.36, True), (.60, False)])
def test_ball_reservation_uses_existing_source_freshness(clock, capture_age, expected):
    node, _, _ = begin(clock)
    duration = node.active_line_motion_duration_sec
    for ratio in (.90, .915, .93):
        emit(node, clock, ratio - capture_age / duration, received_ratio=ratio, track=54)
    assert (node.queued_general_command_id is not None) is expected


def test_pre_window_frames_do_not_authorize_a_walk(clock):
    node, _, _ = begin(clock)
    for ratio in (.55, .60, .65, .71, .75):
        emit(node, clock, ratio)
    assert node.queued_general_command_id is None
    emit(node, clock, .80)
    assert node.queued_general_action == 'STRAIGHT_2'


def test_repeated_capture_does_not_count_as_three_frames(clock):
    node, _, _ = begin(clock)
    for ratio in (.76, .80, .84):
        emit(node, clock, .75, received_ratio=ratio)
    assert node.queued_general_command_id is None
