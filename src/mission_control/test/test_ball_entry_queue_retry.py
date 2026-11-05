"""Check a transient Ball confirmation gap at the Line reservation boundary."""

import pytest

from test_ball_entry_line_alignment import node_with_entry, start_line, late_line, status
from test_ball_approach_latency import sample
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge


@pytest.mark.parametrize('recovers_before_completion', [True, False])
@pytest.mark.parametrize('raw_detected', [True, False])
def test_entry_retries_after_confirmation_gap_without_replaying_old_ball(clock, recovers_before_completion, raw_detected):
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_line(node, bridge, clock)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .65
    fresh(node, clock, 'ball', sample(clock[0]), plan=False)
    assert node.ball_entry_line_alignment.active
    for _ in range(3):
        fresh(node, clock, 'ball', {**sample(clock[0], detected=False),
                                  'raw_detected': raw_detected}, plan=False)
    assert node.ball_approach_observation.anchor is not None
    late_line(node, clock, line_info())
    assert node.planner.ball_lock_active
    assert node.queued_general_command_id is None
    if recovers_before_completion:
        fresh(node, clock, 'ball', sample(clock[0]), plan=False)
        assert node.queued_general_command_id is not None
        assert node.publisher.messages[-1]['source_command']['ball_entry_line_alignment']
        assert node.general_motion_gate.active_command_id == request['command_id']
    else:
        status(node, bridge, request, 'SUCCEEDED')
        assert node.queued_general_command_id is None
        assert not any(m['valid'] for m in node.publisher.messages[1:])
