"""Replay fine-walk entry separately from camera-down pickup, without hardware."""

import json
from pathlib import Path

import pytest
import yaml

from test_ball_entry_line_alignment import (
    clock, fresh, ground_ball_info, node_with_entry, send, start_line, status,
)
from test_motion_command_bridge import FakeBridge, decoded_messages


@pytest.mark.parametrize('distance,action', [
    (.600001, 'STRAIGHT_2'), (.600, 'BALL_APPROACH_FINE_FORWARD_5'),
    (.580, 'BALL_APPROACH_FINE_FORWARD_5'), (.550001, 'BALL_APPROACH_FINE_FORWARD_5'),
    (.550, 'PICKUP_NOW'),
])
def test_line_completion_uses_separate_fine_and_pickup_boundaries(clock, distance, action):
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=distance), plan=False)
    assert node.ball_pickup_entry_pending is (distance <= .550)
    status(node, bridge, request, 'SUCCEEDED')
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == action
    if action != 'BALL_APPROACH_FINE_FORWARD_5':
        return
    assert node.ball_entry_line_alignment.state == 'DONE'
    prepare = send(bridge, command)
    assert prepare['motion_id'] == 'pickup_fine_prepare'
    assert bridge.transition_dwell_until is None
    status(node, bridge, prepare, 'RUNNING')
    status(node, bridge, prepare, 'SUCCEEDED')
    fine = decoded_messages(bridge.executor_request_publisher)[-1]
    assert fine['motion_id'] == 'ball_approach_fine_forward_45_5'
    assert bridge.transition_dwell_until is None
    status(node, bridge, fine, 'RUNNING')
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.550), plan=False)
    status(node, bridge, fine, 'SUCCEEDED')
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.550))
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
    assert getattr(node, 'ball_post_motion_dwell_until', None) is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None


@pytest.mark.parametrize('changes', [
    {'depth_age_sec': 1.}, {'depth_valid': False}, {'confidence': .1},
    {'confirmation_confirmed': False},
])
def test_invalid_fine_distance_does_not_release_entry(clock, changes):
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_line(node, bridge, clock)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=1.2), plan=False)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.580, **changes), plan=False)
    status(node, bridge, request, 'SUCCEEDED')
    assert node.ball_entry_line_alignment.pending
    assert not node.ball_pickup_entry_pending


def test_fine_alias_keeps_camera_45_and_existing_body_cycle():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())
    assert aliases['motion_aliases']['ball_approach_fine_forward_45_5'] == 'R미세(5회)'
    motions = json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    fine = next(m for m in motions if m['name'] == 'R미세(5회)')
    assert fine['repeat_count'] == 5
    assert all(-34 < frame['angles']['0'] < -30 for frame in fine['frames'])


def test_camera45_fine_preparation_keeps_both_one_second_pauses(clock):
    from test_transition_dwells import check_both_pauses
    from test_motion_command_bridge import navigation_message
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'line_forward_4'
    bridge.navigation_command_callback(navigation_message(
        source='ball', action='BALL_APPROACH_FINE_FORWARD_5', command_id=1,
    ))
    check_both_pauses(bridge, clock, 'pickup_fine_prepare', 'ball_approach_fine_forward_45_5')
