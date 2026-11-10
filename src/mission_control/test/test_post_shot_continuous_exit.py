"""Check region 5–8 exits as one turn/forward motion, then ordinary Line."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import forward_status
from test_post_ball_goal_handoff import receive
from test_wait_refresh_and_fine_settle import LiveInputHarness


ROOT = Path(__file__).resolve().parents[3]


def start_exit(clock, section, region):
    node = LiveInputHarness(phase='POST_SHOT_TURN')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.phase_manager.ball_sections_processed = section
    node.phase_manager.last_completed_shot_region = region
    node.queued_general_command_id = None
    node.queued_general_action = None
    node.queued_general_source = None
    bridge = FakeBridge(advance_transition_dwells=False)
    receive(node, clock, 'line', {'detected': False, 'raw_detected': False})
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    expected = ('RIGHT_5' if section == 1 else 'LEFT_3')
    assert command['action'] == f'POST_SHOT_EXIT_{expected}_FORWARD_4'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request, = decoded_messages(bridge.executor_request_publisher)
    assert request['motion_id'] == f'post_shot_exit_{expected.lower()}_forward_4'
    assert bridge.transition_dwell_until is None
    forward_status(node, bridge, request, 'RUNNING')
    return node, bridge, request


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [5, 6, 7, 8])
def test_single_command_finishes_in_normal_line_without_alignment_or_reservation(clock, section, region):
    node, bridge, request = start_exit(clock, section, region)
    for heading in (0., 31., -31.):
        clock[0] += .2
        receive(node, clock, 'line', line_info(heading))
        Node._publish_decision(node)
        assert node.mission_phase == 'POST_SHOT_TURN'
        assert node.general_motion_gate.locked
        assert node.queued_general_command_id is None
        assert bridge.queued_request_id is None
        assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')
    assert not node.general_motion_gate.locked
    assert node.post_shot_dwell_until is None
    assert node.latest_info['line'] is None
    assert bridge.last_physical_motion_id == 'line_forward_4'
    assert not bridge.motion_in_progress
    assert bridge._transition_prepare_motion('goal_camera_90_fine_forward_1') == 'goal_fine_prepare_90'
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')
    clock[0] += .01
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['action'] == 'STRAIGHT'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert bridge.transition_dwell_until is None
    next_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert next_request['command_id'] == command['command_id']
    assert next_request['motion_id'] == bridge.motion_id_for_action('STRAIGHT')


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_composite_never_replays_or_starts_normal_line(clock, section, status):
    node, bridge, request = start_exit(clock, section, 8)
    forward_status(node, bridge, request, status)
    assert node.phase_manager.post_shot_failed
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert bridge.last_physical_motion_id is None
    clock[0] += 1.
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['reason'] == 'post_shot_motion_failed'
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
@pytest.mark.parametrize('name,alias', [
    ('t골대후제자리우회전(5회)', 'post_shot_exit_right_5_forward_4'),
    ('p골대후제자리좌회전(3회)', 'post_shot_exit_left_3_forward_4'),
])
def test_catalog_contains_exact_turn_and_four_forward_cycles_with_no_join_hold(kind, name, alias):
    motions = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    catalog = {m['name']: m for m in motions}
    assert len(catalog) == len(motions)
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    combined, turn, forward = catalog[aliases[alias]], catalog[name], catalog['p전진45(4회)']
    prefix_size = len(turn['frames'])
    assert combined['frames'][:prefix_size] == turn['frames']
    assert combined['playback_speed'] == turn['playback_speed']
    assert combined['repeat_count'] == 1
    assert len(combined['frames']) == prefix_size + 4 * len(forward['frames'])
    turn_end = turn['frames'][-1]['start_ms'] + turn['frames'][-1]['time_ms']
    assert combined['frames'][prefix_size]['start_ms'] == turn_end
    for index, actual in enumerate(combined['frames'][prefix_size:]):
        original = forward['frames'][index % len(forward['frames'])]
        for field in ('angles', 'torques', 'name', 'lift_early_arrival'):
            assert actual[field] == original[field]
        assert abs(actual['time_ms'] / combined['playback_speed']
                   - original['time_ms'] / forward['playback_speed']) <= 1.
        assert not actual.get('stop_at_end', False)
    assert combined['end_pose'] == forward['end_pose']
    expected_forward_ms = (4 * (forward['frames'][-1]['start_ms'] + forward['frames'][-1]['time_ms'])
                           - forward['frames'][0]['start_ms']) / forward['playback_speed']
    assert (combined['max_seq_ms'] - turn_end) / combined['playback_speed'] == pytest.approx(expected_forward_ms, abs=1.)
    assert combined['max_seq_ms'] / combined['playback_speed'] + combined['completion']['settle_timeout_ms'] < FakeBridge.DEFAULT_TIMEOUT_MS
    if kind == 'runtime':
        assert combined['completion']['position_tolerance_deg'] == 5.
