"""Check region 5–8 turn/recovery composites, then exit alignment and ordinary Line."""

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
    expected = (('RIGHT_9_RECOVERY_RIGHT_4' if section == 1 else 'LEFT_4_RECOVERY_LEFT_4')
                if region in {1, 2} else
                ('RIGHT_5' if section == 1 else 'LEFT_3') + '_FORWARD_4')
    assert command['action'] == f'POST_SHOT_EXIT_{expected}'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request, = decoded_messages(bridge.executor_request_publisher)
    assert request['motion_id'] == f'post_shot_exit_{expected.lower()}'
    assert bridge.transition_dwell_until is None
    forward_status(node, bridge, request, 'RUNNING')
    return node, bridge, request


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 6, 7, 8])
def test_early_frames_finish_in_exit_alignment_without_extra_forward_or_reservation(clock, section, region):
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
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
    assert not node.general_motion_gate.locked
    assert node.post_shot_dwell_until is None
    assert node.latest_info['line'] is None
    recovery_direction = 'right' if section == 1 else 'left'
    assert bridge.last_physical_motion_id == f'line_recovery_{recovery_direction}_4'
    assert not bridge.motion_in_progress
    assert bridge._transition_prepare_motion('goal_camera_90_fine_forward_1') == 'goal_fine_prepare_90'
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
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
@pytest.mark.parametrize('name,alias,recovery_name,blocks', [
    ('t골대후제자리우회전', 'post_shot_exit_right_9_recovery_right_4', 'm라인복귀우회전45도', 1),
    ('p골대후제자리좌회전', 'post_shot_exit_left_4_recovery_left_4', 'm라인복귀좌회전45도', 1),
    ('t골대후제자리우회전(6회)', 'post_shot_exit_right_5_forward_4', 'm라인복귀우회전45도', 1),
    ('p골대후제자리좌회전(4회)', 'post_shot_exit_left_3_forward_4', 'm라인복귀좌회전45도', 1),
])
def test_catalog_preserves_turn_and_recovery_with_half_second_join_hold(kind, name, alias, recovery_name, blocks):
    motions = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    catalog = {m['name']: m for m in motions}
    assert len(catalog) == len(motions)
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    combined, turn, recovery = catalog[aliases[alias]], catalog[name], catalog[recovery_name]
    forward = catalog['m전진45(2회)'] if alias.endswith('_forward_4') else None
    prefix_size = len(turn['frames'])
    assert combined['frames'][:prefix_size] == turn['frames']
    assert combined['playback_speed'] == turn['playback_speed']
    assert combined['repeat_count'] == 1
    forward_frames = 2 * len(forward['frames']) if forward else 0
    assert len(combined['frames']) == prefix_size + forward_frames + 4 * blocks * len(recovery['frames'])
    turn_end = turn['frames'][-1]['start_ms'] + turn['frames'][-1]['time_ms']
    assert ((combined['frames'][prefix_size]['start_ms'] - turn_end)
            / combined['playback_speed']) == pytest.approx(500.)
    if forward:
        for index, actual in enumerate(combined['frames'][prefix_size:prefix_size + forward_frames]):
            original = forward['frames'][index % len(forward['frames'])]
            for field in ('angles', 'torques', 'name', 'lift_early_arrival'):
                assert actual[field] == original[field]
            assert abs(actual['time_ms'] / combined['playback_speed']
                       - original['time_ms'] / forward['playback_speed']) <= 1.
            cycle, frame_index = divmod(index, len(forward['frames']))
            cycle_ms = max(f['start_ms'] + f['time_ms'] for f in forward['frames'])
            assert (actual['start_ms'] - combined['frames'][prefix_size]['start_ms']) / combined['playback_speed'] == pytest.approx(
                (cycle * cycle_ms + original['start_ms'] - forward['frames'][0]['start_ms']) / forward['playback_speed'], abs=1.)
            assert actual.get('playback_cycle_end', False) == (frame_index == len(forward['frames']) - 1)
        join = prefix_size + forward_frames
        assert (combined['frames'][join]['start_ms'] - combined['frames'][join - 1]['start_ms']
                - combined['frames'][join - 1]['time_ms']) / combined['playback_speed'] == pytest.approx(
                    recovery['frames'][0]['start_ms'] / recovery['playback_speed'], abs=1.)
    recovery_offset = prefix_size + forward_frames
    for index, actual in enumerate(combined['frames'][recovery_offset:]):
        original = recovery['frames'][index % len(recovery['frames'])]
        for field in ('angles', 'torques', 'name', 'lift_early_arrival'):
            assert actual[field] == original[field]
        assert abs(actual['time_ms'] / combined['playback_speed']
                   - original['time_ms'] / recovery['playback_speed']) <= 1.
        assert not actual.get('stop_at_end', False)
        cycle, frame_index = divmod(index, len(recovery['frames']))
        cycle_ms = max(f['start_ms'] + f['time_ms'] for f in recovery['frames'])
        expected_start_ms = (cycle * cycle_ms + original['start_ms']
                             - recovery['frames'][0]['start_ms']) / recovery['playback_speed']
        assert (actual['start_ms'] - combined['frames'][recovery_offset]['start_ms']) / combined['playback_speed'] == pytest.approx(expected_start_ms, abs=1.)
        assert actual.get('playback_cycle_end', False) == (frame_index == len(recovery['frames']) - 1)
    assert combined['end_pose'] == recovery['end_pose']
    expected_recovery_ms = (4 * blocks * (recovery['frames'][-1]['start_ms'] + recovery['frames'][-1]['time_ms'])
                            - recovery['frames'][0]['start_ms']) / recovery['playback_speed']
    expected_forward_ms = (2 * max(f['start_ms'] + f['time_ms'] for f in forward['frames'])
                           - forward['frames'][0]['start_ms']) / forward['playback_speed'] if forward else 0.
    recovery_lead_ms = recovery['frames'][0]['start_ms'] / recovery['playback_speed'] if forward else 0.
    assert (combined['max_seq_ms'] - turn_end) / combined['playback_speed'] == pytest.approx(
        expected_recovery_ms + expected_forward_ms + recovery_lead_ms + 500., abs=1.)
    assert combined['max_seq_ms'] / combined['playback_speed'] + combined['completion']['settle_timeout_ms'] < FakeBridge.DEFAULT_TIMEOUT_MS
    if kind == 'runtime':
        assert combined['completion']['position_tolerance_deg'] == 5.


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('heading', [-44.56, 44.56])
def test_exit_corrects_above_40_then_resumes_without_another_fixed_forward(clock, section, heading):
    node, bridge, request = start_exit(clock, section, 8)
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
    clock[0] += .6
    receive(node, clock, 'line', line_info(heading))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    direction = 'LEFT' if heading < 0 else 'RIGHT'
    assert command['action'].startswith(f'POST_SHOT_LINE_TURN_{direction}_')
    assert command['source_command']['heading_tolerance_deg'] == 40.
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    turn = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, turn, 'RUNNING')
    clock[0] += 3.
    forward_status(node, bridge, turn, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
    clock[0] += .6
    Node._publish_decision(node)
    clock[0] += .01
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT'
    assert all(m['action'] != 'POST_SHOT_FORWARD' for m in node.publisher.messages)


@pytest.mark.parametrize('section,direction', [(1, 'RIGHT'), (2, 'LEFT')])
def test_exit_missing_line_searches_course_side(clock, section, direction):
    node, bridge, request = start_exit(clock, section, 5)
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    clock[0] += .6
    receive(node, clock, 'line', {'detected': False, 'raw_detected': False})
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['valid']
    assert command['action'].startswith(f'POST_SHOT_LINE_TURN_{direction}_')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'


@pytest.mark.parametrize('heading,aligned', [(-40.001, False), (-40., True), (40., True), (40.001, False)])
def test_exit_alignment_boundary_is_40_only(heading, aligned):
    from mission_control.motion_decision_planner import MotionDecisionPlanner
    planner = MotionDecisionPlanner()
    decision = planner.plan('POST_SHOT_EXIT_ALIGN', {'line': line_info(heading)}, .1)
    assert decision.valid
    assert (decision.action == 'POST_SHOT_EXIT_ALIGNED') is aligned
    assert planner.line_planner.config.in_place_ground_heading_deg == 50.


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_exit_alignment_never_resumes_line(clock, status):
    node, bridge, request = start_exit(clock, 2, 8)
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    clock[0] += .6
    receive(node, clock, 'line', line_info(-44.56))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    turn = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, turn, 'RUNNING')
    clock[0] += 3.
    forward_status(node, bridge, turn, status)
    clock[0] += 1.
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['reason'] == 'post_shot_motion_failed'
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 6, 7, 8])
@pytest.mark.parametrize('heading', [-44.56, 44.56])
def test_exit_stationary_alignment_waits_direction_specific_rest_from_composite_end(clock, section, region, heading):
    node, bridge, request = start_exit(clock, section, region)
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    ended = clock[0]
    initial_commands = len(node.publisher.messages)
    dwell = .3 if heading > 0 else .5
    for elapsed in (.01, .25, dwell - .001):
        clock[0] = ended + elapsed
        receive(node, clock, 'line', line_info(heading))
        Node._publish_decision(node)
        assert not any(m['valid'] for m in node.publisher.messages[initial_commands:])
        assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = ended + dwell + .001
    receive(node, clock, 'line', line_info(heading))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    direction = 'LEFT' if heading < 0 else 'RIGHT'
    assert command['valid']
    assert command['action'].startswith(f'POST_SHOT_LINE_TURN_{direction}_')
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert len(bridge.executor_request_publisher.messages) == 2


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 6, 7, 8])
@pytest.mark.parametrize('heading', [-31., 31.])
def test_exit_recovery_walk_starts_on_first_fresh_frame_without_dwell(clock, section, region, heading):
    node, bridge, request = start_exit(clock, section, region)
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    clock[0] += .01
    receive(node, clock, 'line', line_info(heading, offset=.3 if heading > 0 else -.3))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    direction = 'LEFT' if heading < 0 else 'RIGHT'
    assert command['valid']
    assert command['action'] == f'RECOVER_{direction}_TURN_{direction}_4'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert len(bridge.executor_request_publisher.messages) == 2
    assert bridge.transition_dwell_until is None
    assert node.post_shot_dwell_until is None


@pytest.mark.parametrize('section', [1, 2])
def test_turn_wait_is_cancelled_when_fresh_line_now_requires_recovery_walk(clock, section):
    node, bridge, request = start_exit(clock, section, 8)
    clock[0] += 5.
    forward_status(node, bridge, request, 'SUCCEEDED')
    clock[0] += .01
    receive(node, clock, 'line', line_info(44.56))
    Node._publish_decision(node)
    assert node.pre_motion_settle_action.startswith('POST_SHOT_LINE_TURN_')
    clock[0] += .1
    receive(node, clock, 'line', line_info(31., offset=.3))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert len(bridge.executor_request_publisher.messages) == 2
    assert node.pre_motion_settle_action is None
    assert bridge.transition_dwell_until is None
