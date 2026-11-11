"""Route completed shot regions to complete, shortened composite exit motions."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.mission_phase_manager import MissionPhaseManager
from test_mission_phase_flow import MissionFlowHarness, line_info, release_general
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_shot_start_posture import finish_sequence

ROOT = Path(__file__).resolve().parents[3]
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', (1, 2, 3, 4, 5, 6, 8))
def test_executed_region_routes_through_gate_bridge_and_line_reacquisition(monkeypatch, section, region):
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: 10.)
    node = MissionFlowHarness(phase='GOAL_APPROACH')
    manager = node.phase_manager
    manager.ball_sections_processed = section - 1
    manager.shots_completed = manager.shots_executed = section - 1
    assert manager.start_special_action('SHOT', 100)
    manager.handle_motion_status('SHOT', 100, 'RUNNING', motion_id='goal_fine_prepare_90')
    for _ in range(2):
        manager.handle_motion_status('SHOT', 100, 'RUNNING', motion_id=f'goal_shot_region_{region}')
    manager.handle_motion_status('SHOT', 100, 'SUCCEEDED', motion_id='__NON_BLOCKING_DWELL__')
    assert manager.shots_completed == manager.shots_executed == section
    assert manager.last_completed_shot_region == region
    manager.handle_motion_status('SHOT', 100, 'SUCCEEDED', motion_id='goal_shot_region_1')
    assert manager.last_completed_shot_region == region

    direction = 'RIGHT' if section == 1 else 'LEFT'
    count = (5 if section == 1 else 3) if region >= 5 else (9 if section == 1 else 4)
    turn = node.publish_vision(line=line_info())[-1]
    expected = (f'POST_SHOT_EXIT_{direction}_{count}_FORWARD_4' if region >= 5
                else f'POST_SHOT_EXIT_{direction}_{count}_RECOVERY_{direction}_4' if region in {1, 2}
                else f'POST_SHOT_TURN_{direction}_{count}')
    assert turn['valid'] and turn['action'] == expected
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(turn)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    name = 't골대후제자리우회전' if section == 1 else 'p골대후제자리좌회전'
    recovery = '우' if section == 1 else '좌'
    far_recovery = '좌' if section == 1 else '우'
    turn_count = 6 if section == 1 else 4
    assert ALIASES[request['motion_id']] == (
        f'{name}({turn_count}회)' + f'+라인복귀{far_recovery}회전45도(4회)' * 2 if region >= 5
        else name + f'+라인복귀{recovery}회전45도(4회)' if region in {1, 2} else name
    )
    finish_sequence(bridge)
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'

    node.planner.goal_lock_active = True
    node.planner.last_line_seen_direction = 'LEFT'
    release_general(node, turn)
    assert node.mission_phase == (
        'POST_SHOT_EXIT_ALIGN' if region in {1, 2, 5, 6, 7, 8} else 'POST_SHOT_LINE_ALIGN'
    )
    assert not node.planner.goal_lock_active
    assert node.planner.last_line_seen_direction is None
    assert node.planner.post_ball_line_search_direction == direction
    assert node.post_shot_dwell_until == (None if region in {1, 2, 5, 6, 7, 8} else pytest.approx(10.3 if section == 1 else 10.5))


@pytest.mark.parametrize('terminal_motion', [None, 'goal_shot_region_2', '__NON_BLOCKING_DWELL__'])
def test_next_shot_does_not_inherit_previous_region(terminal_motion):
    manager = MissionPhaseManager(initial_phase='GOAL_APPROACH')
    manager.start_special_action('SHOT', 1)
    manager.handle_motion_status('SHOT', 1, 'RUNNING', motion_id='goal_shot_region_5')
    manager.handle_motion_status('SHOT', 1, 'SUCCEEDED')
    assert manager.post_shot_turn_action() == 'POST_SHOT_EXIT_RIGHT_5_FORWARD_4'
    manager.set_phase('GOAL_APPROACH')
    manager.start_special_action('SHOT', 2)
    manager.handle_motion_status('SHOT', 1, 'RUNNING', motion_id='goal_shot_region_8')
    manager.handle_motion_status('SHOT', 2, 'RUNNING', motion_id='goal_fine_prepare_90')
    manager.handle_motion_status('SHOT', 2, 'SUCCEEDED', motion_id=terminal_motion)
    assert manager.last_completed_shot_region == (2 if terminal_motion == 'goal_shot_region_2' else None)
    assert manager.post_shot_turn_action() == (
        'POST_SHOT_EXIT_LEFT_4_RECOVERY_LEFT_4' if terminal_motion == 'goal_shot_region_2'
        else 'POST_SHOT_TURN_LEFT_4')


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_short_exit_failure_stops_without_forward_or_replay(section, status):
    node = MissionFlowHarness(phase='POST_SHOT_TURN')
    node.phase_manager.ball_sections_processed = section
    node.phase_manager.last_completed_shot_region = 8
    turn = node.publish_vision(line=line_info())[-1]
    node.send_status(turn['action'], turn['command_id'], 'RUNNING')
    node.send_status(turn['action'], turn['command_id'], status)
    decision = node.publish_vision(line=line_info())[-1]
    assert decision['action'] == 'WAIT' and decision['reason'] == 'post_shot_motion_failed'


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_variants_keep_preparation_complete_turns_and_final_support(kind):
    path = Path(f'artifacts/robot_motions_{kind}.json')
    catalog = json.loads((ROOT / path).read_text())['motions']
    motions = {motion['name']: motion for motion in catalog}
    assert len(motions) == len(catalog)
    before = json.loads((ROOT / 'artifacts/20261111_goal_exit_recovery_update/before' / path).read_text())['motions']
    assert all(motions[motion['name']] == motion for motion in before
               if '+라인복귀' not in motion['name'])
    for base, count, pose in [('t골대후제자리우회전', 5, '제우오들25'),
                              ('t골대후제자리우회전', 6, '제우오들25'),
                              ('p골대후제자리좌회전', 3, 'tt제좌왼들45'),
                              ('p골대후제자리좌회전', 4, 'tt제좌왼들45')]:
        original, variant = motions[base], motions[f'{base}({count}회)']
        assert variant['frames'] == original['frames'][:2 + 2 * count]
        assert sum(frame['name'] == pose for frame in variant['frames']) == count
        for key in ('repeat_count', 'playback_speed', 'start_pose', 'end_pose', 'completion'):
            assert variant[key] == original[key]
        for key in ('angles', 'torques'):
            assert variant['frames'][-1][key] == original['frames'][-1][key]
        end_ms = max(frame['start_ms'] + frame['time_ms'] for frame in variant['frames'])
        assert variant['max_seq_ms'] == end_ms
        assert end_ms / variant['playback_speed'] + variant['completion']['settle_timeout_ms'] < FakeBridge.DEFAULT_TIMEOUT_MS
    assert set(ALIASES.values()) <= motions.keys()
