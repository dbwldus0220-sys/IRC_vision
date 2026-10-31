"""Replay calibrated scoring zones through analysis, planning and SDK routing."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from step.goal_navigation_planner import GoalNavigationPlanner
from step.goal_scoring import goal_shot_region
from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages
from test_shot_start_posture import finish_sequence


def goal_info(**overrides):
    return {"detected": True, "confidence": .9, "depth_valid": True,
            "depth_m": .51, "offset_x_px": 0, "offset_x_norm": 0.0,
            "bearing_deg": 0.0, "score_now": True, **overrides}


@pytest.mark.parametrize('depth,offset,region', [
    (.430, 0, 1), (.430, 120, 1), (.509, 120, 1),
    (.430, -120, 4), (.509, -120, 4), (.509, -0.001, 4),
    (.510, 0, 2), (.510, .001, 2), (.510, 100, 2),
    (.510, -.001, 3), (.510, -100, 3),
    (.550, 0, 2), (.569, 100, 2), (.569, -100, 3),
    (.429999, 0, None), (.570, 0, None), (.571, 0, None),
    (.500, 120.001, None), (.500, -120.001, None),
    (.510, 120, None), (.510, -120, None),
    (.510001, 100, 2), (.510001, -100, 3),
    (.510001, 100.001, None), (.510001, -100.001, None),
    (.430001, 120, 1), (.430001, -120, 4),
    (.570, 100, None), (.570, -100, None),
    (.550, 120, None), (.550, -120, None),
    (float('nan'), 0, None), (.5, float('inf'), None),
])
def test_zone_depth_bounds_include_lower_and_exclude_upper(depth, offset, region):
    assert goal_shot_region(depth, offset) == region
    command = GoalNavigationPlanner().plan(goal_info(depth_m=depth, offset_x_px=offset))
    assert command.shot_region == region
    assert (command.action == 'SHOT') is (region is not None)


@pytest.mark.parametrize('depth,offset,bearing,action', [
    (.410, 230, 14.2, 'GOAL_CAMERA90_BACKWARD_1'),
    (.450, 230, 14.2, 'GOAL_CAMERA90_CRAB_RIGHT'),
    (.450, -230, -7.9, 'GOAL_CAMERA90_CRAB_LEFT'),
    (.510, 101, 6.0, 'GOAL_CAMERA90_CRAB_RIGHT'),
    (.510, -101, -6.0, 'GOAL_CAMERA90_CRAB_LEFT'),
    (.570, 0, 0.0, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (.510, 0, 0.0, 'SHOT'),
])
def test_motion_toward_nominal_target_until_zone_entry(depth, offset, bearing, action):
    decision = MotionDecisionPlanner().plan('GOAL_APPROACH', {'goal': goal_info(
        depth_m=depth, offset_x_px=offset, offset_x_norm=offset / 640,
        bearing_deg=bearing,
    )}, .1)
    assert decision.action == action
    assert decision.source_command['depth_error_m'] == pytest.approx(depth - .510)


def test_recorded_wait_geometry_recovers_then_shoots_in_entered_zone():
    planner = MotionDecisionPlanner()
    for depth, offset, action, region in [
        (.410, 230, 'GOAL_CAMERA90_BACKWARD_1', None),
        (.450, 230, 'GOAL_CAMERA90_CRAB_RIGHT', None),
        (.450, 110, 'SHOT', 1),
    ]:
        decision = planner.plan('GOAL_APPROACH', {'goal': goal_info(
            depth_m=depth, offset_x_px=offset, offset_x_norm=offset / 640,
            bearing_deg=14.2,
        )}, .1)
        assert decision.action == action
        assert decision.source_command['shot_region'] == region


@pytest.mark.parametrize('depth,offset,region', [
    (.430, 120, 1), (.509, -120, 4), (.510, 100, 2),
    (.569, -100, 3), (.510, 0, 2),
])
@pytest.mark.parametrize('previous_motion', [
    'goal_camera_90_fine_forward_1', 'goal_camera_90_crab_right',
    'goal_camera_90_crab_left',
    'goal_camera_90_turn_right_1',
])
def test_confirmed_region_reaches_selected_motion_after_existing_preparation(
    depth, offset, region, previous_motion,
):
    planner = MotionDecisionPlanner()
    for index in range(3):
        info = goal_info(depth_m=depth, offset_x_px=offset, score_now=(index == 2))
        # A large bearing must not override an in-zone scoring candidate.
        info['bearing_deg'] = 45.0
        decision = planner.plan('GOAL_APPROACH', {'goal': info}, .1)
        assert decision.action == ('SHOT' if index == 2 else 'WAIT_SCORE_CONFIRMATION')
    assert decision.source_command['shot_region'] == region
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous_motion
    bridge.navigation_command_callback(navigation_message(
        action=decision.action, source_command=decision.source_command,
    ))
    finish_sequence(bridge)
    motions = [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)]
    expected = [f'goal_shot_region_{region}']
    if 'turn' in previous_motion:
        expected.insert(0, 'goal_forward_to_default')
    elif previous_motion.startswith('goal_camera_90_crab_'):
        side = previous_motion.rsplit('_', 1)[1]
        expected.insert(0, f'goal_crab_{side}_to_turn_90')
    assert motions == expected
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('region', [0, 5, True, 1.0, '2'])
def test_invalid_region_cannot_execute_a_shot(region):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source_command={'shot_region': region},
    ))
    assert not bridge.executor_request_publisher.messages
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'INVALID_SHOT_REGION'


@pytest.mark.parametrize('region', range(1, 5))
@pytest.mark.parametrize('previous_shots,turn', [(0, 'RIGHT_9'), (1, 'LEFT_4')])
def test_region_shots_count_once_and_keep_post_shot_route(region, previous_shots, turn):
    manager = MissionPhaseManager(initial_phase='GOAL_APPROACH')
    manager.shots_executed = manager.shots_completed = previous_shots
    manager.ball_sections_processed = previous_shots
    assert manager.start_special_action('SHOT', 100)
    manager.handle_motion_status('SHOT', 100, 'RUNNING', motion_id='goal_forward_to_default')
    assert manager.shots_executed == previous_shots
    for _ in range(2):
        manager.handle_motion_status('SHOT', 100, 'RUNNING', motion_id=f'goal_shot_region_{region}')
    assert manager.shots_executed == previous_shots + 1
    manager.handle_motion_status('SHOT', 100, 'SUCCEEDED', motion_id=f'goal_shot_region_{region}')
    assert manager.shots_completed == previous_shots + 1
    assert manager.current_phase == 'POST_SHOT_TURN'
    assert manager.post_shot_turn_action() == f'POST_SHOT_TURN_{turn}'
    assert manager.handle_post_shot_motion(manager.post_shot_turn_action(), 'SUCCEEDED')
    assert manager.current_phase == 'POST_SHOT_LINE_ALIGN'


def test_import_preserves_source_targets_and_applies_speed_once():
    root = Path(__file__).resolve().parents[3]
    def catalog(relative):
        return {m['name']: m for m in json.loads((root / relative).read_text())['motions']}
    source = catalog('artifacts/20261030_goal_shot_regions/source_shots.json')
    pc = catalog('artifacts/robot_motions_pc.json')
    runtime = catalog('artifacts/robot_motions_runtime.json')
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    for region in range(1, 5):
        name = f'R골넣기{region}번구역'
        assert aliases[f'goal_shot_region_{region}'] == name
        assert pc[name] == source[name]
        expected = copy.deepcopy(source[name])
        expected['playback_speed'] = .95
        expected['completion']['position_tolerance_deg'] = 5.0
        assert runtime[name] == expected
        duration_ms = max(f['start_ms'] + f['time_ms'] for f in expected['frames']) / .95
        assert duration_ms + expected['completion']['settle_timeout_ms'] < FakeBridge.DEFAULT_TIMEOUT_MS
    assert aliases['post_shot_default_turn_right'] == 'R골대후제자리우회전'
    assert aliases['post_shot_default_turn_left'] == 'R골대후제자리좌회전'
