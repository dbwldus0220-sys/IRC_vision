"""Eight scoring regions, calibrated approach and unchanged shot lifecycle."""

import copy
from decimal import Decimal
import json
import math
from pathlib import Path
import runpy

import pytest
import yaml

from step.goal_scoring import goal_shot_region
from step.goal_navigation_planner import GoalNavigationPlanner
from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_decision_node import MotionDecisionNode
from test_catalog64_goal_prepare import goal_node, prepare
from test_mission_phase_flow import score_ready_goal
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message
from test_shot_start_posture import finish_sequence

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261109_goal_regions_5_8'
NAMES = {5: 'r골넣기 5번구역', 6: 'r골넣기 6번구역',
         7: 'r골넣기7번구역', 8: 'r골넣기8번구역'}
BANDS = [(0.590, 0.630, -80, 8), (0.630, 0.660, -80, 5),
         (0.660, 0.700, -80, 6), (0.700, 0.760, -70, 7)]


def sample(depth, offset=-40, confirmed=True):
    return {**score_ready_goal(), 'depth_m': depth, 'offset_x_px': offset,
            'bearing_deg': math.degrees(math.atan(offset / 900)),
            'lateral_offset_m': depth * offset / 900,
            'camera_fx_px': 900., 'score_now': confirmed}


@pytest.mark.parametrize('lower,upper,left,region', BANDS)
def test_all_new_region_edges_and_confirmation(lower, upper, left, region):
    for depth in (lower, (lower + upper) / 2, upper - .000001):
        for offset in (left, -40, 0):
            assert goal_shot_region(depth, offset) == region
            for confirmed in (False, True):
                command = GoalNavigationPlanner().plan(sample(depth, offset, confirmed))
                assert command.shot_region == region
                assert command.action == ('SHOT' if confirmed else 'WAIT_SCORE_CONFIRMATION')
        for offset in (left - .001, .001):
            assert goal_shot_region(depth, offset) is None
            command = GoalNavigationPlanner().plan(sample(depth, offset))
            assert command.action != 'SHOT'
            assert command.action.endswith('LEFT' if offset < left else 'RIGHT')
    assert goal_shot_region(upper, -40) != region


def test_existing_regions_are_identical_to_before_for_every_millimeter():
    previous = runpy.run_path(str(UPDATE / 'before/src/step/step/goal_scoring.py'))['goal_shot_region']
    for mm in range(400, 590):
        for px in (-121, -120, -100.001, -100, -.001, 0, .001, 100, 100.001, 120, 121):
            assert goal_shot_region(mm / 1000, px) == previous(mm / 1000, px)


@pytest.mark.parametrize('region,depth', [(8, .61), (5, .645), (6, .68), (7, .73)])
@pytest.mark.parametrize('previous_shots', [0, 1])
def test_new_shots_dispatch_and_keep_first_second_exit_route(region, depth, previous_shots):
    command = GoalNavigationPlanner().plan(sample(depth))
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_fine_forward_1'
    bridge.goal_fine_forward_completed = True
    bridge.goal_crab_completed = True
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source='goal', source_command=command.to_dict()))
    finish_sequence(bridge)
    assert [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)] == [f'goal_shot_region_{region}']
    assert not bridge.goal_fine_forward_completed and not bridge.goal_crab_completed
    manager = MissionPhaseManager(initial_phase='GOAL_APPROACH')
    manager.shots_executed = manager.shots_completed = previous_shots
    manager.ball_sections_processed = previous_shots
    assert manager.start_special_action('SHOT', 100)
    for _ in range(2):
        manager.handle_motion_status('SHOT', 100, 'RUNNING', motion_id=f'goal_shot_region_{region}')
    manager.handle_motion_status('SHOT', 100, 'SUCCEEDED', motion_id=f'goal_shot_region_{region}')
    assert manager.shots_executed == manager.shots_completed == previous_shots + 1
    assert manager.current_phase == 'POST_SHOT_TURN'
    action = 'POST_SHOT_TURN_RIGHT_5' if previous_shots == 0 else 'POST_SHOT_TURN_LEFT_3'
    assert manager.post_shot_turn_action() == action
    assert manager.handle_post_shot_motion(action, 'SUCCEEDED')
    assert manager.current_phase == 'POST_SHOT_LINE_ALIGN'


@pytest.mark.parametrize('depth,region', [(.61, 8), (.645, 5), (.68, 6), (.73, 7)])
def test_region_reselected_after_same_one_second_observation(goal_node, depth, region):
    node, clock = goal_node
    prepare(node, clock)
    for now in (12., 12.5, 12.999):
        clock[0] = now
        assert all(c['action'] == 'WAIT' for c in node.publish_vision(goal=sample(depth)))
    clock[0] = 13.
    command = node.publish_vision(goal=sample(depth))[-1]
    assert command['action'] == 'SHOT'
    assert command['source_command']['shot_region'] == region
    assert MotionDecisionNode.SHOT_REGION_OBSERVATION_SEC == 1.0


@pytest.mark.parametrize('depth,offset', [(.760, -40), (.61, .001), (.73, -70.001)])
def test_out_of_region_after_observation_does_not_use_previous_region(goal_node, depth, offset):
    node, clock = goal_node
    prepare(node, clock)
    clock[0] = 13.
    assert all(c['action'] != 'SHOT' for c in node.publish_vision(goal=sample(depth, offset)))


def test_target_projection_uses_630mm_and_minus40px_in_calibrated_coordinates():
    planner = GoalNavigationPlanner()
    assert planner.config.score_target_depth_m == .630
    for fx in (600., 900., 1200.):
        for depth in (.9, 1., 2.):
            # This goal X already equals the desired X at the stopping point.
            offset = -40 * .630 / depth
            info = sample(depth, offset, False)
            info.update(camera_fx_px=fx, lateral_offset_m=depth * offset / fx)
            command = planner.plan(info)
            assert command.target_steering_angle_deg == pytest.approx(0)
            assert command.target_lateral_error_m == pytest.approx(0)
            assert command.depth_error_m == pytest.approx(depth - .630)
            assert command.action == 'GOAL_CAMERA_90_FORWARD_2'
    centered = planner.plan(sample(1., 0, False))
    assert centered.target_lateral_error_m == pytest.approx(.630 * 40 / 900)
    assert centered.target_steering_angle_deg > 0


@pytest.mark.parametrize('invalid', [None, 0, -1, float('nan')])
def test_unavailable_calibration_at_center_cannot_guess_target_direction(invalid):
    info = sample(1., 0, False)
    info['camera_fx_px'] = invalid
    assert GoalNavigationPlanner().plan(info).action == 'WAIT'


def test_only_four_goal_motions_and_aliases_are_added():
    source = {m['name']: m for m in json.loads((UPDATE / 'source_robot_motions.json').read_text())['motions']}
    for kind in ('pc', 'runtime'):
        path = Path(f'artifacts/robot_motions_{kind}.json')
        before = {m['name']: m for m in json.loads((UPDATE / 'before' / path).read_text())['motions']}
        after = {m['name']: m for m in json.loads((ROOT / path).read_text())['motions']}
        assert after.keys() - before.keys() == set(NAMES.values())
        assert all(after[name] == motion for name, motion in before.items())
        for name in NAMES.values():
            expected = copy.deepcopy(source[name])
            if kind == 'runtime':
                expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
                expected['completion']['position_tolerance_deg'] = 5.
            assert after[name] == expected
            duration = max(f['start_ms'] + f['time_ms'] for f in expected['frames']) * expected['repeat_count'] / expected['playback_speed']
            assert duration + expected['completion']['settle_timeout_ms'] < FakeBridge.DEFAULT_TIMEOUT_MS
    runtime_names = set(after)
    alias_path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    before = yaml.safe_load((UPDATE / 'before' / alias_path).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / alias_path).read_text())['motion_aliases']
    assert after == {**before, **{f'goal_shot_region_{region}': name for region, name in NAMES.items()}}
    assert set(after.values()) <= runtime_names
