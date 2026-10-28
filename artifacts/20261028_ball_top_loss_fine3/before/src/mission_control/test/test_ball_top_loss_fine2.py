"""Top-edge reacquisition takes exactly two fine cycles in the current posture."""

import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_command_gate import GENERAL_ACTIONS
from test_ball_lost_turn_revision import tracked_planner, plan, MISSING


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('catalog_name', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
@pytest.mark.parametrize('source,alias', [
    ('건미세45도(5회)', 'ball_top_loss_fine_forward_45_2'),
    ('건미세0도', 'pickup_top_loss_fine_forward_0_2'),
])
def test_two_cycle_catalog_changes_only_name_and_repeat_count(catalog_name, source, alias):
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = json.loads((ROOT / 'artifacts' / catalog_name).read_text())['motions']
    motions = {m['name']: m for m in catalog}
    assert len(motions) == len(catalog)
    original = motions[source]
    assert original['repeat_count'] == 5
    expected = {**original, 'name': aliases[alias], 'repeat_count': 2}
    assert motions[aliases[alias]] == expected
    assert alias in Bridge.FINE_FORWARD_MOTION_IDS


@pytest.mark.parametrize('stage', ['normal', 'approach', 'INITIAL', 'FINE'])
def test_top_loss_selects_fine_search_once_per_visible_observation(stage):
    planner = tracked_planner('RIGHT')
    planner.last_ball_inside_image = False
    planner.last_ball_top_edge_ratio = .05
    planner.pickup_last_visible_bottom_distance_px = 300
    decision = plan(planner, stage, MISSING)
    expected = (f'BALL_PICKUP_{stage}_SEARCH_FORWARD'
                if stage in {'INITIAL', 'FINE'} else 'BALL_LOST_FINE_FORWARD_2')
    assert decision.valid and decision.action == expected
    assert decision.reason == 'ball_lost_at_top_edge_forward_search'
    assert decision.source_command['lost_ball_top_forward']
    if stage in {'normal', 'approach'}:
        assert decision.action in GENERAL_ACTIONS
        assert Bridge.motion_id_for_action(decision.action) == 'ball_top_loss_fine_forward_45_2'
    planner.mark_ball_top_loss_forward_sent()
    assert plan(planner, stage, MISSING).action != expected


def test_preparation_uses_camera_zero_after_two_cycle_pickup_search():
    from test_motion_command_bridge import FakeBridge
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'pickup_top_loss_fine_forward_0_2'
    assert bridge._turn_prepare_motion('pickup_camera_down_turn_right_2') == 'fine_to_turn_ready_0'
