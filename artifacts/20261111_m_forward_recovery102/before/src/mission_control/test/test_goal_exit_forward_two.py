"""Check the forward-two exit revision against the immediately preceding catalog."""

import json
from pathlib import Path

import pytest

from mission_control.mission_phase_manager import MissionPhaseManager


ROOT = Path(__file__).resolve().parents[3]
CHANGED_NAMES = {
    't골대후제자리우회전(6회)' + '+라인복귀좌회전45도(4회)' * 2,
    'p골대후제자리좌회전(4회)' + '+라인복귀우회전45도(4회)' * 2,
}


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_composite_bodies_change(kind):
    relative = f'artifacts/robot_motions_{kind}.json'
    before = json.loads((ROOT / 'artifacts/20261111_goal_exit_forward2/before' / relative).read_text())
    after = json.loads((ROOT / relative).read_text())
    old = {m['name']: m for m in before['motions']}
    new = {m['name']: m for m in after['motions']}
    assert old.keys() == new.keys()
    assert {name for name in old if old[name] != new[name]} == CHANGED_NAMES
    assert {k: v for k, v in before.items() if k != 'motions'} == {
        k: v for k, v in after.items() if k != 'motions'}


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [5, 6, 7, 8])
def test_far_region_exit_selection(section, region):
    manager = MissionPhaseManager(initial_phase='POST_SHOT_TURN')
    manager.ball_sections_processed = section
    manager.last_completed_shot_region = region
    expected = 'RIGHT_5' if section == 1 else 'LEFT_3'
    assert manager.post_shot_turn_action() == f'POST_SHOT_EXIT_{expected}_FORWARD_4'
