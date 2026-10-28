"""Keep 1x speed changes separate from source frame holds and queue timing."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_decision_node import MotionDecisionNode

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_forward_speed1_boundary'
NAMES = set(json.loads((UPDATE / 'speed_update.json').read_text())['motions'])


@pytest.mark.parametrize('filename', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
def test_only_requested_motion_speeds_change(filename):
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    actual = json.loads((ROOT / 'artifacts' / filename).read_text())
    expected = copy.deepcopy(before)
    assert len(NAMES) == 20
    for motion in expected['motions']:
        if motion['name'] in NAMES:
            motion['playback_speed'] = 1.0
    assert actual == expected


def test_capture_timing_matches_deployed_motion_speeds():
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    def duration(alias):
        m = motions[aliases[alias]]
        return max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000
    for action, count in [('STRAIGHT', 6), ('STRAIGHT_1', 2), ('STRAIGHT_2', 4), ('STRAIGHT_3', 6), ('STRAIGHT_4', 8)]:
        assert MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(duration(f'line_forward_{count}'))
    for action, side in [('LEFT', 'left'), ('RIGHT', 'right')]:
        assert MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(duration(f'line_recovery_{side}_4'))
    for action, count in [('GOAL_CAMERA_90_FORWARD', 6), ('GOAL_CAMERA_90_FORWARD_1', 2), ('GOAL_CAMERA_90_FORWARD_2', 4)]:
        assert MotionDecisionNode.GOAL_FORWARD_DURATION_SEC[action] == pytest.approx(duration(f'goal_camera_90_forward_{count}'))
    for count in range(1, 5):
        assert MotionDecisionNode.GOAL_FORWARD_DURATION_SEC[f'GOAL_CAMERA90_FINE_FORWARD_{count}'] == pytest.approx(duration(f'goal_camera_90_fine_forward_{count}'))
