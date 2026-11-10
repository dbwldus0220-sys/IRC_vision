"""Verify source87 check-pose replacement and the recorded correction pause."""

import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_continuous_approach import clock
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog87_check_pose'
ALIAS = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    return json.loads(path.read_text())


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_check_pose_is_replaced(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    expected = catalog(UPDATE / 'before' / relative)
    source = next(m for m in catalog(UPDATE / 'source_robot_motions.json')['motions']
                  if m['name'] == 'n공확인자세')
    index = next(i for i, m in enumerate(expected['motions']) if m['name'] == 'p공확인자세')
    expected['motions'][index] = (copy.deepcopy(source) if kind == 'pc'
                                 else runtime_motion_from_source(source))
    actual = catalog(ROOT / relative)
    assert actual == expected
    names = [m['name'] for m in actual['motions']]
    assert len(names) == len(set(names)) == 156
    assert 'p공확인자세' not in names


def test_alias_pose_and_completion_budget():
    aliases = yaml.safe_load((ROOT / ALIAS).read_text())['motion_aliases']
    before = yaml.safe_load((UPDATE / 'before' / ALIAS).read_text())['motion_aliases']
    assert aliases == {**before, 'pickup_grasp_check_pose': 'n공확인자세'}
    runtime = {m['name']: m for m in catalog(ROOT / 'artifacts/robot_motions_runtime.json')['motions']}
    assert set(aliases.values()) <= runtime.keys()
    check = runtime[aliases[Bridge.PICKUP_GRASP_CHECK_MOTION_ID]]
    assert check['start_pose'] == check['end_pose'] == check['frames'][0]['name'] == 'n공확인자세'
    assert check['playback_speed'] == 1.0
    duration = max(f['start_ms'] + f['time_ms'] for f in check['frames'])
    duration *= check['repeat_count'] / check['playback_speed']
    assert duration == 400
    assert duration + check['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    assert check['frames'][0]['angles']['2'] == -9.0
    for motion in runtime.values():
        if '전진' in motion['name'] or '라인복귀' in motion['name']:
            assert motion['playback_speed'] == 1.0


def test_third_line_recovery_waits_half_second_after_second_turn(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'post_ball_line_turn_right_1'
    bridge.stationary_since = clock[0]
    bridge.navigation_command_callback(navigation_message(
        action='RECOVER_RIGHT_TURN_RIGHT_4', command_id=302,
        phase='LINE_TRACK_AFTER_PICKUP', source='line',
        source_command={'post_pickup_line_index': 3, 'goal_head_prepare_id': 302},
    ))
    assert not decoded_messages(bridge.executor_request_publisher)
    assert bridge.transition_dwell_until == clock[0] + .5
    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]['motion_id'] == '__CORRECTION_PRE_DWELL__'
    clock[0] += .499
    bridge._check_atomic_dwell()
    assert not decoded_messages(bridge.executor_request_publisher)
    clock[0] += .001
    bridge._check_atomic_dwell()
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 1
    assert requests[0]['command_id'] == 302
    assert requests[0]['motion_id'] == 'line_recovery_right_4'
