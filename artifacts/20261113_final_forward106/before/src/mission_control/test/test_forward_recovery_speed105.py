"""Ordinary gaits run at 1.05; fine walking and non-gait motion stages stay intact."""

import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.ball_fine_approach import duration_sec
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261112_forward_recovery_speed105'
DEPLOYED = ROOT / 'artifacts/20261112_pickup_prepare105/before'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from build_post_shot_forward_exits import GOAL_EXIT_RECOVERIES, FORWARD_EXIT_NAMES, build_exit
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_ordinary_speed_and_embedded_gait_timing_change(kind):
    relative = f'artifacts/robot_motions_{kind}.json'
    old, new = catalog(UPDATE / 'before' / relative), catalog(DEPLOYED / relative)
    assert new.keys() == old.keys()
    gaits = {name for name in old if name.startswith(('m전진', 'm라인복귀'))}
    assert gaits == set(MANIFEST['gaits'])
    assert sum(name.startswith('m전진') for name in gaits) == 12
    assert len(gaits) == 18
    for name in gaits:
        expected = copy.deepcopy(old[name])
        expected['playback_speed'] = 1.05
        assert new[name] == expected
    for name in old.keys() - set(MANIFEST['replaced']):
        assert new[name] == old[name], name
    for name, seconds in MANIFEST['duration_seconds'].items():
        m = new[name]
        assert seconds == pytest.approx(max(f['start_ms'] + f['time_ms'] for f in m['frames'])
                                        * m['repeat_count'] / m['playback_speed'] / 1000)
    assert new['m라인복귀우회전90']['frames'][-1]['angles']['0'] == 1.0


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_embedded_gaits_preserve_turns_holds_and_preparation_tail(kind):
    relative = f'artifacts/robot_motions_{kind}.json'
    old, new = catalog(UPDATE / 'before' / relative), catalog(DEPLOYED / relative)
    pc = catalog(DEPLOYED / 'artifacts/robot_motions_pc.json')
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery], blocks,
                              forward=pc['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert new[expected['name']] == expected
        prefix = len(new[turn]['frames'])
        assert expected['frames'][:prefix] == old[turn]['frames']
        end = max(f['start_ms'] + f['time_ms'] for f in new[turn]['frames'])
        assert (expected['frames'][prefix]['start_ms'] - end) / expected['playback_speed'] == 500

    name = 'n공에서 전진하고 후진준비자세'
    gait, prepare = new['m전진0도(2회)'], new[name]
    assert prepare['playback_speed'] == old[name]['playback_speed']
    for actual, source in zip(prepare['frames'][:4], gait['frames'], strict=True):
        expected = copy.deepcopy(source)
        start = round(source['start_ms'] / 1.05)
        end = round((source['start_ms'] + source['time_ms']) / 1.05)
        expected.update(start_ms=start, time_ms=end - start)
        assert actual == expected
    for actual, source in zip(prepare['frames'][4:], old[name]['frames'][4:], strict=True):
        expected = copy.deepcopy(source)
        expected['start_ms'] -= 24
        assert actual == expected
    assert prepare['max_seq_ms'] == 730


def test_all_reservation_estimates_and_aliases_match_catalog():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    assert (ROOT / relative).read_bytes() == (UPDATE / 'before' / relative).read_bytes()
    aliases = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(aliases.values()) <= motions.keys()

    def seconds(identifier):
        motion = motions[aliases[identifier]]
        return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000

    for action, (estimate, _) in Node.LINE_MOTION_CAPTURE_CONFIG.items():
        identifier = Bridge.ACTION_TO_MOTION_ID.get(action)
        if identifier in aliases and motions[aliases[identifier]]['name'] in MANIFEST['gaits']:
            assert estimate == pytest.approx(seconds(identifier)), action
    for action, estimate in Node.APPROACH_WALK_DURATION_SEC.items():
        # Ball/hurdle STRAIGHT explicitly selects four cycles in the bridge.
        identifier = 'line_forward_4' if action == 'STRAIGHT' else Bridge.ACTION_TO_MOTION_ID[action]
        assert estimate == pytest.approx(seconds(identifier)), action
    for action, estimate in Node.GOAL_FORWARD_DURATION_SEC.items():
        assert estimate == pytest.approx(seconds(Bridge.ACTION_TO_MOTION_ID[action])), action
    for identifier, estimate in Node.BALL_RECOVERY_DURATION_SEC.items():
        assert estimate == pytest.approx(seconds(identifier))
    for action, (estimate, _) in Node.POST_SHOT_EXIT_CAPTURE_CONFIG.items():
        assert estimate == pytest.approx(seconds(Bridge.ACTION_TO_MOTION_ID[action]))
    assert PICKUP_WALK_DURATIONS['ball_camera_down_forward_2'] == pytest.approx(seconds('ball_camera_down_forward_2'))
    assert duration_sec('ball_align_forward_2') == pytest.approx(seconds('ball_align_forward_2'))
