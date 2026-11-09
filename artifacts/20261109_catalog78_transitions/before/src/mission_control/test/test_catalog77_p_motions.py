"""Check imported targets, dispatch aliases and reservation clocks without motors."""

import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261108_catalog77_p_motions'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
@pytest.mark.parametrize('name', MANIFEST['imports'])
def test_imported_joint_targets_timing_and_speed(kind, name):
    source_name, count = MANIFEST['derived'].get(name, (name, SOURCE.get(name, {}).get('repeat_count')))
    expected = copy.deepcopy(SOURCE[source_name])
    expected.update(name=name, repeat_count=count)
    for frame in expected['frames']:
        frame['name'] = MANIFEST['pose_renames'].get(frame['name'], frame['name'])
    for key in ('start_pose', 'end_pose'):
        expected[key] = MANIFEST['pose_renames'].get(expected[key], expected[key])
    if kind == 'runtime':
        expected['completion']['position_tolerance_deg'] = 5.0
        if name.startswith(('p제자리좌회전', 'p미세')):
            expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
    assert catalog(ROOT / f'artifacts/robot_motions_{kind}.json')[name] == expected


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_other_data_only_changes_documented_speeds_and_transition_endpoints(kind):
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    assert set(after) == (set(before) - set(MANIFEST['renames'])) | set(MANIFEST['imports'])
    for name, old in before.items():
        if name in MANIFEST['renames']:
            continue
        expected = copy.deepcopy(old)
        source_name = MANIFEST['speed_sources'].get(name, name)
        if source_name in SOURCE:
            expected['playback_speed'] = SOURCE[source_name]['playback_speed']
        if name in MANIFEST['endpoint_updates']:
            target = SOURCE[MANIFEST['endpoint_updates'][name]]['frames'][-1]
            expected['frames'][-1]['angles'] = target['angles']
            expected['frames'][-1]['name'] = target['name']
            expected['end_pose'] = target['name']
        assert after[name] == expected, name


def test_every_alias_and_bridge_dispatch_resolves_with_counts_preserved():
    before_aliases = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    before = catalog(UPDATE / 'before/artifacts/robot_motions_runtime.json')
    assert ALIASES == {a: MANIFEST['renames'].get(n, n) for a, n in before_aliases.items()}
    assert set(ALIASES.values()) <= RUNTIME.keys()
    for alias, name in ALIASES.items():
        assert RUNTIME[name]['repeat_count'] == before[before_aliases[alias]]['repeat_count'], alias
    for alias in Bridge.ACTION_TO_MOTION_ID.values():
        assert alias in ALIASES, alias


def runtime_duration(alias):
    motion = RUNTIME[ALIASES[alias]]
    cycle = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000
    return cycle * motion['repeat_count'] / motion['playback_speed']


def test_reservation_and_capture_durations_match_new_playback():
    estimates = list(PICKUP_WALK_DURATIONS.items()) + list(Node.BALL_RECOVERY_DURATION_SEC.items())
    for table in (Node.LINE_MOTION_CAPTURE_CONFIG, Node.LINE_TURN_CAPTURE_CONFIG,
                  Node.POST_SHOT_ALIGN_CAPTURE_CONFIG):
        for action, (seconds, _) in table.items():
            alias = Bridge.motion_id_for_action(action)
            if alias in ALIASES:
                estimates.append((alias, seconds))
    for table in (Node.GOAL_FORWARD_DURATION_SEC, Node.GOAL_CRAB_DURATION_SEC,
                  Node.APPROACH_WALK_DURATION_SEC):
        for action, seconds in table.items():
            alias = Bridge.motion_id_for_action(action)
            if table is Node.APPROACH_WALK_DURATION_SEC and action == 'STRAIGHT':
                alias = 'line_forward_4'
            if alias in ALIASES:
                estimates.append((alias, seconds))
    for alias in ALIASES:
        if alias.startswith('ball_align_'):
            estimates.append((alias, duration_sec(alias)))
    for alias, seconds in estimates:
        assert seconds == pytest.approx(runtime_duration(alias)), alias


def test_preparations_reach_new_fine_support_without_ambiguous_startup_pose():
    for name, fine in MANIFEST['endpoint_updates'].items():
        assert RUNTIME[name]['frames'][-1]['angles'] == RUNTIME[fine]['frames'][-1]['angles']
    frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == '오뒤무게중심앞']
    assert frames and all(f['angles'] == frames[0]['angles'] for f in frames)
