"""Source105 pickup transition stays isolated and cannot reserve another gait."""

import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.pickup_walk_reservation import PickupWalkReservation

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261112_pickup_prepare105'
NAME = 'n공에서 전진하고 후진준비자세'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    payload = json.loads(path.read_text())
    result = {m['name']: m for m in payload['motions']}
    assert len(result) == len(payload['motions'])
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_motion_changes_and_retains_source_timing(kind):
    path = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / path)
    after = catalog(ROOT / 'artifacts/20261113_final_forward106/before' / path)
    source_path = UPDATE / 'source_robot_motions.json'
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == manifest['source_sha256']
    source = catalog(source_path)[NAME]
    assert before.keys() == after.keys()
    assert {name for name in before if before[name] != after[name]} == {NAME}
    expected = runtime_motion_from_source(source) if kind == 'runtime' else source
    assert after[NAME] == expected
    frames = after[NAME]['frames']
    assert len(frames) == 6 and after[NAME]['repeat_count'] == 1
    assert after[NAME]['playback_speed'] == 1.0
    assert max(f['start_ms'] + f['time_ms'] for f in frames) == 939
    assert frames[4]['start_ms'] - frames[3]['start_ms'] - frames[3]['time_ms'] == 300
    assert frames[-1]['name'] == after[NAME]['end_pose'] == '미세후진오뒤(0도)'
    assert all(f['angles']['0'] == -64 for f in frames)
    alias = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    assert yaml.safe_load((ROOT / 'artifacts/20261113_final_forward106/before' / alias).read_text()) == yaml.safe_load(
        (UPDATE / 'before' / alias).read_text())


def test_preparation_running_clears_previous_walk_reservation():
    reservation = PickupWalkReservation()
    reservation.on_status(dict(status='RUNNING', motion_id='ball_camera_down_forward_2',
                               pickup_alignment_stage='initial', command_id=1, request_id=2),
                          10., 10_000_000_000)
    assert reservation.key is not None
    reservation.pending = True
    reservation.on_status(dict(status='RUNNING', motion_id='pickup_forward_to_backward_prepare',
                               pickup_alignment_stage='fine', command_id=1, request_id=3),
                          12., 12_000_000_000)
    assert reservation.key is None and not reservation.pending
    assert reservation.capture_after_ns is None and reservation.commit_after_ns is None
    assert reservation.decision(12., .7) is None
