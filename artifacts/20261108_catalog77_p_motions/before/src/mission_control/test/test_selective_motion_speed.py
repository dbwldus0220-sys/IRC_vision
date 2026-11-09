"""Preserve excluded gaits and all motion data except the requested speed scale."""

import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261028_line_late_queue_speed'
BEFORE = json.loads((UPDATE / 'robot_motions_runtime.before.json').read_text())
CURRENT = json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())


@pytest.mark.parametrize('before,after', list(zip(BEFORE['motions'], CURRENT['motions'])),
                         ids=[m['name'] for m in BEFORE['motions']])
def test_requested_speed_scale_preserves_other_motion_fields(before, after):
    compact = before['name'].replace(' ', '')
    excluded = any(word in compact for word in ('전진', '라인복귀', '제자리좌회전', '제자리우회전'))
    expected_speed = before['playback_speed'] if excluded else before['playback_speed'] * .95
    assert after['playback_speed'] == pytest.approx(expected_speed, rel=0., abs=1e-12)
    assert {**after, 'playback_speed': before['playback_speed']} == before
    assert after['completion']['position_tolerance_deg'] == 5.


def test_speed_update_preserves_catalog_structure():
    assert len(CURRENT['motions']) == len(BEFORE['motions']) == 88
    assert {**CURRENT, 'motions': BEFORE['motions']} == BEFORE
