"""Check selective catalog63 replacement without hardware or transition changes."""

import copy
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_catalog63_crab_fine'
NAMES = {
    'r미세45도', '유미세오옆꽃게0도', '유미세왼옆꽃게0도',
    '유미세오옆꽃게90도', '유미세왼옆꽃게90도',
}


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    assert len(motions) == len({m['name'] for m in motions})
    return {m['name']: m for m in motions}


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_five_requested_motions_replaced_with_source63(filename):
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    assert hashlib.sha256((UPDATE / 'source63.json').read_bytes()).hexdigest() == manifest['source_sha256']
    assert set(manifest['motion_names']) == NAMES
    source = catalog(UPDATE / 'source63.json')
    before = catalog(UPDATE / 'before/artifacts' / filename)
    # Preserve the five-gait import contract before the later transition import.
    actual = catalog(ROOT / 'artifacts/20261031_catalog63_transitions/before/artifacts' / filename)
    assert actual.keys() == before.keys()
    for name, motion in actual.items():
        expected = copy.deepcopy(source[name] if name in NAMES else before[name])
        if name in NAMES and 'runtime' in filename:
            expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
            expected['completion']['position_tolerance_deg'] = 5.0
        # Compare complete frames, including angles, torques and timing.
        assert motion == expected, name


def test_transition_routes_are_unchanged():
    path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    assert (ROOT / 'artifacts/20261031_catalog63_transitions/before' / path).read_bytes() == (UPDATE / 'before' / path).read_bytes()
