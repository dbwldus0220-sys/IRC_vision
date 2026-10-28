"""The requested pickup keeps source arms without altering other runtime motions."""

import copy
import hashlib
import json
from pathlib import Path
import runpy

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261009_pickup_source_arms_update'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())


def test_archived_pickup_matches_source_before_catalog51_update():
    source_path = ROOT / MANIFEST['source_file']
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = next(m for m in json.loads(source_path.read_text())['motions'] if m['name'] == '건공잡기')
    runtime = json.loads((ROOT / 'artifacts/20261009_catalog51_pickup_startup_update/robot_motions_runtime.before.json').read_text())
    pickup = next(m for m in runtime['motions'] if m['name'] == '건공잡기')
    normalize = runpy.run_path(str(ROOT / 'tools/upsert_motion_catalog.py'))['_with_runtime_policy']
    assert pickup == normalize(source)
    assert normalize(pickup) == pickup
    for original, actual in zip(source['frames'], pickup['frames'], strict=True):
        assert actual['angles'] == original['angles']
    before = json.loads((UPDATE / 'pickup.before.json').read_text())
    restored = copy.deepcopy(pickup)
    inverse_names = {new: old for old, new in MANIFEST['pose_renames'].items()}
    for frame, old in zip(restored['frames'], before['frames'], strict=True):
        frame['name'] = inverse_names.get(frame['name'], frame['name'])
        for motor in ('4', '5'):
            frame['angles'][motor] = old['angles'][motor]
    restored['end_pose'] = inverse_names.get(restored['end_pose'], restored['end_pose'])
    assert restored == before


def test_pickup_restoration_does_not_add_pose_name_conflicts():
    motions = json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    pickup = next(m for m in motions if m['name'] == '건공잡기')
    for frame in pickup['frames']:
        for motion in motions:
            for other in motion['frames']:
                if other['name'] == frame['name']:
                    assert other['angles'] == frame['angles']
