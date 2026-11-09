"""Replace only camera45/count6 with the user's source74 motion."""

import copy
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

NAME = 't제자리좌회전45도(6회)'
source_path = HERE / 'source_robot_motions.json'
source = next(m for m in json.loads(source_path.read_text())['motions'] if m['name'] == NAME)
pose_renames = {
    't제좌왼들45도': 't제좌왼들45도(catalog74_left6)',
    '오뒤무게중심앞': '오뒤무게중심앞(catalog75_left)',
}
for kind in ('pc', 'runtime'):
    path = Path(f'artifacts/robot_motions_{kind}.json')
    catalog = json.loads((HERE / 'before' / path).read_text())
    replacement = copy.deepcopy(source)
    for frame in replacement['frames']:
        frame['name'] = pose_renames[frame['name']]
    for key in ('start_pose', 'end_pose'):
        replacement[key] = pose_renames[replacement[key]]
    if kind == 'runtime':
        replacement = runtime_motion_from_source(replacement)
    matches = [i for i, m in enumerate(catalog['motions']) if m['name'] == NAME]
    assert len(matches) == 1
    catalog['motions'][matches[0]] = replacement
    (ROOT / path).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
manifest = {'motion': NAME, 'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
            'pose_renames': pose_renames, 'runtime_speed': .855, 'cycle_ms': 295}
(HERE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
