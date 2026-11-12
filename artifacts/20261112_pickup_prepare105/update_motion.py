"""Import only the requested source105 pickup transition, preserving other motions."""

import hashlib
import json
from pathlib import Path
import sys

UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
NAME = 'n공에서 전진하고 후진준비자세'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source = json.loads(source_path.read_text())
    matches = [motion for motion in source['motions'] if motion['name'] == NAME]
    assert len(matches) == 1
    motion = matches[0]
    assert motion['repeat_count'] == 1 and len(motion['frames']) == 6
    for kind in ('pc', 'runtime'):
        path = ROOT / f'artifacts/robot_motions_{kind}.json'
        payload = json.loads(path.read_text())
        assert sum(item['name'] == NAME for item in payload['motions']) == 1
        replacement = runtime_motion_from_source(motion) if kind == 'runtime' else motion
        payload['motions'] = [replacement if item['name'] == NAME else item
                              for item in payload['motions']]
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    manifest = {
        'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
        'replaced': [NAME],
        'motion_id': 'pickup_forward_to_backward_prepare',
        'duration_sec': max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
                        * motion['repeat_count'] / motion['playback_speed'] / 1000,
        'completion': 'Executor success, followed by the existing total 1-second hold before backward.',
    }
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
