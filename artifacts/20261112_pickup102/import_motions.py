"""Reimport only the two requested pickup motions from the preserved source."""

import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

NAMES = ('r공잡기 400', 'r공잡기500')


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source_list = json.loads(source_path.read_text())['motions']
    source = {m['name']: m for m in source_list}
    assert len(source) == len(source_list)
    report = {'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
              'motions': list(NAMES), 'changed': {}, 'duration_seconds': {}}
    for kind in ('pc', 'runtime'):
        path = ROOT / f'artifacts/robot_motions_{kind}.json'
        payload = json.loads(path.read_text())
        current = {m['name']: m for m in payload['motions']}
        assert len(current) == len(payload['motions'])
        assert set(NAMES) <= current.keys()
        replacements = {name: (source[name] if kind == 'pc'
                              else runtime_motion_from_source(source[name])) for name in NAMES}
        report['changed'][kind] = [name for name in NAMES if current[name] != replacements[name]]
        payload['motions'] = [replacements.get(m['name'], m) for m in payload['motions']]
        if report['changed'][kind]:
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
        for name, motion in replacements.items():
            report['duration_seconds'][name] = max(
                f['start_ms'] + f['time_ms'] for f in motion['frames']
            ) * motion['repeat_count'] / motion['playback_speed'] / 1000
    (UPDATE / 'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
