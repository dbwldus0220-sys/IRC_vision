"""Rebuild the deployed forward/recovery variants from source89 exactly once."""

import copy
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    previous = read(ROOT / 'artifacts/20261110_catalog88_forward_recovery/manifest.json')
    source = {m['name']: m for m in read(UPDATE / 'source_robot_motions.json')['motions']}
    manifest = {
        'source_sha256': hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest(),
        'renames': {},
        'derived': previous['derived'],
        'pose_renames': previous['pose_renames'],
        'imports': previous['imports'],
        'duration_seconds': {},
        'changed_aliases': {},
        'runtime_speed_policy': 'Original source89 playback speed: 1.0 for all 15 deployed forward/recovery motions.',
    }
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        for index, old in enumerate(payload['motions']):
            name = old['name']
            if name not in manifest['imports']:
                continue
            source_name, count = manifest['derived'].get(name, (name, old['repeat_count']))
            motion = copy.deepcopy(source[source_name])
            motion.update(name=name, repeat_count=count)
            poses = manifest['pose_renames'][name]
            for frame in motion['frames']:
                frame['name'] = poses.get(frame['name'], frame['name'])
            for field in ('start_pose', 'end_pose'):
                motion[field] = poses.get(motion[field], motion[field])
            if kind == 'runtime':
                motion = runtime_motion_from_source(motion)
                def seconds(value):
                    return max(f['start_ms'] + f['time_ms'] for f in value['frames']) * value['repeat_count'] / value['playback_speed'] / 1000
                manifest['duration_seconds'][name] = {'before': seconds(old), 'after': seconds(motion)}
            expected = copy.deepcopy(old)
            expected['playback_speed'] = motion['playback_speed']
            assert motion == expected, f'Unexpected change beyond source speed: {name}'
            payload['motions'][index] = motion
        write(ROOT / relative, payload)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
