"""Rebuild source99 walking and goal exits from immutable before snapshots."""

import copy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, EXIT_RECOVERIES


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source = {m['name']: m for m in read(source_path)['motions']}
    variants = {n: copy.deepcopy(m) for n, m in source.items()
                if n.startswith(('n전진', 'n라인복귀'))}
    assert source['n전진45도(6회)']['frames'] == source['n전진45도(4회)']['frames']
    derived = {}
    for count in (6, 8):
        name = f'n전진45도({count}회)'
        variants[name] = copy.deepcopy(source['n전진45도(4회)'])
        variants[name].update(name=name, repeat_count=count)
        derived[name] = ['n전진45도(4회)', count]
    pose_renames = {'오뒤무게중심앞': '오뒤무게중심앞(catalog80)'}
    for motion in variants.values():
        for frame in motion['frames']:
            frame['name'] = pose_renames.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            motion[key] = pose_renames.get(motion[key], motion[key])
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    composites = [build_exit(pc[turn], variants[recovery]) for turn, recovery in EXIT_RECOVERIES]
    replacements = {**variants, **{m['name']: m for m in composites}}
    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    imports=list(variants), derived=derived, pose_renames=pose_renames,
                    composites=[m['name'] for m in composites], duration_seconds={})
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        assert replacements.keys() <= {m['name'] for m in payload['motions']}
        merged = []
        for old in payload['motions']:
            replacement = replacements.get(old['name'])
            motion = old if replacement is None else (
                runtime_motion_from_source(replacement) if kind == 'runtime' else replacement)
            merged.append(motion)
            if kind == 'runtime' and replacement is not None:
                def duration(m):
                    return max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000
                manifest['duration_seconds'][motion['name']] = dict(before=duration(old), after=duration(motion))
        payload['motions'] = merged
        write(ROOT / relative, payload)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
