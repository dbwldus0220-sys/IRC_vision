"""Reapply source103 right turns without changing unrelated deployed motions."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from build_post_shot_forward_exits import build_exit, build_turn_variant
from upsert_motion_catalog import runtime_motion_from_source


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source = {m['name']: m for m in json.loads(source_path.read_text())['motions']}
    paths = {kind: ROOT / f'artifacts/robot_motions_{kind}.json'
             for kind in ('pc', 'runtime')}
    payloads = {kind: json.loads(path.read_text()) for kind, path in paths.items()}
    pc = {m['name']: m for m in payloads['pc']['motions']}
    variants, origins = {}, {}
    for camera in (0, 45, 90):
        for count in (1, 2, 3, 4, 5, 6, 7, 9):
            name = f't제자리우회전{camera}도({count}회)'
            origin = name if name in source else f't제자리우회전{camera}도(2회)'
            motion = copy.deepcopy(source[origin])
            motion.update(name=name, repeat_count=count)
            variants[name], origins[name] = motion, origin
    for name in ('t공후 제자리우회전', 't골대후제자리우회전'):
        variants[name], origins[name] = copy.deepcopy(source[name]), name
    for motion in variants.values():
        # The startup pose has the same source name but different joint targets.
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog70)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog70)'
    for count in (5, 6):
        motion = build_turn_variant(variants['t골대후제자리우회전'], count)
        variants[motion['name']] = motion
    for name in ('t골대후제자리우회전', 't골대후제자리우회전(6회)'):
        motion = build_exit(variants[name], pc['m라인복귀우회전45도'],
                            forward=pc['m전진45(2회)'] if name.endswith('(6회)') else None)
        variants[motion['name']] = motion

    aliases = yaml.safe_load((ROOT /
        'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    changed = {}
    for kind, payload in payloads.items():
        names = [m['name'] for m in payload['motions']]
        assert len(names) == len(set(names))
        assert variants.keys() <= set(names)
        assert set(aliases.values()) <= set(names)
        changed[kind] = []
        for index, previous in enumerate(payload['motions']):
            name = previous['name']
            if name not in variants:
                continue
            updated = (runtime_motion_from_source(variants[name]) if kind == 'runtime'
                       else variants[name])
            if updated != previous:
                changed[kind].append(name)
            payload['motions'][index] = updated
    # Validate both catalogs before writing either one; identical imports are no-ops.
    for kind, payload in payloads.items():
        if changed[kind]:
            backup = UPDATE / 'before' / paths[kind].name
            backup.parent.mkdir(exist_ok=True)
            if not backup.exists():
                backup.write_bytes(paths[kind].read_bytes())
            paths[kind].write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    manifest = {
        'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
        'source_mapping': origins,
        'verified_motions': list(variants),
        'changed_motions': changed,
        'aliases': {key: value for key, value in aliases.items() if value in variants},
        'pose_renames': {'오뒤무게중심앞': '오뒤무게중심앞(catalog70)'},
        'duration_seconds': {
            name: max(f['start_ms'] + f['time_ms'] for f in m['frames'])
            * m['repeat_count'] / m['playback_speed'] / 1000
            for name, m in variants.items()},
    }
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'verified': len(variants), 'aliases': len(manifest['aliases']),
                      'changed': changed}, ensure_ascii=False))


if __name__ == '__main__':
    main()
