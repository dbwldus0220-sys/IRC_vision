"""Import source100 gait data under stable catalog names; preserve command counts."""

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


def replacements():
    source = {m['name']: m for m in read(UPDATE / 'source_robot_motions.json')['motions']}
    mapping = {
        'n라인복귀좌회전45도': 'p라인복귀좌회전45도',
        'n라인복귀좌회전90도': 'p라인복귀좌회전90도',
        'n라인복귀좌회전공용': 'p라인복귀좌회전공용',
        'n라인복귀우회전45도': 't라인복귀우실전',
        'n라인복귀우회전90도': 't라인복귀우실전90도',
        'n라인복귀우회전공용': 't라인복귀우공용',
    }
    for camera, counts in [(45, (2, 4, 6, 8)), (90, (1, 2, 3, 4, 6)), (0, (2, 4, 6))]:
        for count in counts:
            source_name = (f'p전진45({min(count, 6)}회)' if camera == 45 and count != 2
                           else f'p전진실전0도({count}회)' if camera == 0 and count != 2
                           else f'p전진{camera}도({count}회)')
            mapping[f'n전진{camera}도({count}회)'] = source_name
    new_motion = 'n공에서 전진하고 후진준비자세'
    mapping[new_motion] = new_motion
    variants = {}
    count_overrides = {}
    for target, original in mapping.items():
        motion = copy.deepcopy(source[original])
        motion['name'] = target
        if target.startswith('n전진'):
            count = int(target.split('(')[1].split('회')[0])
            if motion['repeat_count'] != count:
                count_overrides[target] = {'source': motion['repeat_count'], 'command': count}
            motion['repeat_count'] = count
        # Preserve the existing catalog's shared-pose identity, not new joint values.
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog80)'
        variants[target] = motion
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    for turn, recovery in EXIT_RECOVERIES:
        composite = build_exit(pc[turn], variants[recovery])
        variants[composite['name']] = composite
    return variants, mapping, count_overrides


def main():
    variants, mapping, count_overrides = replacements()
    manifest = {'source_sha256': hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest(),
                'source_mapping': mapping, 'count_overrides': count_overrides,
                'replaced': list(variants), 'duration_seconds': {}}
    for kind in ('pc', 'runtime'):
        path = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / path)
        merged = {m['name']: m for m in payload['motions']}
        for name, source in variants.items():
            motion = runtime_motion_from_source(source) if kind == 'runtime' else source
            if kind == 'runtime':
                manifest['duration_seconds'][name] = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
            merged[name] = motion
        payload['motions'] = list(merged.values())
        write(ROOT / path, payload)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
