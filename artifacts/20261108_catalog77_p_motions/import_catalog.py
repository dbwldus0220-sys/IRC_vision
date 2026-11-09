"""Reproduce catalog77 migration from the preserved pre-update snapshot."""

import copy
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import yaml

UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
ALIAS = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
SOURCE_PATH = UPDATE / 'source_robot_motions.json'
SOURCE = {m['name']: m for m in json.loads(SOURCE_PATH.read_text())['motions']}


def main():
    renames = {'t전진실전': 'p전진45(6회)',
               't라인복귀좌실전': 'p라인복귀좌회전45도',
               'R라인복귀좌90도': 'p라인복귀좌회전90도',
               't라인복귀좌공용': 'p라인복귀좌회전공용',
               'r미세 0도 허들용': 'p미세0도(5회)',
               'r미세 0도 허들용(10회)': 'p미세0도(10회)'}
    for camera, counts in [(45, (2, 4, 8)), (90, (2, 4, 6)), (0, (4, 6))]:
        prefix = {45: 'p전진45', 90: 'p전진90도', 0: 'p전진실전0도'}[camera]
        for count in counts:
            renames[f'R전진{camera}도({count}회)'] = f'{prefix}({count}회)'
    for camera in (0, 45, 90):
        for count in range(1, 7):
            suffix = f'{camera}' + ('' if camera == 45 else '도')
            renames[f't제자리좌회전{camera}도({count}회)'] = f'p제자리좌회전{suffix}({count}회)'
        for count in range(1, 6):
            renames[f't미세{camera}도({count}회)'] = f'p미세{camera}도({count}회)'
    derived = {
        'p전진45(2회)': ('p전진45(4회)', 2),
        'p전진45(8회)': ('p전진45(4회)', 8),
        'p전진90도(2회)': ('p전진90도(4회)', 2),
        'p미세0도(10회)': ('p미세0도(5회)', 10),
    }
    imports = {n: copy.deepcopy(m) for n, m in SOURCE.items() if n.startswith('p')}
    for name, (source, count) in derived.items():
        imports[name] = {**copy.deepcopy(SOURCE[source]), 'name': name, 'repeat_count': count}
    endpoints = {
        't오뒤에서 미세오뒤(45도)': 'p미세45도(5회)',
        't오뒤에서 미세오뒤(0도)': 'p미세0도(5회)',
        't오뒤에서 미세오뒤90도': 'p미세90도(5회)',
        'r오옆에서 미세오뒤(0도)': 'p미세0도(5회)',
        'r왼옆에서 미세오뒤(0도)': 'p미세0도(5회)',
        'r오옆에서 미세오뒤(90도)': 'p미세90도(5회)',
        'r왼옆에서 미세오뒤(90도)': 'p미세90도(5회)',
    }
    # Locally derived left-crab exits share the source right-crab speed policy.
    speed_sources = {
        'r왼옆에서 미세오뒤(0도)': 'r오옆에서 미세오뒤(0도)',
        'r왼옆에서 미세오뒤(90도)': 'r오옆에서 미세오뒤(90도)',
        'r왼옆에서 오뒤0도': 'r오옆에서 오뒤0도',
        'r왼옆에서 오뒤90도': 'r오옆에서 오뒤90도',
        'tt미세왼옆0도(2회)': 'tt미세왼옆0도',
    }
    pose_renames = {}
    before_runtime = json.loads((UPDATE / 'before/artifacts/robot_motions_runtime.json').read_text())
    retained = [m for m in before_runtime['motions'] if m['name'] not in renames]
    for motion in imports.values():
        for frame in motion['frames']:
            if any(f['name'] == frame['name'] and f['angles'] != frame['angles']
                   for m in retained for f in m['frames']):
                pose_renames[frame['name']] = frame['name'] + '(catalog77)'
    speed_changes = {}
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        catalog = json.loads((UPDATE / 'before' / relative).read_text())
        result = []
        for original in catalog['motions']:
            name = original['name']
            if name in renames:
                continue
            motion = copy.deepcopy(original)
            source_name = speed_sources.get(name, name)
            if source_name in SOURCE:
                speed = SOURCE[source_name]['playback_speed']
                if kind == 'runtime' and motion['playback_speed'] != speed:
                    speed_changes[name] = {'before': motion['playback_speed'], 'after': speed,
                                           'source': source_name}
                motion['playback_speed'] = speed
            if name in endpoints:
                frame = motion['frames'][-1]
                target = imports[endpoints[name]]['frames'][-1]
                frame['angles'] = copy.deepcopy(target['angles'])
                frame['name'] = target['name']
                motion['end_pose'] = target['name']
            result.append(motion)
        for original in imports.values():
            motion = copy.deepcopy(original)
            for frame in motion['frames']:
                frame['name'] = pose_renames.get(frame['name'], frame['name'])
            for key in ('start_pose', 'end_pose'):
                motion[key] = pose_renames.get(motion[key], motion[key])
            if kind == 'runtime':
                if motion['name'].startswith(('p제자리좌회전', 'p미세')):
                    motion['playback_speed'] = float(Decimal(str(motion['playback_speed'])) * Decimal('.95'))
                motion['completion']['position_tolerance_deg'] = 5.0
            result.append(motion)
        catalog['motions'] = result
        assert len({m['name'] for m in result}) == len(result)
        (ROOT / relative).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    alias_text = (UPDATE / 'before' / ALIAS).read_text()
    before_aliases = yaml.safe_load(alias_text)['motion_aliases']
    for old, new in renames.items():
        alias_text = alias_text.replace(f'"{old}"', f'"{new}"')
    alias_text = alias_text.replace('Catalog76 t left-turn gait', 'Catalog77 p left-turn gait')
    alias_text = alias_text.replace('Catalog52 R forward variants; 2/8-cycle commands retain their existing repeat counts.',
                                    'Catalog77 p forward variants; derived 2/8-cycle commands retain their counts.')
    alias_text = alias_text.replace('Source75 fine gaits retain the existing five / three repeat counts.',
                                    'Catalog77 p fine gaits retain the existing five / three repeat counts.')
    (ROOT / ALIAS).write_text(alias_text)
    aliases = yaml.safe_load(alias_text)['motion_aliases']
    assert set(aliases.values()) <= {m['name'] for m in result}
    manifest = {
        'source_sha256': hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest(),
        'renames': renames, 'imports': list(imports), 'derived': derived,
        'runtime_speed_factor': {'p제자리좌회전': .95, 'p미세': .95, 'other_source_motions': 1.0},
        'speed_changes': speed_changes, 'speed_sources': speed_sources,
        'endpoint_updates': endpoints, 'pose_renames': pose_renames,
        'changed_aliases': {a: {'before': before_aliases[a], 'after': n}
                            for a, n in aliases.items() if n != before_aliases[a]},
    }
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f'Imported {len(imports)} motions; remapped {len(manifest["changed_aliases"])} aliases; '
          f'updated {len(speed_changes)} other speeds and {len(endpoints)} endpoints.')


if __name__ == '__main__':
    main()
