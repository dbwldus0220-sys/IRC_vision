"""Apply only source78 support transitions and stationary-left speeds."""

import copy
import hashlib
import json
from pathlib import Path

import yaml

UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
SOURCE_PATH = UPDATE / 'source_robot_motions.json'
SOURCE = {m['name']: m for m in json.loads(SOURCE_PATH.read_text())['motions']}
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def main():
    renames = {}
    for camera in (0, 45, 90):
        renames[f'rr미세오뒤에서 오뒤{camera}도'] = f'p미세오뒤에서 오뒤{camera}도'
        suffix = f'{camera}도' if camera == 90 else f'({camera}도)'
        renames[f't오뒤에서 미세오뒤{suffix}'] = f'p오뒤에서 미세오뒤{suffix}'
    turns = [n for n in SOURCE if n.startswith('p제자리좌회전')]
    assert len(turns) == 18
    pose_renames = {}
    before = json.loads((UPDATE / 'before/artifacts/robot_motions_runtime.json').read_text())
    retained = [m for m in before['motions'] if m['name'] not in renames]
    for name in renames.values():
        for frame in SOURCE[name]['frames']:
            if any(f['name'] == frame['name'] and f['angles'] != frame['angles']
                   for m in retained for f in m['frames']):
                pose_renames[frame['name']] = frame['name'] + '(catalog78)'
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        catalog = json.loads((UPDATE / 'before' / relative).read_text())
        for index, old in enumerate(catalog['motions']):
            name = old['name']
            if name in renames:
                motion = copy.deepcopy(SOURCE[renames[name]])
                for frame in motion['frames']:
                    frame['name'] = pose_renames.get(frame['name'], frame['name'])
                for key in ('start_pose', 'end_pose'):
                    motion[key] = pose_renames.get(motion[key], motion[key])
                if kind == 'runtime':
                    motion['completion']['position_tolerance_deg'] = 5.0
                catalog['motions'][index] = motion
            elif name in turns:
                # Always use source speed directly; preserve every gait frame.
                old['playback_speed'] = SOURCE[name]['playback_speed']
        (ROOT / relative).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    alias_text = (UPDATE / 'before' / ALIAS_PATH).read_text()
    previous = yaml.safe_load(alias_text)['motion_aliases']
    for old, new in renames.items():
        alias_text = alias_text.replace(f'"{old}"', f'"{new}"')
    (ROOT / ALIAS_PATH).write_text(alias_text)
    aliases = yaml.safe_load(alias_text)['motion_aliases']
    manifest = {
        'source_sha256': hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest(),
        'renames': renames, 'speed_only_motions': turns, 'pose_renames': pose_renames,
        'changed_aliases': {a: {'before': previous[a], 'after': n}
                            for a, n in aliases.items() if n != previous[a]},
        'speed_policy': 'Source78 playback_speed verbatim for all 24 requested entries.',
    }
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f'Updated {len(renames)} transitions, {len(turns)} turn speeds and '
          f'{len(manifest["changed_aliases"])} aliases.')


if __name__ == '__main__':
    main()
