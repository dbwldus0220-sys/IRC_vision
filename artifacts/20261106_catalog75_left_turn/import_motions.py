"""Replace all 18 stationary left turns from source75, preserving other motions."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def read(path):
    return json.loads(path.read_text())


source = {m['name']: m for m in read(HERE / 'source_robot_motions.json')['motions']}
renames = {
    f'R제자리좌회전{str(camera) + "도" if camera != 45 else ""}({count}회)':
    f't제자리좌회전{camera}도({count}회)'
    for camera in (0, 45, 90) for count in range(1, 7)
}
before_pc = read(HERE / 'before/artifacts/robot_motions_pc.json')
retained = [m for m in before_pc['motions'] if m['name'] not in renames]
pose_renames = {}
for name in renames.values():
    for frame in source[name]['frames']:
        if any(old['name'] == frame['name'] and old['angles'] != frame['angles']
               for m in retained for old in m['frames']):
            pose_renames[frame['name']] = frame['name'] + '(catalog75_left)'

for kind in ('pc', 'runtime'):
    catalog = read(HERE / f'before/artifacts/robot_motions_{kind}.json')
    for index, old in enumerate(catalog['motions']):
        if old['name'] not in renames:
            continue
        motion = copy.deepcopy(source[renames[old['name']]])
        for frame in motion['frames']:
            frame['name'] = pose_renames.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            motion[key] = pose_renames.get(motion[key], motion[key])
        if kind == 'runtime':
            # Explicit user override: camera45/count6 uses the other turns' speed.
            motion['playback_speed'] = .9
            motion = runtime_motion_from_source(motion)
        catalog['motions'][index] = motion
    (ROOT / f'artifacts/robot_motions_{kind}.json').write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')

alias_path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
alias_text = (HERE / 'before' / alias_path).read_text()
for old, new in renames.items():
    alias_text = alias_text.replace(f'"{old}"', f'"{new}"')
alias_text = alias_text.replace('# Catalog53 R left-turn gait:', '# Catalog75 t left-turn gait:')
(ROOT / alias_path).write_text(alias_text)
manifest = {
    'source_sha256': hashlib.sha256((HERE / 'source_robot_motions.json').read_bytes()).hexdigest(),
    'renames': renames,
    'pose_renames': pose_renames,
    'runtime_speed': .855,
    'source_speed_override': {'t제자리좌회전45도(6회)': .9},
    'aliases_after': yaml.safe_load(alias_text)['motion_aliases'],
}
(HERE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
