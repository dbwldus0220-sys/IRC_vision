"""Reproduce the selective source75 import from the saved pre-update catalogs."""

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


source = read(HERE / 'source_robot_motions.json')
by_source = {m['name']: m for m in source['motions']}
names = [f't미세{camera}도({count}회)' for camera in (0, 45, 90)
         for count in range(1, 6)] + ['t허들찐']
prepared_name = 't오뒤에서 미세오뒤(45도)'
imported = {name: copy.deepcopy(by_source[name]) for name in names}

# The existing queued entry must end at the updated fine gait's actual target.
# Preserve every other preparation field, including its existing speed.
speed_sources = {}
for camera in (0, 45, 90):
    for count in range(1, 7):
        old_camera = '' if camera == 45 else f'{camera}도'
        speed_sources[f'R제자리좌회전{old_camera}({count}회)'] = (
            f't제자리좌회전{camera}도({count}회)')
for name in ('t공후 제자리좌회전', 't골대후 제자리좌회전'):
    speed_sources[name] = name

before_pc = read(HERE / 'before/artifacts/robot_motions_pc.json')
retained = [m for m in before_pc['motions']
            if m['name'] not in imported and m['name'] not in ('찐허들', prepared_name)]
# A globally reused pose name must not resolve to different joint targets.
pose_renames = {}
for motion in imported.values():
    for frame in motion['frames']:
        if any(old['name'] == frame['name'] and old['angles'] != frame['angles']
               for m in retained for old in m['frames']):
            pose_renames[frame['name']] = frame['name'] + '(catalog75)'


def rename_poses(motion):
    for frame in motion['frames']:
        frame['name'] = pose_renames.get(frame['name'], frame['name'])
    for key in ('start_pose', 'end_pose'):
        motion[key] = pose_renames.get(motion[key], motion[key])
    return motion


for kind in ('pc', 'runtime'):
    catalog = read(HERE / f'before/artifacts/robot_motions_{kind}.json')
    result = []
    for old in catalog['motions']:
        name = old['name']
        if name == '찐허들':
            name = 't허들찐'
        if name in imported:
            motion = rename_poses(copy.deepcopy(imported[name]))
            if kind == 'runtime':
                motion = runtime_motion_from_source(motion)
        else:
            motion = copy.deepcopy(old)
            if name == prepared_name:
                motion['frames'][-1]['angles'] = copy.deepcopy(
                    by_source[prepared_name]['frames'][-1]['angles'])
                rename_poses(motion)
            if name in speed_sources:
                original = by_source[speed_sources[name]]
                motion['playback_speed'] = (original['playback_speed'] if kind == 'pc'
                                            else runtime_motion_from_source(original)['playback_speed'])
        result.append(motion)
    existing = {m['name'] for m in result}
    for name, original in imported.items():
        if name not in existing:
            motion = rename_poses(copy.deepcopy(original))
            result.append(runtime_motion_from_source(motion) if kind == 'runtime' else motion)
    catalog['motions'] = result
    (ROOT / f'artifacts/robot_motions_{kind}.json').write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')

alias_path = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
alias_text = (HERE / 'before' / alias_path).read_text().replace('hurdle: "찐허들"', 'hurdle: "t허들찐"')
for count in range(1, 5):
    alias_text = alias_text.replace(f'"t미세0도({count}회)(골대74)"', f'"t미세90도({count}회)"')
alias_text = alias_text.replace('# Source62 fine gaits retain', '# Source75 fine gaits retain')
(ROOT / alias_path).write_text(alias_text)
manifest = {
    'source_sha256': hashlib.sha256((HERE / 'source_robot_motions.json').read_bytes()).hexdigest(),
    'imported_motions': names,
    'removed_motions': ['찐허들'],
    'speed_only_sources': speed_sources,
    'pose_renames': pose_renames,
    'preparation_endpoint_update': prepared_name,
    'runtime_speed_factor': .95,
    'goal_fine_camera': 90,
    'aliases_after': yaml.safe_load(alias_text)['motion_aliases'],
}
(HERE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
