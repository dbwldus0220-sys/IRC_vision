"""Import only the requested source62 fine gaits and retain mission repeat counts."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import yaml

UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def main():
    source_bytes = (UPDATE / 'source62.json').read_bytes()
    source = {m['name']: m for m in json.loads(source_bytes)['motions']}
    originals = ('r미세45도', 'r미세0도 공잡기용', 'r미세 0도 허들용', 'r미세 골대전용')
    specs = {name: {'source': name, 'repeat_count': source[name]['repeat_count']}
             for name in originals}
    specs['r미세0도 공잡기용(상단탐색3회)'] = {'source': originals[1], 'repeat_count': 3}
    specs['r미세 0도 허들용(10회)'] = {'source': originals[2], 'repeat_count': 10}
    specs.update({f'r미세 골대전용({count}회)': {'source': originals[3], 'repeat_count': count}
                  for count in range(1, 5)})
    imported = {}
    for name, spec in specs.items():
        motion = copy.deepcopy(source[spec['source']])
        motion.update(name=name, repeat_count=spec['repeat_count'])
        imported[name] = motion

    for filename in ('robot_motions_pc.json', 'robot_motions_runtime.json'):
        path = ROOT / 'artifacts' / filename
        catalog = json.loads(path.read_text())
        positions = {m['name']: i for i, m in enumerate(catalog['motions'])}
        for name, original in imported.items():
            motion = runtime_motion_from_source(original) if 'runtime' in filename else original
            if name in positions:
                catalog['motions'][positions[name]] = motion
            else:
                catalog['motions'].append(motion)
        path.write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')

    aliases = {
        'ball_approach_fine_forward_45_5': originals[0],
        'ball_general_fine_forward_8': originals[1],
        'pickup_fine_forward_0': originals[1],
        'ball_top_loss_fine_forward_45_3': 'r미세0도 공잡기용(상단탐색3회)',
        'pickup_top_loss_fine_forward_0_3': 'r미세0도 공잡기용(상단탐색3회)',
        'hurdle_fine_forward_5': originals[2],
        'hurdle_fine_forward_10': 'r미세 0도 허들용(10회)',
        **{f'goal_camera_90_fine_forward_{n}': f'r미세 골대전용({n}회)' for n in range(1, 5)},
    }
    path = ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    lines = path.read_text().splitlines()
    remaining = dict(aliases)
    for index, line in enumerate(lines):
        key = line.strip().split(':', 1)[0]
        if key in remaining:
            lines[index] = f'  {key}: "{remaining.pop(key)}"'
    for key, name in remaining.items():
        index = next(i for i, line in enumerate(lines) if line.startswith('  hurdle_fine_forward_10:'))
        lines.insert(index, f'  {key}: "{name}"')
    text = '\n'.join(lines).replace(
        '# Stable IDs use source60 r미세0도 (five / three repeats).',
        '# Source62 fine gaits retain the existing five / three repeat counts.') + '\n'
    assert all(yaml.safe_load(text)['motion_aliases'][key] == value for key, value in aliases.items())
    path.write_text(text)
    manifest = {'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
                'runtime_speed_factor': 0.95, 'motions': specs, 'aliases': aliases}
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
