"""Keep each shot exit's preparation and only the requested complete turn cycles."""

import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def main():
    catalogs = {
        kind: json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())
        for kind in ('pc', 'runtime')
    }
    source = {motion['name']: motion for motion in catalogs['pc']['motions']}
    variants = []
    for name, count, cycle_pose in (
        ('t골대후제자리우회전', 5, '제우오들25'),
        ('p골대후제자리좌회전', 3, 'tt제좌왼들45'),
    ):
        original = source[name]
        frames = original['frames']
        assert original['repeat_count'] == 1
        assert len(frames) >= 2 + 2 * count
        assert frames[-1]['name'] == original['end_pose']
        for index in range(2, len(frames), 2):
            assert frames[index]['name'] == cycle_pose
            assert frames[index + 1]['name'] == original['end_pose']
        variant = copy.deepcopy(original)
        variant['name'] = f'{name}({count}회)'
        # The first two frames prepare support; each remaining pair is one turn.
        variant['frames'] = variant['frames'][:2 + 2 * count]
        assert variant['frames'][-1]['angles'] == frames[-1]['angles']
        assert variant['frames'][-1]['torques'] == frames[-1]['torques']
        variant['max_seq_ms'] = max(f['start_ms'] + f['time_ms'] for f in variant['frames'])
        variants.append(variant)
    for kind, catalog in catalogs.items():
        replacements = {
            motion['name']: (motion if kind == 'pc' else runtime_motion_from_source(motion))
            for motion in variants
        }
        catalog['motions'] = [
            motion for motion in catalog['motions'] if motion['name'] not in replacements
        ] + list(replacements.values())
        (ROOT / f'artifacts/robot_motions_{kind}.json').write_text(
            json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    for motion in variants:
        print(motion['name'], len(motion['frames']), motion['max_seq_ms'], motion['playback_speed'])


if __name__ == '__main__':
    main()
