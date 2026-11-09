"""Add only the four long-range scoring motions from the supplied catalog80."""

import copy
import json
from pathlib import Path
import sys

UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

NAMES = {5: 'r골넣기 5번구역', 6: 'r골넣기 6번구역',
         7: 'r골넣기7번구역', 8: 'r골넣기8번구역'}


def main():
    source = {m['name']: m for m in json.loads((UPDATE / 'source_robot_motions.json').read_text())['motions']}
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        catalog = json.loads((UPDATE / 'before' / relative).read_text())
        assert not set(NAMES.values()) & {m['name'] for m in catalog['motions']}
        for name in NAMES.values():
            motion = copy.deepcopy(source[name])
            if kind == 'runtime':
                motion = runtime_motion_from_source(motion)
            catalog['motions'].append(motion)
        (ROOT / relative).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    marker = '  goal_shot_region_4: "R골넣기4번구역"\n'
    assert aliases.count(marker) == 1
    additions = ''.join(f'  goal_shot_region_{region}: "{name}"\n' for region, name in NAMES.items())
    (ROOT / relative).write_text(aliases.replace(marker, marker + additions))


if __name__ == '__main__':
    main()
