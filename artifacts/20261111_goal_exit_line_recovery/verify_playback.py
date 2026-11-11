"""Verify both goal exits with the SDK's simulated hardware (no robot connection)."""

import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / 'artifacts/robot_motions_runtime.json'
PROBE = ROOT / 'build/irc_step_motion_executor/robot_motion_sdk/gui_execution_probe'


def main():
    catalog = {m['name']: m for m in json.loads(CATALOG.read_text())['motions']}
    results = []
    for name in (
        't골대후제자리우회전(5회)+라인복귀좌회전45도(4회)',
        'p골대후제자리좌회전(3회)+라인복귀우회전45도(4회)',
    ):
        motion, turn = catalog[name], catalog[name.split('+')[0]]
        turn_end = max(f['start_ms'] + f['time_ms'] for f in turn['frames'])
        recovery_start = motion['frames'][len(turn['frames'])]['start_ms']
        assert (recovery_start - turn_end) / motion['playback_speed'] == 500
        pose = {int(j): a for j, a in turn['frames'][-1]['angles'].items()}
        for continuous in (False, True):
            output = subprocess.check_output([
                str(PROBE), str(CATALOG), name, '5', '0', '0',
                *(['--continuous'] if continuous else []),
            ], text=True)
            goals, holds, completions, first_change = {}, [], [], None
            for line in output.splitlines():
                fields = line.split('\t')
                if fields[0] == 'W':
                    goals.update({int(j): float(a) for j, a, _ in
                                  (value.split(':') for value in fields[2:])})
                elif fields[0] == 'C':
                    completions.append(float(fields[1]))
                elif fields[0] == 'T':
                    wall_ms, timeline_ms = map(float, fields[1:3])
                    same = all(abs(goals[j] - a) < 1e-9 for j, a in pose.items())
                    if turn_end <= timeline_ms <= recovery_start:
                        assert same, (name, continuous, timeline_ms)
                        holds.append(wall_ms)
                    if timeline_ms > recovery_start and first_change is None and not same:
                        first_change = wall_ms
            assert len(completions) == 1 and completions[0] > first_change
            # Allow one 5 ms scheduler tick when observing the 500 ms hold.
            assert 500 <= first_change - holds[0] <= 505
            results.append(dict(motion=name, interpolation='continuous' if continuous else 'cosine',
                                hold_samples=len(holds), observed_hold_ms=first_change - holds[0],
                                completion_events=len(completions)))
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
