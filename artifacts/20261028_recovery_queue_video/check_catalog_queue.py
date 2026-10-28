"""Replay deployed line gaits on a virtual bus; never connects to hardware."""
import csv
import itertools
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
PROBE = ROOT / 'build/irc_step_motion_executor/robot_motion_sdk/gui_execution_probe'
CATALOG = ROOT / 'artifacts/robot_motions_runtime.json'
NAMES = ['R전진45도(6회)', 'R전진라인복귀좌', 'R라인복귀우회전']
catalog = {m['name']: m for m in json.loads(CATALOG.read_text())['motions']}
assert all(catalog[n]['frames'][0] == catalog[NAMES[0]]['frames'][0] for n in NAMES)
results = []
with tempfile.TemporaryDirectory(prefix='step_recovery_queue_') as directory:
    for continuous, delay, first, second in itertools.product(
        [False, True], [0, 16], NAMES, NAMES,
    ):
        trace = Path(directory) / 'trace.csv'
        command = [str(PROBE), str(CATALOG), first, '5', str(delay), '0',
                   '--account-forward-feedback', '--queue', second, '--trace', str(trace)]
        if continuous:
            command.append('--continuous')
        subprocess.run(command, stdout=subprocess.DEVNULL, check=True)
        text = trace.read_text()
        assert text.rstrip().endswith('# dropped_rows=0')
        rows = list(csv.DictReader(line for line in text.splitlines() if not line.startswith('#')))
        assert sum(r['event'] == 'initial' for r in rows) == 1
        assert sum(r['event'] == 'queue_activate' for r in rows) == 1
        starts = [r for r in rows if r['event'] == 'run_start']
        assert [r['motion'] for r in starts] == [first, second]
        goals = [r for r in rows if r['event'] == 'goals_complete']
        assert len(goals) == 2
        assert int(goals[0]['repeat']) == catalog[first]['repeat_count']
        assert int(goals[1]['repeat']) == catalog[second]['repeat_count']
        gap_ms = float(starts[1]['begin_ms']) - float(goals[0]['begin_ms'])
        assert abs(gap_ms) < 1e-6
        # The next gait inherits every final commanded target, not a new measured pose.
        endpoint = {}
        for frame in catalog[first]['frames']:
            endpoint.update(frame['angles'])
        for motor, angle in endpoint.items():
            assert abs(float(starts[1][f'deg_{motor}']) - angle) < 1e-8
        results.append(dict(first=first, second=second, continuous=continuous,
                            read_delay_ms=delay, initial_reads=1, boundary_dispatch_gap_ms=gap_ms))
(OUT / 'catalog_queue_results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')
print(f'{len(results)} catalog queue cases passed')
