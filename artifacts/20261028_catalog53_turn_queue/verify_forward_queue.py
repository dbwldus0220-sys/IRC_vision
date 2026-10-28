"""Compare real-catalog queues with equivalent internal repeats on a virtual bus."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PROBE = ROOT / 'build/irc_step_motion_executor/robot_motion_sdk/gui_execution_probe'
CATALOG = ROOT / 'artifacts/robot_motions_runtime.json'
source = json.loads(CATALOG.read_text())
motions = {m['name']: m for m in source['motions']}
results = []
with tempfile.TemporaryDirectory(prefix='step53_queue_') as temp:
    for camera in (0, 45, 90):
        first, second = (f'R전진{camera}도({n}회)' for n in (4, 6))
        baseline = copy.deepcopy(motions[first])
        baseline.update(repeat_count=10, name='reference_10')
        reference = Path(temp) / 'reference.json'
        reference.write_text(json.dumps({'motions': [baseline]}, ensure_ascii=False))
        for continuous in (False, True):
            for periods, read_delay, write_delay in [('5', '0', '0'), ('5,7,9', '16', '1')]:
                options = ['--continuous'] if continuous else []

                def run(path, name, queue=None):
                    cmd = [str(PROBE), str(path), name, periods, read_delay, write_delay] + options
                    if queue:
                        cmd += ['--queue', queue]
                    p = subprocess.run(cmd, text=True, capture_output=True)
                    assert p.returncode == 0, (cmd, p.returncode, p.stderr)
                    rows = p.stdout.splitlines()
                    return ([r for r in rows if r.startswith(('W\t', 'R\t'))],
                            [r for r in rows if r.startswith('C\t')])

                expected, ce = run(reference, 'reference_10')
                actual, ca = run(CATALOG, first, second)
                assert actual == expected, (camera, continuous, periods, 'bus trace differs')
                assert ca[-1].split('\t')[1] == ce[-1].split('\t')[1]
                assert len(ca) == 2 and ca[-1].endswith('\t2')
                results.append(dict(camera=camera, continuous=continuous, periods_ms=periods,
                                    read_delay_ms=read_delay, write_delay_ms=write_delay,
                                    bus_events=len(actual), exact_bus_trace_match=True,
                                    completion_events=ca))
Path(__file__).with_name('forward_queue_comparison.json').write_text(
    json.dumps(results, ensure_ascii=False, indent=2) + '\n')
print(f'PASS {len(results)} scenarios: queued 4+6 matches 10 internal repeats, including all holds.')
