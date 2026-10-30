"""Compare C++ fake-hardware playback with independently scaled source60."""
from decimal import Decimal
import copy
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
source = json.loads((OUT / 'source60.json').read_text())
reference = copy.deepcopy(source)
for motion in reference['motions']:
    motion['playback_speed'] = float(Decimal(str(motion['playback_speed'])) * Decimal('.95'))
reference_path = OUT / 'playback_reference.json'
reference_path.write_text(json.dumps(reference, ensure_ascii=False, indent=2) + '\n')
probe = ROOT / 'build/irc_step_motion_executor/robot_motion_sdk/gui_execution_probe'
runtime = ROOT / 'artifacts/robot_motions_runtime.json'
cases = []
for motion in reference['motions']:
    for tick, read_delay, write_delay in [('5', '0', '0'), ('5,17,83,6', '16', '1')]:
        def run(catalog):
            result = subprocess.run(
                [str(probe), str(catalog), motion['name'], tick, read_delay, write_delay,
                 '--continuous', '--account-forward-feedback'],
                capture_output=True, check=True, timeout=30)
            assert b'\nC\t' in result.stdout, motion['name']
            return result.stdout
        actual, expected = run(runtime), run(reference_path)
        assert actual == expected, (motion['name'], tick)
        cases.append({'motion': motion['name'], 'tick_ms': tick,
                      'read_delay_ms': read_delay, 'write_delay_ms': write_delay,
                      'source60_scaled_trace_equal': True,
                      'trace_sha256': hashlib.sha256(actual).hexdigest()})
report = {'hardware_used': False, 'continuous_interpolation': True,
          'passed': len(cases), 'cases': cases}
(OUT / 'playback_verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(f"{len(cases)} fake-hardware playback comparisons passed")
