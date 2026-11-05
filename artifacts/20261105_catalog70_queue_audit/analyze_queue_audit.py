"""Compare actual SDK results and simulate the existing late-frame reservation gate."""
import gzip
import json
from pathlib import Path

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS, PickupWalkReservation
from test_pickup_continuous_walk import ball

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
MANIFEST = json.loads((ROOT / 'artifacts/20261105_catalog70_requested/manifest.json').read_text())


def matrix(stage):
    path = OUT / f'{stage}_queue.tsv'
    text = path.read_text() if path.exists() else gzip.open(str(path) + '.gz', 'rt').read()
    return {tuple(parts[:3]): int(parts[3]) for line in text.splitlines()
            if (parts := line.split('\t'))}


before, after = matrix('before'), matrix('after')
# These are the walking groups submitted to the SDK by the bridge. Turns and
# posture changes are deferred; pickup reservations require the identical ID.
line = ({'forward', 'sdk_forward_4', 'line_recovery_left_4',
         'line_recovery_left_6', 'line_recovery_right_4'} |
        {f'{prefix}_{n}' for prefix in ('line_forward', 'ball_camera_down_forward',
                                      'post_ball_forward') for n in (2, 4, 6, 8)})
goal = {f'goal_camera_90_forward_{n}' for n in (2, 4, 6, 8)} | {
    'line_recovery_left_4', 'line_recovery_right_4'}
fine = {f'goal_camera_90_fine_forward_{n}' for n in (1, 2, 3, 4)}
existing = set(MANIFEST['aliases_before'])
active = {(a, b) for group in (line, goal, fine)
          for a in group & existing for b in group & existing}
active |= {(a, b) for a in line & existing for b in goal & existing}
active |= {(name, name) for name in PICKUP_WALK_DURATIONS}
active_keys = {(str(head), a, b) for head in (0, 1) for a, b in active}
assert active_keys <= before.keys() & after.keys()
new_blocks = [k for k in active_keys if before[k] == 0 and after[k] != 0]
assert not new_blocks

runtime = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
old_runtime = {m['name']: m for m in json.loads((Path(MANIFEST['backup']) / 'artifacts/robot_motions_runtime.json').read_text())['motions']}

def duration(motion):
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']


def rate_sweep():
    key = 'pickup_crab_right_0'
    original = PICKUP_WALK_DURATIONS[key]
    rows = []
    try:
        for stage, catalog in [('before', old_runtime), ('after', runtime)]:
            motion = catalog[MANIFEST[f'aliases_{stage}'][key]]
            seconds = duration(motion)
            PICKUP_WALK_DURATIONS[key] = seconds
            for fps in (10, 15, 20, 30):
                for delay in (0, .030, .050, .100):
                    successful = 0
                    for phase_index in range(100):
                        phase = phase_index / 100 / fps
                        planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
                        reservation.on_status(dict(status='RUNNING', command_id=1, request_id=1,
                                                   motion_id=key, pickup_alignment_stage='initial'),
                                              10., 10_000_000_000)
                        captured = 10. + phase
                        while captured + delay < 10. + seconds:
                            now = captured + delay
                            reservation.observe(planner, ball(offset_x_px=100,
                                rgb_stamp_ns=round(captured * 1e9)), now, round(now * 1e9), .5)
                            if reservation.decision(now, .5) is not None:
                                successful += 1
                                break
                            captured += 1 / fps
                    rows.append(dict(stage=stage, fps=fps, processing_delay_sec=delay,
                                     phases_tested=100, phases_with_reservation=successful,
                                     duration_sec=seconds, capture_window_sec=.3 * seconds))
    finally:
        PICKUP_WALK_DURATIONS[key] = original
    return rows

report = dict(
    total_sdk_pairs_before=len(before), total_sdk_pairs_after=len(after),
    active_automatic_queue_cases=len(active_keys),
    automatic_queue_cases_changed=[k for k in sorted(active_keys) if before[k] != after[k]],
    new_automatic_queue_blocks=new_blocks,
    existing_automatic_queue_rejections=sum(after[k] != 0 for k in active_keys),
    source_camera_variants={prefix: [m['name'] for m in json.loads(Path(MANIFEST['source_file']).read_text())['motions']
                                    if m['name'].startswith(prefix)]
                            for prefix in ('r오뒤에서 오옆', 'r미세오뒤에서 오옆', 't미세오옆꽃게')},
    timing_simulation=rate_sweep(),
    assumptions='Ideal periodic fresh confirmed Ball frames, stable right-crab decision, configurable fixed processing delay; no hardware or measured FPS.',
)
(OUT / 'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print('automatic queue cases:', len(active_keys), 'new blocks:', len(new_blocks))
for row in report['timing_simulation']:
    if row['processing_delay_sec'] in (0, .05):
        print(row['stage'], row['fps'], row['processing_delay_sec'], row['phases_with_reservation'])
