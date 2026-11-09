"""Extract the recorded mission timeline without contacting robot hardware."""

import json
import re
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


HERE = Path(__file__).resolve().parent
EVENTS = (
    'ball_lost_inside_image_backward_search',
    'Post-pickup Line motion started:',
    'Post-pickup Goal camera ready;',
    'Post-pickup Line driving completed:',
    'Goal camera starts with third Line motion:',
    'HURDLE positioning entry latched:',
    'HURDLE final sequence latched:',
    'Mission phase advanced after special motion:',
    '[GRASP_VERIFY_START]',
    'Execution failure:',
)


def main():
    events = []
    waits = []
    for path in sorted(HERE.glob('*.log')):
        if not path.name.startswith(('python3_', 'sdk_motion_executor_')):
            continue
        for line_number, line in enumerate(path.read_text().splitlines(), 1):
            match = re.search(r'\[(\d+\.\d+)\]', line)
            if match is None:
                continue
            stamp = float(match[1])
            if any(marker in line for marker in EVENTS):
                events.append({
                    'stamp': stamp,
                    'kst': datetime.fromtimestamp(stamp, ZoneInfo('Asia/Seoul')).isoformat(),
                    'log': path.name, 'line': line_number, 'message': line,
                })
            if 'reason=goal_not_detected,' in line:
                if not waits or stamp - waits[-1]['last_stamp'] > 1.0:
                    waits.append({'first_stamp': stamp, 'last_stamp': stamp, 'messages': 1})
                else:
                    waits[-1]['last_stamp'] = stamp
                    waits[-1]['messages'] += 1
    for wait in waits:
        wait['duration_sec'] = round(wait['last_stamp'] - wait['first_stamp'], 3)
        for edge in ('first', 'last'):
            wait[edge + '_kst'] = datetime.fromtimestamp(
                wait[edge + '_stamp'], ZoneInfo('Asia/Seoul')).isoformat()
    result = {
        'events': sorted(events, key=lambda event: event['stamp']),
        'goal_not_detected_waits': waits,
        'limitations': [
            'Screen recording contains RGB overlays, not raw aligned depth or goal_info messages.',
            'RAW backboards and goal_not_detected establish rejection before confirmed Goal input; '
            'they do not distinguish invalid depth from depth above 2m.',
            'SDK memory-hardware checks do not validate physical balance or motor communication.',
        ],
    }
    (HERE / 'timeline.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
