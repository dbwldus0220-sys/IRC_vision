#!/usr/bin/env python3
"""Compare archived motion requests/completions; never connects to the robot."""
import json
import re
from decimal import Decimal
from pathlib import Path
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = Path('/home/jet/.ros/log')
current = (LOG/'python3_10206_1794044171300.log').read_text().splitlines()
previous = (LOG/'python3_10176_1793946701820.log').read_text().splitlines()
def ns(line):
    return int(Decimal(re.search(r'\[(\d+\.\d+)\]',line)[1])*10**9)
def event(lines, text):
    return next(ns(line) for line in lines if text in line)
def diff(end, begin):
    return (end-begin)/1e6
request = event(current,'BALL entry Line alignment requested: command_id=15,')
finish = event(current,'BALL entry Line alignment ended: command_id=15, status=SUCCEEDED')
motion = next(m for m in json.loads((ROOT/'artifacts/robot_motions_runtime.json').read_text())['motions'] if m['name']=='t라인복귀좌공용')
archived_motion = next(m for m in json.loads((ROOT/'artifacts/20261107_left_crab_source_speed/before/artifacts/robot_motions_runtime.json').read_text())['motions'] if m['name']==motion['name'])
assert motion == archived_motion
cycle = max(f['start_ms']+f['time_ms'] for f in motion['frames'])
nominal = cycle*motion['repeat_count']/motion['playback_speed']
result = {
 'ball_first_approach': {
  'request_epoch_ns': request, 'completion_epoch_ns': finish,
  'observed_request_to_completion_ms': diff(finish,request),
  'catalog_cycle_ms': cycle, 'repeat_count': motion['repeat_count'], 'speed': motion['playback_speed'],
  'nominal_catalog_duration_ms': nominal,
  'observed_minus_nominal_ms': diff(finish,request)-nominal,
  'motion_unchanged_from_archived_snapshot': True,
  'limitation': 'Excess includes startup, bus/tick overhead and status transport; not an isolated measurement of startup or torque reads.'
 },
 'same_run_known_post_pickup_boundaries': [
  {'transition':'pickup complete -> first line RUNNING', 'gap_ms':diff(event(current,'Post-pickup Line motion started: 1/3,'),event(current,'Special motion lock released: status=SUCCEEDED, action=PICKUP_NOW,'))},
  {'transition':'first line complete -> second line RUNNING', 'gap_ms':diff(event(current,'Post-pickup Line motion started: 2/3,'),event(current,'General motion lock released: status=SUCCEEDED, action=LINE_HEADING_TURN_RIGHT_5'))},
  {'transition':'second line complete -> third line RUNNING', 'gap_ms':diff(event(current,'Post-pickup Line motion started: 3/3,'),next(ns(line) for line in current if 'General motion lock released: status=SUCCEEDED, action=STRAIGHT' in line and 1794044292*10**9 < ns(line) <1794044293*10**9))},
 ],
 'nov6_first_ball': {
  'confirmed_epoch_ns':event(previous,'BALL mode consumed at confirmation: index=1,'),
  'next_ball_walk_request_epoch_ns':event(previous,'BALL entry Line alignment requested: command_id=7,'),
  'line_complete_epoch_ns':next(ns(line) for line in previous if 'General motion lock released: status=SUCCEEDED, action=STRAIGHT' in line and 1793946749*10**9<ns(line)<1793946750*10**9),
  'confirmed_during_motion':True,'distance_at_entry_m':1.045,
 },
 'nov7_first_ball':{'confirmed_during_motion':False,'distance_at_entry_m':.860},
}
o=result['nov6_first_ball']
o['confirmation_before_line_end_ms']=diff(o['line_complete_epoch_ns'],o['confirmed_epoch_ns'])
o['ball_walk_reserved_before_line_end_ms']=diff(o['line_complete_epoch_ns'],o['next_ball_walk_request_epoch_ns'])
(OUT/'execution_gap_metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
