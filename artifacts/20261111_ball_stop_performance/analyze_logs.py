"""Summarize the recorded run without starting ROS nodes or hardware."""
import collections
import json
from pathlib import Path
import re

LOG=Path('/home/jet/.ros/log')
FILES={
 'detector':'python3_5119_1794349742320.log',
 'analyzers':'python3_5121_1794349741893.log',
 'decision':'python3_5123_1794349741250.log',
 'bridge':'python3_5125_1794349741201.log',
 'camera':'realsense2_camera_node_5117_1794349740527.log',
 'executor':'sdk_motion_executor_5127_1794349740541.log',
}
records=[]
for node,name in FILES.items():
 for line in (LOG/name).read_text().splitlines():
  m=re.match(r'\[(\w+)\] \[([\d.]+)\] \[[^]]+\]: (.*)',line)
  if m:records.append((float(m[2]),node,m[1],m[3],len(line.encode())+1))
windows={'before_stop_10s':(1794349798,1794349808),
         'stop_context_4s':(1794349808,1794349812),
         'stop_wait_0758s':(1794349809.227850687,1794349809.985677785),
         'after_stop_10s':(1794349812,1794349822)}
report={'log_files':FILES,'windows':{}}
for label,(start,end) in windows.items():
 rows=[r for r in records if start<=r[0]<end]
 groups=collections.Counter()
 for _,node,_,message,_ in rows:
  prefix=message.split(':',1)[0].split(' {',1)[0]
  if message.startswith('[BALL_PIPELINE_TIMING]'):prefix='BALL_PIPELINE_TIMING'
  elif message.startswith('[DETECTOR_PIPELINE_TIMING]'):prefix='DETECTOR_PIPELINE_TIMING'
  elif message.startswith('[YOLO_INFERENCE_TIMING]'):prefix='YOLO_INFERENCE_TIMING'
  groups[node+': '+prefix]+=1
 report['windows'][label]={
  'duration_sec':end-start,'lines':len(rows),'bytes':sum(r[4] for r in rows),
  'lines_per_second':round(len(rows)/(end-start),2),
  'kib_per_second':round(sum(r[4] for r in rows)/(end-start)/1024,2),
  'per_node':dict(collections.Counter(r[1] for r in rows)),
  'top_messages':groups.most_common(6),
 }
report['nearby_timing']=[]
for t,node,level,message,_ in records:
 if not 1794349806<=t<=1794349814:continue
 for tag in ['BALL_PIPELINE_TIMING','DETECTOR_PIPELINE_TIMING']:
  prefix='['+tag+'] '
  if message.startswith(prefix):
   data=json.loads(message[len(prefix):])
   report['nearby_timing'].append({'wall_time':t,'tag':tag,**data})
 if message.startswith('[YOLO_INFERENCE_TIMING]'):
  report['nearby_timing'].append({'wall_time':t,'tag':'YOLO_INFERENCE_TIMING','message':message})
report['warnings_errors_during_stop']=[r[:4] for r in records if 1794349805<=r[0]<=1794349815 and r[2] in ('WARN','ERROR','FATAL')]
path=Path(__file__).with_name('timing_summary.json')
path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report['windows'],ensure_ascii=False,indent=2))
print('warnings/errors:',report['warnings_errors_during_stop'])
for row in report['nearby_timing']:
 if 'summary_ms' in row:
  print(round(row['wall_time']-1794349800,3),row['tag'], {k:(v['avg'],v['max']) for k,v in row['summary_ms'].items()})
 else:print(row['message'])

# Windows are selected by summary emission time; these are windowed estimates.
detector_rows=[]
for t,node,_,message,_ in records:
 prefix='[DETECTOR_PIPELINE_TIMING] '
 if message.startswith(prefix):
  detector_rows.append((t,json.loads(message[len(prefix):])))
comparison={}
for label,a,b in [
 ('before_recording_filename_time',1794349747,1794349754),
 ('after_recording_filename_time',1794349755,1794349765),
 ('around_ball_stop',1794349807,1794349811),
]:
 selected=[row for t,row in detector_rows if a<=t<b]
 def weighted_average(key):
  samples=[row['summary_ms'][key] for row in selected]
  return round(sum(v['avg']*v['n'] for v in samples)/sum(v['n'] for v in samples),3)
 comparison[label]={
  'summary_emission_epoch_range':[a,b],
  'effective_fps':round(sum(r['samples'] for r in selected)/sum(r['window_ms']/1000 for r in selected),3),
  'mean_ms':{key:weighted_average(key) for key in (
   'capture_to_detector','convert_preprocess','inference','overlay_display','callback')},
 }
report['recording_time_comparison']=comparison
report['limitations']=[
 'Recording start inferred from filename 07:29:14; temporal association is not an on/off experiment.',
 'No historical CPU/GPU load, thermal, disk IO or per-log-call durations were recorded.',
 'Some timing summaries exclude their own final logging cost.',
 'Image age includes camera/driver/transport/scheduling; these components cannot be separated from these logs.',
 'The 66.7 ms RGB-depth mismatch is visible in a nearby video frame, not a raw packet for every STOP command.',
]
path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
