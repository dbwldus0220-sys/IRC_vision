#!/usr/bin/env python3
"""Run the unmodified GUI with bounded, asynchronous playback trace recording."""
import argparse
import copy
import functools
import hashlib
import importlib.util
import json
from pathlib import Path
import queue
import sys
import threading
import time

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gui',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    spec=importlib.util.spec_from_file_location('instrumented_sdk_gui',args.gui.resolve())
    gui=importlib.util.module_from_spec(spec);sys.path.insert(0,str(args.gui.resolve().parent))
    spec.loader.exec_module(gui)
    pending=queue.Queue(maxsize=8192); dropped=[0];run=[0]
    def writer():
        with args.output.open('w') as stream:
            stream.write(json.dumps(dict(event='source',sha256=hashlib.sha256(args.gui.read_bytes()).hexdigest()))+'\n')
            while True:
                item=pending.get()
                if item is None: break
                stream.write(json.dumps(item,ensure_ascii=False)+'\n')
            stream.write(json.dumps(dict(event='dropped',count=dropped[0]))+'\n')
    thread=threading.Thread(target=writer);thread.start()
    def emit(row):
        try:pending.put_nowait(row)
        except queue.Full:dropped[0]+=1
    def wrap(name):
        original=getattr(gui.SDKMotionEditor,name)
        @functools.wraps(original)
        def call(self,*a,**kw):
            if name=='start_motion_playback':run[0]+=1
            begin=time.perf_counter(); result=None
            try:
                result=original(self,*a,**kw)
                return result
            finally:
                row=dict(event=name,run=run[0],begin_sec=begin,end_sec=time.perf_counter(),
                         timeline_ms=getattr(self,'current_timeline_ms',None),
                         repeat=getattr(self,'playback_repeat_current',None),
                         speed=getattr(self,'playback_speed',None),result=result)
                if name=='write_goal_positions' and a:
                    angles=self.normalize_angles(a[0]); ids=kw.get('target_ids',sorted(self.online_joints))
                    row['goal_deg']={i:angles[i] for i in ids if i in angles}
                    row['raw']={i:self.angle_to_dxl_position(v) for i,v in row['goal_deg'].items()}
                if name in ('read_feedback_and_report_fk','sync_sequence_start_from_robot'):
                    row['present_deg']=dict(self.feedback_angles)
                if name=='start_motion_playback':
                    row['frames']=copy.deepcopy(self.motion_sequence)
                    row['initial_deg']=dict(self.start_angles)
                    row['repeat_target']=self.playback_repeat_target
                    row['context']=self.playback_context
                emit(row)
        setattr(gui.SDKMotionEditor,name,call)
    for name in ('start_motion_playback','write_goal_positions','read_feedback_and_report_fk',
                 'sync_sequence_start_from_robot','restart_playback_cycle','stop_motion_sequence','anim_step'):
        wrap(name)
    try:
        app=gui.QApplication([str(args.gui)]);app.setStyle('Fusion')
        editor=gui.SDKMotionEditor();editor.show();app.exec_()
    finally:
        pending.put(None);thread.join()
if __name__=='__main__':main()
