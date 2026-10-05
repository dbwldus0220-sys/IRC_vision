#!/usr/bin/env python3
"""Record GUI playback and actual paint work without changing playback policy."""
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


class TraceWriter:
    def __init__(self, path, source):
        self.stream = Path(path).open('x', encoding='utf-8')
        self.pending = queue.Queue(maxsize=8192)
        self.dropped = 0
        self.error = None
        self.emit(dict(event='session', schema_version=2, platform='gui',
                       clock='perf_counter_ns', source=source))
        self.thread = threading.Thread(target=self._write)
        self.thread.start()

    def emit(self, row):
        try:
            self.pending.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        while True:
            row = self.pending.get()
            if row is None:
                break
            if self.error is not None:
                continue  # Drain even after an I/O error so shutdown cannot deadlock.
            try:
                if 'snapshot' in row:
                    encoded = json.dumps(row['snapshot'], sort_keys=True, ensure_ascii=False, allow_nan=False)
                    row['snapshot_sha256'] = hashlib.sha256(encoded.encode()).hexdigest()
                self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            except Exception as error:
                self.error = error

    def close(self):
        self.pending.put(None)
        self.thread.join()
        try:
            if self.error is None:
                self.stream.write(json.dumps(dict(event='trace_end', dropped_rows=self.dropped)) + '\n')
                self.stream.flush()
        finally:
            self.stream.close()
        if self.error is not None:
            raise RuntimeError('playback trace writer failed') from self.error


class GuiTrace:
    def __init__(self, emit, motion_name=None, clock=time.perf_counter_ns):
        self.emit, self.motion_name, self.clock = emit, motion_name, clock
        self.run = self.tick = self.sequence = 0
        self.evaluate_ns = None
        self.context = self.tick_context = None
        self.last_write_packet = None

    def state(self, editor):
        timeline = getattr(editor, 'current_timeline_ms', 0)
        frames = getattr(editor, 'motion_sequence', [])
        frame = next((i for i, f in enumerate(frames)
                      if f['start_ms'] <= timeline < f['start_ms'] + f['time_ms']), -1)
        loaded = getattr(editor, 'loaded_sequence_id', None)
        saved = next((s for s in getattr(editor, 'saved_sequences', [])
                      if loaded is not None and s.get('sequence_id') == loaded), {})
        return dict(run=self.run, tick=self.tick, motion=self.motion_name or saved.get('name', ''),
                    timeline_ms=timeline, repeat=getattr(editor, 'playback_repeat_current', 1),
                    frame=frame, evaluate_ns=self.evaluate_ns,
                    gui_context=getattr(editor, 'playback_context', 'motion'))

    def record(self, event, begin, end, context, success=True, **extra):
        self.sequence += 1
        self.emit(dict(context, event=event, event_seq=self.sequence,
                       begin_ns=begin, end_ns=end, success=success, **extra))

    def install(self, gui):
        tracer = self

        def wrap(name, event):
            original = getattr(gui.SDKMotionEditor, name)

            @functools.wraps(original)
            def call(editor, *args, **kwargs):
                if name == 'start_motion_playback':
                    tracer.run += 1
                    tracer.tick = 0
                    tracer.evaluate_ns = None
                if name == 'anim_step':
                    tracer.tick += 1
                prior_context = tracer.context
                context = ({k: v for k, v in prior_context.items() if k != 'write_kind'}
                           if prior_context is not None else tracer.state(editor))
                extra = {}
                if name == 'gate_timeline_on_frame_arrival':
                    tracer.evaluate_ns = int(args[1] * 1e9)
                    context['evaluate_ns'] = tracer.evaluate_ns
                    extra['requested_timeline_ms'] = args[0]
                    if tracer.tick_context is not None:
                        tracer.tick_context['evaluate_ns'] = tracer.evaluate_ns
                    frame = getattr(editor, 'timed_gate_frame', None)
                    if frame is not None and args[0] >= frame['start_ms'] + frame['time_ms']:
                        context['timeline_ms'] = frame['start_ms'] + frame['time_ms']
                        context['frame'] = editor.motion_sequence.index(frame)
                    tracer.context = dict(context, write_kind='endpoint')
                elif name == 'scrub_timeline':
                    timeline = args[0] if args else kwargs['t_ms']
                    end_ms = max((f['start_ms'] + f['time_ms'] for f in editor.motion_sequence), default=0)
                    boundary = kwargs.get('cycle_end', False) or timeline == end_ms
                    context['timeline_ms'] = timeline
                    context['frame'] = next((i for i, f in enumerate(editor.motion_sequence)
                        if f['start_ms'] <= timeline < f['start_ms'] + f['time_ms']
                        or (boundary and timeline == f['start_ms'] + f['time_ms'])), -1)
                    extra['cycle_end'] = boundary
                    tracer.context = dict(context, write_kind='sample')
                elif prior_context is not None:
                    context = {k: v for k, v in prior_context.items() if k != 'write_kind'}
                event_name = event
                if name == 'write_goal_positions':
                    tracer.last_write_packet = None
                    event_name = (prior_context or {}).get('write_kind', 'write')
                    angles = editor.normalize_angles(args[0] if args else kwargs['angles_dict'])
                    ids = kwargs.get('target_ids', args[1] if len(args) > 1 else None)
                    ids = list(editor.online_joints) if ids is None else ids
                    extra['angles'] = {int(i): angles[int(i)] for i in ids if int(i) in angles}
                    extra['raw'] = {i: editor.angle_to_dxl_position(v) for i, v in extra['angles'].items()}
                if name == 'anim_step':
                    tracer.tick_context = context
                begin = tracer.clock()
                success = False
                try:
                    result = original(editor, *args, **kwargs)
                    success = result is not False
                    return result
                finally:
                    end = tracer.clock()
                    if name == 'write_goal_positions':
                        extra['goal_source'] = 'requested_goals'
                        if tracer.last_write_packet is not None:
                            extra['requested_raw'] = extra['raw']
                            extra['raw'] = tracer.last_write_packet if success else {}
                            extra['angles'] = {i: v for i, v in extra['angles'].items() if i in extra['raw']}
                            extra['goal_source'] = 'packet_params'
                    if name in ('read_feedback_and_report_fk', 'sync_sequence_start_from_robot'):
                        extra['angles'] = dict(getattr(editor, 'feedback_angles', {})) if success else {}
                    if name == 'start_motion_playback' and success and editor.is_playing:
                        extra['snapshot'] = dict(frames=copy.deepcopy(editor.motion_sequence),
                            max_seq_ms=editor.max_seq_ms, repeat_count=editor.playback_repeat_target,
                            playback_speed=editor.playback_speed,
                            composer_segments=copy.deepcopy(getattr(editor, 'composer_playback_segments', [])),
                            initial_deg=dict(editor.start_angles))
                        extra['playback_origin_ns'] = int(editor.anim_start_time * 1e9)
                    if name == 'restart_playback_cycle':
                        extra.update(next_repeat=editor.playback_repeat_current,
                                     next_timeline_ms=editor.current_timeline_ms,
                                     playback_origin_ns=int(editor.anim_start_time * 1e9))
                    if name == 'anim_step':
                        extra.update(next_timeline_ms=editor.current_timeline_ms,
                                     next_repeat=editor.playback_repeat_current, running=editor.is_playing)
                        tracer.tick_context = None
                    tracer.record(event_name, begin, end, context, success, **extra)
                    tracer.context = prior_context
            setattr(gui.SDKMotionEditor, name, call)

        for name, event in (
            ('start_motion_playback', 'run_start'), ('anim_step', 'tick'),
            ('gate_timeline_on_frame_arrival', 'gate'), ('scrub_timeline', 'evaluate'),
            ('write_goal_positions', 'sample'), ('read_feedback_and_report_fk', 'feedback'),
            ('sync_sequence_start_from_robot', 'initial'),
            ('restart_playback_cycle', 'restart'), ('stop_motion_sequence', 'goals_complete'),
            ('update_3d_robot', 'gui_3d'),
        ):
            wrap(name, event)

        def wrap_ui(cls, name, event):
            original = getattr(cls, name)

            @functools.wraps(original)
            def call(widget, *args, **kwargs):
                editor = getattr(widget, 'parent_gui', None)
                begin = tracer.clock()
                success = False
                try:
                    result = original(widget, *args, **kwargs)
                    success = True
                    return result
                finally:
                    end = tracer.clock()
                    if editor is not None and getattr(editor, 'is_playing', False):
                        context = ({k: v for k, v in tracer.context.items() if k != 'write_kind'}
                                   if tracer.context is not None else tracer.state(editor))
                        tracer.record(event, begin, end, context, success)
            setattr(cls, name, call)

        if hasattr(gui, 'JointTrajectoryWidget'):
            for method, event in (('paintEvent', 'gui_plot_paint'), ('trajectory_samples', 'gui_plot_compute')):
                wrap_ui(gui.JointTrajectoryWidget, method, event)
        if hasattr(gui, 'TimelineBlockWidget'):
            for method in ('set_playing_style', 'set_default_style'):
                wrap_ui(gui.TimelineBlockWidget, method, 'gui_style')

    def attach_bus(self, gui, editor):
        """Capture the actual SyncWrite parameters, including partial addParam failures."""
        tracer = self

        class WriteProxy:
            def __init__(self, bus):
                self.bus = bus
                self.raw = {}

            def __getattr__(self, name):
                return getattr(self.bus, name)

            def clearParam(self):
                result = self.bus.clearParam()
                self.raw.clear()
                return result

            def addParam(self, motor_id, data):
                result = self.bus.addParam(motor_id, data)
                if result:
                    self.raw[int(motor_id)] = int.from_bytes(bytes(data), 'little')
                return result

            def txPacket(self):
                begin = tracer.clock()
                result = self.bus.txPacket()
                end = tracer.clock()
                ok = result == gui.COMM_SUCCESS
                tracer.last_write_packet = dict(self.raw) if ok else {}
                context = ({k: v for k, v in tracer.context.items() if k != 'write_kind'}
                           if tracer.context is not None else tracer.state(editor))
                tracer.record('packet_write', begin, end, context, ok,
                              raw=dict(self.raw), sdk_result=result)
                return result

        class ReadProxy:
            def __init__(self, bus):
                self.bus = bus

            def __getattr__(self, name):
                return getattr(self.bus, name)

            def txRxPacket(self):
                begin = tracer.clock()
                result = self.bus.txRxPacket()
                end = tracer.clock()
                ok = result == gui.COMM_SUCCESS
                raw = {}
                if ok:
                    for motor_id in editor.online_joints:
                        if self.bus.isAvailable(motor_id, gui.ADDR_PRESENT_POSITION, gui.LEN_PRESENT_POSITION):
                            raw[int(motor_id)] = self.bus.getData(motor_id, gui.ADDR_PRESENT_POSITION, gui.LEN_PRESENT_POSITION)
                context = ({k: v for k, v in tracer.context.items() if k != 'write_kind'}
                           if tracer.context is not None else tracer.state(editor))
                tracer.record('packet_read', begin, end, context, ok,
                              raw=raw, sdk_result=result)
                return result

        editor.groupSyncWrite = WriteProxy(editor.groupSyncWrite)
        editor.groupSyncRead = ReadProxy(editor.groupSyncRead)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--motion-name', help='Label for an unsaved/composed motion')
    args = parser.parse_args()
    source = dict(path=str(args.gui.resolve()), sha256=hashlib.sha256(args.gui.read_bytes()).hexdigest())
    spec = importlib.util.spec_from_file_location('instrumented_sdk_gui', args.gui.resolve())
    gui = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(args.gui.resolve().parent))
    spec.loader.exec_module(gui)
    writer = TraceWriter(args.output, source)
    try:
        tracer = GuiTrace(writer.emit, args.motion_name)
        tracer.install(gui)
        app = gui.QApplication([str(args.gui)])
        app.setStyle('Fusion')
        editor = gui.SDKMotionEditor()
        tracer.attach_bus(gui, editor)
        editor.show()
        app.exec_()
    finally:
        writer.close()


if __name__ == '__main__':
    main()
