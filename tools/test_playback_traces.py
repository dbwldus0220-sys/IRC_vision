"""Trace instrumentation must preserve commands and reject incomplete replay inputs."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from trace_sdk_gui import GuiTrace, TraceWriter
from compare_playback_traces import load_trace, select_run, definition, compare, export_replay
from verify_latest_gui_execution import load_gui, oracle

ROOT = Path(__file__).resolve().parents[1]


def fake_gui():
    class Editor:
        def __init__(self):
            self.motion_sequence = [dict(name='frame', start_ms=0, time_ms=20, angles={'0': 10})]
            self.current_timeline_ms = 0
            self.playback_repeat_current = self.playback_repeat_target = 1
            self.playback_speed = 1.0
            self.playback_context = 'motion'
            self.max_seq_ms = 20
            self.start_angles = {i: 0 for i in range(23)}
            self.feedback_angles = {0: 99}
            self.online_joints = {0}
            self.is_playing = False
            self.anim_start_time = 1.0
            self.timed_gate_frame = None
            self.commands = []
            self.read_ok = True

        def start_motion_playback(self, **kw):
            self.is_playing = True

        def anim_step(self):
            t = self.gate_timeline_on_frame_arrival(20, 1.020)
            self.scrub_timeline(t)

        def gate_timeline_on_frame_arrival(self, t, now):
            if self.timed_gate_frame:
                self.write_goal_positions({0: 10}, target_ids=[0], read_feedback=False)
                self.timed_gate_frame = None
            return t

        def scrub_timeline(self, t_ms, **kw):
            self.current_timeline_ms = t_ms
            self.update_3d_robot({0: 10})
            self.write_goal_positions({0: 10}, [0], False)
            self.read_feedback_and_report_fk()

        def normalize_angles(self, angles):
            return {int(k): float(v) for k, v in angles.items()}

        def angle_to_dxl_position(self, angle):
            return round(2048 + angle * 4096 / 360)

        def write_goal_positions(self, angles_dict, target_ids=None, read_feedback=True):
            self.commands.append((angles_dict.copy(), target_ids, read_feedback))
            return True

        def read_feedback_and_report_fk(self):
            return self.read_ok

        def sync_sequence_start_from_robot(self):
            return self.read_ok

        def restart_playback_cycle(self, *args):
            self.current_timeline_ms = 0

        def stop_motion_sequence(self, **kwargs):
            self.is_playing = False

        def update_3d_robot(self, *args):
            pass

    class Graph:
        def __init__(self, editor):
            self.parent_gui = editor

        def paintEvent(self, event):
            self.trajectory_samples([])

        def trajectory_samples(self, ids):
            return []

    class Block(Graph):
        def set_default_style(self):
            pass

        def set_playing_style(self):
            pass

    return SimpleNamespace(SDKMotionEditor=Editor, JointTrajectoryWidget=Graph, TimelineBlockWidget=Block)


def test_gui_preserves_commands_and_records_endpoint_paint_and_failed_read():
    plain = fake_gui().SDKMotionEditor()
    plain.start_motion_playback()
    plain.timed_gate_frame = plain.motion_sequence[0]
    plain.anim_step()
    gui = fake_gui()
    rows = []
    GuiTrace(rows.append, 'test').install(gui)
    editor = gui.SDKMotionEditor()
    original = copy.deepcopy(editor.motion_sequence)
    editor.start_motion_playback()
    editor.timed_gate_frame = editor.motion_sequence[0]
    editor.read_ok = False
    editor.anim_step()
    gui.JointTrajectoryWidget(editor).paintEvent(None)
    gui.TimelineBlockWidget(editor).set_default_style()
    assert editor.commands == plain.commands
    assert editor.motion_sequence == original
    writes = [r for r in rows if r['event'] in {'endpoint', 'sample'}]
    assert [r['event'] for r in writes] == ['endpoint', 'sample']
    assert all(r['timeline_ms'] == 20 and r['frame'] == 0 for r in writes)
    assert all(r['evaluate_ns'] == 1020000000 for r in writes)
    failed = next(r for r in rows if r['event'] == 'feedback')
    assert failed['success'] is False and failed['angles'] == {}
    assert {'gui_plot_paint', 'gui_plot_compute', 'gui_style'} <= {r['event'] for r in rows}
    assert next(r for r in rows if r['event'] == 'tick')['evaluate_ns'] == 1020000000
    editor.motion_sequence[0]['angles']['0'] = 50
    assert next(r for r in rows if r['event'] == 'run_start')['snapshot']['frames'][0]['angles']['0'] == 10


def test_writer_closes_and_does_not_overwrite(tmp_path):
    path = tmp_path/'gui.jsonl'
    writer = TraceWriter(path, {})
    writer.emit(dict(event='example', snapshot={'angles': {'0': 10}}))
    writer.close()
    data = load_trace(path)
    assert data['complete'] and data['dropped_rows'] == 0
    assert len(data['rows'][1]['snapshot_sha256']) == 64
    with pytest.raises(FileExistsError):
        TraceWriter(path, {})


def test_gui_packet_capture_excludes_rejected_motor_parameters():
    gui = fake_gui()
    gui.COMM_SUCCESS = 0

    class Bus:
        def clearParam(self):
            pass

        def addParam(self, motor_id, data):
            return motor_id == 0

        def txPacket(self):
            return 0

    def write(editor, angles_dict, target_ids=None, read_feedback=True):
        editor.groupSyncWrite.clearParam()
        for motor_id, angle in angles_dict.items():
            editor.groupSyncWrite.addParam(motor_id, editor.angle_to_dxl_position(angle).to_bytes(4, 'little'))
        return editor.groupSyncWrite.txPacket() == 0

    gui.SDKMotionEditor.write_goal_positions = write
    rows = []
    tracer = GuiTrace(rows.append)
    tracer.install(gui)
    editor = gui.SDKMotionEditor()
    editor.groupSyncWrite, editor.groupSyncRead = Bus(), object()
    tracer.attach_bus(gui, editor)
    assert editor.write_goal_positions({0: 10, 1: 20}, target_ids=[0, 1])
    row = next(r for r in rows if r['event'] == 'write')
    assert row['goal_source'] == 'packet_params'
    assert set(row['raw']) == {0}
    assert set(row['requested_raw']) == {0, 1}
    assert set(row['angles']) == {0}


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    directory = tmp_path_factory.mktemp('playback-binaries')
    sdk = ROOT/'external_sdk/gui_aligned'
    result = {}
    for name in ['gui_execution_probe', 'recorded_execution_probe']:
        binary = directory/name
        subprocess.run(['g++', '-std=c++20', '-O0', '-pthread', '-I', str(sdk),
                        str(sdk/f'{name}.cpp'), str(sdk/'robot_motion_player.cpp'),
                        str(sdk/'motion_pattern.cpp'), '-o', str(binary)], check=True)
        result[name] = binary
    return result


def motion_fixture():
    return dict(name='recorded_gui', max_seq_ms=1000, repeat_count=2, playback_speed=1,
                start_pose='same', end_pose='same', frames=[
                    dict(name='오들', start_ms=0, time_ms=20, angles={'0': 10, '1': -20}, lift_early_arrival=True),
                    dict(name='end', start_ms=25, time_ms=30, angles={'0': -5})])


def test_sdk_trace_preserves_io_and_disambiguates_queued_runs(tmp_path, binaries):
    catalog = tmp_path/'catalog.json'
    motion = motion_fixture()
    catalog.write_text(json.dumps({'motions': [motion]}, ensure_ascii=False))
    cmd = [str(binaries['gui_execution_probe']), str(catalog), motion['name'], '5,17', '2', '1', motion['name']]
    plain = subprocess.check_output(cmd, text=True)
    path = tmp_path/'sdk.csv'
    traced = subprocess.check_output([*cmd, str(path)], text=True)
    assert traced == plain  # Every write/read/raw/timeline event, with tracing enabled vs disabled.
    trace = load_trace(path)
    assert trace['complete'] and trace['dropped_rows'] == 0
    first, start = select_run(trace, 1)
    second, _ = select_run(trace, 2)
    assert any(r['event'] == 'queue_activate' for r in second)
    initial = next(r for r in first if r['event'] == 'initial')
    assert (initial['frame'], initial['repeat'], initial['timeline_ms']) == (-1, 1, 0)
    effective, _ = definition(first, start)
    assert effective['frames'][0]['angles'] == {'0': 10., '1': -20.}
    report = compare(trace, trace, 1, 1)
    assert report['effective_definition_equal']
    assert report['matched_raw_mismatches'] == 0
    assert report['matched_schedule_abs_error_ms']['max'] == 0


def measured_fixture(tmp_path):
    """Synthetic log made from real GUI methods; explicitly not a physical measurement."""
    motion = motion_fixture()
    ns = load_gui(ROOT/'artifacts/20261006_gui_alignment/reference/sdk_gui.py')
    periods, read_delay, write_delay = [5, 17, 6], 2, 1
    records = oracle(ns, motion, periods, read_delay, write_delay)
    initial = {i: i*.25-3 for i in range(23)}
    origin_ms = 1000 + read_delay
    rows = [dict(event='session', schema_version=2),
            dict(event='run_start', run=1, event_seq=1, tick=0, begin_ns=1000000000,
                 end_ns=int(origin_ms*1e6), playback_origin_ns=int(origin_ms*1e6),
                 timeline_ms=0, repeat=1, success=True, gui_context='motion',
                 snapshot=dict(motion, initial_deg=initial))]
    tick = 1
    elapsed = origin_ms
    pending = []
    for record in records[1:]:  # Initial read is captured by initial_deg/run origin.
        if record[0] in ('R', 'W'):
            pending.append(record)
        elif record[0] == 'T':
            evaluate_ms = elapsed + periods[(tick-1) % len(periods)]
            for value in pending:
                write = value[0] == 'W'
                angles = {str(k): v[0] for k, v in value[2].items()} if write else {str(k): v for k, v in initial.items()}
                raw = {str(k): v[1] for k, v in value[2].items()} if write else {}
                rows.append(dict(event='sample' if write else 'feedback', run=1, tick=tick,
                                 event_seq=len(rows), begin_ns=round(value[1]*1e6),
                                 end_ns=round((value[1]+(write_delay if write else read_delay))*1e6),
                                 success=True, angles=angles, raw=raw))
            rows.append(dict(event='tick', run=1, tick=tick, event_seq=len(rows),
                             begin_ns=round(evaluate_ms*1e6), evaluate_ns=round(evaluate_ms*1e6),
                             end_ns=round(record[1]*1e6), success=True))
            elapsed = record[1]
            tick += 1
            pending = []
    rows.append(dict(event='trace_end', dropped_rows=0))
    path = tmp_path/'synthetic_gui.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
    return path


def test_recorded_clock_drives_real_cpp_player_and_detects_changed_goal(tmp_path, binaries):
    trace = load_trace(measured_fixture(tmp_path))
    directory = tmp_path/'replay'
    export_replay(trace, 1, directory)
    cmd = [str(binaries['recorded_execution_probe']), str(directory/'catalog.json'), str(directory/'schedule.txt')]
    result = subprocess.run(cmd, text=True, capture_output=True)
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert json.loads(result.stdout)['hardware_accessed'] is False
    catalog = json.loads((directory/'catalog.json').read_text())
    catalog['motions'][0]['frames'][0]['angles']['0'] += 10
    (directory/'catalog.json').write_text(json.dumps(catalog, ensure_ascii=False))
    result = subprocess.run(cmd, text=True, capture_output=True)
    assert result.returncode == 1
    assert json.loads(result.stdout)['goal_mismatches'] > 0


def test_replay_rejects_dropped_resume_and_composer(tmp_path):
    trace = load_trace(measured_fixture(tmp_path))
    for field, value in [('dropped_rows', 1), ('complete', False)]:
        invalid = copy.deepcopy(trace)
        invalid[field] = value
        with pytest.raises(ValueError):
            export_replay(invalid, 1, tmp_path/'invalid')
    for field, value in [('timeline_ms', 10), ('gui_context', 'composer')]:
        invalid = copy.deepcopy(trace)
        next(r for r in invalid['rows'] if r['event'] == 'run_start')[field] = value
        with pytest.raises(ValueError):
            export_replay(invalid, 1, tmp_path/'invalid')
