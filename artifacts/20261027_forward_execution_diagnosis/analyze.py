"""Offline data audit and virtual-clock playback; never opens a motor port."""
import ast
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SDK = Path('/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step')


class Dummy:
    def __getattr__(self, name):
        if name == 'findChildren':
            return lambda *args: []
        if name == 'height':
            return lambda: 100
        if name == 'horizontalScrollBar':
            return lambda: self
        return lambda *args, **kwargs: None


def gui_virtual(motion, read_delay, visual_delay):
    methods = {'normalize_angles', 'shortest_angle_delta', 'interpolate_angle_shortest',
               'frame_motion_progress', 'anim_step', 'scrub_timeline',
               'gate_timeline_on_frame_arrival', 'resort_motion_sequence',
               'start_motion_playback'}
    tree = ast.parse((SDK / 'sdk_gui.py').read_text())
    clock = [1000.0]
    ns = dict(math=math, copy=copy, time=SimpleNamespace(perf_counter=lambda: clock[0]/1000),
              TimelineBlockWidget=object)
    for node in tree.body:
        if isinstance(node, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets):
            try:
                exec(compile(ast.Module(body=[node], type_ignores=[]), '<constants>', 'exec'), ns)
            except (NameError, TypeError):
                pass
    selected = [n for c in tree.body if isinstance(c, ast.ClassDef)
                for n in c.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    assert {n.name for n in selected} == methods
    cls = ast.ClassDef(name='Gui', bases=[], keywords=[], body=selected, decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), '<gui>', 'exec'), ns)
    p = ns['Gui']()
    p.online_joints = set(range(23))
    p.joints = {i: i*.25-3 for i in range(23)}
    p.start_angles = {}
    p.robot_scrub_command_angles = {}
    p.robot_sync_enabled = p.execute_on_real_robot = True
    p.is_playing = p.is_paused = p.playback_paused_by_button = False
    p.playback_context = 'motion'
    p.SCALE = 1
    p.last_playback_ui_time = 0
    for attr in ('anim_timer', 'robot_scrub_timer', 'timeline_container', 'timeline_scroll', 'spin_max_time'):
        setattr(p, attr, Dummy())
    p.update_playback_buttons = lambda: None
    p.refresh_timeline_ui = lambda: None
    p.set_tracking_target_angles = lambda *args: None
    p.configure_unlimited_motor_profiles = lambda *args: True
    p.sequence_joint_ids = lambda: list(range(23))
    reads, writes, visual = [], [], []

    def read():
        reads.append(clock[0])
        clock[0] += read_delay
        return True

    def write(*args, **kwargs):
        writes.append(clock[0])
        return True

    def render(*args):
        visual.append(clock[0])
        clock[0] += visual_delay

    p.sync_sequence_start_from_robot = read
    p.read_feedback_and_report_fk = read
    p.write_goal_positions = write
    p.update_3d_robot = render
    p.stop_motion_sequence = lambda **kwargs: setattr(p, 'is_playing', False)
    p.motion_sequence = copy.deepcopy(motion['frames'])
    p.max_seq_ms = motion['max_seq_ms']
    p.playback_repeat_target = motion['repeat_count']
    p.playback_repeat_current = 1
    p.playback_speed = motion['playback_speed']
    with contextlib.redirect_stdout(io.StringIO()):
        p.start_motion_playback(real_robot=True)
        start = clock[0]
        next_tick = start + 5
        for _ in range(10000):
            clock[0] = max(clock[0], next_tick)
            tick_start = clock[0]
            p.anim_step()
            next_tick = tick_start + 5
            if not p.is_playing:
                return dict(duration_ms=round(clock[0]-start, 3), reads=len(reads)-1,
                            writes=len(writes), assumed_read_ms=read_delay,
                            assumed_visual_ms=visual_delay)
    raise AssertionError('GUI playback did not finish')


def main():
    source_path = ROOT / 'motion_imports/robot_motions(40)(1).json'
    runtime_path = ROOT / 'artifacts/robot_motions_runtime.json'
    source = next(m for m in json.loads(source_path.read_text())['motions'] if m['name'] == '전.')
    runtime = next(m for m in json.loads(runtime_path.read_text())['motions'] if m['name'] == '전45도(6회)')
    comparison = copy.deepcopy(source)
    comparison['repeat_count'] = 6
    catalog = OUT / 'virtual_catalog.json'
    catalog.write_text(json.dumps({'version': 1, 'motions': [comparison, runtime]}, ensure_ascii=False, indent=2))
    binary = '/tmp/step_forward_probe'
    subprocess.run(['g++', '-std=c++20', '-pthread', '-I', str(SDK), str(OUT/'probe.cpp'),
                    str(ROOT/'build/irc_step_motion_executor/robot_motion_sdk/libmotion_player_core.a'),
                    str(ROOT/'build/irc_step_motion_executor/robot_motion_sdk/libmotion_core.a'),
                    '-o', binary], check=True)
    results = {'scope': 'Synthetic clock and mock hardware; not measured robot timing or yaw',
               'sdk': str(SDK), 'inputs': {}, 'angle_differences': [], 'sdk_runs': [], 'gui_runs': []}
    for path in (source_path, runtime_path, SDK/'sdk_gui.py', SDK/'robot_motion_player.cpp',
                 ROOT/'build/irc_step_motion_executor/sdk_motion_executor'):
        results['inputs'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    for i, (a, b) in enumerate(zip(source['frames'], runtime['frames'])):
        assert (a['start_ms'], a['time_ms']) == (b['start_ms'], b['time_ms'])
        for joint, degree in a['angles'].items():
            if degree != b['angles'][joint]:
                results['angle_differences'].append([i+1, joint, degree, b['angles'][joint]])
    for name, queued in [('전.', False), ('전45도(6회)', False), ('전45도(6회)', True)]:
        result = subprocess.run([binary, str(catalog), name, str(int(queued))], text=True,
                                capture_output=True, check=True)
        tag = ('runtime' if name != '전.' else 'source') + ('_queued' if queued else '')
        (OUT/f'{tag}_mock.txt').write_text(result.stdout)
        rows = [line.split() for line in result.stdout.splitlines()]
        completions = [dict(time_ms=int(row[1]), sequence=int(row[2]), reads=int(row[3]))
                       for row in rows if row and row[0]=='COMPLETE']
        ending = next(row for row in rows if row and row[0]=='RESULT')
        results['sdk_runs'].append(dict(name=name, queued=queued, completion=completions,
            duration_ms=int(ending[1]), writes=int(ending[2]), reads=int(ending[3]),
            last_motion_endpoint_count=int(ending[4])))
    for read_ms, ui_ms in [(0, 0), (5, 0), (10, 0), (0, 20), (10, 20)]:
        results['gui_runs'].append(gui_virtual(comparison, read_ms, ui_ms))
    results['runtime_gui_same_speed_zero_cost'] = gui_virtual(runtime, 0, 0)
    assert results['sdk_runs'][0]['duration_ms'] == results['gui_runs'][0]['duration_ms']
    assert results['sdk_runs'][1]['duration_ms'] == results['runtime_gui_same_speed_zero_cost']['duration_ms']
    assert all(r['reads'] == 1 and r['last_motion_endpoint_count'] == 24 for r in results['sdk_runs'])
    assert results['sdk_runs'][2]['completion'][0]['time_ms'] > results['sdk_runs'][1]['duration_ms']
    (OUT/'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
