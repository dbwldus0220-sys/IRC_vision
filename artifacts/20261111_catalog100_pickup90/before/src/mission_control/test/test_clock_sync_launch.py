"""Test startup gating without launching hardware or querying host services."""

import subprocess

import pytest
from launch import LaunchContext
from launch.actions import EmitEvent, TimerAction

from mission_control import clock_sync_launch as sync


@pytest.fixture(autouse=True)
def log_directory(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))


def context(**overrides):
    value = LaunchContext()
    value.launch_configurations.update({
        'enable_camera': 'true', 'require_time_sync': 'true',
        'time_sync_timeout_sec': '60.0', **overrides,
    })
    return value


def test_graph_waits_for_sync_and_one_stable_second(monkeypatch):
    now, ready = [10.], [False]
    monkeypatch.setattr(sync.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(sync.time, 'time', lambda: now[0] + 1000.)
    monkeypatch.setattr(sync, 'system_clock_synchronized', lambda: ready[0])
    marker = object()
    gate = sync.ClockSynchronizedStart([marker])
    assert marker not in gate.execute(context())
    ready[0] = True
    now[0] += 1.
    assert marker not in gate.execute(context())
    now[0] += 1.
    assert marker in gate.execute(context())


def test_timeout_never_releases_graph(monkeypatch):
    now = [10.]
    monkeypatch.setattr(sync.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(sync, 'system_clock_synchronized', lambda: False)
    marker = object()
    gate = sync.ClockSynchronizedStart([marker])
    assert isinstance(gate.execute(context())[-1], TimerAction)
    now[0] += 61.
    result = gate.execute(context())
    assert len(result) == 1 and isinstance(result[0], EmitEvent)
    assert marker not in result


@pytest.mark.parametrize('override', [{'enable_camera': 'false'}, {'require_time_sync': 'false'}])
def test_external_camera_or_explicit_offline_override(monkeypatch, override):
    def unexpected():
        pytest.fail('Synchronization probe must not run')
    monkeypatch.setattr(sync, 'system_clock_synchronized', unexpected)
    marker = object()
    assert sync.ClockSynchronizedStart([marker]).execute(context(**override)) == [marker]


@pytest.mark.parametrize('reply', ['yes\n', 'no\n', ''])
def test_service_probe_requires_affirmative_response(monkeypatch, reply):
    monkeypatch.setattr(sync.subprocess, 'run', lambda *args, **kwargs:
                        subprocess.CompletedProcess(args[0], 0, stdout=reply))
    assert sync.system_clock_synchronized() is (reply == 'yes\n')


def test_missing_service_fails_closed(monkeypatch):
    def unavailable(*args, **kwargs):
        raise FileNotFoundError()
    monkeypatch.setattr(sync.subprocess, 'run', unavailable)
    assert not sync.system_clock_synchronized()


def test_launch_service_releases_children_once_after_timer(monkeypatch):
    from launch import LaunchDescription, LaunchService
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.events import Shutdown

    monkeypatch.setattr(sync, 'system_clock_synchronized', lambda: True)
    executed = []

    def child(_context):
        executed.append(True)
        return [EmitEvent(event=Shutdown(reason='test complete'))]

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('require_time_sync', default_value='true'),
        DeclareLaunchArgument('time_sync_timeout_sec', default_value='5.0'),
        sync.ClockSynchronizedStart([OpaqueFunction(function=child)]),
    ]))
    assert service.run() == 0
    assert executed == [True]
