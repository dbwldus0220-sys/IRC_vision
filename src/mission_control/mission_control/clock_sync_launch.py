"""Delay the camera and motion graph until the host clock is synchronized."""

import subprocess
import time

from launch import Action
from launch.actions import EmitEvent, LogInfo, TimerAction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def system_clock_synchronized():
    try:
        result = subprocess.run(
            ['timedatectl', 'show', '--property=NTPSynchronized', '--value'],
            capture_output=True, text=True, timeout=1.0, check=False,
        )
        return result.returncode == 0 and result.stdout.strip() == 'yes'
    except (OSError, subprocess.TimeoutExpired):
        return False


class ClockSynchronizedStart(Action):
    """Poll without blocking launch shutdown or starting any robot process."""

    def __init__(self, actions):
        super().__init__()
        self.actions = actions
        self.started_at = None
        self.synchronized_at = None
        self.last_clock_pair = None

    def get_sub_entities(self):
        return self.actions

    def execute(self, context):
        enabled = LaunchConfiguration('enable_camera').perform(context).lower() in {'true', '1'}
        required = LaunchConfiguration('require_time_sync').perform(context).lower() in {'true', '1'}
        if not enabled or not required:
            return self.actions
        now = time.monotonic()
        wall = time.time()
        first = self.started_at is None
        if first:
            self.started_at = now
        stable = (self.last_clock_pair is None or
                  abs((wall - self.last_clock_pair[0]) - (now - self.last_clock_pair[1])) <= 0.1)
        self.last_clock_pair = (wall, now)
        if system_clock_synchronized() and stable:
            if self.synchronized_at is None:
                self.synchronized_at = now
            if now - self.synchronized_at >= 1.0:
                return [LogInfo(msg='System time synchronized; starting camera and motion graph.'), *self.actions]
        else:
            self.synchronized_at = None
        timeout = float(LaunchConfiguration('time_sync_timeout_sec').perform(context))
        if now - self.started_at >= timeout:
            return [EmitEvent(event=Shutdown(reason='System time synchronization timed out; camera and motion graph were not started.'))]
        retry = TimerAction(period=1.0, actions=[self])
        return [LogInfo(msg='Waiting for system time synchronization before camera startup.'), retry] if first else [retry]
