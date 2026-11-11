"""Diagnostic traces must not change Ball entry decisions or reservations."""

import time
import pytest

from test_ball_entry_candidate_history import start, emit
from test_continuous_approach import clock


@pytest.mark.parametrize('metadata', ['valid', 'wrong_frame', 'malformed'])
def test_diagnostic_metadata_does_not_change_control_messages(clock, metadata):
    outputs = []
    for enabled in (False, True):
        clock[0] = 10.
        node, _, _ = start(clock)
        logs = []
        class Logger:
            def info(self, value):
                logs.append(value)
            def warning(self, value):
                logs.append(value)
        node.get_logger = lambda: Logger()
        for ratio in (.61, .64, .71):
            stamp = round((node.active_line_motion_started_at + node.active_line_motion_duration_sec * ratio) * 1e9)
            now = time.monotonic_ns()
            trace = dict(rgb_stamp_ns=stamp, detector_receive_ros_ns=stamp,
                         detector_receive_mono_ns=now-100_000_000,
                         detector_publish_mono_ns=now-70_000_000,
                         ball_receive_mono_ns=now-50_000_000,
                         ball_publish_mono_ns=now-40_000_000)
            if metadata == 'wrong_frame':
                trace['rgb_stamp_ns'] -= 1
            elif metadata == 'malformed':
                trace = 'bad diagnostic metadata'
            emit(node, clock, ratio, confirmed=ratio == .71,
                 changes={'diagnostic_timing': trace} if enabled else None)
        outputs.append((node.publisher.messages, node.queued_general_command_id))
        timing_logs = [line for line in logs if line.startswith('[BALL_PIPELINE_TIMING]')]
        assert len(timing_logs) == (1 if enabled and metadata == 'valid' else 0)
    assert outputs[0] == outputs[1]
