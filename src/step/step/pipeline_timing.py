"""Optional diagnostic metadata; never used to accept or reject motion inputs.

ROS time measures capture age. Monotonic timestamps measure processing and
callback handoffs between processes on the same robot/host.
"""

import json
import math
import time


def elapsed_ms(start, end):
    if type(start) is not int or type(end) is not int or start <= 0 or end < start:
        return None
    return (end - start) / 1e6


def ros_time_ns(node):
    try:
        return int(node.get_clock().now().nanoseconds)
    except Exception:
        return None


def extend_trace(trace, stamp, **times):
    """Ignore missing/mismatched diagnostics instead of touching control data."""
    if (type(stamp) is not int or stamp <= 0 or not isinstance(trace, dict)
            or trace.get('rgb_stamp_ns') != stamp):
        return None
    keys = ('rgb_stamp_ns', 'detector_receive_ros_ns', 'detector_receive_mono_ns',
            'detector_publish_mono_ns', 'ball_receive_mono_ns', 'ball_publish_mono_ns')
    result = {k: v for k, v in trace.items() if k in keys and type(v) is int and v > 0}
    result.update(times)
    return result


def record_timing(node, attribute, tag, values, context, now_ns, *, force=False):
    """Bound memory and routine log rate to one summary every two seconds."""
    try:
        state = getattr(node, attribute, None)
        if state is None:
            state = {'started': now_ns, 'last': now_ns, 'metrics': {}, 'samples': 0}
            setattr(node, attribute, state)
        state['samples'] += 1
        for key, value in values.items():
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                continue
            metric = state['metrics'].setdefault(key, [0, 0., 0.])
            metric[0] += 1
            metric[1] += value
            metric[2] = max(metric[2], value)
        if not force and now_ns - state['last'] < 2_000_000_000:
            return
        output = dict(context, samples=state['samples'],
                      window_ms=round((now_ns - state['started']) / 1e6, 3),
                      latest_ms={k: round(v, 3) if v is not None else None for k, v in values.items()},
                      summary_ms={k: {'n': v[0], 'avg': round(v[1]/v[0], 3), 'max': round(v[2], 3)}
                                  for k, v in state['metrics'].items()})
        node.get_logger().info(f'[{tag}] ' + json.dumps(output, separators=(',', ':')))
        state.update(started=now_ns, last=now_ns, metrics={}, samples=0)
    except Exception:
        # Diagnostic failures must never skip detection, alter a decision or stop motion.
        return


def ball_pipeline_values(trace, decision_receive_ns, decision_receive_ros_ns, decision_done_ns):
    return {
        'capture_to_detector': elapsed_ms(trace.get('rgb_stamp_ns'), trace.get('detector_receive_ros_ns')),
        'detector_work': elapsed_ms(trace.get('detector_receive_mono_ns'), trace.get('detector_publish_mono_ns')),
        'detector_to_ball': elapsed_ms(trace.get('detector_publish_mono_ns'), trace.get('ball_receive_mono_ns')),
        'ball_work': elapsed_ms(trace.get('ball_receive_mono_ns'), trace.get('ball_publish_mono_ns')),
        'ball_to_decision': elapsed_ms(trace.get('ball_publish_mono_ns'), decision_receive_ns),
        'capture_to_decision': elapsed_ms(trace.get('rgb_stamp_ns'), decision_receive_ros_ns),
        'decision_callback': elapsed_ms(decision_receive_ns, decision_done_ns),
    }


def record_ball_pipeline(node, payload, received_ns, received_ros_ns, status):
    try:
        if not isinstance(payload, dict):
            return
        stamp = payload.get('rgb_stamp_ns')
        trace = extend_trace(payload.get('diagnostic_timing'), stamp)
        if trace is None:
            return
        now_ns = time.monotonic_ns()
        confirmed = payload.get('detected') is True and payload.get('confirmation_confirmed') is True
        track = payload.get('confirmation_track_id')
        force = confirmed and type(track) is int and track != getattr(node, '_timing_confirmed_track', None)
        if force:
            node._timing_confirmed_track = track
        record_timing(node, '_ball_pipeline_timing', 'BALL_PIPELINE_TIMING',
                      ball_pipeline_values(trace, received_ns, received_ros_ns, now_ns),
                      dict(rgb_stamp_ns=stamp, track_id=track,
                           hits=payload.get('confirmation_hits'), confirmed=confirmed,
                           raw_detected=payload.get('raw_detected'), input_status=status,
                           confirmation_event=force), now_ns, force=force)
    except Exception:
        return
