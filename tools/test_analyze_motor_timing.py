"""Keep command gaps distinct from stored holds and independent head movement."""
from analyze_motor_timing import motion_transitions


def row(event, at, run='1', **fields):
    return dict(event=event, begin_ms=str(at), end_ms=str(at), duration_ms='0',
                run=run, motion='forward6', success='1', speed='0.9', **fields)


def test_queued_boundary_preserves_saved_hold_and_ignores_head_change():
    rows = [row('sample', 0, raw_0='2000', raw_6='2100'),
            row('goals_complete', 1), row('run_start', 1, '2'),
            row('definition', 1, '2', frame_start_ms='90'),
            row('queue_activate', 1, '2'),
            row('sample', 5, '2', raw_0='2100', raw_6='2100'),
            row('sample', 120, '2', raw_0='2100', raw_6='2101')]
    transition, = motion_transitions(rows)
    assert transition['kind'] == 'queued'
    assert transition['saved_leading_hold_ms'] == 100
    assert transition['last_tx_end_to_next_tx_start_ms'] == 5
    assert transition['last_tx_end_to_first_body_raw_change_ms'] == 120
    assert transition['initial_position_reads'] == 0


def test_restart_reports_initial_position_read_and_command_gap():
    rows = [row('sample', 0), row('goals_complete', 1),
            row('initial', 200, '2'), row('run_start', 230, '2'),
            row('definition', 230, '2', frame_start_ms='90'), row('sample', 235, '2')]
    transition, = motion_transitions(rows)
    assert transition['kind'] == 'restart'
    assert transition['initial_position_reads'] == 1
    assert transition['last_tx_end_to_next_tx_start_ms'] == 235
    assert transition['last_tx_end_to_first_body_raw_change_ms'] is None


def test_failed_or_later_motion_write_is_not_counted_as_movement():
    failed = row('sample', 5, raw_6='2101')
    failed['success'] = '0'
    rows = [row('sample', 0, raw_6='2100'), row('repeat', 1), failed,
            row('fault', 6), row('sample', 100, '2', raw_6='2200')]
    transition, = motion_transitions(rows)
    assert transition['kind'] == 'repeat'
    assert transition['last_tx_end_to_first_body_raw_change_ms'] is None


def test_final_completion_is_not_a_transition():
    assert motion_transitions([row('sample', 0), row('goals_complete', 1)]) == []


def test_forced_feedback_before_boundary_is_included_in_gap_accounting():
    feedback = row('feedback', 1)
    feedback.update(end_ms='17', duration_ms='16')
    transition, = motion_transitions([
        row('sample', 0), feedback, row('repeat', 17), row('sample', 22)])
    assert transition['feedback_read_duration_before_next_tx_ms'] == 16
    assert transition['last_tx_end_to_next_tx_start_ms'] == 22
