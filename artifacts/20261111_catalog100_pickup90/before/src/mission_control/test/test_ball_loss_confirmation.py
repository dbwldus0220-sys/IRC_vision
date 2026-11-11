"""Ball-loss debounce tests without ROS or robot motion."""

import pytest

from mission_control.ball_loss_confirmation import BallLossConfirmation


def observe(loss, t, *, stamp=None, **fields):
    info = {"detected": False, "raw_detected": False,
            "rgb_stamp_ns": round(t * 1e9) if stamp is None else stamp}
    info.update(fields)
    return loss.update(info, t, max_gap_sec=0.5)


def test_both_duration_and_five_frames_are_required():
    loss = BallLossConfirmation()
    for t in (10.0, 10.1, 10.2, 10.3, 10.49):
        observe(loss, t)
        assert not loss.confirmed
    observe(loss, 10.5)
    assert loss.confirmed

    loss = BallLossConfirmation()
    for t in (10.0, 10.25, 10.5, 10.75):
        observe(loss, t)
        assert not loss.confirmed
    observe(loss, 11.0)
    assert loss.confirmed


@pytest.mark.parametrize("visible", [
    {"detected": True}, {"raw_detected": True},
])
def test_visible_ball_resets_even_confirmed_loss(visible):
    loss = BallLossConfirmation()
    for t in (10.0, 10.125, 10.25, 10.375, 10.5):
        observe(loss, t)
    assert loss.confirmed
    observe(loss, 10.6, **visible)
    assert not loss.confirmed
    assert loss.missing_frames == 0
    observe(loss, 10.7)
    assert not loss.confirmed
    assert loss.missing_frames == 1


def test_duplicates_and_out_of_order_frames_do_not_refresh_or_count():
    loss = BallLossConfirmation()
    observe(loss, 10.0)
    for t in (10.1, 10.2, 10.3, 10.4, 10.6):
        assert not observe(loss, t, stamp=10_000_000_000)
    assert not observe(loss, 10.7, stamp=9_000_000_000)
    assert loss.missing_frames == 1
    assert loss.last_received_at == 10.0
    assert not loss.confirmed


@pytest.mark.parametrize("gap", ["receipt", "camera", "both"])
def test_input_gap_restarts_continuous_evidence(gap):
    loss = BallLossConfirmation()
    for t in (10.0, 10.1, 10.2, 10.3):
        observe(loss, t)
    observe(loss, 11.0 if gap != "camera" else 10.4,
            stamp=11_000_000_000 if gap != "receipt" else 10_400_000_000)
    assert loss.missing_frames == 1
    assert not loss.confirmed


def test_delayed_burst_cannot_confirm_loss_on_arrival():
    loss = BallLossConfirmation()
    for i in range(6):
        observe(loss, 20.0 + i * 0.01, stamp=round((10.0 + i * 0.1) * 1e9))
    assert loss.missing_frames == 6
    assert not loss.confirmed


@pytest.mark.parametrize("stamp", [None, 0, True, "10000000000"])
def test_unstamped_misses_do_not_prove_new_frames(stamp):
    loss = BallLossConfirmation()
    for i in range(10):
        loss.update({"detected": False, "rgb_stamp_ns": stamp}, 10 + i * 0.1, 0.5)
    assert not loss.confirmed
    assert loss.missing_frames == 0
