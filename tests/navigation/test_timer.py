"""Unit tests for missile.navigation.timer.InternalTimer."""
import time

from missile.navigation.timer import InternalTimer


def test_timer_not_running_initially():
    t = InternalTimer()
    assert t.is_running is False
    assert t.absolute_start_time is None
    assert t.get_time_elapsed() == 0.0


def test_get_launched_time_none_before_start():
    t = InternalTimer()
    assert t.get_launched_time() is None


def test_start_sets_running_and_wallclock():
    t = InternalTimer()
    before = time.time()
    t.start()
    after = time.time()
    assert t.is_running is True
    assert before <= t.get_launched_time() <= after


def test_elapsed_time_increases_after_start():
    t = InternalTimer()
    t.start()
    e1 = t.get_time_elapsed()
    time.sleep(0.01)
    e2 = t.get_time_elapsed()
    assert e1 >= 0.0
    assert e2 > e1


def test_elapsed_is_zero_when_never_started():
    t = InternalTimer()
    time.sleep(0.005)
    assert t.get_time_elapsed() == 0.0
