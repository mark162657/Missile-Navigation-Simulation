"""
clock.py -- wall-clock pacing for the simulation loop.

Keeps sim time locked to real time (or a multiple of it) by sleeping until
the absolute deadline for the current sim_time:

    deadline = t0 + sim_time / realtime_factor

Using an absolute deadline (instead of sleep(dt) every tick) corrects for
accumulated drift: a slow tick shortens the next sleep, a fast tick lengthens
it. At 500 Hz this matters because time.sleep resolution is coarse relative
to dt=0.002 s.

realtime_factor:
    1.0  -> 1 sim second = 1 wall second  (real-time)
    2.0  -> 1 sim second = 0.5 wall seconds (2x faster)
    0.5  -> 1 sim second = 2 wall seconds  (half speed)
    0.0  -> no pacing (as-fast-as-possible)
"""

from __future__ import annotations

import time


class RealtimePacer:
    """Pace a fixed-dt sim loop against wall-clock time."""

    def __init__(self, realtime_factor: float = 1.0) -> None:
        """
        Args:
            realtime_factor: sim_seconds / wall_seconds. 1.0 = real-time,
                0.0 (or negative) disables pacing.
        """
        self.realtime_factor = float(realtime_factor)
        self._t0: float | None = None

    def start(self) -> None:
        """Mark the wall-clock origin for this run (call just before the loop)."""
        self._t0 = time.perf_counter()

    def wait_until(self, sim_time: float) -> float:
        """
        Sleep until wall time catches up to sim_time at the configured rate.

        Returns:
            Seconds slept (0.0 if already behind or pacing disabled).
        """
        if self.realtime_factor <= 0.0 or self._t0 is None:
            return 0.0

        deadline = self._t0 + sim_time / self.realtime_factor
        delay = deadline - time.perf_counter()
        if delay <= 0.0:
            return 0.0

        time.sleep(delay)
        return delay

    @property
    def wall_elapsed(self) -> float:
        """Wall seconds since start(), or 0 if not started."""
        if self._t0 is None:
            return 0.0
        return time.perf_counter() - self._t0
