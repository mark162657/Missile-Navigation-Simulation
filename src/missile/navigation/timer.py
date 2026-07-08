import time


class InternalTimer:
    """
    Onboard navigation clock for multi-rate sensor scheduling (INS / GPS / TERCOM).
    """
    def __init__(self):
        self.absolute_start_time = None
        self._start_perf_counter = None
        self.is_running = False

    def start(self) -> None:
        """Start the timer at launch."""
        self.absolute_start_time = time.time()
        self._start_perf_counter = time.perf_counter()
        self.is_running = True

    def get_time_elapsed(self) -> float:
        """Return seconds elapsed since start()."""
        if not self.is_running:
            return 0.0
        return time.perf_counter() - self._start_perf_counter

    def get_launched_time(self) -> float:
        """Return the wall-clock time when the missile was launched."""
        return self.absolute_start_time
