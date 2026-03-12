import time

class InternalTimer:
    """
    This serve as an internal timer in the missile. Serve as both a stopwatch and a clock.
    """
    def __init__(self):
        self.absolute_start_time = None
        self._start_perf_counter = None

        self.is_running = False

    def start(self) -> None:
        """ Start the timer for launching. """
        self.absolute_start_time = time.time()
        self._start_perf_counter = time.perf_counter()
        self.is_running = True

    def get_time_elapsed(self) -> float:
        """ Return time elapsed since 'start' in seconds. """
        if not self.is_running:
            return 0.0
        return time.perf_counter() - self._start_perf_counter

    def get_launched_time(self) -> float:
        """ Return the time when the missile was launched. """
        return self.absolute_start_time
