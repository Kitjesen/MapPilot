"""Unified clock source -- switchable between real-time and simulation time.

Usage::

    from runtime.clock import clock

    now = clock.now()        # seconds (float)
    clock.sleep(0.1)         # sleep (respects sim speed)
    clock.set_sim_time(t)    # set sim time externally
    clock.set_real_time()    # switch back to wall clock
"""

import threading
import time


class Clock:
    """Time source that can switch between real and simulated time.

    In *real-time* mode (the default), :meth:`now` delegates to
    ``time.time()`` and :meth:`sleep` to ``time.sleep()``.

    In *simulation* mode (activated by :meth:`set_sim_time`), ``now()``
    returns the last value set externally and ``sleep()`` scales the
    wall-clock sleep by the inverse of :attr:`sim_speed`.
    """

    def __init__(self) -> None:
        self._sim_time: float | None = None
        self._sim_speed: float = 1.0
        self._lock = threading.Lock()

    # -- Properties ----------------------------------------------------------

    @property
    def is_sim(self) -> bool:
        """True when the clock is in simulation mode."""
        return self._sim_time is not None

    @property
    def sim_speed(self) -> float:
        """Current simulation speed multiplier (default 1.0)."""
        return self._sim_speed

    # -- Core API ------------------------------------------------------------

    def now(self) -> float:
        """Return current time in seconds.

        Returns simulated time if set, otherwise ``time.time()``.
        """
        with self._lock:
            if self._sim_time is not None:
                return self._sim_time
        return time.time()

    def sleep(self, seconds: float) -> None:
        """Sleep for *seconds*, scaled by sim speed when in sim mode.

        In simulation mode the actual wall-clock sleep is
        ``seconds / sim_speed``, so a speed of 2.0 halves the wait.
        """
        if seconds <= 0:
            return
        if self._sim_time is not None:
            time.sleep(seconds / self._sim_speed)
        else:
            time.sleep(seconds)

    # -- Mode switching ------------------------------------------------------

    def set_sim_time(self, t: float) -> None:
        """Enter simulation mode and set the current sim time to *t*."""
        with self._lock:
            self._sim_time = t

    def set_real_time(self) -> None:
        """Switch back to real-time (wall-clock) mode."""
        with self._lock:
            self._sim_time = None

    def set_sim_speed(self, speed: float) -> None:
        """Set the simulation speed multiplier (clamped to >= 0.01)."""
        self._sim_speed = max(0.01, speed)


# ---------------------------------------------------------------------------
# Global singleton
# ---------------------------------------------------------------------------

clock = Clock()
