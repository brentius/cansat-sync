"""GPL-3.0-only. Decides when a telemetry packet may be transmitted.

The primary mission needs one packet per second from release to landing, and
the can may sit powered on the pad for hours first. In a sub-band with a tight
duty-cycle limit those two facts conflict unless the pad rate is throttled and
airtime is held back for the flight. This scheduler does both, and never sends
a packet the limiter has not approved.
"""
from .flight_state import State
from .lora import DutyCycleLimiter


class DownlinkScheduler:
    def __init__(self, airtime_s, duty_limit, window_s=3600.0, flight_interval_s=1.0,
                 flight_reserve_s=60.0, ground_interval_s=None):
        """flight_reserve_s is the flight duration to keep airtime for."""
        self.airtime = airtime_s
        self.limiter = DutyCycleLimiter(duty_limit, window_s)
        self.flight_interval = flight_interval_s
        self.reserve = flight_reserve_s/flight_interval_s*airtime_s
        spare = duty_limit*window_s - self.reserve
        if spare < airtime_s:
            raise ValueError('duty limit cannot carry the flight reserve')
        steady = window_s*airtime_s/spare
        self.ground_interval = max(flight_interval_s, steady, ground_interval_s or 0.0)
        self._last = None

    def due(self, t, state):
        """True when a packet should be sent now; the caller must then send it."""
        flying = state in (State.ASCENT, State.DESCENT)
        interval = self.flight_interval if flying else self.ground_interval
        if self._last is not None and t - self._last < interval - 1e-9:
            return False
        reserve = self.reserve if state == State.PAD else 0.0
        if not self.limiter.allowed(t, self.airtime, reserve):
            return False
        self.limiter.record(t, self.airtime)
        self._last = t
        return True
