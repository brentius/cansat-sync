"""GPL-3.0-only. Flight phase detection from accelerometer magnitude and pressure.

PAD -> ASCENT -> DESCENT -> LANDED. Every transition has an inertial path, a
barometric path and (after launch) a timeout, so one failed sensor delays a
transition rather than preventing it. Thresholds are design values taken from
the competition's stated launch profile (10-20 g for 0.5-1.5 s, 5-7 s coast)
and have not been tuned on flight data.

accel is the magnitude of specific force in m/s^2: about 9.8 at rest, near
zero in free fall. Pass None or NaN for a sensor that has no valid sample.
"""
import math
from collections import deque
from dataclasses import dataclass
from enum import IntEnum

from .atmosphere import G0, pressure_altitude


class State(IntEnum):
    PAD = 0
    ASCENT = 1
    DESCENT = 2
    LANDED = 3


@dataclass
class Config:
    # Launch.
    launch_accel: float = 4*G0
    launch_hold_s: float = 0.15
    launch_alt_m: float = 30.0
    launch_climb_rate: float = 20.0
    launch_alt_hold_s: float = 0.5
    # Pad pressure reference: low-pass of samples older than the delay, so the
    # first seconds of flight never leak into it.
    baseline_tau_s: float = 30.0
    baseline_delay_s: float = 3.0
    # Altitude conditioning.
    median_window_s: float = 0.25
    speed_window_s: float = 1.0
    mean_temp_k: float = 283.15
    # Ejection.
    eject_lockout_s: float = 3.0
    eject_accel: float = 3*G0
    apogee_drop_m: float = 5.0
    apogee_hold_s: float = 0.3
    ascent_timeout_s: float = 15.0
    # Touchdown.
    touchdown_lockout_s: float = 5.0
    touchdown_accel: float = 4*G0
    touchdown_alt_m: float = 60.0
    rest_speed: float = 1.0
    rest_hold_s: float = 2.0
    descent_timeout_s: float = 180.0


class _Hold:
    """True once a condition has held continuously for a duration."""

    def __init__(self, duration):
        self.duration = duration
        self.since = None

    def __call__(self, t, condition):
        if not condition:
            self.since = None
            return False
        if self.since is None:
            self.since = t
        return t - self.since >= self.duration


def _valid(x):
    return x is not None and math.isfinite(x)


class FlightStateMachine:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.state = State.PAD
        self.events = []
        self.baseline_pa = None
        self.altitude = float('nan')
        self.vertical_speed = float('nan')
        self.max_altitude = float('nan')
        self.t_launch = self.t_descent = self.t_landed = None
        self._pending = deque()
        self._baseline_t = None
        self._recent = deque()
        self._history = deque()
        c = self.cfg
        self._launch_accel = _Hold(c.launch_hold_s)
        self._launch_alt = _Hold(c.launch_alt_hold_s)
        self._apogee = _Hold(c.apogee_hold_s)
        self._rest = _Hold(c.rest_hold_s)

    def _track_baseline(self, t, pressure):
        self._pending.append((t, pressure))
        while self._pending and self._pending[0][0] <= t - self.cfg.baseline_delay_s:
            ts, p = self._pending.popleft()
            if self.baseline_pa is None:
                self.baseline_pa = p
            else:
                alpha = 1 - math.exp(-(ts - self._baseline_t)/self.cfg.baseline_tau_s)
                self.baseline_pa += alpha*(p - self.baseline_pa)
            self._baseline_t = ts

    def _update_altitude(self, t, pressure):
        c = self.cfg
        if _valid(pressure) and pressure > 0 and self.baseline_pa is not None:
            self._recent.append((t, pressure_altitude(pressure, self.baseline_pa, c.mean_temp_k)))
        while self._recent and self._recent[0][0] < t - c.median_window_s:
            self._recent.popleft()
        if not self._recent:
            self.altitude = self.vertical_speed = float('nan')
            self._history.clear()
            return
        ordered = sorted(a for _, a in self._recent)
        self.altitude = ordered[len(ordered)//2]
        self._history.append((t, self.altitude))
        while self._history[0][0] < t - c.speed_window_s:
            self._history.popleft()
        t0, a0 = self._history[0]
        span = t - t0
        self.vertical_speed = (self.altitude - a0)/span if span >= 0.8*c.speed_window_s else float('nan')

    def _go(self, t, state, reason):
        self.state = state
        self.events.append((t, state.name, reason))

    def update(self, t, accel, pressure_pa):
        c = self.cfg
        if self.state == State.PAD and _valid(pressure_pa) and pressure_pa > 0:
            self._track_baseline(t, pressure_pa)
        self._update_altitude(t, pressure_pa)
        alt, speed = self.altitude, self.vertical_speed
        has_alt, has_acc = _valid(alt), _valid(accel)

        if self.state == State.PAD:
            by_accel = self._launch_accel(t, has_acc and accel > c.launch_accel)
            by_alt = self._launch_alt(t, has_alt and alt > c.launch_alt_m
                                      and _valid(speed) and speed > c.launch_climb_rate)
            if by_accel or by_alt:
                if self.baseline_pa is None and self._pending:
                    self.baseline_pa = self._pending[0][1]
                self._pending.clear()
                self.t_launch = t
                self.max_altitude = alt if has_alt else float('nan')
                self._go(t, State.ASCENT, 'acceleration' if by_accel else 'altitude')

        elif self.state == State.ASCENT:
            if has_alt and not alt <= self.max_altitude:
                self.max_altitude = alt
            since = t - self.t_launch
            armed = since >= c.eject_lockout_s
            falling = self._apogee(t, armed and has_alt and alt < self.max_altitude - c.apogee_drop_m)
            if armed and has_acc and accel > c.eject_accel:
                reason = 'acceleration'
            elif falling:
                reason = 'altitude'
            elif since >= c.ascent_timeout_s:
                reason = 'timeout'
            else:
                reason = None
            if reason:
                self.t_descent = t
                self._go(t, State.DESCENT, reason)

        elif self.state == State.DESCENT:
            since = t - self.t_descent
            low = not has_alt or alt < c.touchdown_alt_m
            resting = self._rest(t, _valid(speed) and abs(speed) < c.rest_speed)
            if since >= c.touchdown_lockout_s and low and has_acc and accel > c.touchdown_accel:
                reason = 'acceleration'
            elif since >= c.touchdown_lockout_s and resting:
                reason = 'altitude'
            elif since >= c.descent_timeout_s:
                reason = 'timeout'
            else:
                reason = None
            if reason:
                self.t_landed = t
                self._go(t, State.LANDED, reason)

        return self.state
