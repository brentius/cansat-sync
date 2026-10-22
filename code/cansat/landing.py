"""GPL-3.0-only. Live landing prediction (x, y, t) with a covariance (proposal section 13, App. J.2).

Flight software for the Raspberry Pi Zero 2 W. One LandingPredictor per
velocity source (TRN, GPS, dead reckoning) runs beside the navigation filter;
its result fills the pred_* fields of the 40-byte packet (cansat/telemetry.py),
which the STM32 transmits.

What it is given, and where that comes from on the flight stack:

    t          seconds on the STM32 timer that stamps the camera trigger, IMU and PPS
    position   east, north, up from the pad (m): the navigation filter, or GNSS
    velocity   east, north (m/s): the navigation filter, or UBX-NAV-PVT velE/velN
               forwarded by the STM32 over the Pi UART; None for dead reckoning
    variances  the filter's own, or the receiver's sAcc/hAcc squared

Cost is fixed by design. Propagating the wind state to the ground is the
expensive part (hundreds of small matrix exponentials), so it is done once, on
the pad, into a DriftTable of 7 numbers per 2 m of height. In flight an update
is one WindFilter step (a few 12 x 12 matrix exponentials, capped by
max_substeps) and a table interpolation.

It is written not to stop. update() does not raise and does not let one bad
sample damage the filter: input that is malformed, not finite or out of range
is refused and counted; a velocity far outside what the filter expects is
gated (only when it misses by metres per second, so that swing is not mistaken
for a fault); a jump in the clock is believed only when a second sample confirms it,
and then the wind filter restarts from its prior; and if anything still goes
wrong the previous state is restored. The counters ignored, gated, resets and
faults say how often each happened.

Needs NumPy only (present on Raspberry Pi OS; no feature newer than 1.24 is
used). It does not run on the STM32F405. If a GPS-driven prediction has to
survive the loss of the Pi, the per-update arithmetic here (14-state filter
step, table lookup) is small enough for a C port in single precision; that
port does not exist.

Design-stage code checked only in simulation; not timed on the Pi (run
code/tests/bench_onboard.py there).
"""
import math
from dataclasses import dataclass, replace

import numpy as np

from .telemetry import FLAG_PRED_VALID, cov_to_ellipse
from .windprofile import N_AXIS, U, WindFilter, WindModel, axis_step, drift_from_row, measurement_rows

FLIGHT_MAX_SUBSTEPS = 8
MAX_GAP_S = 5.0             # a longer step between samples is a clock fault or a restart
POSITION_LIMIT_M = 1.0e6    # larger coordinates are a fault, not a position
VELOCITY_LIMIT = 60.0       # m/s, horizontal; beyond any wind a launch would go ahead in
GATE = 50.0                 # chi-square (2 degrees of freedom) beyond which a velocity is doubted
GATE_FLOOR = 4.0            # m/s: and only if it also misses by this much. The gate is for sensor
                            # glitches; swing the model does not contain misses by a metre per second
                            # or two and must not be shut out (in simulation it was, one sample in ten)
GATE_RESTART = 5            # consecutive gated velocities before the filter is doubted instead


class DriftTable:
    """Drift to the ground per unit of each wind-state component, tabulated against height.

    For one horizontal axis, drift = row(z) @ state + noise of variance q(z),
    where row and q come from propagating the model from z to the ground at
    `descent_speed`. Built once from the ground up; lookups interpolate.
    """

    def __init__(self, model, descent_speed, top=450.0, step=2.0):
        if descent_speed <= 0 or top <= 0 or step <= 0:
            raise ValueError('positive descent speed, top and step required')
        self.descent_speed = float(descent_speed)
        self.heights = np.arange(0.0, top + step, step)
        n = N_AXIS + 1
        self.rows = np.zeros((len(self.heights), N_AXIS))
        self.noise = np.zeros(len(self.heights))
        row = np.zeros(n)
        row[N_AXIS] = 1.0                      # at the ground the remaining drift is the integral so far
        q = 0.0
        for i in range(1, len(self.heights)):
            phi, step_q = axis_step(model, self.heights[i], self.heights[i - 1], step/descent_speed, drift=True)
            q = float(row @ step_q @ row) + q
            row = row @ phi
            self.rows[i] = row[:N_AXIS]
            self.noise[i] = q

    def lookup(self, height):
        """Drift row (N_AXIS,), process-noise variance and time to touchdown at `height`.

        Above the top of the table the can is taken to keep its present
        horizontal velocity until it reaches the top.
        """
        top = float(self.heights[-1])
        z = min(max(float(height), 0.0), top)
        x = z/(self.heights[1] - self.heights[0])
        i = min(int(x), len(self.heights) - 2)
        f = x - i
        row = (1 - f)*self.rows[i] + f*self.rows[i + 1]
        above = max(float(height) - top, 0.0)/self.descent_speed
        if above:
            row = row.copy()
            row[U] += above
        return row, (1 - f)*self.noise[i] + f*self.noise[i + 1], z/self.descent_speed + above


@dataclass
class Prediction:
    t: float                    # s, when it was made
    east: float                 # m, predicted touchdown point
    north: float
    time_to_go: float           # s
    cov: np.ndarray             # (2, 2) m^2
    drift: np.ndarray           # (2,) m, from the present position

    def ellipse(self):
        """1-sigma semi-axes (m) and major-axis angle from east (rad)."""
        return cov_to_ellipse(float(self.cov[0, 0]), float(self.cov[0, 1]), float(self.cov[1, 1]))

    def fill(self, packet):
        """Write the prediction into a telemetry.Telemetry packet and mark it valid."""
        major, minor, angle = self.ellipse()
        packet.pred_e, packet.pred_n, packet.time_to_go_s = self.east, self.north, self.time_to_go
        packet.pred_major_m, packet.pred_minor_m, packet.pred_angle_rad = major, minor, angle
        packet.flags |= FLAG_PRED_VALID
        return packet


def _vector(value, sizes):
    """value as a finite float vector whose length is in sizes, or None."""
    try:
        out = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if len(out) in sizes and bool(np.all(np.isfinite(out))) else None


class LandingPredictor:
    """Wind filter plus drift table for one velocity source."""

    def __init__(self, model=None, descent_speed=10.5, anemometer=None, top=450.0, step=2.0,
                 height_sd=1.5, descent_speed_sd=0.03, rate_window_s=3.0, table=None):
        """height_sd (m) and descent_speed_sd (fraction) set the touchdown-time uncertainty.

        The table is built for `descent_speed`; in flight the drift is scaled by
        that speed over the measured one. That is an approximation: the wind is
        a function of height but the lag is a time, so at 8 m/s against a
        10.5 m/s table the scaled drift is about 1 % short from 300 m and about
        a tenth short from 40 m. Build the table at the descent rate measured in
        drop tests. Predictors with the same model can share one `table`.
        """
        model = model or WindModel()
        self.model = replace(model, max_substeps=model.max_substeps or FLIGHT_MAX_SUBSTEPS)
        self.table = table or DriftTable(self.model, descent_speed, top, step)
        self._anemometer = anemometer
        self.filter = WindFilter(self.model, anemometer)
        self.height_sd = height_sd
        self.descent_speed_sd = descent_speed_sd
        self.rate_window = rate_window_s
        self.descent_rate = float(self.table.descent_speed)
        self._rate_ref = None               # (t, height) the measured descent rate is taken from
        self._pending = None                # time of an out-of-sequence sample awaiting confirmation
        self._refused = 0                   # consecutive gated velocities
        self.ignored = 0                    # inputs refused as malformed, late, not finite or out of range
        self.gated = 0                      # velocities refused as inconsistent with the filter
        self.resets = 0                     # times the wind filter was restarted from its prior
        self.faults = 0                     # updates abandoned by the last-resort guard
        self.last = None

    def _restart(self):
        self.filter = WindFilter(self.model, self._anemometer)
        self._rate_ref = None
        self._refused = 0
        self.resets += 1

    def _measure_rate(self, t, height):
        if self._rate_ref is None:
            self._rate_ref = (t, height)
            return
        t0, h0 = self._rate_ref
        if t - t0 >= self.rate_window:
            rate = (h0 - height)/(t - t0)
            nominal = self.table.descent_speed
            if 0.5*nominal <= rate <= 2.0*nominal:      # a wild value is a sensor fault, not a canopy
                self.descent_rate = rate
            self._rate_ref = (t, height)

    def update(self, t, position, velocity=None, velocity_var=None, position_cov=None, error_gain=(1.0, 1.0)):
        """One epoch. Returns a Prediction, or None when the sample was unusable. Never raises.

        position (3,) east, north, up; velocity (2,) or None; velocity_var
        scalar or (2,); position_cov (2, 2) horizontal, or None for exact;
        error_gain scalar or (2,), the size of the correlated velocity error.
        """
        f = self.filter
        saved = (f, f.mean.copy(), f.cov.copy(), f.t, f.z, self.descent_rate, self._rate_ref, self._refused)
        try:
            out = self._update(t, position, velocity, velocity_var, position_cov, error_gain)
            if out is not None and not (np.all(np.isfinite(out.cov)) and np.all(np.isfinite(out.drift))
                                        and math.isfinite(out.east + out.north + out.time_to_go)
                                        and np.all(np.isfinite(self.filter.mean)) and np.all(np.isfinite(self.filter.cov))):
                raise ArithmeticError('non-finite prediction')
        except Exception:                               # noqa: BLE001 - flight loop: nothing may escape
            self.faults += 1
            self.filter, f.mean, f.cov, f.t, f.z, self.descent_rate, self._rate_ref, self._refused = saved
            return None
        if out is not None:
            self.last = out
        return out

    def _update(self, t, position, velocity, velocity_var, position_cov, error_gain):
        try:
            t = float(t)
        except (TypeError, ValueError, OverflowError):
            t = math.nan
        position = _vector(position, (3,))
        if not math.isfinite(t) or position is None or float(np.abs(position).max()) > POSITION_LIMIT_M:
            self.ignored += 1
            return None
        if self.filter.t is not None:
            dt = t - self.filter.t
            if 0 < dt <= MAX_GAP_S:
                self._pending = None
            elif -MAX_GAP_S < dt <= 0:                  # late or repeated
                self.ignored += 1
                return None
            elif self._pending is not None and 0 < t - self._pending <= MAX_GAP_S:
                self._pending = None                    # a second sample agrees: the clock really moved
                self._restart()
            else:
                self._pending = t
                self.ignored += 1
                return None
        height = max(float(position[2]), 0.0)
        self.filter.advance(t, height)

        if velocity is not None:
            v = _vector(velocity, (2,))
            var = _vector(velocity_var, (1, 2))
            gain = _vector(error_gain, (1, 2))
            if (v is None or var is None or gain is None or float(np.abs(v).max()) > VELOCITY_LIMIT
                    or not np.all(var > 0) or not np.all(gain >= 0)):
                self.ignored += 1
            else:
                var, gain = np.broadcast_to(var, (2,)), np.broadcast_to(gain, (2,))
                h = measurement_rows(self.model, height, True, None, gain)
                nu = v - h @ self.filter.mean
                s = h @ self.filter.cov @ h.T + np.diag(var)
                if float(nu @ np.linalg.solve(s, nu)) <= GATE or float(np.abs(nu).max()) <= GATE_FLOOR:
                    self.filter.update(v, var, gain)
                    self._refused = 0
                else:
                    self.gated += 1
                    self._refused += 1
                    if self._refused >= GATE_RESTART:   # the data keep disagreeing: doubt the filter
                        self._restart()
                        self.filter.advance(t, height)
                        self.filter.update(v, var, gain)

        self._measure_rate(t, height)
        row, noise, _ = self.table.lookup(height)
        scale = self.table.descent_speed/self.descent_rate
        drift, cov = drift_from_row(row*scale, noise*scale*scale, self.filter.mean, self.filter.cov)
        time_to_go = height/self.descent_rate
        if position_cov is not None:
            extra = _vector(position_cov, (4,))
            extra = None if extra is None or np.shape(position_cov) != (2, 2) else extra.reshape(2, 2)
            if extra is None or float(np.linalg.eigvalsh((extra + extra.T)/2)[0]) < 0:
                self.ignored += 1
            else:
                cov = cov + (extra + extra.T)/2
        if time_to_go > 0:
            # An error in the touchdown time moves the landing point along the mean drift velocity.
            time_var = (self.height_sd/self.descent_rate)**2 + (self.descent_speed_sd*time_to_go)**2
            mean_speed = drift/time_to_go
            cov = cov + time_var*np.outer(mean_speed, mean_speed)
        vals, vecs = np.linalg.eigh((cov + cov.T)/2)
        cov = (vecs*np.maximum(vals, 0.0)) @ vecs.T
        return Prediction(t, float(position[0] + drift[0]), float(position[1] + drift[1]), time_to_go, cov, drift)
