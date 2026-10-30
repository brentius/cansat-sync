"""GPL-3.0-only. Single-worker Pi navigation orchestration (PI-NAV contract).

Connects link.ImuDifferencer, coarse.Leveller/Acquirer, firstfix.initial_covariance,
eskf.Eskf, fine.FineMatcher and landing.LandingPredictor. Sensor timestamps are
already unwrapped STM32 seconds. Delayed images rewind to exposure time and
replay ordered, split IMU increments and barometer updates to the live time.
History is bounded by both seconds and item count. GNSS is a separate logged
comparison/predictor and can never initialise or correct terrain navigation.
Policy defaults are [assumed], verified on synthetic host data only.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np

from . import link
from .atmosphere import pressure_altitude
from .coarse import Acquirer, Leveller
from .eskf import Eskf, VEL
from .firstfix import initial_covariance
from .flight_state import State
from .geo import enu
from .landing import LandingPredictor
from .rectify import Rectifier
from .telemetry import FLAG_CAMERA_OK, FLAG_PRED_VALID, FLAG_TRN_VALID


@dataclass
class NavigatorConfig:
    min_height_m: float = 80.
    max_height_m: float = 450.
    history_seconds: float = 8.
    history_max_events: int = 1500  # cap EACH sensor deque, independent of time
    max_imu_gap_s: float = .5
    max_baro_age_s: float = 1.
    mean_temperature_k: float = 283.15
    baro_sigma_m: float = 1.5
    coast_after_s: float = .6
    lost_after_s: float = 3.
    low_altitude_coast_s: float = 12.  # [assumed] bounded IMU/baro continuation below map floor
    max_protection_m: float = 30.
    max_nav_age_s: float = .6
    prediction_interval_s: float = .2
    min_landmarks: int = 4
    prior_radius_m: float = 300.
    velocity_sigma_mps: float = 3.
    accel_bias_sigma: float = .1
    gyro_bias_sigma: float = .005
    baro_bias_sigma_m: float = 3.
    rng_seed: int = 20261026
    log_inertial_comparison: bool = True

    def __post_init__(self):
        positive = ('history_seconds', 'max_imu_gap_s', 'max_baro_age_s', 'mean_temperature_k',
                    'baro_sigma_m', 'coast_after_s', 'lost_after_s', 'low_altitude_coast_s', 'max_protection_m',
                    'max_nav_age_s', 'prediction_interval_s', 'prior_radius_m', 'velocity_sigma_mps',
                    'accel_bias_sigma', 'gyro_bias_sigma', 'baro_bias_sigma_m')
        if (any(not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0 for key in positive)
                or not math.isfinite(self.min_height_m + self.max_height_m)
                or not 0 <= self.min_height_m < self.max_height_m
                or self.coast_after_s >= self.lost_after_s
                or not isinstance(self.history_max_events, int) or not 2 <= self.history_max_events <= 20000
                or not isinstance(self.min_landmarks, int) or not 4 <= self.min_landmarks <= 60):
            raise ValueError('invalid navigation limits')


@dataclass
class _Span:
    start: float
    end: float
    increment: link.ImuIncrement
    level_start: Leveller
    mean_arrays: tuple = None


def _ellipse(cov):
    vals, vecs = np.linalg.eigh((cov + cov.T)/2)
    return (math.sqrt(max(float(vals[1]), 0.)), math.sqrt(max(float(vals[0]), 0.)),
            math.atan2(float(vecs[1, 1]), float(vecs[0, 1])))


class Navigator:
    """Not thread-safe. The shell delivers messages and frames on one worker."""

    MISSED_BRIDGE_S = .03     # [assumed] protocol section 11 IMU bridging bound

    def __init__(self, config, site, log=None):
        self.config, self.site, self.log = config, site, log
        self._rectifier = Rectifier(site.camera, site.distortion)
        self._rng = np.random.default_rng(config.rng_seed)
        self.restart()

    def _record(self, kind, **values):
        if self.log is not None:
            self.log(dict(kind=kind, generation=self._generation, epoch=self._epoch, **values))

    @staticmethod
    def _horizontal_state(f):
        """Joint ESKF marginal; cross terms are needed for position/velocity NEES.

        Covariance is of navigation error in the site's pad ENU frame. It is
        not GNSS measurement covariance or the uncertainty of the difference.
        """
        indices = [0, 1, 3, 4]
        return dict(horizontal_state=[*f.p[:2].tolist(), *f.v[:2].tolist()],
                    horizontal_covariance=f.cov[np.ix_(indices, indices)].tolist(),
                    horizontal_order=['p_e', 'p_n', 'v_e', 'v_n'],
                    horizontal_units=['m', 'm', 'm/s', 'm/s'],
                    covariance_units=[['m^2', 'm^2', 'm^2/s', 'm^2/s'],
                                      ['m^2', 'm^2', 'm^2/s', 'm^2/s'],
                                      ['m^2/s', 'm^2/s', 'm^2/s^2', 'm^2/s^2'],
                                      ['m^2/s', 'm^2/s', 'm^2/s^2', 'm^2/s^2']],
                    covariance_source='trn_eskf', coordinate_frame='site_pad_enu',
                    time_basis='unwrapped STM32 seconds')

    def _record_navigation(self, **context):
        if self.filter is not None:
            self._record('nav', t=self._filter_t, mode=self.mode, source='trn',
                         position=self.filter.p.tolist(), velocity=self.filter.v.tolist(),
                         protection_m=self.filter.protection_level(),
                         **self._horizontal_state(self.filter), **context)

    def _new_predictor(self):
        return LandingPredictor(model=self.site.wind_model, table=self.site.drift_table)

    def _new_acquirer(self):
        return Acquirer(self.site.coarse_map, prior_radius=self.config.prior_radius_m)

    def restart(self):
        """STM32 reset: discard all clock/epoch-dependent state, retain site."""
        self._generation = getattr(self, '_generation', -1) + 1
        self.filter = self._anchor = None
        self._anchor_t = self._filter_t = self._imu_t = None
        self._imus, self._baros = deque(), deque()
        self._differencer = link.ImuDifferencer(max_gap_s=self.config.max_imu_gap_s)
        self._leveller = Leveller()
        self.acquirer = self._new_acquirer()
        self.trn_predictor, self.gnss_predictor = self._new_predictor(), self._new_predictor()
        self.inertial_filter = None
        self.inertial_predictor = self._new_predictor()
        self._inertial_t = self._last_inertial_prediction_t = None
        self._prediction = None
        self._last_prediction_t = None
        self._last_visual_t = self._last_frame_t = None
        self._ck, self._ck_anchor = [], None
        self._last_frame = self._landmarks = 0
        self._state, self._epoch = State.PAD, None
        self._status_t = None
        self._baseline = None
        self._output_origin = None
        self._output_offset = np.zeros(3)
        self._had_solution = False
        self._mode = 'waiting'
        self._record('event', event='restart')

    @property
    def mode(self):
        return self._mode

    @property
    def pi_flags(self):
        return link.PIFLAG_MAP | (link.PIFLAG_NAV if self.filter is not None and self._mode != 'done' else 0)

    def _lose(self, reason):
        self.filter = self._anchor = None
        self._anchor_t = self._filter_t = None
        self._prediction = None
        self._last_prediction_t = None
        self.trn_predictor = self._new_predictor()
        self.acquirer = self._new_acquirer()
        self._mode = 'done' if self._state == State.LANDED else ('lost' if self._had_solution else 'waiting')
        self._record('event', event='lost', reason=reason)

    def _break_imu(self, reason):
        self._lose(reason)
        self.inertial_filter = None
        self.inertial_predictor = self._new_predictor()
        self._inertial_t = self._last_inertial_prediction_t = None
        self._imus.clear()
        self._baros.clear()
        self._leveller.reset()
        self._differencer = link.ImuDifferencer(max_gap_s=self.config.max_imu_gap_s)
        self._imu_t = None

    def on_status(self, msg, t):
        if not math.isfinite(t):
            return
        if self._epoch is not None and msg.epoch != self._epoch:
            self.restart()
        if self._status_t is not None and t < self._status_t:
            return
        self._status_t, self._epoch = t, msg.epoch
        if msg.state in tuple(State):
            self._state = State(msg.state)
        valid_baseline = bool(msg.health & link.HEALTH_BASELINE) and math.isfinite(msg.baseline_pa) and msg.baseline_pa > 0
        baseline = float(msg.baseline_pa) if valid_baseline else None
        if baseline != self._baseline:
            # A changed pressure reference cannot be applied to old frame heights.
            if self.filter is not None:
                self._lose('baseline changed')
            self._baros.clear()
            self.acquirer = self._new_acquirer()
        self._baseline = baseline
        origin = (msg.origin_lat_deg, msg.origin_lon_deg, msg.origin_h_msl_m)
        if (msg.health & link.HEALTH_ORIGIN and np.isfinite(origin).all()
                and abs(origin[0]) <= 90 and abs(origin[1]) <= 180):
            self._output_origin = origin
            self._output_offset = np.array(enu(*self.site.pad_origin, *origin))
        else:
            self._output_origin = None
        if self._state == State.LANDED:
            self._mode = 'done'
            self._prediction = None

    def on_imu(self, msg, t):
        if not math.isfinite(t):
            return
        if self._epoch is not None and msg.epoch != self._epoch:
            self.restart()
        self._epoch = msg.epoch
        if msg.flags & link.IMU_RESTART:
            self._break_imu('integral restart')
        # Saturation/faults leave the integrals wrong by an unknown amount. A missed
        # sample does not: the STM32 integrates across it, costing time resolution
        # only (protocol section 6), so short flagged intervals stay in the thread.
        bad_flags = (link.IMU_GYRO_SATURATED | link.IMU_ACCEL_SATURATED | link.IMU_GYRO_FAULT
                     | link.IMU_ACCEL_FAULT)
        if msg.flags & bad_flags:
            self._break_imu('IMU quality flags')
            return
        prior_t = self._imu_t
        inc = self._differencer.feed(msg)
        if inc is None:
            if prior_t is not None:
                self._break_imu('IMU discontinuity')
                # A valid new baseline may start the next interval after a gap.
                self._differencer.feed(msg)
            self._imu_t = t
            return
        if prior_t is None or abs((t - prior_t) - inc.dt) > 2e-6:
            self._break_imu('unwrapped time mismatch')
            return
        if msg.flags & (link.IMU_GYRO_MISSED | link.IMU_ACCEL_MISSED) and inc.dt > self.MISSED_BRIDGE_S:
            # The flags carry no gap length: a long flagged interval may hide an
            # internal stall bridged by one trapezoid. Restart from this snapshot.
            self._break_imu('IMU missed samples over a long interval')
            self._differencer.feed(msg)
            self._imu_t = t
            return
        accel, gyro = inc.mean()
        span = _Span(prior_t, t, inc, self._leveller.clone(), (np.asarray(accel), np.asarray(gyro)))
        self._imus.append(span)
        self._leveller.update(inc.dt, accel, gyro)
        self._imu_t = t
        if self.inertial_filter is not None and self._state != State.LANDED:
            # Common first-fix initial conditions, then IMU only: neither later
            # images, pressure corrections nor GNSS enter this comparison.
            self.inertial_filter.propagate(inc.dt, *span.mean_arrays)
            self._inertial_t = t
            self._predict_inertial()
        if self.filter is not None:
            self._advance(self.filter, self._filter_t, t)
            self._filter_t = t
            self._check_lock()
            self._predict()
        self._prune()

    def on_baro(self, msg, t):
        if (self._baseline is None or not math.isfinite(t) or not math.isfinite(msg.pressure_pa)
                or not 1000. < msg.pressure_pa < 120000. or not msg.flags & 1):
            return
        if self._baros and t <= self._baros[-1][0]:
            self._record('event', event='baro_rejected', reason='out of order', t=t)
            return
        height = pressure_altitude(msg.pressure_pa, self._baseline, self.config.mean_temperature_k)
        self._baros.append((t, height))
        if self.filter is not None and self._anchor_t < t <= self._filter_t:
            # Replaying anchor -> newest redoes, operation for operation, what an earlier replay did up
            # to its last pressure update: resume from the nearest saved state instead.
            if self._ck_anchor is not self._anchor:
                self._ck, self._ck_anchor = [], self._anchor
            self.filter, start = self._resume(self._filter_t)
            self._advance(self.filter, start, self._filter_t, self._ck)
            del self._ck[:-self.CHECKPOINTS]
            self._record_navigation(update_source='baro_replay', observation_t=t)
        self._prune()

    def _height(self, t):
        for bt, height in reversed(self._baros):
            if bt <= t + 1e-9:
                return height if t - bt <= self.config.max_baro_age_s else None
        return None

    def wants_frames(self):
        if self._state != State.DESCENT or self._imu_t is None:
            return False
        height = self._height(self._imu_t)
        return height is not None and self.config.min_height_m <= height <= self.config.max_height_m

    CHECKPOINTS = 12          # saved replay states (about one per pressure epoch)

    def _resume(self, t):
        """(copy of the replayed anchor state, its time) from the newest saved state at or before t."""
        if self._anchor is not None and self._ck_anchor is self._anchor:
            for ct, state in reversed(self._ck):
                if self._anchor_t < ct <= t:
                    return deepcopy(state), ct
        return deepcopy(self._anchor), self._anchor_t

    def _advance(self, nav, start, end, checkpoint=None):
        """Replay (start,end], splitting an increment at every pressure epoch.

        A list `checkpoint` receives (time, copy of nav) after each of the last few pressure
        updates, for those that fell inside their increment."""
        # Both histories are time ordered: walk back from the newest entry and
        # stop at the first one that cannot contribute, instead of scanning all.
        due = []
        for t, h in reversed(self._baros):
            if t <= start:
                break
            if t <= end + 1e-9:
                due.append((t, h))
        due.reverse()
        keep = {bt for bt, _ in due[-self.CHECKPOINTS:]} if checkpoint is not None else ()
        baros = iter(due)
        baro = next(baros, None)
        cursor = start
        tail = []
        for span in reversed(self._imus):
            if span.end <= start + 1e-12:
                break
            tail.append(span)
        tail.reverse()
        for span in tail:
            left, right = max(start, span.start), min(end, span.end)
            if right <= left + 1e-12:
                continue
            if left - cursor > 2e-6:
                raise ValueError('inertial history gap')
            while baro is not None and baro[0] <= right + 1e-9:
                bt, bh = baro
                self._part(nav, span, left, min(bt, right))
                nav.update_baro(bh, self.config.baro_sigma_m)
                left = min(bt, right)
                if bt in keep and bt <= right:
                    checkpoint.append((bt, deepcopy(nav)))
                baro = next(baros, None)
            self._part(nav, span, left, right)
            cursor = right
            if cursor >= end - 1e-9:
                break
        if end - cursor > 2e-6:
            raise ValueError('inertial history does not reach epoch')

    @staticmethod
    def _part(nav, span, left, right):
        if right <= left + 1e-12:
            return
        dt = span.end - span.start
        inc = span.increment
        whole = True
        if left > span.start + 1e-12:
            _, inc = inc.split((left - span.start)/dt)
            whole = False
        if right < span.end - 1e-12:
            inc, _ = inc.split((right - left)/(span.end - left))
            whole = False
        if whole:
            if span.mean_arrays is None:
                accel, gyro = inc.mean()
                span.mean_arrays = (np.asarray(accel), np.asarray(gyro))
            accel, gyro = span.mean_arrays
        else:
            accel, gyro = inc.mean()
            accel, gyro = np.asarray(accel), np.asarray(gyro)
        nav.propagate(inc.dt, accel, gyro)

    def _prune(self):
        if self._imu_t is None:
            cutoff = self._baros[-1][0] - self.config.history_seconds if self._baros else 0.
            while len(self._baros) > 1 and (len(self._baros) > self.config.history_max_events
                                           or self._baros[1][0] < cutoff):
                self._baros.popleft()
            return
        cutoff = self._imu_t - self.config.history_seconds
        # If pressure arrives much faster than IMU, preserve its accumulated
        # correction in the anchor before discarding any required observation.
        if len(self._baros) > self.config.history_max_events:
            edge = self._baros[len(self._baros) - self.config.history_max_events - 1][0]
            if self._anchor is not None and self._anchor_t < edge:
                if edge > self._filter_t:
                    self._lose('pressure backlog exceeds inertial horizon')
                else:
                    self._advance(self._anchor, self._anchor_t, edge)
                    self._anchor_t = edge
            cutoff = max(cutoff, edge)
        remove = 0
        for span in self._imus:
            if span.end < cutoff or len(self._imus) - remove > self.config.history_max_events:
                remove += 1
            else:
                break
        if remove:
            edge = self._imus[remove - 1].end
            if self._anchor is not None and self._anchor_t < edge:
                self._advance(self._anchor, self._anchor_t, edge)
                self._anchor_t = edge
            for _ in range(remove):
                self._imus.popleft()
            cutoff = max(cutoff, edge)
        while len(self._baros) > 1 and (self._baros[1][0] < cutoff
                or len(self._baros) > self.config.history_max_events):
            self._baros.popleft()

    def _check_lock(self):
        if self.filter is None or self._mode == 'done':
            return
        age = self._filter_t - self._last_visual_t
        height = self._height(self._filter_t)
        below_map = height is not None and 0 <= height < self.config.min_height_m
        age_limit = self.config.low_altitude_coast_s if below_map else self.config.lost_after_s
        if (not np.isfinite(self.filter.p).all() or not np.isfinite(self.filter.cov).all()
                or age > age_limit
                or self.filter.protection_level() > self.config.max_protection_m):
            self._lose('visual age/protection limit')
        else:
            self._mode = 'coasting' if below_map or age > self.config.coast_after_s else 'tracking'

    def _predict(self):
        if self.filter is None or self._mode == 'done':
            return
        if self._last_prediction_t is not None and self._filter_t - self._last_prediction_t < self.config.prediction_interval_s:
            return
        f = self.filter
        # Retain regular state/covariance even without images or a successful
        # landing prediction, including bounded low-altitude coasting.
        self._record_navigation(update_source='inertial_cadence')
        self._prediction = self.trn_predictor.update(self._filter_t, f.p, f.v[:2],
            np.maximum(np.diag(f.cov[VEL, VEL])[:2], 1e-6), position_cov=f.cov[:2, :2])
        self._last_prediction_t = self._filter_t
        if self._prediction is not None:
            self._record('pred', t=self._filter_t, source='trn', east=self._prediction.east,
                         north=self._prediction.north, time_to_go=self._prediction.time_to_go,
                         covariance=self._prediction.cov.tolist())
            self._record_wind(self.trn_predictor, self._filter_t, 'trn')

    def _record_wind(self, predictor, t, source):
        mean, cov = predictor.filter.wind()
        self._record('wind', t=t, source=source, height=predictor.filter.z,
                     velocity=mean.tolist(), covariance=cov.tolist(),
                     model='canopy drag prior; absolute wind requires calibration')

    def _predict_inertial(self):
        if (self._last_inertial_prediction_t is not None
                and self._inertial_t - self._last_inertial_prediction_t < self.config.prediction_interval_s):
            return
        f = self.inertial_filter
        velocity_variance = np.maximum(np.diag(f.cov[VEL, VEL])[:2], 1e-6)
        # The predictor consumes per-axis variances (its public API). This
        # one-way output never updates the IMU-only ESKF or uses later sensors.
        pred = self.inertial_predictor.update(self._inertial_t, f.p, f.v[:2], velocity_variance,
                                              position_cov=f.cov[:2, :2])
        self._last_inertial_prediction_t = self._inertial_t
        if pred is not None:
            self._record('inertial_pred', t=self._inertial_t, source='inertial_only',
                         initialisation='first confirmed TRN fix; no later measurements',
                         position=f.p.tolist(), velocity=f.v.tolist(),
                         velocity_variance=velocity_variance.tolist(),
                         position_covariance=f.cov[:2, :2].tolist(),
                         east=pred.east, north=pred.north,
                         time_to_go=pred.time_to_go, covariance=pred.cov.tolist())

    def on_frame(self, image, t, frame):
        reason = None
        image = np.asarray(image)
        if not math.isfinite(t) or not self._imus or t < self._imus[0].start or t > self._imu_t + 1e-9:
            reason = 'outside inertial history'
        elif self._last_frame_t is not None and t <= self._last_frame_t:
            reason = 'out of order frame'
        elif self._anchor_t is not None and t < self._anchor_t:
            reason = 'before retained filter anchor'
        elif self._state != State.DESCENT:
            reason = 'flight state'
        elif image.shape != (self.site.camera.height, self.site.camera.width) or image.dtype not in (np.uint8, np.uint16):
            reason = 'image format'
        height = self._height(t) if math.isfinite(t) else None
        if reason is None and (height is None or not self.config.min_height_m <= height <= self.config.max_height_m):
            reason = 'height unavailable/outside window'
        if reason is not None:
            self._record('event', event='frame_rejected', t=t, frame=frame, reason=reason)
            return
        image = self._rectifier.apply(image)
        self._last_frame_t = t
        span = next(s for s in self._imus if s.start <= t <= s.end + 1e-9)
        level = span.level_start.clone()
        fraction = min(1., max(0., (t - span.start)/(span.end - span.start)))
        if fraction > 0:
            inc, _ = span.increment.split(fraction)
            accel, gyro = inc.mean()
            level.update(inc.dt, accel, gyro)
        r0 = level.attitude()
        if r0 is None:
            self._record('event', event='frame_rejected', t=t, frame=frame, reason='levelling unavailable')
            return
        if self.filter is None:
            self._mode = 'acquiring'
            state, result, velocity = self.acquirer.update(t, image, self.site.camera, r0, height, rng=self._rng)
            self._record('acq', t=t, frame=frame, state=state, reason=result.reason, matches=result.matches)
            if state != 'confirmed':
                return
            fix = result.fix
            cfg = self.config
            cov = initial_covariance(fix, cfg.velocity_sigma_mps, cfg.accel_bias_sigma,
                                     cfg.gyro_bias_sigma, cfg.baro_bias_sigma_m)
            candidate = Eskf(fix.p, velocity, fix.r, cov, baro_bias=height - fix.p[2])
            if self.config.log_inertial_comparison and self.inertial_filter is None:
                self.inertial_filter = deepcopy(candidate)
                for inertial_span in self._imus:
                    self._part(self.inertial_filter, inertial_span,
                               max(t, inertial_span.start), min(self._imu_t, inertial_span.end))
                self._inertial_t = self._imu_t
                self._predict_inertial()
            accepted = result.matches
        else:
            candidate, start = self._resume(t)
            self._advance(candidate, start, t)
            matches = self.site.fine_matcher.match_filter(image, self.site.camera, candidate)
            accepted = 0
            if len(matches.points) >= self.config.min_landmarks:
                info = candidate.update_landmarks(self.site.camera, matches.points, matches.pixels,
                    matches.pixel_var, map_var=matches.map_var, iterations=3, robust=True, recover=False)
                accepted = int(np.count_nonzero(info['accepted']))
            self._record('fix', t=t, frame=frame, matches=len(matches.points), accepted=accepted,
                         reason=matches.reason, rejections=matches.rejections)
            if accepted < self.config.min_landmarks:
                self._check_lock()
                return
        self._anchor, self._anchor_t = deepcopy(candidate), t
        self.filter = candidate
        self._advance(self.filter, t, self._imu_t)
        self._filter_t = self._imu_t
        self._last_visual_t, self._last_frame, self._landmarks = t, frame, accepted
        self._had_solution = True
        self._check_lock()
        self._predict()
        self._record_navigation(update_source='image_replay', exposure_t=t, frame=frame)

    def on_gnss(self, msg, t):
        if (not math.isfinite(t) or self._state != State.DESCENT or not msg.usable
                or msg.link_flags & link.GNSS_STALE
                or not np.isfinite((msg.lon_deg, msg.lat_deg, msg.h_msl_m, msg.vel_d,
                                    msg.h_acc_m, msg.v_acc_m, msg.s_acc)).all()
                or min(msg.h_acc_m, msg.v_acc_m, msg.s_acc) < 0):
            return
        p = np.array(enu(msg.lat_deg, msg.lon_deg, msg.h_msl_m, *self.site.pad_origin))
        v = np.array([msg.vel_e, msg.vel_n, -msg.vel_d])
        prediction = self.gnss_predictor.update(t, p, v[:2], max(msg.s_acc**2, 1e-6),
                                               position_cov=np.eye(2)*msg.h_acc_m**2)
        self._record('gnss', t=t, source='gnss', position=p.tolist(), velocity=v.tolist())
        if prediction is not None:
            self._record('gnss_pred', t=t, source='gnss', east=prediction.east,
                         north=prediction.north, time_to_go=prediction.time_to_go,
                         covariance=prediction.cov.tolist())
            self._record_wind(self.gnss_predictor, t, 'gnss')
        # Compare at the GNSS epoch when it is retained, never at a newer TRN epoch.
        if self._anchor is not None and self._anchor_t <= t <= self._filter_t:
            comparison, start = self._resume(t)
            self._advance(comparison, start, t)
            self._record('comparison', t=t, source='trn_minus_gnss',
                         position=(comparison.p - p).tolist(), velocity=(comparison.v - v).tolist(),
                         **self._horizontal_state(comparison))

    def nav_message(self, tick):
        if not self._had_solution:
            return None
        msg = link.Nav(tick=tick, frame=self._last_frame & 0xffff)
        if self.filter is None or self._mode == 'done' or self._output_origin is None:
            return msg
        now = link.unwrap32(tick, round(self._filter_t*link.TICK_HZ))/link.TICK_HZ
        if not -1e-6 <= now - self._filter_t <= self.config.max_nav_age_s:
            return msg
        self._check_lock()
        if self.filter is None:
            return msg
        f = self.filter
        msg.tick = round(self._filter_t*link.TICK_HZ) & 0xffffffff
        msg.flags = FLAG_TRN_VALID
        if now - self._last_frame_t <= self.config.max_nav_age_s:
            msg.flags |= FLAG_CAMERA_OK
        msg.landmarks = min(self._landmarks, 254)
        msg.trn_e, msg.trn_n, msg.trn_u = f.p + self._output_offset
        msg.vel_e, msg.vel_n, msg.vel_u = f.v
        msg.trn_major_m, msg.trn_minor_m, msg.trn_angle_rad = _ellipse(f.cov[:2, :2])
        msg.protection_m = f.protection_level()
        pred = self._prediction
        if pred is not None and 0 <= now - pred.t <= self.config.max_nav_age_s:
            msg.flags |= FLAG_PRED_VALID
            msg.pred_e, msg.pred_n = pred.east + self._output_offset[0], pred.north + self._output_offset[1]
            msg.time_to_go_s = pred.time_to_go
            msg.pred_major_m, msg.pred_minor_m, msg.pred_angle_rad = pred.ellipse()
        return msg
