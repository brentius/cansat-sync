"""GPL-3.0-only. Closed-loop check of the STM32-to-Pi link on the navigation Monte Carlo.

Usage: python sim/nav/run_link_mc.py [--descents N] [--workers W] [--only a,b] [--scenario s] [--list]

Takes the descents, sensor errors and filter of sim/nav/run_nav_mc.py
(imported, not modified) and puts the serial link of code/cansat/link.py
between the simulated sensors and the filter:

    "STM32 side"  the 100 Hz IMU increments and 10 Hz barometer samples the
                  Monte Carlo already draws are integrated and encoded
                  (link.ImuAccumulator, link.Encoder), together with the
                  other traffic of the link (Gnss, Trig, Pps, Status) so the
                  byte stream has its real mix of frames
    UART          frames are corrupted with a set probability (one bit of one
                  byte, delimiter included), bytes are corrupted at a set
                  byte-error rate, or everything is cut for an interval
    "Pi side"     link.Decoder and link.ImuDifferencer; the filter is given
                  what comes out

Three ways of carrying the IMU are compared. Only the first is the protocol;
the other two exist here to be measured against it:

    cumulative  link.Imu: snapshots of running integrals; a gap is bridged by
                subtracting the snapshots either side of it
    delta       each message carries its own 10 ms increment (6 x int16, the
                rounding residual carried forward); a missing interval is
                filled by interpolating its neighbours
    redundant   delta, plus the previous interval's increment in every message

The variant `baseline` calls run_nav_mc.run_descent unchanged and `direct`
runs this file's copy of that loop without the link; the two must agree
exactly (sim/nav/test_link_mc.py), which is what makes the other rows
comparable with the stored Monte Carlo.

Everything here is SYNTHETIC and runs on a development machine. Neither side
is the real processor; no firmware exists.
"""
import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import run_nav_mc as nm  # noqa: E402
from cansat import link  # noqa: E402
from cansat.atmosphere import pressure_altitude, pressure_at  # noqa: E402
from cansat.eskf import ATT, N_ERR, VEL, Eskf, ImuNoise, exp_so3  # noqa: E402

OUT = HERE/'results'/'link'
STEP_US = round(nm.NAV_DT*link.TICK_HZ)
T0 = link.TICK_WRAP - 5*link.TICK_HZ             # the 32-bit tick wraps 5 s into every descent
BASELINE_PA, MEAN_T = 101325.0, 283.15
GAP_USE = 3                                      # steps: an image or barometer sample inside a longer IMU gap is not used
CONCEAL_ATT, CONCEAL_VEL = 1.0e-3, 5.0e-3        # rad, m/s added (1 sigma) per interpolated interval. ASSUMED
OUTAGE = (8.0, 8.5)                              # s into the descent, at about 215 m
DELTA_ID, REDUNDANT_ID = 0x60, 0x61              # experimental ids, not part of the protocol
DTHETA_LSB, DV_LSB = 2.0**-16, 2.0**-13
SLOTS = {'imu': 0, 'baro': 1, 'gnss': 2, 'trig': 3, 'status': 4, 'pps': 5}


def variant(scheme=None, loss=0.0, outage=None, ber=0.0, scenario=None, note=''):
    return {'scheme': scheme, 'loss': loss, 'outage': outage, 'ber': ber, 'scenario': scenario, 'note': note}


VARIANTS = {
    'baseline': variant(note='run_nav_mc.run_descent, unmodified'),
    'direct': variant('direct', note="this file's loop, no link: must equal baseline"),
}
for _scheme in ('cumulative', 'delta', 'redundant'):
    VARIANTS[_scheme + '_clean'] = variant(_scheme, note='through the codec, no errors: quantisation only')
    for _loss in (0.001, 0.01, 0.05):
        VARIANTS['%s_loss_%s' % (_scheme, ('%g' % (100*_loss)).replace('.', 'p'))] = variant(
            _scheme, loss=_loss, note='%g %% of frames corrupted' % (100*_loss))
    VARIANTS[_scheme + '_outage_0p5s'] = variant(_scheme, outage=OUTAGE, note='nothing received from 8.0 s to 8.5 s')
VARIANTS['cumulative_loss_5_and_outage'] = variant('cumulative', loss=0.05, outage=OUTAGE)
VARIANTS['cumulative_byte_errors_1e-3'] = variant('cumulative', ber=1e-3, note='each byte replaced with probability 0.001')
VARIANTS['cumulative_boxcar'] = variant('cumulative', scenario='imu_boxcar_100hz',
                                        note='STM32 sums 1 kHz samples with no coning or sculling term')
VARIANTS['baseline_boxcar'] = variant(scenario='imu_boxcar_100hz', note='run_nav_mc imu_boxcar_100hz, unmodified')


# --- "STM32 side" ------------------------------------------------------------------

class Stm32:
    """Encodes one navigation step's worth of traffic. Returns [(slot, wire bytes)]."""

    def __init__(self, scheme):
        self.scheme = scheme
        self.enc = link.Encoder()
        self.acc = link.ImuAccumulator(epoch=1)
        self.acc.snapshot(0)                     # the restart flag went out long before the descent
        self.residual = np.zeros(6)
        self.previous = (0,)*6

    def _raw(self, msg_id, payload):
        wire = link.frame(msg_id, self.enc.seq, payload)
        self.enc.seq = (self.enc.seq + 1) & 0xFF
        return wire

    def step(self, k, dtheta, dv, height):
        """Traffic at time k: the IMU message closing interval k-1 (dtheta, dv; None at k = 0),
        the barometer sample taken at k (height, or None) and the slower messages."""
        tick = (T0 + k*STEP_US) % link.TICK_WRAP
        out = []
        if self.scheme == 'cumulative':
            if dtheta is not None:
                self.acc.add_increment(dtheta, dv)
            out.append(('imu', self.enc.encode(self.acc.snapshot(tick))))
        else:
            if dtheta is None:
                counts = (0,)*6
            else:
                want = np.r_[np.asarray(dtheta)/DTHETA_LSB, np.asarray(dv)/DV_LSB] + self.residual
                counts = np.clip(np.floor(want + 0.5), -32767, 32767)
                self.residual = want - counts
                counts = tuple(int(c) for c in counts)
            if self.scheme == 'delta':
                out.append(('imu', self._raw(DELTA_ID, struct.pack('<I6h', tick, *counts))))
            else:
                out.append(('imu', self._raw(REDUNDANT_ID, struct.pack('<I12h', tick, *counts, *self.previous))))
            self.previous = counts
        if height is not None:
            out.append(('baro', self.enc.encode(link.Baro(
                tick=tick, pressure_pa=pressure_at(height, BASELINE_PA, MEAN_T), temperature_c=15.0, n_samples=5, flags=1))))
        if k % 10 == 5:
            out.append(('gnss', self.enc.encode(link.Gnss(
                tick=tick, itow_ms=k*10, year=2027, month=6, day=1, fix_type=3, fix_flags=1, num_sv=12, valid=7,
                lon_deg=-1.5, lat_deg=52.0, h_msl_m=300.0, h_acc_m=1.5, v_acc_m=2.5, vel_n=1.0, vel_e=2.0, vel_d=10.5,
                s_acc=0.05, p_dop=1.5))))
        if k % 20 == 0:
            out.append(('trig', self.enc.encode(link.Trig(frame=k//20, tick=tick, period_us=200_000, pulse_us=300))))
        if k % 100 == 0:
            out.append(('status', self.enc.encode(link.Status(tick=tick, state=2, battery_v=7.6, baseline_pa=BASELINE_PA))))
        if k % 100 == 50:
            out.append(('pps', self.enc.encode(link.Pps(tick=tick, itow_ms=k*10, interval=1_000_000, count=k//100, flags=1))))
        return out


# --- UART --------------------------------------------------------------------------

class Uart:
    """Frame corruption, byte errors and an outage. The loss draws depend only on the
    descent and the step, so every scheme loses the same messages."""

    def __init__(self, v, j, steps):
        self.v = v
        rng = np.random.default_rng([nm.SEED, j, 77])
        self.hit = rng.random((steps + 1, len(SLOTS)))
        self.where = rng.random((steps + 1, len(SLOTS)))
        self.bit = rng.integers(0, 8, (steps + 1, len(SLOTS)))
        self.rng = np.random.default_rng([nm.SEED, j, 78])
        self.sent = self.corrupted = self.cut = 0

    def carry(self, k, frames):
        v = self.v
        out = bytearray()
        t = k*nm.NAV_DT
        for name, wire in frames:
            self.sent += 1
            slot = SLOTS[name]
            if v['outage'] and v['outage'][0] <= t < v['outage'][1]:
                self.cut += 1
                if k == round(v['outage'][0]/nm.NAV_DT) and name == 'imu':
                    out += wire[:len(wire)//2]            # the cut falls inside a frame
                elif k == round(v['outage'][1]/nm.NAV_DT) - 1 and name == 'imu':
                    out += wire[len(wire)//2:]
                continue
            if self.hit[k, slot] < v['loss']:
                wire = bytearray(wire)
                wire[int(self.where[k, slot]*len(wire))] ^= 1 << int(self.bit[k, slot])
                self.corrupted += 1
            if v['ber']:
                bad = np.flatnonzero(self.rng.random(len(wire)) < v['ber'])
                if len(bad):
                    wire = bytearray(wire)
                    for i in bad:
                        wire[i] = int(self.rng.integers(256))
                    self.corrupted += 1
            out += wire
        return bytes(out)


# --- "Pi side" ---------------------------------------------------------------------

def step_of(tick):
    return round(((tick - T0) % link.TICK_WRAP)/STEP_US)


class Pi:
    """Decodes the stream. receive() returns (increments, barometer samples):
    increments are (end step, link.ImuIncrement, steps since the previous IMU
    message received, interpolated?), barometer samples are (step, height)."""

    def __init__(self, scheme):
        self.scheme = scheme
        self.dec = link.Decoder(raw_unknown=True)
        self.dif = link.ImuDifferencer(max_gap_s=2.0)
        self.last = None                         # delta schemes: (step, counts)
        self.concealed = self.recovered = self.imu_received = 0
        self.longest = 0

    def receive(self, data):
        increments, baro = [], []
        for m in self.dec.feed(data):
            if isinstance(m, link.Imu):
                self.imu_received += 1
                inc = self.dif.feed(m)
                if inc is not None:
                    gap = round(inc.dt/nm.NAV_DT)
                    increments.append((step_of(m.tick), inc, gap, False))
                    self.longest = max(self.longest, gap)
            elif isinstance(m, link.Raw) and m.msg_id in (DELTA_ID, REDUNDANT_ID):
                self.imu_received += 1
                values = struct.unpack('<I6h' if m.msg_id == DELTA_ID else '<I12h', m.data)
                k, new = step_of(values[0]), np.array(values[1:7], dtype=float)
                if self.last is None and k > 0:
                    self.last = (0, new)             # nothing earlier arrived: hold this one back to the start
                if self.last is not None and k > self.last[0]:
                    k0, old = self.last
                    gap = k - k0
                    self.longest = max(self.longest, gap)
                    for i in range(1, gap):
                        if i == gap - 1 and m.msg_id == REDUNDANT_ID:
                            counts, concealed = np.array(values[7:13], dtype=float), False
                            self.recovered += 1
                        else:
                            counts, concealed = old + (new - old)*i/gap, True
                            self.concealed += 1
                        increments.append((k0 + i, self._increment(k0 + i, counts), gap, concealed))
                    increments.append((k, self._increment(k, new), gap, False))
                self.last = (k, new)
            elif isinstance(m, link.Baro) and math.isfinite(m.pressure_pa) and m.flags & 1:
                baro.append((step_of(m.tick), pressure_altitude(m.pressure_pa, BASELINE_PA, MEAN_T)))
        return increments, baro

    @staticmethod
    def _increment(k, counts):
        return link.ImuIncrement((T0 + k*STEP_US) % link.TICK_WRAP, nm.NAV_DT, tuple(counts[:3]*DTHETA_LSB),
                                 tuple(counts[3:]*DV_LSB))


# --- One descent -------------------------------------------------------------------

def run_descent(job):
    name, j, n = job
    v = VARIANTS[name]
    scenario = v['scenario'] or DEFAULT_SCENARIO
    if v['scheme'] is None:
        return nm.run_descent((scenario, j, n))
    sc = nm.SCENARIOS[scenario]
    s, c = sc['sensors'], sc['filter']
    if c.start != 'drawn':
        raise ValueError('only scenarios started from a drawn state are supported')
    truth = nm.load_truth(sc['canopy'], n)
    steps = truth['accel'].shape[0]
    t_p, t_v, t_r = (np.asarray(truth[k][:, j]) for k in ('p', 'v', 'r'))
    frames = truth['delay_p'].shape[1]
    r_map, r_init, r_pix = (np.random.default_rng([nm.SEED, j, k]) for k in range(1, 4))
    choices = s.camera_jitter or (s.camera_delay,)
    d_idx = np.array([nm.DELAYS.index(d) for d in choices])[
        np.random.default_rng([nm.SEED, j, 4]).integers(len(choices), size=frames)]
    cam_p = np.asarray(truth['delay_p'][:, :, j])[d_idx, np.arange(frames)]
    cam_r = np.asarray(truth['delay_r'][:, :, j])[d_idx, np.arange(frames)]
    true_delay = np.array(nm.DELAYS)[d_idx]

    accel, gyro, ba, bg, baro_bias, baro_noise = nm.imu_stream(s, truth, j)

    # --- From here to the filter's construction this is run_nav_mc.run_descent, line for line.
    reach = 1.3*t_p[0, 2]
    lo, hi = t_p[:, :2].min(axis=0) - reach, t_p[:, :2].max(axis=0) + reach
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0], s.landmark_spacing), np.arange(lo[1], hi[1], s.landmark_spacing))
    grid = np.column_stack((gx.ravel(), gy.ravel()))
    grid = grid + r_map.uniform(-0.4, 0.4, grid.shape)*s.landmark_spacing
    map_points = np.column_stack((grid, np.zeros(len(grid))))
    unit = r_map.normal(size=(len(grid), 3))
    world_points = map_points + unit*np.array([s.map_xy_sd, s.map_xy_sd, s.relief_sd])
    world_points[:, :2] += np.array(s.datum)

    true_cam = nm.nominal_camera()
    calib = r_map.normal(size=6)
    true_cam.fx = true_cam.fy = nm.FOCAL*(1 + s.focal_error*calib[0])
    true_cam.cx += s.principal_error*calib[1]
    true_cam.cy += s.principal_error*calib[2]
    true_cam.r_cb = exp_so3(s.misalignment*calib[3:])
    cam = nm.nominal_camera()

    sd = np.r_[c.init_pos, c.init_vel, c.init_att, np.full(3, c.init_accel_bias), np.full(3, c.init_gyro_bias),
               c.init_baro_bias, c.init_time_offset]
    e0 = sd*np.r_[r_init.normal(size=16), 0.0]
    noise = ImuNoise(c.accel_noise*c.noise_scale, c.gyro_noise*c.noise_scale, c.accel_bias_walk,
                     c.gyro_bias_walk, c.baro_bias_walk, gyro_scale=c.gyro_scale, scale_tau=nm.FRAME_DT)
    f = Eskf(t_p[0] - e0[nm.POS], t_v[0] - e0[VEL], t_r[0] @ exp_so3(-e0[ATT]), np.diag(sd**2), noise)
    f.rate = gyro[0].copy()

    map_var = None if c.map_var is None else np.array(c.map_var)*c.map_inflation
    frame_every = round(s.frame_dt/nm.NAV_DT)
    base_every = round(nm.FRAME_DT/nm.NAV_DT)
    rec = {k: np.full((frames,) + shape, np.nan, dtype=np.float32) for k, shape in (
        ('err', (N_ERR,)), ('sig', (N_ERR,)), ('nees', (8,)), ('counts', (6,)), ('innov', ()), ('pl', ()),
        ('height', ()), ('swing', ()))}
    swing = truth['swing']
    images_dropped = [0]

    def at_time(k, usable, height):
        """Everything run_nav_mc does at step k before it propagates: the image, the record,
        the barometer. `usable` is False when the IMU thread up to k was bridged over a long gap."""
        if k % base_every == 0 and k//base_every < frames:
            fi = k//base_every
            t_now = k*nm.NAV_DT
            counts = np.zeros(6)
            innov = np.nan
            use = (k % frame_every == 0 and f.p[2] >= c.map_floor
                   and not any(a <= t_now < b for a, b in s.dropout))
            if use and not usable:
                images_dropped[0] += 1
                use = False
            if use:
                pix_true, depth = true_cam.project(world_points, cam_p[fi], cam_r[fi])
                chosen = nm.select_landmarks(pix_true, depth, s.max_landmarks, s.fov_fraction, search_m=s.search_margin_m)
                counts[0] = len(chosen)
                if len(chosen):
                    m = len(chosen)
                    z = np.asarray(depth[chosen])
                    pixel_sd = np.sqrt(s.pixel_noise**2 + (s.ground_noise*nm.FOCAL/z)**2)
                    pixels = pix_true[chosen] + pixel_sd[:, None]*r_pix.normal(size=(m, 2))
                    bad = r_pix.random(m) < s.outlier_rate
                    pixels[bad] += (r_pix.uniform(-s.outlier_range, s.outlier_range, (int(bad.sum()), 2))
                                    * (nm.FOCAL/z[bad])[:, None])
                    if s.alias_count:
                        alias = np.zeros(m, dtype=bool)
                        alias[:min(s.alias_count, m)] = True
                        shift_px = (cam_r[fi].T @ np.array([*s.alias_shift, 0.0]))[:2]
                        pixels[alias] += shift_px*(nm.FOCAL/z[alias])[:, None]
                        bad |= alias
                    predicted, z_hat = cam.project(map_points[chosen], f.p, f.r)
                    z_hat = np.maximum(z_hat, 1.0)
                    pixel_var = c.pixel_noise**2 + (c.ground_noise*nm.FOCAL/z_hat)**2
                    good = ~bad
                    if good.any():
                        innov = float(np.max(np.linalg.norm(pixels[good] - predicted[good], axis=1)*z_hat[good]/nm.FOCAL))
                    out = f.update_landmarks(cam, map_points[chosen], pixels, pixel_var, map_var, c.gate,
                                             c.iterations, c.robust, c.recover)
                    acc = out['accepted']
                    counts[1:] = acc.sum(), (good & ~acc).sum(), (bad & acc).sum(), bad.sum(), out['recovered']
            e = f.error_from(t_p[k], t_v[k], t_r[k], ba[k], bg[k], baro_bias[k], true_delay[fi])
            cov = f.cov
            rec['err'][fi] = e
            rec['sig'][fi] = np.sqrt(np.diag(cov))
            try:
                rec['nees'][fi] = [e[:16] @ np.linalg.solve(cov[:16, :16], e[:16]),
                                   e[:2] @ np.linalg.solve(cov[:2, :2], e[:2]),
                                   e[3:5] @ np.linalg.solve(cov[3:5, 3:5], e[3:5])] + [
                    e[b] @ np.linalg.solve(cov[b, b], e[b]) for b in (ATT, nm.BA, nm.BG)] + [
                    e[nm.BARO]**2/cov[nm.BARO, nm.BARO], e[2]**2/cov[2, 2]]
            except np.linalg.LinAlgError:
                pass
            rec['counts'][fi] = counts
            rec['innov'][fi] = innov
            rec['pl'][fi] = f.protection_level()
            rec['height'][fi] = t_p[k, 2]
            rec['swing'][fi] = swing[k, j]
        if height is not None and usable:
            f.update_baro(height, c.baro_sigma)

    def measured_height(k):
        return t_p[k, 2] + baro_bias[k] + baro_noise[k] if k % 10 == 0 else None

    stats = np.zeros(8, dtype=np.float32)
    if v['scheme'] == 'direct':
        at_time(0, True, measured_height(0))
        for k in range(1, steps):
            f.propagate(nm.NAV_DT, accel[k - 1], gyro[k - 1])
            at_time(k, True, measured_height(k))
    else:
        stm32, uart, pi = Stm32(v['scheme']), Uart(v, j, steps), Pi(v['scheme'])
        pending = {}                                 # barometer samples received, by step
        done = 0                                     # the filter's state is at this step
        hold = None
        for k in range(steps):
            if k == 0:
                dtheta = dv = None
            else:
                # The simulated IMU reports mean rate and mid-interval mean specific force
                # (truth.simulate). As increments in start-of-interval axes:
                dtheta = gyro[k - 1]*nm.NAV_DT
                dv = exp_so3(dtheta/2) @ accel[k - 1]*nm.NAV_DT
            data = uart.carry(k, stm32.step(k, dtheta, dv, measured_height(k)))
            increments, baro = pi.receive(data)
            for kb, height in baro:
                if kb >= done:
                    pending[kb] = height
            if k == 0:
                at_time(0, True, pending.pop(0, None))
            for k_end, inc, gap, concealed in increments:
                k_start = k_end - round(inc.dt/nm.NAV_DT)
                if k_start < done:
                    stats[7] += 1                    # overlaps what is already integrated: should never happen
                    continue
                if k_start > done:
                    # No inertial thread from `done` to here (the first snapshot never arrived,
                    # or the differencer restarted). Coast on this increment's own mean.
                    stats[7] += 1
                    a, w = inc.mean()
                    for ke in range(done + 1, k_start + 1):
                        f.propagate(nm.NAV_DT, np.array(a), np.array(w))
                        f.cov[ATT, ATT] += np.eye(3)*CONCEAL_ATT**2
                        f.cov[VEL, VEL] += np.eye(3)*CONCEAL_VEL**2
                        at_time(ke, False, pending.pop(ke, None))
                cur, rest = k_start, inc
                for ke in range(k_start + 1, k_end + 1):
                    wanted = ke == k_end or ke in pending or (ke % base_every == 0 and ke//base_every < frames)
                    if not wanted:
                        continue
                    part, rest = (rest, None) if ke == k_end else rest.split((ke - cur)/(k_end - cur))
                    a, w = part.mean()
                    f.propagate(part.dt, np.array(a), np.array(w))
                    hold = (np.array(a), np.array(w))
                    cur = ke
                    at_time(ke, gap <= GAP_USE, pending.pop(ke, None))
                if concealed:
                    f.cov[ATT, ATT] += np.eye(3)*CONCEAL_ATT**2
                    f.cov[VEL, VEL] += np.eye(3)*CONCEAL_VEL**2
                done = k_end
        # Images after the last IMU message received are scored on a held rate.
        for ke in range(done + 1, steps):
            if hold is not None:
                f.propagate(nm.NAV_DT, *hold)
            at_time(ke, False, None)
        d = pi.dec
        stats[:7] = (uart.sent, uart.sent - d.frames, pi.imu_received, pi.longest, images_dropped[0],
                     pi.concealed, d.crc + d.stuffing + d.short + d.too_long + d.length)
    rec['first_fix'] = np.array([0.0, 0.0, np.nan, np.nan], dtype=np.float32)
    rec['link'] = stats
    return rec


DEFAULT_SCENARIO = 'nominal_moderate'
KEYS = ('horizontal_rms_m', 'horizontal_p95_m', 'horizontal_max_m', 'T1_fraction_within_10m',
        'T1_fraction_within_10m_after_2s', 'tilt_rms_deg', 'velocity_rms_mps', 'height_rms_m', 'nees_pos2_mean',
        'protection_level_exceedance', 'landmarks_accepted_per_frame', 'recoveries_per_descent')


def summarise(name, recs):
    v = VARIANTS[name]
    scenario = v['scenario'] or DEFAULT_SCENARIO
    summary, _, stack = nm.summarise(scenario, recs)
    row = {'scenario': scenario, 'scheme': v['scheme'] or 'none', 'frame_loss': v['loss'], 'byte_error_rate': v['ber'],
           'outage_s': list(v['outage']) if v['outage'] else None, 'note': v['note'], 'descents': len(recs)}
    row.update({k: summary[k] for k in KEYS})
    if 'link' in stack:
        st = stack['link']
        sent = max(st[:, 0].sum(), 1.0)
        row.update({'frames_sent_per_descent': float(st[:, 0].mean()), 'frames_not_decoded_fraction': float(st[:, 1].sum()/sent),
                    'imu_messages_received_per_descent': float(st[:, 2].mean()), 'longest_imu_gap_steps': float(st[:, 3].max()),
                    'images_not_used_per_descent': float(st[:, 4].mean()), 'intervals_interpolated_per_descent': float(st[:, 5].mean()),
                    'frames_rejected_per_descent': float(st[:, 6].mean()), 'thread_breaks': float(st[:, 7].sum())})
    return row, stack


def stored_difference(scenario, recs):
    """Largest difference between these descents and the same ones in the stored Monte Carlo
    (sim/nav/generated/raw_<scenario>.npz, written by run_nav_mc.py), or None if it is absent."""
    path = nm.CACHE/f'raw_{scenario}.npz'
    if not path.exists():
        return None
    stored = np.load(path)['err']
    mine = np.stack([r['err'] for r in recs])
    if stored.shape[0] < mine.shape[0] or stored.shape[1:] != mine.shape[1:]:
        return None
    return float(np.nanmax(np.abs(stored[:mine.shape[0]] - mine)))


def main():
    global DEFAULT_SCENARIO
    parser = argparse.ArgumentParser()
    parser.add_argument('--descents', type=int, default=16, help='the first N of the truth batch')
    parser.add_argument('--batch', type=int, default=160, help='size of the cached truth batch to draw from')
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--only', default='')
    parser.add_argument('--scenario', default=DEFAULT_SCENARIO)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.list:
        print('\n'.join(VARIANTS))
        return
    DEFAULT_SCENARIO = args.scenario
    names = [x for x in args.only.split(',') if x] or list(VARIANTS)
    OUT.mkdir(parents=True, exist_ok=True)
    for canopy in sorted({nm.SCENARIOS[VARIANTS[x]['scenario'] or args.scenario]['canopy'] for x in names}):
        nm.build_truth(canopy, args.batch)
    path = OUT/f'summary_{args.scenario}.json'
    report = json.loads(path.read_text()) if path.exists() else {}
    report.update({'label': 'SYNTHETIC', 'seed': nm.SEED, 'truth_batch': args.batch, 'gap_use_steps': GAP_USE,
                   'conceal_att_rad': CONCEAL_ATT, 'conceal_vel_mps': CONCEAL_VEL,
                   'wire_bytes_per_imu_message': {'cumulative': link.wire_size(link.Imu), 'delta': 20 + link.OVERHEAD,
                                                  'redundant': 32 + link.OVERHEAD},
                   'wire_bytes_note': 'delta and redundant counted with the 4 bytes of epoch, flags and sample counts '
                                      'that the cumulative message carries; the experimental frames here omit them'})
    report.setdefault('variants', {})
    from multiprocessing import Pool
    with Pool(args.workers, initializer=_init, initargs=(args.scenario,)) as pool:
        for name in names:
            t0 = time.perf_counter()
            recs = pool.map(run_descent, [(name, j, args.batch) for j in range(args.descents)], chunksize=2)
            row, _ = summarise(name, recs)
            row['seconds'] = time.perf_counter() - t0
            if VARIANTS[name]['scheme'] is None:
                row['max_abs_difference_from_stored_run'] = stored_difference(row['scenario'], recs)
            report['variants'][name] = row
            print('%-30s n=%3d %5.0fs  horiz rms %.3f p95 %.2f max %.2f m | T1 %.3f | tilt %.3f deg | vel %.3f m/s | '
                  'lost %.4f | gap %2.0f | images dropped %.2f' % (
                      name, row['descents'], row['seconds'], row['horizontal_rms_m'], row['horizontal_p95_m'],
                      row['horizontal_max_m'], row['T1_fraction_within_10m'], row['tilt_rms_deg'],
                      row['velocity_rms_mps'], row.get('frames_not_decoded_fraction', 0.0),
                      row.get('longest_imu_gap_steps', 0.0), row.get('images_not_used_per_descent', 0.0)), flush=True)
            path.write_text(json.dumps(report, indent=1))


def _init(scenario):
    global DEFAULT_SCENARIO
    DEFAULT_SCENARIO = scenario


if __name__ == '__main__':
    main()
