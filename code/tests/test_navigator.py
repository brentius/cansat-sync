"""GPL-3.0-only. Navigation orchestration, delay, reset and GNSS isolation tests."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import link
from cansat.atmosphere import pressure_at
from cansat.eskf import LEVEL
from cansat.fine import FineMatches
from cansat.firstfix import FirstFix
from cansat.navigator import Navigator, NavigatorConfig
from cansat.telemetry import FLAG_TRN_VALID
from test_sitemap import small_site


def status(epoch=0, state=2):
    return link.Status(epoch=epoch, state=state, baseline_pa=101325., altitude_m=30.,
                       health=link.HEALTH_BASELINE | link.HEALTH_ORIGIN,
                       origin_lat_deg=22., origin_lon_deg=114., origin_h_msl_m=15.)


def nav_ready(**kwargs):
    records = []
    nav = Navigator(NavigatorConfig(min_height_m=5., max_height_m=400., **kwargs), small_site(), records.append)
    nav.on_status(status(), 0.)
    return nav, records, link.ImuAccumulator()


def feed(nav, acc, t):
    acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
    nav.on_imu(acc.snapshot(round(t * link.TICK_HZ)), t)
    nav.on_baro(link.Baro(pressure_pa=pressure_at(30. - t, 101325., 283.15), flags=1), t)


def confirm(nav, t):
    fix = FirstFix(True, 'ok', np.array([t, 0., 30. - t]), LEVEL.copy(), np.eye(6)*.001)
    nav.acquirer = SimpleNamespace(update=lambda *a, **kw: (
        'confirmed', SimpleNamespace(ok=True, reason='ok', fix=fix, matches=12), np.array([1., 0., -1.])))
    nav.on_frame(np.zeros((120, 160), dtype=np.uint8), t, 9)


def test_delayed_acquisition_replays_imu_and_baro_to_latest_time():
    a, _, aa = nav_ready()
    b, _, ab = nav_ready()
    for i in range(61):
        t = i*.01
        feed(a, aa, t)
        feed(b, ab, t)
        if i == 30:
            confirm(a, .30)
    confirm(b, .30)
    assert a.mode == b.mode == 'tracking'
    np.testing.assert_allclose(a.filter.p, b.filter.p, atol=1e-10)
    np.testing.assert_allclose(a.filter.v, b.filter.v, atol=1e-10)
    np.testing.assert_allclose(a.filter.cov, b.filter.cov, atol=1e-10)
    msg = b.nav_message(600000)
    assert msg.flags & FLAG_TRN_VALID
    assert msg.tick == 600000 and msg.frame == 9
    assert not b.nav_message(2000000).flags & FLAG_TRN_VALID


def test_gnss_never_initialises_or_changes_trn():
    nav, logs, acc = nav_ready()
    gnss = link.Gnss(fix_type=3, fix_flags=1, lat_deg=22., lon_deg=114., h_msl_m=45.,
                     vel_e=10., vel_n=-3., vel_d=1., h_acc_m=1., v_acc_m=2., s_acc=.2)
    nav.on_gnss(gnss, .1)
    assert nav.filter is None and nav.nav_message(100000) is None
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    before = nav.filter.p.copy(), nav.filter.cov.copy()
    nav.on_gnss(gnss, .4)
    np.testing.assert_array_equal(nav.filter.p, before[0])
    np.testing.assert_array_equal(nav.filter.cov, before[1])
    assert any(r['kind'] == 'gnss_pred' and r['source'] == 'gnss' for r in logs)


def test_inertial_comparison_shares_first_fix_then_ignores_other_measurements():
    a, logs, aa = nav_ready()
    b, _, ab = nav_ready()
    points = np.array([[x, y, 0.] for x in (-8., 0., 8.) for y in (-8., 0., 8.)])
    pixels, _ = a.site.camera.project(points, np.array([.7, 0., 29.5]), LEVEL)
    matches = FineMatches(points=points, pixels=pixels, pixel_var=np.ones(9),
                          map_var=np.ones((9, 3))*.01)
    a.site.fine_matcher = SimpleNamespace(match_filter=lambda *args: matches)
    for i in range(81):
        t = i*.01
        feed(a, aa, t)
        feed(b, ab, t)
        if i == 30:
            confirm(a, .3)
            confirm(b, .3)
        if i == 50:
            a.on_frame(np.zeros((120, 160), np.uint8), .5, 10)
            a.on_baro(link.Baro(pressure_pa=pressure_at(32., 101325., 283.15), flags=1), .505)
    assert a._last_frame == 10 and b._last_frame == 9
    np.testing.assert_array_equal(a.inertial_filter.p, b.inertial_filter.p)
    np.testing.assert_array_equal(a.inertial_filter.v, b.inertial_filter.v)
    np.testing.assert_array_equal(a.inertial_filter.cov, b.inertial_filter.cov)
    np.testing.assert_array_equal(a.inertial_predictor.last.drift, b.inertial_predictor.last.drift)
    np.testing.assert_array_equal(a.inertial_predictor.last.cov, b.inertial_predictor.last.cov)
    assert not np.allclose(a.filter.p, b.filter.p)
    before = a.inertial_filter.p.copy()
    a.on_gnss(link.Gnss(fix_type=3, fix_flags=1, lat_deg=22., lon_deg=114., h_msl_m=45.,
                       vel_e=10., vel_n=-3., vel_d=1., h_acc_m=1., v_acc_m=2., s_acc=.2), .8)
    np.testing.assert_array_equal(a.inertial_filter.p, before)
    np.testing.assert_array_equal(a.inertial_filter.v, b.inertial_filter.v)
    np.testing.assert_array_equal(a.inertial_filter.cov, b.inertial_filter.cov)
    np.testing.assert_array_equal(a.inertial_predictor.last.drift, b.inertial_predictor.last.drift)
    np.testing.assert_array_equal(a.inertial_predictor.last.cov, b.inertial_predictor.last.cov)
    assert any(r['kind'] == 'inertial_pred' and r['source'] == 'inertial_only' for r in logs)
    assert {r['source'] for r in logs if r['kind'] == 'wind'} == {'trn', 'gnss'}
    a.on_imu(link.Imu(epoch=1), .81)
    assert a.inertial_filter is None


def test_inertial_landing_uses_propagated_velocity_and_uncertainty():
    from cansat.landing import LandingPredictor

    nav, logs, acc = nav_ready()
    samples = []
    for i in range(61):
        feed(nav, acc, i*.01)
        if i == 30:
            confirm(nav, .3)
        if nav.inertial_filter is not None and nav.inertial_predictor.last.t == i*.01:
            f = nav.inertial_filter
            samples.append((i*.01, f.p.copy(), f.v.copy(), f.cov.copy()))
    pred = nav.inertial_predictor.last
    assert pred.time_to_go > 0
    assert pred.drift[0] > .1 and abs(pred.drift[1]) < 1e-9
    assert pred.east > nav.inertial_filter.p[0]
    # Replay the recorded inertial states through the real predictor. Incorrect
    # velocity or variance wiring changes both drift and prediction uncertainty.
    reference = LandingPredictor(model=nav.site.wind_model, table=nav.site.drift_table)
    predictions = [r for r in logs if r['kind'] == 'inertial_pred']
    assert len(samples) == len(predictions) >= 2
    for r, (t, position, velocity, covariance) in zip(predictions, samples):
        np.testing.assert_allclose(r['position'], position)
        np.testing.assert_allclose(r['velocity'], velocity)
        np.testing.assert_allclose(r['velocity_variance'], np.diag(covariance)[3:5])
        np.testing.assert_allclose(r['position_covariance'], covariance[:2, :2])
        expected = reference.update(t, position, velocity[:2], np.diag(covariance)[3:5],
                                    position_cov=covariance[:2, :2])
    np.testing.assert_allclose(pred.drift, expected.drift)
    np.testing.assert_allclose(pred.cov, expected.cov)


def test_comparison_logs_joint_covariance_at_delayed_epoch_and_coasting_states():
    from copy import deepcopy

    nav, logs, acc = nav_ready(coast_after_s=.1)
    for i in range(91):
        feed(nav, acc, i*.01)
        if i == 30:
            confirm(nav, .3)
    reference = deepcopy(nav._anchor)
    nav._advance(reference, nav._anchor_t, .505)
    live_cov = nav.filter.cov.copy()
    nav.on_gnss(link.Gnss(fix_type=3, fix_flags=1, lat_deg=22., lon_deg=114., h_msl_m=44.5,
                          vel_e=1., vel_n=0., vel_d=1., h_acc_m=1., v_acc_m=2., s_acc=.2), .505)
    record = [r for r in logs if r['kind'] == 'comparison'][-1]
    indices = [0, 1, 3, 4]
    expected_cov = reference.cov[np.ix_(indices, indices)]
    np.testing.assert_allclose(record['horizontal_covariance'], expected_cov)
    assert not np.allclose(expected_cov, live_cov[np.ix_(indices, indices)])
    assert abs(expected_cov[0, 2]) > 1e-6  # preserve position/velocity cross terms
    assert record['t'] == .505 and record['covariance_source'] == 'trn_eskf'
    assert record['horizontal_order'] == ['p_e', 'p_n', 'v_e', 'v_n']
    assert record['horizontal_units'] == ['m', 'm', 'm/s', 'm/s']
    assert record['covariance_units'][0][2] == 'm^2/s'
    assert record['time_basis'] == 'unwrapped STM32 seconds'
    assert record['coordinate_frame'] == 'site_pad_enu'
    assert isinstance(record['generation'], int) and record['epoch'] == 0
    states = [r for r in logs if r['kind'] == 'nav' and r['mode'] == 'coasting'
              and r['update_source'] == 'inertial_cadence']
    assert len(states) >= 2 and states[-1]['t'] > .7
    assert max(np.diff([r['t'] for r in states])) <= .22
    for r in states:
        assert np.asarray(r['horizontal_covariance']).shape == (4, 4)
        assert r['covariance_source'] == 'trn_eskf' and r['source'] == 'trn'
    generation = record['generation']
    nav.restart()
    assert logs[-1]['generation'] == generation + 1


def test_restart_clears_all_time_dependent_state():
    nav, _, acc = nav_ready()
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    nav.restart()
    assert nav.mode == 'waiting' and nav.filter is None
    assert nav.nav_message(0) is None
    assert nav.pi_flags == link.PIFLAG_MAP
    assert not nav.wants_frames()
    nav.on_status(status(epoch=1), 0.)
    assert not nav.wants_frames()  # needs a fresh pressure/levelling history


def test_bounded_history_refuses_old_future_and_out_of_order_frames():
    nav, logs, acc = nav_ready(history_seconds=.5, history_max_events=35)
    for i in range(151):
        feed(nav, acc, i*.01)
    assert len(nav._imus) <= 35 and len(nav._baros) <= 35
    confirm(nav, .1)
    assert nav.filter is None
    confirm(nav, 2.)
    assert nav.filter is None
    confirm(nav, 1.4)
    assert nav.filter is not None
    p = nav.filter.p.copy()
    confirm(nav, 1.3)
    np.testing.assert_array_equal(nav.filter.p, p)
    assert sum(r.get('event') == 'frame_rejected' for r in logs) == 3


def test_inertial_break_and_landed_invalidate_solution():
    nav, _, acc = nav_ready()
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    msg = acc.snapshot(310000)
    msg.flags = link.IMU_GYRO_SATURATED
    nav.on_imu(msg, .31)
    assert nav.mode == 'lost' and not nav.nav_message(310000).flags
    nav.on_status(status(state=3), .4)
    assert nav.mode == 'done' and not nav.wants_frames()


def test_coast_loss_reacquisition_and_epoch_change():
    nav, _, acc = nav_ready(coast_after_s=.1, lost_after_s=.3)
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    for i in range(31, 51):
        feed(nav, acc, i*.01)
    assert nav.mode == 'coasting'
    for i in range(51, 71):
        feed(nav, acc, i*.01)
    assert nav.mode == 'lost' and nav.wants_frames()
    nav.on_status(status(epoch=1), .71)
    assert nav.filter is None and nav.mode == 'waiting'


def test_below_map_floor_continues_bounded_inertial_baro_coasting():
    nav, _, acc = nav_ready(coast_after_s=.1, lost_after_s=.3, low_altitude_coast_s=3.)
    nav.config.min_height_m = 29.6
    for i in range(201):
        feed(nav, acc, i*.01)
        if i == 30:
            confirm(nav, .3)
    assert nav.mode == 'coasting' and nav.filter is not None
    assert not nav.wants_frames()
    assert nav.nav_message(2000000).flags & FLAG_TRN_VALID
    # A missing pressure stream cannot grant the below-map exception forever.
    for i in range(201, 320):
        acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
        nav.on_imu(acc.snapshot(i*10000), i*.01)
    assert nav.filter is None


@pytest.mark.parametrize('values', [dict(history_max_events=0), dict(history_seconds=-1),
                                    dict(coast_after_s=4, lost_after_s=1)])
def test_bad_configuration_refused(values):
    with pytest.raises(ValueError):
        NavigatorConfig(**values)


def test_fractional_exposure_tracking_replay_matches_prompt_delivery():
    a, _, aa = nav_ready()
    b, _, ab = nav_ready()
    points = np.array([[x, y, 0.] for x in (-8., 0., 8.) for y in (-8., 0., 8.)])
    pixels, _ = a.site.camera.project(points, np.array([.455, 0., 29.545]), LEVEL)
    matches = FineMatches(points=points, pixels=pixels, pixel_var=np.ones(9),
                          map_var=np.ones((9, 3))*.01)
    a.site.fine_matcher = b.site.fine_matcher = SimpleNamespace(match_filter=lambda *args: matches)
    for i in range(71):
        t = i*.01
        feed(a, aa, t)
        feed(b, ab, t)
        if i == 31:
            confirm(a, .305)
        if i == 46:
            a.on_frame(np.zeros((120, 160), np.uint16), .455, 10)
        if i == 60:
            confirm(b, .305)
            b.on_frame(np.zeros((120, 160), np.uint16), .455, 10)
    np.testing.assert_allclose(a.filter.p, b.filter.p, atol=1e-10)
    np.testing.assert_allclose(a.filter.cov, b.filter.cov, atol=1e-10)
    assert a._last_frame == b._last_frame == 10


def test_baro_history_is_bounded_even_without_imu():
    nav, _, _ = nav_ready(history_max_events=10)
    for i in range(100):
        nav.on_baro(link.Baro(pressure_pa=101000., flags=1), i*.01)
    assert len(nav._baros) <= 10


def test_invalid_pressure_does_not_enable_frames():
    nav, _, acc = nav_ready()
    for i in range(20):
        acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
        nav.on_imu(acc.snapshot(i*10000), i*.01)
    nav.on_baro(link.Baro(pressure_pa=101000., flags=0), .19)
    assert not nav.wants_frames()


def test_nav_wraps_time_and_uses_status_output_origin():
    nav, _, acc = nav_ready()
    start = link.TICK_WRAP/link.TICK_HZ - .2
    for i in range(61):
        feed(nav, acc, start + i*.01)
    # The helper height would be negative at this absolute time: supply an
    # independent pressure measurement before acquiring in the wrapped epoch.
    nav.on_baro(link.Baro(pressure_pa=pressure_at(30., 101325., 283.15), flags=1), start+.601)
    # Advance to include that pressure epoch.
    acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
    nav.on_imu(acc.snapshot(round((start+.61)*link.TICK_HZ)), start+.61)
    confirm(nav, start+.605)
    msg = nav.nav_message(round((start+.61)*link.TICK_HZ) & 0xffffffff)
    assert msg.flags & FLAG_TRN_VALID
    assert msg.tick < 1000000
    moved = status()
    moved.origin_lon_deg += .00001
    nav.on_status(moved, start+.61)
    translated = nav.nav_message(msg.tick)
    assert translated.trn_e < msg.trn_e


def test_malformed_imu_recovers_without_bridging_bad_interval():
    nav, _, acc = nav_ready()
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    nav.on_imu(link.Imu(tick=310000, q=(0, 0, 0, 0)), .31)
    assert nav.filter is None
    for i in range(32, 51):
        feed(nav, acc, i*.01)
    confirm(nav, .45)
    assert nav.filter is not None


def test_pressure_backlog_pruning_preserves_filter_corrections():
    a, _, aa = nav_ready(history_max_events=10)
    b, _, ab = nav_ready(history_max_events=100)
    for i in range(31):
        feed(a, aa, i*.01)
        feed(b, ab, i*.01)
    confirm(a, .3)
    confirm(b, .3)
    # A 500 Hz pressure source and 100 Hz IMU deliberately exercise the
    # independent count bound, not just the normal ten-Hz flight rate.
    for i in range(151, 251):
        t = i*.002
        if i % 5 == 0:
            aa.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
            ab.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
            a.on_imu(aa.snapshot(round(t*link.TICK_HZ)), t)
            b.on_imu(ab.snapshot(round(t*link.TICK_HZ)), t)
        msg = link.Baro(pressure_pa=pressure_at(30. - t + .1, 101325., 283.15), flags=1)
        a.on_baro(msg, t)
        b.on_baro(msg, t)
    assert len(a._baros) <= 10 and a._anchor_t > .3
    np.testing.assert_allclose(a.filter.p, b.filter.p, atol=1e-9)
    np.testing.assert_allclose(a.filter.cov, b.filter.cov, atol=1e-9)


def test_pending_acquisition_is_cleared_when_pressure_reference_changes():
    nav, _, acc = nav_ready()
    for i in range(31):
        feed(nav, acc, i*.01)
    nav.acquirer.pending = object()
    changed = status()
    changed.baseline_pa = 101300.
    nav.on_status(changed, .31)
    assert nav.acquirer.pending is None
    assert not nav.wants_frames()


def test_lost_imu_packets_bridge_integrals_without_losing_navigation():
    nav, _, acc = nav_ready()
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    for i in range(31, 61):
        acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
        if i % 5 == 0:
            nav.on_imu(acc.snapshot(i*10000), i*.01)
    assert nav.mode == 'tracking'
    assert nav._differencer.bridged >= 6
    np.testing.assert_allclose(nav.filter.p, [.6, 0., 29.4], atol=1e-4)


def test_missed_sample_flags_keep_the_inertial_thread():
    # Protocol section 6: the STM32 integrates across a missed sample, so the
    # snapshot stays usable. Only saturation/fault/restart break the thread.
    nav, logs, acc = nav_ready()
    for i in range(31):
        feed(nav, acc, i*.01)
    confirm(nav, .3)
    before = len(logs)
    for i, flags in enumerate((link.IMU_GYRO_MISSED, link.IMU_ACCEL_MISSED,
                               link.IMU_GYRO_MISSED | link.IMU_ACCEL_MISSED), start=31):
        acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
        msg = acc.snapshot(i*10000)
        msg.flags |= flags
        nav.on_imu(msg, i*.01)
        assert nav.mode == 'tracking' and nav.filter is not None
    assert not any(r.get('event') == 'lost' for r in logs[before:])
    assert nav.nav_message(330000).flags & FLAG_TRN_VALID
    # The flags carry no gap length; a long flagged interval may hide a stall.
    for _ in range(5):
        acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
    msg = acc.snapshot(380000)
    msg.flags |= link.IMU_GYRO_MISSED
    nav.on_imu(msg, .38)
    assert nav.mode == 'lost' and nav.filter is None
    acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
    nav.on_imu(acc.snapshot(390000), .39)       # the flagged snapshot is the new baseline
    assert nav._imus and nav._imus[-1].start == pytest.approx(.38)
