"""GPL-3.0-only. Deterministic Pi navigation replay with real rendered matching.

Synthetic host evidence only: flat procedural farmland, level constant-speed
descent, candidate camera calibration, no sensor bias or real target timing.
Acquisition uses the actual two-image Acquirer/firstfix path, never injected
poses. Images arrive 120 ms late; link bytes cross Encoder/Decoder and ticks
cross TickClock. Textureless images demonstrate rejection/loss, followed by
an explicit STM32 restart. GNSS is optional and logged separately.
"""
import argparse
from functools import lru_cache
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'code'))
from cansat import link, sitemap
from cansat.atmosphere import pressure_at
from cansat.coarse import bin_camera
from cansat.eskf import Camera, LEVEL
from cansat.navigator import Navigator, NavigatorConfig
from cansat.telemetry import FLAG_TRN_VALID
from scene import Pyramid, render


@lru_cache(maxsize=2)
def prepared_site(seed):
    full = Camera(724.6, 724.6, 728., 544., 1456, 1088, lever=np.array([0., 0., .045]))
    return sitemap.from_farmland(seed, camera=bin_camera(full, 4))


def run_replay(*, with_gnss=True, seed=20261026, output=None):
    site = prepared_site(seed)
    records = []
    cfg = NavigatorConfig(coast_after_s=.4, lost_after_s=1., history_seconds=2.)
    nav = Navigator(cfg, site, records.append)
    encoder, decoder, clock = link.Encoder(), link.Decoder(), link.TickClock()
    accumulator = link.ImuAccumulator()
    pyramid = Pyramid(site.raster, site.resolution_m, site.raster_origin)
    count = 0

    def deliver(message):
        nonlocal count
        data = encoder.encode(message)
        # Deliberately fragmented transport, not direct message injection.
        messages = decoder.feed(data[:7]) + decoder.feed(data[7:])
        for msg in messages:
            count += 1
            t = clock.update(msg.tick)/link.TICK_HZ
            handler = {link.Imu: nav.on_imu, link.Baro: nav.on_baro,
                       link.Status: nav.on_status, link.Gnss: nav.on_gnss}.get(type(msg))
            if handler:
                handler(msg, t)

    deliver(link.Status(tick=0, epoch=0, state=2, baseline_pa=101325.,
        health=link.HEALTH_BASELINE | link.HEALTH_ORIGIN, origin_lat_deg=site.pad_origin[0],
        origin_lon_deg=site.pad_origin[1], origin_h_msl_m=site.pad_origin[2], altitude_m=300.))
    pending = []
    errors = []
    final = None
    loss = False
    for i in range(801):
        t = i*.01
        tick = round(t*link.TICK_HZ)
        if i:
            accumulator.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
        deliver(accumulator.snapshot(tick))
        height = 300. - 8.*t
        if i % 10 == 0:
            deliver(link.Baro(tick=tick, pressure_pa=pressure_at(height, 101325., cfg.mean_temperature_k),
                              temperature_c=10., n_samples=1, flags=1))
        if with_gnss and i % 20 == 0:
            # Independent labelled comparison offset [assumed] .3 m east/.2 m north.
            lat = site.pad_origin[0] + (t*-.15 + .2)/111000.
            lon = site.pad_origin[1] + (t*.35 + .3)/(111000.*np.cos(np.radians(site.pad_origin[0])))
            deliver(link.Gnss(tick=tick, fix_type=3, fix_flags=1, lat_deg=lat, lon_deg=lon,
                h_msl_m=site.pad_origin[2] + height, h_acc_m=.8, v_acc_m=1.5,
                vel_e=.35, vel_n=-.15, vel_d=8., s_acc=.15))
        if i >= 50 and i % 50 == 0:
            if t <= 4.5 or t >= 6.:
                image, _ = render(pyramid, site.camera, np.array([.35*t, -.15*t, height]),
                                  LEVEL, (site.camera.height, site.camera.width), quantise=True)
                image = image.astype(np.uint8)
            else:
                image = np.full((site.camera.height, site.camera.width), 127, dtype=np.uint8)
            pending.append((t + .12, t, i//50, image))
        while pending and pending[0][0] <= t + 1e-9:
            _, exposure, frame, image = pending.pop(0)
            nav.on_frame(image, exposure, frame)
        message = nav.nav_message(tick)
        if message is not None and message.flags & FLAG_TRN_VALID:
            truth = np.array([.35*t, -.15*t, height])
            errors.append(float(np.linalg.norm(nav.filter.p[:2] - truth[:2])))
            final = nav.filter.p.tolist()
            # Output remains encodable on the real protocol path.
            link.encode(message)
        loss |= nav.mode == 'lost'
    before_restart = len(records)
    nav.restart()
    summary = dict(evidence_kind='synthetic host replay', seed=seed,
        gnss_enabled=with_gnss, image_delay_s=.12,
        acquisition_confirmed=any(r['kind'] == 'acq' and r['state'] == 'confirmed' for r in records),
        confirmed_acquisitions=sum(r['kind'] == 'acq' and r['state'] == 'confirmed' for r in records),
        tracking_updates=sum(r['kind'] == 'fix' and r['accepted'] >= cfg.min_landmarks for r in records),
        rejected_tracking_frames=sum(r['kind'] == 'fix' and r['accepted'] < cfg.min_landmarks for r in records),
        gnss_predictions=sum(r['kind'] == 'gnss_pred' for r in records),
        comparisons=sum(r['kind'] == 'comparison' for r in records),
        max_horizontal_error_m=max(errors) if errors else None, final_trn_position=final,
        loss_seen=loss, restart_seen=any(r.get('event') == 'restart' for r in records[before_restart:]),
        restart_mode=nav.mode, restart_has_nav=nav.nav_message(0) is not None,
        wire_messages=count, decoder_errors=sum(decoder.stats()[name] for name in (
            'too_long', 'stuffing', 'short', 'crc', 'length', 'faults')),
        limitations=['synthetic flat-map replay', 'no real imagery', 'no Pi or STM32 hardware timing'])
    if output is not None:
        Path(output).write_text(json.dumps(dict(summary=summary, records=records), indent=2, allow_nan=False),
                                encoding='utf-8')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gnss', action='store_true')
    parser.add_argument('--seed', type=int, default=20261026)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(json.dumps(run_replay(with_gnss=not args.no_gnss, seed=args.seed, output=args.output), indent=2))
