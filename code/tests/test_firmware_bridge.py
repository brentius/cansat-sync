"""GPL-3.0-only. Actual C application output consumed by the Python flight code.

Generate build/sitl captures with sitl/build.py and the flight executable first.
Ordinary Python-only checkouts skip these tests; verify_flight.py requires them.
All inputs are synthetic host C application output, never physical measurements.
"""
import importlib.util
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

from cansat import link, telemetry
from cansat.flight_state import State

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'code/firmware/stm32'


def artifact(name, kind):
    path = FW / 'build/sitl' / f'{name}.{kind}.bin'
    if not path.exists():
        if os.environ.get('CANSAT_REQUIRE_SITL') == '1':
            pytest.fail(f'required C SITL output missing: {path}')
        pytest.skip('build/run C SITL scenarios to enable firmware/Python bridge')
    return path


def messages(name):
    decoder = link.Decoder()
    with artifact(name, 'link').open('rb') as stream:
        while data := stream.read(1024):
            yield from decoder.feed(data)
    assert all(decoder.stats()[key] == 0 for key in (
        'too_long', 'stuffing', 'short', 'crc', 'length', 'faults'))


@pytest.mark.parametrize('name', ['flight', 'failures', 'reset', 'wrap'])
def test_c_application_wire_and_primary_radio_contract(name):
    seen = set()
    imus = 0
    epochs = set()
    triggers = []
    for message in messages(name):
        if isinstance(message, link.Status):
            seen.add(message.state)
        if isinstance(message, link.Imu):
            imus += 1
            epochs.add(message.epoch)
        if isinstance(message, link.Trig):
            triggers.append(message)
    assert seen == set(State)
    assert imus > 1000
    assert len(epochs) == (2 if name == 'reset' else 1)
    if name == 'flight':
        assert triggers and triggers[0].flags & 1
        assert any(not t.flags & 1 for t in triggers[1:])
    raw = artifact(name, 'radio').read_bytes()
    assert raw and len(raw) % telemetry.PACKET_SIZE == 0
    packets = [telemetry.decode(raw[i:i+telemetry.PACKET_SIZE])
               for i in range(0, len(raw), telemetry.PACKET_SIZE)]
    descent = [p for p in packets if p.state == State.DESCENT]
    assert len(descent) >= 10
    if name == 'failures':
        assert not any(p.flags & (telemetry.FLAG_TRN_VALID | telemetry.FLAG_GPS_VALID) for p in packets)
        assert any(math.isnan(p.temperature_c) and math.isnan(p.pressure_pa) for p in descent)
    elif name == 'reset':
        # Reset restarts converters. Missing fields during the first second
        # must be explicit NA; subsequent primary packets must recover.
        unavailable = [p for p in descent if not (math.isfinite(p.temperature_c) and math.isfinite(p.pressure_pa))]
        assert len(unavailable) <= 1 and all(p.t_ms < 1000 for p in unavailable)
    else:
        assert all(math.isfinite(p.temperature_c) and math.isfinite(p.pressure_pa) for p in descent)
    if name in ('flight', 'wrap'):
        assert all((b.t_ms-a.t_ms) % (1 << 32) in range(999, 1002)
                   for a, b in zip(descent, descent[1:]))


@pytest.mark.parametrize('name', ['flight', 'reset'])
def test_c_nor_pages_and_records_decode_with_python(name):
    spec = importlib.util.spec_from_file_location('firmware_dump_bridge', FW / 'tools/flashlog_dump.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    records, pages, errors = module.decode_image(artifact(name, 'flash').read_bytes())
    assert not errors and pages and records
    assert any(p['protected_start'] is not None for p in pages)
    epochs = {r['epoch'] for r in records}
    assert len(epochs) == (2 if name == 'reset' else 1)
    latest = {}
    for record in records:
        assert record['tick_us'] >= latest.get(record['epoch'], 0)
        latest[record['epoch']] = record['tick_us']
    assert {link.Imu.ID, link.Baro.ID, link.Status.ID, 0x30, 0x31} <= {r['type'] for r in records}


def test_real_c_sensor_stream_acquires_and_tracks_rendered_images():
    # Components and image model match the separate rendered replay, but the
    # sensor bytes here come from the complete C scheduler/integrator/codec.
    artifact('flight', 'link')
    sys.path.insert(0, str(ROOT / 'sim/nav'))
    from run_flight_replay import prepared_site
    from scene import Pyramid, render
    from cansat.eskf import LEVEL
    from cansat.navigator import Navigator, NavigatorConfig

    site = prepared_site(20261026)
    records = []
    nav = Navigator(NavigatorConfig(), site, records.append)
    clock = link.TickClock()
    pyramid = Pyramid(site.raster, site.resolution_m, site.raster_origin)
    next_frame = 126.
    frame = 0
    valid = 0
    epoch = None
    for message in messages('flight'):
        if not hasattr(message, 'tick'):
            continue
        if isinstance(message, (link.Hello, link.Status, link.Imu)):
            if epoch is not None and message.epoch != epoch:
                clock.restart()
                nav.restart()
            epoch = message.epoch
        t = clock.update(message.tick)/link.TICK_HZ
        handler = {link.Imu: nav.on_imu, link.Baro: nav.on_baro,
                   link.Status: nav.on_status}.get(type(message))
        if handler:
            handler(message, t)  # GNSS deliberately omitted from TRN branch.
        if isinstance(message, link.Imu) and next_frame <= t <= 130.1:
            exposure = next_frame - .12
            height = (155.-exposure)*10.
            image, _ = render(pyramid, site.camera, np.array([0., 0., height]), LEVEL,
                              (site.camera.height, site.camera.width), quantise=True)
            nav.on_frame(image.astype(np.uint8), exposure, frame)
            next_frame += .5
            frame += 1
        if 126 <= t <= 131:
            msg = nav.nav_message(message.tick)
            valid += msg is not None and bool(msg.flags & telemetry.FLAG_TRN_VALID)
        if t > 131:
            break
    assert any(r['kind'] == 'acq' and r['state'] == 'confirmed' for r in records)
    assert any(r['kind'] == 'fix' and r['accepted'] >= nav.config.min_landmarks for r in records)
    assert valid > 20
