"""GPL-3.0-only. C application output checked by existing Python wire oracles.

All executable outputs live under ignored build/sitl; no device required.
"""
import collections
import importlib.util
import math
import os
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT.parents[1]
sys.path.insert(0, str(CODE))
from cansat import link, telemetry


def module_at(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def outputs():
    out = ROOT / 'build/sitl'
    scenarios = ('flight', 'failures', 'reset', 'wrap', 'budget')
    if os.environ.get('CANSAT_SITL_PREBUILT') == '1':
        # verify_flight.py has just regenerated each capture and report. Avoid
        # rebuilding/rerunning the same scenarios in this combined gate.
        for name in scenarios:
            assert all((out / f'{name}.{kind}.bin').is_file() for kind in ('link', 'radio', 'flash'))
        return out, {name: (out / f'{name}.txt').read_text(encoding='utf-8') for name in scenarios}
    build = module_at('app_sitl_build', ROOT / 'sitl/build.py')
    binary = build.build()
    reports = {}
    for scenario in scenarios:
        result = subprocess.run([str(binary), scenario, str(out)], capture_output=True, text=True, check=True)
        (out / f'{scenario}.txt').write_text(result.stdout, encoding='utf-8')
        reports[scenario] = result.stdout
    return out, reports


def messages(raw):
    decoder = link.Decoder()
    result = []
    # Small chunks exercise streaming, including split COBS/CRC boundaries.
    for i in range(0, len(raw), 137):
        result.extend(decoder.feed(raw[i:i+137]))
    assert decoder.crc == decoder.length == decoder.stuffing == decoder.faults == 0
    return result


@pytest.mark.parametrize('scenario', ['flight', 'failures', 'reset', 'wrap', 'budget'])
def test_python_wire_and_flash_oracles(outputs, scenario):
    out, reports = outputs
    msgs = messages((out / f'{scenario}.link.bin').read_bytes())
    assert len(msgs) > 1000
    statuses = [m for m in msgs if isinstance(m, link.Status)]
    assert {m.state for m in statuses} == {0, 1, 2, 3}
    radio = (out / f'{scenario}.radio.bin').read_bytes()
    assert len(radio) % 40 == 0
    packets = [telemetry.decode(radio[i:i+40]) for i in range(0, len(radio), 40)]
    assert packets and all(p.state in range(4) for p in packets)
    assert any(p.state == 1 for p in packets) and any(p.state == 2 for p in packets)
    boot_count = 2 if scenario == 'reset' else 1
    assert sum(math.isnan(p.pressure_pa) for p in packets if p.state == 0) <= boot_count
    assert all(100900 < p.pressure_pa < 101600 for p in packets if p.state == 0 and math.isfinite(p.pressure_pa))
    if scenario != 'failures':
        assert any(p.flags & link.FLAG_TRN_VALID for p in packets)
        assert sum(math.isnan(p.temperature_c) for p in packets) <= boot_count
        assert any(isinstance(m, link.Trig) and m.period_us == 200000 and m.pulse_us == 100 for m in msgs)
        trigs = [m for m in msgs if isinstance(m, link.Trig)]
        assert sum(bool(m.flags & 1) for m in trigs) == (2 if scenario == 'reset' else 1)
    else:
        assert all(not (p.flags & link.FLAG_TRN_VALID) for p in packets)
        assert any(math.isnan(p.pressure_pa) for p in packets if p.state == 2)
    dump = module_at('app_log_dump', ROOT / 'tools/flashlog_dump.py')
    records, pages, errors = dump.decode_image((out / f'{scenario}.flash.bin').read_bytes())
    assert not errors
    assert pages and records
    by_epoch = collections.defaultdict(list)
    for r in records:
        by_epoch[r['epoch']].append(r['tick_us'])
    assert all(ticks == sorted(ticks) for ticks in by_epoch.values())
    assert {r['type'] for r in records} >= {0x10, 0x11, 0x15, 0x30, 0x31, 0x32}
    assert 'log_drop=0' in reports[scenario]
    assert 'nor_errors=0 power_errors=0' in reports[scenario]
    if scenario == 'reset':
        assert len(by_epoch) == 2
        assert len({m.epoch for m in msgs if isinstance(m, link.Imu)}) == 2
        assert 'restored=1' in reports[scenario]
        assert 'unavailable=0' in reports[scenario]


def test_primary_radio_cadence_and_nav_age(outputs):
    out, _ = outputs
    raw = (out / 'flight.radio.bin').read_bytes()
    packets = [telemetry.decode(raw[i:i+40]) for i in range(0, len(raw), 40)]
    inflight = [p for p in packets if p.state in (1, 2)]
    assert all(999 <= b.t_ms-a.t_ms <= 1001 for a, b in zip(inflight, inflight[1:]))
    assert all(math.isfinite(p.pressure_pa) and math.isfinite(p.temperature_c) for p in inflight)
    assert not any(p.flags & link.FLAG_TRN_VALID for p in packets if p.t_ms > 460000)


def test_budget_retention_and_landed_temperature(outputs):
    out, _ = outputs
    dump = module_at('app_budget_dump', ROOT / 'tools/flashlog_dump.py')
    records, pages, errors = dump.decode_image((out / 'budget.flash.bin').read_bytes())
    assert not errors
    events = [r for r in records if r['type'] == 0x31]
    launch = next(r['tick_us'] for r in events if bytes.fromhex(r['payload_hex'])[1] == 1)
    protected = next(p['protected_start'] for p in reversed(pages) if p['protected_start'] is not None)
    boundary = next(p for p in pages if p['address'] == protected)
    assert boundary['first_tick_us'] <= launch-60000000
    assert any(r['tick_us'] <= launch-60000000 for r in records)
    assert records[-1]['tick_us'] > 3754000000  # >1 h after synthetic touchdown
    occupied = len(pages)*256
    assert occupied < 4*1024*1024  # actual record/page overhead included
    # Raw Temp envelope is explicit LE, no native C struct padding.
    temps = [bytes.fromhex(r['payload_hex']) for r in records if r['type'] == 0x30]
    assert temps and all(len(p) == 20 and p[14] == 0 for p in temps)
    assert all(struct.unpack_from('<I', p, 16)[0] == 6256 for p in temps)
    landed_imu = [r for r in records if r['type'] == 0x10 and r['tick_us'] > 160000000]
    assert len(landed_imu) <= 361  # high-rate integrals still run, logging thinned
