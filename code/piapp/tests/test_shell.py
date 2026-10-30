"""GPL-3.0-only. Host acceptance tests; no physical devices are exercised."""
import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from cansat import link
from cansat.flight_state import State


def wait_for(predicate, timeout=3.):
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, 'worker did not reach expected state'
        time.sleep(.005)


class FakeNavigator:
    def __init__(self):
        self.calls = []
        self.threads = set()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.block = False
        self.mode = 'tracking'
        self.pi_flags = link.PIFLAG_MAP | link.PIFLAG_NAV
        self.last_tick = None

    def record(self, *args):
        self.threads.add(threading.get_ident())
        self.calls.append(args)

    def on_imu(self, msg, t):
        self.record('imu', t)
        self.last_tick = msg.tick

    def on_baro(self, msg, t):
        self.record('baro', t)

    def on_status(self, msg, t):
        self.record('status', t)

    def on_gnss(self, msg, t):
        self.record('gnss', t)

    def on_frame(self, image, t, frame):
        self.record('frame', frame, t)
        self.entered.set()
        if self.block:
            assert self.release.wait(5.)

    def restart(self):
        self.record('restart')
        self.last_tick = None

    def wants_frames(self):
        self.threads.add(threading.get_ident())
        return True

    def nav_message(self, tick):
        self.threads.add(threading.get_ident())
        return None if self.last_tick is None else link.Nav(tick=self.last_tick, flags=1)


class FakeSerial:
    def __init__(self):
        self.rx = bytearray()
        self.tx = bytearray()
        self.closed = False
        self.write_threads = set()
        self.partial = 7

    def read(self, size):
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data):
        self.write_threads.add(threading.get_ident())
        n = min(len(data), self.partial)
        self.tx.extend(data[:n])
        return n

    def drain(self, timeout):
        return True

    def close(self):
        self.closed = True


class ManualCamera:
    ready = False
    error = None

    def start(self, callback):
        self.callback = callback
        self.ready = True

    def emit(self, image, local_s):
        self.callback(image, local_s)

    def stop(self):
        self.ready = False


class MemoryWriter:
    def __init__(self):
        self.records = []
        self.frames = []
        self.raw = []
        self.healthy = True
        self.closed = False
        self.dropped = 0

    def start(self):
        pass

    def record(self, stream, data):
        if not self.closed:
            self.records.append((stream, data))
        return True

    def raw_bytes(self, direction, data):
        self.raw.append((direction, data))
        return True

    def frame(self, image, **meta):
        self.frames.append((image.copy(), meta))
        return True

    def close(self, timeout=5.):
        self.closed = True
        return self.healthy


def make_service(**kwargs):
    from piapp.config import Config
    from piapp.service import Service
    nav, serial, camera, writer = FakeNavigator(), FakeSerial(), ManualCamera(), MemoryWriter()
    clock = [100.]
    hooks = []
    config = Config(**kwargs)
    service = Service(config, serial, camera, nav, writer,
                      monotonic=lambda: clock[0], poweroff=lambda: hooks.append('poweroff'),
                      health=lambda: (42, 25, 0))
    service.start()
    wait_for(lambda: camera.ready)
    return service, nav, serial, camera, writer, clock, hooks


def receive(service, serial, encoder, *messages):
    serial.rx.extend(b''.join(encoder.encode(m) for m in messages))
    service.step()


def sent(serial):
    return link.Decoder().feed(serial.tx)


def test_config_strict_and_relative_paths(tmp_path):
    from piapp.config import load
    path = tmp_path / 'flight.toml'
    path.write_text('[service]\nmessage_capacity=12\nsite="site.npz"\ndata_dir="data"\n'
                    '[camera]\nkind="synthetic"\nwidth=32\nheight=24\n'
                    '[navigator]\nmin_height_m=90.0\n')
    config = load(path)
    assert config.message_capacity == 12
    assert config.site == tmp_path / 'site.npz'
    assert config.data_dir == tmp_path / 'data'
    assert config.camera.width == 32
    assert config.navigator['min_height_m'] == 90.
    for content in ('[service]\nmessage_capacity=0', '[service]\ntypo=1',
                    '[camera]\nkind="wrong"', '[service]\nframe_latency_s=nan'):
        path.write_text(content)
        with pytest.raises(ValueError):
            load(path)


def test_ordered_worker_newest_frame_restart_and_no_cross_thread_calls():
    from piapp.worker import NavigatorWorker
    nav = FakeNavigator()
    nav.block = True
    worker = NavigatorWorker(nav, capacity=8)
    worker.start()
    try:
        worker.submit_message(link.Imu(tick=10), .00001)
        worker.submit_frame(np.zeros((4, 4), np.uint8), .00001, 1)
        assert nav.entered.wait(2.)
        for i in range(2, 7):
            worker.submit_message(link.Imu(tick=i * 10), i * .00001)
            worker.submit_frame(np.zeros((4, 4), np.uint8), i * .00001, i)
        nav.block = False
        nav.release.set()
        wait_for(lambda: ('frame', 6, 6 * .00001) in nav.calls)
        assert [c[1] for c in nav.calls if c[0] == 'frame'] == [1, 6]
        assert [c[1] for c in nav.calls if c[0] == 'imu'] == pytest.approx([i * .00001 for i in range(1, 7)])
        assert worker.frames_dropped == 4
        worker.restart()
        wait_for(lambda: ('restart',) in nav.calls)
        assert worker.snapshot().nav is None
        assert len(nav.threads) == 1
    finally:
        nav.release.set()
        assert worker.stop(2.)


def test_bounded_history_overflow_breaks_inertial_continuity():
    from piapp.worker import NavigatorWorker
    nav = FakeNavigator()
    nav.block = True
    worker = NavigatorWorker(nav, capacity=3)
    worker.start()
    try:
        worker.submit_frame(np.zeros((2, 2), np.uint8), 0., 1)
        assert nav.entered.wait(2.)
        for i in range(10):
            worker.submit_message(link.Imu(tick=i), i / 1e6)
        assert worker.pending_messages <= 3
        assert worker.messages_dropped > 0
        assert worker.snapshot().nav is None
        nav.block = False
        nav.release.set()
        wait_for(lambda: ('imu', 9 / 1e6) in nav.calls)
        assert ('restart',) in nav.calls
        assert worker.overflows > 0
    finally:
        nav.release.set()
        assert worker.stop(2.)


def test_heartbeat_controls_continue_during_acquisition_and_partial_writes():
    s, nav, serial, camera, writer, clock, hooks = make_service(capture_on_pad=True)
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Hello(tick=1_000_000), link.Imu(tick=1_001_000))
        wait_for(lambda: nav.last_tick is not None)
        nav.block = True
        s.worker.submit_frame(np.zeros((4, 4), np.uint8), 1., 7)
        assert nav.entered.wait(2.)
        for _ in range(41):
            s.step()
            clock[0] += .05
        messages = sent(serial)
        statuses = [m for m in messages if isinstance(m, link.PiStatus)]
        assert len(statuses) >= 4
        assert statuses[-1].tick_echo == 1_001_000
        assert statuses[-1].echo_age_ms >= 1500
        assert any(isinstance(m, link.TrigCfg) and m.enable for m in messages)
        assert not [m for m in messages if isinstance(m, link.Nav) and m.tick == 0]
        assert len(serial.write_threads) == 1
    finally:
        nav.release.set()
        s.close()


def test_restart_same_chunk_epoch_tick_wrap_and_delayed_frame_stamp():
    s, nav, serial, camera, writer, clock, hooks = make_service(capture_on_pad=True)
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Hello(epoch=2, tick=0xFFFFFF00),
                link.Imu(epoch=2, tick=0xFFFFFFF0), link.Imu(epoch=2, tick=0x100))
        wait_for(lambda: len([x for x in nav.calls if x[0] == 'imu']) == 2)
        imus = [x[1] for x in nav.calls if x[0] == 'imu']
        assert imus[1] > imus[0] > 4000
        clock[0] = 101.
        receive(s, serial, enc, link.Hello(epoch=3, tick=1_000_000),
                link.Status(epoch=3, tick=1_000_000, state=State.DESCENT),
                link.Trig(frame=10, tick=1_000_000, period_us=200_000, pulse_us=3000, flags=1),
                link.Imu(epoch=3, tick=1_010_000))
        camera.emit(np.zeros((1088, 1456), np.uint8), 101.02)
        clock[0] = 101.10  # dequeued late: use camera's receive time
        s.step()
        wait_for(lambda: any(x[0] == 'frame' for x in nav.calls))
        frame = [x for x in nav.calls if x[0] == 'frame'][-1]
        assert frame[1] == 10
        assert frame[2] == pytest.approx(1.00150713)
        assert writer.frames[-1][1]['generation'] == s.generation
        assert s.decoder.lost == 0
        assert nav.calls.count(('restart',)) >= 1
        # Epoch and restart flag both invalidate timestamps and pending output.
        receive(s, serial, enc, link.Imu(epoch=4, flags=link.IMU_RESTART, tick=100))
        assert s.stamper.last is None
        assert s.worker.snapshot().nav is None
    finally:
        s.close()


def test_duplicate_prepare_landed_policy_and_shutdown_order():
    s, nav, serial, camera, writer, clock, hooks = make_service(landed_capture_s=1.)
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Cmd(cmd_seq=1, cmd=link.CMD_PREPARE))
        receive(s, serial, enc, link.Cmd(cmd_seq=1, cmd=link.CMD_PREPARE))
        assert s.trigger_enabled
        clock[0] += .1
        receive(s, serial, enc, link.Status(state=State.LANDED))
        clock[0] += 1.1
        s.step()
        assert not s.trigger_enabled
        command = link.Cmd(cmd_seq=2, cmd=link.CMD_SHUTDOWN, arg=10)
        receive(s, serial, enc, command, command)
        assert s.stopping
        wait_for(lambda: s.shutdown_finished.is_set())
        for _ in range(20):
            s.step()
        acks = [m for m in sent(serial) if isinstance(m, link.Ack)]
        results = [m.result for m in acks if m.cmd_seq == 2]
        assert results[0] == link.ACK_ACCEPTED
        assert results[-1] == link.ACK_DONE
        assert writer.closed and not camera.ready and not s.worker.alive
        assert hooks == ['poweroff']
    finally:
        s.close()


def test_storage_fault_does_not_claim_clean_shutdown():
    s, nav, serial, camera, writer, clock, hooks = make_service()
    try:
        writer.healthy = False
        receive(s, serial, link.Encoder(), link.Cmd(cmd=link.CMD_SHUTDOWN, arg=2))
        wait_for(lambda: s.shutdown_finished.is_set())
        for _ in range(20):
            s.step()
        assert not any(isinstance(m, link.Ack) and m.result == link.ACK_DONE for m in sent(serial))
        assert not hooks
    finally:
        s.close()


def test_writer_sessions_raw_json_frames_and_faults(tmp_path):
    from piapp.storage import Writer
    w = Writer(tmp_path, capacity=8, sync_interval_s=.1)
    w.start()
    assert w.raw_bytes('rx', b'\x01\x00')
    assert w.record('navigator', {'kind': 'nav', 'value': np.array([1., np.nan])})
    assert w.frame(np.arange(12, dtype=np.uint16).reshape(3, 4),
                   tick=42.5, frame=1, generation=2, local_s=3.)
    assert w.close(3.)
    assert (w.session_dir / 'raw_rx.bin').read_bytes() == b'\x01\x00'
    record = json.loads((w.session_dir / 'navigator.jsonl').read_text())
    assert record['value'] == [1., None]
    index = json.loads((w.session_dir / 'frames.jsonl').read_text())
    assert index['tick'] == 42.5 and index['generation'] == 2
    assert np.load(w.session_dir / index['file']).dtype == np.uint16
    other = Writer(tmp_path)
    assert other.session_dir != w.session_dir
    other.start()
    assert other.close()
    bad = tmp_path / 'file'
    bad.write_text('preserve')
    fault = Writer(bad)
    fault.start()
    wait_for(lambda: not fault.healthy)
    assert not fault.close()
    assert bad.read_text() == 'preserve'


def test_writer_capacity_never_blocks_producer(tmp_path):
    from piapp.storage import Writer
    w = Writer(tmp_path, capacity=1, record_capacity=3)
    # Queue before start to deterministically represent an unavailable SD writer.
    assert w.record('events', {'n': 1})
    assert w.record('events', {'n': 2})
    image = np.zeros((2, 2), dtype=np.uint8)
    assert w.frame(image, tick=1, frame=1, generation=0, local_s=0.)
    assert not w.frame(image, tick=2, frame=2, generation=0, local_s=0.)
    # Pending frames have their own budget: they never evict small records.
    assert w.record('events', {'n': 3})
    assert not w.record('events', {'n': 4})
    assert w.dropped == 2
    w.start()
    assert w.close()
    assert len((w.session_dir / 'events.jsonl').read_text().splitlines()) == 3 + 1  # + storage_closed
    assert len((w.session_dir / 'frames.jsonl').read_text().splitlines()) == 1


def test_camera_synthetic_and_replay_lazy_hardware(tmp_path):
    from piapp.camera import ReplayCamera, SyntheticCamera
    frames = []
    c = SyntheticCamera(width=16, height=12, period_s=.01)
    c.start(lambda image, t: frames.append((image, t)))
    wait_for(lambda: len(frames) >= 2)
    c.stop()
    assert frames[0][0].shape == (12, 16)
    assert frames[0][0].dtype == np.uint8
    image = np.arange(192, dtype=np.uint8).reshape(12, 16)
    np.save(tmp_path / 'image.npy', image)
    (tmp_path / 'frames.jsonl').write_text(json.dumps({'file': 'image.npy', 'local_s': 1.}) + '\n')
    replay = ReplayCamera(tmp_path / 'frames.jsonl')
    out = []
    replay.start(lambda image, t: out.append(image))
    wait_for(lambda: len(out) == 1)
    replay.stop()
    assert np.array_equal(out[0], image)


def test_late_acquisition_result_cannot_refresh_old_nav_age():
    s, nav, serial, camera, writer, clock, hooks = make_service()
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Imu(tick=1_000_000))
        wait_for(lambda: s.worker.snapshot().nav is not None)
        nav.block = True
        s.worker.submit_frame(np.zeros((4, 4), np.uint8), 1., 1)
        assert nav.entered.wait(2.)
        clock[0] += 3.
        nav.block = False
        nav.release.set()
        wait_for(lambda: s.worker.snapshot().published_s == clock[0])
        serial.tx.clear()
        s.step()
        assert not any(isinstance(m, link.Nav) and m.flags for m in sent(serial))
    finally:
        nav.release.set()
        s.close()


def test_trigger_disable_rejects_late_frames_and_unknown_command_ack():
    s, nav, serial, camera, writer, clock, hooks = make_service(capture_on_pad=True, landed_capture_s=0.)
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Hello(tick=1_000_000),
                link.Trig(frame=0, tick=1_000_000, period_us=200_000, pulse_us=3000))
        receive(s, serial, enc, link.Status(tick=1_010_000, state=State.LANDED),
                link.Cmd(cmd_seq=99, cmd=99))
        assert not s.trigger_enabled
        camera.emit(np.zeros((1088, 1456), np.uint8), 100.02)
        clock[0] += .3
        s.step()
        assert not writer.frames
        assert any(isinstance(m, link.Ack) and m.result == link.ACK_UNKNOWN for m in sent(serial))
    finally:
        s.close()


def test_cli_help_and_config_check(tmp_path):
    import os
    import subprocess
    import sys
    env = {**os.environ, 'PYTHONPATH': str(Path(__file__).resolve().parents[2])}
    result = subprocess.run([sys.executable, '-m', 'piapp', '--help'], env=env,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    config = tmp_path / 'flight.toml'
    config.write_text('[camera]\nkind="synthetic"\n')
    result = subprocess.run([sys.executable, '-m', 'piapp', '--config', str(config), '--check-config'],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'valid' in result.stdout


def test_shutdown_timeout_keeps_heartbeat_and_withholds_done():
    s, nav, serial, camera, writer, clock, hooks = make_service(shutdown_timeout_s=.1)
    try:
        nav.block = True
        s.worker.submit_frame(np.zeros((4, 4), np.uint8), 0., 1)
        assert nav.entered.wait(2.)
        receive(s, serial, link.Encoder(), link.Cmd(cmd=link.CMD_SHUTDOWN, arg=1))
        wait_for(lambda: s.shutdown_finished.is_set())
        clock[0] += .6
        s.step()
        messages = sent(serial)
        assert any(isinstance(m, link.PiStatus) and m.state == link.PI_STOPPING for m in messages)
        assert not any(isinstance(m, link.Ack) and m.result == link.ACK_DONE for m in messages)
        assert not hooks
    finally:
        nav.release.set()
        s.worker.stop(2.)
        s.close()


def test_delayed_trigger_and_duplicate_capture_are_not_misstamped():
    s, nav, serial, camera, writer, clock, hooks = make_service(capture_on_pad=True)
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Hello(tick=1_000_000))
        camera.emit(np.zeros((1088, 1456), np.uint8), 100.02)
        clock[0] = 100.02
        s.step()
        assert not writer.frames
        clock[0] = 100.03
        receive(s, serial, enc, link.Trig(frame=3, tick=1_000_000, period_us=200_000, pulse_us=3000))
        assert writer.frames[0][1]['frame'] == 3
        # Different callback, same exposure bucket: it must be rejected.
        camera.emit(np.zeros((1088, 1456), np.uint8), 100.021)
        s.step()
        assert len(writer.frames) == 1
        assert s.frames_refused >= 1
    finally:
        s.close()


def test_writer_midstream_sync_failure_stays_unhealthy(tmp_path, monkeypatch):
    from piapp.storage import Writer
    import piapp.storage as storage
    w = Writer(tmp_path)
    w.start()
    wait_for(lambda: w.session_dir.exists())

    def fail_sync(fd):
        raise OSError('injected full/faulty SD')

    monkeypatch.setattr(storage.os, 'fsync', fail_sync)
    w.frame(np.zeros((2, 2), np.uint8), generation=0, frame=1, tick=0., local_s=0.)
    wait_for(lambda: not w.healthy)
    assert not w.close()
    assert 'injected full/faulty SD' in w.error
    assert list(w.session_dir.glob('*.part'))  # preserve failed raw writes for recovery


def test_real_navigator_service_writer_and_cli_replay(tmp_path):
    import os
    import subprocess
    import sys
    from cansat import sitemap
    from cansat.eskf import Camera
    from cansat.navigator import Navigator, NavigatorConfig
    from cansat.windprofile import WindModel
    from piapp.config import CameraConfig, Config
    from piapp.service import Service
    from piapp.storage import Writer
    site = sitemap.Site(np.random.default_rng(3).uniform(0, 255, (128, 128)),
                        1., (-64., -64.), (22., 114., 15.), 'synthetic MSL flat ground',
                        Camera(100., 100., 79.5, 59.5, 160, 120), coarse_n1=32,
                        wind_model=WindModel(max_substeps=8), table_top_m=40., table_step_m=10.)
    sitemap.save(tmp_path / 'site.npz', site)
    writer = Writer(tmp_path / 'sessions', capacity=128)
    nav = Navigator(NavigatorConfig(min_height_m=5., max_height_m=40.), site,
                    log=lambda record: writer.record('navigator', record))
    serial, camera = FakeSerial(), ManualCamera()
    local = [100.]
    config = Config(camera=CameraConfig(kind='synthetic', width=160, height=120), capture_on_pad=True)
    service = Service(config, serial, camera, nav, writer, monotonic=lambda: local[0])
    service.start()
    wait_for(lambda: camera.ready)
    enc = link.Encoder()
    accumulator = link.ImuAccumulator()
    receive(service, serial, enc, link.Hello(),
            link.Status(state=State.DESCENT, baseline_pa=101325.,
                        health=link.HEALTH_BASELINE | link.HEALTH_ORIGIN,
                        origin_lat_deg=22., origin_lon_deg=114., origin_h_msl_m=15.))
    for i in range(11):
        tick = i * 10_000
        local[0] = 100. + i * .01
        accumulator.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
        receive(service, serial, enc, accumulator.snapshot(tick),
                link.Baro(tick=tick, pressure_pa=100970., temperature_c=10., flags=1))
    wait_for(lambda: service.worker.pending_messages == 0)
    receive(service, serial, enc, link.Trig(frame=1, tick=70_000, period_us=200_000, pulse_us=3000))
    camera.emit(np.zeros((120, 160), np.uint8), 100.09)
    local[0] = 100.11
    service.step()
    wait_for(lambda: service.worker.snapshot().mode == 'acquiring')
    assert service.close()
    rows = [json.loads(line) for line in (writer.session_dir / 'frames.jsonl').read_text().splitlines()]
    assert len(rows) == 1 and rows[0]['frame'] == 1
    assert np.array_equal(np.load(writer.session_dir / rows[0]['file']), np.zeros((120, 160), np.uint8))
    assert any(json.loads(line)['kind'] == 'acq' for line in
               (writer.session_dir / 'navigator.jsonl').read_text().splitlines())
    raw_path = tmp_path / 'rx.bin'
    raw_path.write_bytes(link.encode(link.Hello()) + link.encode(link.Status(), seq=1))
    config_path = tmp_path / 'bench.toml'
    config_path.write_text('[service]\nsite="site.npz"\ndata_dir="cli-sessions"\n'
                           '[camera]\nkind="synthetic"\nwidth=160\nheight=120\n')
    env = {**os.environ, 'PYTHONPATH': str(Path(__file__).resolve().parents[2])}
    result = subprocess.run([sys.executable, '-m', 'piapp', '--config', str(config_path),
                             '--serial-replay', str(raw_path), '--duration', '.15', '--no-poweroff'],
                            env=env, capture_output=True, text=True, timeout=15.)
    assert result.returncode == 0, result.stderr
    sessions = list((tmp_path / 'cli-sessions').iterdir())
    assert len(sessions) == 1
    assert (sessions[0] / 'raw_rx.bin').read_bytes() == raw_path.read_bytes()
    assert (sessions[0] / 'raw_tx.bin').stat().st_size > 0


def test_same_epoch_hello_reappearance_preserves_live_navigation_and_triggers():
    s, nav, serial, camera, writer, clock, hooks = make_service(capture_on_pad=True)
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Hello(epoch=7, tick=1_000_000),
                link.Status(epoch=7, tick=1_000_000, state=State.DESCENT),
                link.Imu(epoch=7, tick=1_010_000),
                link.Trig(frame=5, tick=1_000_000, period_us=200_000, pulse_us=3000))
        wait_for(lambda: s.worker.snapshot().nav is not None)
        generation = s.generation
        trigger = s.stamper.last
        restarts = nav.calls.count(('restart',))
        clock[0] += .02
        receive(s, serial, enc, link.Hello(epoch=7, tick=1_020_000))
        assert s.generation == generation
        assert s.stamper.last == trigger
        assert s.worker.snapshot().nav is not None
        assert nav.calls.count(('restart',)) == restarts
        # A subsequent integral restart is independent reset evidence even
        # without a boot-epoch change.
        receive(s, serial, enc, link.Imu(epoch=7, tick=1_030_000, flags=link.IMU_RESTART))
        assert s.generation == generation + 1
        generation = s.generation
        # A new epoch remains definitive even if the low tick happens to grow.
        receive(s, serial, enc, link.Hello(epoch=8, tick=1_040_000))
        assert s.generation == generation + 1
        assert s.stamper.last is None
    finally:
        s.close()


def test_heartbeat_echoes_current_boot_epoch_in_reserved_flags():
    s, nav, serial, camera, writer, clock, hooks = make_service()
    enc = link.Encoder()
    try:
        s.step()
        first = [m for m in sent(serial) if isinstance(m, link.PiStatus)][-1]
        assert not first.flags & link.PIFLAG_EPOCH_ECHO
        receive(s, serial, enc, link.Hello(epoch=173, tick=1000000,
                                          flags=link.HELLO_REQUIRE_EPOCH_ECHO))
        current = [m for m in sent(serial) if isinstance(m, link.PiStatus)][-1]
        assert current.flags & link.PIFLAG_EPOCH_ECHO
        assert current.flags >> link.PIFLAG_EPOCH_SHIFT == 173
        assert current.sw_version == 2
        clock[0] += .01
        receive(s, serial, enc, link.Hello(epoch=174, tick=10000))
        changed = [m for m in sent(serial) if isinstance(m, link.PiStatus)][-1]
        assert changed.flags >> link.PIFLAG_EPOCH_SHIFT == 174
    finally:
        s.close()


def test_queued_nav_expires_while_serial_cannot_write():
    s, nav, serial, camera, writer, clock, hooks = make_service()
    enc = link.Encoder()
    try:
        receive(s, serial, enc, link.Imu(tick=1_000_000))
        wait_for(lambda: s.worker.snapshot().nav is not None)
        # Reach next Nav publish with no UART progress; later unblocking must
        # not publish that expired solution.
        serial.partial = 0
        clock[0] += .21
        s.step()
        clock[0] += 2.
        serial.partial = 7
        serial.tx.clear()
        s.step()
        assert not any(isinstance(m, link.Nav) and m.flags for m in sent(serial))
    finally:
        s.close()


def test_shutdown_timeout_still_closes_storage_best_effort():
    s, nav, serial, camera, writer, clock, hooks = make_service(shutdown_timeout_s=.1)
    try:
        nav.block = True
        s.worker.submit_frame(np.zeros((4, 4), np.uint8), 0., 1)
        assert nav.entered.wait(2.)
        receive(s, serial, link.Encoder(), link.Cmd(cmd=link.CMD_SHUTDOWN, arg=1))
        wait_for(lambda: s.shutdown_finished.is_set())
        assert writer.closed
        assert not s.shutdown_ok
    finally:
        nav.release.set()
        s.worker.stop(2.)
        s.close()


def test_camera_start_failure_is_visible_to_cli_and_status():
    from piapp.config import Config
    from piapp.service import Service

    class FailedCamera(ManualCamera):
        def start(self, callback):
            raise OSError('injected camera unavailable')

    camera = FailedCamera()
    service = Service(Config(capture_on_pad=True), FakeSerial(), camera,
                      FakeNavigator(), MemoryWriter())
    service.start()
    try:
        wait_for(lambda: not service._camera_thread.is_alive())
        service.step()
        assert service.fault and 'camera unavailable' in service.fault
        assert not service.trigger_enabled
        assert any(isinstance(m, link.PiStatus) and m.state == link.PI_FAULT
                   for m in sent(service.serial))
    finally:
        service.close()


def test_transient_serial_fault_recovers_after_hold():
    from piapp.service import SERIAL_FAULT_HOLD_S
    s, nav, serial, camera, writer, clock, hooks = make_service(capture_on_pad=True)
    failures, read = [2], serial.read

    def flaky(size):
        if failures[0]:
            failures[0] -= 1
            raise OSError('injected transient read fault')
        return read(size)

    serial.read = flaky
    try:
        s.step()
        s.step()
        assert not s.trigger_enabled and s.serial_faults == 2
        events = [d for k, d in writer.records if k == 'events' and d['event'] == 'serial_read_fault']
        assert len(events) == 1
        clock[0] += SERIAL_FAULT_HOLD_S + .1
        s.step()
        assert s.trigger_enabled and s.fault is None
        states = [m.state for m in sent(serial) if isinstance(m, link.PiStatus)]
        assert states[0] == link.PI_FAULT and states[-1] != link.PI_FAULT
    finally:
        s.close()
