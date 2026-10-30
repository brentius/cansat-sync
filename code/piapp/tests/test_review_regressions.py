"""GPL-3.0-only. Reproductions of Pi review publication and shutdown defects."""
import json
import sys
import threading
from pathlib import Path

from cansat import link
from cansat.telemetry import FLAG_TRN_VALID
from piapp.config import Config
from piapp.service import Service
from piapp.worker import NavigatorWorker
from test_shell import (FakeNavigator, FakeSerial, ManualCamera,
                        make_service, sent, wait_for)

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tests'))
from test_navigator import nav_ready, feed, confirm


def test_delayed_gnss_and_baro_preserve_actual_nav_before_next_imu(tmp_path):
    from piapp.storage import Writer

    nav, records, acc = nav_ready()
    for i in range(61):
        t = i*.01
        if i <= 40:
            feed(nav, acc, t)
        else:
            acc.add_sample((0., 0., 0.), (0., 0., -9.80665), .01)
            nav.on_imu(acc.snapshot(i*10000), t)
        if i == 30:
            confirm(nav, .3)
    callbacks = []
    for name in ('on_gnss', 'on_baro'):
        original = getattr(nav, name)
        def capture(message, t, original=original, name=name):
            callbacks.append((name, t))
            original(message, t)
        setattr(nav, name, capture)
    serial, clock = FakeSerial(), [100.]
    writer = Writer(tmp_path, capacity=128)
    service = Service(Config(), serial, ManualCamera(), nav, writer,
                      monotonic=lambda: clock[0])
    def log(record):
        records.append(record)
        service._nav_record(record)
    nav.log = log
    service.clock.update(600000, clock[0])
    service.start()
    try:
        service.worker.submit_message(link.Status(epoch=0, state=2, baseline_pa=101325.,
            health=link.HEALTH_BASELINE | link.HEALTH_ORIGIN,
            origin_lat_deg=22., origin_lon_deg=114., origin_h_msl_m=15.), .6)
        wait_for(lambda: service.worker.snapshot().nav is not None)
        for name, t, message in (
            ('on_gnss', .5, link.Gnss(tick=500000, fix_type=3, fix_flags=1,
                lat_deg=22., lon_deg=114., h_msl_m=44.5, vel_e=1., vel_n=0., vel_d=1.,
                h_acc_m=1., v_acc_m=2., s_acc=.2)),
            ('on_baro', .55, link.Baro(tick=550000, pressure_pa=100970., flags=1))):
            clock[0] += .01
            service.worker.submit_message(message, t)
            wait_for(lambda: service.worker.snapshot().published_s == clock[0])
            serial.tx.clear()
            service._next_nav = clock[0]
            service.step()
            actual = [m for m in sent(serial) if isinstance(m, link.Nav)]
            assert actual and actual[-1].flags & FLAG_TRN_VALID
            assert actual[-1].tick == 600000
            assert (name, t) in callbacks
            assert nav._filter_t == .6  # no next IMU has arrived
        assert [r for r in records if r['kind'] == 'comparison'][-1]['t'] == .5
    finally:
        service.close()
    assert writer.dropped == 0
    persisted = [json.loads(line) for line in (writer.session_dir / 'navigator.jsonl').read_text().splitlines()]
    comparison = [r for r in persisted if r['kind'] == 'comparison'][-1]
    assert comparison['t'] == .5 and comparison['shell_generation'] == 0
    assert comparison['generation'] == 0 and comparison['epoch'] == 0
    assert len(comparison['horizontal_covariance']) == 4
    assert any(r['kind'] == 'nav' and r['update_source'] == 'baro_replay' for r in persisted)


def test_publication_watermark_unwraps_before_max_and_resets_per_generation():
    class PublicationNavigator(FakeNavigator):
        def nav_message(self, tick):
            self.record('publication', tick)
            return link.Nav(tick=tick)

    nav = PublicationNavigator()
    worker = NavigatorWorker(nav)
    worker.start()
    try:
        for message, tick in ((link.Imu(), link.TICK_WRAP - 10),
                              (link.Imu(), link.TICK_WRAP + 20),
                              (link.Gnss(), link.TICK_WRAP - 5),
                              (link.Baro(), link.TICK_WRAP + 5)):
            count = len([x for x in nav.calls if x[0] == 'publication'])
            worker.submit_message(message, tick/link.TICK_HZ)
            wait_for(lambda: len([x for x in nav.calls if x[0] == 'publication']) > count)
        assert [x[1] for x in nav.calls if x[0] == 'publication'] == [link.TICK_WRAP-10, 20, 20, 20]
        worker.restart()
        worker.submit_message(link.Imu(), .000003)
        wait_for(lambda: worker.snapshot().generation == 1 and worker.snapshot().nav is not None
                 and worker.snapshot().nav.tick == 3)
        assert ('imu', .000003) in nav.calls
    finally:
        assert worker.stop()


def test_shutdown_full_ack_queue_and_active_partial_frame_requires_done_and_drain():
    s, nav, serial, camera, writer, clock, hooks = make_service()
    try:
        original_write = serial.write
        def start_then_block(data):
            serial.partial = 3
            count = original_write(data)
            serial.partial = 0
            serial.write = original_write
            return count
        serial.write = start_then_block
        s._send(link.Ack(100, link.CMD_PREPARE, link.ACK_ACCEPTED))
        s._transmit()
        assert s._wire_offset == 3 and s._wire
        for seq in range(64):
            assert s._send(link.Ack(seq, link.CMD_PREPARE, link.ACK_ACCEPTED))
        shutdown = link.Cmd(cmd_seq=80, cmd=link.CMD_SHUTDOWN, arg=10)
        s._command(shutdown)
        wait_for(s.shutdown_finished.is_set)
        assert s.shutdown_ok
        s.step()
        assert not s._shutdown_done_queued
        assert s._commands[(80, link.CMD_SHUTDOWN)][1] == link.ACK_ACCEPTED
        assert not s.finished and not hooks
        serial.partial = 4096
        serial.drain = lambda timeout: False
        for _ in range(3):
            s.step()
        done = [m for m in sent(serial) if isinstance(m, link.Ack)
                and m.cmd_seq == 80 and m.cmd == link.CMD_SHUTDOWN and m.result == link.ACK_DONE]
        assert len(done) == 1
        assert not s.finished and not hooks
        serial.drain = lambda timeout: True
        s.step()
        assert s.finished and hooks == ['poweroff']
    finally:
        s.close()


def test_shutdown_write_fault_during_cleanup_cannot_be_overwritten_by_cleanup_success():
    s, nav, serial, camera, writer, clock, hooks = make_service()
    entered, release = threading.Event(), threading.Event()
    original_stop, original_write = camera.stop, serial.write
    def blocked_stop():
        entered.set()
        assert release.wait(2.)
        original_stop()
    def fail_once(data):
        serial.write = original_write
        raise OSError('injected ambiguous UART write')
    camera.stop = blocked_stop
    try:
        s._command(link.Cmd(cmd_seq=80, cmd=link.CMD_SHUTDOWN, arg=10))
        assert entered.wait(2.)
        serial.write = fail_once
        s.step()
        release.set()
        wait_for(s.shutdown_finished.is_set)
        s.step()
        assert not hooks and not s.finished
        assert not s.shutdown_ok and not s._shutdown_done_sent
    finally:
        release.set()
        s.close()
