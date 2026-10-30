"""GPL-3.0-only. Final ACK cancellation, commit and ambiguous-UART regressions."""
import pytest

from cansat import link
from test_shell import make_service, wait_for


def decoded(serial):
    decoder = link.Decoder()
    return decoder, decoder.feed(serial.tx)


def done(messages):
    return [m for m in messages if isinstance(m, link.Ack) and m.cmd_seq == 80
            and m.cmd == link.CMD_SHUTDOWN and m.result == link.ACK_DONE]


def begin(service):
    command = link.Cmd(cmd_seq=80, cmd=link.CMD_SHUTDOWN, arg=10)
    service._command(command)
    service._transmit()  # accepted is already sent before final-ACK fault injection
    wait_for(service.shutdown_finished.is_set)
    return command


@pytest.mark.parametrize('accepted', ['none', 'prefix', 'body', 'complete'])
def test_final_ack_write_error_latches_refusal_and_stops_ambiguous_tx(accepted):
    s, nav, serial, camera, writer, clock, hooks = make_service()
    command = begin(s)
    original = serial.write
    try:
        def ambiguous_write(data):
            serial.partial = {'none': 0, 'prefix': 3, 'body': len(data)-1,
                              'complete': len(data)}[accepted]
            original(data)  # emulate bytes accepted before the driver raises
            raise OSError('injected ambiguous final ACK write')
        serial.write = ambiguous_write
        s._finish_shutdown(clock[0])
        before = bytes(serial.tx)
        _, messages = decoded(serial)
        assert len(done(messages)) == (1 if accepted == 'complete' else 0)
        serial.write, serial.partial = original, 4096
        encoder = link.Encoder()
        # Incoming retries use the actual codecs. Churn the bounded command
        # cache too: the abort decision must outlive its eviction policy.
        for seq in range(40):
            serial.rx.extend(encoder.encode(link.Cmd(cmd_seq=seq, cmd=link.CMD_PREPARE)))
            s.step()
        serial.rx.extend(encoder.encode(command))
        s.step()
        assert s._commands[(80, link.CMD_SHUTDOWN)][1] == link.ACK_REFUSED
        assert bytes(serial.tx) == before  # never append a delimiter to unknown bytes
        assert not s.shutdown_ok and not s.finished and not hooks
    finally:
        serial.write = original
        s.close()


@pytest.mark.parametrize('behind_partial_frame', [False, True])
def test_expired_unsent_done_is_withdrawn_before_retry_or_regular_transmit(behind_partial_frame):
    s, nav, serial, camera, writer, clock, hooks = make_service(shutdown_timeout_s=.1)
    original = serial.write
    try:
        command = begin(s)
        if behind_partial_frame:
            s._send(link.Ack(17, link.CMD_PREPARE, link.ACK_ACCEPTED))
            def start_then_block(data):
                serial.partial = 3
                count = original(data)
                serial.partial = 0
                serial.write = original
                return count
            serial.write = start_then_block
            s._transmit()
            assert s._wire_offset == 3
        serial.partial = 0
        s._finish_shutdown(clock[0])
        assert s._shutdown_done_queued and not s._shutdown_done_sent
        assert s._commands[(80, link.CMD_SHUTDOWN)][1] == link.ACK_ACCEPTED
        clock[0] += .2
        # Recover UART and deliver the duplicate in the *first* expired poll.
        # Both command handling and ordinary TX must consult the abort state.
        serial.partial = 4096
        serial.rx.extend(link.Encoder().encode(command))
        s.step()
        decoder, messages = decoded(serial)
        assert not done(messages)
        assert not decoder.stuffing and not decoder.crc and not decoder.length
        assert any(isinstance(m, link.Ack) and m.cmd_seq == 80
                   and m.result == link.ACK_REFUSED for m in messages)
        if behind_partial_frame:
            assert any(isinstance(m, link.Ack) and m.cmd_seq == 17 for m in messages)
        assert not s._shutdown_done_queued
        assert not s.shutdown_ok and not s.finished and not hooks
    finally:
        serial.write = original
        s.close()


@pytest.mark.parametrize('prefix', [3, 'body'])
def test_started_final_ack_finishes_drains_and_halts_after_local_deadline(prefix):
    s, nav, serial, camera, writer, clock, hooks = make_service(shutdown_timeout_s=.1)
    original = serial.write
    try:
        begin(s)
        def start_then_block(data):
            serial.partial = len(data)-1 if prefix == 'body' else prefix
            count = original(data)
            serial.partial = 0
            serial.write = original
            return count
        serial.write = start_then_block
        serial.drain = lambda timeout: False
        s._finish_shutdown(clock[0])
        assert 0 < s._wire_offset < len(s._wire)
        assert not done(decoded(serial)[1])
        clock[0] += .2
        serial.partial = 4096
        s.step()
        decoder, messages = decoded(serial)
        assert len(done(messages)) == 1
        assert not decoder.stuffing and not decoder.crc and not decoder.length
        assert not s.finished and not hooks  # drain remains necessary
        serial.drain = lambda timeout: True
        s.step()
        assert s.finished and s.shutdown_ok and hooks == ['poweroff']
    finally:
        serial.write = original
        s.close()


def test_committed_done_retries_drain_and_halt_errors_after_local_deadline():
    s, nav, serial, camera, writer, clock, hooks = make_service(shutdown_timeout_s=.1)
    try:
        begin(s)
        def drain_failure(timeout):
            raise OSError('injected transient drain error')
        serial.drain = drain_failure
        s._finish_shutdown(clock[0])
        assert len(done(decoded(serial)[1])) == 1
        assert not s.finished and not hooks
        clock[0] += .2
        serial.drain = lambda timeout: True
        calls = []
        def halt():
            calls.append('attempt')
            if len(calls) == 1:
                raise OSError('injected transient halt error')
            hooks.append('poweroff')
        s.poweroff = halt
        s.step()
        assert not s.finished and not hooks
        s.step()
        assert len(calls) == 2 and s.finished and hooks == ['poweroff']
    finally:
        s.close()
