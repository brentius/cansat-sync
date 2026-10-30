"""GPL-3.0-only. PI-SHELL serial/control service (interface contract section 5).

The calling thread exclusively owns UART, codecs, clock and stamper. Camera,
navigation, health and storage run separately. No camera/SD acquisition wait
occurs in step(). Default timing/queue policies are [assumed], host tested only.
"""
from collections import OrderedDict, deque
from dataclasses import asdict
import math
import subprocess
import threading
import time

import numpy as np

from cansat import link
from cansat.flight_state import State

from .worker import NavigatorWorker


# A transient serial exception suspends triggers/Nav/healthy status for this
# long after the most recent one; it does not latch for the rest of the flight.
SERIAL_FAULT_HOLD_S = 1.


def system_poweroff():
    subprocess.run(['systemctl', 'poweroff'], check=True, timeout=5.)


class Service:
    def __init__(self, config, serial, camera, navigator, writer, *,
                 monotonic=time.monotonic, poweroff=None, health=None):
        self.config, self.serial, self.camera, self.writer = config, serial, camera, writer
        self.monotonic = monotonic
        self.poweroff = poweroff
        self.health = health or (lambda: (-128, 255, 0))
        self.decoder, self.encoder = link.Decoder(raw_unknown=True), link.Encoder()
        self.clock = link.TickClock()
        self.stamper = link.FrameStamper(latency_s=config.frame_latency_s)
        self.worker = NavigatorWorker(navigator, config.message_capacity,
                                      monotonic=monotonic, log=self._nav_record)
        self.generation = 0
        self._epoch = None
        self._suppress_restart_flag = False
        self._last_restart_imu = None
        self._frame_lock = threading.Lock()
        self._camera_frame = None
        self._pending_frame = None
        self._last_camera_local = -math.inf
        self._last_stamped = None
        self._trigger_received = None
        self._trigger_start_tick = None
        self.frames_received = self.frames_dropped = self.frames_refused = 0
        self.tx_dropped = self.serial_faults = 0
        self._queue = deque()
        self._current = None
        self._wire = b''
        self._wire_offset = 0
        self._started = False
        self._closed = False
        self.finished = False
        self.fault = None
        self._io_fault, self._io_fault_until = None, -math.inf
        self._camera_thread = None
        self._camera_error = None
        self._state = State.PAD
        self._landed_at = None
        self._prepared = False
        self.trigger_enabled = False
        self._cfg_seq = 0
        self._commands = OrderedDict()
        self.stopping = False
        self.shutdown_finished = threading.Event()
        self.shutdown_ok = False
        self._shutdown_thread = None
        self._shutdown_key = None
        self._shutdown_arg = None
        self._shutdown_aborted = False
        self._shutdown_done_queued = False
        self._shutdown_done_ack = None
        self._shutdown_done_sent = False
        self._shutdown_done_started = False
        self._shutdown_tx_failed = False
        self._poweroff_called = False
        self._shutdown_deadline = None
        self._start_s = monotonic()
        self._next_status = self._next_nav = self._next_cfg = self._start_s
        self._echo_tick, self._echo_received = 0, None

    def _event(self, event, **kwargs):
        self.writer.record('events', {'kind': 'event', 'event': event,
                                      'local_s': self.monotonic(), 'generation': self.generation, **kwargs})

    def _nav_record(self, record):
        self.writer.record('navigator', {**record, 'shell_generation': self.worker.executing_generation})

    def _active_fault(self, now):
        """Latched fault, else a serial I/O fault within SERIAL_FAULT_HOLD_S."""
        if self.fault:
            return self.fault
        return self._io_fault if now < self._io_fault_until else None

    def _serial_fault(self, kind, error, now):
        self.serial_faults += 1
        if now >= self._io_fault_until:
            # One event per fault episode; a dead port must not flood the writer.
            self._event(kind, error=repr(error))
        self._io_fault, self._io_fault_until = repr(error), now + SERIAL_FAULT_HOLD_S

    def _nav_fresh(self, message, now):
        current_tick = self.clock.to_ticks(now)
        age = (math.inf if current_tick is None else
               (current_tick - self.clock.extend(message.tick)) / link.TICK_HZ)
        return (not self._active_fault(now) and not self.stopping
                and -.05 <= age <= self.config.max_nav_cache_age_s)

    def start(self):
        if self._started:
            raise RuntimeError('service already started')
        self._started = True
        self.writer.start()
        self.writer.record('session', {'kind': 'configuration', 'config': asdict(self.config)})
        self.worker.start()

        def start_camera():
            try:
                self.camera.start(self._capture)
            except Exception as error:
                self._camera_error = repr(error)
                self.fault = 'camera startup: ' + repr(error)
                self._event('camera_start_fault', error=repr(error))

        self._camera_thread = threading.Thread(target=start_camera, name='camera-start', daemon=True)
        self._camera_thread.start()

    def _capture(self, image, local_s):
        # Adapters transfer an owned image. Service marks it immutable so SD and
        # navigation can share one bounded frame allocation without alteration.
        if (not isinstance(image, np.ndarray) or image.ndim != 2
                or image.dtype not in (np.uint8, np.uint16)
                or image.shape != (self.config.camera.height, self.config.camera.width)
                or not math.isfinite(local_s)):
            with self._frame_lock:
                self.frames_refused += 1
            return
        image.setflags(write=False)
        with self._frame_lock:
            if self.stopping or local_s <= self._last_camera_local:
                self.frames_dropped += 1
                return
            self._last_camera_local = local_s
            self.frames_received += 1
            self.frames_dropped += self._camera_frame is not None
            self._camera_frame = (image, local_s, self.generation, self.monotonic())

    def _send(self, message):
        if (self._shutdown_tx_failed or (self._shutdown_aborted
                and self._is_shutdown_done(message))):
            self.tx_dropped += 1
            return False
        # Coalesce periodic messages; never accumulate stale Nav or heartbeats.
        if isinstance(message, (link.PiStatus, link.Nav, link.TrigCfg)):
            self._queue = deque(m for m in self._queue if type(m) is not type(message))
        if len(self._queue) >= 64:
            self.tx_dropped += 1
            return False
        if isinstance(message, link.Ack):
            self._queue.appendleft(message)
        else:
            self._queue.append(message)
        return True

    def _transmit(self):
        # Bounded work per poll, even for adapters returning one byte at a time.
        budget = 4096
        for _ in range(128):
            # All write paths, including ordinary step() and direct completion
            # polls, must withdraw an expired, wholly unsent final ACK first.
            self._check_shutdown(self.monotonic())
            if self._shutdown_tx_failed:
                # A failed write may have accepted an unknown suffix, even a
                # delimiter. Never append another frame/delimiter to it here.
                return
            if not self._wire:
                if not self._queue:
                    return
                self._current = self._queue.popleft()
                if isinstance(self._current, link.Nav) and not self._nav_fresh(self._current, self.monotonic()):
                    self.tx_dropped += 1
                    self._current = None
                    continue
                self._wire = self.encoder.encode(self._current)
                self._wire_offset = 0
            if (self._wire_offset == 0 and isinstance(self._current, link.Nav)
                    and not self._nav_fresh(self._current, self.monotonic())):
                self.tx_dropped += 1
                self._wire, self._current = b'', None
                continue
            remaining = self._wire[self._wire_offset:self._wire_offset + budget]
            try:
                count = self.serial.write(remaining)
                if count is None or not 0 <= count <= len(remaining):
                    raise OSError('invalid serial write count')
            except Exception as error:
                # Bytes may already have reached the UART. Avoid retransmitting
                # a partially sent frame after an ambiguous transport failure;
                # the peer's decoder resynchronises at the next delimiter.
                self._serial_fault('serial_write_fault', error, self.monotonic())
                self._wire = b''
                self._current = None
                self._queue.clear()
                if self.stopping:
                    # Cleanup runs concurrently and can later report success;
                    # retain the ambiguous transport failure independently.
                    self.fault = repr(error)
                    self._shutdown_tx_failed = True
                    self._abort_shutdown(self.fault)
                return
            if not count:
                return
            if self._current is self._shutdown_done_ack:
                # Once a frame has started, preserve it and finish drain/halt.
                # The local budget is a pre-commit limit; already handed-off
                # bytes cannot be recalled. Ambiguous errors still fail closed.
                self._shutdown_done_started = True
            self.writer.raw_bytes('tx', remaining[:count])
            self._wire_offset += count
            budget -= count
            if self._wire_offset == len(self._wire):
                if self._current is self._shutdown_done_ack:
                    self._shutdown_done_sent = True
                    self._cache_command(self._shutdown_key, self._shutdown_arg, link.ACK_DONE)
                self.writer.record('decoded', {'direction': 'tx', 'local_s': self.monotonic(),
                                              'generation': self.generation,
                                              'type': type(self._current).__name__,
                                              'message': asdict(self._current)})
                self._wire, self._current = b'', None
            if budget <= 0:
                return

    def _restart(self, message):
        self.generation += 1
        self.clock.restart()
        self.decoder.reset_sequence()
        self.stamper.last = None
        self.worker.restart()
        self._epoch = getattr(message, 'epoch', self._epoch)
        self._suppress_restart_flag = True
        self._trigger_received = self._trigger_start_tick = self._last_stamped = None
        self._pending_frame = None
        self._prepared = False
        self._state, self._landed_at = State.PAD, None
        self._commands.clear()
        self._echo_received = None
        self._queue = deque(m for m in self._queue if not isinstance(m, link.Nav))
        if isinstance(self._current, link.Nav) and self._wire_offset == 0:
            self._wire, self._current = b'', None
        # A frame already partly written must finish to preserve COBS framing.
        with self._frame_lock:
            self.frames_dropped += self._camera_frame is not None
            self._camera_frame = None
        self._next_cfg = self._next_status = self.monotonic()
        self._event('stm32_restart', source=type(message).__name__, epoch=self._epoch)

    def _handle(self, message, now):
        # Hello is also sent when Pi TX reappears; it is not itself reset
        # evidence. A known matching boot epoch preserves navigation. Low-word
        # tick regression alone is ambiguous (wrap/delayed messages); TickClock
        # unwraps it. First epoch, changed epoch or a new IMU_RESTART is proof.
        restart = False
        if isinstance(message, (link.Hello, link.Status, link.Imu)):
            restart |= self._epoch is None or message.epoch != self._epoch
        if isinstance(message, link.Imu) and message.flags & link.IMU_RESTART:
            identity = (message.epoch, message.tick)
            restart |= not self._suppress_restart_flag and identity != self._last_restart_imu
            self._last_restart_imu = identity
        if restart:
            self._restart(message)
        if isinstance(message, link.Imu):
            self._suppress_restart_flag = False
        if hasattr(message, 'tick') and message.ID < 0x80:
            self.clock.update(message.tick, now)
            self._echo_tick, self._echo_received = message.tick, now
        if isinstance(message, link.Hello):
            if message.proto != link.PROTO_VERSION or message.tick_hz != link.TICK_HZ:
                self.fault = 'incompatible STM32 protocol or tick rate'
                self._event('protocol_mismatch', proto=message.proto, tick_hz=message.tick_hz)
        elif isinstance(message, link.Trig) and not self.stopping:
            if message.flags & 1:
                self._trigger_start_tick = self.clock.extend(message.tick)
                self._last_stamped = None
            self.stamper.add(message, self.clock)
            self._trigger_received = now
        elif isinstance(message, link.Cmd):
            self._command(message)
        elif isinstance(message, (link.Imu, link.Baro, link.Status, link.Gnss)) and not self.stopping:
            if isinstance(message, link.Status):
                if message.state in tuple(State):
                    self._state = State(message.state)
                if self._state == State.LANDED and self._landed_at is None:
                    self._landed_at = now
                elif self._state != State.LANDED:
                    self._landed_at = None
            if self.fault != 'incompatible STM32 protocol or tick rate':
                self.worker.submit_message(message, self.clock.extend(message.tick) / link.TICK_HZ)
        return restart

    def _receive(self, data, now):
        self.writer.raw_bytes('rx', data)
        # Decode at delimiter boundaries so a Hello/epoch reset takes effect
        # before the next frame in the SAME OS read, not after a decoded batch.
        parts = data.split(b'\x00')
        for i, part in enumerate(parts):
            if not part and i == len(parts) - 1:
                break
            chunk = part + (b'\x00' if i < len(parts) - 1 else b'')
            lost, repeats = self.decoder.lost, self.decoder.repeats
            for message in self.decoder.feed(chunk):
                if getattr(message, 'ID', 0) >= 0x80:
                    self._event('wrong_direction', type=type(message).__name__)
                    continue
                restarted = self._handle(message, now)
                if restarted:
                    # Decoder learned about the reset only after decoding this
                    # frame; its boundary jump is not packet loss/repetition.
                    self.decoder.lost, self.decoder.repeats = lost, repeats
                self.writer.record('decoded', {'direction': 'rx', 'local_s': now,
                                              'generation': self.generation, 'seq': message.seq,
                                              'type': type(message).__name__, 'message': asdict(message)})

    def _frames(self, now):
        with self._frame_lock:
            newest, self._camera_frame = self._camera_frame, None
        if newest is not None:
            self.frames_dropped += self._pending_frame is not None
            self._pending_frame = newest
        if self._pending_frame is None:
            return
        image, received, generation, queued = self._pending_frame
        if (self.stopping or generation != self.generation
                or now - received > self.config.max_frame_age_s or received > now + .001):
            self.frames_refused += 1
            self._pending_frame = None
            return
        stamp = None
        if (self._trigger_received is not None
                and now - self._trigger_received <= self.config.max_trigger_age_s):
            stamp = self.stamper.stamp(received, self.clock)
        if stamp is None:
            if now - queued >= self.config.frame_wait_s:
                self.frames_refused += 1
                self._pending_frame = None
            return
        frame, tick = stamp
        # Refuse duplicate requests or extrapolation into a previous trigger run.
        if ((self._trigger_start_tick is not None and tick < self._trigger_start_tick)
                or (self._last_stamped is not None and tick <= self._last_stamped)):
            self.frames_refused += 1
            self._pending_frame = None
            return
        self._last_stamped = tick
        self.writer.frame(image, tick=tick, frame=frame, generation=generation, local_s=received)
        self.worker.submit_frame(image, tick / link.TICK_HZ, frame)
        self._pending_frame = None

    def _trigger_policy(self, now):
        ready = bool(self.camera.ready) and not self._camera_error and not self.camera.error
        enabled = (ready and not self.stopping and not self._active_fault(now)
                   and (self.config.capture_on_pad or self._prepared
                        or self._state in (State.ASCENT, State.DESCENT)))
        if self._landed_at is not None and now - self._landed_at >= self.config.landed_capture_s:
            enabled = False
        if enabled != self.trigger_enabled:
            self.trigger_enabled = enabled
            self._cfg_seq = (self._cfg_seq + 1) & 255
            self._next_cfg = now
            if not enabled:
                self.stamper.last = None
                self._trigger_received = None
        if now >= self._next_cfg:
            self._send(link.TrigCfg(enable=int(enabled), cfg_seq=self._cfg_seq,
                                    period_ms=self.config.trigger_period_ms,
                                    pulse_us=self.config.trigger_pulse_us))
            self._next_cfg = now + 1.

    def _status(self, now):
        snapshot = self.worker.snapshot()
        temp, load, health_flags = self.health()
        flags = health_flags | (snapshot.flags & (link.PIFLAG_MAP | link.PIFLAG_NAV))
        if self._epoch is not None:
            flags |= link.PIFLAG_EPOCH_ECHO | ((self._epoch & 255) << link.PIFLAG_EPOCH_SHIFT)
        camera_ready = self.camera.ready and not self._camera_error and not self.camera.error
        if camera_ready and not self.stopping:
            flags |= link.PIFLAG_CAMERA
        if self.writer.healthy:
            flags |= link.PIFLAG_STORAGE
        fault = (self._active_fault(now) or self._camera_error or self.camera.error or snapshot.error
                 or not self.writer.healthy)
        if self.stopping:
            state = link.PI_STOPPING
        elif fault:
            state = link.PI_FAULT
        elif snapshot.mode in ('tracking', 'coasting'):
            state = link.PI_NAVIGATING
        elif camera_ready:
            state = link.PI_CAMERA
        elif self._camera_thread and self._camera_thread.is_alive():
            state = link.PI_BOOTING
        else:
            state = link.PI_READY
        age = 65535 if self._echo_received is None else min(65535, max(0, round((now - self._echo_received) * 1000)))
        self._send(link.PiStatus(uptime_ms=int((now - self._start_s) * 1000),
                                tick_echo=self._echo_tick, echo_age_ms=age, state=state, flags=flags,
                                sw_version=2, rx_crc=min(65535, self.decoder.crc),
                                rx_lost=min(65535, self.decoder.lost), frames=self.frames_received,
                                cpu_temp_c=temp, cpu_load=load))

    def _command(self, message):
        self._check_shutdown(self.monotonic())
        key = (message.cmd_seq, message.cmd)
        if key == self._shutdown_key:
            # This transaction's terminal decision outlives the bounded cache.
            result = (link.ACK_REFUSED if self._shutdown_aborted or message.arg != self._shutdown_arg
                      else link.ACK_DONE if self._shutdown_done_sent else link.ACK_ACCEPTED)
            self._cache_command(key, self._shutdown_arg, result)
            self._send(link.Ack(message.cmd_seq, message.cmd, result))
            return
        if key in self._commands:
            arg, result = self._commands[key]
            self._send(link.Ack(message.cmd_seq, message.cmd,
                                result if arg == message.arg else link.ACK_REFUSED))
            return
        if message.cmd == link.CMD_PREPARE and not self.stopping:
            self._prepared = True
            result = link.ACK_ACCEPTED
        elif message.cmd == link.CMD_SHUTDOWN and not self.stopping:
            result = link.ACK_ACCEPTED
        else:
            result = link.ACK_REFUSED if self.stopping else link.ACK_UNKNOWN
        self._cache_command(key, message.arg, result)
        self._send(link.Ack(message.cmd_seq, message.cmd, result))
        self._event('command', cmd=message.cmd, cmd_seq=message.cmd_seq, arg=message.arg, result=result)
        if message.cmd == link.CMD_SHUTDOWN and result == link.ACK_ACCEPTED:
            self._shutdown_key = key
            self._shutdown_arg = message.arg
            self._begin_shutdown(min(self.config.shutdown_timeout_s, max(.1, float(message.arg))))

    def _cache_command(self, key, arg, result):
        self._commands[key] = (arg, result)
        while len(self._commands) > 32:
            self._commands.popitem(last=False)

    def _is_shutdown_done(self, message):
        return (isinstance(message, link.Ack) and self._shutdown_key is not None
                and (message.cmd_seq, message.cmd) == self._shutdown_key
                and message.result == link.ACK_DONE)

    def _abort_shutdown(self, reason):
        self.shutdown_ok = False
        if self._shutdown_aborted:
            return
        self._shutdown_aborted = True
        self.fault = reason
        if self._shutdown_key is not None:
            self._cache_command(self._shutdown_key, self._shutdown_arg, link.ACK_REFUSED)
        self._queue = deque(m for m in self._queue if not self._is_shutdown_done(m))
        # Only zero-byte frames can be withdrawn. A cleanly started final ACK
        # commits to completing/draining/halt; a write exception stops all TX.
        if self._is_shutdown_done(self._current) and self._wire_offset == 0:
            self._wire, self._current = b'', None
        self._shutdown_done_queued = False
        self._event('shutdown_aborted', reason=reason,
                    done_started=self._shutdown_done_started, done_sent=self._shutdown_done_sent)

    def _check_shutdown(self, now):
        if not self.stopping or self.finished:
            return
        if self._shutdown_aborted:
            self.shutdown_ok = False
        elif self._shutdown_tx_failed:
            self._abort_shutdown(self.fault or 'ambiguous UART write during shutdown')
        elif self.shutdown_finished.is_set() and not self.shutdown_ok:
            self._abort_shutdown(self.fault or 'shutdown cleanup failed')
        elif (not self._shutdown_done_started and self._shutdown_deadline is not None
              and now > self._shutdown_deadline):
            self._abort_shutdown('shutdown command budget expired before Ack done started')

    def _begin_shutdown(self, timeout):
        if self.stopping:
            return
        self.stopping = True
        self._shutdown_deadline = self.monotonic() + timeout
        # Invalidate cached output now, not when a busy on_frame eventually exits.
        self.worker.restart()
        self._queue = deque(m for m in self._queue if not isinstance(m, link.Nav))

        def shutdown():
            end = time.monotonic() + timeout
            remaining = lambda: max(0., end - time.monotonic())
            failures = []
            try:
                if self._camera_thread is not None:
                    self._camera_thread.join(remaining())
                    if self._camera_thread.is_alive():
                        raise TimeoutError('camera startup did not stop')
                self.camera.stop()
            except Exception as error:
                failures.append(repr(error))
            try:
                if not self.worker.stop(remaining()):
                    raise TimeoutError('navigator did not stop before shutdown deadline')
            except Exception as error:
                failures.append(repr(error))
            try:
                self._event('shutdown_flush', frames_received=self.frames_received,
                            frames_dropped=self.frames_dropped, frames_refused=self.frames_refused,
                            nav_frames_dropped=self.worker.frames_dropped,
                            messages_dropped=self.worker.messages_dropped, writer_dropped=self.writer.dropped,
                            failures=failures)
                if not self.writer.close(remaining()):
                    raise OSError('storage did not drain/sync cleanly')
            except Exception as error:
                failures.append(repr(error))
            finally:
                self.shutdown_ok = not failures and remaining() > 0 and not self._shutdown_aborted
                if failures:
                    self.fault = '; '.join(failures)
                self.shutdown_finished.set()

        self._shutdown_thread = threading.Thread(target=shutdown, name='shutdown', daemon=True)
        self._shutdown_thread.start()

    def _finish_shutdown(self, now):
        self._check_shutdown(now)
        if self._shutdown_aborted:
            return
        if not self.shutdown_finished.is_set() or not self.shutdown_ok:
            return
        if self._shutdown_key is None:
            self.finished = True
            return
        if not self._shutdown_done_queued:
            seq, command = self._shutdown_key
            ack = link.Ack(seq, command, link.ACK_DONE)
            if not self._send(ack):
                # Keep the command accepted and retry on the next poll. An
                # active partial frame and a full ACK queue may block capacity.
                self._transmit()
                return
            self._shutdown_done_ack = ack
            self._shutdown_done_queued = True
        self._transmit()
        if (not self.shutdown_ok or not self._shutdown_done_sent
                or self._queue or self._wire or self._poweroff_called):
            return
        try:
            if not self.serial.drain(.1):
                return
            if self.poweroff is not None:
                self.poweroff()
            elif self.config.allow_poweroff:
                system_poweroff()
            self._poweroff_called = True
            self.finished = True
        except Exception as error:
            self.fault = repr(error)

    def step(self):
        if not self._started or self._closed:
            raise RuntimeError('service must be started and open')
        now = self.monotonic()
        try:
            data = self.serial.read(4096)
        except Exception as error:
            self._serial_fault('serial_read_fault', error, now)
            data = b''
        if data:
            self._receive(data, now)
        if not self.stopping:
            self._frames(now)
        self._trigger_policy(now)
        if now >= self._next_status:
            self._status(now)
            self._next_status = now + .5
        if now >= self._next_nav and not self.stopping:
            snapshot = self.worker.snapshot()
            # Publication can follow a long on_frame call. Compare the solution
            # epoch with live STM32 time too, not only the fresh cache timestamp.
            if (not self._active_fault(now) and snapshot.nav is not None
                    and now - snapshot.published_s <= self.config.max_nav_cache_age_s
                    and self._nav_fresh(snapshot.nav, now)):
                self._send(snapshot.nav)
            self._next_nav = now + .2
        self._transmit()
        if self.stopping:
            self._finish_shutdown(now)

    def run(self, stop=None, duration=None):
        stop = stop or threading.Event()
        started = self.monotonic()
        try:
            self.start()
            while not self.finished and not stop.is_set():
                self.step()
                if duration is not None and self.monotonic() - started >= duration:
                    break
                stop.wait(self.config.poll_interval_s)
        finally:
            self.close()

    def close(self):
        if self._closed:
            return self.shutdown_ok
        if self._started and not self.stopping:
            self._begin_shutdown(self.config.shutdown_timeout_s)
            self._trigger_policy(self.monotonic())
            self._transmit()
        if self._shutdown_thread is not None:
            self._shutdown_thread.join(self.config.shutdown_timeout_s)
        self.serial.close()
        self._closed = True
        return self.shutdown_ok
