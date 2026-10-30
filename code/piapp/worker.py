"""GPL-3.0-only. Single-owner Navigator worker, implementing PI-SHELL section 5.

Sensor delivery is FIFO. Frames occupy one newest-wins slot, with a sensor
arrival barrier so already-received IMU spans reach navigation before a frame.
An overflow is an explicit discontinuity, never a silently missing IMU span.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import threading
import time

from cansat import link


@dataclass(frozen=True)
class Snapshot:
    generation: int = 0
    mode: str = 'waiting'
    flags: int = 0
    wants_frames: bool = False
    nav: link.Nav | None = None
    published_s: float = 0.
    error: str | None = None


class NavigatorWorker:
    def __init__(self, navigator, capacity=1500, *, monotonic=time.monotonic, log=None):
        if not 2 <= capacity <= 20000:
            raise ValueError('capacity must be 2..20000')
        self.navigator = navigator
        self.capacity = capacity
        self.monotonic = monotonic
        self.log = log or (lambda record: None)
        self._condition = threading.Condition()
        self._messages = deque()
        self._frame = None
        self._generation = self._order = 0
        self._restart = False
        self._stopping = False
        self._snapshot = Snapshot()
        self.executing_generation = 0  # read only by worker-owned log callbacks
        self.frames_dropped = self.messages_dropped = self.overflows = self.faults = 0
        self._thread = threading.Thread(target=self._run, name='navigator', daemon=True)

    @property
    def alive(self):
        return self._thread.is_alive()

    @property
    def pending_messages(self):
        with self._condition:
            return len(self._messages)

    def start(self):
        self._thread.start()

    def snapshot(self):
        with self._condition:
            return deepcopy(self._snapshot)

    def _invalidate(self):
        self._generation += 1
        self.messages_dropped += len(self._messages)
        self._messages.clear()
        self.frames_dropped += self._frame is not None
        self._frame = None
        self._restart = True
        self._snapshot = Snapshot(generation=self._generation)
        self._condition.notify()

    def restart(self):
        with self._condition:
            self._invalidate()

    def submit_message(self, message, t):
        if not isinstance(message, (link.Imu, link.Baro, link.Status, link.Gnss)):
            raise TypeError('not a navigator sensor message')
        with self._condition:
            if self._stopping:
                return False
            if len(self._messages) >= self.capacity:
                self.overflows += 1
                self._invalidate()
            self._order += 1
            self._messages.append((self._order, message, t))
            self._condition.notify()
            return True

    def submit_frame(self, image, t, frame):
        # Caller transfers an immutable image, shared with the SD writer.
        with self._condition:
            if self._stopping:
                return False
            self.frames_dropped += self._frame is not None
            self._frame = (self._order, image, t, frame)
            self._condition.notify()
            return True

    def stop(self, timeout=5.):
        with self._condition:
            self._stopping = True
            self._messages.clear()
            self._frame = None
            self._snapshot = Snapshot(generation=self._generation)
            self._condition.notify()
        if self._thread.ident is not None:
            self._thread.join(timeout)
        return not self.alive

    def _publish(self, generation, tick):
        # Every navigator API/property access is confined to this thread.
        snapshot = Snapshot(generation, self.navigator.mode, self.navigator.pi_flags,
                            self.navigator.wants_frames(), self.navigator.nav_message(tick),
                            self.monotonic())
        with self._condition:
            if generation == self._generation and not self._stopping:
                self._snapshot = snapshot

    def _run(self):
        # Arrival order is not observation-time order (notably PPS GNSS and
        # delayed pressure). Compare unwrapped ticks, reducing only at the API.
        publication_tick = 0
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopping or self._restart
                                         or self._messages or self._frame is not None)
                if self._stopping:
                    return
                generation = self._generation
                if self._restart:
                    self._restart = False
                    kind, item = 'restart', None
                elif self._messages and (self._frame is None or self._messages[0][0] <= self._frame[0]):
                    kind, item = 'message', self._messages.popleft()
                else:
                    kind, item, self._frame = 'frame', self._frame, None
            try:
                self.executing_generation = generation
                if kind == 'restart':
                    self.navigator.restart()
                    publication_tick = 0
                elif kind == 'message':
                    _, message, t = item
                    publication_tick = max(publication_tick, int(round(t * link.TICK_HZ)))
                    method = {link.Imu: 'on_imu', link.Baro: 'on_baro',
                              link.Status: 'on_status', link.Gnss: 'on_gnss'}[type(message)]
                    getattr(self.navigator, method)(message, t)
                else:
                    _, image, t, frame = item
                    publication_tick = max(publication_tick, int(round(t * link.TICK_HZ)))
                    if self.navigator.wants_frames():
                        self.navigator.on_frame(image, t, frame)
                self._publish(generation, publication_tick % link.TICK_WRAP)
            except Exception as error:
                self.faults += 1
                self.log({'kind': 'event', 'event': 'navigator_fault', 'error': repr(error)})
                with self._condition:
                    self._invalidate()
                    self._snapshot = Snapshot(generation=self._generation, error=repr(error))
                    # A failing restart cannot be retried in a hot infinite loop.
                    if kind == 'restart':
                        self._restart = False
                        self._stopping = True
