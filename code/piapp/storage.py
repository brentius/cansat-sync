"""GPL-3.0-only. Bounded asynchronous SD writer for the PI-SHELL contract.

Each launch creates a unique directory. Frames are write-exclusive, then
atomically renamed before their index row is emitted. No prior data is removed.
Power-loss durability is limited by fsync interval and the card/controller.
Frames (about 1.6 MB each) and small records/raw UART chunks are bounded
separately, so a slow frame write/fsync cannot evict the log records that
arrive at hundreds per second meanwhile. One FIFO preserves write order.
"""
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
from uuid import uuid4

import numpy as np


def json_value(value):
    if is_dataclass(value):
        value = asdict(value)
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, bytes):
        return value.hex()
    return value


class Writer:
    def __init__(self, root, capacity=32, sync_interval_s=1., record_capacity=2048):
        """capacity bounds pending frames; record_capacity pending records/raw chunks."""
        if not 1 <= capacity <= 128 or not 1 <= record_capacity <= 65536 or sync_interval_s <= 0:
            raise ValueError('invalid writer limits')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
        self.session_dir = Path(root) / f'{stamp}-{uuid4().hex}'
        self.sync_interval_s = sync_interval_s
        self._limits = {True: capacity, False: record_capacity}
        self._pending = {True: 0, False: 0}     # keyed by "is a frame"
        self._queue = queue.Queue()             # bounded by the pending counts
        self._lock = threading.Lock()
        self._closed = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='sd-writer', daemon=True)
        self._files = {}
        self._frame_number = 0
        self.healthy = True
        self.error = None
        self.dropped = self.written = 0

    def start(self):
        self._thread.start()

    def _put(self, item):
        frame = item[0] == 'frame'
        with self._lock:
            if self._closed or not self.healthy or self._pending[frame] >= self._limits[frame]:
                self.dropped += 1
                return False
            self._pending[frame] += 1
            self._queue.put_nowait(item)
            return True

    def record(self, stream, data):
        if stream not in ('events', 'decoded', 'navigator', 'frames', 'session'):
            raise ValueError('unknown record stream')
        return self._put(('record', stream, data))

    def raw_bytes(self, direction, data):
        if direction not in ('rx', 'tx'):
            raise ValueError('raw direction must be rx or tx')
        return self._put(('raw', direction, bytes(data)))

    def frame(self, image, **metadata):
        return self._put(('frame', metadata, image))

    def _stream(self, name, binary=False):
        if name not in self._files:
            self._files[name] = (self.session_dir / name).open('xb' if binary else 'x',
                                                             **({} if binary else {'encoding': 'utf-8'}))
        return self._files[name]

    def _record(self, name, value):
        self._stream(name + '.jsonl').write(json.dumps(json_value(value), allow_nan=False,
                                                     separators=(',', ':')) + '\n')

    def _sync(self):
        for stream in self._files.values():
            stream.flush()
            os.fsync(stream.fileno())
        if os.name == 'posix':
            fd = os.open(self.session_dir, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)

    def _write(self, item):
        kind, name, data = item
        if kind == 'raw':
            self._stream('raw_' + name + '.bin', True).write(data)
        elif kind == 'record':
            self._record(name, data)
        else:
            self._frame_number += 1
            filename = f"g{int(name['generation']):04d}-f{int(name['frame']):010d}-{self._frame_number:010d}.npy"
            final = self.session_dir / filename
            temporary = final.with_suffix('.npy.part')
            with temporary.open('xb') as stream:
                np.save(stream, data, allow_pickle=False)
                stream.flush()
                os.fsync(stream.fileno())
            temporary.rename(final)
            self._record('frames', {**name, 'file': filename})

    def _fault(self, error):
        self.healthy = False
        self.error = repr(error)

    def _run(self):
        try:
            self.session_dir.mkdir(parents=True, exist_ok=False)
            self._record('session', {'kind': 'session', 'schema': 1, 'host_verified_only': True})
            last_sync = time.monotonic()
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=.05)
                except queue.Empty:
                    item = None
                if item is not None:
                    try:
                        if self.healthy:
                            self._write(item)
                            self.written += 1
                        else:
                            self.dropped += 1
                    except Exception as error:
                        self._fault(error)
                        self.dropped += 1
                    finally:
                        with self._lock:
                            self._pending[item[0] == 'frame'] -= 1
                        self._queue.task_done()
                if self.healthy and time.monotonic() - last_sync >= self.sync_interval_s:
                    self._sync()
                    last_sync = time.monotonic()
            if self.healthy:
                self._record('events', {'kind': 'storage_closed', 'written': self.written,
                                        'dropped': self.dropped})
                self._sync()
        except Exception as error:
            self._fault(error)
        finally:
            for stream in self._files.values():
                try:
                    stream.close()
                except Exception as error:
                    self._fault(error)

    def close(self, timeout=5.):
        with self._lock:
            self._closed = True
            self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout)
        return not self._thread.is_alive() and self.healthy
