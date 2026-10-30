"""GPL-3.0-only. Lazy Picamera2 IMX296, synthetic and indexed replay adapters.

PI-SHELL section 5: start(callback(image, receive_monotonic_s)), stop(),
ready/error properties. Callbacks transfer owned greyscale arrays. XTR mode,
fixed gain/exposure and YUV420 layout follow Raspberry Pi docs cited in deploy
README. Frame latency is a configuration assumption requiring bench calibration.
"""
import json
from pathlib import Path
import threading
import time

import numpy as np


class _ThreadCamera:
    def __init__(self):
        self.ready = False
        self.error = None
        self._stop = threading.Event()
        self._thread = None

    def start(self, callback):
        self.callback = callback
        self._stop.clear()
        self.ready = True
        self._thread = threading.Thread(target=self._run_guarded, name='camera-source', daemon=True)
        self._thread.start()

    def _run_guarded(self):
        try:
            self._run()
        except Exception as error:
            self.error = repr(error)
        finally:
            self.ready = False

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.)
            if self._thread.is_alive():
                raise TimeoutError('camera source did not stop')
        self.ready = False


class SyntheticCamera(_ThreadCamera):
    """Moving deterministic grey pattern; no claim of real terrain imagery."""
    def __init__(self, width=1456, height=1088, period_s=.2):
        super().__init__()
        self.width, self.height, self.period_s = width, height, period_s

    def _run(self):
        rows, cols = np.indices((self.height, self.width), dtype=np.uint16)
        frame = 0
        while not self._stop.is_set():
            local_s = time.monotonic()
            image = ((rows * 3 + cols * 7 + frame * 11) % 256).astype(np.uint8)
            self.callback(image, local_s)
            frame += 1
            self._stop.wait(max(0., self.period_s - (time.monotonic() - local_s)))


class ReplayCamera(_ThreadCamera):
    """Stream a writer frames.jsonl with rebased receive times; no pickle.

    Reads one index row/image at a time. Index paths must stay in its directory.
    An original 'local_s' is necessary for meaningful relative receive timing.
    """
    def __init__(self, index):
        super().__init__()
        self.index = Path(index).resolve()

    def _run(self):
        base = None
        started = time.monotonic()
        previous = -float('inf')
        with self.index.open(encoding='utf-8') as stream:
            for line in stream:
                if self._stop.is_set():
                    return
                row = json.loads(line)
                local = float(row['local_s'])
                if not np.isfinite(local) or local < previous:
                    raise ValueError('replay receive times must be finite and ordered')
                previous = local
                if base is None:
                    base = local
                due = started + local - base
                if self._stop.wait(max(0., due - time.monotonic())):
                    return
                path = (self.index.parent / row['file']).resolve()
                if not path.is_relative_to(self.index.parent):
                    raise ValueError('frame path escapes replay directory')
                image = np.load(path, allow_pickle=False)
                if image.ndim != 2 or image.dtype not in (np.uint8, np.uint16):
                    raise ValueError('replay image must be 2-D uint8/uint16')
                self.callback(image, due)


class PicameraCamera:
    def __init__(self, config):
        self.config = config
        self.ready = False
        self.error = None
        self._camera = None

    def start(self, callback):
        # Provisioned at boot by the deployment helper, never silently free-run.
        if Path(self.config.trigger_mode_path).read_text().strip() != '1':
            raise RuntimeError('IMX296 external trigger_mode must be 1 before start')
        from picamera2 import Picamera2
        camera = self._camera = Picamera2()
        try:
            if 'imx296' not in camera.camera_properties.get('Model', '').lower():
                raise RuntimeError('IMX296 required for calibrated external-trigger path')
            controls = {'AeEnable': False, 'AwbEnable': False,
                        'ExposureTime': self.config.exposure_us, 'AnalogueGain': self.config.gain,
                        'ColourGains': (1., 1.)}
            # Mono variants need not advertise AWB controls.
            controls = {k: v for k, v in controls.items() if k in camera.camera_controls}
            config = camera.create_video_configuration(
                main={'size': (self.config.width, self.config.height), 'format': 'YUV420'},
                raw=None, controls=controls, buffer_count=4, queue=False)
            camera.configure(config)
            size = camera.camera_configuration()['main']['size']
            if tuple(size) != (self.config.width, self.config.height):
                raise RuntimeError('camera silently changed calibrated output dimensions')

            def completed(request):
                received = time.monotonic()  # before copy or queueing, not dequeue time
                try:
                    # make_array copies the request; crop Y plane and stride padding.
                    image = request.make_array('main')[:self.config.height, :self.config.width].copy()
                    callback(image, received)
                except Exception as error:
                    self.error = repr(error)
                    self.ready = False

            camera.post_callback = completed
            camera.start()
            self.ready = True
        except Exception:
            camera.close()
            self._camera = None
            raise

    def stop(self):
        self.ready = False
        if self._camera is not None:
            self._camera.post_callback = None
            try:
                self._camera.stop()
            finally:
                self._camera.close()
                self._camera = None


def create(config):
    if config.kind == 'synthetic':
        return SyntheticCamera(config.width, config.height, config.period_s)
    if config.kind == 'replay':
        return ReplayCamera(config.replay_index)
    return PicameraCamera(config)
