"""GPL-3.0-only. Linux CPU telemetry, sampled off the serial-service thread."""
import os
from pathlib import Path
import subprocess
import threading

from cansat import link


class HealthMonitor:
    def __init__(self):
        self.value = (-128, 255, 0)  # unavailable, rather than invented readings
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='pi-health', daemon=True)

    def start(self):
        self._thread.start()

    def __call__(self):
        return self.value

    def _run(self):
        while not self._stop.is_set():
            temp, load, flags = -128, 255, 0
            try:
                temp = round(float(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000.)
            except (OSError, ValueError):
                pass
            try:
                load = min(100, round(os.getloadavg()[0] * 100 / (os.cpu_count() or 1)))
            except (AttributeError, OSError):
                pass
            try:
                text = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                                      text=True, check=True, timeout=.5).stdout
                bits = int(text.strip().split('=')[1], 16)
                # Raspberry Pi get_throttled bits 0 current under-voltage,
                # 1 frequency-capped, 2 throttled, 3 soft temp limit; bit 16
                # under-voltage has occurred since boot (see docs). PiStatus
                # reports under-voltage *seen*, so include the sticky bit.
                flags = ((link.PIFLAG_UNDERVOLT if bits & 0x10001 else 0)
                         | (link.PIFLAG_THROTTLED if bits & 14 else 0))
            except (OSError, ValueError, IndexError, subprocess.SubprocessError):
                pass
            self.value = (max(-128, min(127, temp)), load, flags)
            self._stop.wait(1.)

    def stop(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(1.)
