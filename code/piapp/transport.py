"""GPL-3.0-only. Nonblocking UART and paced byte replay for PI-SHELL.

Only Service touches this interface: read(size), write(bytes), drain(timeout),
close(). Reads must return promptly; writes may be partial. pyserial is lazy.
"""
from pathlib import Path
import time


class SerialPort:
    def __init__(self, port='/dev/ttyAMA0', baud=230400):
        import serial  # Pi OS dependency; intentionally absent on host tests
        self.port = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=.02,
                                  bytesize=8, parity='N', stopbits=1,
                                  xonxoff=False, rtscts=False, dsrdtr=False,
                                  exclusive=True)

    def read(self, size=4096):
        return self.port.read(size)

    def write(self, data):
        return self.port.write(data)

    def drain(self, timeout=1.):
        # pyserial.flush/tcdrain can wait indefinitely after a driver fault.
        deadline = time.monotonic() + timeout
        while self.port.out_waiting:
            if time.monotonic() >= deadline:
                return False
            time.sleep(.005)
        return True

    def close(self):
        self.port.close()


class ReplaySerial:
    """Raw RX replay at UART byte rate; not a timestamp-faithful flight replay.

    Pi TX bytes are recorded by Service's writer, not fed back into RX. This
    permits CLI bench execution without hardware or importing pyserial.
    """
    def __init__(self, path, baud=230400, *, monotonic=time.monotonic):
        self._file = Path(path).open('rb')
        self._clock = monotonic
        self._last = monotonic()
        self._credit = 0.
        self._rate = baud / 10.
        self.eof = False

    def read(self, size=4096):
        now = self._clock()
        self._credit = min(float(size), self._credit + (now - self._last) * self._rate)
        self._last = now
        count = min(size, int(self._credit))
        if not count:
            return b''
        data = self._file.read(count)
        self._credit -= len(data)
        self.eof = not data
        return data

    def write(self, data):
        return len(data)

    def drain(self, timeout=1.):
        return True

    def close(self):
        self._file.close()
