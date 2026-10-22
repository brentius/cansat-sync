"""GPL-3.0-only. LoRa packet airtime, duty-cycle accounting and link budget.

Airtime follows the Semtech SX1276/77/78/79 datasheet, section 4.1.1.7.
Sensitivity uses the Semtech AN1200.22 noise-floor form with an assumed
receiver noise figure; it is a planning estimate, not a measured figure.
"""
import math
from collections import deque

# Demodulator SNR limits in dB per spreading factor (SX1276 datasheet, table 13).
SNR_LIMIT_DB = {6: -5.0, 7: -7.5, 8: -10.0, 9: -12.5, 10: -15.0, 11: -17.5, 12: -20.0}
SPEED_OF_LIGHT = 299792458.0


def symbol_time(sf, bw_hz):
    return 2**sf / bw_hz


def low_data_rate_required(sf, bw_hz):
    """Datasheet mandates the optimisation when a symbol exceeds 16 ms."""
    return symbol_time(sf, bw_hz) > 16e-3


def payload_symbols(payload_bytes, sf, coding_rate=1, crc=True, implicit_header=False, ldro=False):
    """coding_rate 1..4 means 4/5..4/8."""
    numerator = 8*payload_bytes - 4*sf + 28 + 16*bool(crc) - 20*bool(implicit_header)
    blocks = math.ceil(numerator / (4*(sf - 2*bool(ldro))))
    return 8 + max(blocks*(coding_rate + 4), 0)


def time_on_air(payload_bytes, sf=7, bw_hz=125e3, coding_rate=1, preamble=8,
                crc=True, implicit_header=False, ldro=None):
    """Seconds on air for one packet."""
    if not 6 <= sf <= 12 or not 1 <= coding_rate <= 4 or payload_bytes < 0 or bw_hz <= 0:
        raise ValueError('invalid LoRa parameters')
    if ldro is None:
        ldro = low_data_rate_required(sf, bw_hz)
    symbols = preamble + 4.25 + payload_symbols(payload_bytes, sf, coding_rate, crc, implicit_header, ldro)
    return symbols * symbol_time(sf, bw_hz)


def duty_cycle(airtime_s, interval_s):
    return airtime_s / interval_s


def min_interval(airtime_s, duty_limit):
    """Shortest steady repeat interval that stays inside a duty-cycle limit."""
    return airtime_s / duty_limit


def sensitivity_dbm(sf, bw_hz, noise_figure_db=6.0):
    """Thermal floor + noise figure + demodulator SNR limit."""
    return -174.0 + 10*math.log10(bw_hz) + noise_figure_db + SNR_LIMIT_DB[sf]


def free_space_loss_db(distance_m, frequency_hz):
    return 20*math.log10(4*math.pi*distance_m*frequency_hz/SPEED_OF_LIGHT)


def link_margin_db(distance_m, frequency_hz, tx_dbm, sf, bw_hz,
                   tx_gain_dbi=0.0, rx_gain_dbi=0.0, other_loss_db=0.0, noise_figure_db=6.0):
    """Free-space margin above estimated sensitivity. Negative means no link."""
    received = tx_dbm + tx_gain_dbi + rx_gain_dbi - free_space_loss_db(distance_m, frequency_hz) - other_loss_db
    return received - sensitivity_dbm(sf, bw_hz, noise_figure_db)


class DutyCycleLimiter:
    """Rolling-window transmit budget.

    The regulatory duty cycle is assessed over an observation period, taken
    here as one hour. Call allowed() before a transmission and record() after.
    Time is any monotonic clock in seconds.
    """

    def __init__(self, duty_limit, window_s=3600.0):
        if not 0 < duty_limit <= 1 or window_s <= 0:
            raise ValueError('duty limit must be in (0, 1] and window positive')
        self.duty_limit = duty_limit
        self.window_s = window_s
        self._sent = deque()
        self._used = 0.0

    def _expire(self, now):
        while self._sent and self._sent[0][0] <= now - self.window_s:
            self._used -= self._sent.popleft()[1]
        if not self._sent:
            self._used = 0.0

    def used(self, now):
        self._expire(now)
        return self._used

    def remaining(self, now):
        return self.duty_limit*self.window_s - self.used(now)

    def allowed(self, now, airtime_s, reserve_s=0.0):
        """reserve_s keeps airtime back for a later high-rate phase."""
        return airtime_s <= self.remaining(now) - reserve_s + 1e-12

    def record(self, now, airtime_s):
        self._expire(now)
        self._sent.append((now, airtime_s))
        self._used += airtime_s
