"""GPL-3.0-only. STM32 <-> Raspberry Pi serial link: framing, messages, time base, IMU stream.

The two flight processors are joined by one buffered 3.3 V UART and nothing
else (wiki/flight_computer_connections.md section 9): no flow control, no
trigger or PPS wire to the Pi. This module is the reference codec for that
link and the executable specification the STM32 firmware has to match byte
for byte (code/tests/link_vectors.json). The prose specification, with the
reasons and the numbers, is wiki/stm32_pi_link_protocol.md.

Which processor runs what:

    Raspberry Pi Zero 2 W   this module as it stands: Decoder for the STM32's
                            stream, ImuDifferencer, TickClock, FrameStamper,
                            and Encoder for PiStatus, Nav, Ack, TrigCfg
    STM32F405               a C implementation that does not exist yet. The
                            encode side here (Encoder, ImuAccumulator) is the
                            model it must reproduce; it is not flight code
    ground tools, tests     everything

What each side needs from the other:

    the Pi needs     every sensor it navigates with (it has none of its own
                     except the camera): Imu, Baro, Gnss; the time of every
                     camera exposure (Trig), since CSI frames carry no hardware
                     time; the flight state and pad references (Status)
    the STM32 needs  the TRN and landing-prediction fields of the 40-byte
                     packet (Nav), a sign of life (any valid frame; PiStatus at
                     2 Hz guarantees one), the exposure the camera wants
                     (TrigCfg) and an acknowledgement before it removes power
                     (Ack)

Frame, on the wire (STM32 USART1 <-> Pi PL011, 8N1, 230400 baud proposed):

    COBS( id | seq | payload | crc_lo | crc_hi )  0x00

    id       message identifier; bit 7 set means Pi -> STM32
    seq      one counter per direction, +1 per frame of any kind, wraps at 256
    payload  fixed length per id, little-endian, at most 249 bytes
    crc      CRC-16/CCITT-FALSE (telemetry.crc16) over id, seq and payload
    COBS     consistent-overhead byte stuffing; a frame is at most 253 bytes,
             so stuffing adds exactly one byte and the code 0xFF never occurs
    0x00     delimiter. It appears nowhere else, so a receiver that has lost
             its place is in step again at the next delimiter

Overhead is 6 bytes per message. Integers are little-endian. "NA" is the
most negative value of a signed field and the largest of an unsigned one;
scaled values saturate one step short of it. Scaled values are rounded with
floor(x/step + 0.5).

Time base. `tick` is the STM32's TIM2 counter: 32 bits at 1 MHz, free
running, wrapping every 4294.967296 s (71.6 min). Every time in every message
is a TIM2 value, captured in hardware where a pin exists (PPS, camera
trigger, IMU data-ready). The Pi never needs the high word to navigate
(unwrap32); Hello and Status carry it for the logs.

Payload layouts (offset, size, field, unit or coding).

  0x01 Hello, STM32 -> Pi, 16 bytes, at boot and when the Pi (re)appears
    0  1  proto        protocol version, 1
    1  1  epoch        boot counter kept in a backup register
    2  2  fw_version
    4  1  reset_cause  RESET_* bits
    5  1  flags        reserved, 0
    6  2  tick_hi      wraps of TIM2 since boot
    8  4  tick
    12 4  tick_hz      1000000

  0x10 Imu, STM32 -> Pi, 36 bytes, 100 Hz: a snapshot of running integrals
    0  4  tick         time of the last gyro sample included
    4  16 q            w, x, y, z of the gyro-integrated attitude, Q30
    20 12 u            specific force integrated in the q frame, 2^-16 m/s,
                       32-bit wrapping
    32 1  epoch        changes whenever q and u restart
    33 1  flags        IMU_* bits
    34 1  n_gyro       gyro samples since the previous snapshot
    35 1  n_accel      accelerometer samples since the previous snapshot

  0x11 Baro, STM32 -> Pi, 12 bytes, 10 Hz
    0  4  tick         mid-time of the samples averaged
    4  4  pressure     0.01 Pa
    8  2  temperature  0.01 degC, signed (sensor die)
    10 1  n_samples
    11 1  flags        bit 0 valid

  0x12 Gnss, STM32 -> Pi, 60 bytes, per navigation epoch (10 Hz budgeted).
  Fields and units are those of UBX-NAV-PVT, copied without arithmetic.
    0  4  tick         time of the navigation epoch (GNSS_TICK_FROM_PPS says how)
    4  4  itow_ms
    8  2  year         UTC
    10 5  month, day, hour, minute, second
    15 1  valid        UBX valid bits
    16 1  fix_type     0 none, 2 2D, 3 3D
    17 1  fix_flags    UBX flags (bit 0 gnssFixOK)
    18 1  num_sv
    19 1  link_flags   GNSS_* bits
    20 4  lon          1e-7 deg, signed
    24 4  lat          1e-7 deg, signed
    28 4  h_msl        mm, signed
    32 4  h_acc        mm
    36 4  v_acc        mm
    40 12 vel_n, vel_e, vel_d   mm/s, signed
    52 4  s_acc        mm/s
    56 2  p_dop        0.01
    58 2  reserved

  0x13 Trig, STM32 -> Pi, 16 bytes, one per camera trigger (5 Hz)
    0  4  frame        trigger counter since triggers last started
    4  4  tick         TIM2 capture of the falling edge (exposure start)
    8  4  period_us    trigger period; lost Trig messages are rebuilt from it
    12 2  pulse_us     low time; exposure = pulse + 14.26 us
    14 1  flags        bit 0 first trigger after a start
    15 1  reserved

  0x14 Pps, STM32 -> Pi, 16 bytes, 1 Hz while the receiver gives a pulse
    0  4  tick         TIM2 capture of the rising edge
    4  4  itow_ms      GNSS time of week of that edge, or NA
    8  4  interval     ticks since the previous pulse, 0 if it was missed
    12 2  count
    14 1  flags        bit 0 itow valid
    15 1  reserved

  0x15 Status, STM32 -> Pi, 48 bytes, 1 Hz and at once on a state change
    0  4  tick
    4  2  tick_hi
    6  1  epoch
    7  1  state        flight_state.State
    8  4  tick_state   tick at which that state was entered
    12 2  health       HEALTH_* bits
    14 2  battery      mV
    16 4  baseline     pad pressure, 0.01 Pa, or NA
    20 12 origin lat, lon (1e-7 deg), h_msl (mm); valid if HEALTH_ORIGIN
    32 2  altitude     0.1 m above pad, signed
    34 2  temperature  0.01 degC, signed (thermistor, primary mission)
    36 2  telemetry_seq
    38 2  rx_crc       frames from the Pi that failed
    40 2  rx_lost      frames from the Pi missing by sequence number
    42 2  tx_dropped   frames the STM32 could not queue
    44 1  pi_age       0.1 s since the last valid frame from the Pi, 255 NA
    45 1  mcu_temp     degC, signed
    46 2  reserved

  0x16 Cmd, STM32 -> Pi, 8 bytes, repeated at 2 Hz until acknowledged
    0  4  tick
    4  1  cmd_seq
    5  1  cmd          CMD_* value
    6  2  arg          CMD_SHUTDOWN: seconds until power is removed regardless

  0x80 PiStatus, Pi -> STM32, 24 bytes, 2 Hz (the heartbeat)
    0  4  uptime_ms
    4  4  tick_echo    tick of the newest STM32 frame that carried one
    8  2  echo_age     ms between receiving it and sending this
    10 1  proto
    11 1  state        PI_* value
    12 2  flags        PIFLAG_* bits
    14 2  sw_version
    16 2  rx_crc
    18 2  rx_lost
    20 2  frames       camera frames received, wraps
    22 1  cpu_temp     degC, signed
    23 1  cpu_load     percent

  0x81 Nav, Pi -> STM32, 32 bytes, 5 Hz. Shared fields use the coding of the
  40-byte packet so the STM32 copies them.
    0  4  tick         time the solution is valid at
    4  2  frame        trigger counter of the newest image used
    6  1  flags        telemetry FLAG_TRN_VALID | FLAG_PRED_VALID | FLAG_CAMERA_OK
    7  1  landmarks
    8  6  trn e, n, u  0.1 m, signed
    14 2  trn ellipse major, minor   0.1 m
    16 1  trn ellipse angle          180/256 deg
    17 1  protection   0.5 m
    18 4  predicted e, n   0.1 m, signed
    22 1  time to go   0.2 s
    23 2  predicted ellipse major, minor   0.5 m
    25 1  predicted ellipse angle
    26 6  velocity e, n, u   0.01 m/s, signed

  0x82 Ack, Pi -> STM32, 4 bytes
    0  1  cmd_seq
    1  1  cmd
    2  1  result       ACK_* value
    3  1  reserved

  0x83 TrigCfg, Pi -> STM32, 8 bytes, on change and repeated at 1 Hz
    0  1  enable
    1  1  cfg_seq
    2  2  period_ms
    4  2  pulse_us
    6  2  reserved

Written not to stop: Decoder.feed, ImuDifferencer.feed, TickClock.update and
FrameStamper.stamp never raise in the flight loop, hold a bounded amount of
memory and count every kind of fault instead. Standard library only.

Design-stage code. It has been executed on a development laptop only: not on
the Pi, and no STM32 firmware exists.
"""
import math
import struct
from dataclasses import dataclass, fields

from .telemetry import FLAG_CAMERA_OK, FLAG_PRED_VALID, FLAG_TRN_VALID, Telemetry, crc16

PROTO_VERSION = 1
TICK_HZ = 1_000_000
TICK_WRAP = 1 << 32
MAX_PAYLOAD = 249
MAX_FRAME = MAX_PAYLOAD + 4          # id, seq, payload, crc: before stuffing
MAX_WIRE = MAX_FRAME + 2             # one stuffing byte and the delimiter
OVERHEAD = 6
NAN = float('nan')

Q_ONE = 1 << 30                      # Imu.q scale
U_LSB = 2.0**-16                     # m/s per count of Imu.u
EXPOSURE_EXTRA_US = 14.26            # Raspberry Pi camera documentation, external trigger

RESET_POWER, RESET_PIN, RESET_WATCHDOG, RESET_SOFTWARE, RESET_BROWNOUT = 1, 2, 4, 8, 16
IMU_GYRO_SATURATED, IMU_ACCEL_SATURATED, IMU_GYRO_MISSED, IMU_ACCEL_MISSED = 1, 2, 4, 8
IMU_RESTART, IMU_GYRO_FAULT, IMU_ACCEL_FAULT = 16, 32, 64
GNSS_TICK_FROM_PPS, GNSS_STALE = 1, 2
(HEALTH_5V, HEALTH_3V3, HEALTH_PI_POWER, HEALTH_UART, HEALTH_IMU, HEALTH_BARO, HEALTH_GNSS, HEALTH_FIX,
 HEALTH_PPS, HEALTH_RADIO, HEALTH_FLASH, HEALTH_TEMP, HEALTH_TRIGGERS, HEALTH_ORIGIN, HEALTH_BASELINE,
 HEALTH_PI_ALIVE) = (1 << i for i in range(16))
CMD_SHUTDOWN, CMD_PREPARE = 1, 2
ACK_ACCEPTED, ACK_DONE, ACK_REFUSED, ACK_UNKNOWN = 0, 1, 2, 3
PI_BOOTING, PI_READY, PI_CAMERA, PI_NAVIGATING, PI_STOPPING, PI_FAULT = range(6)
PIFLAG_CAMERA, PIFLAG_MAP, PIFLAG_STORAGE, PIFLAG_UNDERVOLT, PIFLAG_THROTTLED, PIFLAG_NAV = 1, 2, 4, 8, 16, 32

_RANGE = {'B': (0, 0xFF), 'b': (-0x80, 0x7F), 'H': (0, 0xFFFF), 'h': (-0x8000, 0x7FFF),
          'I': (0, 0xFFFFFFFF), 'i': (-0x80000000, 0x7FFFFFFF)}
WRAP = 'wrap'


# --- CRC and byte stuffing ----------------------------------------------------------

def _crc_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC = _crc_table()


def crc(data, value=0xFFFF):
    """telemetry.crc16 by table: CRC-16/CCITT-FALSE."""
    table = _CRC
    for byte in data:
        value = ((value << 8) & 0xFFFF) ^ table[(value >> 8) ^ byte]
    return value


def cobs_encode(data):
    out = bytearray()
    for i, chunk in enumerate(bytes(data).split(b'\x00')):
        while len(chunk) >= 254:
            out.append(0xFF)
            out += chunk[:254]
            chunk = chunk[254:]
        out.append(len(chunk) + 1)
        out += chunk
    return bytes(out)


def cobs_decode(data):
    """The stuffed bytes between two delimiters, or None if they are not a valid encoding."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        j = i + code
        if code == 0 or j > n:
            return None
        out += data[i + 1:j]
        i = j
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def frame(msg_id, seq, payload):
    """Wire bytes of one frame, delimiter included."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too long')
    body = bytes((msg_id & 0xFF, seq & 0xFF)) + bytes(payload)
    return cobs_encode(body + struct.pack('<H', crc(body))) + b'\x00'


# --- Field coding -------------------------------------------------------------------

def _na(code):
    lo, hi = _RANGE[code]
    return lo if lo < 0 else hi


def _integer(value, code, wrap=False):
    lo, hi = _RANGE[code]
    try:
        value = int(value)
    except (TypeError, ValueError, OverflowError):
        return 0
    if wrap:
        value &= hi - lo
        return value - (hi - lo + 1) if value > hi else value
    return max(lo, min(hi, value))


def _scaled(value, code, step):
    lo, hi = _RANGE[code]
    try:
        value = float(value)
    except (TypeError, ValueError):
        return _na(code)
    if not math.isfinite(value):
        return _na(code)
    raw = math.floor(value/step + 0.5)
    return max(lo + 1, min(hi, raw)) if lo < 0 else max(0, min(hi - 1, raw))


def _angle(value):
    """Ellipse angle in [0, pi) as 0..255, as in the telemetry packet."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0
    if not math.isfinite(value):
        return 0
    return math.floor((value % math.pi)/math.pi*256 + 0.5) % 256


class _Message:
    """Base of the message dataclasses. LAYOUT rows are (field, struct code, kind):
    kind None is a clamped integer, WRAP a wrapping one, 'angle' an ellipse
    angle and a number the step of a scaled value. A field of None is padding."""
    ID = 0
    LAYOUT = ()

    def raw(self):
        """Field name -> the integer (or list of integers) that goes on the wire."""
        out = {}
        for name, code, kind in self.LAYOUT:
            if name is None:
                continue
            value = getattr(self, name)
            if len(code) > 1:
                try:
                    items = list(value)
                except TypeError:
                    items = []
                items = (items + [0]*int(code[:-1]))[:int(code[:-1])]
                out[name] = [_integer(v, code[-1], kind == WRAP) for v in items]
            elif kind is None or kind == WRAP:
                out[name] = _integer(value, code, kind == WRAP)
            elif kind == 'angle':
                out[name] = _angle(value)
            else:
                out[name] = _scaled(value, code, kind)
        return out

    def payload(self):
        values = []
        for item in self.raw().values():
            values.extend(item) if isinstance(item, list) else values.append(item)
        return self.STRUCT.pack(*values)

    @classmethod
    def from_raw(cls, raw):
        kwargs = {}
        for name, code, kind in cls.LAYOUT:
            if name is None:
                continue
            value = raw[name]
            if len(code) > 1:
                kwargs[name] = tuple(value)
            elif kind is None or kind == WRAP:
                kwargs[name] = value
            elif kind == 'angle':
                kwargs[name] = value*math.pi/256
            else:
                kwargs[name] = NAN if value == _na(code) else value*kind
        return cls(**kwargs)

    @classmethod
    def from_payload(cls, payload):
        values = cls.STRUCT.unpack(payload)
        raw, i = {}, 0
        for name, code, _ in cls.LAYOUT:
            if name is None:
                continue
            count = int(code[:-1]) if len(code) > 1 else 0
            raw[name] = list(values[i:i + count]) if count else values[i]
            i += count or 1
        return cls.from_raw(raw)


MESSAGES = {}


def _message(msg_id, layout):
    def wrap(cls):
        cls = dataclass(cls)
        cls.ID = msg_id
        cls.LAYOUT = layout
        cls.STRUCT = struct.Struct('<' + ''.join(('%dx' % int(c[:-1] or 1)) if n is None else c for n, c, _ in layout))
        cls.SIZE = cls.STRUCT.size
        names = [n for n, _, _ in layout if n is not None]
        if names != [f.name for f in fields(cls)]:
            raise TypeError('%s: layout and fields disagree' % cls.__name__)
        MESSAGES[msg_id] = cls
        return cls
    return wrap


@_message(0x01, (('proto', 'B', None), ('epoch', 'B', WRAP), ('fw_version', 'H', None), ('reset_cause', 'B', None),
                 ('flags', 'B', None), ('tick_hi', 'H', WRAP), ('tick', 'I', WRAP), ('tick_hz', 'I', None)))
class Hello(_Message):
    proto: int = PROTO_VERSION
    epoch: int = 0
    fw_version: int = 0
    reset_cause: int = 0
    flags: int = 0
    tick_hi: int = 0
    tick: int = 0
    tick_hz: int = TICK_HZ


@_message(0x10, (('tick', 'I', WRAP), ('q', '4i', None), ('u', '3i', WRAP), ('epoch', 'B', WRAP), ('flags', 'B', None),
                 ('n_gyro', 'B', None), ('n_accel', 'B', None)))
class Imu(_Message):
    tick: int = 0
    q: tuple = (Q_ONE, 0, 0, 0)
    u: tuple = (0, 0, 0)
    epoch: int = 0
    flags: int = 0
    n_gyro: int = 0
    n_accel: int = 0


@_message(0x11, (('tick', 'I', WRAP), ('pressure_pa', 'I', 0.01), ('temperature_c', 'h', 0.01), ('n_samples', 'B', None),
                 ('flags', 'B', None)))
class Baro(_Message):
    tick: int = 0
    pressure_pa: float = NAN
    temperature_c: float = NAN
    n_samples: int = 0
    flags: int = 0


@_message(0x12, (('tick', 'I', WRAP), ('itow_ms', 'I', None), ('year', 'H', None), ('month', 'B', None), ('day', 'B', None),
                 ('hour', 'B', None), ('minute', 'B', None), ('second', 'B', None), ('valid', 'B', None),
                 ('fix_type', 'B', None), ('fix_flags', 'B', None), ('num_sv', 'B', None), ('link_flags', 'B', None),
                 ('lon_deg', 'i', 1e-7), ('lat_deg', 'i', 1e-7), ('h_msl_m', 'i', 1e-3), ('h_acc_m', 'I', 1e-3),
                 ('v_acc_m', 'I', 1e-3), ('vel_n', 'i', 1e-3), ('vel_e', 'i', 1e-3), ('vel_d', 'i', 1e-3),
                 ('s_acc', 'I', 1e-3), ('p_dop', 'H', 0.01), (None, '2x', None)))
class Gnss(_Message):
    tick: int = 0
    itow_ms: int = 0
    year: int = 0
    month: int = 0
    day: int = 0
    hour: int = 0
    minute: int = 0
    second: int = 0
    valid: int = 0
    fix_type: int = 0
    fix_flags: int = 0
    num_sv: int = 0
    link_flags: int = 0
    lon_deg: float = NAN
    lat_deg: float = NAN
    h_msl_m: float = NAN
    h_acc_m: float = NAN
    v_acc_m: float = NAN
    vel_n: float = NAN
    vel_e: float = NAN
    vel_d: float = NAN
    s_acc: float = NAN
    p_dop: float = NAN

    @property
    def usable(self):
        """A 3D fix the receiver itself calls good."""
        return self.fix_type == 3 and bool(self.fix_flags & 1) and math.isfinite(self.vel_n + self.vel_e + self.lat_deg)


@_message(0x13, (('frame', 'I', WRAP), ('tick', 'I', WRAP), ('period_us', 'I', None), ('pulse_us', 'H', None),
                 ('flags', 'B', None), (None, '1x', None)))
class Trig(_Message):
    frame: int = 0
    tick: int = 0
    period_us: int = 200_000
    pulse_us: int = 0
    flags: int = 0


@_message(0x14, (('tick', 'I', WRAP), ('itow_ms', 'I', None), ('interval', 'I', None), ('count', 'H', WRAP),
                 ('flags', 'B', None), (None, '1x', None)))
class Pps(_Message):
    tick: int = 0
    itow_ms: int = 0xFFFFFFFF
    interval: int = 0
    count: int = 0
    flags: int = 0


@_message(0x15, (('tick', 'I', WRAP), ('tick_hi', 'H', WRAP), ('epoch', 'B', WRAP), ('state', 'B', None),
                 ('tick_state', 'I', WRAP), ('health', 'H', None), ('battery_v', 'H', 1e-3), ('baseline_pa', 'I', 0.01),
                 ('origin_lat_deg', 'i', 1e-7), ('origin_lon_deg', 'i', 1e-7), ('origin_h_msl_m', 'i', 1e-3),
                 ('altitude_m', 'h', 0.1), ('temperature_c', 'h', 0.01), ('telemetry_seq', 'H', WRAP),
                 ('rx_crc', 'H', None), ('rx_lost', 'H', None), ('tx_dropped', 'H', None), ('pi_age_s', 'B', 0.1),
                 ('mcu_temp_c', 'b', None), (None, '2x', None)))
class Status(_Message):
    tick: int = 0
    tick_hi: int = 0
    epoch: int = 0
    state: int = 0
    tick_state: int = 0
    health: int = 0
    battery_v: float = NAN
    baseline_pa: float = NAN
    origin_lat_deg: float = NAN
    origin_lon_deg: float = NAN
    origin_h_msl_m: float = NAN
    altitude_m: float = NAN
    temperature_c: float = NAN
    telemetry_seq: int = 0
    rx_crc: int = 0
    rx_lost: int = 0
    tx_dropped: int = 0
    pi_age_s: float = NAN
    mcu_temp_c: int = 0


@_message(0x16, (('tick', 'I', WRAP), ('cmd_seq', 'B', WRAP), ('cmd', 'B', None), ('arg', 'H', None)))
class Cmd(_Message):
    tick: int = 0
    cmd_seq: int = 0
    cmd: int = 0
    arg: int = 0


@_message(0x80, (('uptime_ms', 'I', WRAP), ('tick_echo', 'I', WRAP), ('echo_age_ms', 'H', None), ('proto', 'B', None),
                 ('state', 'B', None), ('flags', 'H', None), ('sw_version', 'H', None), ('rx_crc', 'H', None),
                 ('rx_lost', 'H', None), ('frames', 'H', WRAP), ('cpu_temp_c', 'b', None), ('cpu_load', 'B', None)))
class PiStatus(_Message):
    uptime_ms: int = 0
    tick_echo: int = 0
    echo_age_ms: int = 0
    proto: int = PROTO_VERSION
    state: int = PI_BOOTING
    flags: int = 0
    sw_version: int = 0
    rx_crc: int = 0
    rx_lost: int = 0
    frames: int = 0
    cpu_temp_c: int = 0
    cpu_load: int = 0


@_message(0x81, (('tick', 'I', WRAP), ('frame', 'H', WRAP), ('flags', 'B', None), ('landmarks', 'B', None),
                 ('trn_e', 'h', 0.1), ('trn_n', 'h', 0.1), ('trn_u', 'h', 0.1),
                 ('trn_major_m', 'B', 0.1), ('trn_minor_m', 'B', 0.1), ('trn_angle_rad', 'B', 'angle'),
                 ('protection_m', 'B', 0.5), ('pred_e', 'h', 0.1), ('pred_n', 'h', 0.1), ('time_to_go_s', 'B', 0.2),
                 ('pred_major_m', 'B', 0.5), ('pred_minor_m', 'B', 0.5), ('pred_angle_rad', 'B', 'angle'),
                 ('vel_e', 'h', 0.01), ('vel_n', 'h', 0.01), ('vel_u', 'h', 0.01)))
class Nav(_Message):
    tick: int = 0
    frame: int = 0
    flags: int = 0
    landmarks: int = 0
    trn_e: float = NAN
    trn_n: float = NAN
    trn_u: float = NAN
    trn_major_m: float = NAN
    trn_minor_m: float = NAN
    trn_angle_rad: float = 0.0
    protection_m: float = NAN
    pred_e: float = NAN
    pred_n: float = NAN
    time_to_go_s: float = NAN
    pred_major_m: float = NAN
    pred_minor_m: float = NAN
    pred_angle_rad: float = 0.0
    vel_e: float = NAN
    vel_n: float = NAN
    vel_u: float = NAN


@_message(0x82, (('cmd_seq', 'B', WRAP), ('cmd', 'B', None), ('result', 'B', None), (None, '1x', None)))
class Ack(_Message):
    cmd_seq: int = 0
    cmd: int = 0
    result: int = ACK_ACCEPTED


@_message(0x83, (('enable', 'B', None), ('cfg_seq', 'B', WRAP), ('period_ms', 'H', None), ('pulse_us', 'H', None),
                 (None, '2x', None)))
class TrigCfg(_Message):
    enable: int = 0
    cfg_seq: int = 0
    period_ms: int = 200
    pulse_us: int = 0


@dataclass
class Raw:
    """A frame with a good CRC whose id this version does not know."""
    msg_id: int = 0
    data: bytes = b''


def apply_nav(telemetry, nav, max_age_s=None, now_tick=None):
    """What the STM32 does with a Nav message: copy it into the 40-byte packet.

    Returns a new Telemetry; other fields are left as they were. With
    max_age_s and now_tick, a solution older than that is not copied and its
    flags are cleared (a Pi that has stopped must not leave a stale fix in the
    downlink).
    """
    names = ('trn_e', 'trn_n', 'trn_major_m', 'trn_minor_m', 'trn_angle_rad', 'protection_m', 'landmarks',
             'pred_e', 'pred_n', 'time_to_go_s', 'pred_major_m', 'pred_minor_m', 'pred_angle_rad')
    mask = FLAG_TRN_VALID | FLAG_PRED_VALID | FLAG_CAMERA_OK
    out = Telemetry(**telemetry.as_dict())
    out.flags &= ~mask
    stale = (nav is not None and max_age_s is not None and now_tick is not None
             and ((now_tick - nav.tick) % TICK_WRAP)/TICK_HZ > max_age_s)
    if nav is None or stale:
        for name in names:
            setattr(out, name, 0 if name == 'landmarks' else (0.0 if name.endswith('angle_rad') else NAN))
        return out
    for name in names:
        setattr(out, name, getattr(nav, name))
    out.flags |= nav.flags & mask
    return out


# --- Encoder and streaming decoder ---------------------------------------------------

def encode(message, seq=0):
    return frame(message.ID, seq, message.payload())


class Encoder:
    """One direction of the link: numbers the frames."""

    def __init__(self):
        self.seq = 0

    def encode(self, message):
        wire = frame(message.ID, self.seq, message.payload())
        self.seq = (self.seq + 1) & 0xFF
        return wire


COUNTERS = ('bytes', 'frames', 'empty', 'too_long', 'stuffing', 'short', 'crc', 'unknown', 'length', 'lost',
            'repeats', 'ignored', 'faults')


class Decoder:
    """Streaming decoder for one direction. feed() takes any bytes in any chunking.

    It never raises, keeps at most MAX_WIRE bytes between calls and counts:

        bytes     bytes fed            frames    frames accepted (good CRC)
        empty     consecutive delimiters (idle line, break: harmless)
        too_long  runs longer than a frame can be
        stuffing  invalid byte stuffing
        short     fewer than 4 bytes after unstuffing
        crc       CRC mismatch
        unknown   good CRC, id not known to this version
        length    good CRC, known id, wrong payload length
        lost      frames missing by sequence number (modulo 256)
        repeats   a sequence number seen twice running (the sender restarted)
        ignored   feed() given something that is not bytes
        faults    anything unexpected, caught

    After any damage it is in step at the next delimiter: the first frame
    that starts after a delimiter following the last damaged byte is decoded.
    """

    def __init__(self, raw_unknown=False):
        self.raw_unknown = raw_unknown
        self._buf = bytearray()
        self._skip = False
        self._last_seq = None
        for name in COUNTERS:
            setattr(self, name, 0)

    def stats(self):
        return {name: getattr(self, name) for name in COUNTERS}

    def reset_sequence(self):
        """Forget the last sequence number (the peer is known to have restarted)."""
        self._last_seq = None

    def feed(self, data):
        out = []
        try:
            data = bytes(data)
        except Exception:
            self.ignored += 1
            return out
        try:
            self.bytes += len(data)
            parts = (bytes(self._buf) + data).split(b'\x00')
            tail = parts.pop()
            for part in parts:
                if self._skip:
                    self._skip = False          # the rest of a run already counted as too long
                elif not part:
                    self.empty += 1
                elif len(part) > MAX_WIRE - 1:
                    self.too_long += 1
                else:
                    message = self._frame(part)
                    if message is not None:
                        out.append(message)
            if self._skip:
                tail = b''
            elif len(tail) > MAX_WIRE - 1:
                self.too_long += 1
                self._skip = True
                tail = b''
            self._buf = bytearray(tail)
        except Exception:
            self.faults += 1
            self._buf = bytearray()
        return out

    def _frame(self, part):
        body = cobs_decode(part)
        if body is None:
            self.stuffing += 1
            return None
        if len(body) < 4:
            self.short += 1
            return None
        if len(body) > MAX_FRAME:
            self.too_long += 1
            return None
        if crc(body[:-2]) != body[-2] | body[-1] << 8:
            self.crc += 1
            return None
        self.frames += 1
        seq = body[1]
        if self._last_seq is not None:
            step = (seq - self._last_seq) & 0xFF
            if step == 0:
                self.repeats += 1
            else:
                self.lost += step - 1
        self._last_seq = seq
        cls = MESSAGES.get(body[0])
        payload = body[2:-2]
        if cls is None:
            self.unknown += 1
            message = Raw(body[0], payload) if self.raw_unknown else None
        elif len(payload) != cls.SIZE:
            self.length += 1
            return None
        else:
            message = cls.from_payload(payload)
        if message is not None:
            message.seq = seq
        return message


# --- Time base -----------------------------------------------------------------------

def unwrap32(tick, reference):
    """The integer nearest `reference` that equals `tick` modulo 2^32."""
    return reference + ((int(tick) - int(reference) + TICK_WRAP//2) % TICK_WRAP) - TICK_WRAP//2


class TickClock:
    """Pi side: extends 32-bit ticks and relates them to the Pi's own clock.

    update(tick, local_s) is called for every message that carries a tick,
    with the Pi's monotonic time at which its last byte was read. Transport
    only ever adds delay, so the smallest (local - tick) over the recent
    window is the best estimate of the clock offset; it is good to about the
    serial latency (milliseconds), which is enough to tell which trigger a
    camera frame belongs to and is not used for anything finer.
    """

    def __init__(self, window=64):
        self.window = int(window)
        self.ticks = None               # extended tick of the newest message, an integer
        self._offsets = []
        self.rejected = 0

    def update(self, tick, local_s=None):
        try:
            tick = int(tick)
            self.ticks = tick % TICK_WRAP if self.ticks is None else unwrap32(tick, self.ticks)
            if local_s is not None and math.isfinite(local_s):
                self._offsets.append(local_s - self.ticks/TICK_HZ)
                del self._offsets[:-self.window]
            return self.ticks
        except Exception:
            self.rejected += 1
            return self.ticks

    def restart(self):
        """The STM32 restarted: its counter and the offset start again."""
        self.ticks = None
        self._offsets = []

    @property
    def offset_s(self):
        return min(self._offsets) if self._offsets else None

    def extend(self, tick):
        """Extended tick of a 32-bit value near the newest message."""
        return int(tick) % TICK_WRAP if self.ticks is None else unwrap32(tick, self.ticks)

    def to_ticks(self, local_s):
        """Pi monotonic seconds -> extended ticks (float), or None before any message."""
        offset = self.offset_s
        return None if offset is None else (local_s - offset)*TICK_HZ


class FrameStamper:
    """Pi side: gives each camera frame the STM32 time of its exposure.

    Trig messages say when each trigger edge was captured. A frame arrives
    over CSI with only the Pi's own receive time, which TickClock turns into
    an approximate tick; the frame belongs to the trigger nearest that time
    less `latency_s` (exposure, readout and driver delay, to be measured on
    the bench). Triggers are periodic on the STM32's own clock, so a frame
    whose Trig message was lost is still stamped exactly from the newest one
    received and the period it carries.
    """

    def __init__(self, latency_s=0.02, tolerance=0.25):
        self.latency_s = latency_s
        self.tolerance = tolerance      # fraction of the period
        self.last = None                # (frame, extended tick, period_us, pulse_us)
        self.stamped = self.refused = self.extrapolated = 0

    def add(self, trig, clock):
        try:
            if trig.period_us <= 0:
                self.refused += 1
                return
            if trig.flags & 1:
                self.last = None
            self.last = (int(trig.frame), clock.extend(trig.tick), int(trig.period_us), int(trig.pulse_us))
        except Exception:
            self.refused += 1

    def stamp(self, frame_local_s, clock):
        """(trigger counter, extended tick of mid-exposure as a float) or None."""
        try:
            approx = clock.to_ticks(frame_local_s)
            if self.last is None or approx is None or not math.isfinite(approx):
                self.refused += 1
                return None
            frame0, tick0, period, pulse = self.last
            steps = (approx - self.latency_s*TICK_HZ - tick0)/period
            n = math.floor(steps + 0.5)
            if abs(steps - n) > self.tolerance or frame0 + n < 0:
                self.refused += 1
                return None
            self.stamped += 1
            self.extrapolated += n != 0
            return (frame0 + n) % TICK_WRAP, tick0 + n*period + (pulse + EXPOSURE_EXTRA_US)/2
        except Exception:
            self.refused += 1
            return None


# --- IMU stream ----------------------------------------------------------------------

def _qmul(a, b):
    return (a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0])


def _qexp(v):
    """Quaternion of a rotation vector."""
    angle = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    k = 0.5 - angle*angle/48 if angle < 1e-6 else math.sin(angle/2)/angle
    return (math.cos(angle/2), k*v[0], k*v[1], k*v[2])


def _qlog(q):
    """Rotation vector of a unit quaternion, by the short way round."""
    w, x, y, z = q if q[0] >= 0 else (-q[0], -q[1], -q[2], -q[3])
    s = math.sqrt(x*x + y*y + z*z)
    k = 2.0/w if s < 1e-9 else 2*math.atan2(s, w)/s
    return (k*x, k*y, k*z)


def _qrot(q, v):
    """Rotate v by q (body to reference)."""
    w, x, y, z = q
    tx, ty, tz = 2*(y*v[2] - z*v[1]), 2*(z*v[0] - x*v[2]), 2*(x*v[1] - y*v[0])
    return (v[0] + w*tx + y*tz - z*ty, v[1] + w*ty + z*tx - x*tz, v[2] + w*tz + x*ty - y*tx)


def _qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def rotate(rotation_vector, v):
    """v turned by a rotation vector."""
    return _qrot(_qexp(rotation_vector), v)


def _wrap32(value):
    value &= 0xFFFFFFFF
    return value - TICK_WRAP if value & 0x80000000 else value


class ImuAccumulator:
    """Model of the STM32's IMU integration (the firmware must reproduce it).

    Two running integrals are kept from the raw, uncorrected samples and are
    never reset while the firmware runs:

        q   attitude from the gyro alone: body axes -> the arbitrary frame the
            can had when integration started
        u   specific force rotated by q and summed: a velocity (without
            gravity) in that same frame

    A snapshot of both goes out every 10 ms. The Pi subtracts consecutive
    snapshots (ImuDifferencer), so what a lost message costs is resolution in
    time, never the integral.

    add_sample() is the 1 kHz step: trapezoidal in rate with the two-sample
    coning term, and each force sample rotated by the attitude at its own
    instant, which is what removes sculling. Gyro and accelerometer are taken
    as sampled together; the BMI088's two sensors are not, and the firmware
    must rotate each accelerometer sample by q interpolated to its capture
    time (not modelled here). add_increment() takes ready-made increments
    (used by the simulation, whose truth is already integrated).

    The firmware must keep u in 64-bit fixed point or double precision; a
    single-precision u loses the low bits within minutes on the pad.
    """

    def __init__(self, epoch=0):
        self.epoch = epoch & 0xFF
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.u = [0.0, 0.0, 0.0]
        self._rate = self._force = None
        self._n = 0
        self._flags = IMU_RESTART

    def _normalise(self):
        w, x, y, z = self.q
        n = math.sqrt(w*w + x*x + y*y + z*z)
        self.q = (w/n, x/n, y/n, z/n)

    def add_sample(self, rate, force, h, flags=0):
        """One synchronous sample pair: rate rad/s, specific force m/s^2, h seconds since the last."""
        self._flags |= flags
        self._n += 1
        if self._rate is None:
            self._rate, self._force = tuple(rate), tuple(force)
            return
        a, b = self._rate, rate
        k = h*h/12
        phi = (h*(a[0] + b[0])/2 + k*(a[1]*b[2] - a[2]*b[1]),
               h*(a[1] + b[1])/2 + k*(a[2]*b[0] - a[0]*b[2]),
               h*(a[2] + b[2])/2 + k*(a[0]*b[1] - a[1]*b[0]))
        start = _qrot(self.q, self._force)
        self.q = _qmul(self.q, _qexp(phi))
        self._normalise()
        end = _qrot(self.q, force)
        for i in range(3):
            self.u[i] += h*(start[i] + end[i])/2
        self._rate, self._force = tuple(rate), tuple(force)

    def add_increment(self, dtheta, dv, flags=0):
        """An interval's rotation vector and its velocity increment in start-of-interval body axes."""
        self._flags |= flags
        self._n += 1
        step = _qrot(self.q, dv)
        for i in range(3):
            self.u[i] += step[i]
        self.q = _qmul(self.q, _qexp(dtheta))
        self._normalise()

    def snapshot(self, tick):
        message = Imu(tick=tick & 0xFFFFFFFF,
                      q=tuple(max(-Q_ONE, min(Q_ONE, math.floor(c*Q_ONE + 0.5))) for c in self.q),
                      u=tuple(_wrap32(math.floor(c/U_LSB + 0.5)) for c in self.u),
                      epoch=self.epoch, flags=self._flags, n_gyro=min(self._n, 255), n_accel=min(self._n, 255))
        self._flags = 0
        self._n = 0
        return message


@dataclass
class ImuIncrement:
    """What the navigation filter is given for the time between two snapshots received."""
    tick: int               # 32-bit tick at the end of the interval
    dt: float               # s
    dtheta: tuple           # rotation vector over the interval, rad
    dv: tuple               # velocity increment in start-of-interval body axes, m/s
    flags: int = 0
    samples: int = 0

    def mean(self):
        """(specific force, rate) in the form Eskf.propagate takes: the velocity increment over
        dt in mid-interval body axes, and the rotation vector over dt."""
        half = (-self.dtheta[0]/2, -self.dtheta[1]/2, -self.dtheta[2]/2)
        mid = rotate(half, self.dv)
        return tuple(c/self.dt for c in mid), tuple(c/self.dt for c in self.dtheta)

    def split(self, fraction):
        """Two increments covering `fraction` and the rest, taking rate and (inertial)
        acceleration as constant across the interval. Their composition is this increment."""
        f = min(1.0, max(0.0, fraction))
        first = tuple(f*c for c in self.dtheta)
        back = (-first[0], -first[1], -first[2])
        a = ImuIncrement((self.tick - int(round((1 - f)*self.dt*TICK_HZ))) % TICK_WRAP, f*self.dt, first,
                         tuple(f*c for c in self.dv), self.flags, self.samples)
        b = ImuIncrement(self.tick, (1 - f)*self.dt, tuple((1 - f)*c for c in self.dtheta),
                         rotate(back, tuple((1 - f)*c for c in self.dv)), self.flags, self.samples)
        return a, b


class ImuDifferencer:
    """Pi side: consecutive Imu snapshots -> ImuIncrement, across any number of lost messages.

    feed() returns None for the first snapshot, after a restart of the
    STM32's integrals (epoch change or IMU_RESTART), for a snapshot that is
    not later than the last one, for a gap longer than max_gap_s and for a
    quaternion that is not close to unit length. In each case the navigation
    filter has lost its inertial thread and must be told (see the counters).
    """

    def __init__(self, max_gap_s=2.0, period_s=0.01):
        self.max_gap_s = max_gap_s
        self.period_s = period_s        # nominal snapshot spacing, to count bridged gaps
        self.last = None
        self.increments = self.bridged = self.restarts = self.refused = self.faults = 0
        self.longest_gap_s = 0.0

    def feed(self, imu):
        try:
            return self._feed(imu)
        except Exception:
            self.faults += 1
            self.last = None
            return None

    def _feed(self, imu):
        q = tuple(c/Q_ONE for c in imu.q)
        norm = math.sqrt(sum(c*c for c in q))
        if len(q) != 4 or len(imu.u) != 3 or not 0.999 < norm < 1.001:
            self.refused += 1
            return None
        q = tuple(c/norm for c in q)
        last, self.last = self.last, (imu.tick, q, tuple(imu.u), imu.epoch)
        if last is None:
            return None
        if imu.epoch != last[3] or imu.flags & IMU_RESTART:
            self.restarts += 1
            return None
        dt = ((imu.tick - last[0]) % TICK_WRAP)/TICK_HZ
        if dt <= 0 or dt > self.max_gap_s:
            self.restarts += 1
            return None
        dtheta = _qlog(_qmul(_qconj(last[1]), q))
        du = tuple(_wrap32(a - b)*U_LSB for a, b in zip(imu.u, last[2]))
        dv = _qrot(_qconj(last[1]), du)
        self.increments += 1
        if dt > 1.5*self.period_s:
            self.bridged += 1           # at least one snapshot between these two never arrived
        self.longest_gap_s = max(self.longest_gap_s, dt)
        return ImuIncrement(imu.tick, dt, dtheta, dv, imu.flags, imu.n_gyro)


# --- Budget --------------------------------------------------------------------------

STM32_RATES = {Imu: 100.0, Baro: 10.0, Gnss: 10.0, Trig: 5.0, Pps: 1.0, Status: 1.0}
PI_RATES = {PiStatus: 2.0, Nav: 5.0, TrigCfg: 1.0}
BAUD_RATES = (115200, 230400, 460800, 921600)


def wire_size(cls):
    """Bytes on the wire for one message, delimiter included (exact: stuffing adds one byte)."""
    return cls.SIZE + OVERHEAD


def budget(baud, rates=None, first=Imu):
    """Load and worst-case latency of one direction at `baud`, 10 bits per byte (8N1).

    latency_priority_s: `first` jumps the queue, so it waits for at most the
    longest other frame, just started. latency_fifo_s: one of every other
    message is ahead of it. Both include its own transmission.
    """
    rates = STM32_RATES if rates is None else rates
    byte_s = 10.0/baud
    load = sum(wire_size(cls)*rate for cls, rate in rates.items())
    others = [wire_size(cls) for cls in rates if cls is not first]
    own = wire_size(first)
    return {'baud': baud, 'bytes_per_s': load, 'capacity_bytes_per_s': baud/10.0, 'utilisation': load*byte_s,
            'latency_priority_s': (max(others, default=0) + own)*byte_s,
            'latency_fifo_s': (sum(others) + own)*byte_s}
