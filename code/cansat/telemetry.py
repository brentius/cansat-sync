"""GPL-3.0-only. Fixed 40-byte downlink packet shared by flight and ground.

One packet per second carries the primary mission (pressure, temperature) and
the secondary mission summary (GPS truth, TRN position with uncertainty, and
the landing prediction). Full-rate data stays in the onboard log; the packet is
deliberately coarse. Positions are east/north/up metres from the launch pad.

Layout, little-endian:

    offset size field           unit / coding
    0      1    magic           protocol identifier
    1      2    seq             packet counter, wraps at 65536
    3      4    t_ms            milliseconds since boot
    7      1    state|flags     low nibble flight state, high nibble flags
    8      2    pressure        Pa - 50000, 1 Pa steps
    10     2    temperature     0.01 degC steps, signed
    12     2    altitude        0.1 m above pad, signed
    14     6    gps e, n, u     0.1 m, signed
    20     4    trn e, n        0.1 m, signed
    24     3    trn ellipse     major, minor in 0.1 m; angle in 180/256 deg
    27     1    protection      0.5 m steps
    28     1    landmarks       accepted landmarks in the last fix
    29     4    predicted e, n  0.1 m, signed
    33     1    time to go      0.2 s steps
    34     3    pred. ellipse   major, minor in 0.5 m; angle in 180/256 deg
    37     1    battery         2.50 V + 0.01 V steps, mean cell voltage (2S pack / 2)
    38     2    crc             CRC-16/CCITT-FALSE over bytes 0..37

Signed 16-bit fields use -32768 for "not available"; unsigned 8-bit fields
saturate at 254 and use 255 for "not available".
"""
import math
import struct
from dataclasses import dataclass, fields

MAGIC = 0xC5
_BODY = struct.Struct('<BHIBHhhhhhhhBBBBBhhBBBBB')
PACKET_SIZE = _BODY.size + 2

FLAG_GPS_VALID = 0x10
FLAG_TRN_VALID = 0x20
FLAG_PRED_VALID = 0x40
FLAG_CAMERA_OK = 0x80

_I16_NA = -32768
_U8_NA = 255
NAN = float('nan')


@dataclass
class Telemetry:
    seq: int = 0
    t_ms: int = 0
    state: int = 0
    flags: int = 0
    pressure_pa: float = NAN
    temperature_c: float = NAN
    altitude_m: float = NAN
    gps_e: float = NAN
    gps_n: float = NAN
    gps_u: float = NAN
    trn_e: float = NAN
    trn_n: float = NAN
    trn_major_m: float = NAN
    trn_minor_m: float = NAN
    trn_angle_rad: float = 0.0
    protection_m: float = NAN
    landmarks: int = 0
    pred_e: float = NAN
    pred_n: float = NAN
    time_to_go_s: float = NAN
    pred_major_m: float = NAN
    pred_minor_m: float = NAN
    pred_angle_rad: float = 0.0
    battery_v: float = NAN

    def as_dict(self):
        return {f.name: getattr(self, f.name) for f in fields(self)}


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no reflection."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _i16(value, step):
    if value is None or not math.isfinite(value):
        return _I16_NA
    return max(-32767, min(32767, round(value/step)))


def _u8(value, step, offset=0.0):
    if value is None or not math.isfinite(value):
        return _U8_NA
    return max(0, min(254, round((value - offset)/step)))


def _u16(value, step, offset=0.0):
    if value is None or not math.isfinite(value):
        return 0xFFFF
    return max(0, min(0xFFFE, round((value - offset)/step)))


def _angle(value):
    if value is None or not math.isfinite(value):
        return 0
    return round((value % math.pi)/math.pi*256) % 256


def _from_i16(raw, step):
    return NAN if raw == _I16_NA else raw*step


def _from_u8(raw, step, offset=0.0):
    return NAN if raw == _U8_NA else raw*step + offset


def encode(t, magic=MAGIC):
    body = _BODY.pack(
        magic, t.seq & 0xFFFF, t.t_ms & 0xFFFFFFFF, (t.state & 0x0F) | (t.flags & 0xF0),
        _u16(t.pressure_pa, 1.0, 50000.0), _i16(t.temperature_c, 0.01), _i16(t.altitude_m, 0.1),
        _i16(t.gps_e, 0.1), _i16(t.gps_n, 0.1), _i16(t.gps_u, 0.1),
        _i16(t.trn_e, 0.1), _i16(t.trn_n, 0.1),
        _u8(t.trn_major_m, 0.1), _u8(t.trn_minor_m, 0.1), _angle(t.trn_angle_rad),
        _u8(t.protection_m, 0.5), max(0, min(255, int(t.landmarks))),
        _i16(t.pred_e, 0.1), _i16(t.pred_n, 0.1), _u8(t.time_to_go_s, 0.2),
        _u8(t.pred_major_m, 0.5), _u8(t.pred_minor_m, 0.5), _angle(t.pred_angle_rad),
        _u8(t.battery_v, 0.01, 2.5))
    return body + struct.pack('<H', crc16(body))


def decode(packet, magic=MAGIC):
    """Raises ValueError for a wrong length, foreign magic or failed CRC."""
    if len(packet) != PACKET_SIZE:
        raise ValueError('wrong packet length')
    body = packet[:-2]
    if struct.unpack('<H', packet[-2:])[0] != crc16(body):
        raise ValueError('CRC mismatch')
    v = _BODY.unpack(body)
    if v[0] != magic:
        raise ValueError('foreign packet')
    return Telemetry(
        seq=v[1], t_ms=v[2], state=v[3] & 0x0F, flags=v[3] & 0xF0,
        pressure_pa=NAN if v[4] == 0xFFFF else v[4] + 50000.0,
        temperature_c=_from_i16(v[5], 0.01), altitude_m=_from_i16(v[6], 0.1),
        gps_e=_from_i16(v[7], 0.1), gps_n=_from_i16(v[8], 0.1), gps_u=_from_i16(v[9], 0.1),
        trn_e=_from_i16(v[10], 0.1), trn_n=_from_i16(v[11], 0.1),
        trn_major_m=_from_u8(v[12], 0.1), trn_minor_m=_from_u8(v[13], 0.1),
        trn_angle_rad=v[14]*math.pi/256,
        protection_m=_from_u8(v[15], 0.5), landmarks=v[16],
        pred_e=_from_i16(v[17], 0.1), pred_n=_from_i16(v[18], 0.1),
        time_to_go_s=_from_u8(v[19], 0.2),
        pred_major_m=_from_u8(v[20], 0.5), pred_minor_m=_from_u8(v[21], 0.5),
        pred_angle_rad=v[22]*math.pi/256,
        battery_v=_from_u8(v[23], 0.01, 2.5))


def cov_to_ellipse(cxx, cxy, cyy):
    """1-sigma semi-axes and major-axis angle from east, in [0, pi)."""
    mean = (cxx + cyy)/2
    half = math.hypot((cxx - cyy)/2, cxy)
    if mean - half < -1e-9*max(1.0, mean + half):
        raise ValueError('covariance is not positive semidefinite')
    angle = 0.5*math.atan2(2*cxy, cxx - cyy) % math.pi
    return math.sqrt(mean + half), math.sqrt(max(mean - half, 0.0)), angle


def ellipse_to_cov(major, minor, angle):
    c, s = math.cos(angle), math.sin(angle)
    a, b = major*major, minor*minor
    return a*c*c + b*s*s, (a - b)*c*s, a*s*s + b*c*c
