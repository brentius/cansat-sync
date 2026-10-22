"""GPL-3.0-only. STM32 <-> Pi serial link: codec, streaming decoder, time base, IMU stream.

Run on a development machine. Nothing here runs on the Pi or the STM32.
`python code/tests/test_link.py --write-vectors` regenerates link_vectors.json
from the reference codec; do that only when the protocol itself changes.
"""
import json
import math
import random
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import link  # noqa: E402
from cansat import telemetry as tm  # noqa: E402

VECTORS = Path(__file__).with_name('link_vectors.json')
SIZES = {link.Hello: 16, link.Imu: 36, link.Baro: 12, link.Gnss: 60, link.Trig: 16, link.Pps: 16, link.Status: 48,
         link.Cmd: 8, link.PiStatus: 24, link.Nav: 32, link.Ack: 4, link.TrigCfg: 8}
IDS = {0x01: link.Hello, 0x10: link.Imu, 0x11: link.Baro, 0x12: link.Gnss, 0x13: link.Trig, 0x14: link.Pps,
       0x15: link.Status, 0x16: link.Cmd, 0x80: link.PiStatus, 0x81: link.Nav, 0x82: link.Ack, 0x83: link.TrigCfg}
RANGE = {'B': (0, 255), 'b': (-128, 127), 'H': (0, 65535), 'h': (-32768, 32767),
         'I': (0, 2**32 - 1), 'i': (-2**31, 2**31 - 1)}


def random_message(cls, rng):
    """A message with every field drawn across its whole representable range."""
    kwargs = {}
    for name, code, kind in cls.LAYOUT:
        if name is None:
            continue
        lo, hi = RANGE[code[-1]]
        if len(code) > 1:
            kwargs[name] = tuple(rng.randint(lo, hi) for _ in range(int(code[:-1])))
        elif kind is None or kind == link.WRAP:
            kwargs[name] = rng.randint(lo, hi)
        elif kind == 'angle':
            kwargs[name] = rng.uniform(0, math.pi)
        else:
            low, high = (lo + 1, hi) if lo < 0 else (0, hi - 1)
            kwargs[name] = rng.uniform(low*kind, high*kind)
    return cls(**kwargs)


def traffic(rng, count):
    """A realistic mix: mostly Imu, with the other STM32 messages among them."""
    kinds = [link.Imu]*20 + [link.Baro, link.Baro, link.Gnss, link.Gnss, link.Trig, link.Pps, link.Status]
    return [random_message(rng.choice(kinds), rng) for _ in range(count)]


def same(a, b):
    return type(a) is type(b) and a.raw() == b.raw()


class Framing(unittest.TestCase):
    def test_crc_is_the_telemetry_crc(self):
        self.assertEqual(link.crc(b'123456789'), 0x29B1)
        rng = random.Random(1)
        for _ in range(300):
            data = bytes(rng.randrange(256) for _ in range(rng.randrange(80)))
            self.assertEqual(link.crc(data), tm.crc16(data))

    def test_stuffing_round_trip_has_no_zero_and_one_byte_of_overhead(self):
        rng = random.Random(2)
        for length in list(range(0, 40)) + [100, 252, 253]:
            for zero_share in (0.0, 0.1, 0.9, 1.0):
                data = bytes(0 if rng.random() < zero_share else rng.randrange(1, 256) for _ in range(length))
                stuffed = link.cobs_encode(data)
                self.assertNotIn(0, stuffed)
                self.assertEqual(len(stuffed), length + 1)
                self.assertNotIn(0xFF, stuffed[:1])
                self.assertEqual(link.cobs_decode(stuffed), data)

    def test_stuffing_of_long_runs(self):
        # Longer than any frame of this protocol, where the 0xFF code appears: still a round trip.
        for length in (254, 255, 508, 600):
            data = bytes(range(1, 255))*3
            data = data[:length]
            self.assertEqual(link.cobs_decode(link.cobs_encode(data)), data)

    def test_invalid_stuffing_is_refused(self):
        self.assertIsNone(link.cobs_decode(b'\x05\x01\x02'))        # code runs past the end
        self.assertIsNone(link.cobs_decode(b'\x02\x01\x00\x01'))    # a zero where a code belongs

    def test_frame_layout(self):
        wire = link.frame(0x11, 7, b'\x01\x00\x02')
        self.assertEqual(wire[-1], 0)
        body = link.cobs_decode(wire[:-1])
        self.assertEqual(body[:5], b'\x11\x07\x01\x00\x02')
        self.assertEqual(body[5] | body[6] << 8, tm.crc16(body[:5]))
        with self.assertRaises(ValueError):
            link.frame(1, 0, bytes(link.MAX_PAYLOAD + 1))
        self.assertEqual(len(link.frame(1, 0, bytes(link.MAX_PAYLOAD))), link.MAX_WIRE)


class Messages(unittest.TestCase):
    def test_identifiers_and_sizes(self):
        self.assertEqual(link.MESSAGES, IDS)
        for cls, size in SIZES.items():
            self.assertEqual(cls.SIZE, size, cls.__name__)
            self.assertEqual(len(cls().payload()), size)
            self.assertEqual(len(link.encode(cls())), size + link.OVERHEAD)
            self.assertEqual(link.wire_size(cls), size + 6)
            self.assertEqual(cls.ID >= 0x80, cls in (link.PiStatus, link.Nav, link.Ack, link.TrigCfg))

    def test_round_trip_of_every_message_within_half_a_step(self):
        rng = random.Random(3)
        for cls in SIZES:
            for _ in range(300):
                sent = random_message(cls, rng)
                seq = rng.randrange(256)
                got, = link.Decoder().feed(link.encode(sent, seq))
                self.assertIs(type(got), cls)
                self.assertEqual(got.seq, seq)
                self.assertEqual(got.raw(), sent.raw())
                for name, code, kind in cls.LAYOUT:
                    if name is None:
                        continue
                    a, b = getattr(sent, name), getattr(got, name)
                    if kind is None or kind == link.WRAP:
                        self.assertEqual(a, b, name)
                    elif kind == 'angle':
                        self.assertLessEqual(abs((b - a + math.pi/2) % math.pi - math.pi/2), math.pi/512 + 1e-12, name)
                    else:
                        self.assertLessEqual(abs(a - b), kind/2*(1 + 1e-9) + abs(a)*1e-15, name)

    def test_defaults_survive(self):
        for cls in SIZES:
            got, = link.Decoder().feed(link.encode(cls()))
            for f_name in (n for n, _, _ in cls.LAYOUT if n):
                a, b = getattr(cls(), f_name), getattr(got, f_name)
                if isinstance(a, float) and math.isnan(a):
                    self.assertTrue(math.isnan(b), f_name)
                else:
                    self.assertEqual(a, b, f_name)

    def test_missing_values_use_the_reserved_code(self):
        raw = link.Baro(pressure_pa=float('nan'), temperature_c=float('inf')).raw()
        self.assertEqual((raw['pressure_pa'], raw['temperature_c']), (0xFFFFFFFF, -32768))
        raw = link.Gnss(lat_deg=None, h_acc_m='x').raw()
        self.assertEqual((raw['lat_deg'], raw['h_acc_m']), (-2**31, 0xFFFFFFFF))
        got, = link.Decoder().feed(link.encode(link.Nav()))
        self.assertTrue(all(math.isnan(getattr(got, n)) for n in ('trn_e', 'pred_n', 'time_to_go_s', 'vel_u')))

    def test_out_of_range_saturates_one_short_of_missing(self):
        got, = link.Decoder().feed(link.encode(link.Baro(pressure_pa=1e12, temperature_c=-1e9)))
        self.assertAlmostEqual(got.pressure_pa, (2**32 - 2)*0.01)
        self.assertAlmostEqual(got.temperature_c, -327.67)
        got, = link.Decoder().feed(link.encode(link.Baro(pressure_pa=-5.0, temperature_c=1e9)))
        self.assertEqual(got.pressure_pa, 0.0)
        self.assertAlmostEqual(got.temperature_c, 327.67)
        got, = link.Decoder().feed(link.encode(link.Nav(trn_e=1e9, trn_major_m=1e9, time_to_go_s=1e9, vel_u=-1e9)))
        self.assertAlmostEqual(got.trn_e, 3276.7)
        self.assertAlmostEqual(got.trn_major_m, 25.4)
        self.assertAlmostEqual(got.time_to_go_s, 50.8)
        self.assertAlmostEqual(got.vel_u, -327.67)
        got, = link.Decoder().feed(link.encode(link.Status(battery_v=1e3, pi_age_s=1e3, altitude_m=-1e6)))
        self.assertAlmostEqual(got.battery_v, 65.534)
        self.assertAlmostEqual(got.pi_age_s, 25.4)
        self.assertAlmostEqual(got.altitude_m, -3276.7)

    def test_integers_clamp_counters_wrap_and_nonsense_does_not_raise(self):
        raw = link.Trig(frame=2**32 + 5, tick=-1, period_us=10**12, pulse_us=-4, flags=999).raw()
        self.assertEqual(raw, {'frame': 5, 'tick': 2**32 - 1, 'period_us': 2**32 - 1, 'pulse_us': 0, 'flags': 255})
        self.assertEqual(link.Imu(u=(2**31, -2**31 - 1, 2**33 + 3)).raw()['u'], [-2**31, 2**31 - 1, 3])
        self.assertEqual(link.Imu(q=(2**40, -2**40, 0, 1)).raw()['q'], [2**31 - 1, -2**31, 0, 1])
        self.assertEqual(link.Status(mcu_temp_c=-300, state='x', health=None).raw()['mcu_temp_c'], -128)
        for cls in SIZES:
            for junk in (None, 'x', float('nan'), float('inf'), -float('inf'), 1e300, -1e300, [1, 2], b'\x00'):
                message = cls(**{n: junk for n, _, _ in cls.LAYOUT if n})
                self.assertEqual(len(link.encode(message)), cls.SIZE + link.OVERHEAD)

    def test_rounding_is_half_up_not_half_even(self):
        # 0.5 and 2.5 counts: Python's round() would give 0 and 2. The C side must use floor(x + 0.5).
        self.assertEqual(link.Nav(trn_e=0.05, trn_n=0.25).raw()['trn_e'], 1)
        self.assertEqual(link.Baro(temperature_c=0.025).raw()['temperature_c'], 3)
        self.assertEqual(link.Baro(temperature_c=-0.025).raw()['temperature_c'], -2)

    def test_gnss_usable(self):
        good = link.Gnss(fix_type=3, fix_flags=1, lat_deg=52.0, lon_deg=-1.0, vel_n=1.0, vel_e=2.0, vel_d=3.0)
        self.assertTrue(good.usable)
        self.assertFalse(link.Gnss(fix_type=2, fix_flags=1, lat_deg=52.0, vel_n=1.0, vel_e=1.0).usable)
        self.assertFalse(link.Gnss(fix_type=3, fix_flags=0, lat_deg=52.0, vel_n=1.0, vel_e=1.0).usable)
        self.assertFalse(link.Gnss(fix_type=3, fix_flags=1).usable)

    def test_nav_fills_the_downlink_packet_byte_for_byte(self):
        rng = random.Random(4)
        for _ in range(300):
            values = dict(trn_e=rng.uniform(-900, 900), trn_n=rng.uniform(-900, 900), trn_major_m=rng.uniform(0, 30),
                          trn_minor_m=rng.uniform(0, 30), trn_angle_rad=rng.uniform(0, math.pi),
                          protection_m=rng.uniform(0, 140), landmarks=rng.randrange(40), pred_e=rng.uniform(-900, 900),
                          pred_n=rng.uniform(-900, 900), time_to_go_s=rng.uniform(0, 60), pred_major_m=rng.uniform(0, 140),
                          pred_minor_m=rng.uniform(0, 140), pred_angle_rad=rng.uniform(0, math.pi))
            flags = rng.choice((0, tm.FLAG_TRN_VALID, tm.FLAG_TRN_VALID | tm.FLAG_PRED_VALID | tm.FLAG_CAMERA_OK))
            own = tm.Telemetry(seq=5, t_ms=1234, state=2, flags=tm.FLAG_GPS_VALID | tm.FLAG_TRN_VALID, pressure_pa=99000.0,
                               gps_e=1.0, battery_v=3.9)
            got, = link.Decoder().feed(link.encode(link.Nav(tick=77, flags=flags | 0x0F, **values)))
            through_link = tm.encode(link.apply_nav(own, got))
            direct = tm.Telemetry(**dict(own.as_dict(), **values))
            direct.flags = tm.FLAG_GPS_VALID | flags
            self.assertEqual(through_link, tm.encode(direct))

    def test_stale_or_absent_nav_clears_the_fix(self):
        own = tm.Telemetry(flags=tm.FLAG_GPS_VALID | tm.FLAG_TRN_VALID | tm.FLAG_CAMERA_OK, trn_e=5.0, landmarks=9, gps_e=1.0)
        nav = link.Nav(tick=1_000_000, flags=tm.FLAG_TRN_VALID, trn_e=7.0, landmarks=3)
        fresh = link.apply_nav(own, nav, max_age_s=2.0, now_tick=2_500_000)
        self.assertEqual((fresh.trn_e, fresh.landmarks, fresh.flags), (7.0, 3, tm.FLAG_GPS_VALID | tm.FLAG_TRN_VALID))
        for stale in (link.apply_nav(own, nav, max_age_s=2.0, now_tick=3_100_000), link.apply_nav(own, None)):
            self.assertTrue(math.isnan(stale.trn_e))
            self.assertEqual((stale.landmarks, stale.flags, stale.gps_e), (0, tm.FLAG_GPS_VALID, 1.0))
        # Age is taken modulo the 32-bit counter.
        wrapped = link.apply_nav(own, link.Nav(tick=2**32 - 500_000, trn_e=1.0), max_age_s=2.0, now_tick=500_000)
        self.assertEqual(wrapped.trn_e, 1.0)


class Streaming(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(5)

    def stream(self, count, seq=0):
        messages = traffic(self.rng, count)
        encoder = link.Encoder()
        encoder.seq = seq
        return messages, [encoder.encode(m) for m in messages]

    def test_any_chunking_gives_the_same_messages(self):
        messages, wires = self.stream(200)
        data = b''.join(wires)
        whole = link.Decoder().feed(data)
        self.assertTrue(all(same(a, b) for a, b in zip(whole, messages)) and len(whole) == 200)
        one = link.Decoder()
        by_byte = [m for i in range(len(data)) for m in one.feed(data[i:i + 1])]
        ragged = link.Decoder()
        by_chunk, i = [], 0
        while i < len(data):
            n = self.rng.choice((0, 1, 2, 3, 17, 64, 500))
            by_chunk += ragged.feed(data[i:i + n])
            i += n
        for other, decoder in ((by_byte, one), (by_chunk, ragged)):
            self.assertEqual(len(other), 200)
            self.assertTrue(all(same(a, b) for a, b in zip(other, messages)))
            self.assertEqual(dict(decoder.stats(), bytes=0), dict(link.Decoder().stats(), frames=200))

    def test_accepts_bytearray_and_memoryview_and_counts_what_is_not_bytes(self):
        wire = link.encode(link.Baro(pressure_pa=1e5))
        d = link.Decoder()
        self.assertEqual(len(d.feed(bytearray(wire)) + d.feed(memoryview(wire))), 2)
        for junk in (None, 'text', 3.5, object(), [1, 2, 300], {'a': 1}):
            self.assertEqual(d.feed(junk), [])
        self.assertEqual(d.ignored, 6)
        self.assertEqual(len(d.feed(wire)), 1)

    def test_idle_line_and_break_are_harmless(self):
        d = link.Decoder()
        self.assertEqual(d.feed(bytes(5000)), [])
        self.assertEqual((d.empty, d.crc, d.stuffing, d.short), (5000, 0, 0, 0))
        self.assertEqual(len(d.feed(link.encode(link.Ack()))), 1)

    def test_random_bytes_never_raise_and_are_never_accepted(self):
        d = link.Decoder()
        accepted = []
        for _ in range(400):
            accepted += d.feed(bytes(self.rng.randrange(256) for _ in range(self.rng.randrange(1, 1200))))
        self.assertEqual(accepted, [])
        self.assertEqual(d.faults, 0)
        self.assertGreater(d.stuffing + d.crc + d.short, 100)
        self.assertLessEqual(len(d._buf), link.MAX_WIRE)

    def test_memory_is_bounded_without_a_delimiter(self):
        d = link.Decoder()
        for _ in range(200):
            d.feed(b'\x55'*5000)
            self.assertLessEqual(len(d._buf), link.MAX_WIRE)
        self.assertEqual(d.too_long, 1)
        messages, wires = self.stream(3)
        # The first frame runs on from the junk and is lost with it; the next is decoded.
        got = d.feed(b''.join(wires))
        self.assertEqual(len(got), 2)
        self.assertTrue(same(got[0], messages[1]) and same(got[1], messages[2]))

    def test_every_truncation_is_survived(self):
        messages, wires = self.stream(4)
        for cut in range(len(wires[1])):
            d = link.Decoder()
            got = d.feed(wires[0] + wires[1][:cut] + wires[2] + wires[3])
            self.assertTrue(same(got[0], messages[0]))
            self.assertTrue(same(got[-1], messages[3]), cut)
            self.assertTrue(all(any(same(g, m) for m in messages) for g in got))
            self.assertLessEqual(len(got), 3)

    def test_every_single_bit_error_is_rejected_and_costs_at_most_two_frames(self):
        for cls in SIZES:
            messages = [link.Ack(cmd_seq=1), random_message(cls, self.rng), link.Ack(cmd_seq=2), link.Ack(cmd_seq=3)]
            wires = [link.encode(m, i) for i, m in enumerate(messages)]
            for bit in range(len(wires[1])*8):
                damaged = bytearray(wires[1])
                damaged[bit//8] ^= 1 << (bit % 8)
                d = link.Decoder()
                got = d.feed(wires[0] + bytes(damaged) + wires[2] + wires[3])
                self.assertTrue(same(got[0], messages[0]))
                self.assertTrue(same(got[-1], messages[3]), (cls.__name__, bit))
                for g in got[1:-1]:
                    self.assertTrue(same(g, messages[2]), (cls.__name__, bit))
                self.assertGreaterEqual(len(got), 2)
                # Only a hit on the delimiter takes the following frame too.
                if bit//8 != len(wires[1]) - 1:
                    self.assertEqual(len(got), 3, (cls.__name__, bit))

    def check_resynchronisation(self, damage, trials=1500):
        """Damage one stretch of a stream. Nothing may raise, nothing that was not sent may come
        out, and every frame that starts after the first delimiter following the damage must."""
        lost_most = 0
        for _ in range(trials):
            messages, wires = self.stream(self.rng.randrange(6, 14))
            starts = [0]
            for w in wires:
                starts.append(starts[-1] + len(w))
            data = bytearray(b''.join(wires))
            at = self.rng.randrange(len(data))
            end = damage(data, at)                      # index, in the damaged stream, just past the damage
            shift = len(data) - starts[-1]
            first_clean = None
            delimiter = data.find(b'\x00', end)
            if delimiter >= 0:
                first_clean = next((i for i, s in enumerate(starts[:-1]) if s + shift > delimiter and s >= at), None)
            d = link.Decoder()
            got = []
            i = 0
            while i < len(data):
                n = self.rng.randrange(1, 90)
                got += d.feed(bytes(data[i:i + n]))
                i += n
            self.assertEqual(d.faults, 0)
            for g in got:
                self.assertTrue(any(same(g, m) for m in messages))
            before = [i for i, s in enumerate(starts[1:]) if s <= at]
            for i in before:
                self.assertTrue(any(same(g, messages[i]) for g in got))
            if first_clean is not None:
                tail = got[len(got) - (len(messages) - first_clean):]
                self.assertEqual(len(tail), len(messages) - first_clean)
                self.assertTrue(all(same(g, m) for g, m in zip(tail, messages[first_clean:])))
                lost_most = max(lost_most, first_clean - len(before))
        return lost_most

    def test_resynchronises_after_overwritten_bytes(self):
        def damage(data, at):
            n = self.rng.randrange(1, 40)
            for i in range(at, min(len(data), at + n)):
                data[i] = self.rng.randrange(256)
            return min(len(data), at + n)
        self.check_resynchronisation(damage)

    def test_resynchronises_after_deleted_bytes(self):
        def damage(data, at):
            del data[at:at + self.rng.randrange(1, 120)]
            return at
        self.check_resynchronisation(damage)

    def test_resynchronises_after_inserted_bytes(self):
        def damage(data, at):
            junk = bytes(self.rng.randrange(256) for _ in range(self.rng.randrange(1, 400)))
            data[at:at] = junk
            return at + len(junk)
        self.check_resynchronisation(damage)

    def test_a_single_damaged_byte_costs_at_most_two_frames(self):
        def flip(data, at):
            data[at] ^= 1 << self.rng.randrange(8)
            return at + 1
        self.assertLessEqual(self.check_resynchronisation(flip, 3000), 2)

    def test_sequence_gaps_are_counted_exactly(self):
        messages, wires = self.stream(1000, seq=250)
        keep = [self.rng.random() > 0.2 for _ in wires]
        keep[0] = keep[-1] = True
        d = link.Decoder()
        got = d.feed(b''.join(w for w, k in zip(wires, keep) if k))
        self.assertEqual(len(got), sum(keep))
        self.assertEqual(d.lost, len(wires) - sum(keep))
        self.assertEqual(d.repeats, 0)

    def test_corrupted_frames_show_up_as_lost_by_sequence(self):
        messages, wires = self.stream(600)
        data = bytearray()
        bad = 0
        for i, w in enumerate(wires):
            w = bytearray(w)
            if 0 < i < len(wires) - 1 and self.rng.random() < 0.1:
                w[self.rng.randrange(len(w) - 1)] ^= 0x10      # not the delimiter
                bad += 1
            data += w
        d = link.Decoder()
        got = d.feed(bytes(data))
        self.assertEqual(len(got), 600 - bad)
        self.assertEqual(d.lost, bad)
        # A flipped bit can make a zero and split the frame in two: one loss, two rejects.
        self.assertGreaterEqual(d.crc + d.stuffing + d.short, bad)
        self.assertLessEqual(d.crc + d.stuffing + d.short, 2*bad)

    def test_a_gap_of_256_frames_is_invisible_and_restarts_are_repeats(self):
        d = link.Decoder()
        d.feed(link.encode(link.Ack(), 10))
        d.feed(link.encode(link.Ack(), 10))                    # 256 frames later, or a restarted sender
        self.assertEqual((d.lost, d.repeats), (0, 1))
        d.feed(link.encode(link.Ack(), 9))
        self.assertEqual(d.lost, 254)
        d.reset_sequence()
        d.feed(link.encode(link.Ack(), 200))
        self.assertEqual(d.lost, 254)

    def test_unknown_identifier_and_wrong_length(self):
        future = link.frame(0x55, 3, b'new')
        d = link.Decoder()
        self.assertEqual(d.feed(future), [])
        self.assertEqual((d.unknown, d.frames), (1, 1))
        d = link.Decoder(raw_unknown=True)
        got, = d.feed(future)
        self.assertEqual((got.msg_id, got.data, got.seq), (0x55, b'new', 3))
        wrong = link.frame(link.Baro.ID, 4, bytes(11))
        self.assertEqual(d.feed(wrong), [])
        self.assertEqual((d.length, d.lost), (1, 0))
        self.assertEqual(len(d.feed(link.frame(0, 5, b''))), 1)    # the shortest frame there is: id, seq, crc
        self.assertEqual(d.unknown, 2)
        self.assertEqual(d.feed(b'\x02\x01\x00'), [])
        self.assertEqual(d.short, 1)

    def test_throughput_is_far_above_the_link_rate(self):
        messages, wires = self.stream(5226*10//42)                 # about ten seconds of traffic
        data = b''.join(wires)
        best = float('inf')
        for _ in range(3):
            d = link.Decoder()
            start = time.perf_counter()
            count = sum(len(d.feed(data[i:i + 256])) for i in range(0, len(data), 256))
            best = min(best, time.perf_counter() - start)
        self.assertEqual(count, len(messages))
        load = link.budget(230400)['bytes_per_s']
        # On a development laptop; the Pi Zero 2 W is several times slower and has not been timed.
        self.assertGreater(len(data)/best, 20*load)
        start = time.perf_counter()
        encoder = link.Encoder()
        for m in messages:
            encoder.encode(m)
        self.assertGreater(len(data)/(time.perf_counter() - start), 10*load)


class Budget(unittest.TestCase):
    def test_load_and_utilisation(self):
        b = link.budget(230400)
        self.assertEqual(b['bytes_per_s'], 42*100 + 18*10 + 66*10 + 22*5 + 22 + 54)
        self.assertAlmostEqual(b['utilisation'], 5226/23040)
        self.assertAlmostEqual(link.budget(115200)['utilisation'], 0.4536, places=4)
        self.assertAlmostEqual(link.budget(460800)['utilisation'], 0.1134, places=4)
        self.assertAlmostEqual(link.budget(921600)['utilisation'], 0.0567, places=4)
        self.assertEqual(link.budget(230400, link.PI_RATES, link.Nav)['bytes_per_s'], 30*2 + 38*5 + 14)

    def test_worst_case_imu_latency(self):
        b = link.budget(230400)
        self.assertAlmostEqual(b['latency_priority_s'], (66 + 42)*10/230400)
        self.assertAlmostEqual(b['latency_fifo_s'], (18 + 66 + 22 + 22 + 54 + 42)*10/230400)
        # Without priority at 115200 an IMU message can wait longer than its own period.
        self.assertGreater(link.budget(115200)['latency_fifo_s'], 0.010)
        self.assertLess(link.budget(230400)['latency_fifo_s'], 0.010)

    def test_every_message_fits_a_frame(self):
        for cls in SIZES:
            self.assertLessEqual(cls.SIZE, link.MAX_PAYLOAD)


class TimeBase(unittest.TestCase):
    def test_unwrap_follows_the_counter_through_a_three_hour_wait(self):
        clock = link.TickClock()
        true = 4_000_000_000                                    # close to the first wrap
        rng = random.Random(6)
        while true < 4_000_000_000 + 4*3600*link.TICK_HZ:
            self.assertEqual(clock.update(true % 2**32), true)
            true += rng.randrange(1, 30*60*link.TICK_HZ)        # messages up to half an hour apart
        self.assertGreater(true//2**32, 3)

    def test_unwrap32_is_nearest(self):
        self.assertEqual(link.unwrap32(5, 2**32 - 10), 2**32 + 5)
        self.assertEqual(link.unwrap32(2**32 - 5, 2**32 + 10), 2**32 - 5)
        self.assertEqual(link.unwrap32(100, 7*2**32 + 90), 7*2**32 + 100)

    def test_offset_is_the_least_delayed_message(self):
        clock = link.TickClock(window=50)
        rng = random.Random(7)
        offset, tick = 1234.5, 2**32 - 300_000
        for _ in range(400):
            tick += 10_000
            clock.update(tick % 2**32, tick/1e6 + offset + 0.002 + abs(rng.gauss(0, 0.003)))
        self.assertGreater(clock.offset_s, offset + 0.002)
        self.assertLess(clock.offset_s, offset + 0.0025)
        self.assertAlmostEqual(clock.to_ticks(offset + clock.offset_s - offset + 5000.0), 5000.0*1e6, delta=1)
        self.assertIsNone(link.TickClock().to_ticks(1.0))
        clock.update('junk', None)
        clock.update(None)
        self.assertEqual(clock.rejected, 2)

    def run_frames(self, lose_trig, latency=0.021, jitter=0.004):
        rng = random.Random(8)
        clock, stamper = link.TickClock(), link.FrameStamper(latency_s=0.02)
        offset = 77.0
        tick = 2**32 - 3_000_000                                # triggers straddle a wrap
        wrong = stamped = 0
        for step in range(1, 3001):
            tick += 10_000
            clock.update(tick % 2**32, tick/1e6 + offset + 0.001 + rng.random()*0.002)
            if step % 20 == 0:
                frame = step//20
                if not lose_trig(frame):
                    stamper.add(link.Trig(frame=frame, tick=tick % 2**32, period_us=200_000, pulse_us=400,
                                          flags=frame == 1), clock)
                arrival = tick/1e6 + offset + latency + rng.uniform(-jitter, jitter)
                got = stamper.stamp(arrival, clock)
                if got is not None:
                    stamped += 1
                    wrong += got != (frame, tick + (400 + 14.26)/2)
        return stamped, wrong, stamper

    def test_frames_get_the_tick_of_their_own_trigger(self):
        stamped, wrong, stamper = self.run_frames(lambda frame: False)
        self.assertEqual((stamped, wrong), (150, 0))
        self.assertEqual(stamper.extrapolated, 0)

    def test_frames_whose_trigger_message_was_lost_are_still_stamped_exactly(self):
        stamped, wrong, stamper = self.run_frames(lambda frame: 40 <= frame < 60 or frame % 7 == 0)
        self.assertEqual((stamped, wrong), (150, 0))
        self.assertGreater(stamper.extrapolated, 30)

    def test_a_frame_far_from_any_trigger_is_refused(self):
        clock, stamper = link.TickClock(), link.FrameStamper(latency_s=0.02)
        self.assertIsNone(stamper.stamp(1.0, clock))
        clock.update(1_000_000, 10.0)
        stamper.add(link.Trig(frame=3, tick=1_000_000, period_us=200_000, pulse_us=100), clock)
        self.assertIsNotNone(stamper.stamp(10.02, clock))
        self.assertIsNone(stamper.stamp(10.02 + 0.1, clock))     # half way between two triggers
        self.assertIsNone(stamper.stamp(float('nan'), clock))
        stamper.add(link.Trig(period_us=0), clock)
        stamper.add(None, clock)
        self.assertEqual(stamper.refused, 5)


def compose(increments):
    """Exact composition of (dtheta, dv) increments: total rotation vector and velocity
    increment in the axes at the start of the first."""
    q = (1.0, 0.0, 0.0, 0.0)
    total = [0.0, 0.0, 0.0]
    for dtheta, dv in increments:
        step = link._qrot(q, dv)
        total = [a + b for a, b in zip(total, step)]
        q = link._qmul(q, link._qexp(dtheta))
    return link._qlog(q), tuple(total)


class ImuStream(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(9)

    def increments(self, count, rate=6.0, force=40.0):
        return [(tuple(self.rng.uniform(-rate, rate)*0.01 for _ in range(3)),
                 tuple(self.rng.uniform(-force, force)*0.01 for _ in range(3))) for _ in range(count)]

    def test_differenced_snapshots_return_the_increments(self):
        acc, dif = link.ImuAccumulator(), link.ImuDifferencer()
        self.assertIsNone(dif.feed(acc.snapshot(0)))               # the first snapshot: nothing to subtract from
        self.assertEqual(dif.feed(acc.snapshot(10_000)).dv, (0.0, 0.0, 0.0))
        sum_error = [0.0, 0.0, 0.0]
        for k, (dtheta, dv) in enumerate(self.increments(3000), 2):
            acc.add_increment(dtheta, dv)
            message, = link.Decoder().feed(link.encode(acc.snapshot(k*10_000)))
            inc = dif.feed(message)
            self.assertAlmostEqual(inc.dt, 0.01)
            for a, b in zip(inc.dtheta, dtheta):
                self.assertLess(abs(a - b), 2e-8)                 # Q30 quaternion: nanoradians
            for i, (a, b) in enumerate(zip(inc.dv, dv)):
                self.assertLess(abs(a - b), 1.01*link.U_LSB*math.sqrt(3))
                sum_error[i] += a - b
        # The rounding does not accumulate: over 3000 intervals the summed error is still about a count.
        self.assertLess(max(abs(e) for e in sum_error), 40*link.U_LSB)
        self.assertEqual((dif.increments, dif.restarts, dif.bridged), (3001, 0, 0))

    def test_resolution_against_sensor_noise(self):
        gyro = math.radians(0.014)*math.sqrt(0.01)               # rad per 10 ms, BMI088 typical
        accel = 175e-6*9.80665*math.sqrt(0.01)                   # m/s per 10 ms
        self.assertLess(2/link.Q_ONE, gyro/1000)
        self.assertLess(link.U_LSB/math.sqrt(6), accel/25)
        self.assertGreater(2**31*link.U_LSB/9.80665, 3000)       # seconds of 1 g that a difference can span

    def test_lost_messages_cost_nothing_but_time_resolution(self):
        for lost in (1, 2, 5, 49, 150):
            acc, dif = link.ImuAccumulator(), link.ImuDifferencer(max_gap_s=2.0)
            acc.snapshot(0)
            dif.feed(acc.snapshot(0))
            gap = self.increments(lost + 1, rate=3.0, force=30.0)
            for dtheta, dv in gap:
                acc.add_increment(dtheta, dv)
            inc = dif.feed(acc.snapshot((lost + 1)*10_000))
            want_theta, want_v = compose(gap)
            self.assertAlmostEqual(inc.dt, (lost + 1)*0.01)
            self.assertLess(max(abs(a - b) for a, b in zip(inc.dtheta, want_theta)), 1e-7)
            self.assertLess(max(abs(a - b) for a, b in zip(inc.dv, want_v)), 2*link.U_LSB)
            self.assertEqual(dif.bridged, 1)

    def test_integrals_wrap_without_harm(self):
        # Three hours at 1 g takes u through its 32 bits more than once; tick wraps every 71.6 minutes.
        acc, dif = link.ImuAccumulator(), link.ImuDifferencer()
        acc.snapshot(0)
        acc.u = [2**31*link.U_LSB - 0.004, -2**31*link.U_LSB + 0.004, 105_900.0]
        tick = 2**32 - 25_000
        dif.feed(acc.snapshot(tick))
        for dtheta, dv in self.increments(20, rate=1.0, force=12.0):
            dv = (abs(dv[0]), -abs(dv[1]), dv[2])
            acc.add_increment((0.0, 0.0, 0.0), dv)
            tick += 10_000
            inc = dif.feed(acc.snapshot(tick))
            self.assertAlmostEqual(inc.dt, 0.01)
            self.assertLess(max(abs(a - b) for a, b in zip(inc.dv, dv)), 2*link.U_LSB)

    def test_restart_long_gap_and_bad_quaternion_break_the_thread(self):
        acc, dif = link.ImuAccumulator(epoch=3), link.ImuDifferencer(max_gap_s=1.0)
        acc.snapshot(0)
        dif.feed(acc.snapshot(0))
        self.assertIsNotNone(dif.feed(acc.snapshot(10_000)))
        self.assertIsNone(dif.feed(acc.snapshot(10_000)))          # no time has passed
        self.assertIsNone(dif.feed(acc.snapshot(1_500_000)))       # longer than max_gap_s
        self.assertIsNotNone(dif.feed(acc.snapshot(1_510_000)))
        reborn = link.ImuAccumulator(epoch=4)
        self.assertIsNone(dif.feed(reborn.snapshot(1_520_000)))    # new epoch, and flagged
        self.assertIsNotNone(dif.feed(reborn.snapshot(1_530_000)))
        late = reborn.snapshot(1_540_000)
        late.flags |= link.IMU_RESTART                             # integrals restarted without a new epoch
        self.assertIsNone(dif.feed(late))
        self.assertIsNotNone(dif.feed(reborn.snapshot(1_545_000)))
        self.assertEqual(dif.restarts, 4)
        self.assertIsNone(dif.feed(link.Imu(tick=1_550_000, q=(0, 0, 0, 0), epoch=4)))
        self.assertIsNone(dif.feed(link.Imu(tick=1_550_000, q=(2**30, 2**29, 0, 0), epoch=4)))
        self.assertEqual(dif.refused, 2)
        self.assertIsNotNone(dif.feed(reborn.snapshot(1_560_000)))
        self.assertIsNone(dif.feed(None))
        self.assertIsNone(dif.feed(link.Imu(q=(1, 2), u=())))
        self.assertGreaterEqual(dif.faults + dif.refused, 4)

    def test_mean_is_what_the_filter_takes(self):
        dtheta, dv = (0.03, -0.02, 0.05), (0.1, 0.2, -0.05)
        inc = link.ImuIncrement(0, 0.01, dtheta, dv)
        force, rate = inc.mean()
        self.assertEqual(rate, tuple(c/0.01 for c in dtheta))
        # Turning the mid-interval force forward by half the rotation gives back dv/dt.
        back = link.rotate(tuple(c/2 for c in dtheta), force)
        for a, b in zip(back, dv):
            self.assertAlmostEqual(a*0.01, b, places=12)

    def test_split_composes_to_the_whole(self):
        inc = link.ImuIncrement(500_000, 0.5, (0.4, -0.3, 0.9), (1.0, -2.0, 4.5))
        for fraction in (0.0, 0.1, 0.5, 0.98, 1.0):
            a, b = inc.split(fraction)
            self.assertAlmostEqual(a.dt + b.dt, 0.5)
            theta, v = compose([(a.dtheta, a.dv), (b.dtheta, b.dv)])
            for x, y in zip(theta + v, inc.dtheta + inc.dv):
                self.assertAlmostEqual(x, y, places=12)
            self.assertEqual(b.tick, inc.tick)
            self.assertEqual(a.tick, 500_000 - round((1 - fraction)*500_000))

    def motion(self, t):
        """A swinging, spinning can: body rate (rad/s) and specific force (m/s^2)."""
        rate = (1.8*math.sin(2*math.pi*0.9*t), 1.8*math.cos(2*math.pi*0.9*t + 0.4), 1.0 + 0.5*math.sin(2*math.pi*0.3*t))
        force = (3.0*math.sin(2*math.pi*0.9*t + 1.0), 3.0*math.cos(2*math.pi*0.9*t), 9.8 + 2.0*math.sin(2*math.pi*1.8*t))
        return rate, force

    def integrate(self, h, seconds=2.0, coning=True, trapezoid=True):
        acc = link.ImuAccumulator()
        steps = round(seconds/h)
        if coning and trapezoid:
            for k in range(steps + 1):
                acc.add_sample(*self.motion(k*h), h)
        else:
            # What the firmware must not do: one sample held for the step (rectangle rule).
            for k in range(steps):
                rate, force = self.motion(k*h)
                acc.add_increment(tuple(c*h for c in rate), tuple(c*h for c in force))
        return acc.q, tuple(acc.u)

    def test_one_kilohertz_integration_of_swing_is_far_inside_the_sensor_noise(self):
        ref_q, ref_u = self.integrate(1/32000)
        errors = {}
        for label, h, kwargs in (('1 kHz', 1e-3, {}), ('2 kHz', 5e-4, {}), ('1 kHz rectangle', 1e-3, {'coning': False}),
                                 ('100 Hz rectangle', 1e-2, {'coning': False})):
            q, u = self.integrate(h, **kwargs)
            angle = math.sqrt(sum(c*c for c in link._qlog(link._qmul(link._qconj(ref_q), q))))
            errors[label] = (angle, max(abs(a - b) for a, b in zip(u, ref_u)))
        # 20 deg swing at 0.9 Hz with spin, 2 s. Gyro noise alone gives 0.014 deg/s/rtHz * sqrt(2 s) = 3.5e-4 rad.
        self.assertLess(errors['1 kHz'][0], 1e-5)
        self.assertLess(errors['1 kHz'][1], 1e-4)
        self.assertLess(errors['2 kHz'][0], errors['1 kHz'][0]/3)             # second order in the step
        self.assertGreater(errors['1 kHz rectangle'][0], 100*errors['1 kHz'][0])
        self.assertGreater(errors['100 Hz rectangle'][0], 1e-3)


class GoldenVectors(unittest.TestCase):
    """The bytes a C implementation on the STM32 must reproduce. No C exists; these pin the Python."""

    @classmethod
    def setUpClass(cls):
        cls.v = json.loads(VECTORS.read_text())

    def test_crc_and_stuffing_vectors(self):
        for item in self.v['crc16']:
            self.assertEqual(link.crc(bytes.fromhex(item['data'])), item['crc'])
        for item in self.v['cobs']:
            self.assertEqual(link.cobs_encode(bytes.fromhex(item['data'])).hex(), item['stuffed'])
            self.assertEqual(link.cobs_decode(bytes.fromhex(item['stuffed'])).hex(), item['data'])

    def test_every_message_vector_both_ways(self):
        seen = set()
        for item in self.v['messages']:
            cls = IDS[item['id']]
            seen.add(cls)
            self.assertEqual(cls.__name__, item['message'])
            message = cls.from_raw(item['raw'])
            self.assertEqual(message.raw(), item['raw'], item['name'])
            self.assertEqual(message.payload().hex(), item['payload'], item['name'])
            self.assertEqual(link.encode(message, item['seq']).hex(), item['wire'], item['name'])
            got, = link.Decoder().feed(bytes.fromhex(item['wire']))
            self.assertEqual((type(got), got.seq, got.raw()), (cls, item['seq'], item['raw']), item['name'])
        self.assertEqual(seen, set(SIZES))
        self.assertGreaterEqual(len(self.v['messages']), 3*len(SIZES))

    def test_stream_vectors(self):
        for item in self.v['streams']:
            d = link.Decoder()
            got = d.feed(bytes.fromhex(item['wire']))
            self.assertEqual([[type(m).__name__, m.seq] for m in got], item['decoded'], item['name'])
            self.assertEqual({k: v for k, v in d.stats().items() if v}, item['counters'], item['name'])

    def test_imu_model_vectors_within_tolerance(self):
        for item in self.v['imu_model']:
            acc = link.ImuAccumulator(epoch=item['epoch'])
            for step, expect in zip(item['steps'], item['snapshots']):
                for rate, force in step['samples']:
                    acc.add_sample(rate, force, item['h'])
                raw = acc.snapshot(step['tick']).raw()
                self.assertLessEqual(max(abs(a - b) for a, b in zip(raw['q'], expect['q'])), item['q_tolerance'])
                self.assertLessEqual(max(abs(a - b) for a, b in zip(raw['u'], expect['u'])), item['u_tolerance'])
                self.assertEqual(raw['flags'], expect['flags'])


def write_vectors():
    rng = random.Random(20261011)
    out = {'about': 'Golden vectors for the STM32 <-> Pi link, generated by code/cansat/link.py (protocol version %d). '
                    'raw: the integers of each field in layout order; payload: those packed little-endian; wire: the '
                    'whole frame, COBS(id seq payload crc_lo crc_hi) then 0x00. A C implementation must produce wire '
                    'from raw and seq, and raw and seq from wire. Hex, no separators.' % link.PROTO_VERSION,
           'crc16': [{'data': d.hex(), 'crc': link.crc(d)} for d in (b'', b'123456789', bytes(4), b'\xff'*8, bytes(range(40)))],
           'cobs': [{'data': d.hex(), 'stuffed': link.cobs_encode(d).hex()} for d in (
               b'', b'\x00', b'\x00\x00', b'\x11\x22\x00\x33', b'\x11\x00\x00\x00', b'\x00\x11\x00', bytes(range(1, 254)),
               bytes(253), b'\x01'*252 + b'\x00')],
           'messages': [], 'streams': [], 'imu_model': []}

    def add(name, message, seq):
        out['messages'].append({'name': name, 'message': type(message).__name__, 'id': message.ID, 'seq': seq,
                                'raw': message.raw(), 'payload': message.payload().hex(),
                                'wire': link.encode(message, seq).hex()})

    typical = {
        link.Hello: link.Hello(epoch=7, fw_version=0x0102, reset_cause=link.RESET_POWER, tick_hi=2, tick=123_456_789),
        link.Imu: link.Imu(tick=4_294_960_000, q=(759250125, -759250125, 0, 0), u=(-81234, 5, 2147480000), epoch=7,
                           flags=link.IMU_GYRO_SATURATED, n_gyro=10, n_accel=16),
        link.Baro: link.Baro(tick=1_000_000, pressure_pa=98765.43, temperature_c=-12.34, n_samples=5, flags=1),
        link.Gnss: link.Gnss(tick=2_000_000, itow_ms=302_400_000, year=2027, month=6, day=12, hour=10, minute=30,
                             second=59, valid=7, fix_type=3, fix_flags=1, num_sv=14, link_flags=1, lon_deg=-1.2345678,
                             lat_deg=52.1234567, h_msl_m=312.345, h_acc_m=1.234, v_acc_m=2.345, vel_n=-3.21,
                             vel_e=4.567, vel_d=10.5, s_acc=0.05, p_dop=1.23),
        link.Trig: link.Trig(frame=1234, tick=3_000_000, period_us=200_000, pulse_us=350, flags=0),
        link.Pps: link.Pps(tick=4_000_000, itow_ms=302_401_000, interval=1_000_013, count=321, flags=1),
        link.Status: link.Status(tick=5_000_000, tick_hi=1, epoch=7, state=2, tick_state=4_100_000,
                                 health=link.HEALTH_5V | link.HEALTH_3V3 | link.HEALTH_PI_POWER | link.HEALTH_IMU
                                 | link.HEALTH_PI_ALIVE, battery_v=7.412, baseline_pa=101325.0, origin_lat_deg=52.1,
                                 origin_lon_deg=-1.2, origin_h_msl_m=95.5, altitude_m=287.3, temperature_c=14.25,
                                 telemetry_seq=77, rx_crc=1, rx_lost=2, tx_dropped=0, pi_age_s=0.2, mcu_temp_c=31),
        link.Cmd: link.Cmd(tick=6_000_000, cmd_seq=9, cmd=link.CMD_SHUTDOWN, arg=15),
        link.PiStatus: link.PiStatus(uptime_ms=95_000, tick_echo=5_000_000, echo_age_ms=3, state=link.PI_NAVIGATING,
                                     flags=link.PIFLAG_CAMERA | link.PIFLAG_MAP | link.PIFLAG_NAV, sw_version=3, rx_crc=4,
                                     rx_lost=5, frames=450, cpu_temp_c=58, cpu_load=71),
        link.Nav: link.Nav(tick=7_000_000, frame=455, flags=tm.FLAG_TRN_VALID | tm.FLAG_PRED_VALID | tm.FLAG_CAMERA_OK,
                           landmarks=21, trn_e=-123.4, trn_n=56.7, trn_u=201.3, trn_major_m=1.2, trn_minor_m=0.7,
                           trn_angle_rad=1.0, protection_m=3.5, pred_e=-150.0, pred_n=80.2, time_to_go_s=19.4,
                           pred_major_m=12.0, pred_minor_m=7.5, pred_angle_rad=2.5, vel_e=-2.31, vel_n=1.07, vel_u=-10.42),
        link.Ack: link.Ack(cmd_seq=9, cmd=link.CMD_SHUTDOWN, result=link.ACK_DONE),
        link.TrigCfg: link.TrigCfg(enable=1, cfg_seq=4, period_ms=200, pulse_us=350),
    }
    for seq, (cls, message) in enumerate(typical.items(), 250):
        add('default', cls(), 0)
        add('typical', message, seq % 256)
        low, high = {}, {}
        for name, code, kind in cls.LAYOUT:
            if name is None:
                continue
            lo, hi = RANGE[code[-1]]
            count = int(code[:-1]) if len(code) > 1 else 0
            low[name] = [lo]*count if count else lo
            high[name] = [hi]*count if count else hi
        add('every field at its minimum', cls.from_raw(low), 0)
        add('every field at its maximum', cls.from_raw(high), 255)
        add('random', random_message(cls, rng), rng.randrange(256))

    encoder = link.Encoder()
    encoder.seq = 254
    stream = [typical[link.Imu], typical[link.Baro], typical[link.Trig], typical[link.Gnss], typical[link.Status],
              typical[link.Imu]]
    wires = [encoder.encode(m) for m in stream]

    def add_stream(name, data):
        d = link.Decoder()
        got = d.feed(data)
        out['streams'].append({'name': name, 'wire': data.hex(), 'decoded': [[type(m).__name__, m.seq] for m in got],
                               'counters': {k: v for k, v in d.stats().items() if v}})

    add_stream('six frames, sequence wrapping 255 -> 0', b''.join(wires))
    add_stream('idle zeros before and between', b'\x00\x00' + wires[0] + b'\x00' + wires[1])
    flipped = bytearray(wires[1])
    flipped[5] ^= 0x04
    add_stream('one bit flipped in the second frame', wires[0] + bytes(flipped) + wires[2] + wires[3])
    add_stream('second frame cut short', wires[0] + wires[1][:9] + wires[2] + wires[3])
    add_stream('delimiter of the second frame lost', wires[0] + wires[1][:-1] + wires[2] + wires[3])
    add_stream('two bytes inserted in the second frame', wires[0] + wires[1][:4] + b'\x5a\x00' + wires[1][4:] + wires[2])
    add_stream('third and fourth frames missing', wires[0] + wires[1] + wires[4] + wires[5])
    add_stream('unknown identifier, then a known one with a wrong length',
               link.frame(0x7E, 1, b'\x01\x02') + link.frame(link.Baro.ID, 2, bytes(5)) + link.encode(link.Ack(), 3))
    add_stream('300 bytes with no delimiter, then frames', b'\x41'*300 + wires[0] + wires[1])

    h = 1/1024                                               # exact in binary; ten samples per snapshot
    acc = link.ImuAccumulator(epoch=1)
    steps, snaps = [], []
    for s in range(4):
        samples = []
        for i in range(10):
            t = (s*10 + i)*h
            rate = (round(1.5*math.sin(6*t), 6), round(-0.8*math.cos(4*t), 6), 0.75)
            force = (round(2.0*math.sin(5*t), 6), -1.25, round(9.8 + math.cos(7*t), 6))
            samples.append([list(rate), list(force)])
            acc.add_sample(rate, force, h)
        tick = round((s + 1)*10*h*link.TICK_HZ)
        raw = acc.snapshot(tick).raw()
        steps.append({'tick': tick, 'samples': samples})
        snaps.append({'q': raw['q'], 'u': raw['u'], 'flags': raw['flags']})
    out['imu_model'].append({
        'about': 'ImuAccumulator.add_sample: h seconds between synchronous (rate rad/s, force m/s^2) samples; the first '
                 'sample only primes the trapezoid. Arithmetic in double precision here. A single-precision quaternion '
                 'on the STM32 will not match to the last count: q within q_tolerance counts (Q30), u within '
                 'u_tolerance counts (2^-16 m/s).',
        'epoch': 1, 'h': h, 'q_tolerance': 512, 'u_tolerance': 4, 'steps': steps, 'snapshots': snaps})
    VECTORS.write_text(json.dumps(out, indent=1) + '\n')
    print('wrote %s: %d messages, %d streams' % (VECTORS.name, len(out['messages']), len(out['streams'])))


if __name__ == '__main__':
    if '--write-vectors' in sys.argv:
        write_vectors()
    else:
        unittest.main()
