# STM32-to-Pi serial link protocol

Prepared 11 October 2026; status cross-checked 13 October 2026. Protocol version 1. Companion to the current [design baseline](design_baseline.md), [connection guide](flight_computer_connections.md) (sections 4 and 9), [`code/cansat/telemetry.py`](../code/cansat/telemetry.py), the executable Python reference [`code/cansat/link.py`](../code/cansat/link.py), and [the navigation simulation report](navigation_closed_loop_simulation.md). The 32-byte `Nav` link payload is not the STM32's separate 40-byte radio telemetry packet.

**Status.** This is a protocol design and software simulation, not an implemented or hardware-qualified link. The reference codec `code/cansat/link.py` is Python and has been executed on a development laptop only. It is written for the Raspberry Pi Zero 2 W (decoder, IMU differencing, frame time-stamping, the Pi's own messages) and is the model the STM32F405 firmware must reproduce; it has been executed on neither target processor. No STM32 firmware exists. The available reference is this specification, the Python implementation, and 60 golden message vectors plus 9 stream vectors in `code/tests/link_vectors.json`.

Every number below is tagged: **[calc]** arithmetic shown here, **[sim]** measured in simulation on the laptop, **[src]** from a named source, **[recalled]** from a named source but not re-read while writing this page (check before relying on it), **[assumed]**.

## 1. What the link has to do

The two processors are joined only by a buffered 3.3 V UART: STM32 USART1 (PA9 TX, PA10 RX) through U10 SN74LVC2G125 to Pi GPIO15 (RXD) and GPIO14 (TXD). There is no flow control and no trigger or PPS wire to the Pi. The STM32 can remove the Pi's power (PI_PWR_EN) and tri-state the buffer (PI_UART_OE_N).

| Direction | Carries | Why |
|---|---|---|
| STM32 → Pi | IMU, barometer, GNSS PVT, camera-trigger times, PPS captures, flight state, housekeeping | The Pi navigates but owns no sensor except the camera, and CSI frames carry no hardware time |
| Pi → STM32 | TRN fix and landing prediction in packet units, heartbeat, trigger (exposure) request, acknowledgements | The STM32 builds and transmits the 40-byte packet and decides when the Pi may lose power |

## 2. Decisions

| Question | Decision | Main reason (numbers in the section named) |
|---|---|---|
| IMU content | Snapshots of two running integrals (gyro-integrated attitude quaternion `q`, specific force integrated in that frame `u`) every 10 ms; the Pi subtracts consecutive snapshots | A lost message then costs time resolution only, never the integral (§6, §11) |
| IMU integration on the STM32 | At the sensor rate (1 kHz or more): trapezoidal rate with the two-sample coning term; each force sample rotated by the attitude at its own instant | Error 500 times below sensor noise per interval in the violent descent class [sim] (§6) |
| Framing | COBS with a 0x00 delimiter, CRC-16/CCITT-FALSE, 6 bytes of overhead | Resynchronises at the next delimiter; a floating or broken line reads as empty frames (§4) |
| Time base | TIM2, 32 bits at 1 MHz, in every message; wraps every 71.6 min and is unwrapped by the Pi | 1 µs resolution against a 0.5 ms alignment target; wrap handled by arithmetic, not by resets (§7) |
| Camera frame time | From the TIM2 capture of the trigger edge (Trig message); the Pi's clock is used only to decide *which* trigger a frame belongs to | Linux arrival times are milliseconds uncertain; the capture is not (§7) |
| Baud rate | **230400** 8N1, IMU frames queued ahead of the rest | 22.7 % utilisation, 4.7 ms worst IMU latency, 347 µs for the Pi to service its FIFO [calc] (§9). 460800 is the alternative if the bench shows no overruns |
| Pi serial port | PL011 (`/dev/ttyAMA0`) on GPIO14/15, Bluetooth moved off it, console disabled | The mini UART's baud rate follows the core clock [recalled] (§3) |

## 3. Physical layer

8 data bits, no parity, 1 stop bit, 3.3 V logic, idle high. Ten bit-times per byte.

Baud-rate generation [calc]. STM32 USART1 is on APB2 (84 MHz with the 168 MHz system clock); with 16× oversampling, baud = 84 MHz / BRR. The Pi's PL011 divides its UART clock, 48 MHz by default [recalled: Raspberry Pi firmware `init_uart_clock`], by (IBRD + FBRD/64) × 16.

| Nominal | STM32 BRR | STM32 actual | Pi divisor ×64 | Pi actual | Mismatch |
|---:|---:|---:|---:|---:|---:|
| 115200 | 729 | 115226 (+0.02 %) | 1667 | 115177 (−0.02 %) | 0.04 % |
| **230400** | 365 | 230137 (−0.11 %) | 833 | 230492 (+0.04 %) | 0.15 % |
| 460800 | 182 | 461538 (+0.16 %) | 417 | 460432 (−0.08 %) | 0.24 % |
| 921600 | 91 | 923077 (+0.16 %) | 208 | 923077 (+0.16 %) | 0.00 % |

All four are far inside what an 8N1 receiver tolerates (a few percent; RM0090 has the USART receiver tolerance table [recalled]). The STM32 ceiling is fPCLK/16 = 5.25 Mbit/s [calc; the STM32F405 datasheet's USART feature table was not re-read]. The PL011 ceiling is 48 MHz/16 = 3 Mbit/s [calc].

Pi configuration [recalled, from the Raspberry Pi documentation's UART section, <https://www.raspberrypi.com/documentation/computers/configuration.html>; could not be re-read in this session, verify on the device]:

- On a Pi with Bluetooth the PL011 serves Bluetooth and GPIO14/15 get the mini UART, whose baud rate depends on the VPU core clock. Put the PL011 on the pins with `dtoverlay=disable-bt` (or `miniuart-bt`), and remove the serial console from `cmdline.txt`.
- The PL011 has 16-deep FIFOs [recalled: BCM2835 ARM Peripherals manual]. Linux services the receive FIFO at half full or after an idle timeout [recalled: `amba-pl011` driver]. After the half-full interrupt there are 8 byte-times before an overrun: 694, 347, 174 and 87 µs at the four rates [calc]. There is no flow control, so this margin is the only protection against a late interrupt. It is why the recommendation is not the fastest rate.
- GPIO15 (RXD) floats when U10 is disabled. Enable the Pi's pull-up on it (`gpio=15=pu` in `config.txt`) [assumed sufficient; not tested]. A line held low is a break, which the decoder counts as empty frames and ignores.

STM32 side: transmit and receive by DMA (receive circular with the idle-line interrupt), so the STM32 has no overrun exposure of its own. PA10 already has a 10 kΩ pull-up.

## 4. Framing

```
COBS( id | seq | payload | crc_lo | crc_hi )   0x00
```

| Part | Size | Meaning |
|---|---:|---|
| id | 1 | Message identifier. Bit 7 set: Pi → STM32 |
| seq | 1 | One counter per direction, +1 for every frame sent, wraps at 256 |
| payload | fixed per id, ≤ 249 | Little-endian fields (§5) |
| crc | 2 | CRC-16/CCITT-FALSE (polynomial 0x1021, initial 0xFFFF, no reflection, check value 0x29B1 for "123456789") over id, seq and payload; low byte first. The same CRC as the downlink packet |
| stuffing | 1 | Consistent-overhead byte stuffing over the four parts above. With at most 253 bytes to stuff, the overhead is exactly one byte and the code 0xFF never occurs |
| delimiter | 1 | 0x00, which appears nowhere else |

Overhead is 6 bytes per message. COBS encoding for this size: output a code byte *n*+1 followed by the *n* non-zero bytes that precede each zero (or the end); the zero itself is not output. Decoding: read a code *c*, copy *c*−1 bytes, and emit a zero unless that was the end.

Receiver rules (implemented by `link.Decoder`, which never raises and keeps at most 255 bytes):

1. Collect bytes up to a 0x00. An empty run is an idle line: ignore it.
2. A run longer than 254 bytes is discarded up to the next delimiter.
3. Unstuff; refuse invalid stuffing, fewer than 4 bytes, a CRC mismatch, or a known id with the wrong payload length. A good frame with an unknown id is counted and skipped, so new messages can be added without breaking an old receiver.
4. Frames missing by sequence number are counted (modulo 256: a gap of exactly 256 frames is invisible; at 127 frames/s that is 2.0 s, and the Imu message's own tick shows such a gap).

Every kind of rejection has its own counter (`bytes, frames, empty, too_long, stuffing, short, crc, unknown, length, lost, repeats, ignored, faults`). Both sides report their `crc` and `lost` counts to the other (Status, PiStatus) so a marginal link is visible in telemetry logs.

Resynchronisation bound [sim, `test_link.py`]: after any damage the decoder is in step at the first delimiter that follows it. A single damaged byte costs one frame, or two if the byte was a delimiter. Undetected corruption needs a 16-bit CRC collision together with a valid id and the exact length for that id; none occurred in about 0.24 MB of random bytes or in any of the 2,816 single-bit errors tested (every single-bit error is detected by construction).

## 5. Messages

Scaled values are rounded with floor(x/step + 0.5) (not round-half-even). "NA" is the most negative value of a signed field and the largest of an unsigned one; scaled values saturate one step short of it. Counters wrap. Reserved bytes are sent as zero and ignored.

Rates and sizes [calc]:

| id | Name | Direction | Payload | On wire | Rate | Bytes/s |
|---|---|---|---:|---:|---:|---:|
| 0x01 | Hello | STM32 → Pi | 16 | 22 | at boot, and when the Pi appears | – |
| 0x10 | Imu | STM32 → Pi | 36 | 42 | 100 Hz | 4200 |
| 0x11 | Baro | STM32 → Pi | 12 | 18 | 10 Hz | 180 |
| 0x12 | Gnss | STM32 → Pi | 60 | 66 | 10 Hz budgeted | 660 |
| 0x13 | Trig | STM32 → Pi | 16 | 22 | 5 Hz | 110 |
| 0x14 | Pps | STM32 → Pi | 16 | 22 | 1 Hz | 22 |
| 0x15 | Status | STM32 → Pi | 48 | 54 | 1 Hz + on state change | 54 |
| 0x16 | Cmd | STM32 → Pi | 8 | 14 | 2 Hz until acknowledged | – |
| | | | | | **STM32 → Pi total** | **5226** |
| 0x80 | PiStatus | Pi → STM32 | 24 | 30 | 2 Hz | 60 |
| 0x81 | Nav | Pi → STM32 | 32 | 38 | 5 Hz | 190 |
| 0x82 | Ack | Pi → STM32 | 4 | 10 | on command | – |
| 0x83 | TrigCfg | Pi → STM32 | 8 | 14 | 1 Hz + on change | 14 |
| | | | | | **Pi → STM32 total** | **264** |

### 0x10 Imu (36 bytes)

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | tick | u32, TIM2 time of the last gyro sample included |
| 4 | 16 | q | 4 × i32, w x y z, Q30 (1.0 = 2^30). Body axes → the frame the can had when integration started |
| 20 | 12 | u | 3 × i32, 2^-16 m/s per count, wrapping. Specific force rotated by q and integrated |
| 32 | 1 | epoch | changes whenever q and u restart (boot counter) |
| 33 | 1 | flags | 1 gyro saturated, 2 accel saturated, 4 gyro sample missed, 8 accel sample missed, 16 first snapshot after a restart, 32 gyro fault, 64 accel fault; each covers the time since the previous snapshot |
| 34 | 1 | n_gyro | gyro samples since the previous snapshot |
| 35 | 1 | n_accel | accelerometer samples since the previous snapshot |

Section 6 defines the integrals. Samples are raw: the STM32 applies the fixed axis mapping and scale to SI units and nothing else. Bias, scale-factor and misalignment calibration are applied by the Pi, where the filter that estimates them lives.

### 0x11 Baro (12 bytes)

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | tick | mid-time of the conversions averaged |
| 4 | 4 | pressure | u32, 0.01 Pa, compensated (MS5611 second-order algorithm) |
| 8 | 2 | temperature | i16, 0.01 °C, sensor die |
| 10 | 1 | n_samples | pressure conversions averaged |
| 11 | 1 | flags | 1 valid |

At OSR 4096 one pressure and one temperature conversion take up to 2 × 9.04 ms [src: TE datasheet via [navigation_parameter_sources](navigation_parameter_sources.md)], so about 50 pressure/temperature pairs per second are possible and the 10 Hz message averages up to 5. The Pi converts pressure to height with the pad baseline from Status and `atmosphere.pressure_altitude`.

### 0x12 Gnss (60 bytes)

Fields, units and widths are those of UBX-NAV-PVT, so the STM32 copies them without arithmetic [recalled: u-blox M10 SPG 5.10 interface description, UBX-21035062; the field list was written from memory of that document and must be checked against it].

| Offset | Size | Field | Coding (UBX-NAV-PVT name) |
|---:|---:|---|---|
| 0 | 4 | tick | TIM2 time of the navigation epoch: the last PPS capture plus (iTOW mod 1000) ms when link_flags bit 0 is set, otherwise the arrival time of the UBX frame |
| 4 | 4 | itow_ms | iTOW |
| 8 | 2+5 | year, month, day, hour, minute, second | UTC; lets the Pi set its clock, which has no battery |
| 15 | 1 | valid | valid |
| 16 | 1 | fix_type | fixType |
| 17 | 1 | fix_flags | flags (bit 0 gnssFixOK) |
| 18 | 1 | num_sv | numSV |
| 19 | 1 | link_flags | 1 tick derived from PPS, 2 stale (repeat of an old epoch) |
| 20 | 4 | lon | i32, 1e-7 deg |
| 24 | 4 | lat | i32, 1e-7 deg |
| 28 | 4 | h_msl | i32, mm (hMSL) |
| 32 | 4 | h_acc | u32, mm |
| 36 | 4 | v_acc | u32, mm |
| 40 | 12 | vel_n, vel_e, vel_d | i32, mm/s |
| 52 | 4 | s_acc | u32, mm/s |
| 56 | 2 | p_dop | u16, 0.01 |
| 58 | 2 | reserved | |

This is the GNSS velocity `landing.py` asks for (velE, velN, sAcc). The receiver's rate is a configuration choice (10 Hz needs GPS+Galileo only [src: parameter sources §5]); the budget assumes 10 Hz.

### 0x13 Trig (16 bytes)

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | frame | u32, trigger counter since triggers last started |
| 4 | 4 | tick | TIM2 channel 4 capture of the falling edge on PB0 (exposure start) |
| 8 | 4 | period_us | trigger period (200000 at 5 Hz) |
| 12 | 2 | pulse_us | low time; exposure = pulse + 14.26 µs [src: Raspberry Pi camera documentation] |
| 14 | 1 | flags | 1 first trigger after a start |
| 15 | 1 | reserved | |

### 0x14 Pps (16 bytes)

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | tick | TIM2 channel 1 capture of the rising edge |
| 4 | 4 | itow_ms | GNSS time of week of that edge, or NA |
| 8 | 4 | interval | ticks since the previous pulse, 0 if it was missed. Nominally 1,000,000; the difference is the crystal's frequency error |
| 12 | 2 | count | pulses since boot |
| 14 | 1 | flags | 1 itow valid |
| 15 | 1 | reserved | |

### 0x15 Status (48 bytes)

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | tick | |
| 4 | 2 | tick_hi | wraps of TIM2 since boot |
| 6 | 1 | epoch | boot counter |
| 7 | 1 | state | `flight_state.State`: 0 PAD, 1 ASCENT, 2 DESCENT, 3 LANDED |
| 8 | 4 | tick_state | tick at which that state was entered |
| 12 | 2 | health | bits 0–15: 5 V good, 3.3 V good, Pi power on, UART buffer enabled, IMU, barometer, GNSS talking, GNSS fix, PPS, radio, flash, temperature ADC, triggers running, origin valid, baseline valid, Pi alive |
| 14 | 2 | battery | u16, mV |
| 16 | 4 | baseline | u32, 0.01 Pa: the pad pressure the state machine holds |
| 20 | 12 | origin lat, lon, h_msl | i32 1e-7 deg, 1e-7 deg, mm: the pad position the STM32 uses for gps e/n/u |
| 32 | 2 | altitude | i16, 0.1 m above pad |
| 34 | 2 | temperature | i16, 0.01 °C, thermistor (primary mission) |
| 36 | 2 | telemetry_seq | sequence of the last downlink packet |
| 38 | 2 | rx_crc | frames from the Pi rejected |
| 40 | 2 | rx_lost | frames from the Pi missing by sequence |
| 42 | 2 | tx_dropped | frames the STM32 could not queue |
| 44 | 1 | pi_age | u8, 0.1 s since the last valid frame from the Pi; 255 NA |
| 45 | 1 | mcu_temp | i8, °C |
| 46 | 2 | reserved | |

The origin and baseline are here because both processors must use the same ones: the STM32 needs them without the Pi (it fills gps e/n/u and altitude itself), and the Pi needs them to put GNSS and TRN in the same frame.

### 0x01 Hello (16 bytes) and 0x16 Cmd (8 bytes)

| Message | Offset | Size | Field | Coding |
|---|---:|---:|---|---|
| Hello | 0 | 1 | proto | protocol version, 1 |
| | 1 | 1 | epoch | boot counter kept in a backup register |
| | 2 | 2 | fw_version | |
| | 4 | 1 | reset_cause | 1 power, 2 pin, 4 watchdog, 8 software, 16 brown-out |
| | 5 | 1 | flags | reserved |
| | 6 | 2 | tick_hi | |
| | 8 | 4 | tick | |
| | 12 | 4 | tick_hz | 1000000 |
| Cmd | 0 | 4 | tick | |
| | 4 | 1 | cmd_seq | |
| | 5 | 1 | cmd | 1 SHUTDOWN, 2 PREPARE (triggers are about to start) |
| | 6 | 2 | arg | SHUTDOWN: seconds until power is removed regardless |

### 0x80 PiStatus (24 bytes)

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | uptime_ms | |
| 4 | 4 | tick_echo | tick of the newest STM32 frame that carried one |
| 8 | 2 | echo_age | ms between receiving it and sending this; with tick_echo it gives the STM32 the round-trip delay |
| 10 | 1 | proto | 1 |
| 11 | 1 | state | 0 booting, 1 ready, 2 camera running, 3 navigating, 4 stopping, 5 fault |
| 12 | 2 | flags | 1 camera OK, 2 map loaded, 4 storage OK, 8 under-voltage seen, 16 throttled, 32 filter initialised |
| 14 | 2 | sw_version | |
| 16 | 2 | rx_crc | |
| 18 | 2 | rx_lost | |
| 20 | 2 | frames | camera frames received, wraps |
| 22 | 1 | cpu_temp | i8, °C |
| 23 | 1 | cpu_load | percent |

### 0x81 Nav (32 bytes)

Fields shared with the 40-byte packet have the packet's own coding, so the STM32 copies bytes (`link.apply_nav` is the reference; a test checks the packet is identical byte for byte to one built directly).

| Offset | Size | Field | Coding |
|---:|---:|---|---|
| 0 | 4 | tick | STM32 time the solution is valid at |
| 4 | 2 | frame | trigger counter of the newest image used |
| 6 | 1 | flags | 0x20 TRN valid, 0x40 prediction valid, 0x80 camera OK (the packet's bits); other bits ignored |
| 7 | 1 | landmarks | accepted in the last fix |
| 8 | 6 | trn e, n, u | i16, 0.1 m (u is for the flash log; the packet has no field for it) |
| 14 | 2 | trn ellipse major, minor | u8, 0.1 m |
| 16 | 1 | trn ellipse angle | u8, 180/256° |
| 17 | 1 | protection | u8, 0.5 m |
| 18 | 4 | predicted e, n | i16, 0.1 m |
| 22 | 1 | time to go | u8, 0.2 s |
| 23 | 2 | predicted ellipse major, minor | u8, 0.5 m |
| 25 | 1 | predicted ellipse angle | u8, 180/256° |
| 26 | 6 | velocity e, n, u | i16, 0.01 m/s (flash log) |

The STM32 must clear the TRN, prediction and camera flags and send "not available" when the newest Nav is older than a limit (2 s proposed [assumed]); a stopped Pi must not leave a stale fix in the downlink.

### 0x82 Ack (4 bytes) and 0x83 TrigCfg (8 bytes)

| Message | Offset | Size | Field | Coding |
|---|---:|---:|---|---|
| Ack | 0 | 1 | cmd_seq | of the Cmd answered |
| | 1 | 1 | cmd | |
| | 2 | 1 | result | 0 accepted, 1 done, 2 refused, 3 unknown command |
| | 3 | 1 | reserved | |
| TrigCfg | 0 | 1 | enable | 1 run triggers |
| | 1 | 1 | cfg_seq | |
| | 2 | 2 | period_ms | 200 for 5 Hz |
| | 4 | 2 | pulse_us | trigger low time, which sets the exposure |
| | 6 | 2 | reserved | |

TrigCfg exists because the exposure is set by the trigger pulse width, which only the STM32 generates, while only the Pi sees the images. The STM32 reports what it actually does in every Trig message, which is the acknowledgement.

## 6. The IMU stream

### What the STM32 integrates

Two integrals run from the raw samples for as long as the firmware runs:

- **q**, the attitude from the gyro alone. For consecutive gyro samples ω₀, ω₁ a time h apart: φ = h(ω₀+ω₁)/2 + (h²/12)(ω₀×ω₁); q ← q ⊗ exp(φ); normalise. The cross-product term is the two-sample coning correction.
- **u**, specific force in the q frame: u ← u + h(q₀·f₀ + q₁·f₁)/2, each force sample rotated by the attitude at its own instant. Rotating before summing is what removes sculling.

A snapshot (tick, q, u) is sent after every 10th sample at a 1 kHz gyro rate. `link.ImuAccumulator.add_sample` is this algorithm; the firmware must match it (tolerance vectors in `link_vectors.json`, `imu_model`). Not modelled: the BMI088's gyro (2000 or 1000 Hz) and accelerometer (1600 Hz) are separate sensors with separate data-ready captures, so the firmware must rotate each accelerometer sample by q interpolated to its own capture time; and the sensors' group delay, which the datasheet does not give [src: parameter sources §1].

Arithmetic on the STM32 [calc]: u must be kept in 64-bit fixed point or double precision. At rest it grows by 9.8 m/s every second; after 3 h it is 1.06 × 10⁵ m/s, and a single-precision float at that size has a resolution of 0.008 m/s, 500 times the 2^-16 m/s count. The 32-bit field on the wire is the low 32 bits and wraps every 2^32 × 2^-16 / 9.81 = 6683 s at 1 g; differences are taken modulo 2^32 and are exact for any gap the link allows. q may be single precision: its rounding (6 × 10⁻⁸ per update) is 100 times below the gyro noise of one 1 ms sample (7.7 × 10⁻⁶ rad). Single precision was not simulated.

How good the integration is [sim: the descent model's own 1 kHz samples through `ImuAccumulator.add_sample`, against the model's exact pose increments; 8 descents × 10 s per class. `sim/nav/test_link_mc.py` repeats it on 3 descents × 4 s with pass limits]:

| Descent class (max swing, RMS rate) | Error per 10 ms, Δθ RMS / max | Δv RMS / max | Attitude drift over 10 s |
|---|---|---|---|
| violent (51°, 112 °/s) | 5 × 10⁻⁸ / 3 × 10⁻⁷ rad | 7 × 10⁻⁶ / 4 × 10⁻⁵ m/s | 1.2 × 10⁻⁶ rad |
| moderate (40°, 66 °/s) | 5 × 10⁻⁸ / 9 × 10⁻⁷ rad | 6 × 10⁻⁶ / 2 × 10⁻⁵ m/s | 0.9 × 10⁻⁶ rad |
| Sensor noise for comparison [calc from src] | 2.4 × 10⁻⁵ rad (0.014 °/s/√Hz) | 1.7 × 10⁻⁴ m/s (175 µg/√Hz) | 7.7 × 10⁻⁴ rad random walk |

Averaging the 1 kHz samples over 10 ms with no coning term (a "boxcar") drifts 1.9 × 10⁻³ rad in 10 s in the violent class [sim], 2.4 times the gyro's own random walk. The filter absorbs that in simulation (`imu_boxcar_100hz`, and the boxcar row of §11), but there is no reason to send it.

### Resolution against sensor noise [calc]

| Field | Step | Error in an interval's increment | Sensor noise in 10 ms | Ratio |
|---|---|---|---|---:|
| q, Q30 | 9.3 × 10⁻¹⁰ | about 2 × 10⁻⁹ rad | 2.4 × 10⁻⁵ rad | 10⁻⁴ |
| u, 2^-16 m/s | 1.53 × 10⁻⁵ m/s | 6.2 × 10⁻⁶ m/s RMS (difference of two roundings) | 1.7 × 10⁻⁴ m/s | 0.036 |
| Baro pressure, 0.01 Pa | 0.01 Pa | 0.003 Pa RMS | 1.2 Pa RMS per conversion, 0.54 Pa for 5 averaged | 0.005 |

Because the fields are running totals, their rounding does not accumulate: over 3000 intervals the summed Δv error stayed under 40 counts where independent rounding of each increment would random-walk [sim, `test_link.py`]. No field saturates in flight: q is bounded, u wraps by design, and sensor saturation is reported in flags.

### What the Pi does with it

`link.ImuDifferencer` turns consecutive snapshots i, j into an `ImuIncrement`: dt = (tick_j − tick_i) mod 2^32; Δθ = log(q_i⁻¹ q_j); Δv = q_i⁻¹ (u_j − u_i), in the body axes at i. `ImuIncrement.mean()` gives the form `Eskf.propagate` takes (rate Δθ/dt; force = Δv/dt turned back by Δθ/2 to mid-interval axes). If snapshots between i and j were lost, the increment simply spans the gap and is exact for the mean motion. The filter subtracts its bias estimates from the means as it does now; over a bridged gap of T seconds that is a first-order correction, with an error of order (bias × T) × (rotation in the gap), negligible for a calibrated bias of 0.1 °/s and T ≤ 0.5 s [calc: 9 × 10⁻⁴ rad × rotation].

The thread breaks, and the filter must be told its inertial information stopped, when: the epoch changes or the restart flag is set (STM32 reset), the gap exceeds a limit (2 s proposed [assumed]), or a quaternion is not of unit length. Each has a counter.

### The alternatives, and what loss costs in each

| Scheme | Each message carries | Wire bytes | Bytes/s at 100 Hz | A lost message |
|---|---|---:|---:|---|
| delta | Δθ, Δv of its own 10 ms (6 × int16, 2^-16 rad and 2^-13 m/s, rounding residual carried) | 26 | 2600 | That interval is gone; the Pi interpolates its neighbours |
| redundant | the same, plus the previous interval's | 38 | 3800 | Recovered unless two in a row are lost; a burst is not helped |
| **cumulative (chosen)** | q and u as above | 42 | 4200 | Nothing is lost but time resolution, for any pattern of loss up to the gap limit |

(The delta and redundant sizes include the 4 bytes of epoch, flags and sample counts for a like-for-like comparison.) The cumulative scheme costs 1600 bytes/s more than delta, 7 % of the capacity at 230400 baud. The measured navigation consequences are in §11.

## 7. Time

**Counter.** TIM2 is a 32-bit timer [recalled: STM32F405 datasheet]. Run it at 1 MHz (prescaler 84 from the 84 MHz APB1 timer clock [calc]) and never reset it. Resolution 1 µs against the 0.5 ms camera-to-IMU target. It wraps every 2^32 µs = 4294.97 s = 71.6 min, so two or three times in a 3 h pad wait plus flight [calc]. The firmware counts wraps in `tick_hi` (update interrupt), sent in Hello and Status, for the logs.

**The Pi never needs the high word to navigate.** Every interval it forms is a difference modulo 2^32, exact for anything under 35.8 min. `link.TickClock` extends ticks to 64 bits by taking each new tick as the value nearest the last one (`unwrap32`); tested through four wraps with messages up to 30 min apart [sim].

**All navigation runs in STM32 time**, as `landing.py` already assumes. The Pi's clock is used for one thing: deciding which trigger a camera frame belongs to.

**Camera frames.** A frame arrives over CSI with the Pi's receive timestamp only. The procedure (`link.FrameStamper`):

1. For every message with a tick, note (Pi monotonic time at reception − tick). Transport only adds delay, so the smallest value over the last 64 messages is the clock offset, good to about the serial latency (milliseconds).
2. Convert the frame's receive time to an approximate tick, subtract the expected exposure-to-reception latency (20 ms [assumed]; measure it), and take the nearest trigger. Triggers are 200 ms apart, so milliseconds of error do not matter; a frame further than a quarter period from any trigger is refused and counted.
3. The frame's time is that trigger's captured tick plus half of (pulse + 14.26 µs): the middle of the exposure, to the microsecond, on the same counter as the IMU.
4. If the Trig message for a frame was lost, the newest Trig received plus a whole number of periods gives the same tick exactly, because TIM3 (triggers) and TIM2 (capture) run from the same clock. Tested with 20 consecutive Trig messages lost and across a wrap [sim].

What remains is the trigger-edge-to-exposure latency inside the sensor, which is not documented [src: parameter sources §3]; the filter's camera time-offset state is there to absorb it.

**PPS.** The Pps message gives the tick of each pulse and its GNSS time of week, and the interval between pulses in ticks. That calibrates the crystal (the navigation does not need it at ppm level over a 30 s descent, but the logs do) and ties STM32 time to UTC for the ground.

## 8. Start-up, shutdown and faults

**STM32 boot.** Buffer disabled, Pi off (hardware defaults). Start TIM2, sensors, the integrals (new epoch, restart flag on the first snapshot), telemetry and the flash log. None of this waits for the Pi.

**Pi start.** OE high → check PGOOD_5V → PI_PWR_EN high → settle → OE low. The STM32 then transmits its normal stream at once and continuously; it does not wait for the Pi to listen. Bytes sent while the Pi boots are lost and that is harmless: every message is self-contained and the IMU stream can be joined at any snapshot. The Pi's service opens the port, discards up to the first delimiter, and starts sending PiStatus (state booting, then ready). On the first valid frame from a Pi that was silent, the STM32 sends Hello. The Pi sends TrigCfg when the camera is ready; the STM32 starts TIM3 and Trig messages follow.

**Normal running.** Any valid frame from the Pi resets the STM32's Pi-alive timer; PiStatus at 2 Hz guarantees one.

**Pi loss.** No valid frame for 3 s [assumed] → clear Pi-alive, mark the TRN and prediction fields not available (the 2 s Nav age limit will already have done so), keep transmitting. Recovery policy [assumed, to be decided with the power budget]: after 10 s of silence in PAD state, power-cycle the Pi (OE high, power off, 2 s, power on); in ASCENT or DESCENT a power cycle costs the whole descent's imagery (Pi boot is tens of seconds [assumed]) and is only worth it if the Pi has been silent since before launch.

**Clean shutdown.** STM32 sends Cmd SHUTDOWN (arg = seconds allowed) at 2 Hz. The Pi answers Ack accepted, stops the camera and navigation, syncs its storage, sends Ack done, and halts. The STM32 stops triggers and drives PB0 low, waits for Ack done plus 2 s or for the allowed time, then OE high and PI_PWR_EN low. The data partition must tolerate the timeout path (the connection guide and BOM already require a read-only OS).

**STM32 reset in flight.** TIM2, the integrals and the sequence counter restart. The Pi sees, in whichever arrives first: Hello, a new epoch in Imu or Status, or the restart flag. It then calls `TickClock.restart()` and `Decoder.reset_sequence()`, drops its trigger history, and tells the filter that inertial data stopped at the last snapshot before the reset. The epoch is a boot counter in a backup register so it differs across resets; after a power loss the Pi has lost power too.

**Buffer disabled while the Pi runs.** The Pi's receiver sees idle or break: empty frames, counted, ignored. The STM32's receiver sees idle (pull-up).

**Version mismatch.** Hello and PiStatus carry the protocol version. Unknown ids are skipped; a changed layout of an existing id must take a new id.

## 9. Throughput and latency [calc]

Load STM32 → Pi: 5226 bytes/s = 52.3 kbit/s on the wire (§5).

| Baud | Capacity, bytes/s | Utilisation | Worst IMU latency, IMU first | Worst IMU latency, plain FIFO | Pi FIFO margin |
|---:|---:|---:|---:|---:|---:|
| 115200 | 11520 | 45.4 % | 9.4 ms | 19.4 ms | 694 µs |
| **230400** | 23040 | **22.7 %** | **4.7 ms** | 9.7 ms | **347 µs** |
| 460800 | 46080 | 11.3 % | 2.3 ms | 4.9 ms | 174 µs |
| 921600 | 92160 | 5.7 % | 1.2 ms | 2.4 ms | 87 µs |

"IMU first": the transmit queue lets an Imu frame pass everything except the frame already being sent, so it waits at most for the longest other frame (Gnss, 66 bytes) plus its own 42. "Plain FIFO": one of every other message is ahead of it (182 bytes) plus its own. Both are queue-entry to last bit. At 115200 without priority an Imu frame can wait longer than its own period.

Latency does not affect accuracy, because every measurement carries its own tick; it only delays the navigation output, which is already hundreds of milliseconds behind the image it uses. So the choice is between headroom and the Pi's interrupt margin:

- **115200**: 45 % used. No room to add a message or raise a rate; rejected.
- **230400 (recommended)**: 4.4 times headroom, and twice the interrupt margin of 460800. With the cumulative IMU stream an occasional overrun costs nothing measurable (§11), so the margin need not be perfect.
- **460800**: take it if the bench test shows no overruns under full camera and matching load; it halves the latency and leaves room for raw 1 kHz logging to the Pi later.
- **921600**: no benefit for this load.

Pi → STM32: 264 bytes/s, 1.1 % at 230400.

Codec speed [sim, laptop, pure Python]: decode 1.8 MB/s (43,000 frames/s), encode 1.2 MB/s. That is 340 and 230 times the link's load. The Pi Zero 2 W has not been timed; if it were 20 times slower the decoder would still use about 6 % of one core [calc from an assumed factor].

## 10. What the STM32 firmware must implement

1. Frame, COBS, CRC, sequence counter: reproduce `wire` from `raw` and `seq` for all 60 message vectors, and the decoded list and counters for the 9 stream vectors.
2. The two integrals of §6 at the sensor rate, u in 64-bit fixed point or double; match the `imu_model` vectors within the stated tolerance.
3. A transmit queue that sends Imu first and drops (and counts in `tx_dropped`) rather than blocks.
4. Receive by DMA; decode Nav, PiStatus, Ack, TrigCfg; apply the Nav age limit.
5. The sequences of §8.

## 11. Closed-loop check

`sim/nav/run_link_mc.py` puts the codec between the simulated sensors and the navigation filter of the existing Monte Carlo (`run_nav_mc.py`, imported unmodified): the 100 Hz IMU increments and 10 Hz barometer samples are encoded on a simulated STM32 side together with Gnss, Trig, Pps and Status traffic, carried over a simulated UART that corrupts or cuts the byte stream, decoded with `link.Decoder`, and given to the filter. The copy of the Monte Carlo loop used for this reproduces `run_nav_mc.run_descent` exactly when the link is left out (tested, and the `direct` row below).

Conventions. "Loss" is the fraction of frames (of every kind) given one flipped bit, delimiter included; every scheme sees the same losses. The outage removes every byte from 8.0 s to 8.5 s into the descent (about 215 m up), cutting a frame at each end. An image or barometer sample whose time falls in an IMU gap longer than 30 ms is not used [assumed policy]; in shorter gaps the bridged increment is split at the measurement time. Under the delta and redundant schemes a missing interval is interpolated from its neighbours and the filter's attitude and velocity variances are raised by (1 mrad)² and (5 mm/s)² per interpolated interval [assumed]. Errors at image times inside the outage are scored on the bridged or interpolated state. T1 is the fraction of descents that stay within 10 m horizontally above the 80 m map floor.

All figures are [sim], on the descents stated, with the baseline sensor errors of `run_nav_mc.py`. They come from `sim/nav/results/link/summary_nominal_moderate.json` and `summary_nominal_violent.json`. "Cumulative" is the scheme adopted (§6): each message carries the running integrals. "Delta" carries one interval per message; "redundant" carries the current and the previous interval.

**Moderate canopy (12° trim), 160 descents.**

| Case | Horizontal RMS, m | 95 %, m | Worst, m | Within 10 m (T1) | Tilt RMS, deg | IMU messages lost | Longest IMU gap, ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| unmodified Monte Carlo | 0.607 | 1.106 | 2.74 | 100.0 % | 0.22 | - | - |
| this loop, no link | 0.607 | 1.106 | 2.74 | 100.0 % | 0.22 | - | - |
| cumulative, no errors | 0.607 | 1.106 | 2.74 | 100.0 % | 0.22 | 0.0 % | 10 |
| cumulative, 0.1 % loss | 0.607 | 1.107 | 2.74 | 100.0 % | 0.22 | 0.1 % | 30 |
| cumulative, 1 % loss | 0.607 | 1.107 | 2.73 | 100.0 % | 0.22 | 1.0 % | 40 |
| cumulative, 5 % loss | 0.607 | 1.108 | 2.75 | 100.0 % | 0.22 | 5.1 % | 50 |
| cumulative, 0.5 s outage | 0.608 | 1.108 | 2.77 | 100.0 % | 0.40 | 1.8 % | 510 |
| cumulative, 5 % loss and the outage | 0.608 | 1.104 | 2.79 | 100.0 % | 0.40 | 6.8 % | 520 |
| cumulative, 1 byte in 1000 replaced | 0.607 | 1.106 | 2.73 | 100.0 % | 0.22 | 4.2 % | 50 |
| delta, no errors | 0.607 | 1.106 | 2.74 | 100.0 % | 0.22 | 0.0 % | 10 |
| delta, 0.1 % loss | 0.607 | 1.104 | 2.70 | 100.0 % | 0.22 | 0.1 % | 30 |
| delta, 1 % loss | 0.611 | 1.100 | 2.57 | 100.0 % | 0.22 | 1.0 % | 40 |
| delta, 5 % loss | 0.633 | 1.159 | 2.13 | 100.0 % | 0.24 | 5.2 % | 50 |
| delta, 0.5 s outage | 0.667 | 1.204 | 3.01 | 100.0 % | 1.08 | 1.8 % | 510 |
| redundant, no errors | 0.607 | 1.106 | 2.74 | 100.0 % | 0.22 | 0.0 % | 10 |
| redundant, 0.1 % loss | 0.607 | 1.107 | 2.74 | 100.0 % | 0.22 | 0.1 % | 30 |
| redundant, 1 % loss | 0.607 | 1.108 | 2.73 | 100.0 % | 0.22 | 1.0 % | 40 |
| redundant, 5 % loss | 0.608 | 1.106 | 2.74 | 100.0 % | 0.22 | 5.1 % | 50 |
| redundant, 0.5 s outage | 0.667 | 1.204 | 3.01 | 100.0 % | 1.07 | 1.8 % | 510 |
| unmodified Monte Carlo, IMU as 10 ms boxcar means | 0.607 | 1.105 | 2.74 | 100.0 % | 0.22 | - | - |
| cumulative, STM32 sums samples with no coning term | 0.607 | 1.105 | 2.74 | 100.0 % | 0.22 | 0.0 % | 10 |

**Violent canopy (19° RMS swing), 64 descents.**

| Case | Horizontal RMS, m | 95 %, m | Worst, m | Within 10 m (T1) | Tilt RMS, deg | IMU messages lost | Longest IMU gap, ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| unmodified Monte Carlo | 0.381 | 0.667 | 1.27 | 100.0 % | 0.14 | - | - |
| this loop, no link | 0.381 | 0.667 | 1.27 | 100.0 % | 0.14 | - | - |
| cumulative, no errors | 0.381 | 0.667 | 1.27 | 100.0 % | 0.14 | 0.0 % | 10 |
| cumulative, 1 % loss | 0.382 | 0.667 | 1.27 | 100.0 % | 0.14 | 1.0 % | 40 |
| cumulative, 5 % loss | 0.381 | 0.666 | 1.27 | 100.0 % | 0.14 | 5.1 % | 50 |
| cumulative, 0.5 s outage | 0.387 | 0.685 | 1.27 | 100.0 % | 1.73 | 1.8 % | 510 |
| delta, no errors | 0.381 | 0.667 | 1.27 | 100.0 % | 0.14 | 0.0 % | 10 |
| delta, 1 % loss | 0.382 | 0.671 | 1.27 | 100.0 % | 0.14 | 1.0 % | 40 |
| delta, 5 % loss | 0.385 | 0.683 | 1.25 | 100.0 % | 0.14 | 5.2 % | 50 |
| delta, 0.5 s outage | 40.932 | 15.130 | 653.26 | 50.0 % | 14.49 | 1.8 % | 510 |
| redundant, no errors | 0.381 | 0.667 | 1.27 | 100.0 % | 0.14 | 0.0 % | 10 |
| redundant, 1 % loss | 0.382 | 0.668 | 1.27 | 100.0 % | 0.14 | 1.0 % | 40 |
| redundant, 5 % loss | 0.381 | 0.671 | 1.27 | 100.0 % | 0.14 | 5.1 % | 50 |
| redundant, 0.5 s outage | 40.891 | 14.955 | 652.73 | 50.0 % | 14.44 | 1.8 % | 510 |

What the tables show:

- **The loop is the Monte Carlo.** With the link left out it reproduces the unmodified Monte Carlo (0.607 m against 0.607 m RMS, 160 descents; the stored 9 October figure is 0.61 m).
- **Quantisation costs nothing measurable.** Through the codec with no errors the RMS is unchanged to three decimals in both classes, for all three schemes.
- **With cumulative integrals, loss costs nothing measurable either**: 0.607 m at 5 % frame loss and 0.608 m with a 0.5 s outage on the moderate canopy, 0.387 m with the outage on the violent canopy (baseline 0.381 m).
- **Without them an outage under swing loses the attitude.** On the violent canopy a 0.5 s outage leaves the delta and redundant schemes at 41 m RMS with 50 % of descents inside 10 m, against 0.39 m and 100 % for the cumulative scheme. On the moderate canopy the same outage costs them only 0.667 m against 0.608 m, which is why the swinging case has to be the one tested.
- **Redundancy covers isolated losses but not a burst**: it equals the cumulative scheme up to 5 % random loss and equals the delta scheme in the outage.
- The cumulative message is the largest of the three (42 bytes on the wire against 26 and 38), which is the price; the budget of §9 is computed with it.

Limits of this check: 160 descents for the moderate class and 64 for the violent one, so a "100 %" means no failure in that many; the violent class was run on fewer cases (no 0.1 % loss, byte-error or boxcar rows); the loss model is as stated above. This section's table was assembled from the result files after the runs finished.

Cautions on reading them:

- The tilt figures in the outage rows include the images inside the outage, where the state scored is a constant-rate interpolation across 0.5 s of swing. That, not an error after the outage, is most of the cumulative scheme's 0.40° and 1.73°.
- The delta and redundant failure depends on choices made here: interpolation across the gap, a fixed variance inflation per interpolated interval, and no re-initialisation. A filter that discarded its attitude after a long gap and restarted from a first fix would recover. The cumulative scheme needs none of that.
- Where a descent's very first Imu snapshot was among the frames lost (7 of 160 moderate descents at 5 % loss, 4 of 64 violent), the filter coasted on the second snapshot's mean rate for that interval; this is counted as `thread_breaks` in the result files.
- One seed, one outage position, a filter started from a drawn state and not from the first-fix solver.

## 12. Verified, and not

Verified on the development laptop [sim]:

- 52 unit tests of the codec (`code/tests/test_link.py`): round trips of all 12 messages across their full ranges, saturation and missing values, rounding rule, chunking invariance, random bytes, every truncation, every single-bit error of a frame of each type, overwritten, deleted and inserted bytes with the resynchronisation bound checked on 7,500 damaged streams, bounded memory, sequence accounting, the time base through wraps, frame stamping with lost Trig messages, IMU differencing across gaps and wraps, the Nav-to-packet copy, the budget arithmetic and a throughput floor.
- 7 tests of the closed loop and of the 1 kHz integration against the descent model (`sim/nav/test_link_mc.py`).
- The golden vectors are produced by, and checked against, the Python. They pin the format; they do not show that any C matches it.

Not verified:

- Anything on the Pi Zero 2 W or the STM32F405. No firmware exists. Nothing was timed on the Pi.
- The physical layer: baud-rate accuracy, the PL011 overrun rate under camera and matching load, behaviour of the Pi's RXD with the buffer disabled, Pi boot time.
- Single-precision quaternion arithmetic on the STM32, asynchronous gyro and accelerometer sampling, sensor group delay.
- The exposure-to-reception latency of camera frames and whether the libcamera timestamp is steady enough for the quarter-period rule (it should be by a wide margin).
- The UBX-NAV-PVT field list against the u-blox document, and the items marked [recalled].
- The simulation's IMU is the Monte Carlo's 100 Hz model (white noise, bias, scale and cross-axis errors); the UART model is random frame corruption, random byte errors or a clean cut. Real overruns drop a run of bytes mid-frame, which the decoder tests cover but the Monte Carlo represents only as lost frames.
- GNSS data through the link into `landing.py`; the Monte Carlo carries Gnss frames as traffic only.

## 13. First bench tests on the hardware

1. Loop the STM32 stream (a Python script on a PC with a USB-UART adapter is enough to begin) into `link.Decoder` and compare against the vectors; then the real Pi at 230400 and 460800, PL011 on GPIO14/15.
2. With the camera running and matching at full load, log `crc`, `lost`, `stuffing` and the longest IMU gap for 10 minutes at each rate. Accept a rate if loss is under 0.1 % and no gap exceeds 30 ms.
3. Check the idle and disabled-buffer states on a scope, and that the Pi's decoder counts only empty frames.
4. Measure frame reception time against the Trig tick (an LED driven from the trigger, or simply the spread of reception time minus tick) to set the latency constant, and the trigger-to-exposure offset.
5. Pull the Pi's power and the STM32's reset mid-stream and confirm the sequences of §8.
