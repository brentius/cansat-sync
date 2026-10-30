# Flight software implementation report

28 October 2026, including the continuous prelaunch Pi-power correction; updated 29 October after a code-versus-wiki cross-check (section 8). Built on the retained Python navigation/reference implementation
and the existing C protocol/sensor cores. **Integrated candidate flight software.
Host verification and the complete target cross-build are recorded below;
hardware and flight qualification remain open.**

## 1. Delivered scope and requirements

The onboard stack has two independently scheduled processors. STM32F405 firmware
owns primary temperature/pressure telemetry, sensing, timing, recovery and an
independent NOR journal. The Raspberry Pi Zero 2 W owns images, terrain navigation,
wind/landing products and SD storage. Linux, vision and GNSS availability are not
preconditions for the STM32 primary path.

The current [design baseline](design_baseline.md), [component BOM](flight_computer_bom.md),
[connection guide](flight_computer_connections.md), [mission proposal](trn_secondary_mission_proposal.md)
and [serial protocol](stm32_pi_link_protocol.md) define the selected system. The
26 October plan/interface records and 27 October integration record under
`BOTS/PLANS/` supply implementation decisions. Historical mathematics and Monte
Carlo records retain their original assumptions and are not relabelled as flight
measurements. The Drive CANSAT 2027 reference folder was listed; Control Systems
was empty and MASTER TODO returned empty text, so no additional content was inferred.

| Requirement | Implementation | Evidence boundary |
|---|---|---|
| Independent 1 Hz primary radio telemetry | STM32 compensated MS5611 pressure and external NTC temperature, freshness/NA handling, 40-byte packet and airtime scheduler | Host/SITL; no measured RF delivery |
| Shared sensor/camera timing | TIM2 1 MHz capture/wrap handling, cumulative asynchronous IMU integrals, PPS and captured XTR messages | Register implementation and synthetic checks; scope alignment still required |
| T1 acquisition and tracking | Existing coarse search/confirmed first fix to ESKF, fine matching, delayed-image replay, loss/reacquisition and bounded lower-altitude coasting | Rendered synthetic images; real-terrain success and 5 s deadline unproved |
| T2 uncertainty evidence | Protection/ellipse outputs and retained joint horizontal position/velocity covariance, at matching reconstructed comparison epochs and during coasting | Logging/consistency inputs implemented; covariance calibration remains a measurement task |
| S1 wind and T3 landing comparisons | Separately labelled TRN, GNSS and inertial prediction branches; wind/covariance records | Aerodynamic priors, not absolute wind validation |
| Recoverable operation and records | Backup state, protected NOR ring, fast warm-reset resume, bounded queues, clean Pi shutdown, buzzer/LED recovery | Fault-injection models; actual power cuts and hardware timing unqualified |
| Operable before launch | Power Pi at system startup, finish Linux/service startup before loading, leave Pi powered through pad hold and launch; default silence cycling disabled | Supplied 2025-26 rules 5 and Rocket Launches; user explicitly requires prebooted Linux; physical endurance/readiness unqualified |

Ground-station UI, real-orthoimage preparation, below-80 m visual odometry and
optional T4 hazard/safe-target selection are outside this selected onboard
baseline. S2-S4 remain conditional post-flight scientific analyses. There are no
steering actuators or invented sensor alternatives. The software pin map follows
the saved combined control-board wiring; the POWER/MAIN CAD/interconnect split
must still be reconciled with the assembled article.

## 2. Source structure

```text
code/
  firmware/stm32/
    core/             Pure bounded C protocol, maths, sensor and policy modules
    app/              Flight scheduler, health/packet assembly, configuration, backup state
    board/board.h     Shared board contract
    board/stm32/      Actual STM32 register/peripheral implementation
    board/sitl/       Timed host implementation of the same contract
    drivers/          BMI088, MS5611, ADS1115, SX1262/E22 and W25Q128 sequencing
    hal/              GPIO/timers, interrupt I2C, DMA UART and bounded helpers
    boot/             Clock/startup/vector/linker/runtime support
    tests/, sitl/     C unit/integration tests and reproducible synthetic flights
    tools/            Host runner, reference-vector generators, NOR decoder
    build.py          Full Cortex-M4F ELF/binary build and memory inspection
  cansat/
    link.py           Reference wire codec, TickClock, IMU differencer, frame stamping
    coarse.py, firstfix.py, fine.py, eskf.py   Retained navigation components
    navigator.py      Live orchestration, rewind/replay, quality and comparison branches
    sitemap.py        Validated prepared map/camera/datum/wind bundle
    rectify.py        Optional calibrated lens correction into pinhole geometry
    windprofile.py, landing.py, geo.py        Science/prediction/coordinates
  piapp/              UART/control service, navigator worker, camera, storage, health, CLI
  deploy/pi/          Candidate systemd, UART/boot/camera configuration and provisioning notes
  tools/verify_flight.py   Single-command combined verification
```

The firmware core has no heap allocation or device access. It ports the existing
Python link, telemetry, flight-phase and radio-policy behavior, checked against
byte-level golden vectors and numerical/behavioral oracles. Wire-coded C members
are integers even when their names contain SI suffixes; app conversion helpers
apply the documented scales. Serial uses `floor(x/step + 0.5)`; radio helper
rounding retains the Python reference's ties-to-even convention.

`board.h` keeps application policy separate from target registers. The same
`app.c` executes on the Cortex-M4F target and the timed host board. SITL does not
replace the target implementation and cannot establish peripheral behavior.

All 56 board APIs have target implementations. The reproducible Windows build
uses the existing Python environment and pinned Zig 0.16.0 compiler/runtime math;
the upstream math routines and license notices are retained, with explicit ABI
glue for the ARM build. It links without libc or syscall success stubs. The optional
GNU Arm toolchain route is documented but was not exercised. Build metadata retains
source/runtime hashes, vector addresses, section sizes and image hashes.

## 3. STM32 execution and component interfaces

Boot sets the Pi off, UART buffer disabled, trigger low, excitation off and safe
radio controls. Startup configures the 8 MHz HSE/168 MHz clock tree, vector table,
FPU/runtime, TIM2, interrupts, DMA UARTs and watchdog. Candidate policy lives in
`app/config.h`; pin/peripheral implementation lives under `board/stm32` and `hal`.

The initial Pi-off level is a safe hardware-startup transient. Once supply is
good, the manager powers the Pi and allows 100 ms electrical settling before
enabling UART. This happens at system switch-on, **before launch**, and is not a
Linux boot deadline. Linux, the flight service and camera/map readiness must be
confirmed before sealing/loading. The Pi remains powered through the pad wait
and launch. The default does not power-cycle for missing UART traffic: silence
invalidates secondary health/Nav while primary telemetry continues. An explicit
nonzero silence-cycle configuration is an opt-in bench recovery policy, not the
selected flight default. Low-voltage safety and commanded post-flight shutdown
remain separate. Camera triggers/expensive image work may wait for PREPARE/ascent;
`capture_on_pad=false` does not power off or suspend Linux.

This corrects the former 10-second default, which could repeatedly interrupt a
normal Linux boot. The supplied [2025-26 guidelines](https://drive.google.com/file/d/1LubKeC729i9NbmgHwG-wzu4gwhDO1PfJ/view),
page 9 rule 5, require three continuous hours switched on; page 13 describes
5–10 minutes enclosed launch preparation during which CanSats must remain operable.
The specific requirement for prebooted Linux is also the user's selected policy.
The supplied document is not a verified current-season rulebook.

Continuity here means normal pad/flight operation, including UART outages.
Cold and warm MCU startup still command Pi off; MCU reset pin defaults and
power-enable circuitry can also interrupt its supply. Restored MCU mission state
does not preserve the Linux session. After any MCU reset, repeat Linux/service
readiness checks before loading; uninterrupted Pi power across MCU resets needs
a separately validated hardware retention design. The silence-policy correction
does not fix or claim that fault-tolerance property.

Each cooperative pass services board state machines, bounded logger work, ordered
IMU samples, pressure, temperature, GNSS, Pi control, status/radio deadlines,
backup state and recovery outputs. Slow conversion, NOR erase/program and RF
transmit operations run through staged state machines. The loop does not wait
for Linux to boot, a GNSS fix or a camera frame.

| Component | Implemented path |
|---|---|
| BMI088 / SPI1 | Independent gyro/accel initialization and readback; data-ready capture with raw bytes acquired at the interrupt boundary, bounded queues and overcapture/drop accounting; configured axes map into body coordinates |
| MS5611 / I2C1 | PROM/CRC, interleaved pressure/die-temperature conversions, integer first/second-order compensation; averaged pressure at 10 Hz with mean conversion midpoint; pressure/acceleration flight-phase detection |
| ADS1115 + OPA333 + external NTC / I2C2 | Paired single-shot AIN0/AIN1 readout with per-channel PGA scaling, ratiometric resistance, nominal Steinhart-Hart coefficients, fault/NA detection and hardware-limited excitation |
| MAX-M10S / USART3 | UBX streaming/checksum, VALSET, ACK/baud/settling/retry policy, PVT and PPS handling; usable PAD origin freezes at launch |
| E22/SX1262 + W25Q128 / SPI2 | Per-device chip select/rate, bounded radio command/TX stages and NOR busy/read/program/erase handling; independent radio airtime and journal policies |
| Power/recovery | PGOOD and Pi enable/OE ordering, battery ADC and MCU temperature housekeeping, watchdog/reset cause, backup state, trigger timer, buzzer and LEDs |

The app forms a packet once per second in every phase. PAD and LANDED have also
used 1 s since the 29 October cross-check (previously 10 s, contrary to the
[power budget](mass_power_budget.md)), so primary 1 Hz telemetry no longer
depends on launch detection. The 10% duty limiter still holds back the 240 s
flight reserve on the pad: 82.2 ms of airtime per second is 8.2% of the window.
A primary value that has failed or aged out becomes explicit NA; a just-reset
converter may require the first new-boot packet to carry NA. GNSS and Pi fields
have their own validity and age limits. The MCU never presents missing vision
as valid TRN. The packet's battery byte (2.50–5.04 V) carries the mean cell
voltage of the 2S pack; before the cross-check it carried the whole-pack
voltage and always saturated at 5.04 V. Status carries whole-pack millivolts.
On PAD or LANDED, a battery below 6.5 V for 5 s requests the commanded Pi
shutdown. The Pi then stays off. This [assumed] protection threshold needs
measurement against the pack and rail under Pi load.

## 4. Pi execution and navigation structure

The service thread exclusively owns the UART, framing/sequence state, TickClock,
FrameStamper, command acknowledgements and periodic control messages. PiStatus
is scheduled at 2 Hz, Nav at 5 Hz, and TrigCfg on change plus refresh. Camera,
health sampling, SD storage and the single navigator worker run separately.
Only that navigator worker touches the stateful Navigator.

Sensor callbacks preserve arrival order and their original observation epochs.
A separate nondecreasing publication watermark prevents delayed GNSS/pressure
observations from making a healthy current Nav cache appear to run backwards.
Both cache publication age and actual solution age are checked before UART output.
A bounded FIFO holds sensor work; a newest-wins image slot drops stale image work.
Overflow invalidates the solution and restarts the inertial thread, with counters.

The navigation chain is:

1. Validate map/camera/datum products; obtain pressure height and IMU levelling.
2. Rectify an image if calibrated Brown-Conrady coefficients are supplied; run
   the existing unknown-heading acquisition, pose solver and two-frame confirmation.
3. Initialize the ESKF and its covariance at the confirmed exposure epoch. Replay
   retained split IMU increments and pressure updates to the newest sensor time.
4. Match local map landmarks, update/replay the filter, and report quality,
   protection, velocity and landing prediction. Reject unusable frames; coast,
   declare loss, and reacquire rather than manufacture a fix.
5. Below the configured 80 m map floor, use bounded IMU/barometer coasting with
   fresh pressure, IMU quality, protection and age gates. This is not visual odometry.

Histories are bounded by time and item count. Default assumptions include eight
seconds/1500 events per sensor history, a 0.5 s maximum bridged IMU interval,
0.6 s coasting threshold, three seconds to visual loss above the map floor,
12 seconds maximum low-altitude coasting and a 30 m protection ceiling. Further
defaults are a 450 m upper image height, 1 s maximum barometer age, at least four
accepted landmarks, 1 s maximum trigger age and 8 s maximum frame age. Capture
continues for 5 s after LANDED. These are candidate settings, not calibrated
reliability guarantees. Known terrain aliases, map season/appearance changes and
capture-range failures remain algorithm limits.

IMU saturation, fault and restart flags break the inertial thread and drop TRN.
Missed-sample flags do not break it on an increment of up to 30 ms (the protocol
§11 bridging bound). The STM32 integrates across a missing sample, so a late
data-ready costs only time resolution (protocol §6). The flags carry no gap
length, so a longer flagged increment still breaks the thread, because it may
hide a stall bridged by one trapezoid. Before the 29 October cross-check, every
missed-sample flag dropped TRN. Radio Nav TRN and
prediction fields are expressed in the STM32's GNSS pad-origin frame. Without a
valid Status origin, Nav carries no TRN, while SD logs keep the site-frame
solution. Delayed baro, image and GNSS replays resume from up to 12 saved
post-pressure-update filter states. These states are tied to the current anchor
and are bit-identical to replaying from the anchor. Coasting with an 8 s-old
anchor is no longer an O(history) replay at 10 Hz.

GNSS position/velocity never initializes or corrects TRN. GNSS prediction and
comparison records are separate. The inertial-only comparison shares the first
confirmed TRN initial conditions, then uses IMU propagation without later image,
pressure or GNSS updates; its separate predictor consumes that trajectory's
velocity and uncertainty. It does not feed back into either navigation filter.
The selected radio Nav carries the TRN branch; comparison predictors remain in logs.

## 5. Timing, frames and compatible reset handshake

All navigation uses unwrapped STM32 microseconds. UART arrival time does not
replace sensor capture time. Quaternion/specific-force cumulative integrals let
consecutive snapshots span missing packets without losing the integrated motion,
within the configured gap/quality contract. Asynchronous snapshots wait for real
accelerometer coverage at a retained gyro tick.

TIM2 captures gyro/accel data-ready, PPS and trigger edges; TIM3 creates XTR pulses.
Trig reports the actual period/pulse and marks only the first trigger after a
start. The app separately retains the accepted requested period/pulse, so an
identical refresh preserves timer phase even when hardware rounded those values.
Changing a run discards/counts pending old captures before resetting frame/FIRST.
The Pi associates camera callback time with a recent captured trigger,
then uses the protocol's exposure-centre correction. The 20 ms callback latency
and external-trigger/camera delay must be measured on the actual camera.

Frames saved by the real adapter are unrectified full-resolution **8-bit YUV420
luminance**, not native Bayer RAW10. Their pixel arrays are preserved unchanged
on SD; distortion correction happens in the navigation worker. The pinhole
intrinsics and five-coefficient model must describe the actual ISP crop/output
resolution. Zero coefficients mean already-rectified or synthetic input, not a
claim that the selected wide-angle lens has no distortion. The
[BOM](flight_computer_bom.md) expects strong barrel distortion and recommends a
fisheye calibration. Only Brown-Conrady is implemented, so the calibration must
show acceptable residuals across the used field, or a fisheye model must be added.

The candidate trigger low pulse is 486 µs: about 500 µs of exposure with the
14.26 µs sensor offset, as in the BOM timing section. The service and example
configuration defaulted to 3000 µs (about 6 times the blur) until the
29 October cross-check. Launch-day light and gain still need checking.

Wire layouts remain protocol 1, with COBS/CRC and the existing 32-byte Nav versus
40-byte radio distinction. A compatible semantic extension uses Hello.flags bit
0 to advertise an epoch gate. PiStatus.flags bit 6 indicates an echo of the MCU
epoch in bits 15:8. Firmware accepts Nav only after the current-epoch heartbeat
and clears the gate on Pi/UART loss. This prevents an old partially transmitted
Nav from becoming valid after timer restart: FIFO UART order places it before
the fresh heartbeat. Old Pi software can still support primary/heartbeat operation
but must be upgraded for Nav. The eight-bit epoch is not authentication.

Initial/changed epochs or a new IMU_RESTART indication clear time-dependent state.
A same-epoch Hello can indicate Pi reappearance and preserves healthy navigation.
Queues and generations prevent stale worker publications from surviving a reset.

## 6. Persistence, loss handling and storage budget

The NOR journal packs CRC-protected records into CRC32-protected 256-byte pages,
with sequence and boot epoch. Envelopes use monotonic ingestion time within each
boot; sensor capture timestamps remain in payloads. Before launch it is a rolling
ring. At launch it protects at least the requested 60 seconds of prelaunch data,
rounded to whole erase sectors, plus the descent and recovery records. A full
protected device stops/counts new records instead of overwriting retained data.

Cold mount scans incrementally; sensing, primary radio and watchdog continue.
A bounded 256-sector/epoch RAM index makes normal launch retention read-free.
If required history is unindexed, the logger conservatively preserves remaining
data and may stop earlier. Valid protected warm-reset checkpoints verify their
predecessor and consecutive pages in at most 128 one-page steps; app buffering
is under 4 KiB. Invalid checkpoints fall back to a full conservative scan.
Already staged RAM records can be lost at the reset instant; NOR power-loss
atomicity is not inferred from the model.

Backup state is a CRC-protected explicit little-endian record committed with magic
last. It retains phase/times, pressure reference, origin, sequences, log protection
and checkpoint, shutdown intent and conservative radio airtime. Invalid warm
backup fails closed for log reuse and airtime. Radio airtime is stored as an
8-bucket age histogram (450 s buckets, whole milliseconds rounded up), and each
bucket is restored at its newest possible time. Airtime never expires earlier
than in the true history. Before the 29 October cross-check, the whole hour's
use was re-reserved for another full hour; with 1 Hz ground radio that could
silence pad or landed packets for about 50 minutes after a warm reset. The
flight reserve was protected either way. Reset downtime is not measured;
mission elapsed time excludes it. A true power loss has no retained absolute
radio-airtime clock. Retrieve the prior mission's NOR data before starting a new
powered mission that permits rolling reuse.

In the retained nominal synthetic budget, 3760 seconds includes a 120-second PAD
wait, ascent/descent and over an hour after touchdown. With 1 Hz ground radio
(3760 packets, none refused by the duty limiter) it occupies **3,183,872 B**
(3,130,624 B with the former 10 s ground interval), including page headers/padding
and all raw temperature records; zero core/app log or UART congestion drops were
reported. The reduced landed rate is about 539 B/s (was 527 B/s);
active logging is about 8.3 kB/s. A conservative calculated 60 s prelaunch +195 s
flight +1 h landed budget is about 4.1 MB against 16 MiB. This is calculated/synthetic
storage evidence, not a physical endurance measurement.

Cold scanning omits initial records with explicit counters (about 7.8 simulated
seconds and 1101 records under the assumed read timing; 1094 before 1 Hz ground radio). It cannot provide 60 s
of history before that history exists. The normal synthetic flight waits 120 s.
The tested warm in-flight reset resumes after two reads with zero unavailable
records. Exceptional corrupt-checkpoint recovery can omit records and counts them.

Pi SD writing is bounded/asynchronous. Sessions use unique directories; frames
use exclusive temporary files, sync and rename, then index entries. Raw UART,
decoded messages, frames, events, navigation states/covariance, winds and predictor
outputs are retained where storage keeps up. The writer bounds pending frames
(32, `writer_capacity`) and small records/raw UART chunks (2048) separately in
one ordered FIFO. A 1.6 MB frame write and fsync therefore cannot evict the
hundreds of log records per second that arrive meanwhile; before 29 October all
of them shared 32 slots. It drops/counts new work under overload and reports IO
faults. Gaps are unavailable evidence; they
must not be counted as successful T1/T2 coverage. No old sessions are deleted.

SHUTDOWN accepts the command, stops capture/navigation, drains/syncs storage,
sends and drains a tracked ACK_DONE, then requests OS halt. Before any final-ACK
byte is accepted, cleanup failure or deadline expiry cancels that ACK and caches
refusal for retries. Once a clean write starts the final ACK, the service commits
to finishing the frame, draining and halting, even after its local start deadline.
An ambiguous write exception quarantines further TX and withholds local poweroff.
Outside shutdown, a serial read or write exception suspends triggers, Nav and a
healthy PiStatus for 1 s after the latest exception, with one event per
episode. Before 29 October it latched for the rest of the flight. PiStatus
under-voltage means "seen since boot" (get_throttled bit 0 or 16).
Bytes already accepted by the UART cannot be recalled; if a driver accepts a whole
frame before raising, the peer may already have received DONE. The MCU independently
enforces its command timeout before cutting Pi power. Actual halt, card power-cut
behavior and UART acceptance/drain timing require bench tests.

## 7. Important implementation choices

- **Build on the existing mathematics and algorithms.** Reuse acquisition,
  ESKF, fine matching, wind, landing and Python wire oracles; add orchestration
  and real device layers instead of creating competing message/filter models.
- **Bound resources and isolate primary operation.** Static MCU buffers, staged
  peripheral work, independent radio/logger and bounded Pi workers limit failure
  propagation. Queue exhaustion is counted and can invalidate secondary output.
- **Use 8 Hz temperature pairs.** At 860 SPS and 2300 us settling, the driver's
  reserved 6256 us per 125000 us is **5.0048% calculated duty**. The former 10 Hz
  proposal exceeds the driver's conservative 6% budget and is rejected. Hardware
  forces excitation off; aborted attempts obey the driver's actual-on/off-time
  guard. The app's completed-reading monitor permits one bounded startup pulse
  then a 6% sustained allowance. Its counter excludes aborted attempts and is not
  a whole-lifecycle physical excitation audit. Nominal coefficients need calibration.
- **Keep uncertainty/source identity.** Camera/map TRN, GNSS and inertial-only
  comparisons retain separate predictors and labels. T2 records retain the joint
  horizontal covariance and cross terms at the correct reconstructed epoch.
  Wind-model agreement does not establish absolute wind accuracy.
- **Keep candidate settings explicit.** Sensor/body mounting matrices, camera
  calibration/datum, latency, canopy/descent model, thresholds, TCXO supply,
  channel/power and scheduling need article-specific validation. No hardware
  part was silently replaced and no physical performance was invented.

## 8. Verification and review

The complete reproducible gate is:

```powershell
.venv/Scripts/python.exe code/tools/verify_flight.py
```

It builds/runs strict C tests, compiles Cortex-M4F objects, links the full target
ELF/binary, regenerates five SITL scenarios, checks their output with the actual
Python codecs/NOR decoder, runs the flight Python/service/rendered/bridge tests
and validates the example Pi configuration. Each run has unique ignored logs,
versions and source SHA256s; source changes during a gate cause failure. It does
not install dependencies, flash a device, commit or touch physical measurements.

The current combined run, after the 29 October cross-check,
**20261029T065846Z-120b7a9d, passed every gate** in 138.76 seconds of stage time.
It hashed 182 source/configuration files before and after execution; none changed.
The 28 October run 20261028T091754Z-6d479a88 had also passed, in 192.66 s. The environment was Python 3.12.10, NumPy 2.5.4, pytest 9.1.1 and
ziglang 0.16.0. Raw logs and machine-readable results are retained under
`code/firmware/stm32/build/verification/20261029T065846Z-120b7a9d/` (ignored build
artifacts, not committed project records).

| Gate | Fresh result | What it establishes |
|---|---|---|
| C host tests and Cortex-M4F object checks | **16 programs, 201 tests, 7,433,019 checks passed** at `-O2 -g0`; strict target objects passed | Protocol oracles, policy, sensors, bounded storage, app/driver and HAL models |
| Complete target link | **ELF and binary passed**, including vector, source-hash and linker memory assertions | All app, driver, board, startup and runtime objects link for the selected MCU |
| Python/integration suite | **305 tests and 14 subtests passed** in 120.65 s | Navigation, Pi service, real C wire/NOR bridge, five SITL oracles, rendered replay and runtime checks |
| Native math runtime | **25,074 comparisons** within the above Python gate | Native no-libc math against host libm over selected ranges; no ARM execution |
| Synthetic scenarios | **flight, failures, reset, wrap and 3760-second budget passed** | Same app under modeled peripherals, faults, reboot and timer wrap |
| Pi example configuration | **Passed** | Schema/policy validation; no camera, systemd or OS-halt execution |

The final image occupies **163,688 B of 1 MiB flash** and **108,408 B of 128 KiB
ordinary SRAM**, leaving **22,664 B SRAM**. A separate **16 KiB stack is reserved
in CCM**, with no CCM objects and 49,152 B CCM remaining. DMA buffers stay in ordinary
SRAM. These are linked sizes, not measured stack/heap high-water marks or deadline
measurements. `build/target/image.json`, `cansat.elf`, `cansat.bin`, `sections.txt`,
`cansat.map` and `RUNTIME-NOTICES.txt` are generated by the build. The `.map` is a
symbol listing; section/layout details are in the accompanying metadata.

Earlier review builds intermittently encountered Windows Application Control
WinError 4551; those blocked executions were not counted as passes. The final
combined run had no blocked executables. Repository-wide pytest collection still
encounters historical duplicate `test_models.py` module names; the explicit flight
selection above avoids that unrelated collection conflict and states its scope.

The C-to-Python bridge additionally feeds the complete C application's encoded
sensor stream into the real Navigator with rendered images. It demonstrates
confirmed acquisition and fine tracking across the processor seam. The separate
8-second rendered replay exercises delayed images, rejection, loss/reacquisition
and reset; GNSS-on/off runs produce identical TRN, with 0.77537465 m maximum valid
horizontal error in that single synthetic level/flat-map case. These figures do
not establish real-site accuracy, mission success probability or Pi throughput.

Implementation was split between actual Sol specialist tasks for core policy,
sensor/math cores, navigation, hardware/drivers, flight application and Pi service,
with exclusive file ownership. The parent integrated shared contracts and added
storage, rectification, application-seam and C-to-Python checks. Independent Sol
reviews reproduced the following defects before fixes:

| Review finding | Resolution and retained regression |
|---|---|
| P1: delayed GNSS callback invalidated current TRN publication | Preserve observation time; publish against a nondecreasing per-generation watermark |
| P1: normal first NTC pulse tripped the cumulative duty monitor forever | Bounded one-pulse startup credit, saturated arithmetic and actual-driver/app startup/overrun/burst tests |
| P2: inertial predictor ignored propagated velocity; T2 logs lacked velocity covariance | Use propagated velocity/uncertainty; record labelled joint 4x4 horizontal covariance and comparison epochs; retain isolation tests |
| P2: shutdown could halt without final ACK, or send cached/pending DONE after abort | Track exact ACK enqueue/write/drain; cancel unstarted DONE and terminal-cache retries; quarantine ambiguous TX; test partial writes and committed halt retries |
| P2: old trigger captures acquired new settings, rounded requests restarted on refresh, first target pulse lacked capture, UART timeout overflowed, RTC access lacked clock setup | Discard/count old captures under IRQ mask in both backends; compare accepted requests independently of rounded actual values; prime active-low PWM before arming; widen UART arithmetic; initialize/read back backup epoch without domain reset |

Detailed package reports, red/green probes and review records remain under
`BOTS/PLANS/flight-work/`. The firmware R2 report was preserved before its provider
run ended at a usage limit; the earlier Pi R3 launch was also interrupted. Failed
rounds are not counted as completed reviews. Fresh final **Pi R4 passed 59 focused
tests** and **firmware R4 passed 7 tests / 168 checks**, including the unchanged
trigger reproducer and independent cache/reset/queue checks. Both completed with
no remaining concrete finding within their reviewed scopes. The review totals
overlap permanent regressions in the combined gate; they are not added as unique
coverage. Their reports were written and reread, and production source hashes
still match the final passing gate. No commits were requested or made; unrelated
checkout edits and historical/raw results were preserved.

The 28 October prelaunch power audit reproduced a slow-boot restart loop and a
hard power cut after an already-booted Pi's PAD UART outage. The selected default
now disables both silence-cycle paths. Fresh independent correction review passed
the 24 manager/application tests / 391 checks, two long-PAD probes with 30/120 s
startup delays, and an actual Service-with-fake-devices test over three hours plus
ten minutes. It checked heartbeat continuity, disabled PAD capture and PREPARE
enabling capture. These are model/scheduling checks, not physical boot/endurance.
The separate warm MCU-reset rail-interruption finding remains a documented fault
limitation in section 3; it is not claimed resolved by the silence-policy change.

### 29 October code-versus-wiki cross-check

Three read-only audits compared the code with this wiki: firmware, Pi and
navigation, and efficiency. Every finding was re-checked against source before
any change. An independent review followed; its two should-fix items, the
missed-flag gap bound and the airtime histogram, were fixed and the gate rerun.
The details and the flagged-but-unchanged items are in
`BOTS/PLANS/2026-10-29-flight-software-wiki-crosscheck.md`.

| Discrepancy (code side) | Correction | Retained check |
|---|---|---|
| Radio battery byte carried 2S pack volts, so it always read 5.04 V | Mean cell voltage | App test: 8000 mV pack gives 4.00 V |
| PAD/LANDED radio at 10 s, against the 1 s in the power budget | 1 s; warm reset restores an airtime age histogram | SITL budget 3760/3760 packets; warm-reset ageing test |
| Missed-sample IMU flags dropped TRN | Kept for increments of 30 ms or less | Navigator keep/break test |
| 3 ms trigger pulse, against about 0.5 ms in the BOM | 486 µs / 500 µs | `--check-config` |
| Transient serial exception latched a Pi fault for the whole flight | 1 s hold | Shell recovery test |
| SD frames and records shared 32 slots | 32 frames plus 2048 records | Writer bound test |
| Under-voltage flag, trigger script/README | Sticky bit 16; bounded wait and install line | `sh -n` |

Efficiency changes:

- **STM32 IMU histories are rings.** Each gyro/accel sample previously moved 63
  nodes through the target's byte-loop `memmove`: about 10.8 MB/s, an
  estimated large share of the 168 MHz core, not measured. All five SITL
  scenarios give byte-identical UART and radio output.
- **Pi replays resume from up to 12 post-baro checkpoints.** With an 8 s-old
  anchor, a laptop baro replay fell from about 41 ms to 0.6 ms. On the
  10–20 times slower Pi Zero 2 W, the old cost would have exceeded real time
  while coasting. The reviewer's bench matched state and log hashes over 90
  randomised scenarios.
- **Smaller Python changes:** reversed history scans, cached span means,
  `Leveller.clone`, `binascii` CRC (about 25 times faster) and no float64 copy
  of integer frames in fine matching (about 12%).

The new regression tests failed against the previous code and pass now. Target
CPU use, ISR latency, SD throughput and Pi timing remain unmeasured.

## 9. Deployment and next steps

Use the [Pi provisioning guide](../code/deploy/pi/README.md), example `flight.toml`,
boot fragments and systemd unit. The CLI supports `--check-config` and bench replay
without hardware imports/poweroff. Provision the writable `/data` partition and
read-only OS, the real calibrated site bundle, NumPy, pyserial and matching OS
Picamera2/libcamera packages. Configuration validation does not open a camera,
validate a map or prove the OS unit works. Linux `systemd-analyze`/shell execution
were unavailable on the Windows development host.

The next physical work is ordered by dependency:

1. **Bring up the assembled STM32 board.** Confirm supply/clock/pin map, reset
   epoch/backup retention, watchdog, DMA/bus recovery, first trigger edge and hard
   excitation cutoff. Measure ISR/main-loop latency and stack watermark under
   maximum sensor/radio/flash traffic; compare actual buses with driver assumptions.
2. **Calibrate sensor and camera geometry.** Survey gyro/accel body axes and biases,
   the real lens/ISP distortion and camera/IMU extrinsics, map/pad height datum,
   exposure timing, pressure installation and each actual thermistor. Verify cold
   range/PGA behavior, ADC completion/settling, installed air response and actual
   excitation duty including failed attempts.
3. **Run sustained Pi and link tests.** Measure 5 Hz capture/processing, acquisition
   latency, memory, UART gaps/CRC/sequence loss, SD backlog and thermal throttling
   on the Pi Zero 2 W. Replay representative site/season imagery; quantify aliases,
   false fixes, loss, reacquisition and T2 covariance coverage with missing data marked.
4. **Exercise complete fault/recovery rehearsals.** Reset either processor during
   flight replay, interrupt NOR/SD writes, fail sensors, block UART, force Pi power
   loss and run full-duration pad waiting plus recovery. Retrieve logs and verify
   retained prelaunch/descent pages, shutdown timeout and frame association.
5. **Qualify the mission article.** Confirm receiver/module revisions, GNSS/PPS and
   TCXO/RF switch settings, organiser channel/power and measured RF duty/range;
   measure full-system power/endurance/mass, calibrate the canopy and repeat
   representative descent tests before judging T1-T3/S1. Confirm current-season rules.

A linked firmware image and passing synthetic tests are software evidence.
Hardware bring-up, calibration, target performance and representative flight tests
are separate gates before use as a flight-qualified article.
