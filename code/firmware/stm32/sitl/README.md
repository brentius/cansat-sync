# Candidate application SITL

All results are synthetic host evidence. This is not hardware or flight qualification.
Run from the repository root with the existing environment; no dependencies are installed:

```powershell
.venv/Scripts/python.exe code/firmware/stm32/sitl/build.py --run flight
.venv/Scripts/python.exe code/firmware/stm32/sitl/build.py --run failures
.venv/Scripts/python.exe code/firmware/stm32/sitl/build.py --run reset
.venv/Scripts/python.exe code/firmware/stm32/sitl/build.py --run wrap
.venv/Scripts/python.exe code/firmware/stm32/sitl/build.py --run budget
.venv/Scripts/python.exe code/firmware/stm32/sitl/build.py --target-check
.venv/Scripts/python.exe code/firmware/stm32/tools/host_test.py app_ --cflags=-O2
.venv/Scripts/python.exe -m pytest code/firmware/stm32/sitl/test_sitl.py -q
```

Each scenario writes `build/sitl/NAME.link.bin` (STM32-to-Pi COBS stream),
`NAME.radio.bin` (concatenated 40-byte telemetry packets) and `NAME.flash.bin`
(16 MiB NOR image). The Python test also saves CLI reports as `NAME.txt`.
The existing `tools/flashlog_dump.py IMAGE --summary` checks/decodes images.
All outputs are in ignored `build/`; no measured/raw project data is overwritten.

The synthetic flight waits 120 seconds, ascends to 300 m over five seconds,
descends for 30 seconds, and has a synthetic impact at 155 seconds. Force is
body-z negative, for the Python ENU convention with body-to-world
`LEVEL=diag(1,-1,-1)` and a nadir camera. It is a state/transport scenario,
not a dynamically self-consistent six-degree-of-freedom rocket simulation.
Zero gyro and piecewise force stimulate the real asynchronous C integrator.
GNSS supplies fixed lat/lon and changing hMSL in actual UBX NAV-PVT frames;
fake Pi traffic uses real C link encoding. No real images/TRN are generated.

`failures` keeps Pi and GNSS silent and removes all primary sensors/IMU at
130 seconds, so the real FSM completes by its timeouts and the radio continues
with unavailable fields. `reset` performs a watchdog reset at 136 seconds,
restores the flight state/protection/baseline/origin/sequence/budget and remounts
flash incrementally. `wrap` starts TIM2 just before rollover. `budget` runs
until 3760 seconds, more than an hour after landing. The C behavior tests also
wrap a deliberately small NOR ring, protect launch data, fill it, and verify
the protected sector remains unchanged while subsequent records are counted.

The board model has 128-event bounded queues and 8N1 UART timing, 700 us page
program and 45 ms sector erase. Programs only clear bits; erases set FF and
enforce alignment. Accepted operations change the byte array at operation
start and hold BUSY until completion. It does not model analog power loss,
torn writes, flash wear, electrical failure or CPU execution latency. A boot
scan reads one page per app pass. Reads charge an assumed 21 MHz transfer plus
20 us overhead (119 us for 256 bytes); the app omits optional idle during the
scan. A full blank-device scan takes about 7.8 simulated seconds. Sensors/radio
continue and unavailable cold-start records are counted. Protected in-flight
warm resets instead use a persisted writer/sequence checkpoint and validate
the predecessor plus at most 127 forward pages; the reset scenario takes two
page reads with zero unavailable or queue-dropped records. A failed checkpoint
falls back to a conservative full scan. A 3520-byte app record queue plus the
core's two 256-byte pages stays below 4 KiB of pending records and absorbs
resume and erase busy times. Log envelope ticks are extended arrival times;
payload capture ticks preserve measurement timing. An initial pre-gyro accel
sample is rejected as unbracketed and logged with RESTART/ACCEL_MISSED while
UART is disabled. The first transmitted IMU carries RESTART; subsequent
nominal wire snapshots have no missed-sample flags. Queued UART frames are
discarded on disable, and sensors continue logging without enqueueing until
enable; intentional suppression has a separate counter from congestion drops.

Candidate policies are in `app/config.h`. NTC uses 860 SPS, 2300 us settle,
one pair per 125000 us (8 Hz): real-driver reservation 6256 us, 5.0048% duty,
below 6%. The board model reports that full reserved on-time. This replaces
the brief's infeasible 10 Hz schedule without shortening settling. Radio is
still 1 Hz in flight, 10 seconds on ground, with a 10%/3600-second limiter.
LANDED high-rate log streams reduce to 0.1 Hz; all 8 Hz temperature readings,
1 Hz status, radio transmissions, and phase/command events remain logged.

Persistence uses explicit little-endian, versioned backup bytes and CRC16;
the magic commits last. Only non-power resets restore it. Unknown reset
downtime is treated as zero, excluding it from mission elapsed time and
conservatively extending log retention. Prior airtime is collapsed into one
current reservation, retaining it for a full window after reset. Invalid warm
backup reserves a full duty window and preserves mounted flash conservatively.
The radio may therefore remain silent for up to an hour on invalid backup.
The invalid-backup indication survives repeated warm resets until power-on.

Firmware version `0x0002` advertises `HELLO_REQUIRE_EPOCH_ECHO` in Hello flags.
Nav is accepted only after a protocol-matching PiStatus has `PIFLAG_EPOCH_ECHO`
and the current MCU epoch in its upper eight flag bits. Power/OE-off and Pi
silence clear this gate before decoding returning Nav, preventing pre-reset
partial Nav from aliasing the new low ticks. This parent-coordinated semantic
extension uses existing reserved bits with unchanged message sizes/version.
Older Pi software retains primary telemetry but cannot supply a Nav overlay.

For the HAL-owned firmware README, the parent can link this document and copy
the first build/run command. No HAL-owned files were edited by this package.
