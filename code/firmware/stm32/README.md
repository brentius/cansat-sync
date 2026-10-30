# STM32F405 candidate flight firmware

The real application in `app/` runs against the register-level `board/stm32/`
backend or the host `board/sitl/` backend. All 56 `board/board.h` functions have
target implementations. This is build- and host-verified candidate software;
there has been **no MCU execution, electrical measurement or flight qualification**.
Primary references and assumptions are in [SOURCES.md](SOURCES.md). The package
handoff is [fw-hal-report.md](../../../BOTS/PLANS/flight-work/fw-hal-report.md).

## Build and checks

From the repository root, using the existing environment (no installation):

```powershell
.venv/Scripts/python.exe code/firmware/stm32/build.py
.venv/Scripts/python.exe code/firmware/stm32/tools/host_test.py --cflags=-O2
.venv/Scripts/python.exe -m pytest code/firmware/stm32/tests/test_hal_runtime.py -q -s
```

The complete build compiles current core, app, board, drivers, HAL and startup.
It uses C99, Cortex-M4F hard-float, `-Os`, freestanding operation, and
`-Wall -Wextra -Wpedantic -Werror -Wshadow -Wdouble-promotion` without warning
suppression. Zig **0.16.0** supplies compiler runtime/math; `boot/runtime.zig`
adds the C ABI `acos`/`ldexp` wrappers to standard-library implementations.
There are no local transcendental approximations, heap or semihosting hooks.

Outputs are under ignored `build/target/`: `cansat.elf`, flashable-format
`cansat.bin` (base `0x08000000`), `image.json`, `sections.txt`, `cansat.map`
and `RUNTIME-NOTICES.txt`. With Zig, `.map` is an **ELF symbol-address listing**,
not a linker cross-reference map (`-Map` is unsupported). `image.json` records
input hashes, image hashes, memory occupancy and actual handler addresses.
The build rejects source changes during compilation, memory overflow and an
incorrect vector table/initial stack. Building does not flash a device.

An optional installed GNU Arm toolchain can be selected using
`build.py --gcc arm-none-eabi-gcc`; it uses that toolchain's libc/libm/libgcc.
That route has not been run here. Inside this directory, `make target`,
`make gcc`, and `make test` are convenience wrappers; override `PYTHON` on
non-Windows systems. `clean` deliberately removes nothing in the shared build tree.
If Windows reports `WinError 4551`, execution was blocked by Application Control;
that is neither a failing assertion nor a passed test. Record the executable and
retry an available optimization variant rather than disabling system protection.

## Hardware execution model

Startup enables the FPU, initializes SRAM/optional zero-initialized CCM, sets
VTOR and calls the real app `main`. The 8 MHz HSE drives a 168 MHz CPU, APB1
42 MHz/APB2 84 MHz; HSE/PLL failure stays at safe outputs. TIM2 is a free-running
1 MHz 32-bit counter with interrupt-assisted extension. Stack occupies the top
16 KiB of CCM (`MSP=0x10010000`); DMA buffers remain in ordinary 128 KiB SRAM.
The RTC boot epoch uses explicit PWREN/DBP/RTCSEL/RTCEN setup without BDRST.
Backup SRAM offers the application all 4096 bytes; retention across loss of
board power is not promised. The nominal one-second IWDG depends on LSI tolerance.

Pi power is enabled at system switch-on after PGOOD and electrical settling,
before launch. Linux/service readiness is checked before loading. The selected
default `cycle_silence_ms=0` leaves power on throughout boot, PAD and flight;
three-second silence still clears secondary health/Nav. A nonzero silence-cycle
setting is an opt-in bench recovery policy. Initial/warm MCU startup still uses
safe Pi-off levels, so uninterrupted Pi power across MCU resets is not promised;
repeat preflight readiness after a reset and verify reset/rail behavior on hardware.

BMI088 setup is staged, checks IDs and configuration readback, and uses
1000 Hz/2000 deg/s gyro and 1600 Hz/24 g accel. TIM2 capture ISR immediately
reads and queues **six raw bytes plus that edge's tick**; main-loop draining
does the SI conversion. Overcaptures, late edges, bus failures and a new DRDY
during the read are dropped, never relabelled as older samples. These counters
are lower bounds when hardware has already coalesced multiple edges. SPI1 is
5.25 MHz. SPI2 switches between local flash at 21 MHz and radio at 1.3125 MHz;
each CS transaction has a 150 us timeout and resets its SPI peripheral on error.

MS5611 uses I2C1/100 kHz, OSR4096, D2 followed by nine D1 conversions, and a
10 ms conversion wait. Reported midpoint uses **8.22 ms typical / 2**, with
conversion tolerance and polling latency still uncertain. ADS1115 uses
I2C2/400 kHz, single-shot AIN0 then AIN1, with OS readiness checks. The app's
accepted candidate is **8 Hz**, 2300 us settle, 860 SPS, 2048/4096 mV full scales.
The reserved on-time is 6256 us (5.0048% of 125000 us). TIM8_CH3N/PB1 opens the
PMOS independently at the cutoff even if polling stalls; its counter starts
before output enable so register setup cannot extend the on-time. Configurations
over 6% or the 16-bit cutoff range are rejected. Delayed completion/fault drains
the pending bus transaction and conservatively extends the off interval.

I2C transactions have a 3 ms deadline. Fault handling disables the peripheral,
performs nine staged open-drain SCL clocks and STOP, and reinitializes it; stuck
SCL has a bounded 1 ms stretch wait. Every recovery step returns to the main loop.
USART1/Pi is 230400 baud; USART3/GNSS starts at 9600 and app owns receiver
configuration/baud transitions. RX uses 2048-byte circular DMA per UART, TX a
copied 512-byte normal DMA buffer, with wire-idle, timeout and error checks.
All TX-time arithmetic widens before multiplying on the 32-bit target. Pi
isolation aborts old TX/discards RX; power removal stops triggers first.

NOR checks JEDEC `EF 40 18` and protection flags without clearing protection.
On a warm reset during program/erase, it polls BUSY before identification.
Reads/programs are limited to 256 bytes; programs cannot cross a page, and
erases require 4096-byte alignment. Operations poll WIP with bounded deadlines;
SPI faults disable the device rather than pretend data was stored. SX1262 setup
is staged around BUSY/reset, calibration, TCXO, PA, modulation and IRQ checks.
TX loads at most 16 payload bytes per poll, drives TXEN, and counts only TxDone.
The fixed radio TX timeout is **5 s**; alternate long-airtime configurations
require a timeout/policy review. Radio receive is outside `board.h`.

Battery/MCU housekeeping uses ADC1 with VREFINT and factory calibration when
valid, otherwise documented typical constants. Divider tolerance, die-temperature
accuracy, PGOOD levels, recovery buzzer and LED behavior need bench validation.

## Camera trigger contract

PB0/TIM3_CH3 produces **active-low PWM2 pulses**; PA3/TIM2_CH4 captures their
falling edges through the documented loopback. UART enable establishes idle-high
before camera startup; shutdown/power-off holds low. To start, TIM3 is primed at
`CNT=ARR` in the high phase, PB0 is connected to AF while high, TIM2 flags are
cleared/capture armed, and CEN is set. The first falling edge is at the next
rollover, **one timer tick after CEN**, not a period later. At 200000/100 us,
that tick is 4 us. Physical AF/edge propagation has not been measured.

`actual_period_us` and `actual_pulse_us` report the timer-quantized period and
low width, not startup latency: divisor `ceil(period/65536)` microseconds,
prescaler `84*divisor-1`, rounded period and width counts. Unrepresentable or
zero-width settings fail with triggers stopped. Stops/reconfigurations discard
and count queued and pending CH4 captures under the IRQ mask, so old edges cannot
acquire the new settings or FIRST flag. Identical refreshes preserve phase in
the app. Parent-owned SITL/app tests must maintain this same discontinuity and
first-edge contract.

## Verification boundaries and SITL

The runtime test builds a native DLL **without libc** from Zig standard math /
compiler runtime and checks it against Python host libm over representative NTC,
atmosphere, IMU/quaternion, geodetic-angle and wire-scaling ranges, with fixed
random seed and boundaries. It does not execute ARM instructions or establish
exhaustive accuracy. Fake-device tests exercise the actual acquisition/radio/NOR
state machines; I2C tests exercise actual IRQ sequencing on host registers.
Timer and queue models check first-edge timing and old-run discard, not electrical
waveforms. Worst-case board-call/ISR latency (including compound fault paths),
stack watermark, DMA timing under load and sensor calibration remain unmeasured.

The same app's synthetic flight and failure scenarios are documented in
[sitl/README.md](sitl/README.md). Run their independent checks with:

```powershell
.venv/Scripts/python.exe -m pytest code/firmware/stm32/sitl/test_sitl.py -q
```

Before hardware use: reconcile actual assembled pins/clock and module TCXO;
scope startup/first-trigger/PMOS cutoff and SPI/I2C/UART; measure ISR and polling
latency/stack; test sensor faults and watchdog resets during NOR operations;
verify radio, probe calibration and physical IMU axes. Host tests and a linked
image do not discharge any of those checks.
