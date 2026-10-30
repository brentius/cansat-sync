# FW-HAL source register

Checked 27 October 2026. These are implementation references, not physical
measurements. Local connections are a design plan, not an as-built survey.
No vendor HAL or sensor-driver implementation was copied into this package.

## Local authority

- [Connections](../../../wiki/flight_computer_connections.md): pin/net allocation,
  8 MHz crystal, 100k/47k battery divider, active-low PMOS and camera loopback.
- [BOM](../../../wiki/flight_computer_bom.md) and
  [baseline](../../../wiki/design_baseline.md): selected F405RG/BMI088/MS5611/
  ADS1115/W25Q128JV/E22/MAX-M10S and <=6% excitation goal.
- [Interfaces](../../../BOTS/PLANS/2026-10-26-flight-software-interfaces.md),
  [board.h](board/board.h), [protocol](../../../wiki/stm32_pi_link_protocol.md)
  and [app decisions](../../../BOTS/PLANS/flight-work/fw-app-report.md) govern
  the software seam. GNSS UBX configuration/compensation/navigation policy belongs
  to core/app, not these byte-transport drivers.

## Primary references actually consulted

| Source | Used in this package | Evidence boundary |
|---|---|---|
| ST [DS8626 Rev 12, STM32F405/407](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf), Table 9, memory map, Tables 70/73 | Pin AFs including PB1 AF3 TIM8_CH3N; 1 MiB flash/128 KiB SRAM/64 KiB CCM/4 KiB backup SRAM; calibration addresses 0x1FFF7A2A/2C/2E | Device capabilities, not board wiring or calibrated measurements |
| ST [RM0090](https://www.st.com/resource/en/reference_manual/dm00031020.pdf), backup-domain-access and RCC RTCPRE/RTCSEL/RTCEN excerpts | Required RTC clock setup before BKP epoch access; retain backup domain | Relevant indexed manufacturer excerpts were read; full web PDF exceeded tool size limits and direct download timed out. No claim to a full manual audit |
| ST [stm32f405xx.h](https://github.com/STMicroelectronics/cmsis-device-f4/blob/master/Include/stm32f405xx.h) | Register layouts/offsets, addresses, IRQ numbering, DMA/timer/RCC/PWR masks | Own minimal register declarations; no incorporated vendor header |
| ST [I2C HAL source](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_i2c.c) | ACK/POS/ADDR/STOP ordering for 1/2/3-byte receives | Sequence comparison, not reuse or hardware validation |
| Bosch [BMI088 datasheet rev 1.9](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf), sections 3, 4.2/4.7/4.8, 5.3/5.5, 6 | SPI selection/dummy byte, IDs, reset delays, ODR/ranges, data/interrupt registers; gyro bandwidth read-only bit 7; per-axis shadowing | Raw register acquisition and decode; mounting/latency/analog bandwidth not measured |
| TI [ADS111x SBAS444E](https://www.ti.com/lit/ds/symlink/ads1115.pdf), sections 7.3.6, 7.4.2.1, 8.1.3 | Single-cycle conversion, approximately 25 us wake, OS-ready, MUX/PGA/SPS config words | Polling and bus budgets are engineering assumptions; no settling measurement |
| TE [MS5611-01BA03, June 2017, manufacturer datasheet mirrored by Avnet](https://my.avnet.com/wcm/connect/83d782cc-0b60-4045-a710-a8a2daf90b20/TE-Connectivity-MS5611-Series-EN-Datasheet.pdf?CACHE=NONE&CVID=oMx6Zo2&ContentCache=NONE&MOD=AJPERES), pp. 3, 7–13 | OSR4096 typical/max 8.22/9.04 ms, reset/PROM/D1/D2/ADC I2C commands | TE direct document endpoints were unavailable. Conversion midpoint is estimated, not captured completion |
| Winbond [W25Q128JV Revision F](https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf), sections 7, 8.1/8.2 | ID, BUSY/WEL, protection, standard-SPI read/program/erase/page boundaries; only status reads accepted during write/erase | Runtime timeouts are conservative candidate limits; no flash endurance/torn-write evidence |
| Semtech [official SX126x driver](https://github.com/Lora-net/sx126x_driver/blob/master/src/sx126x.c) and [header](https://github.com/Lora-net/sx126x_driver/blob/master/src/sx126x.h), plus [LoRaMac-node SX126x reference](https://github.com/Lora-net/LoRaMac-node/blob/master/src/radio/sx126x/sx126x.c) | Command framing/opcodes, calibration, PLL factor, modulation/BW, TX/IRQ/error/TCXO controls and 64 kHz timeout units | Primary manufacturer software read. Current datasheet Salesforce download did not expose its contents; do not describe it as read |
| Installed Zig 0.16.0 `lib/std/math/acos.zig`, `ldexp.zig`, `lib/compiler_rt/`, bundled musl math sources/COPYRIGHT and Zig LICENSE | Freestanding C math/compiler-runtime ABI; standard implementations and original notices | Native no-libc host comparisons only; final ARM runtime is linked but unexecuted |

## Candidate assumptions requiring hardware confirmation

- **E22 module TCXO supply is unverified.** App's 1800 mV placeholder is
  `[assumed]`, not derived from the Semtech command encoding. Module oscillator,
  RF switch arrangement, PA setting, LDO choice, antenna and frequency/power/duty
  authorization need confirmation. Private LoRa sync word 0x1424 must match ground.
- SPI rates, 150 us CS deadline, 3 ms I2C transaction timeout, recovery margins,
  12% ADS conversion margin/500 us bus allowance/300 us reserve and GPIO setup
  latency are candidate engineering settings. App-selected 8 Hz/2300 us settle
  preserves the <=6% budget; it does not prove analogue settling or self-heating.
- Crystal loading, VDD=3.3 V, actual divider values, pullups, PMOS pullup,
  loopback, Pi buffer wiring and sensor orientation must match the assembled PCB.
  IWDG's nominal one second uses typical LSI frequency. RTC is used only for
  the reset epoch; no calendar, reset downtime or retained wall-clock claim.
- Peripheral reset/retry is implemented; electrical bus faults, simultaneous
  interrupts, CPU deadlines, recovery from spontaneous sensor reset, watchdog
  behavior and NOR data integrity across real power loss remain bench work.

## Runtime notices

Package source is GPL-3.0-only. Zig standard/compiler runtime incorporates
upstream permissive math code, including musl/Sun/Arm work; the algorithms and
notices remain in the installed toolchain. `build.py` checks Zig 0.16.0 and
copies Zig's LICENSE, musl COPYRIGHT and original math copyright blocks to
`build/target/RUNTIME-NOTICES.txt`. Preserve that file with redistributed images.
The no-libc probe's detailed numerical tolerances/results are under
`build/host/runtime/results.json`; this is representative accuracy evidence,
not an exhaustive libm conformance test or target-runtime qualification.
