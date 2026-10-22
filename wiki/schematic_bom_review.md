# MAIN schematic review — 4 October 2026

**Historical review record (4 October 2026).** This confirms the saved MAIN schematic connectivity and dated ERC state; it is not a current PCB/manufacturing qualification. Its designators are the checked MAIN references and supersede the earlier proposed numbering in the connection guide. Purchasing remains governed by the [flight-computer BOM](flight_computer_bom.md); see [design baseline](design_baseline.md).

Reviewed the saved `pcb/cansat_control/cansat_control.kicad_sch` against `flight_computer_connections.md`, `flight_computer_bom.md` and `temperature_options.md`. Actual designators differ from the guide; the mapping below uses this schematic's references. The connected Drive Control Systems folder was empty. Verified exported pin connectivity and rendered the schematic after editing; this is a schematic review, not PCB routing or hardware qualification.

## Present circuits

| Component | Checked support and wiring | Result |
|---|---|---|
| U1 STM32F405 | C1-C4 100 nF VDD bypass; C5 **4.7 uF bulk, 0805**; C6 VBAT bypass; R1 10 ohm and C12/C13 VDDA filtering; separate C8/C7 2.2 uF VCAP capacitors; R3/C9 reset; R2/R4 boot defaults; Y1 and C10/C11 HSE loading | Passive review corrected C5 from the earlier guide's 2.2 uF to ST's 4.7 uF VDD-bulk requirement. The earlier review missed this. TP6 provides BOOT0 pin 60 recovery access; existing pull-down stays in place. |
| U1 interfaces | MCU pin allocation, five debug/reset pads and PB0-to-PA3 camera capture loopback | Correct for the currently drawn circuits. Renamed PI_UART_OEN to PI_UART_OE_N before drawing its buffer. |
| U2 BMI088 | FB1 and C14/C15/C16 filtered supply; R5/R6 CS defaults; R8/R9 data-ready pull-ups; R7 33 ohm clock resistor; MOSI/MISO, SPI selection and grounds | Matches guide. Shared MISO and separately controlled CS are intentional. |
| U3 MS5611 | C17/C18 decoupling; R16/R17 bus pull-ups; PS and both CSB pads high; SDO no-connect; I2C1 pins | Correct, 7-bit address 0x76. |
| U4 ADS1115 | C19/C20 decoupling; R10/R11 I2C2 pull-ups to 3.0 V; R15 ALERT/RDY pull-up; grounded ADDR/unused inputs | Correct, address 0x48. AIN0 reads TEMP_BUF; AIN1 reads TEMP_EXC. |
| U5 OPA333 and Q1/J1 | C21 bypass; unity-gain feedback; R12 330 ohm/C22 10 nF output filter; precision R13 150 kohm; R14 100 kohm gate default; probe connector | Correct. TEMP_BUF/TEMP_EXC have no incorrect analogue pull-ups. Updated probe note to the current TE GA10K3MCD1 10 kohm external thermistor. |
| U6 TPS7A2030 | IN/EN to +3V3, OUT to +3V0, GND and NC; C23 input and C24 output capacitors | Correct. Repaired UTF-8 corruption in cached stock-symbol description, eliminating the library mismatch warning. |
| TP7/TP8, F1, Q2, D1/D2 | Battery wire pads, 5 A fuse, reverse-polarity MOSFET, 8.2 V gate clamp and protected-input TVS | Added and pin connectivity checked. Q2 drain is BAT_FUSED; source is VBAT_PROT. The master switch and XT30 remain external cable parts. |
| R19/R20/C39 | 100 kohm / 47 kohm protected-battery divider and 10 nF filter | BAT_ADC connects to U1 PC0, pin 8. |
| U7 TPS62130 | L1 2.2 uH; C25-C27 input bypass; C29/C30 output bulk; R21/R22 feedback; C28 soft start; R23 PG pull-up | Added. Nominal output 5.099 V; VOS/FSW connect after L1. PGOOD_5V reaches U1 PC1, pin 9. |
| U8 TPS62162 | L2 2.2 uH; C31/C32 input bypass; C33/C34 output bulk; R24 PG pull-up | Added. Fixed 3.3 V output with FB grounded and VOS after L2. PGOOD_3V3 reaches U1 PC13, pin 2. |
| U9 TPS22965 | C35 VBIAS bypass, C36 rise-time control, C37/C38 output bulk, R25 enable pull-down | Added. VIN and VBIAS use +5V1_RAW. PI_PWR_EN reaches U1 PC2, pin 10; VOUT creates +5V1_PI. Exposed pad 9 is grounded. |

Added visible component-reference notes to detached supply banks, pull-up groups, clock filtering, MCU VBAT/VDDA/VCAP, reset/boot and test pads. The stock +3V3/+3V0 power symbols remain in use, consistent with the user's naming preference; +3V3_IMU remains the distinct filtered rail.

## Completed mainboard scope

Added a second MAIN page for Pi UART isolation/harness, camera trigger divider/harness, SPI flash and source clock termination, recovery sounder/LEDs, the selected shrouded 2×10 RF ribbon header, and the optional DNP compass interface. Details and actual reference mapping are in [MAIN completion](mainboard_completion.md). RF electronics stay in the separate radio project.

**All 124 board components have footprints**, including the four user-added grounded mounting holes and DNP J5. The 29 added components use the specified IC packages, 0603 resistors/small capacitors and 0805 C44 sounder bulk capacitor. Every selected footprint exists in the installed or project library, and its pad numbers cover the symbol's electrical pins. Prior footprint assignments were retained.

The custom TPS22965 symbol remains in `Cansat.kicad_sym`. The selected Würth 61202021621 RF header has a project footprint registered in `fp-lib-table`, with the manufacturer's recommended 1.1 mm drills. New parts carry BOM MPN fields and JLC codes where specified; the newly selected Würth header has no asserted JLC code.

## Electrical verification

Final ERC: **1 existing error and 0 warnings**. The error is the stock BMI088's output/output classification for intentional shared SPI MISO. No new violations were excluded. The unfinished-interface warnings cleared once the remaining blocks were connected.

Exported netlist comparison confirms all 95 components present before this completion retain their connections. Explicit checks cover both UART directions and common OE/defaults, camera level divider, flash CS/WP/HOLD and SPI2 clock termination, all 20 RF contacts, buzzer drain/source/gate and flyback polarity, LED series paths, supply/ground connections and all footprint pad numbers. Both schematic pages were rendered and inspected.

Reviewed package selection earlier for R1–R25/C1–C39; those decisions were retained. New R26–R38 are 0603; C40–C43 are 0603 and C44 is 0805. C5 remains the corrected 4.7 uF 0805 VDD bulk capacitor. There is no placed PCB yet establishing a need for 0402. See [the earlier package review](passive_package_review.md).

Keep local bypass parts close to their associated supply pins during PCB layout. Crystal load values, effective capacitor values under DC bias, converter stability/thermal performance, Pi startup/inrush, analogue settling/calibration and assembled probe response still require layout/bench checks; this review does not establish those performance results.
