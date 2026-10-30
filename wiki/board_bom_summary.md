# BOM summary — all flight boards

Prepared and revised 22 October 2026 from local project files, including the user-selected POWER/MAIN split. Quantities are for **one complete flight set**, with no procurement spares. This summary consolidates the [purchasing baseline](flight_computer_bom.md) and checks it against the saved schematics; it is not a manufacturing release or an inventory of physically owned parts. No live supplier stock, prices or external sources were checked.

## Board overview

| Board / assembly | Quantity | Main contents | Local implementation evidence |
|---|---:|---|---|
| POWER custom PCB | 1 | Input protection, 5.1 V / 3.3 V bucks, quiet 3.0 V LDO, Pi load switch and battery sensing divider | **35 existing fitted components allocated**; separate schematic/layout and interconnect pending |
| MAIN custom PCB | 1 | STM32, IMU/barometer, temperature readout/excitation, flash, UART buffers, recovery and interfaces | **76 existing fitted components allocated**; separate schematic/layout and interconnect pending |
| Top RF custom PCB | 1 | MAX-M10S GNSS, E22 radio, antenna bias/filtering and local support | **18 electronic components planned**, plus RF inter-board connector TBD. Saved schematic has only U1 MAX-M10S; saved PCB has no footprints |
| Side-mounted Raspberry Pi | 1 | Pi Zero 2 W and microSD | Purchased assembled board; no local component-level design |
| Bottom camera | 1 | Official Raspberry Pi Global Shutter Camera, IMX296 | Purchased assembled board with separate optics; no local component-level design |
| Optional compass daughterboard | 0 baseline | LIS3MDLTR and local bypass | Optional only; MAIN J5 is DNP; no dedicated board design found |

The selected architecture now has **three custom boards: POWER, MAIN and RF**, plus the purchased Pi and camera boards. The two saved `cansat_control` schematic pages still describe the old combined board: 124 unique references including 111 fitted parts, J5 DNP, 8 pads and 4 holes. No separate POWER project exists yet. Review backups and historical alternatives are excluded.

## Allocation and reference policy

**Selected split, 22 October 2026:** POWER contains supply conversion, input protection, Pi power switching and the battery-voltage divider/filter. MAIN contains the MCU, sensors/readout, sensor excitation, UART buffers, logging, recovery and signal connectors. The quiet 3.0 V regulator also moves to POWER; the ADC/buffer retain their local bypass capacitors on MAIN. The thermistor excitation MOSFET Q1 stays with its sensor circuit on MAIN.

**CAD migration is pending.** The saved `cansat_control` project still combines both boards. All references below are its existing identifiers for traceability; they must not be interpreted as newly assigned references in separate PCB projects. Quantities partition the existing 111 fitted parts without duplication. New inter-board connectors, any additional bypass/bulk capacitors and mechanical hardware are **additional TBD items**, outside those counts.

## POWER — selected ICs

| Qty | Legacy references | Value / function | Selected MPN | JLC / LCSC code |
|---:|---|---|---|---|
| 1 | U6 | Quiet 3.0 V LDO | TPS7A2030PDBVR | C963429 |
| 1 | U7 | 5.1 V buck | TPS62130RGTR | C43590 |
| 1 | U8 | 3.3 V buck | TPS62162DSGR | C40256 |
| 1 | U9 | Pi power switch | TPS22965DSGR | C122837 |

## POWER — support components

Resistors are 0603 unless noted. Capacitor/inductor specifications and manufacturers remain those in the detailed purchasing BOM.

| Qty | Legacy references | Value / function | Selected MPN | JLC / LCSC code |
|---:|---|---|---|---|
| 2 | C27, C35 | 100 nF, 0603 | CL10B104KB8NNNC | C1591 |
| 1 | C24 | 2.2 uF, 0603 | CL10A225KP8NNNC | C1607 |
| 1 | C23 | 1 uF, 0603 | CL10A105KB8NNNC | C15849 |
| 3 | C28, C36, C39 | 10 nF, 0603 | CL10B103KB8NNNC | C1589 |
| 4 | C25, C26, C31, C32 | 10 uF, 0805 | CL21A106KAYNNNE | C15850 |
| 6 | C29, C30, C33, C34, C37, C38 | 22 uF, 0805 | CL21A226MAYNNNE | C602037 |
| 3 | R23, R24, R25 | 10 kohm | RC0603FR-0710KL | C98220 |
| 2 | R18, R19 | 100 kohm | RC0603FR-07100KL | C14675 |
| 1 | R20 | 47 kohm | RC0603FR-0747KL | C105579 |
| 1 | R21 | 806 kohm | RC0603FR-07806KL | C185311 |
| 1 | R22 | 150 kohm, 1% feedback | RC0603FR-07150KL | C114660 |
| 2 | L1, L2 | 2.2 uH buck inductors | MWSA0402S-2R2MT | C408335 |
| 1 | F1 | 5 A fuse, 2410 | 0451005.MRL | C48467 |
| 1 | Q2 | Reverse-battery protection, SOT-23 | AO3401A | C15127 |
| 1 | D1 | 8.2 V gate clamp, SOD-123 | BZT52C8V2 | C173432 |
| 1 | D2 | 9 V TVS, SOD-123F | SMF9.0A | C123798 |

**POWER subtotal: 35 allocated fitted components** = 4 ICs + 17 capacitors + 8 resistors + 1 MOSFET + 2 diodes + 2 inductors + 1 fuse. Battery wire pads TP7/TP8 also move to POWER and are not purchased components. The external pack, master switch and XT30 pair remain separate harness purchases.

## MAIN — selected ICs

Each row is one physical package. Exact orderable MPNs come from the purchasing BOM where CAD contains a generic/family value.

| Qty | Legacy references | Value / function | Selected MPN | JLC / LCSC code |
|---:|---|---|---|---|
| 1 | U1 | Timing / telemetry MCU | STM32F405RGT6 | C15742 |
| 1 | U2 | IMU | BMI088 | C194919 |
| 1 | U3 | Pressure sensor | MS561101BA03-50 | C15639 |
| 1 | U4 | Temperature ADC | ADS1115IDGSR | C37593 |
| 1 | U5 | Temperature buffer | OPA333AIDBVR | C30878 |
| 1 | U10 | Dual UART buffer; one package | SN74LVC2G125DCUR | C21404 |
| 1 | U11 | 16 MB SPI NOR flash | W25Q128JVSIQ | C97521 |

## MAIN — support components and connectors

| Qty | Legacy references | Value / function | Selected MPN | JLC / LCSC code |
|---:|---|---|---|---|
| 15 | C1, C2, C3, C4, C6, C9, C12, C14, C15, C17, C19, C21, C40, C42, C43 | 100 nF, 0603 | CL10B104KB8NNNC | C1591 |
| 1 | C5 | 4.7 uF VDD bulk, 0805 | CL21A475KOFNNNE | C1746; JLC stock unconfirmed |
| 4 | C7, C8, C16, C41 | 2.2 uF, 0603 | CL10A225KP8NNNC | C1607 |
| 2 | C10, C11 | 33 pF C0G, 0603 | 0603N330J500CT | C302065 |
| 3 | C13, C18, C20 | 1 uF, 0603 | CL10A105KB8NNNC | C15849 |
| 1 | C22 | 10 nF, 0603 | CL10B103KB8NNNC | C1589 |
| 1 | C44 | 22 uF, 0805 | CL21A226MAYNNNE | C602037 |
| 1 | R1 | 10 ohm | RC0603FR-0710RL | C109318 |
| 12 | R2, R3, R4, R5, R6, R8, R9, R27, R28, R29, R30, R32 | 10 kohm | RC0603FR-0710KL | C98220 |
| 3 | R7, R26, R35 | 33 ohm | RC0603FR-0733RL | C108661 |
| 5 | R10, R11, R15, R16, R17 | 4.7 kohm | RC0603FR-074K7L | C99782 |
| 3 | R12, R37, R38 | 330 ohm | RC0603FR-07330RL | C105881 |
| 1 | R13 | 150 kohm, 0.1%, 25 ppm/K reference | RT0603BRD07150KL | C326734 |
| 3 | R14, R31, R36 | 100 kohm | RC0603FR-07100KL | C14675 |
| 1 | R33 | 1.5 kohm camera divider | RC0603FR-071K5L | C114668 |
| 1 | R34 | 1.8 kohm camera divider | RC0603FR-071K8L | C185354 |
| 1 | FB1 | IMU supply ferrite, 0805 | BLM21PG221SN1D | C85840 |
| 1 | Y1 | 8 MHz two-pin 5032 crystal | X50328MSB2GI | C115962 |
| 1 | Q1 | Pulsed sensor excitation, SOT-23 | AO3401A | C15127 |
| 1 | Q3 | Sounder driver, SOT-23 | AO3400A | C20917 |
| 1 | D3 | Sounder flyback, SMA | SS14 | C2480 |
| 2 | D4, D5 | Red status LEDs, 0603 | 19-217/R6C-AL1M2VY/6T | C2986060 |
| 1 | BZ1 | 4 kHz magnetic sounder | CMT-8504-100-SMT-TR | C22359707 |
| 2 | J1, J3 | 4-pin JST GH: probe / camera trigger | BM04B-GHS-TBT(LF)(SN) | C161692 |
| 1 | J2 | 8-pin JST GH: Pi power / UART | BM08B-GHS-TBT(LF)(SN) | C133062 |
| 1 | J4 | Shrouded 2x10, 2.54 mm RF header | 61202021621 | TBD |

**MAIN subtotal: 76 allocated fitted components** = 7 ICs + 27 capacitors + 30 resistors + 2 MOSFETs + 3 diodes/LEDs + 1 ferrite + 1 crystal + 1 sounder + 4 connectors. **35 POWER + 76 MAIN = 111 original fitted components.**

| Other features | Allocation | Purchasing treatment |
|---|---|---|
| J5, optional compass connector | MAIN | 1 DNP position; 0 fitted |
| TP1–TP6, debug/reset/ground/BOOT0 pads | MAIN | 6 copper features; no purchased component |
| TP7/TP8, battery input/ground wire pads | POWER | 2 copper features; no purchased component |
| Existing H1–H4 | Mechanical allocation TBD | 4 existing grounded M2 holes in the unsplit design; both new board outlines/retention require review |

## POWER–MAIN interface to complete

| Direction | Required rail / signal | Purpose / allocation |
|---|---|---|
| POWER → MAIN | Always-on +3V3 and GND | MCU, digital sensors, flash, buffers and recovery; MAIN retains local bypass and IMU ferrite |
| POWER → MAIN | Quiet +3V0 with a low-impedance ground return | ADS1115, OPA333 and probe excitation; keep this supply/return away from switch-node and Pi-current paths; verify noise after layout |
| POWER → MAIN | Switched +5V1_PI and its current-rated GND returns | Supply the Pi/camera via the retained MAIN J2 combined power/UART harness |
| MAIN → POWER | PI_PWR_EN | STM32 controls the Pi load switch; R25 default-off stays on POWER beside U9 |
| POWER → MAIN | PGOOD_5V and PGOOD_3V3 | Converter status to MCU; R23/R24 pull-ups stay with their converters |
| POWER → MAIN | BAT_ADC | Divider R19/R20 and C39 stay on POWER; MCU ADC is on MAIN; analogue routing/settling and input protection need review |
| MAIN → RF | Existing digital interface, +3V3 and GND through J4 | POWER supplies the always-on rail via MAIN; RF modules and their local support remain on RF |

Select **one POWER–MAIN interconnect assembly**, including both board endpoints and a cable if required. MPNs, contacts, pinout, current rating, wire/trace sizing, local bulk requirements and cost remain TBD. Parallel power/ground contacts may be required. Do not assign the existing RF J4 to this new link: it retains its MAIN–RF role. MAIN keeps J2 and UART buffer U10 so its combined Pi power/UART harness needs no new signal split; review the longer 5.1 V power path for voltage drop and inrush.

The 3.0 V supply crosses boards under this allocation; its local LDO output capacitor C24 remains at U6 on POWER, while C19/C20/C21 remain at the ADC/buffer on MAIN. Check regulator stability and remote-load noise with the chosen interconnect. MAIN retains Q1/R13/R14 and its analogue readout/filter as one sensor circuit. UART OE defaults and PI_UART_OE_N remain local to MAIN. A new connector does not establish electrical or mechanical qualification.

### Combined POWER + MAIN quantities to reconcile before ordering

| Part / value | Purchasing baseline | Saved fitted count | Interpretation |
|---|---:|---:|---|
| 100 nF | 23 allowance, plus separately recorded C43 addition | 17 | Use designators above for fitted quantity; define procurement spares separately |
| 2.2 uF | 8 allowance | 5 | Includes both VCAP capacitors; C5 is separately 4.7 uF |
| 22 uF | 6 allowance + C44 addition | 7 | Sounder bulk capacitor is included once |
| 10 kohm | 16 allowance | 15 | One fewer than allowance |
| 4.7 kohm | 6 allowance | 5 | Optional compass shares existing I2C pull-ups |
| 33 ohm | 4 allowance | 3 | Reserve position is not drawn |
| 100 pF feed-forward | 1 optional DNP allowance | 0 positions | Not present in saved schematic; do not add as a fitted part |

## RF — planned complete board BOM

These quantities describe the selected design, **not the current partial RF schematic**. Only the generic MAX-M10S U1 symbol is saved; the exact -00B MPN is specified in the purchasing BOM. The E22 and all 16 local support components below are absent from that schematic. Remaining RF references are unassigned in this summary.

| Planned qty | Function / value | Selected MPN | JLC code |
|---:|---|---|---|
| 1 | GNSS receiver | MAX-M10S-00B | C4153167 |
| 1 | SX1262 SPI radio module | E22-900M22S | C411293 |
| 1 | GNSS supply ferrite, 0805 | BLM21PG221SN1D | C85840 |
| 3 | 100 nF, 0603 | CL10B104KB8NNNC | C1591 |
| 2 | 10 uF, 0805 | CL21A106KAYNNNE | C15850 |
| 1 | 22 uF, 0805 | CL21A226MAYNNNE | C602037 |
| 1 | 10 nF bias bypass, 0603 | CL10B103KB8NNNC | C1589 |
| 1 | 47 pF GNSS DC block, 0603 | 0603N470J500CT | C302069 |
| 4 | 10 kohm radio defaults, 0603 | RC0603FR-0710KL | C98220 |
| 1 | 10 ohm GNSS bias feed, 0603 | RC0603FR-0710RL | C109318 |
| 1 | 27 nH GNSS choke, 0402 | LQG15HS27NJ02D | C12669 |
| 1 | GNSS antenna U.FL | U.FL-R-SMT-1(80) | C88374 |

**Total: 18 electronic components, plus one RF-side inter-board connector with MPN/footprint TBD.** Add bare PCB fabrication, assembly and mechanical retention separately. POWER generates 3.3 V, routed via MAIN J4 with the STM32 digital interfaces; RF has no second MCU or buck converter. The E22 module includes its own antenna socket; the separate Hirose connector is for GNSS. Resolve the E22 pad-12 documentation discrepancy against the purchased module revision before fabrication.

## Purchased Pi and camera boards

| Assembly | Qty | Required associated items |
|---|---:|---|
| Raspberry Pi Zero 2 W, unheadered | 1 | SanDisk High Endurance 256 GB **SDSQQNR-256G-GN6IA** x1; MAIN J2 power/UART harness |
| Official Raspberry Pi Global Shutter Camera, IMX296 | 1 | Pi Zero CSI adapter cable, 22-pin 0.5 mm to 15-pin 1.0 mm, provisional 150 mm x1; MAIN J3 trigger/GND harness |
| Camera optics | 1 set | Edmund **#34-711** 2.5 mm f/2 lens x1, **#53-675** C-to-M12 adapter x1, **#64-102** M12 lock nut x1; optical selection remains provisional pending focus/fit/calibration |

CSI goes directly from camera to Pi. The camera trigger connector does not supply camera power. The Pi and camera are purchased as assembled modules; their internal components are not separately added to the custom-PCB BOM.

## Shared offboard purchases

| Qty per flight set | Item | Allocation / unresolved detail |
|---|---|---|
| 1 | KeepPower protected NCR18650GA 2S1P pack, **C3-302** | Finished battery pack; no duplicate loose-cell/holder purchase |
| 1 pair | Amass **XT30U-M / XT30U-F** | Battery/master-switch harness |
| 1 | E-Switch **100SP1T1B1M2QEH** | Master disconnect; external hand-fitted part |
| 1 prototype | TE **GA10K3MCD1** 10 kohm wired NTC | External probe through MAIN J1; flight use conditional on calibration and installed air response |
| 1 | Taoglas **AGGP.25F.07.0060A** active GNSS patch/U.FL | RF GNSS antenna |
| 1 | Taoglas **FXP895.07.0200C** FPC/U.FL | E22 radio antenna |
| 1 + 2 | JST **GHR-08V-S** + **GHR-04V-S** housings | Pi harness plus camera trigger/probe harnesses; optional compass excluded |
| 16 + spares allowance | JST **SSHL-002T-P0.2** contacts | Final count depends on fitted wires; camera J3 pin 4 is NC |
| 1 MAIN–RF harness | Keyed 20-contact IDC mates and ribbon cable | RF endpoint MPN, complete mating arrangement and cable length TBD; MAIN J4 is already counted above |
| 1 POWER–MAIN interconnect assembly | Both board endpoints, plus cable if used | Additional to the 111 reallocated components; full selection, rating, pinout and cost TBD |
| 1 support set | Screws, spacers/standoffs, battery cradle, wire, insulation and strain relief | Counts/lengths/materials depend on mechanical design; no final mechanical BOM |
| 1 fabricated | White ventilated temperature-probe radiation shield | Local fabrication; installed response unqualified |
| 1 flight set | Enclosure, camera mounts, parachute, swivel and suspension/hardpoint | Required whole-CanSat budget items; exact selections/quantities not frozen |

Do not count the same antenna, optics, harness or mounting hardware again under multiple board subtotals.

## Optional boards and substitutions

The optional remote compass would add **LIS3MDLTR x1**, **CL10B104KB8NNNC x2** and **CL10A105KB8NNNC x1**, plus a carrier, harness and connector arrangement to be finalised; fit MAIN J5 only if adopted. No optional compass board is included in the baseline counts above. TMP117, ADXL375, ICM-42688-P and alternate barometers are alternatives, not additional baseline purchases.

## Cost and readiness

Reallocating the recorded 1/3 October unit prices gives **POWER ICs $3.0642 + MAIN ICs $24.7348 + RF modules $17.6872 = $45.4862 (about $45.49)**. This preserves the previous combined IC/module total; the added third PCB and POWER–MAIN interconnect require new costs. This subtotal excludes support components, connectors, PCB fabrication/assembly, Pi/camera, optics, battery, antennas, mechanics, VAT and delivery. The recorded probe price is **GBP 12.468 including VAT**, delivery extra. These are historical source figures, not refreshed quotations; no complete delivered total or compliance with the whole-CanSat GBP 400 budget is established.

The immediate purchasing gaps are the new POWER–MAIN interconnect, RF connector/mates/cable, **three** PCB fabrication/assembly quotes and complete mechanical quantities. The saved combined control schematic has assigned footprints; it has not yet been migrated into separate POWER and MAIN designs. RF remains a partial schematic with an empty PCB. Some original control parts lack explicit orderable MPN/JLC fields in CAD; this summary maps them from the purchasing baseline. No source schematic, PCB outline or routed net was changed by this allocation.

## Sources and verification

- Purchasing selections and historical prices: [flight computer BOM](flight_computer_bom.md); context: [design baseline](design_baseline.md), [sensor BOM](sensor_bom.md), [MAIN completion](mainboard_completion.md), [schematic review](schematic_bom_review.md).
- MAIN implementation: [`cansat_control.kicad_sch`](../pcb/cansat_control/cansat_control.kicad_sch) and its [`cansat_interfaces.kicad_sch`](../pcb/cansat_control/cansat_interfaces.kicad_sch) child sheet; RF: [`cansat_radio.kicad_sch`](../pcb/cansat_radio/cansat_radio.kicad_sch) and [`cansat_radio.kicad_pcb`](../pcb/cansat_radio/cansat_radio.kicad_pcb).
- On 22 October 2026, KiCad CLI 10.0.5 exported both current schematic projects to XML netlists. Unique MAIN references were reconciled against every table entry, DNP status and footprint assignments; the multiple U10 units count as one package. RF export contains only U1. Independent read-only RF review confirmed the empty PCB and missing planned parts. KiCad emitted a settings-registry access message, but both exports completed successfully and parsed correctly.
- The POWER/MAIN revision partitions all 111 original fitted references exactly once (35 + 76), retaining each row's MPN/code. Support networks and required boundary signals were reviewed separately. It is a documentation allocation, not a migrated or electrically validated design.
- No physical stocktake, new ERC/DRC run, circuit validation, live price/stock check, hardware measurement or Drive update was performed. Local files were sufficient for this summary. `BOTS/PLANS/` is absent, so this summary records the findings directly.
