# MAIN placement — 4 October 2026

**Historical placement snapshot (4 October 2026).** These coordinates and zero-routing statements describe that saved review state only. The later 11 October inspection recorded discrepancies between this note and the then-saved PCB; consult [the current mass/power report's discrepancy register](mass_power_budget.md) and inspect the current board before using dimensions or coordinates for manufacture. The purchasing authority is the [flight-computer BOM](flight_computer_bom.md); see [design baseline](design_baseline.md).

Updated `pcb/cansat_control/cansat_control.kicad_pcb`. This is component placement only: **zero track segments and zero routed vias**. All 124 original components remain on the front side, with their original footprint assignments, pad geometry, component IDs and net assignments.

## Mechanical arrangement

- Expanded the original 40 × 60 mm board to the authorised **44 × 65 mm** to accommodate the shrouded RF header, harness access, mounting clearances and local circuit groups on one assembly side.
- Outline: X = 78–122 mm, Y = 67.5–132.5 mm. Geometric centre: (100, 100) mm.
- BMI088 U2: **(100, 95) mm**, exactly **5 mm toward the top** from the centre and centred left-to-right, following the user's final correction. Orientation is 270° (KiCad may display −90°); use the final assembly orientation when defining sensor-to-camera/body axes.
- H1/H2/H3/H4: (81, 70.5), (119, 70.5), (81, 129.5), (119, 129.5) mm. Hole centres retain the original **3 mm edge inset**, now symmetric on the enlarged board. The mounting pattern is 38 × 59 mm; update any mating mechanical model to these coordinates.
- Every footprint courtyard is inside the outline, and every copper-pad bounding box has at least 0.5 mm edge clearance. There are no overlapping front-side courtyards.

## Placement intent

The RF ribbon connector spans the top region. Camera and optional compass connectors and the debug pads are accessible above it. The analogue temperature circuit and flash occupy the upper-left region, with the pressure sensor above the IMU. The MCU sits below the IMU, with the crystal to its right and VCAP/decoupling capacitors at their associated pin banks. Clock source resistors remain close to their MCU source pins.

The 3.3 V buck is at the upper right; the 5.1 V buck, input protection and Pi load switch occupy the lower region. Both converters have adjacent inductors and input/output capacitor groups. The Pi harness is on the right and the recovery sounder is on the left. These placements reserve short local power/clock connections; routing and return-current paths still need to be designed.

Small component references are available on F.Fab. Major devices and connectors have visible silkscreen references. No package substitutions or schematic edits were made.

## Keepouts and planes

Added 13 named rule areas:

- Four 0.5 mm board-edge strips prohibit copper pads, tracks, vias and zone fills.
- Four 2.4 mm-radius mounting-hardware regions prohibit tracks and vias, while allowing the original grounded mounting pads and ground-plane connections. Footprint courtyards reserve the screw-head space.
- Three IMU areas reserve the underside against components, prevent signal tracks on the underside/inner layers, and prevent vias beneath the body. Copper planes remain allowed.
- Two oscillator areas prohibit underpassing signal tracks and vias in the local clock region. Front-side oscillator connections and ground planes remain allowed.

The two pre-existing inner copper-zone definitions retain their original nets/layers and have been trimmed inside the new outline. Their old fills were cleared; no copper was filled or routed. The barometer's pressure-port access is marked on the drawing layer and needs to remain unobstructed in the mechanical assembly.

## Verification and remaining work

KiCad 10.0.6 checks and independent saved-board checks confirm:

| Check | Result |
|---|---|
| Component/pad/net preservation | All 124 components match the before-edit board |
| Schematic parity | 0 issues |
| Courtyard overlaps / outside courtyards | 0 / 0 |
| Copper-edge, keepout and silkscreen violations | 0 |
| Track segments / routed vias | 0 / 0 |
| Unconnected items | 315, expected because routing was deliberately omitted |

Fourteen existing footprint-versus-project-rule errors remain: six adjacent-pad clearance reports inside U10 (0.15 mm footprint spacing versus the 0.20 mm net-class clearance), and eight thermal-hole reports in U7/U8/U9 (0.20 mm footprint holes versus the project's 0.30 mm minimum). These are not component-placement collisions. Neither footprint geometry nor manufacturing constraints were changed or waived. Resolve those choices before manufacturing, then route and validate the finished PCB.

The before-edit PCB, final DRC report, verification summary and native assembly preview are under `pcb/cansat_control/review_backups/placement/`. `before.kicad_pcb` is the untouched starting board; `verification.json` records the final checks; `final-drc.json` is the native KiCad report.

Placement references consulted: [TI TPS62130 layout guidance](https://www.ti.com/lit/ds/symlink/tps62130a.pdf), [Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf), and the local schematic/BOM review documents. The connected CANSAT 2027 → Control Systems Drive folder was empty. No Drive files were changed.
