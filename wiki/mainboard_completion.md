# MAIN completion — 4 October 2026

**Historical schematic-completion record (4 October 2026).** The connectivity and ERC results below apply to the reviewed schematic state. Later records define current system interfaces and purchasing baseline; see the [flight-computer BOM](flight_computer_bom.md), [STM32-to-Pi protocol](stm32_pi_link_protocol.md), and [design baseline](design_baseline.md).

The remaining MAIN circuits are drawn on `pcb/cansat_control/cansat_interfaces.kicad_sch`, reached from the new sheet box on the root MAIN page. Both pages belong to the same MAIN PCB. The existing root drawing, component references, footprints and electrical connections were preserved, including the user's four grounded M2 mounting holes.

| Added references | Circuit / assigned package |
|---|---|
| U10, R30–R32, C42 | SN74LVC2G125DCUR, both UART directions and power unit; VSSOP-8 DCU. Common OE pull-up disables the buffers during reset. Pi-side input has a pull-down; MCU-side RX has an idle-high pull-up. |
| J2 | Eight-contact vertical JST GH Pi harness: 1/3/5 switched 5 V, 2/4/6 GND, 7 Pi RXD, 8 Pi TXD. Keep Pi 3.3 V separate. |
| R33/R34, J3 | 1.5 kohm / 1.8 kohm camera trigger divider and four-contact JST GH harness; pins 1 XTR, 2/3 GND, 4 NC. No camera supply through this connector. |
| R26 | 33 ohm MCU SPI2 clock source resistor. Place close to PB13; its output clock serves flash and the RF cable. |
| U11, R27–R29, C40/C41 | W25Q128JVSIQ flash, 208 mil wide SOIC-8; CS/WP/HOLD defaults and 100 nF / 2.2 uF bypass. This is the guide's proposed U13. |
| BZ1, Q3, D3, R35/R36, C43/C44 | CMT-8504 sounder, AO3400A SOT-23 driver, SS14 SMA flyback, 33 ohm gate resistor and 100 kohm default-off. Added 100 nF plus 22 uF local supply decoupling; C44 is an addition to the guide's original passive allowance. |
| D4/D5, R37/R38 | Red 0603 status/fault indicators, each with 330 ohm series resistor. |
| J4 | User-selected short ribbon harness with a shrouded 2×10 header. Selected Würth 61202021621. All 20 contact numbers match the guide. |
| J5 | Optional four-contact JST GH compass interface, marked DNP. Shared I2C1 uses the existing barometer pull-ups. Daughterboard electronics are separate. |

MAIN now has **124 board components**, including J5 DNP and four mounting holes; 29 components were added. All have assigned footprints. New resistors and small capacitors use 0603; C44 uses 0805. Existing package decisions were preserved.

J4 uses the project footprint `Cansat:WR-BHD_61202021621_2x10_P2.54mm`, registered in `fp-lib-table`. It derives from the stock vertical IDC header and uses the manufacturer's recommended **1.1 mm holes**, 2.54 mm row/contact pitch and 22.86 mm first-to-last contact spacing. The shroud/courtyard covers the 33.06 × 9.1 mm housing. [Würth dimensional drawing](https://www.we-online.com/components/products/datasheet/61202021621.pdf).

The ribbon's female IDC mates, cable length and RF-side header placement remain harness/PCB assembly choices. Use a keyed 20-contact 2.54 mm mate, preserve pin 1 at both ends and continuity-check all 20 contacts. Begin SPI testing at roughly 1 MHz and check edges on the assembled harness.

Firmware must keep the UART buffers disabled while Pi power is off, enable them after its rail is stable, and hold the camera trigger low/Hi-Z when the camera is unpowered. The trigger divider provides approximately 1.8 V when driven high. Check the actual camera revision for R11 removal. [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html).

Drive the sounder at about 4 kHz and confirm the required loudness at 3.3 V: its 100 dB specification is at 5 V. [Sounder datasheet](https://www.sameskydevices.com/product/resource/cmt-8504-100-smt-tr.pdf).

Verification artifacts and a before-edit backup are in `pcb/cansat_control/review_backups/main-completion`. The exported netlist is checked against all 95 existing components and explicitly checked for SPI2, flash defaults, both UART directions/OE, camera divider, sounder polarity, RF contact mapping and supply/ground connections. Symbol pin numbers were compared with every assigned footprint's pad numbers. This establishes schematic connectivity; PCB layout and assembled performance remain to be checked.

Final ERC: **one existing error, zero warnings**. The error is the stock BMI088's output/output classification for its shared SPI MISO pins 10 and 15. Separate chip selects make sharing intentional; no new violations were excluded. Both pages were rendered and inspected.
