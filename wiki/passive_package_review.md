# MAIN passive package review — 4 October 2026

**Historical package-selection snapshot (4 October 2026).** Treat package assignments as the reviewed choices for the saved schematic at that date. The later completion record adds parts and confirms C5/C44 details; current purchasing quantities and MPNs are in the [flight-computer BOM](flight_computer_bom.md). See [design baseline](design_baseline.md).

Scope: all 25 resistors and 39 capacitors currently saved in `pcb/cansat_control/cansat_control.kicad_sch`. Package codes here are imperial: 0402 is 1.0 × 0.5 mm, 0603 is 1.6 × 0.8 mm and 0805 is 2.0 × 1.25 mm. At the time of this package review, the PCB file had no placed components, so this review recorded no demonstrated placement constraint requiring 0402. The separate [4 October placement snapshot](mainboard_placement.md) records the subsequent placement state; its geometry later diverged from the board inspected on 11 October.

## Package decisions

| Parts | Package decision | Reason |
|---|---|---|
| R1-R25 | Retain 0603 | Appropriate for the selected BOM parts and practical prototype rework. Pull-ups, dividers, gate defaults and feedback draw small currents; none requires a larger resistor package in normal operation. Keep the exact precision part for R13. |
| C1-C4, C6, C9, C12, C14/C15, C17, C19, C21, C27, C35 | Retain 0603, 100 nF X7R | No demonstrated need to shrink the existing supply bypass parts. During placement, put each close to its associated pin and give it a short ground return. |
| C7/C8 | Retain 0603, 2.2 µF X5R | Dedicated STM32 VCAP capacitors, operating near the internal core voltage. Do not substitute the larger VDD-bulk value onto either VCAP net. ST specifies 2.2 µF and ESR below 2 ohms. |
| C16/C24 | Retain 0603, 2.2 µF X5R, 10 V | Existing IMU bulk and LDO output selections match the selected physical parts. Effective capacitance under bias and temperature still belongs in final component qualification. |
| C13, C18, C20, C23 | Retain 0603, 1 µF X5R, 50 V | Existing low-voltage supply bypass parts; no package change indicated. |
| C10/C11 | Retain 0603, 33 pF C0G/NP0 | Keep the specified stable dielectric and BOM part. A smaller package would require a new part and another crystal-loading/layout check. |
| C22, C28, C36, C39 | Retain 0603, 10 nF X7R | Analogue filter and ramp/soft-start parts do not require 0402. |
| C25/C26, C31/C32 | Retain 0805, 10 µF X5R, 25 V | Buck-input capacitors see the protected 2S pack, up to 8.4 V. Do not shrink them without checking effective capacitance and ripple-current heating for the replacement. |
| C29/C30, C33/C34, C37/C38 | Retain 0805, 22 µF X5R, 25 V | Buck/Pi output bulk. Preserve capacitance under DC bias, converter stability and transient performance. Nominal capacitance and voltage rating alone are insufficient to approve a 0402/0603 replacement. |
| C5 | **Changed to 4.7 µF, 0805** | The earlier 2.2 µF VDD bulk selection came from the guide. ST's supply scheme specifies a separate 4.7 µF capacitor at VDD. Selected replacement: Samsung CL21A475KOFNNNE, 16 V X5R, ±10%, 0805; LCSC C1746. Updated schematic value, footprint, purchasing fields and visible MCU capacitor-bank note. |

## When to use 0402

Use 0402 when actual placement needs the smaller body or shorter high-frequency connections, and assembly/rework can support it. It is often suitable for low-power pull-ups, signal termination and small decoupling capacitors. Select a matching manufacturer part and recheck power/voltage ratings, tolerance, dielectric and effective capacitance. Changing only the footprint leaves an incompatible purchasing BOM.

0603 is the default retained here for prototype handling and the existing parts. Larger packages remain useful for bulk capacitance and power dissipation. A package-size choice does not by itself prove current rating, capacitor stability or high-frequency performance.

Illustrative resistor checks for this design: a 100 kohm gate resistor across 8.4 V dissipates 0.706 mW; a 4.7 kohm pull-up held low at 3.3 V dissipates 2.32 mW; the 330 ohm R12 dissipates at most 27.3 mW for an ideal 3.0 V-to-ground output short. These are calculated circuit cases, not measured values or a complete fault qualification.

## Sources

- [ST STM32F405/F407 datasheet](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf), supply scheme figure and VCAP operating conditions: separate 4.7 µF VDD bulk; two 2.2 µF VCAP capacitors.
- [Samsung 2.2 µF 0603 part](https://product.samsungsem.com/mlcc/CL10A225KP8NNN.do): X5R, 10 V, ±10%.
- [Samsung C5 part](https://product.samsungsem.com/mlcc/CL21A475KOFNNN.do): 4.7 µF, X5R, 16 V, ±10%, 0805. [LCSC C1746](https://www.lcsc.com/product-detail/C1746.html) confirms the purchasing-code mapping; JLC assembly availability was not independently confirmed.
- [TI TPS62130 datasheet](https://www.ti.com/lit/ds/symlink/tps62130.pdf): LC selection accounts for capacitor DC bias; its reference uses 0805 output bulk and 0603 small support parts.
- [TI TPS7A20 datasheet](https://www.ti.com/lit/ds/symlink/tps7a20.pdf): capacitor type, effective capacitance and input/output requirements.
- [Murata MLCC miniaturization guidance](https://article.murata.com/en-eu/article/middle-case-size-bridges-mlcc-miniaturization-path): smaller high-capacitance MLCCs can lose more effective capacitance under DC bias; assess the specific part.
