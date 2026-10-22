# CanSat sensor BOM and component justifications

Prepared **8 October 2026**. Quantities are for **one flight set**. This is a sensor summary for the selected Pi Zero 2 W + STM32 design; the [complete flight-computer BOM](flight_computer_bom.md) is the purchasing authority for the entire set and its MPNs/dated stock snapshots. See the [design baseline](design_baseline.md) for current authority and status.

The baseline has **five sensing assemblies**: IMU, pressure sensor, external air-temperature probe, GNSS receiver and camera. Optional parts and replacements below are not additional baseline purchases. The temperature implementation remains the existing analogue prototype; this document does not switch it to TMP117.

**Stock and price dates:** the JLC figures below are copied from the project's checks on **1 or 3 October 2026**, as labelled. They were **not refreshed on 8 October** and are not reserved stock. USD figures are displayed one-unit component prices before assembly fees, VAT and delivery. Offboard equipment requires separate supplier quotes. The whole-CanSat budget is **£400**, including optics, mechanics and assembly.

## 1. Selected sensors

| Qty | Sensor / exact purchased part | Location / saved reference | Connection and intended acquisition | Checked purchase information |
|---:|---|---|---|---|
| 1 | **Bosch BMI088**, six-axis IMU, 4.5 × 3 mm LGA-16 | Middle MAIN, **U2** | Filtered 3.3 V; SPI1 with separate accelerometer/gyro chip selects and data-ready inputs. Approximately 1 kHz gyro acquisition; timestamp/resample accelerometer separately; 100 Hz navigation fusion. | JLC [C194919](https://jlcpcb.com/partdetail/BoschSensortec-BMI088/C194919): **4,457**, **$6.7920**, Extended, **3 Oct** |
| 1 | **TE MS561101BA03-50**, MS5611-01BA03 pressure sensor, 5 × 3 mm | Middle MAIN, **U3** | 3.3 V, I²C1 at **0x76**. Target approximately 50 Hz pressure with slower compensation-temperature updates; primary telemetry at ≥1 Hz. | JLC [C15639](https://jlcpcb.com/partdetail/TEConnectivity-MS561101BA0350/C15639): **1,700**, **$6.7059**, Extended, **1 Oct** |
| 1 prototype; flight use conditional | **TE GA10K3MCD1**, 10 kΩ wired micro NTC | External ventilated radiation shield; analogue probe connector **MAIN J1** | Two-wire resistance measurement through the existing ADC/buffer/excitation circuit. Characterise at about 10 Hz; transmit temperature at ≥1 Hz. | [DigiKey UK](https://www.digikey.co.uk/en/products/detail/te-connectivity-measurement-specialties/GA10K3MCD1/5277239): **£10.39 ex VAT / £12.47 inc VAT**, **6,574**, **3 Oct**, delivery extra. JLC [C3153883](https://jlcpcb.com/partdetail/TEConnectivity-GA10K3MCD1/C3153883): **4**, **$23.8646**, Extended, **3 Oct**; an external probe, not an ordinary PCB assembly item |
| 1 | **u-blox MAX-M10S-00B**, GNSS receiver | Circular top RF PCB, existing **RF U1** | Filtered 3.3 V; UART to STM32 plus captured PPS and reset. Configure/log a supported navigation rate; PPS provides the timing reference. | JLC [C4153167](https://jlcpcb.com/partdetail/UBLOX-MAX_M10S00B/C4153167): **665**, **$10.7732**, Extended, **1 Oct** |
| 1 | **Official Raspberry Pi Global Shutter Camera**, Sony IMX296 | Bottom camera module; MAIN **J3** carries trigger/GND | CSI ribbon directly to Pi; external XTR trigger from STM32 through the 1.8 V divider. Target **5 frames/s**. | Purchase as an assembled camera; no JLC bare-sensor assembly selection or current retail quote in this file |

These references were checked against the saved schematic and [schematic review](schematic_bom_review.md). RF U1 was read from `pcb/cansat_radio/cansat_radio.kicad_sch`. The old proposed designators in the [connection guide](flight_computer_connections.md) have been reconciled: MAIN U11 is flash; GNSS is RF U1 and radio is RF U2. Inventory and prices above remain the dated 1/3 October observations; they were not refreshed on 8 October.

## 2. Why each sensor is selected

### BMI088 — attitude and motion between images

The navigation system needs acceleration and angular rate between 5 Hz camera frames under the selected [Pi Zero 2 W / STM32F405 baseline](flight_computer_bom.md). BMI088 combines both measurements, supports ±24 g acceleration and is designed for vibration-prone applications. Its range covers the nominal 20 g launch with limited margin; shocks above that range will clip. It also has installed KiCad symbol/footprint support and was cheaper than the checked ICM-42688-P listing. [Bosch datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf).

This is a cost/library compromise: gyro noise is higher than the original ICM option. Validate attitude error using recorded flight-like motion. Accel and gyro are separate internal devices; capture both data-ready times or implement Bosch's synchronisation method. Calibrate bias and camera-to-IMU alignment. A six-axis IMU alone cannot establish absolute initial yaw.

### MS5611 — pressure telemetry and relative altitude

MS5611 is selected for low-noise pressure measurements used both by the primary mission and navigation filter. Its high-resolution mode is appropriate for tracking the short descent; nominal pressure noise must not be confused with guaranteed altitude accuracy. Factory coefficients, PROM CRC and second-order compensation are required. [TE product and datasheet](https://www.te.com/en/product-MS561101BA03-50.html).

Tare before launch and test pressure versus temperature. Use a protected static-pressure vent: a sealed enclosure or ram-facing hole can dominate the sensor error. Its internal temperature reading compensates the pressure sensor; it is not the external air-temperature measurement. The mission's approximately 0.5 m relative-altitude goal remains an installed-system validation target.

### GA10K3MCD1 — affordable external air-temperature prototype

This replaces the approximately $200 FP07, which is excluded from the £400 baseline. The small sensing tip and fine attached leads allow mounting away from board heat, with less construction work than extending a loose bead. The selected part is **10 kΩ**, not the original FP07 resistance. [TE probe specifications](https://www.te.com/en/product-GA10K3MCD1.html).

TE's **200 ms response specification is in liquid**. It does not establish ≤1 s air response in the CanSat shield. Calibrate the actual probe's resistance-temperature curve, test air-to-air response at representative airflow, and quantify radiation and lead-conduction errors. Report t63 and t90 separately. Flight use remains conditional on meeting the intended ≤0.1 K relative-precision and response targets. [TE response condition](https://www.te.com/en/product-CAT-NTC0065.html), [project temperature options](temperature_options.md).

### MAX-M10S — independent position/velocity truth and timing

GNSS measures an independent reference trajectory and velocity against which image-based navigation and wind retrieval can be assessed. Its time pulse also gives the STM32 a timing reference. Configure the supported airborne dynamic model and validate reception, velocity quality and launch recovery in the installation. [u-blox datasheet](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf).

Keep GNSS truth separate from the GPS-denied estimator; feeding it into that estimator would invalidate the intended independent comparison. The top board places the receiver near its upward-looking antenna, but radio transmission can still degrade reception. Test GNSS during actual radio bursts.

### IMX296 global-shutter camera — terrain-relative navigation

Global shutter avoids the row-by-row geometric distortion of rolling shutter during swing and rotation. The official module provides a documented external-trigger path, allowing camera exposure timing to be related to STM32 sensor timestamps. It has a 1456 × 1088 sensor and C/CS lens mounting. The module weighs about 34 g before the lens, so the original proposal's camera mass allowance is obsolete. [Official camera specifications](https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/).

The camera is an assembled sensing module, not a bare IMX296 to solder onto MAIN. Calibrate the actual lens and rigid camera/IMU mounting. Demonstrate 5 Hz capture and processing on the Pi Zero 2 W; selecting the camera does not establish that benchmark. Apply the manufacturer instructions for the purchased revision's XTR trigger connection. [Official camera/trigger documentation](https://www.raspberrypi.com/documentation/accessories/camera.html).

## 3. Temperature readout components — included, but not separate sensors

These are existing MAIN components needed to turn probe resistance into a calibrated temperature reading. Quantities are one each; do not buy them again on top of the complete BOM.

| Qty / saved ref | Exact component | Justification | Checked JLC purchase information |
|---|---|---|---|
| 1 / **U4** | **TI ADS1115IDGSR**, VSSOP-10 | External 16-bit ADC with programmable gain; reads the buffered divider and excitation separately for ratiometric measurement. Connect to 3.0 V I²C2, address **0x48**. [TI datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf). | [C37593](https://jlcpcb.com/partdetail/TexasInstruments-ADS1115IDGSR/C37593): **26,547**, **$1.2193**, Extended, **1 Oct** |
| 1 / **U5** | **TI OPA333AIDBVR**, SOT-23-5 | Low-offset buffer prevents ADC loading from disturbing the high-resistance divider. [TI specifications](https://www.ti.com/product/OPA333). | [C30878](https://jlcpcb.com/partdetail/TexasInstruments-OPA333AIDBVR/C30878): **3,884**, **$1.3054**, Extended, **1 Oct** |
| 1 / **U6** | **TI TPS7A2030PDBVR**, SOT-23-5 | Local 3.0 V LDO supplying the ADC, buffer and probe excitation. This is the temperature circuit's supply; both main buck converters stay in the full BOM. [TI specifications](https://www.ti.com/product/TPS7A20). | [C963429](https://jlcpcb.com/partdetail/TexasInstruments-TPS7A2030PDBVR/C963429): **5,909**, **$0.2309**, Extended, **1 Oct** |

Retain the **150 kΩ, 0.1% reference resistor** and pulsed excitation. At 25°C, a 10 kΩ probe with 3.0 V excitation gives a calculated **0.1875 V** divider output and **3.52 µW** probe dissipation while on. Convert each ADC result using its actual gain setting before taking the voltage ratio. Resolution alone does not establish temperature accuracy. Full circuit and qualification details are in [temperature options](temperature_options.md).

## 4. Required sensor support parts

These quantities cover the listed sensor/readout circuits, rather than the whole MAIN/RF boards. They are **subsets of the complete BOM**, not additional allowances. The MPNs and JLC codes carry the complete BOM's dated checks; passive unit prices were not all recorded. MAIN references/values below were checked in the saved schematic on 8 October. RF support is the specified circuit, not a claim that its schematic is complete.

| Block | Qty / exact parts and saved references where assigned | Why required |
|---|---|---|
| IMU supply | **1** Murata BLM21PG221SN1D / C85840 (**FB1**); **2** Samsung CL10B104KB8NNNC, 100 nF / C1591 (**C14/C15**); **1** CL10A225KP8NNNC, 2.2 µF / C1607 (**C16**) | Filtered local supply and separate bypassing for sensor supplies |
| IMU defaults / SPI | **4** Yageo RC0603FR-0710KL, 10 kΩ / C98220 (**R5/R6/R8/R9**); **1** RC0603FR-0733RL, 33 Ω / C108661 (**R7**) | Separate CS defaults/data-ready pull-ups and source clock damping |
| Pressure supply / bus | **1** CL10B104KB8NNNC, 100 nF / C1591 (**C17**); **1** CL10A105KB8NNNC, 1 µF / C15849 (**C18**); **2** RC0603FR-074K7L, 4.7 kΩ / C99782 (**R16/R17**) | Local bypass/bulk and I²C1 pull-ups; optional compass shares those pull-ups |
| Temperature bypass / supply | **2** CL10B104KB8NNNC, 100 nF / C1591 (**C19/C21**); **2** CL10A105KB8NNNC, 1 µF / C15849 (**C20/C23**); **1** CL10A225KP8NNNC, 2.2 µF / C1607 (**C24**) | ADC/buffer bypass and LDO input/output capacitance; check effective LDO output capacitance |
| Temperature excitation / reference | **1** AOS AO3401A / C15127 (**Q1**); **1** Yageo RT0603BRD07150KL, 150 kΩ, 0.1%, 25 ppm/K / C326734 (**R13**); **1** RC0603FR-07100KL, 100 kΩ / C14675 (**R14**) | Pulsed excitation, stable resistance reference and default-off gate pull-up |
| Temperature ADC interface / filter | **3** RC0603FR-074K7L, 4.7 kΩ / C99782 (**R10/R11/R15**); **1** RC0603FR-07330RL, 330 Ω / C105881 (**R12**); **1** CL10B103KB8NNNC, 10 nF / C1589 (**C22**) | I²C2/ready pull-ups and buffer-to-ADC isolation/filtering |
| Probe connection | **1** JST BM04B-GHS-TBT(LF)(SN) / C161692 (**J1**); **1** GHR-04V-S cable housing plus SSHL-002T-P0.2 contacts for fitted wires | Retained probe wiring; housing/contacts are separate external harness purchases |
| GNSS supply, on RF | **1** BLM21PG221SN1D / C85840; **2** CL10B104KB8NNNC, 100 nF / C1591; **1** CL21A106KAYNNNE, 10 µF / C15850 | Local receiver filtering, VCC/V_IO bypass and bulk |
| GNSS active-antenna feed, on RF | **1** RC0603FR-0710RL, 10 Ω / C109318; **1** CL10B103KB8NNNC, 10 nF / C1589; **1** Murata LQG15HS27NJ02D, 27 nH / C12669; **1** Walsin 0603N470J500CT, 47 pF C0G / C302069; **1** Hirose U.FL-R-SMT-1(80) / C88374 | Antenna bias resistor/decoupling/choke, RF DC block and coax connector; follow the u-blox reference layout |
| Camera trigger, on MAIN | **1** RC0603FR-071K5L, 1.5 kΩ / C114668 (**R33**); **1** RC0603FR-071K8L, 1.8 kΩ / C185354 (**R34**); **1** BM04B-GHS-TBT(LF)(SN) / C161692 (**J3**); **1** GHR-04V-S housing plus contacts | 3.3 V-to-approximately-1.8 V XTR divider and retained trigger/GND harness; no camera supply through J3 |

All listed resistors are 0603. Small capacitors are 0603; CL21A106KAYNNNE is 0805. The IMU/GNSS ferrites are 0805; the GNSS 27 nH choke is 0402. Neither radio support parts nor shared STM32/buck/Pi components are included in these sensor-only quantities. See the [complete RF BOM](flight_computer_bom.md#top-rf-pcb-gnss-and-radio-bom) for the radio and the inter-board harness.

## 5. Offboard items needed to use the sensors

| Qty | Exact part / assembly | Justification and selection status |
|---|---|---|
| 1 | **Taoglas AGGP.25F.07.0060A** active GNSS patch with U.FL | Upward-looking antenna for the receiver; observe its ground-plane and mounting requirements. Separate purchased assembly, already in the full BOM. Current stock/landed price not quoted here. [Manufacturer datasheet](https://www.taoglas.com/datasheets/AGGP.25F.07.0060A.pdf). |
| 1 | **Pi Zero camera adapter cable**, provisional 150 mm, 22-pin 0.5 mm to 15-pin 1.0 mm CSI | Adapts the Pi Zero camera connector to the official camera. Confirm cable length and bend clearance in the actual enclosure. [Official connection documentation](https://www.raspberrypi.com/documentation/accessories/camera.html). |
| 1 | **Edmund Optics #34-711**, 2.5 mm f/2 M12 lens | Existing provisional optical selection for wide ground coverage. Image-circle coverage, infinity focus and distortion calibration need demonstration. It is a substantial budget item; this file does not substitute a cheaper unqualified lens. [Lens specifications](https://www.edmundoptics.com/p/2.5mm-fl-high-resolution-f2-micro-video-lens/3949/?PrintPDF=true). |
| 1 each | **Edmund #53-675 C-to-M12 adapter** and **#64-102 M12 lock nut** | Adapt/retain the selected lens on the camera; verify insertion depth and focus clearance. [Accessories](https://www.edmundoptics.com/f/Micro-Video-Imaging-Lens-Accessories/13898/). |
| 1 fabricated | Ventilated white probe radiation shield and fine-lead retention | Reduce radiant heating while retaining airflow; mounted temperature response must be measured. Geometry, materials and cost depend on the enclosure. |
| 1 installed set | Rigid camera/MAIN mounts and pressure vent protection | Preserve calibrated camera/IMU alignment and provide a useful pressure port. These are shared mechanical items, not extra duplicate purchases. |

## 6. KiCad stock-library assignments

The following identifiers were checked as present in the installed KiCad 10 stock library. Set the exact purchased MPN/JLC code explicitly even where the symbol is a family alias. Do not create a bare-sensor footprint for the purchased camera module or wired probe.

| Purchased part | Stock symbol | Stock footprint / representation |
|---|---|---|
| BMI088 | `Sensor_Motion:BMI088` | `Package_LGA:Bosch_LGA-16_4.5x3mm_P0.5mm_LayoutBorder7x1y_ClockwisePinNumbering` |
| MS561101BA03-50 | `Sensor_Pressure:MS5611-01BA` | `Package_LGA:LGA-8_3x5mm_P1.25mm` |
| GA10K3MCD1 | `Device:Thermistor_NTC` | External probe through MAIN J1; no on-MAIN probe footprint |
| MAX-M10S-00B | `RF_GPS:MAX-M10S` | `RF_GPS:ublox_MAX` |
| Official Global Shutter Camera | `Connector_Generic:Conn_01x04` for trigger harness | Camera is a separate assembly; J3 uses `Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical` |
| ADS1115IDGSR | `Analog_ADC:ADS1115IDGS` | `Package_SO:TSSOP-10_3x3mm_P0.5mm` |
| OPA333AIDBVR | `Amplifier_Operational:OPA333xxDBV` | `Package_TO_SOT_SMD:SOT-23-5` |
| TPS7A2030PDBVR | `Regulator_Linear:TPS7A20xxxDBV` | `Package_TO_SOT_SMD:SOT-23-5`; explicitly select the 3.0 V variant |
| Optional LIS3MDLTR | `Sensor_Magnetic:LIS3MDL` | `Package_LGA:LGA-12_2x2mm_P0.5mm` |
| Alternative TMP117AIDRVR | `Sensor_Temperature:TMP117xxDRV` | `Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm` |

## 7. Optional sensors and replacements — do not populate by default

| Baseline quantity | Part | Justification / decision | Dated JLC information |
|---:|---|---|---|
| 0 | **ST LIS3MDLTR** magnetometer | Optional initial-yaw aid. A calibrated compass can resolve the yaw ambiguity that the six-axis IMU cannot. Place on a remote daughterboard away from cells, buck inductors and buzzer; connect through optional **MAIN J5**, currently DNP. Add two 100 nF capacitors and one 1 µF capacitor, a mating harness and carrier. It shares I²C1 pull-ups. Calibration and installed heading performance remain required. [ST datasheet](https://www.st.com/resource/en/datasheet/lis3mdl.pdf). | [C478483](https://jlcpcb.com/partdetail/STMicroelectronics-LIS3MDLTR/C478483): **7,673**, **$3.7407**, **3 Oct** |
| 0 | **ADI ADXL375BCCZ** high-g accelerometer | Optional ±200 g launch/impact recorder if measurements above BMI088's ±24 g range are needed. Adds cost and another SPI sensor; not necessary for nominal 20 g measurement. No exact installed stock symbol was found in the original review; requires a custom symbol and verified footprint/support circuit. [ADI specifications](https://www.analog.com/en/products/adxl375.html). | [C579466](https://jlcpcb.com/partdetail/AnalogDevices-ADXL375BCCZ/C579466): **191**, **$22.0179**, **1 Oct** |
| 0 | **TI TMP117AIDRVR** digital air-temperature implementation | A replacement for the thermistor/readout chain, not an additional baseline sensor. Cheap I²C sensor, specified ±0.1°C from −20 to +50°C under the stated test conditions. Use an external thermally isolated carrier; installed air response is still unqualified. Requires a different harness pinout and removal of temperature-only analogue circuitry. [TI specifications](https://www.ti.com/product/TMP117), [redesign details](temperature_options.md). | [C699536](https://jlcpcb.com/partdetail/TexasInstruments-TMP117AIDRVR/C699536): **36,347**, **$1.0177**, Extended, **3 Oct** |
| 0 | **TDK ICM-42688-P** alternative IMU | Replace BMI088 only if its attitude performance is insufficient. Lower gyro noise, but higher checked price, ±16 g clipping and custom symbol/footprint work; not pin-compatible. [TDK datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf). | [C1850418](https://jlcpcb.com/partdetail/TDKInvenSense-ICM_42688P/C1850418): **4,938**, **$19.4364**, Extended, **1 Oct** |

BMP388/BMP390 and other thermistors remain alternatives in the [full BOM](flight_computer_bom.md#alternatives-considered-and-optional-parts) and [temperature comparison](temperature_options.md). They are not extra flight sensors. No separate humidity sensor or anemometer is selected for the flight hardware: horizontal wind is inferred from navigation, with GNSS as the independent comparison.

The mission proposal also calls for a **ground anemometer at a known height**. Its exact instrument, logger, mounting and cost are **not yet selected**. It is ground reference equipment and is not silently included in this flight-sensor subtotal. Likewise, calibration reference instruments are ground equipment, not flight BOM sensors.

## 8. Sensor-related housekeeping and cost

The saved MAIN also measures protected battery voltage using the **STM32's internal ADC**, **R19 100 kΩ + R20 47 kΩ**, and **C39 10 nF**. There is no additional battery-sensor IC to purchase. Converter power-good signals are status inputs, not independent sensors. IMU/barometer die temperatures can be logged diagnostically but do not replace the external air probe.

At the recorded snapshots, the **three PCB sensing devices** (BMI088, MS5611 and MAX-M10S) total **$24.2711 ≈ $24.27**. The **three temperature readout ICs** add **$2.7556 ≈ $2.76**, giving **$27.0267 ≈ $27.03** before passives and assembly. Add the externally sourced probe at the recorded **£12.47 including VAT**, camera, antenna, optics, harness, mechanical parts and delivery. Do not mix currencies into an apparent all-in total or add this sensor subtotal again to the complete BOM.

The remaining sensor-specific checks are calibrated temperature/air response, pressure-port error, attitude/initial-yaw performance, camera timing/focus/5 Hz processing, and GNSS quality during transmission. The component choices and stock-library assignments do not establish those measured results.
