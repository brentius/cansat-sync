# Navigation simulation: parameter sources

Collected 8-9 October 2026 for the [closed-loop navigation simulation](navigation_closed_loop_simulation.md); study status cross-checked 13 October 2026. This is the research helper's extraction, kept verbatim so that every sourced input can be traced. Read the evidence codes before using a number: only **[P]** and **[A]** rows were read in the source itself; **[S]** rows are search excerpts, **[D]** rows are derived here, and **[U]** rows are unverified recollection that must be checked before use. This is a dated parameter-source record, not a current design specification. Current selected parts and architecture are in [design_baseline.md](design_baseline.md) and the authoritative [flight computer BOM](flight_computer_bom.md); the navigation simulation identifies which of these values it actually used and which were assumptions.

---


Compiled 2026-10-08/09. All eight questions were worked; gaps are marked NOT FOUND and collected under Caveats.

Evidence codes used in every row:
- **[P]** read directly in the primary document this session (PDF text extracted or page fetched)
- **[A]** read directly on the publisher/repository abstract or record page
- **[S]** search-engine excerpt of the named source; the page itself could not be opened (403 / size limit / blocked)
- **[D]** derived by me from sourced numbers (formula shown)
- **[U]** unverified recollection, not seen this session. Do not use without checking

---

## 1. Bosch BMI088

Source A: "BMI088: Data Sheet", Bosch Sensortec, document BST-BMI088-DS000-19, revision 1.9, January 2024 (file bst-bmi088-ds001.pdf). https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
Datasheet note (Sec. 1): values are over lifetime and full temperature/voltage range unless stated; min/max are ±3σ.
Source B: Bosch product page https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/

### Gyroscope (Table 5, pp. 10-11)

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Noise density | 0.014 °/s/√Hz | typ; quoted inside the output-noise row | A Table 5 p.10 [P]; B labels it "typ." [P] |
| Output noise | 0.1 °/s rms | BW = 47 Hz | A Table 5 p.10 [P] |
| Zero-rate offset | ±1 °/s | typ, 25 °C, nominal VDD, slow and fast offset cancellation off | A Table 5 p.10 [P] |
| Offset over temperature (TCO) | ±0.015 °/s/K | typ, -40..+85 °C, ±2000 °/s range | A Table 5 p.10 [P] |
| Offset supply drift | <0.1 °/s/V | typ, 25 °C | A Table 5 [P] |
| Bias stability | NOT in datasheet. Secondary: "<2 °/h" | Bosch flyer/launch press; a Bosch forum moderator calls it typical, not max; one user measured ~3.2 °/h average on X/Y over 19 parts | [S] https://www.eenewseurope.com/en/bosch-launches-imu-for-drones-robotics ; https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BMI088-Bias-Instability-and-Alignment-errors-not-specified-in/m-p/12037 |
| Full-scale ranges | ±125, ±250, ±500, ±1000, ±2000 °/s | selectable | A Table 5 [P] |
| Sensitivity | 262.144 / 131.072 / 65.536 / 32.768 / 16.384 LSB/(°/s) | for the five ranges, 25 °C | A Table 5 [P] |
| Sensitivity tolerance | ±1 % | typ, 25 °C, ±2000 °/s | A Table 5 [P] |
| Sensitivity temp drift (TCS) | ±0.03 %/K | typ, -40..+85 °C | A Table 5 [P] |
| Nonlinearity | ±0.05 %FS | typ, best-fit line, ±1000 and ±2000 °/s | A Table 5 [P] |
| Cross-axis sensitivity | ±1 % | typ | A Table 5 p.11 [P] |
| g-sensitivity | 0.1 °/s/g | max column (column read from text extraction; verify visually) | A Table 5 [P] |
| ODR / filter bandwidth | 2000/532, 2000/230, 1000/116, 400/47, 200/23, 100/12, 200/64, 100/32 Hz | register GYRO_BANDWIDTH 0x10 = 0x00..0x07. Table 5 lists 523 Hz where the register table says 532 Hz | A Sec. 5.5.6 p.39 and Table 5 p.11 [P] |
| Data-rate tolerance | ±0.3 % | | A Table 5 p.11 [P] |
| Start-up time | 30 ms | to ±1 °/s of final value, from power-off; also 30 ms wake from suspend, 10 ms from fast power-up | A Table 3 p.8 [P] |

### Accelerometer (Table 4, p. 9)

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Noise density | 160 µg/√Hz (X, Y); 190 µg/√Hz (Z) | typ, ±3 g range, 25 °C, normal mode | A Table 4 [P] |
| Noise density, other ranges | NOT in datasheet. Product page: 175 µg/√Hz typ (range unstated). Press says 230 µg/√Hz at ±24 g; Mouser says 175 at ±24 g (conflict) | | B [P]; [S] eenewseurope / mouser |
| Zero-g offset | 20 mg | typ, 25 °C, ±6 g range (B: "±20 mg typ over lifetime") | A Table 4 [P]; B [P] |
| Offset temp drift (TCO) | <0.2 mg/K | typ | A Table 4 [P] |
| Sensitivity tolerance | NOT FOUND (no such row in rev 1.9) | | A [P] |
| Sensitivity | 10920 / 5460 / 2730 / 1365 LSB/g | ±3 / ±6 / ±12 / ±24 g, 25 °C | A Table 4 [P] |
| Sensitivity temp drift | 0.002 %/K | typ | A Table 4 [P] |
| Ranges | ±3, ±6, ±12, ±24 g (±24 g confirmed) | | A Table 4 [P] |
| Nonlinearity | 0.5 %FS | typ, best-fit line, ±3 g | A Table 4 [P] |
| Cross-axis / alignment | 0.5 % / 0.5° | typ | A Table 4 [P] |
| ODR | 12.5 to 1600 Hz (12.5, 25, 50, 100, 200, 400, 800, 1600) | | A Table 4, Table 8 [P] |
| -3 dB bandwidth, Normal | 5, 10, 20, 40, 80, 145, 230 (200 Z), 280 (245 Z) Hz | for the eight ODRs above, acc_bwp = 0xA | A Table 8 p.16 [P] |
| -3 dB bandwidth, OSR2 | 2, 5, 9, 19, 38, 75, 140, 234 (215 Z) Hz | acc_bwp = 0x9 | A Table 8 [P] |
| -3 dB bandwidth, OSR4 | 1, 3, 5, 10, 20, 40, 80, 145 Hz | acc_bwp = 0x8 | A Table 8 [P] |
| Power-up time | 1 ms | first valid sample from suspend; then write ACC_PWR_CTRL = 4 and wait 450 µs | A Table 2 p.8, Sec. 4.1.1 [P] |
| Group delay | NOT FOUND (not specified for either sensor) | | A [P] |
| Sensor-time resolution | 39.0625 µs | 24-bit counter | A Sec. 4.3 [P] |

Derived [D]: gyro ARW = 0.014 x 60 = 0.84 °/√h. Accel VRW = 160 µg/√Hz -> 0.094 m/s/√h (Z: 0.112).

---

## 2. TE MS5611-01BA03

Source: "MS5611-01BA03 Barometric Pressure Sensor, with stainless steel cap", TE Connectivity, 06/2017, pp. 2-4. https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036 [P]

| OSR | RMS resolution (mbar, typ) | Height equivalent near sea level [D] | Conversion time min / typ / max (ms) | RMS temp resolution (°C) |
|---|---|---|---|---|
| 4096 | 0.012 | 0.100 m | 7.40 / 8.22 / 9.04 | 0.002 |
| 2048 | 0.018 | 0.150 m | 3.72 / 4.13 / 4.54 | 0.003 |
| 1024 | 0.027 | 0.225 m | 1.88 / 2.08 / 2.28 | 0.005 |
| 512 | 0.042 | 0.350 m | 0.95 / 1.06 / 1.17 | 0.008 |
| 256 | 0.065 | 0.541 m | 0.48 / 0.54 / 0.60 | 0.012 |

Resolution: "Pressure output characteristics" p.4, VDD = 3 V, 25 °C. Conversion time: "Analog digital converter" p.3.
Conversion [D]: dP/dh = ρg = 1.225 x 9.80665 = 12.01 Pa/m (ISA sea level), so 1 mbar = 8.32 m. Datasheet p.1 itself states "altitude resolution of 10 cm".

| Accuracy parameter | Value | Conditions | Location |
|---|---|---|---|
| Accuracy | ±1.5 mbar (min/max) = ±12.5 m [D] | 25 °C, 750 mbar | p.2 |
| Total error band, no autozero | ±1.5 / ±2.0 / ±3.5 / ±6.0 mbar | 25 °C 700-1100 mbar / 0..50 °C / -20..85 °C / -40..85 °C (450-1100 mbar) | p.4 |
| Total error band, autozero at one pressure point | ±0.5 / ±1.0 / ±2.5 / ±5.0 mbar (±0.5 mbar = ±4.2 m [D]) | 25 °C 700-1100 mbar / 10..50 °C / -20..85 °C / -40..85 °C | p.4 |
| Supply-voltage error | ±2.5 mbar | VDD 1.8-3.6 V | p.4 |
| Long-term stability | ±1 mbar/yr | typ | p.4 |
| Temperature accuracy | ±0.8 °C (25 °C); ±2.0 (-20..85); ±4.0 (-40..85) | min/max | p.4 |
| Range | 10-1200 mbar (full accuracy 450-1100) | | p.4 |

A "relative accuracy" figure is not given separately; the autozero error band is the closest equivalent.

---

## 3. Raspberry Pi Global Shutter Camera (Sony IMX296LQR-C)

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Pixel size | 3.45 x 3.45 µm | | RPi camera docs, hardware table https://www.raspberrypi.com/documentation/accessories/camera.html [P]; product brief (Mar 2023) p.3 https://datasheets.raspberrypi.com/gs-camera/gs-camera-product-brief.pdf [P] |
| Active resolution | 1456 x 1088 (1.58 MP), RAW10 | | same [P] |
| Optical format / diagonal | 1/2.9", 6.3 mm diagonal (computed 6.27 mm [D]) | | same [P] |
| Minimum exposure | "as low as 30 µs" (docs: "down to 30 µs") | adequate light | product brief p.2; docs [P] |
| Minimum exposure, mechanism | 1 line; exposure = lines x timePerLine + 14.26 µs, timePerLine = 550 / 37.125 MHz = 14.815 µs -> 29.07 µs [D] | libcamera helper | libcamera patch 17396 (N. Patuck, Sep 2022) https://patchwork.libcamera.org/patch/17396/ [P] |
| Max exposure | 15.5 s | | docs [P] |
| Free-run frame rate | 60 fps at full resolution (60.38 fps [D]: HMAX 1100 at 74.25 MHz, VMAX = 1088 + 30 min vblank = 1118 lines = 16.563 ms) | | docs [P]; driver constants, raspberrypi/linux rpi-6.6.y drivers/media/i2c/imx296.c [P] |
| XTR: exposure control | Yes, by pulse width: "exposure time is equal to the low pulse-width time plus an additional 14.26 µs" | active-low pulse on XTR | docs, external-trigger section [P] |
| XTR: frame rate | Set by pulse frequency ("directly controlled by how often you pulse the pin") | example 30 Hz PWM | docs [P] |
| XTR: electrical / setup | 1.8 V input (divider e.g. 1.5 k / 1.8 k from 3.3 V); remove R11 if Q2 fitted; `echo 1 > /sys/module/imx296/parameters/trigger_mode`; always pass a fixed `--shutter` (value ignored); fix gain/AWB or early frames are dropped | | docs [P] |
| XTR: sensor registers | trigger_mode = 1 sets CTRL0B.TRIGEN (0x300b) and LOWLAGTRG.FAST (0x30ae) | "fast trigger" bit | driver [P] |
| Trigger edge -> exposure start latency / jitter | NOT FOUND. Not in Raspberry Pi docs; Sony IMX296 datasheet is not public | | |
| Latency/jitter, secondary | Sony Pregius sensors in normal trigger mode align exposure start to the line period: jitter up to 1 line, latency 2-4 lines; low-latency mode starts on the trigger. 1 line = 14.8 µs here [D]. Not IMX296-specific | another vendor's cameras | [S] Balluff https://assets.balluff.com/documents/DRF_957327_AA_000/UseCases_section_Trigger_mvLowLatency.html |
| Max triggered frame rate | NOT FOUND (no figure documented). Bound [D]: period >= exposure + 16.56 ms readout, i.e. under ~60 fps | | |

Recommendation: measure trigger-to-exposure offset on the bench (strobe/LED on a scope); the only documented timing term is the fixed +14.26 µs.

---

## 4. Edmund Optics #34-711 (2.5 mm FL, f/2, High Resolution, M12)

Source: Edmund Optics UK product page, stock #34-711. https://www.edmundoptics.co.uk/p/25mm-fl-high-resolution-f2-micro-video-lens/3949 [P] (US page returned 403)

| Parameter | Value | Conditions | Location |
|---|---|---|---|
| Focal length / aperture | 2.50 mm, f/2, fixed iris | | spec table |
| Stated field of view | 125° horizontal | "@ max sensor format" = 1/2.8" | spec table |
| Max sensor format / image circle | 1/2.8" / 6.50 mm | family blurb says "optimized to cover 2/3"", which conflicts; trust the per-part table | spec table |
| Distortion | -36 % (TV distortion) | | spec table |
| Lens type | Not stated; family is called "Wide Field of View Board Camera Lens" | | page text |
| Resolution | 100 lp/mm at 64 % (axis), 57 % (0.7 field), 34 % (full field) contrast; listed pixel size 3.45 µm | | spec table |
| Working distance | 400 mm to infinity | | spec table |
| Mount / size | M12 x 0.5; length 17.97 mm; max diameter 20 mm; no IR-cut | | spec table |

FOV on the IMX296 (5.023 x 3.754 mm, diagonal 6.271 mm; f = 2.5 mm) [D]:

| Model | HFOV | VFOV | DFOV |
|---|---|---|---|
| Pinhole (rectilinear), 2·atan(d/2f) | 90.3° | 73.8° | 102.9° |
| Equidistant (f-theta), d/f | 115.1° | 86.0° | 143.7° |
| Stereographic, 4·atan(d/4f) | 106.7° | 82.3° | - |

Comparison [D]: a 1/2.8" sensor is about 5.6 mm wide (Edmund does not state the width it used). Pinhole would give 96.5° on that width, f-theta 128.3°; Edmund states 125°. A rectilinear lens would need a 9.6 mm wide sensor to reach 125°. So the lens is strongly barrel-distorted, close to an f-theta (fisheye-like) mapping, not rectilinear. Expect roughly 110-115° horizontal on the IMX296 and use a fisheye calibration model (e.g. Kannala-Brandt), not pinhole + low-order radial terms. The IMX296 diagonal (6.27 mm) fits inside the 6.50 mm image circle with only 0.11 mm radial margin, so expect corner fall-off.

---

## 5. u-blox MAX-M10S

Source A: "MAX-M10S Data sheet", UBX-20035208, R08, 30-Jan-2026, Sec. 1.2 Tables 1-3, pp. 4-5. https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf [P]
Source B: "u-blox M10 SPG 5.10 Interface description", UBX-21035062, R03, 27-Jun-2023. https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf [P]
Source C: "MAX-M10S Integration manual", UBX-20053088 (over the 10 MB fetch limit; search excerpts only). https://content.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf [S]

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Horizontal position accuracy | 1.5 m CEP | typical; CEP 50 %, 24 h static, -130 dBm, >6 SVs per GNSS; all multi-GNSS modes. Single GNSS: GPS 1.5, GLONASS 4, BDS B1I 2, Galileo 3, BDS B1C 2 m | A Tables 2-3, footnote 6 |
| Velocity accuracy | 0.05 m/s | 50 % at 30 m/s, dynamic operation | A Table 1, footnote 2 |
| Heading accuracy | 0.3° | same condition | A Table 1 |
| Time-pulse accuracy | 30 ns RMS; 60 ns 99 % | | A Table 1 |
| Max nav update rate, default mode | 10 / 6 / 3 / 8 / 4 Hz | GPS+GAL / +GLO / +BDS B1I (factory default) / +BDS B1C / +BDS B1C+GLO; min 98 % fix rate | A Table 1, footnote 3 |
| Max nav update rate, high-performance mode | 20 / 16 / 12 / 16 / 10 Hz | same five combinations; "configuration required" | A Table 1, footnote 4 |
| Max nav update rate, single GNSS | 18 Hz default; 25 Hz high performance | any one of GPS, GLONASS, BDS B1I, Galileo, BDS B1C | A Table 3 |
| Rate configuration limits | CFG-RATE-MEAS minimum 25 ms; CFG-RATE-NAV 1..127 | | B, CFG-RATE section |
| Sensitivity | tracking/nav -167 dBm; reacquisition -160; cold start -148; hot start -159 | multi-GNSS modes; "demonstrated with a good external LNA" | A Table 2, footnote 12 |
| TTFF | cold 23-28 s; hot 1 s | multi-GNSS, all SVs at -130 dBm | A Table 2 |
| Operational limits | <= 4 g; 80,000 m; 500 m/s | "assuming Airborne 4 g platform" | A Table 1, footnote 1 |
| Dynamic model constants | CFG-NAVSPG-DYNMODEL: PORT 0, STAT 2, PED 3, AUTOMOT 4, SEA 5, AIR1 6 (<1 g), AIR2 7 (<2 g), AIR4 8 (<4 g) | key 0x20110021 | B Table 23 |
| Airborne <1g limits | 80,000 m; 100 m/s horizontal; 6,400 m/s vertical; sanity check on altitude; max position deviation "Large" | column mapping inferred from excerpt | C, dynamic platform table [S] |
| Airborne <2g limits | 80,000 m; 250 m/s; 10,000 m/s; altitude; Large | "recommended for typical airborne environments" | C [S] |
| Airborne <4g limits | 80,000 m; 500 m/s; 20,000 m/s; altitude; Large | "only recommended for extremely dynamic environments" | C [S] |
| Effect on accuracy | High-acceleration models "can result in a higher standard deviation in the reported position"; no 2D fixes in airborne models | | C [S] |

Note: the factory-default constellation set gives only 3 Hz (12 Hz in high-performance mode); GPS+GAL is needed for 10/20 Hz.

---

## 6. Round-parachute swing

### (a) Knacke Table 5-1: NOT VERIFIED

T. W. Knacke, "Parachute Recovery Systems Design Manual", NWC TP 6575, March 1991 (DTIC AD-A247 666), Table 5-1 "Solid Textile Parachutes" (PDF page ~82). https://apps.dtic.mil/sti/pdfs/ADA247666.pdf returned 403 and the ~20 MB file exceeds the fetch limit, so the table was not seen. The rows below are my recollection and must be checked against the manual.

| Type | CD0 | Avg. angle of oscillation | Status |
|---|---|---|---|
| Flat circular | 0.75-0.80 | ±10° to ±40° | [U] |
| Conical | 0.75-0.90 | ±10° to ±30° | [U] |
| Hemispherical | 0.62-0.77 | ±10° to ±15° | [U] |
| Annular | 0.85-0.95 | < ±6° | [U]; CD0 range corroborated by an annular-parachute paper citing NASA Ames tests [S] |
| Cross | 0.60-0.85 | 0° to ±3° | [U] |

Verified qualitative ranking, NASA SP-8066 "Deployable Aerodynamic Deceleration Systems" (June 1971), Table I p.7 [P; row alignment reconstructed from OCR order]: flat circular "unstable; large oscillations; high shock; high CD0"; annular and cross "medium shock; good stability; high CD0". https://ntrs.nasa.gov/api/citations/19710021827/downloads/19710021827.pdf

### (b) Measured oscillation data

No free-flight amplitude + period measurement was found for 0.2-1 m canopies: NOT FOUND. Closest data:

| Data | Value | Conditions | Source |
|---|---|---|---|
| Drop tests, 11.9 ft (3.63 m) D0 10 % extended-skirt, Ls/D0 = 0.92 | f = 0.286, 0.286, 0.400, 0.445 Hz (T = 3.5-2.25 s); average angle 25, 22, 27, 27°; max 25, 31, 33, 31°; CD 1.048, 0.870, 0.817, 0.908 | descent 10.7, 16.5, 27.0, 41.5 ft/s; lines 3.33 m | Tutt, "Fluid Structure Interaction Parachute Benchmark Models in LS-DYNA", AIAA 2013-1384, Table IV-1 (data from WADD AFTR 5867) [P] https://airborne-sys.com/wp-content/uploads/2016/09/Fluid-Structure-Interaction-Parachute-Benchmark.pdf |
| Same canopy, Ls/D0 = 0.60 | f = 0.200, -, 0.333, - Hz (T = 5.0, 3.0 s); average 20, 10, 25, -°; max 23, 18, 28, 17°; CD 0.845, 0.766, 0.686, 0.700 | descent 11.9, 17.7, 29.2, 45.2 ft/s; lines 2.18 m | same [P] |
| Trends in that data | frequency rises with speed and with longer lines; angle "follows no firm and discernible pattern" | | same [P] |
| Simple-pendulum check [D] | 2π√(L/g) with L = line length: 3.66 s (measured 3.5 s at lowest speed) and 2.96 s (measured 3.0-5.0 s) | line length alone gets within ~5 % at low speed for the long lines only | |
| Cluster of three solid flat circular | ±7.1°; radially vented canopies ±3.4° and ±4.5° | drop tests; CD about 0.64 | Burk, NASA TN D-2271 (1964), abstract [S] https://catalog.hathitrust.org/Record/011433612 |
| Scale effect | "large parachutes are generally more stable than small models of the same type"; imposed oscillations damp "within one full cycle"; 10-15° pendular/coning motion is normally acceptable for unmanned payloads | | NASA SP-8066 Sec. 2 p.10, Sec. 4 p.50 [P] |
| Canopy-loading effect | below W/(CD·S) = 24 N/m² (0.5 psf) oscillation or gliding increases. CanSat at 10.5 m/s: 67.5 N/m² [D], above the threshold | | SP-8066 Sec. 2.2.1.3 p.14 [P] |
| Spill hole, model-rocket canopy | drag-force standard deviation 3.9x larger with a 5 % hole than a 10 % hole (hole diameter / canopy diameter) | wind tunnel, 25 m/s; model size not stated | Martins & Wehmann, Rev. Bras. Fis. Tecnol. Apl. 11(2) 2024 [A] https://periodicos.utfpr.edu.br/rbfta/article/view/18034 |
| Radiosonde under parachute (long string) | τ = 2π√(l/g) = 14.9 s for the 55 m RS41 string; 25-40 s periodicity seen after parachute deployment | different geometry; shows line-length scaling | Ingleby et al., AMT 15, 165 (2022), Sec. 2.5 [P] https://amt.copernicus.org/articles/15/165/2022/ |

### (c) Apparent (added) mass

| Item | Value | Reference volume | Source |
|---|---|---|---|
| Axial added mass | m_a = k_a · ρ · (4/3)π R³, k_a = 1.068 (1 - 1.465p - 0.25975p² + 1.2626p³), p = porosity. k_a = 1.068 at p = 0; 0.755 at p = 0.2 [D] | Eq. 8 as extracted is sphere volume of inflated radius R (Greek symbols dropped in extraction; the prose says "hemisphere"). If sphere: 2.14x the air in a hemisphere of the inflated diameter at p = 0 | Guglieri, "Parachute-Payload System Flight Dynamics and Trajectory Simulation", Int. J. Aerospace Eng. 2012, 182907, Eqs. 8-10 [P]; attributed there to Cockrell (AGARDograph 295) and Klimas. https://staff.polito.it/giorgio.guglieri/Flight_Mechanics_Research_Team/Papers_files/12_182907.pdf |
| Tensor coefficients used in a 6-DOF round-canopy model | k11 = 0.5, k33 = 1.0, k44 = 0.24, k15 = 0.75 | reference mass definition not seen | Dobrokhodov, Yakimenko, Junge, J. Aircraft 40(3) 482-493 (2003) [S] |
| Range across studies | K_ij roughly 0.2 to 1; K33/K11 = 2 used in several studies; defined with ρ and projected diameter cubed | check each paper's normalisation | Ginn, Clark, Braun, AIAA 2014-2390 [S] |
| During inflation | added mass / enclosed mass from 1.2 to 4.1 | model round canopy, FE study | "Added Mass of a Model Round Parachute Canopy during Inflation" [S] |

AGARDograph 295 and White & Wolf were not opened. Normalisations differ between authors (one excerpt notes Cockrell's and Yavuz's scales differ substantially), so treat k as 0.5-1.1 x sphere-of-inflated-radius and sweep it.

### (d) Suspension-line length

| Item | Value | Source |
|---|---|---|
| Knacke Le/D0 "about 1.0" recommendation | NOT VERIFIED | |
| Knacke, as quoted second-hand | Fig. 5-20: CD rises with Le/D0 up to 2.0; "line-length ratios above 1.5 may be detrimental because of the associated weight increase" (p. 5-21) | forum post quoting the manual [S-secondary] https://www.rocketryforum.com/threads/chute-cord-length.82765/post-910983 |
| Measured effect | Ls/D0 0.92 vs 0.60: CD 0.82-1.05 vs 0.69-0.85 | Tutt 2013 Table IV-1 [P] |
| General | suspension-line length is the one shape factor whose effect on CD is "defined adequately in quantitative form" | SP-8066 Sec. 2.2.1.2, Fig. 2b [P] |

### (e) Huygens

| Item | Value | Source |
|---|---|---|
| Tilt | up to 15° early in descent | Karkoschka et al., "DISR imaging and the geometry of the descent of the Huygens probe within Titan's atmosphere", Planet. Space Sci. 55, 1896-1935 (2007), abstract [S; bibliographic record confirmed A] doi:10.1016/j.pss.2007.04.019 |
| Average swing rate | 5 °/s at high and low altitudes; 13 °/s between 110 and 30 km | same [S] |
| Peak swing rate | often above 40 °/s (design limit 6 °/s) | same [S] |
| Swing period | NOT FOUND | |
| Karkoschka 2016, Icarus 270, 326-338 (doi:10.1016/j.icarus.2015.06.012) | abstract not retrievable: NOT FOUND | |
| Lorenz et al. 2007, Planet. Space Sci. 55, 1936-1948 | qualitative only (pendulum motion and buffeting seen by SSP tilt sensors): numbers NOT FOUND | [S] |

### (f) Dropsondes

| Item | Value | Source |
|---|---|---|
| Sampling, fall rate, resolution | 2 Hz; 11-12 m/s near surface; ~5 m vertical | Franklin, Black, Valde, "GPS Dropwindsonde Wind Profiles in Hurricanes and Their Operational Implications", Wea. Forecasting 18, 32-44 (2003), Sec. 2 [P] https://www.e-education.psu.edu/files/meteo241/file/Franklin-etal-2003-GPS%20dropsondes.pdf |
| Wind accuracy | precision 0.2 m/s; absolute 0.5-2.0 m/s (attributed to Hock & Franklin 1999) | same [P] |
| Filtering | anti-alias filter for scales the 0.5 s sampling cannot resolve (3 s or less); final winds smoothed with a 5 s cutoff wavelength, about 55 m of fall | same [P] |
| QC | GPS vertical speed compared with hydrostatic fall rate to catch bad GPS winds; vertical-wind uncertainty 0.5-1.0 m/s from parachute variation | same, Sec. 2-3 [P] |
| Pendulum handling | handled by parachute design, not by a dedicated filter: square-cone parachute "eliminates any pendulum motion of the sonde" | NCAR EOL AVAPS page [P] https://www.eol.ucar.edu/observing_facilities/avaps-dropsonde-system |
| Sonde geometry | dropsondes hang 1 m or less below the parachute; AVAPS applies an inertial (wind-shear lag) correction, radiosonde processing does not | Ingleby et al. 2022 [S for the 1 m figure; P for the correction] |
| RD41 (closest analogue to this CanSat) | 323 g; 7.0 cm dia x 40.6 cm; 30 x 30 cm parachute; ~11 m/s at sea level, ~21 m/s at 12 km; winds 4 Hz, PTU 2 Hz; wind ±0.5 m/s, resolution 0.01 m/s | NCAR EOL table and Vaisala RD41 datasheet B211706EN [S] https://www.eol.ucar.edu/node/11708 |
| Hock & Franklin 1999, BAMS 80, 407-420 | full text not retrievable (403): its own filter description NOT FOUND | doi:10.1175/1520-0477(1999)080<0407:TNGD>2.0.CO;2 |

---

## 7. UK map and terrain data

### (a) Orthoimagery horizontal accuracy

| Product | Value | Conditions | Source |
|---|---|---|---|
| OS MasterMap Imagery Layer | 1.1 m RMSE absolute ("All imagery is captured to an absolute accuracy of 1.1m RMSE"); 25 cm resolution; true-ortho since 2015; geometric fidelity ranks above absolute accuracy | one figure for both capture tiers | OS MasterMap Imagery Layer Technical Specification, docs.os.uk [S; direct fetch blocked] https://docs.os.uk/os-downloads/products/maps-and-imagery-portfolio/os-mastermap-imagery-layer/os-mastermap-imagery-layer-technical-specification |
| Bluesky aerial (APGB supplier) | 1 m RMSE absolute; 12.5 cm urban / 25 cm national GSD | reseller listing, not an APGB spec | UP42 marketplace listing [S-secondary] https://up42.com/marketplace/data/archive/bluesky-aerial-imagery |
| APGB / Getmapping official spec | NOT FOUND | | |
| EA Vertical Aerial Photography | no number: "ortho-rectified using simultaneous LIDAR and GPS to a high spatial accuracy"; 10-50 cm resolution; BNG tiles | NOT FOUND for RMSE | data.gov.uk record [P] https://ckan.publishing.service.gov.uk/dataset/vertical-aerial-photography |
| Google Earth, global | 39.7 m RMSE (range 0.4-171.6 m); 24.1 m RMSE in more-developed countries; 44.4 m developing | 436 points, 109 cities; reference Landsat GeoCover (itself ~50 m) | Potere, Sensors 8(12), 7973-7981 (2008), abstract [S] https://www.mdpi.com/1424-8220/8/12/7973 |
| Google Earth, modern city | ~0.1 m to ~2.7 m | Montreal, 10 GPS check points | Goudarzi & Landry, Geodesy and Cartography 43(2), 56-65 (2017), abstract [S] https://journals.vilniustech.lt/index.php/GAC/article/view/533 |
| Maxar Vivid basemaps | 5 m CE90 (Standard/Advanced); 2 m CE90 (Premium, major urban) | | Maxar product page and developer docs [S] https://maxar.com/products/imagery-basemaps |
| WorldView-3 | < 3.5 m CE90 without ground control | sensor spec as quoted | JRC report JRC99433 [S] |

Conversion [D]: for a circular normal error CE90 = 2.146 σ_axis = 1.52 RMSE_r, so 5 m CE90 is about 3.3 m radial RMSE (2.3 m per axis); 1.1 m RMSE is about 1.7 m CE90 if that RMSE is radial.

### (b) EA National LIDAR Programme

| Parameter | Value | Source |
|---|---|---|
| Resolution / coverage | 1 m; all England; 302 blocks; flown in winter, Jan 2017 to Feb 2023; EPSG:27700; heights to Newlyn | data.gov.uk record [P] https://www.data.gov.uk/dataset/f0db0249-f17b-4036-9e65-309148c97ce4/national-lidar-programme |
| Vertical accuracy | ±15 cm RMSE (absolute, against ground-truth surveys) | dataset record and EA Survey Data Catalogue FAQ [S] https://dsp.agrimetrics.co.uk/support/faqs/275879146 |
| Relative (random) height error | no more than ±5 cm expected; QC pass needs <15 cm RMSE and 10 cm standard deviation / random error; ground-truth points ±3 cm RMSE | same FAQ and "LIDAR Ground Truth Surveys" record [S] |
| Point density | at least 1 pt/m² for the 1 m DSM; can be lower for the DTM under vegetation | FAQ [S] |
| Horizontal accuracy | NOT FOUND | |
| Products | DTM (last return classified as ground); DSM (last or only return); first-return DSM (better for tree tops and building edges); classified .laz point cloud | data.gov.uk record [P] |

### (c) Hedgerow and boundary-tree heights

| Item | Value | Source |
|---|---|---|
| Typical hedge height | "Most hedgerows, whether managed or unmanaged, are between 1 and 6 metres in height": 390,000 km in England | UKCEH press release on Land Cover Plus: Hedgerows (EA LiDAR 2016-2021) [P] https://www.ceh.ac.uk/press/high-tech-aerial-mapping-reveals-englands-hedgerow-landscape |
| Lower / upper classes | 67,000 km below 1 m; 185,000 km above 6 m (mature hedgerow trees, overgrown hedges become lines of trees) | same [P] |
| Managed-hedge trend | majority of surveyed hedges now taller than 2 m rather than 1-2 m (2007 to 2023); scheme hedges 0.2 m taller | UKCEH / Natural England survey, via Hedgelink and UKCEH press [S] https://hedgelink.org.uk/news/new-countyside-survey-results-for-hedgerows/ |
| Favourable-condition threshold | average height > 1.0 m, excluding bank | Defra Hedgerow Survey Handbook 2nd ed. (2007), via NE favourable-conservation-status definition [S] https://nora.nerc.ac.uk/id/eprint/527881/1/N527881CR.pdf |
| Height of field-boundary trees | NOT FOUND (only the ">6 m" class above) | |

Sizing suggestion [D]: hedges 2-3 m typical, 6 m upper bound; boundary trees need a separate allowance taken from the first-return DSM minus DTM for the actual launch site.

---

## 8. Published TRN numbers

### Mars 2020 Lander Vision System

| Parameter | Value | Source |
|---|---|---|
| Position requirement | < 40 m relative to the map, from an inertial error up to 3.2 km; stated on Johnson's 2021 slides as "40 m 99 %tile at 10 seconds" | JPL Robotics TRN page [P] https://www-robotics.jpl.nasa.gov/what-we-do/flight-projects/mars-2020-rover/terrain-relative-navigation/ ; Cheng, Ansar, Johnson, Earth and Space Science 2021, doi:10.1029/2020EA001560 [S]; JHU-APL LuNaMaps slides [S] |
| Time to fix | under 10 s; typically 40 m is reached after the first fine image, about 6 s | NEPP ETW 2021 slides ("landmarks in <10 sec") [P] https://nepp.nasa.gov/docs/etw/2021/15-JUN-21_Tues/1130_Skinner-Mars-2020-NEPP-Contribution-CL21-2593.pdf ; the 6 s figure [S, primary not identified] |
| Altitude range | 4200 m down to 2000 m | Alberico et al., arXiv:2409.04633 Sec. II-A1, citing Johnson 2017 [P] https://arxiv.org/html/2409.04633v1 |
| Maps | 12 m/pixel (coarse) and 6 m/pixel (fine), from orbital imagery | JPL page [P] |
| Coarse matching | 15 landmarks across 3 images; 128x128 templates in 1024x1024 map windows (frequency domain) | JPL page [P] |
| Fine matching | up to 150 landmarks per image; 15x15 templates in 100x100 windows | JPL page [P] |
| Camera | LCAM 1024x1024, 90° x 90° FOV | [S]; primary is Maki et al., Space Sci. Rev. 216:137 (2020), not opened |
| Image rate | NOT FOUND | |
| Achieved, system level | touchdown 5 m from the LVS-selected target, against a 60 m requirement | JPL page [P]; Zheng et al., "Mars 2020 Lander Vision System Flight Performance", NTRS 20230006986 abstract [A] |
| Achieved, error budget | post-landing estimate 8.53 m or better, vs 33 m pre-landing estimate and 60 m requirement | Chen et al., "Assessment of M2020 Terrain Relative Landing Accuracy", NTRS 20230007014 abstract [A] |
| Achieved, LVS position error alone | "a few meters" against 40 m; exact figure NOT FOUND | JGCD paper doi:10.2514/1.G006780, abstract [S] |
| Pre-flight field test | 40 m (3σ) in 10 s on flight-like processing, helicopter over Mars-analogue terrain; later campaign over 600 real-time runs | NTRS 20160009627 and 20220000862 abstracts [S] |

Citation correction: Johnson et al. 2017 is **AAS 17-036** (not 17-038), NTRS 20190025658; NTRS holds no PDF.

### COBALT / LVS on Xodiac

| Parameter | Value | Source |
|---|---|---|
| Flights | two open-loop free flights, spring 2017, to 500 m altitude with 300 m downrange divert | Carson et al., "COBALT: Terrestrial Flight Test of Landing Navigation Using Lander Vision System with Navigation Doppler Lidar", NTRS 20210008166 abstract [A] https://ntrs.nasa.gov/citations/20210008166 |
| Position error vs GPS | "within meters" of the vehicle GPS solution; numeric value NOT FOUND | same [A]. The NTRS PDFs 20170001743 and 20170009437 are outline drafts with no results [P] |

### Draper/MIT balloon and New Shepard TRN (Maggio et al. 2023, arXiv:2302.08011; read via ar5iv [P])

| Parameter | Balloon (World View, 2019) | New Shepard NS-23 (2022) |
|---|---|---|
| Altitude | 33 km down to 4.5 km | 3.5 to 8.5 km |
| Average absolute position error | 287 m (down camera); 284 m (side camera); abstract: < 290 m over > 150 km | 54 m (camera 1); 34 m (camera 2); abstract: < 55 m |
| Per-axis / peak | down camera: Up 78, East 207, North 124 m | peak < 120 m; as low as 10 m at 3.5 km |
| Cameras | 1024x768; 32° FOV (12 mm) down, 76° FOV (4.5 mm) side | 1024x768; 31° FOV (12.5 mm), two side-looking |
| Rates | 20 Hz capture, 2 Hz processed | 20 Hz capture |
| IMU | ADIS16448 at 820 Hz, gyro only, used between frames; without gyro, tracking fails in fast-spin segments (Table II) | not used |
| Dynamics | spins over 20 °/s; cloud and cord occlusion | up to 880 km/h |
| Section | Sec. IV-A, V, VII-A | Sec. IV-B, VII-B |

Scale comparison [D]: errors are roughly 0.4-2 % of height for NS-23 and LVS (40 m at 2-4.2 km) and 1-6 % for the balloon; 1 % of 300 m is 3 m.

---

## Caveats

1. **Knacke Table 5-1 was not seen.** DTIC blocked the download and the file exceeds the fetch limit. The CD0 and oscillation rows are my recollection [U]; check them in the manual before calibrating anything. The "Le/D0 about 1.0" recommendation is likewise unverified.
2. **No small-canopy (0.2-1 m) free-flight swing measurements were found.** The only tabulated amplitude and period data are for a 3.6 m extended-skirt canopy (AFTR 5867 via Tutt 2013). NASA SP-8066 says small models are less stable than large ones, so treat 10-30° as a floor rather than a ceiling, and log your own gyro data on a drop test.
3. **Apparent mass:** the one formula read directly (Guglieri 2012) lost its Greek symbols in text extraction and its prose says "hemisphere" while the equation reads as sphere volume. AGARDograph 295, White & Wolf and Doherr were not opened. Sweep the coefficient.
4. **BMI088:** typ/max column assignments for g-sensitivity and output noise were read from extracted text, not the rendered table. Bias stability (<2 °/h) and the ±24 g noise figure are not in the datasheet; the secondary sources conflict on the latter. No accelerometer sensitivity tolerance and no group delay are specified.
5. **IMX296 trigger latency and jitter are undocumented publicly.** Sony's datasheet was not obtainable. Only the +14.26 µs exposure offset is documented.
6. **Edmund #34-711:** Edmund does not state the sensor width behind its 125° figure or the projection type; the f-theta conclusion is my inference from the stated FOV and -36 % TV distortion.
7. **u-blox dynamic-platform limits** come from search excerpts of the integration manual (file too large to fetch); the column mapping of the 100/250/500 and 6400/10000/20000 figures is inferred.
8. **OS MasterMap 1.1 m RMSE** and the **EA ±15 cm RMSE** come from search excerpts of the official pages (direct fetch blocked). The OS spec does not say whether 1.1 m is radial or per-axis. EA LIDAR horizontal accuracy, EA aerial-photo accuracy and an official APGB figure were not found.
9. **Google Earth** has no vendor accuracy statement; published assessments range from sub-metre to tens of metres by location and date.
10. **Mars 2020 LVS:** image rate and the exact in-flight LVS-only position error were not found; the 4200 m, 6 s and LCAM figures are second-hand. Johnson 2017 (AAS 17-036) itself is not publicly downloadable.
11. **COBALT:** only "within meters of GPS" is public at abstract level.
12. **Huygens:** numbers are from the 2007 paper's abstract via search excerpt; the 2016 Karkoschka paper and Lorenz's figures were not retrievable; no swing period found.
13. **Hock & Franklin 1999** full text was blocked; dropsonde filter details are from Franklin et al. 2003, which cites it.
