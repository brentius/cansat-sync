# Current mission and hardware baseline

Aligned 13 October 2026. This page summarises the selected design and evidence; it does not certify a flight article. The detailed purchasing authority is [the flight computer BOM](flight_computer_bom.md), incorporating its saved MAIN completion and the 3 October temperature replacement. The [mission proposal](trn_secondary_mission_proposal.md) defines objectives; the [connection guide](flight_computer_connections.md) and [serial protocol](stm32_pi_link_protocol.md) define intended interfaces. Actual CAD/schematic artifacts describe implementation progress, not automatically a validated design.

## 1. Selected configuration

| Function | Baseline | Qualification / status |
|---|---|---|
| Image processing and storage | Raspberry Pi Zero 2 W, 512 MB RAM; SanDisk High Endurance 256 GB SDSQQNR-256G-GN6IA | No onboard timing or end-to-end runtime verified |
| Timing, sensing and primary telemetry | STM32F405RGT6 | Firmware not implemented; Python reference software is not STM32 firmware |
| IMU | Bosch BMI088 on MAIN, separate accel/gyro CS and data-ready captures | Gyro about 1 kHz; 100 Hz propagation/link design; synchronization/group delay require bench validation |
| Pressure | TE MS561101BA03-50 on MAIN | Nominal 50 Hz pressure, slower die-temperature compensation; noise resolution is not absolute altitude accuracy; static-pressure vent needed |
| Air-temperature prototype | TE GA10K3MCD1 external 10 kOhm NTC; 150 kOhm excitation resistor, OPA333 buffer and ADS1115 ADC, quiet 3.0 V rail | Replaces FP07; flight selection conditional on calibration and installed air response. Barometer die temperature is not air temperature |
| Camera | Official Raspberry Pi Global Shutter Camera, Sony IMX296, native 1456 x 1088; Zero-specific CSI cable | STM32 XTR pulse/capture intended; camera setup and frame association unverified |
| Optics | Edmund #34-711 2.5 mm f/2 M12 lens; #53-675 C-to-M12 adapter and #64-102 lock nut | Provisional until focus/clearance/calibration. 90.3 x 73.8 degrees is an ideal pinhole estimate; about 110-115 degrees horizontal is an inference for the barrel-distorted real lens, not a calibration |
| Independent reference and timing | u-blox MAX-M10S-00B; Taoglas AGGP.25F.07.0060A active antenna | RF board; GNSS coordinates/velocity isolated from TRN; PPS supplies time reference |
| Radio | Ebyte E22-900M22S; Taoglas FXP895.07.0200C FPC antenna | RF board; band/channel/power, complete antenna match and field range need verification |
| Independent sensor log / recovery | W25Q128JVSIQ 16 MB NOR; CMT-8504-100-SMT-TR sounder; LEDs | MAIN; rolling preflight log with descent retention, not three hours of full-rate raw IMU logging |
| Power | KeepPower protected NCR18650GA 2S1P, SKU C3-302, 7.2 V minimum 3.3 Ah, approximately 100 g | Supplier-approved charging; no balancing claim. Pack usable capacity and assembled loads unmeasured |
| Power distribution | Master switch/fuse/reverse protection -> independent TPS62130 5.1 V and TPS62162 3.3 V; TPS22965 Pi switch, SN74LVC2G125 UART buffers; TPS7A2030 quiet 3.0 V | Primary does not depend on Linux, but shares battery/STM32 failure modes |
| Physical arrangement | Top RF board, middle MAIN, bottom camera, side-mounted Pi; selected MAIN J4 Wuerth 61202021621 shrouded 2x10; compatible RF endpoint, keyed IDC mates and ribbon cable remain to be selected/verified | Complete envelope/access/retention not demonstrated; MAIN saved outline 44 x 64 mm; RF outline and enclosure not complete |
| Recovery | Passive reusable canopy, swivel, retained suspension/hardpoint | Canopy type/material/dimensions not frozen; proof-load and settled-descent tests required |

Optional LIS3MDLTR is a yaw aid, not a mandatory baseline part. ICM-42688-P, alternate barometers, TMP117 digital temperature readout and high-g recorder remain alternatives. Changing to them requires coordinated circuit, driver, budget and qualification updates; they are not populated simply because an options table lists them.

## 2. Rates, interfaces and data separation

- Primary mission: temperature/pressure radio telemetry **once per second during descent**, with graphable ground logs.
- Camera: **5 Hz capture target**. Full-rate image logging and 5 Hz TRN processing are separate targets; processing is unbenchmarked. Proposed lower processing rates/landmark counts are trade studies, not reductions of T1's 10 m / 5 s targets.
- BMI088: high-rate acquisition/integration on STM32; snapshots of cumulative attitude/specific-force integrals every 10 ms over the serial link. The Pi uses interval differences for nominal 100 Hz navigation.
- Camera timing: STM32 TIM3 output, TIM2 capture on the 3.3 V trigger signal, divider to camera's 1.8 V XTR; exposure timestamp corrected to the appropriate exposure centre. PPS and sensor captures share the 1 MHz TIM2 timebase; alignment target **<=0.5 ms**, unverified.
- STM32-Pi UART: **230400 baud, 8N1**, buffered 3.3 V; COBS + CRC-16/CCITT-FALSE reference codec in `code/cansat/link.py`. PL011 configuration and hardware stream loss behavior need bench verification.
- Radio packet: **40 bytes** defined in `code/cansat/telemetry.py`, one per second. Serial Nav's 32-byte payload is a different message on a different link. One radio packet carries one selected prediction; all comparison predictors belong in onboard logs/post-flight analysis, not three full predictions in that packet.
- Candidate radio mode: 869.525 MHz, SF7/BW125 kHz/CR4/5, explicit header, radio CRC, eight-symbol preamble. Calculated 40-byte airtime **82.176 ms**, **8.2176%** at 1 Hz without additional transmissions. [Ofcom IR2030](https://www.ofcom.org.uk/siteassets/resources/documents/spectrum/interface-requirements/ir-2030.pdf), published 7 April 2026 and checked 13 October, permits airborne use under IR2030/1/19 at 869.40-869.65 MHz with 500 mW ERP maximum and a <=10% duty-cycle alternative. Count all radio traffic, use the organisers' allocation, verify radiated power/emissions; this is a candidate mode, not build certification.
- Temperature excitation: BOM's <=6% duty ceiling remains the selected self-heating goal. The characterisation study's 128 SPS paired readings at 10 Hz require about 19.41% duty in its assumed schedule and are a bench candidate, not an adopted compliant acquisition setting. Resolve rate/settling/self-heating together before firmware implementation. Primary 1 Hz delivery remains required.
- TRN navigation uses camera, map, IMU and pressure; GNSS position/velocity is an independent comparison stream. A separately labelled GPS predictor or GPS-only fallback is allowed but must never be reported as GPS-denied TRN. A common PPS clock is not a position/velocity observation.

## 3. Budgets and model parameters

| Quantity | Current statement | Evidence |
|---|---|---|
| Mass | Design allocation around 330 g; latest model expected 335 g, typical straight sum 324 g; 6-15% probability above 350 g under stated assumptions | [11 October budget](mass_power_budget.md); calculated/synthetic, nothing weighed |
| Battery | Minimum nominal energy 23.76 Wh; latest budget uses about 19.6 Wh usable | Vendor input and model derating, not measured capacity |
| Endurance | Latest model: 3 h waiting with Pi idle + flight + 1 h beacon uses about 4.3 Wh; full-system representative 3 h test still required | Model, not metered endurance; active-load mode must be stated |
| Descent | Existing synthetic studies generally use 10.5 m/s; budget recommends drop-calibrating near 11.0 m/s to give margin above the old-rule 10 m/s floor | Recommendation, not selected/measured canopy performance; rerun affected models after measurement |
| Optical geometry | Historical pinhole simulation uses focal length about 724.6 px with 1456 x 1088 sensor; nominal nadir footprint at 300 m about 603 x 450 m under that model | Calculated ideal geometry, not real-lens footprint or guaranteed matching coverage |
| Image storage | Full sensor at 5 Hz: about 9.9 MB/s tightly packed RAW10 or 15.8 MB/s in 16-bit storage, excluding padding/metadata | Calculated; sustained Pi storage throughput unverified |
| Cost | Complete delivered flight cost not established; dated IC/module subtotal is not the whole £400 budget | BOM includes optics, assembly, mechanics, VAT/delivery as unresolved costs |

## 4. Mission objectives and maturity

T1 retains **<=10 m horizontal position error above 80 m and first confirmed fix within 5 s of canopy settling**, judged against independently logged GNSS with map-datum bias reported separately. T2 tests calibrated position/velocity uncertainty and protection-level coverage; the old compulsory attitude-to-map crossover shape is no longer asserted. T3 compares TRN, GNSS and inertial-only landing predictors. T4 remains an optional safe-target *decision*, with no steering hardware.

S1 retains **<=0.5 m/s RMS image-minus-GNSS wind difference in 20 m layers over 80-300 m**, a method-agreement metric rather than proof of absolute wind accuracy. S2's similarity/roughness inference is conditional on near-surface anemometer support and identifiable thermal/wind information. S3 swing/shear and S4 turbulence scaling are exploratory, with explicit identifiability/record-length limits. Lateral-force airspeed correction is calibration-dependent research, not a proven sensor product.

Candidate Python code exists for acquisition, first-fix solving, a 17-error-state ESKF, wind retrieval, landing prediction, telemetry and the serial codec. No integrated flight application or STM32 firmware exists. First-fix-to-filter handoff, a flight fine matcher, below-80 m visual odometry, real imagery and Pi timings remain open. Study numbers are synthetic and scoped to each retained run; they are not combined flight-system results.

## 5. Source and version policy

Current instructions: BOM for selected parts, connection guide/saved schematic for wiring progress, serial/radio reference code for message definitions, latest budget for estimates, proposal for mission targets. If they disagree, record and resolve the difference; do not silently change hardware or relabel a result.

Historical runs retain their original parameters and dates. The 30 September/1 October mathematics examples and original sizing can use OV9281, Orange Pi/H618, 310 g, 10.5 m/s and earlier image/map settings as explicitly historical inputs. They do not establish current Pi throughput or current mass. The archived September proposal is a source snapshot, not the live mission plan.

The supplied competition rules remain **2025-26**, provisionally used for 66 x 115 mm, 300-350 g, 50 N attachment, 20 g launch, 3 h power, £400 and passive recovery. Current-season rules, deadlines and whether parachute mass counts need confirmation; conservatively include the parachute in budgets. No physical measurements or new simulation runs were produced by the 13 October documentation alignment.
