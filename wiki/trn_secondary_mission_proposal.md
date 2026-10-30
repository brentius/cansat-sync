# Mission Proposal: Terrain-Relative Navigation and Wind Profiling

**A Mars-inspired navigation experiment during passive CanSat descent, with an independently assessed image-derived wind profile**

*Revised 13 October 2026 to the current flight computer BOM and the 9-11 October engineering studies. Preliminary design; no physical flight or bench qualification claimed.*

Implementation update, 27 October: the integrated STM32 and Pi candidate software
is described in the [flight software report](flight_software.md), including source
structure, decisions, host/cross-build evidence and hardware next steps. The design
targets and historical study evidence below retain their original scope.

The [current design baseline](design_baseline.md) summarises selected parts, rates and evidence. [The BOM](flight_computer_bom.md) governs component selection; the [connection guide](flight_computer_connections.md) and [STM32-Pi protocol](stm32_pi_link_protocol.md) govern intended interfaces. The September hardware assumptions are superseded in this live proposal. The original is retained as a [dated historical snapshot](history/trn_secondary_mission_proposal_2026-09-27.md).

## 1. Summary

The primary mission measures **air temperature and pressure during descent and transmits both once per second to the ground station**, where records can be plotted and analysed. MS5611 pressure and an external GA10K3MCD1 temperature probe are acquired by an STM32F405, which also controls timing, independent logging and radio telemetry. The temperature implementation remains conditional on calibration and its response in the installed airflow/shield.

The secondary mission uses a downward-facing IMX296 global-shutter camera, an onboard orthoimage and BMI088 inertial measurements to estimate a map-relative trajectory without GNSS position or velocity in the navigation loop. A Raspberry Pi Zero 2 W performs acquisition, navigation, logging and proposed landing prediction. Independent MAX-M10S observations score the trajectory and a separately labelled GNSS-driven predictor. Primary operation is intended to continue if image processing or Linux fails; shared battery/STM32 failures remain common failure modes.

The trajectory is used to retrieve a wind profile and compare image-derived and GNSS-derived estimates. Their agreement cannot reveal aerodynamic errors shared by both methods. True wind accuracy requires a justified canopy-motion model and independent calibration/reference evidence. Temperature and pressure support an exploratory boundary-layer interpretation.

| Quantity | Current design / target | Evidence and limit |
|---|---|---|
| Ejection altitude | Approximately 300-400 m | Supplied 2025-26 UK guidelines; current season unconfirmed |
| Descent | Drop-calibration recommendation near 11.0 m/s; canopy not frozen | Historical studies use 10.5 m/s. Neither rate has been measured on this can |
| Canopy time | Roughly 27-36 s for 300-400 m at 11 m/s | Ideal height/speed arithmetic, excluding inflation/transients |
| Camera | IMX296, native 1456 x 1088; 5 Hz capture target | Official GS camera; complete timing/logging unverified |
| Lens | Edmund #34-711, 2.5 mm f/2; #53-675 adapter, #64-102 lock nut | Focus, clearance and distorted intrinsics require calibration |
| Ideal optical footprint at 300 m | Approximately 603 x 450 m, 0.414 m/native pixel near nadir | Pinhole model with f=724.6 px. Real usable footprint depends on distortion, calibration and tilt |
| TRN performance T1 | <=10 m horizontal error above 80 m; first confirmed fix <=5 s after canopy settles | Target, not achieved flight performance |
| Camera-IMU time alignment | <=0.5 ms | Hardware capture design; requires scope/optical test |
| Wind-method agreement S1 | <=0.5 m/s RMS per 20 m layer over 80-300 m | Compare retrieved image and GNSS winds; not absolute wind accuracy |
| Mass | Expected about 335 g; design allocation around 330 g | Latest calculated budget, with 6-15% modelled chance above 350 g; no weighing |
| Power / cost | 3 h system-on requirement and £400 flight budget | Modelled energy margin; measured runtime and complete delivered cost outstanding |

## 2. Motivation and framing

Terrain-relative navigation converts images into a position estimate by matching stable ground structure against a stored map. Mars 2020's Lander Vision System is the architectural inspiration: image preparation, map matching, inertial fusion and uncertainty-aware outputs. Our CanSat investigates that navigation chain over terrestrial farmland during a short passive descent; it does not reproduce the flight hardware, environmental qualification or powered landing capability of a planetary vehicle.

MER image-based velocity estimation, Mars 2020 LVS, terrestrial COBALT trials and balloon/suborbital TRN work are precedents listed in the references. They motivate the approach; their performance is not evidence for this can's accuracy. The current acquisition implementation differs from the September LVS-like sketch: it uses a whole-view heading search, landmark pose refinement and confirmation over two distinct frames. The Pi's cost, memory and unmeasured throughput determine practical limits.

The atmospheric motivation is to ask whether a camera already flown for navigation can also recover useful horizontal wind information. Huygens' imaging and Doppler analyses motivate comparing complementary measurements. A CanSat samples a short local atmospheric profile, not a regional climatology or a complete turbulence ensemble.

## 3. Scientific value

### Questions and hypotheses

- **S1 — method agreement.** Image-derived and independently GNSS-derived wind estimates agree within 0.5 m/s RMS per horizontal axis in valid 20 m layers over 80-300 m. Also report vector RMS, bias, sample counts and coverage. A decrease with height is a diagnostic, not a compulsory shape. The methods share the parachute and motion model, so S1 alone does not establish absolute wind accuracy.
- **S2 — conditional boundary-layer interpretation.** With a logged wind reference at known height, a calibrated temperature response and identifiable data, assess whether a similarity-law profile is consistent with observed conditions and whether inferred roughness is compatible in order of magnitude with upwind land cover. Without the near-surface reference, report profile/fit uncertainty and failed identifiability rather than claiming a measured roughness length.
- **S3 — exploratory swing/shear.** Estimate the association between swing/spin and shear, separating canopy-generated dynamics from atmospheric forcing where possible. A single short flight may not support a decisive conclusion.
- **S4 — exploratory turbulence scaling.** Compare constant-scale and height-dependent vertical-descent turbulence priors, with averaging kernels, uncertainty and record-length limitations. The 6 m ground wind is a proposed reference, not an existing installed instrument or guaranteed turbulence truth.

### Physical model and limits

For an ideal drag-only probe with positive distance descended s, horizontal velocity u follows the wind w with lag length ell_tau = v_d^2/g. At the historical 10.5 m/s simulation point this is about 11.2 m; at 11.0 m/s it is about 12.3 m. The simple inversion is

$$
\mathbf w(s)=\mathbf u(s)+\ell_\tau\,\frac{d\mathbf u}{ds}.
$$

The implemented wind retrieval uses a state-space prior and filtering/smoothing rather than directly differentiating noisy velocity. The simple equation is conditional on aerodynamic assumptions; canopy glide is not wind. The [end-to-end study](wind_retrieval_end_to_end.md) found shared glide bias even when S1 passes. Time-averaged lateral specific force is a candidate airspeed measurement; its gain, offset and averaging behavior need real-can calibration before the correction can support a science claim.

Potential temperature and the bulk Richardson number combine air temperature/pressure with wind shear. Monin-Obukhov fitting is conditional on surface-layer applicability, adequate height coverage and an independent near-surface constraint; a thermal gradient alone does not establish all flux/stability parameters.

| Measurement | Proposed use | Important limitation |
|---|---|---|
| Full-field images, nominal 5 Hz | Map-relative position, velocity and wind comparison | Actual processing cadence, terrain texture, blur, map season and lens calibration |
| BMI088 inertial streams | Attitude/trajectory propagation and motion correction | Separate accel/gyro timing, launch clipping margin, noise and bias; instantaneous accelerometer tilt is unreliable under swing |
| MS5611 pressure | Primary telemetry, relative height and density model | Static-port error, datum/calibration and conversion timing dominate sensor resolution |
| External GA10K3MCD1 via ADS1115/OPA333 | Primary air temperature; exploratory stability | <=0.1 K relative precision and <=1 s installed response are qualification goals; the quoted 200 ms is in liquid |
| MAX-M10S independent PVT | Navigation comparison and GNSS wind/predictor branch | Receiver lag, uncertainty and GNSS reference quality need recording |
| Ground anemometer at proposed 6 m | Near-surface wind reference and S2 constraint | Hardware/source not selected; siting, calibration and safe setup need planning |

At 11 m/s and 5 Hz capture, height spacing is about 2.2 m: approximately nine images per 20 m layer before rejection. Capture density is not the same as independent information or matched-fix availability. Map-based coverage below 80 m and visual odometry remain unimplemented flight work.

## 4. Mission objectives and success levels

**Primary:** valid temperature/pressure acquisition, independent sensor logging and 1 Hz radio delivery during descent, with analysable ground records.

**T1:** <=10 m horizontal position error above 80 m and first confirmed fix <=5 s after canopy settling. Score against independent GNSS and report map-datum bias separately, failed acquisitions, availability and false-fix rate. The original target is retained despite terrain-dependent synthetic first-fix results.

**T2:** quantify navigation error and test calibrated horizontal position/velocity covariance and protection-level coverage. Compare a geometry-based error budget with real residuals without requiring the September attitude/map crossover prediction to hold. A protection level is a model-dependent uncertainty output, not a certified safety guarantee.

**T3:** compare landing position/time predictions driven by TRN, GNSS and inertial-only sources, using the same prediction model where possible. Report error and ellipse coverage at common heights; GNSS fallback is explicitly labelled.

**T4 — stretch:** demonstrate uncertainty-aware safe-target selection without steering or an actual divert.

| Level | Criterion |
|---|---|
| Primary floor | Safe passive recovery design and primary temperature/pressure telemetry; received data usable even if retrieval fails |
| Secondary floor | Recover imagery and sensor logs where possible; attempt offline map-relative trajectory/wind analysis, with quality and failure reasons reported |
| Minimum live demonstration | Confirmed onboard TRN estimates logged/downlinked with validity and uncertainty |
| Nominal secondary objective | T1 and T2 assessed; S1 comparison made over valid layers; S2 attempted only with required reference/identifiability |
| Further demonstration | T3 landing prediction evaluated; S4 supported only if data permit |
| Stretch | T4 and S3; dropped first if core integration is late |

No success probability is asserted. The earlier subjective percentages have been replaced by observable test gates. Failure of vision does not become a claim of GPS-denied performance merely because the GNSS fallback works.

## 5. Ambition ladder

| Rung | Adds | Gate / fallback |
|---|---|---|
| 0. Offline reconstruction | Logged imagery and sensors -> post-flight trajectory/wind analysis | Requires usable images/map, timestamps and retrieved logs; GNSS science fallback separately labelled |
| 1. Live TRN | Acquisition -> confirmed fix -> fine tracking -> navigation | Requires real-terrain tests and Pi timing; if unavailable, retain primary and Rung 0 |
| 2. Quantified uncertainty | T2 consistency, error budget and protection-level tests | Requires representative data and honest failed-fix accounting |
| 3. Landing prediction | TRN-driven selected live prediction; GNSS/inertial comparisons in logs/offline | Requires velocity and calibrated prediction model; predictor is Pi software, not STM32 redundancy |
| 4. Safe-target decision | Preloaded hazard map and notional reachable region | Only after core integration; no actuators or steering |

Onboard motion-stereo hazard reconstruction, learned feature matching and motorised descent are outside the selected design. Any new algorithm must fit the measured Pi budget before adoption.

## 6. Rules and preliminary compliance

The supplied [UK guidelines](https://drive.google.com/file/d/1LubKeC729i9NbmgHwG-wzu4gwhDO1PfJ/view) are for **2025-26**. The 2026-27 rulebook, template and dates remain unconfirmed. The [PDR checklist](../BOTS/PLANS/PDR_FULL_MARKS_CHECKLIST.md) contains all 14 supplied hardware requirements and report criteria.

Design against the provisional 66 mm diameter x 115 mm height, 300-350 g mass, 50 N parachute connection, 20 g launch load, 3 h powered and £400 flight cost. Include parachute mass until organisers confirm otherwise. Provide accessible battery/master switch, safe materials, a reusable passive recovery system and changeable radio frequency. Antennas/transducers remain inside the diameter before ejection; allowed antenna exceptions are top/bottom, not side protrusions.

The reusable canopy is not yet specified; a round canopy on a swivel is a candidate, not a measured stable solution. The budget recommends trimming by drop measurements near 11 m/s to provide margin above the supplied 10 m/s floor, within the recommended 10-15 m/s range. Replaced launch-provider parachutes can change swing, glide, descent time and science, so bring measured data and log any replacement.

The current radio payload is **40 bytes at 1 Hz**. Candidate 869.525 MHz SF7/BW125/CR4/5 gives **82.176 ms calculated airtime**, or **8.2176%** without extra packets/retries. [Ofcom IR2030](https://www.ofcom.org.uk/siteassets/resources/documents/spectrum/interface-requirements/ir-2030.pdf), published 7 April 2026, IR2030/1/19 permits airborne use at 869.40-869.65 MHz subject to its conditions, including a <=10% duty-cycle alternative and 500 mW ERP maximum. Organiser allocation, total airtime, antenna gain, emissions and finished hardware must still be checked. This mode is not the old 32-byte/7.2% example or a generic 868 MHz 1% mode.

## 7. Concept of operations

| Phase | Intended activity | Verification dependency |
|---|---|---|
| Preparation | Obtain launch-site orthoimage, calibrate camera/extrinsics/probe, build maps/prediction tables, configure radio and ground logs | Real imagery, calibrated build and confirmed rules/channel |
| Pad waiting, up to 3 h | Linux and the Pi flight service already booted before loading; Pi continuously powered, image processing idle until PREPARE/ascent; STM32 logs and independent GNSS continue | Check heartbeat/map/camera/storage readiness before loading; rehearse endurance and enclosed 5–10 minute launch preparation; no GNSS pose seeding of TRN |
| Ascent | High-rate STM32 IMU/pressure logging and capture; retain event history; camera start policy tested | Firmware/event detection and shock qualification outstanding |
| Ejection and inflation | Trigger camera; log full-field frames and STM32 trigger times; establish swing-averaged tilt reference | Settling/event definitions and frame association measured |
| First fix | Search unknown heading/position, refine pose and confirm agreement over two distinct images | T1 clock starts at defined canopy settling; no promise of a fixed three-image sequence |
| Tracking above 80 m | Fine matcher -> 17-state filter; selected prediction/validity summary to STM32; 1 Hz primary-priority telemetry | Fine matcher, handoff and integrated application remain open |
| Lower altitude | Retain valid available estimates and conservative prediction; proposed visual odometry tested before use | Below-80 m vision is a development item, not an existing switch-over capability |
| Touchdown/recovery | Detect/log candidate touchdown; beacon; save ground records before attempting recovery | Accelerometer/contact and independent GNSS/time evidence cross-checked |
| Analysis | Synchronise records; score primary, acquisition, tracking, wind and prediction; label missing/invalid observations | Raw logs retained; simulation truth never substituted for missing flight measurements |

## 8. System architecture

### Hardware/data path

```mermaid
flowchart TD
  BAT[Protected 2S pack / master switch / protection] --> AON[Independent 3.3 V supply]
  BAT --> PI5[5.1 V buck / controlled Pi load switch]
  AON --> MCU[STM32F405 on MAIN]
  AON --> RF[Top RF: MAX-M10S and E22]
  AON --> TEMP[Quiet 3.0 V: external NTC / OPA333 / ADS1115]
  IMU[BMI088 and MS5611 on MAIN] --> MCU
  TEMP --> MCU
  RF -->|GNSS PVT and PPS; radio interface| MCU
  MCU --> NOR[Independent SPI NOR sensor/event log]
  PI5 --> PI[Side-mounted Pi Zero 2 W]
  MCU -->|Buffered UART 230400; captures/integrals| PI
  PI -->|Selected navigation/prediction and heartbeat| MCU
  MCU -->|Level-shifted XTR; TIM2 capture| CAM[Bottom IMX296 GS camera]
  CAM -->|Zero-specific CSI cable| PI
  PI --> SD[256 GB microSD: images / maps / processed log]
  MCU -->|40-byte primary-priority packet via E22| GROUND[Receiver / laptop / graph and map logs]
```

### Primary software path

```mermaid
flowchart LR
  S[MS5611 and external NTC readout] --> C[STM32: calibration / compensation / timestamps]
  C --> L[Independent sensor log]
  C --> P[1 Hz packet: temperature and pressure]
  P --> R[E22 radio]
  R --> G[Ground decode / CRC / timestamps / save]
  G --> A[Pressure and temperature graphs / altitude analysis]
```

### Secondary software path

```mermaid
flowchart TD
  MAP[Prepared site map and calibrated optics] --> ACQ[Coarse heading/position search / pose solve / confirmation]
  IMG[CSI frames linked to STM32 trigger times] --> ACQ
  ACQ -->|Confirmed first pose; handoff to implement| NAV[17-state ESKF]
  IMG --> MATCH[Fine matcher: integration outstanding]
  MAP --> MATCH
  MATCH --> NAV
  I[STM32 cumulative IMU integrals and pressure] --> NAV
  NAV --> PRED[TRN wind / landing predictor]
  NAV --> LOG[Pi navigation/quality log]
  PRED --> LOG
  LOG --> SCI[Post-flight smoothing / S1 and conditional S2-S4]
  GNSS[Separate GNSS PVT reference] --> COMP[GNSS comparison predictor and truth log]
  COMP --> SCI
  PRED --> TX[Selected summary to STM32 radio packet]
```

The GNSS branch is separate from TRN acquisition/tracking/propagation. PPS may synchronise the common timebase. The published Python modules are candidate components, not an integrated flight application.

## 9. Map preparation

Use recent, permitted orthoimagery covering the launch site and plausible drift region. Check georeferencing against independent surveyed points and report datum bias separately. The existing acquisition code uses a **1 m/px map**, with decimated edge-strength views at 2/4 m/px inside its search; the old 2 m/px coarse + 0.5 m/px fine pyramid is not its current default.

The default 1024 m acquisition window only covers roughly 250 m offset from its centre under the study's pose/tilt assumptions. A proposed 1280 m window for 300 m offsets needs memory/time validation. Map extent, permissions, preparation time and real seasonal imagery remain launch-site gates. Select stable boundaries/hedgerows/tracks; corner-based matching is weak on large fields and repetitive tramlines.

## 10. Map-relative localisation

### Rectification and optics

Calibrate the physical lens and camera-window assembly, then undistort/project rays onto the modelled ground surface using estimated altitude and attitude. A ground-plane homography is valid for pinhole/rectified rays; it is not directly valid on raw barrel-distorted images. The ideal pinhole used in the current simulations is a convenient model, not a measured lens calibration. Nominal geometric HFOV is 90.3 degrees; roughly 110-115 degrees horizontal for the actual lens is an inference in the parameter-source study, still to be measured.

### Acquisition implementation

`code/cansat/coarse.py` searches the whole view over position and unknown heading, matches corner-like landmarks, solves an interim pose, and refines using up to 48 patches on the 1 m/px map. `Acquirer` confirms two distinct images through gyro propagation; competing distant explanations cause refusal. A default unknown-heading attempt needs many FFTs. Optional calibrated LIS3MDL can reduce heading-search cost; it is not required by the code or assumed installed.

The [first-fix study](acquisition_first_fix.md) reports approximately 91% per-image availability on synthetic mixed farmland and 41% on synthetic large-field terrain, with about 9% of returned large-field fixes over 10 m. The broader run predates the last review fixes; a smaller final-code rerun is separate. The default 5 s target therefore depends on real launch-site imagery and timing. No real imagery or Pi acquisition timing has been demonstrated.

### Tracking and low altitude

A flight fine matcher must hand calibrated landmark observations to the navigation filter and meet search-window/availability budgets. The earlier 50-landmark/5 Hz description is an ambition, not established throughput; reduced landmark/cadence options remain profiling tradeoffs. The simulation fine matcher is not yet a deployed flight matcher. Below 80 m, frame-to-frame visual odometry is proposed but unimplemented; report availability and propagate uncertainty instead of asserting a successful switch.

## 11. Navigation filter

`code/cansat/eskf.py` implements a **17-error-state** candidate filter: position, velocity, local/right attitude error, accelerometer/gyro biases, barometer bias and camera time offset. It uses body-to-world rotations, east/north/up world coordinates from the launch pad, and body z through the camera boresight. The ideal level attitude is diag(1,-1,-1). Landmarks jointly constrain pose; pressure constrains relative height with calibrated datum/terrain information.

The hardened configuration includes iterated landmark updates, post-fit rejection, cautious prior recovery, rate-dependent gyro noise and correlated-map-error allowances. The plain 15-state form retained in Appendix H is the mathematical starting model, not the selected implementation. Tunings are synthetic-data choices requiring real-data calibration. A majority of coherent false matches can evade innovation checks.

Under parachute swing, instantaneous specific force points largely along the suspension line and is not a reliable gravity/tilt reference. Gyro integration and image geometry provide attitude information; swing-averaged force supports initialisation only within its assumptions. Landmarks spread across a calibrated wide field help separate tilt from translation. Pressure resolution does not remove static-port bias; GNSS does not enter the TRN filter.

First-fix confirmation has not yet been handed into the filter in a complete image-in-the-loop flight sequence. The [navigation study](navigation_closed_loop_simulation.md) establishes synthetic component behavior, not complete onboard TRN.

## 12. Error budget and uncertainty

The first-order design model is

$$
\sigma_p^2 \approx (h\sigma_\theta)^2 + \frac{(g\sigma_c)^2}{n} + \sigma_{map}^2 + (h\dot\theta\delta t)^2 + (\delta z\tan\phi)^2.
$$

Here g is map spacing (not gravitational acceleration), n the number of usable independent landmarks, and phi the off-nadir ray angle. Reused map errors, coherent mismatches, static-port bias and calibration errors need explicit covariance/bias treatment; an independent-term sum cannot guarantee integrity.

| Example term | Illustrative assumption | At 300 m | At 100 m |
|---|---|---|---|
| Attitude | 0.3-1 degree residual | 1.6-5.2 m | 0.5-1.7 m |
| Matching | 1 m/px, 0.2 px, 12 independent landmarks | 0.058 m | 0.058 m |
| Map datum | Assumed 2-3 m, to measure | 2-3 m | 2-3 m |
| Timing | 40 degrees/s, 0.5 ms | 0.105 m | 0.035 m |
| Relief | 2 m height error, approximately 27 degrees off nadir | Approximately 1 m | Approximately 1 m |

These are sizing examples, not the calibrated current-build covariance. The navigation study's approximately 0.6 m synthetic RMS is relative to its map and scenario; adding an unknown georeferencing bias or real-lens errors changes absolute performance. The first-fix study has a different terrain/acceptance scope and is not a common end-to-end accuracy claim.

Use held-out real data to tune uncertainty, compare achieved pose covariance with geometry-based bounds, and score horizontal position/velocity NEES plus protection-level exceedance. Hardware edge capture targets <=0.5 ms, but exposure centre, sensor group delays and frame association must all be included. Exposure near 0.5 ms is a candidate: angular blur depends on focal/ray geometry and spin; short exposure does not guarantee acceptable images in launch-day light.

## 13. Landing prediction

`code/cansat/landing.py` runs on the Pi, using precomputed drift tables and `windprofile.py` for bounded streaming updates. It returns landing position/time and covariance. Tables must be built using the measured descent/canopy model; density rescaling is an approximation, not an exact transfer of every old scenario.

T3 compares separately labelled TRN, GNSS and inertial-only predictors. One selected prediction is available in the 40-byte radio packet; full comparison records stay onboard or are reconstructed after flight. A C port would be needed for prediction to survive Pi loss; none exists. Flight GNSS delivery over the serial reference is defined, but firmware integration is outstanding.

The end-to-end wind study shows that glide can bias wind/prediction and reduce ellipse coverage. A calibrated airspeed or glide model is conditional work, not a guaranteed remedy. Score common-height error, time error, bias and interval coverage against independent touchdown evidence rather than claiming an ellipse must monotonically shrink.

## 14. Safe-target selection — stretch

If core integration succeeds, use a preloaded hazard map and navigation covariance to choose a low-cost target in a **notional** reachable region. The hardware has no steering actuator. A hypothetical divert radius is solely a decision-demo parameter; it does not describe actual reachability of the passive can. Evaluate target safety/agreement/stability offline first; any live summary needs a defined additional packet/airtime allocation before transmission.

## 15. Hardware, mechanics and budgets

| Assembly | Selected parts / function |
|---|---|
| MAIN | STM32F405; BMI088; MS5611; ADS1115/OPA333/150 kOhm temperature readout; independent converters/quiet LDO; Pi switch/buffers; W25Q128; sounder/LEDs |
| RF | MAX-M10S and active GNSS antenna feed; E22-900M22S and FPC radio antenna |
| Bottom | Official IMX296 GS camera, Edmund lens/adapter/lock nut; CSI directly to Pi, XTR from STM32 |
| Side | Pi Zero 2 W, Zero CSI cable and 256 GB microSD; buffered UART and switched power from MAIN |
| Power / harness | Protected KeepPower 2S pack, accessible E-Switch master switch, XT30 harness, GH connectors, selected MAIN J4 Wuerth 61202021621; compatible RF endpoint, IDC mates and ribbon cable still to select/verify |
| Mechanics / recovery | Enclosure/frame, optics/window/probe shield, fasteners, cable retention, passive canopy/lines/swivel/hardpoint: not completely designed or qualified |

The [BOM](flight_computer_bom.md) retains exact purchasing identities and dated stock/price evidence. The intended top-RF/middle-MAIN/bottom-camera/side-Pi layout is a packaging concept, not a proven 66 x 115 mm fit. MAIN's saved outline is 44 x 64 mm; RF outline/enclosure/access need completion. Candidate frame/shell polymers and window materials require mass/heat/strength/RF/optical justification. A metal wall would obstruct the selected internal FPC antenna.

The [latest budget](mass_power_budget.md) models about 335 g expected total with 6-15% chance above 350 g under stated uncertainty assumptions, including the parachute. The previous approximately 20 g mission addition is withdrawn. The approximately 100 g pack, 34 g camera, lens/adapter and structure all count. Weigh the actual flight configuration and retain ballast only if needed.

Minimum nominal pack energy is about 23.76 Wh; the budget's idle-pad/flight/beacon scenario uses roughly 4.3 of 19.6 Wh usable. This does not verify three-hour flight-load endurance. Measure temperature-dependent whole-system power, startup transients, radio bursts and mode policies. Pi shutdown/UART isolation protect the primary branch, while common failures remain explicit.

Full-frame 5 Hz capture needs about 9.9 MB/s tightly packed RAW10 or 15.8 MB/s in 16-bit storage before padding/metadata. A 30-40 s descent is approximately 0.30-0.40 GB packed or 0.48-0.63 GB unpacked. Three hours of unpacked full-rate tests is about 171 GB; 256 GB is the selected storage size, with actual sustained Pi writes still to verify.

The dated mandatory IC/module subtotal, about $45.49 before support parts/assembly, is not the flight cost. Lens/adapter, camera, battery, both PCBs, assembly charges, mechanics, harnesses, VAT and delivery need a complete landed budget. The old £50-70 camera/lens estimate is withdrawn. No current flight-cost compliance is claimed.

## 16. Software and implementation status

| Function | Target / candidate implementation | Status |
|---|---|---|
| STM32 sensor/timing/power/radio firmware | C/C++; captured sensor/PPS/XTR timing, watchdog, independent logging/1 Hz telemetry | Integrated candidate in `code/firmware/stm32`: host tests, SITL and linked Cortex-M4F image ([flight software report](flight_software.md)); no hardware execution |
| Pi sensor/time link | `link.py`, cumulative IMU differencing, trigger association; 230400 baud | Python reference, C twin and Pi service (`code/piapp`) against golden vectors; host only |
| First fix | `coarse.py`, `firstfix.py`; 1 m/px map and two-image confirmation | Synthetic validation; Pi timing and real terrain outstanding |
| Tracking/navigation | Fine matcher + `eskf.py`; nominal 100 Hz propagation, image updates as budget allows | Fine matcher, handoff and live orchestration (`navigator.py`) integrated; rendered synthetic images only, Pi timing unmeasured |
| Wind / landing | `windprofile.py`, `landing.py`; selected predictor to STM32 | Synthetic component/closed-loop studies; hardware untested |
| Radio/ground data | `telemetry.py` defines 40-byte packet | Firmware builds and schedules it (host/SITL); RF link and ground UI untested/outside the onboard baseline |
| Capture/logging/ground UI | Camera setup, bounded queues, save raw frames/quality; laptop graphs/maps | Pi camera/service/SD writer implemented and host tested; sustained writes and ground UI unverified |
| Low-altitude odometry / T4 | Optional future modules | Not implemented flight capability |

Python/NumPy is the current candidate navigation stack. OpenCV or a compiled implementation may support calibrated image processing after profiling; they are not evidence of target execution. Capture cadence, matcher latency, filter update timing and logging throughput must be budgeted independently. Temperature readout: the study's 128 SPS/10 Hz paired-read bench schedule (~19.41% duty) exceeded the <=6% excitation goal; the firmware instead uses 8 Hz pairs at 860 SPS (5.0048% calculated duty, [design baseline](design_baseline.md)). Settling and self-heating still need hardware verification.

## 17. Verification campaign

| Gate | Work | Pass evidence / failure response |
|---|---|---|
| Baseline/document review | Current rules, cost/fit sketches, materials, interfaces and mission thresholds | Resolve contradictions; keep unverified claims labelled |
| Bench instruments | Probe air-step/calibration, pressure-port tests, camera focus/extrinsics/XTR timing, IMU synchronisation | Measured response/noise/timing; revise readout/mounting/calibration on failure |
| Target compute/link | Acquisition/fine matcher/filter/logging on Pi; STM32 codec against golden vectors; serial loss/reboot tests | Timestamped worst-case timing and memory, no corrupted primary stream; reduce load before revising any target |
| Real imagery / integrated replay | Launch-like seasonal imagery -> acquisition -> filter -> predictor with separate GNSS branch | Score T1/T2 availability, false fixes and consistency; use offline reconstruction if live gate fails |
| Recovery/dynamics | Canopy proof load, measured settled descent, swing/glide and side-force tests | 50 N load path, appropriate speed and calibrated motion; redesign/retest before flight |
| Integrated rehearsal | Representative power duration, reset faults, full radio ground records and operations/recovery | Primary 1 Hz mission remains independent of Pi; conserve logs; postpone stretch work if integration fails |

Captive-carry/drop locations, equipment, permissions and supervision are not yet secured. Test plans do not imply an authorised aerial operation or that short-drop average velocity equals terminal speed.

| Provisional requirement | Verification | Failure consequence |
|---|---|---|
| 1 Hz primary telemetry and graphs | Calibrated sensor-to-radio-to-ground timed run | Prioritise primary; repair scheduling/receiver pipeline |
| 66 x 115 mm envelope / no pre-ejection protrusion | Full stowed assembly gauge and access inspection | Repackage/rebuild mounts |
| 300-350 g | Complete flight weighing, including conservatively counted parachute | Trim/add retained ballast and recalibrate canopy |
| Safe materials / passive reusable recovery | Materials review and repeated packing/deployment | Substitute parts/materials |
| 3 h powered / accessible battery and master switch | Representative duration and field access test | Reduce load/revise power or access within mass/cost limits |
| 50 N attachment / 20 g launch | Suitable supervised load-path and functional checks | Reinforce/retain/retest |
| Descent around 10-15 m/s, minimum should be 10 | Settled-flight measurement at flight mass and recorded conditions | Resize canopy; update trajectory/science models |
| Recovery not guaranteed | Save ground data and rehearse lost-can analysis | Results limited to received data |
| £400 flight cost | Complete dated landed quotes and cost ledger | Substitute/simplify before purchase |
| Changeable frequency / compliant radio use | Configuration, total airtime and radiated-power checks; organiser allocation | Change radio mode/configuration before transmitting |

## 18. Risks and degradation

| Risk | Consequence | Mitigation / fallback |
|---|---|---|
| Pi throughput/logging overload | Missed image updates or lost imagery | Target profiling, bounded queues, tile maps; keep 5 Hz capture if feasible, separately budget processing; offline reconstruction |
| Large fields / seasonal changes / coherent aliases | No fix or wrong fix | Real site imagery, edge structure, ambiguity refusal and multi-frame checks; report failures rather than force a solution |
| Lens distortion / tilt / timing | Position bias and false precision | Calibrated rays/extrinsics, hardware captures, optical timing tests; withhold unreliable fixes |
| Canopy glide and swing | Shared false-wind bias; prediction undercoverage | Measured dynamics; calibrate any lateral-force correction; qualify S1 as method agreement |
| Temperature lag / self-heating / radiation | Invalid lapse-rate/stability interpretation | Installed air-step and environmental tests; restrict/withdraw S2 claims if unidentifiable |
| Incomplete firmware/fine matcher/handoff | Components never become a live mission | Integration gates and protected primary schedule; cut T4/S3 first |
| Mass/cost/fit margins | Rule failure or late redesign | Complete mechanical/landed budgets, weigh prototypes, freeze interfaces before manufacture |
| Radio/antenna/ground support | Lost primary data or uncertain range | Configure actual permitted mode, finished antenna tests, offline ground logging and backups |
| Staff skills/time/lead times | Delayed integration and weak report evidence | Real student task ownership, skill gaps/support route, weekly review and procurement contingency |
| Site/rules/channel/weather change | Invalid map or launch/test plan | Confirm issued documents, alternative test data/site coverage; report scope honestly |

Degradation: live TRN/prediction -> live TRN only -> logged imagery/offline reconstruction -> separately labelled GNSS analysis -> received primary data. Every branch requires usable inputs; recovery and science are not guaranteed.

## 19. Results analysis

Retain raw images, captures, sensor/GNSS streams, received radio packets, settings/calibration and test conditions. Never merge synthetic ground truth into flight evidence. Align clocks using the documented captures; score valid observations and rejected/missing samples separately.

| Objective | Data / calculation | Validity and reported output |
|---|---|---|
| Primary | Temperature/pressure vs time; pressure-derived relative altitude | Calibration, gaps, units, radio delivery; air temperature separated from die temperature |
| T1 | Horizontal TRN-GNSS error, acquisition time, availability/false-fix rate | GNSS quality, map datum, camera geometry and stated settling event |
| T2 | Position/velocity NEES and protection-level exceedance by regime | Independent/held-out calibration data, serial/temporal correlation and failed updates |
| S1 | Image/GNSS wind difference per axis/vector in 20 m layers over 80-300 m | Same motion corrections; averaging kernels/coverage; common bias caveat |
| S2 | Wind/temperature profile, similarity fit and roughness uncertainty | Ground reference height/wind, installed thermal response and fit identifiability; report inconclusive cases |
| T3 | Common-height landing/time errors and ellipse coverage for three sources | Same model/settings, separately labelled source branches, independent touchdown evidence |
| S3/S4 | Motion/shear association and candidate turbulence-model comparison | Exploratory record length, canopy confounding and prior sensitivity |
| T4 | Hazard-map target quality/stability | Notional decision region; no physical divert achievement |

## 20. Roles and support

| Workstream | Responsibilities / interfaces |
|---|---|
| TRN | Site map, calibration, acquisition/fine matcher, Pi timing; pose/quality handoff to navigation |
| Navigation/science | Filter/uncertainty, wind/prediction, separate reference stream and analysis; motion/probe calibration requirements to systems |
| Systems | STM32 firmware, power, radio, sensors, mechanics/recovery, serial/timestamp integration and ground equipment |
| Planning/documentation/outreach | Weekly plan, cost/risk/compliance records, report evidence, audience-specific activities |

These are workstreams, not invented student assignments. The actual team must map Student identifiers, skills, gaps and agreed support to them for the PDR. Required support routes include radio/embedded integration, mechanical load testing and atmospheric/thermal calibration; no mentor commitment is asserted.

## 21. Schedule and decision gates

The old monthly chart carried unconfirmed dates and H618 work. Use the following dependency order to build the team's weekly Gantt; student owners, effort and issued deadlines must be supplied from real availability. This proposal does not invent a current competition date.

1. Confirm rules/site imagery/support; document baseline, packaging and landed cost.
2. Bench-qualify camera/probe/power/timing and implement STM32 reference protocol/primary telemetry.
3. Profile Pi, finish fine matcher and first-fix/filter handoff; integrate image/sensor/radio logging.
4. Replay real imagery, run dynamic/recovery tests and calibrated descent measurements; revisit science feasibility.
5. Rehearse complete mission and primary fault survival; freeze tested configuration and prepare report evidence.
6. Perform confirmed launch operations, preserve logs, analyse results and execute scheduled outreach.

Outreach/report work runs alongside all technical phases. Preserve retest/procurement time. Lower-priority features are dropped at failed gates; targets are not silently changed to pass.

## 22. PDR and outreach evidence

The [full-mark checklist](../BOTS/PLANS/PDR_FULL_MARKS_CHECKLIST.md) maps the supplied 36-point PDR scheme. The diagrams here support both mission software paths, but this working proposal is not a final anonymous <=20-page PDR. A mechanical sketch, justified materials, actual team/skills/support, weekly Gantt, grounded outreach schedule and complete verification plan must be assembled into the issued template.

Proposed outreach activities are a younger-pupil terrain-matching demonstration, a school primary-telemetry/engineering presentation, and a community descent-map/wind story with uncertainty explained. Choose audiences/methods deliberately and record real owners, dates and feedback; these are ideas, not completed outreach or booked events. Hypotheses, units, calibrations and explanatory plots should be understandable without reproducing every equation in the main PDR.

## 23. Outstanding decisions and evidence

- Confirm current-season rulebook/rubric/deadlines, regional/final launch sites, channel allocation and parachute mass treatment.
- Obtain representative orthoimages and check permissions/datum/extent; nearest atmospheric reference sources remain to be selected.
- Prove selected lens/adapter infinity focus, retention, envelope and calibrated ray mapping.
- Measure Pi capture/matching/logging worst-case performance and bring up the integrated candidate firmware/Pi software on the assembled hardware.
- Choose/qualify passive canopy and complete mechanical/material/access design; weigh full assembly and obtain delivered flight quotes.
- Verify the selected 8 Hz / 860 SPS thermistor schedule (5.0048% calculated duty) for settling, installed response and science precision; no TMP117 substitution without redesign.
- Secure/select the ground reference instrument/support and calibrate any proposed lateral-force airspeed correction.
- Test GNSS receiver dynamics and independent touchdown scoring; a GNSS feed defined by the protocol is not yet integrated hardware.
- Agree actual student ownership, available hours, support and outreach dates.

## Mathematical appendices: interpretation for the current design

The following appendices retain the mathematical methods and illustrative models from the earlier proposal so equation reasoning remains traceable. Hardware-specific September settings and 10.5 m/s numerical examples are historical illustrative inputs, not current measured parameters. In particular, ideal pinhole rays must follow physical-lens calibration; the 15-state derivation is extended to 17 states in code; drag-only wind inversion omits real-can glide; known-parameter retrievals are special cases of the implemented uncertain model. The [standalone mathematical reference](cansat_mathematics.tex), [audit](mathematics_simulation_audit.md) and newer studies state restrictions and evidence. Their original scenario results are preserved; updating this proposal does not rerun them.

---

## Appendix A: Why the rectification is a homography

For points on the plane $Z = 0$, the third column of $R$ multiplies zero, so the projection $K[R\,|\,\mathbf t]$ collapses to the $3 \times 3$ matrix $K[\mathbf r_1\ \mathbf r_2\ \mathbf t]$: a homography between the ground plane and the image. Rectification samples the image through this homography evaluated at the predicted pose; a pose error appears as a residual warp between rectified image and map, which the landmark updates correct.

## Appendix B: Timing requirement

A timestamp error $\delta t$ during a swing at rate $\dot\theta$ assigns the image an attitude wrong by $\dot\theta\,\delta t$, moving the projected nadir by $h\,\dot\theta\,\delta t$. Requiring this to stay under 1 m at $h = 300$ m and $\dot\theta = 0.7$ rad/s gives $\delta t \lesssim 5$ ms.

## Appendix C: Separating tilt from translation

For a nadir-pointing camera with focal length $f$ in pixels, a ground point seen at angle $\alpha$ from the optical axis has image coordinate $u = f\tan\alpha$.

- A small camera rotation $\delta\theta$ in the plane of $\alpha$ gives $u' = f\tan(\alpha - \delta\theta)$, so $\delta u \approx -f\sec^2\!\alpha\;\delta\theta$.
- A horizontal translation $\delta x$ at height $h$ over flat ground gives $\delta u = -f\,\delta x/h$, the same for every point.

Matching the image-centre shifts requires $\delta x = h\,\delta\theta$, leaving a residual pattern

$$
\delta u_{\text{rot}} - \delta u_{\text{trans}} = -f\,\delta\theta\,\tan^2\!\alpha,
$$

which is the only information separating tilt from translation. It vanishes at the image centre and equals the full shift at $\alpha = 45°$. The Fisher information for tilt, given translation, therefore scales with the spread of $\tan^2\!\alpha$ across the landmarks, which is the quantitative case for a wide field of view and landmarks spread to the image edges.

## Appendix D: Protection level

With horizontal error covariance $\Sigma_p$ and largest eigenvalue $\lambda_{\max}$, the protection level $\text{PL} = k\sqrt{\lambda_{\max}}$ bounds the error with probability set by $k$ under a Gaussian model. Integrity is demonstrated empirically by the fraction of epochs in which the true error, measured against GPS, exceeds PL.

## Appendix E: Mathematical methods index

Mathematical methods retained from the September proposal, with current implementation extensions described in Sections 10-13. Original section references are indicative; use the live sections for current design status. **Core** is needed for the flight results; **Rigour** strengthens T2 and S1–S4 but is post-flight analysis; **Stretch** is done only if time allows; **Open** is unproven.

| Topic | Used for | Where used | Derived in | Status |
| --- | --- | --- | --- | --- |
| Pinhole camera; ground-plane homography | Rectifying descent images onto the map grid | Sec. 10; T1 | App. A | Core |
| Normalised cross-correlation via FFT; sub-pixel peaks; peak covariance | Landmark matching | Sec. 10; T1 | App. G | Core |
| Consensus (RANSAC) | Rejecting aliased matches (tramlines) | Sec. 10 | App. G | Core |
| Terminal velocity; hypsometric altitude | Descent-rate model; barometric height | Secs. 3, 11, 13 | App. F | Core |
| Lag length and inversion conditioning | Recovering wind from drift | Sec. 3; S1 | App. F | Core |
| Pendulum specific force | Why the accelerometer is blind to swing | Sec. 11 | App. F | Core |
| 15-state ESKF derivation; attitude and reprojection Jacobians (17 states in current code) | Navigation filter | Sec. 11; T1 | App. H | Core |
| $\chi^2$ innovation gating | Rejecting bad landmarks | Sec. 11 | App. H | Core |
| Tilt–translation separability | Lens choice; landmark placement | Sec. 11 | App. C | Core |
| Error budget; crossover altitude; timing requirement | Illustrative design reasoning; T2 uncertainty assessment | Sec. 12; T2 | Apps. B, H | Core |
| NEES; Cramér–Rao bound; protection levels | Filter consistency and integrity | Sec. 12; T2 | Apps. D, H | Rigour |
| Shear parabola; frozen-field ratio | Justifying a turbulence model for a vertical path | Sec. 3; S4 | App. F | Rigour |
| Drop-Dryden turbulence model | Wind prior; the S4 test | Secs. 3, 13; S1, S4 | App. I | Core |
| State-space Gaussian processes; Kalman smoothing | Wind retrieval; landing prediction | Secs. 3, 13; S1, T3 | App. J | Core |
| Averaging kernels; degrees of freedom for signal | Retrieval resolution and information content | S1, S4 | App. J | Rigour |
| Mahalanobis distance; $\chi^2$ calibration | Is the landing ellipse honest? | T3 | App. J | Core |
| Marginal-likelihood model selection | Choosing the turbulence model; testing S4 | S4 | App. J | Rigour |
| Potential temperature; Monin–Obukhov similarity; bulk Richardson number | Boundary-layer science | S2 | App. K | Core |
| Pendulum as a turbulence sensor | Swing response to shear | S3 | App. F | Stretch |
| Landing-uncertainty shrinkage | Whether the ellipse must shrink monotonically | T3 | App. J | Open |
| Gauge freedom of the map datum | Separating map error from navigation error | T1; Sec. 9 | App. L | Rigour |
| Certifiable registration (truncated least squares with SDP relaxation) | Globally optimal TRN fixes with a certificate | Sec. 10 | App. L | Stretch |
| Submodular landmark selection | Choosing landmarks with a performance guarantee | Sec. 10 | App. L | Stretch |
| Uncertainty-aware target selection | T4 | Sec. 14 | Sec. 14 | Stretch |

Notation: $L(z)$ is the turbulence length scale, $L_p$ the suspension-line length, and $L_O$ the Obukhov length.

## Appendix F: Descent and pendulum physics

**F.1 Descent rate.** Vertical force balance gives the terminal descent rate, and density follows from the ideal gas law using the primary mission's pressure and temperature:

$$
v_d = \sqrt{\frac{2mg}{\rho\,C_D A}}, \qquad \rho = \frac{p}{R_d T}.
$$

**F.2 Altitude from pressure.** Integrating hydrostatic balance with the ideal gas law over a layer of mean temperature $\bar T$ gives the hypsometric equation:

$$
z_2 - z_1 = \frac{R_d\,\bar T}{g}\,\ln\frac{p_1}{p_2}.
$$

**F.3 Lag length.** Linearising drag in the horizontal relative velocity about the descent speed, and using $\tfrac12\rho C_D A\,v_d^2 = mg$:

$$
m\,\dot u = -\frac{mg}{v_d}\,(u - w) \quad\Rightarrow\quad \tau = \frac{v_d}{g}, \qquad \ell_\tau = v_d\,\tau = \frac{v_d^2}{g} \approx 11\ \text{m}.
$$

Inverting the lag amplifies noise at vertical wavenumber $k$ by $\sqrt{1 + (k\ell_\tau)^2}$, so structure longer than a few $\ell_\tau$ is recovered cleanly.

**F.4 The accelerometer under a pendulum.** For an ideal pendulum, line tension $\mathbf T$ is the only non-gravitational force, so the specific force measured by the accelerometer is

$$
\mathbf f = \mathbf a - \mathbf g = \frac{\mathbf T}{m},
$$

which points along the line in this ideal single-force model. If the can axis follows the line, lateral body-frame components are zero regardless of tilt, so accelerometer-derived tilt reads "level" throughout the swing. Real-can aerodynamic side force, rotation, lever arms and line/body misalignment violate that idealisation; they do not provide a reliable instantaneous gravity reference. The time-averaged lateral-force airspeed candidate in Section 3 explicitly models such additional force and needs calibration.

**F.5 Pendulum dynamics.** The swing angle obeys a driven, damped oscillator with $\omega_0^2 = g/L_p$:

$$
\ddot\theta + 2\zeta\omega_0\dot\theta + \omega_0^2\,\theta = F(t).
$$

In the small-angle, weakly forced, weakly damped ideal model, $\theta \approx -\ddot\theta/\omega_0^2$ is a conditional cross-check. Unknown forcing, damping and body rotation prevent treating gyro angular acceleration as an independent absolute-tilt measurement of the real can. For S3, the linear model's swing spectrum is the forcing spectrum filtered by the pendulum:

$$
S_\theta(\omega) = |H(\omega)|^2\,S_F(\omega), \qquad H(\omega) = \frac{1}{\omega_0^2 - \omega^2 + 2i\zeta\omega_0\omega}.
$$

**F.6 Approximate vertical sampling.** In this ideal wind-following model, each layer moves at $U(z)$ and the probe drifts with the local wind. For linear shear $U = U_0 + Sz$ and $z = z_0 - v_d t$, the sampled point's horizontal offset in the frame of the layer the probe is in is

$$
\xi = \int_0^t \big[U(z(t')) - U(z(t))\big]\,dt' = S\,v_d\int_0^t (t - t')\,dt' = \frac{S\,\Delta z^2}{2\,v_d}.
$$

For $S = 0.05\ \text{s}^{-1}$ and the historical $v_d=10.5$ m/s, a 100 m layer gives about 23.8 m offset, 24% of the layer thickness. Over a 10 m interval it gives about 0.238 m, 2.4% of that interval. The few-percent approximation is therefore local to short intervals under these assumptions, not a bound on the entire descent. Probe lag and canopy glide add further relative-air motion.

**F.7 Frozen-field condition.** Under the assumed turnover-time model, an eddy of size $L$ is crossed in $L/v_d$ and lives for about $L/\sigma$; the illustrative ratio $\sigma/v_d \approx 0.05$–$0.1$ suggests frozen-field sampling may be useful. Independence from $L$ within this model does not establish validity at every height, in strongly sheared flow or for an actual gliding probe. Test sensitivity to departures as part of S4.

## Appendix G: Landmark matching

**G.1 Normalised cross-correlation.** For template $T$ and rectified image $I$, at shift $\mathbf d$:

$$
\gamma(\mathbf d) = \frac{\sum_{\mathbf x}\big(I(\mathbf x + \mathbf d) - \bar I_{\mathbf d}\big)\big(T(\mathbf x) - \bar T\big)}{\sqrt{\sum_{\mathbf x}\big(I(\mathbf x + \mathbf d) - \bar I_{\mathbf d}\big)^2\;\sum_{\mathbf x}\big(T(\mathbf x) - \bar T\big)^2}}.
$$

The numerator for every shift at once comes from the correlation theorem, $\mathcal F^{-1}\big\{\overline{\mathcal F\{T\}}\cdot\mathcal F\{I\}\big\}$, and the local means and energies from running sums, so each search window costs $O(N\log N)$.

**G.2 Sub-pixel peak.** Fitting a parabola through the peak and its two neighbours along each axis gives the offset

$$
\delta = \frac{\gamma_{-1} - \gamma_{+1}}{2\,(\gamma_{-1} - 2\gamma_0 + \gamma_{+1})}.
$$

**G.3 Landmark covariance.** A sharp peak locates the landmark precisely and a flat one does not, so the covariance is taken proportional to the inverse curvature of the correlation surface, $\Sigma \approx c\,\big(-\nabla^2\gamma\big)^{-1}$, with $c$ calibrated on captive-carry data.

**G.4 Consensus.** To draw, with probability $p$, at least one outlier-free minimal set of size $k$ when a fraction $w$ of matches are inliers, RANSAC needs

$$
N = \frac{\ln(1 - p)}{\ln(1 - w^k)}
$$

iterations. A 2D similarity needs $k = 2$, so $w = 0.5$ and $p = 0.999$ give $N \approx 24$.

## Appendix H: Navigation filter and its analysis

**H.1 Error-state dynamics.** The nominal state (position $\mathbf p$, velocity $\mathbf v$, body-to-world rotation $R$, accelerometer and gyro biases $\mathbf b_a$, $\mathbf b_g$) is propagated with the IMU. The 15-dimensional error state uses local attitude errors, $R = \hat R\,(I + [\delta\boldsymbol\theta]_\times)$, and evolves as

$$
\begin{aligned}
\delta\dot{\mathbf p} &= \delta\mathbf v, \\
\delta\dot{\mathbf v} &= -\hat R\,[\mathbf a_m - \hat{\mathbf b}_a]_\times\,\delta\boldsymbol\theta - \hat R\,\delta\mathbf b_a - \hat R\,\mathbf n_a, \\
\delta\dot{\boldsymbol\theta} &= -[\boldsymbol\omega_m - \hat{\mathbf b}_g]_\times\,\delta\boldsymbol\theta - \delta\mathbf b_g - \mathbf n_g, \\
\delta\dot{\mathbf b}_a &= \mathbf n_{ba}, \qquad \delta\dot{\mathbf b}_g = \mathbf n_{bg}.
\end{aligned}
$$

**H.2 Landmark Jacobians.** With the map point in body coordinates $\mathbf p_b = \hat R^{\mathsf T}(\mathbf m - \mathbf p)$, camera coordinates $\mathbf p_c = R_{cb}\,\mathbf p_b = [X\ Y\ Z]^{\mathsf T}$ (camera lever arm neglected here), and $\mathbf z = \pi(K\mathbf p_c)$:

$$
J_\pi = \frac{1}{Z}\begin{bmatrix} f_x & 0 & -f_x X/Z \\ 0 & f_y & -f_y Y/Z \end{bmatrix}, \qquad
\frac{\partial\mathbf z}{\partial\,\delta\mathbf p} = -J_\pi R_{cb}\hat R^{\mathsf T}, \qquad
\frac{\partial\mathbf z}{\partial\,\delta\boldsymbol\theta} = J_\pi R_{cb}\,[\mathbf p_b]_\times .
$$

The second Jacobian is how each landmark corrects attitude as well as position.

**H.3 Gating.** With innovation $\boldsymbol\nu = \mathbf z - \hat{\mathbf z}$ and $S = HPH^{\mathsf T} + R$, a landmark is accepted if $\boldsymbol\nu^{\mathsf T}S^{-1}\boldsymbol\nu < \chi^2_{2}(0.997) = -2\ln 0.003 \approx 11.6$.

**H.4 Consistency.** The normalised estimation error squared $\varepsilon_k = \mathbf e_k^{\mathsf T}P_k^{-1}\mathbf e_k$, averaged over $N$ independent runs, satisfies $N\bar\varepsilon_k \sim \chi^2_{nN}$ under the consistent Gaussian model with known truth for the tested n-dimensional subspace. GNSS alone does not supply all-state truth; current T2 uses observable horizontal position/velocity subspaces, reference uncertainty and temporal correlation. Confidence bounds from independent runs cannot simply be applied to correlated epochs of one flight.

**H.5 Cramér–Rao bound.** For a single fix, the Fisher information from $n$ landmarks is $J = J_0 + \sum_i H_i^{\mathsf T}R_i^{-1}H_i$, and any unbiased estimator satisfies $\operatorname{Cov}(\hat{\mathbf x}) \succeq J^{-1}$. Comparing achieved error with $J^{-1}$ for the actual landmark geometry shows how close the system is to the best possible.

**H.6 Error budget.** The dominant terms (Section 12) cross at

$$
h^* = \frac{\sigma_{\text{map}}}{\sigma_\theta},
$$

~290 m for $\sigma_\theta = 0.5°$ and $\sigma_{\text{map}} = 2.5$ m, if those independent constant-error assumptions hold. This is a historical sizing diagnostic: the newer closed-loop study did not reproduce the predicted crossover, so reading a crossover from a flight curve is not an established way to infer attitude accuracy and is not a current T2 requirement. The illustrative under-1 m timing bound is $\delta t \lesssim 1\ \text{m}/(h\dot\theta) \approx 5$ ms (Appendix B); the selected alignment target is <=0.5 ms, including delay/jitter calibration.

**H.7 Visual odometry.** Below ~80 m, after removing rotation with the gyro, horizontal velocity from the image shift $\Delta u$ of a nadir camera over flat ground is $v \approx (h/f)\,\Delta u/\Delta t$.

## Appendix I: Turbulence for a falling probe (drop-Dryden model)

The Dryden model describes turbulence for an aircraft flying horizontally through a frozen field. This appendix derives the version for a probe falling vertically through the boundary layer.

**I.1 Assumptions checked.** The sampling path through the air is vertical to within a few percent (F.6), and the frozen-field assumption holds at every height (F.7).

**I.2 Which spectrum each component gets.** With a vertical path, vertical wind is the *longitudinal* component and both horizontal components are *transverse*, the opposite of the aircraft case. For isotropic incompressible turbulence with longitudinal correlation $f(r) = e^{-r/L}$, the transverse correlation is

$$
g(r) = f(r) + \frac r2\,f'(r) = \Big(1 - \frac{r}{2L}\Big)e^{-r/L},
$$

so the retrieved horizontal wind uses $g$.

**I.3 Eddies that grow with height.** Near the ground, eddy size scales with height, so the process is not stationary in $z$. Define the eddy-scaled coordinate and covariance

$$
\zeta(z) = \int^{z}\frac{dz'}{L(z')}, \qquad R(z_1, z_2) = \sigma(z_1)\,\sigma(z_2)\;g\big(|\zeta(z_1) - \zeta(z_2)|\big),
$$

with $g$ now in units of one correlation length. $R$ is positive definite: $g$ is a valid kernel on the line, composing it with the monotone map $\zeta$ preserves that, and multiplying by $\sigma(z_1)\sigma(z_2)$ is a Schur product with a rank-one positive kernel.

**I.4 Special cases.**

- Constant $L$: $\zeta = z/L$, ordinary Dryden.
- Attached eddies, $L = az$: $\zeta = \frac1a\ln z$, so correlation depends on the ratio of heights.
- Saturating, $L = \dfrac{az}{1 + az/L_\infty}$: $\zeta = \frac1a\ln z + \dfrac{z}{L_\infty}$, logarithmic near the ground and linear aloft.

**I.5 State-space realisation.** In $\zeta$, with unit white noise $\xi$:

$$
\frac{d}{d\zeta}\begin{bmatrix}x_1\\ x_2\end{bmatrix} = \underbrace{\begin{bmatrix}0 & 1\\ -1 & -2\end{bmatrix}}_{F}\begin{bmatrix}x_1\\ x_2\end{bmatrix} + \underbrace{\begin{bmatrix}0\\ 1\end{bmatrix}}_{B}\xi, \qquad \tilde w = \sigma\,(x_1 + \sqrt3\,x_2).
$$

The Lyapunov equation $FP + PF^{\mathsf T} + BB^{\mathsf T} = 0$ gives $P = \tfrac14 I$, so $\operatorname{Var}(\tilde w) = \sigma^2(\tfrac14 + \tfrac34) = \sigma^2$. With $e^{Fr} = e^{-r}\begin{bmatrix}1 + r & r\\ -r & 1 - r\end{bmatrix}$,

$$
\operatorname{Cov}\big(\tilde w(\zeta), \tilde w(\zeta + r)\big) = \frac{\sigma^2}{4}\,[1\ \ \sqrt3]\,e^{Fr}\,[1\ \ \sqrt3]^{\mathsf T} = \sigma^2\Big(1 - \frac r2\Big)e^{-r},
$$

exactly $g$. Two states per horizontal axis.

**I.6 Time-varying form.** Since $\dot\zeta = v_d/L(z(t))$, and unit white noise in $\zeta$ is $\sqrt{\dot\zeta}$ times unit white noise $\eta$ in time,

$$
\frac{d\mathbf x}{dt} = \frac{v_d}{L(z)}\,F\,\mathbf x + \sqrt{\frac{v_d}{L(z)}}\,B\,\eta(t).
$$

The filter runs faster near the ground, where the eddies are smaller.

**I.7 Intensities.** Two independent priors for $\sigma(z)$: the MIL low-altitude model, driven by the wind $W_{20}$ at 6 m (20 ft),

$$
\sigma_w = 0.1\,W_{20}, \qquad \frac{\sigma_u}{\sigma_w} = \frac{1}{(0.177 + 0.000823\,h)^{0.4}} \quad (h \text{ in feet}),
$$

and neutral surface-layer similarity, $\sigma_u \approx 2.4\,u_*$, $\sigma_v \approx 1.9\,u_*$, $\sigma_w \approx 1.25\,u_*$. Their agreement or disagreement is itself part of S4.

**I.8 Predictions.**

1. *Equal information per factor of height.* With $L = az$, the number of correlation lengths between $z$ and $2z$ is $\ln 2/a$, independent of $z$: as many independent wind samples between 10 and 20 m as between 100 and 200 m.
2. *Ratio scaling.* In the surface layer, correlation depends on $z_2/z_1$, not $z_2 - z_1$ (S4).
3. *A floor from the probe's lag.* Eddies shrink to the lag length at $z \approx \ell_\tau/a$; below that (roughly 20–40 m) the retrieval becomes prior-dominated, which the averaging kernels (J.3) should show.

**I.9 Caveats.** The transverse form relies on isotropy and incompressibility; with anisotropy and warping it is a model, so it is compared against the plain exponential by marginal likelihood (J.5). $a$ depends on stability. Parameters from one descent are reported with their uncertainties.

## Appendix J: Wind retrieval, prediction and diagnostics

**J.1 Retrieval model.** In distance descended $s$ (with $d\zeta/ds = 1/L$), each horizontal axis carries the two turbulence states of I.5, the lagged velocity $u$ and a drift integrator $I$:

$$
\frac{d\mathbf x}{ds} = \frac{1}{L}F\,\mathbf x + \frac{1}{\sqrt L}B\,\xi(s), \qquad
\frac{du}{ds} = \frac{\mu(z) + \sigma(z)(x_1 + \sqrt3\,x_2) - u}{\ell_\tau}, \qquad
\frac{dI}{ds} = \frac{u}{v_d},
$$

where $\mu(z)$ is the mean profile (log law). The measurement is velocity, $y = u + \varepsilon$, from GPS Doppler or from the TRN filter; a Rauch–Tung–Striebel smoother gives the post-flight profile.

**J.2 Landing prediction.** In the ideal model, the state is copied, $I$ reset to zero, and propagated to the ground with no future velocity updates. A near-surface anemometer observation may constrain the model only if a ground reference is actually installed, logged, correctly timed and available to the relevant analysis; the proposed 6 m instrument is unselected. The implemented flight predictor uses precomputed tables and optional reference priors rather than asserting that future measurements are available. Under the linear Gaussian assumptions the landing point has propagated mean and covariance; touchdown time is

$$
t_{\text{impact}} = t_{\text{now}} + \int_0^{h_{\text{AGL}}}\frac{dh}{v_d(\rho(h))}.
$$

**J.3 Averaging kernels.** On a height grid, with forward model $\mathbf y = K\mathbf w + \boldsymbol\varepsilon$ ($K$ the discretised lag operator), noise covariance $S_\varepsilon$ and prior covariance $S_a$ from Appendix I, the retrieval gain and averaging kernel are

$$
G = \big(K^{\mathsf T}S_\varepsilon^{-1}K + S_a^{-1}\big)^{-1}K^{\mathsf T}S_\varepsilon^{-1}, \qquad A = GK = \frac{\partial\hat{\mathbf w}}{\partial\mathbf w}, \qquad \text{DFS} = \operatorname{tr}A.
$$

Each row of $A$ shows how the retrieved wind at one height averages the true profile; its width is the vertical resolution there. DFS is the number of independent wind values the descent measured, reported for image-derived and GPS-derived retrievals (S1).

**J.4 Calibration.** For predicted landing mean $\hat{\mathbf p}$ and covariance $\Sigma$, the squared Mahalanobis distance of the actual landing, $d^2 = (\mathbf p - \hat{\mathbf p})^{\mathsf T}\Sigma^{-1}(\mathbf p - \hat{\mathbf p})$, follows $\chi^2_2$ if the predictor is calibrated, so about 95% of landings should fall inside the 95% ellipses (T3).

**J.5 Model selection.** The Kalman filter gives the log marginal likelihood directly from its innovations:

$$
\ln p(\mathbf y \mid \mathcal M) = -\frac12\sum_k\Big[\ln\det(2\pi S_k) + \boldsymbol\nu_k^{\mathsf T}S_k^{-1}\boldsymbol\nu_k\Big].
$$

Models compared: exponential against transverse correlation, crossed with constant, attached-eddy and saturating $L(z)$. Hyperparameters $(a, L_\infty)$ are fitted by maximising this, and the ratio-scaling test of S4 is the comparison between the attached-eddy and constant-$L$ models.

**J.6 Open problem: does the ellipse always shrink?** Let $V(h)$ be the posterior variance of the remaining drift given data above $h$. Splitting the remaining integral into $A$ (below $h - \delta$) and a thin slab $B$, and using the fact that extra data never increases variance, a sufficient condition for $V(h - \delta) \le V(h)$ is $\operatorname{Var}(B \mid D) + 2\operatorname{Cov}(A, B \mid D) \ge 0$. As $\delta \to 0$ this becomes

$$
\operatorname{Cov}\!\Big(\int_0^h w\,dh',\ w(h)\ \Big|\ D\Big) \ge 0 .
$$

The transverse correlation $g$ goes negative beyond two correlation lengths, so this is not automatic. Proving it, or finding a counterexample, is an open question.

## Appendix K: Boundary-layer physics

**K.1 Potential temperature.** $\theta = T\,(p_0/p)^{R_d/c_p}$ with $R_d/c_p \approx 0.286$ and $p_0 = 1000$ hPa. A layer is statically unstable if $\theta$ decreases with height.

**K.2 Monin–Obukhov similarity.** In the surface layer,

$$
\bar u(z) = \frac{u_*}{\kappa}\left[\ln\frac{z}{z_0} - \psi_m\!\left(\frac{z}{L_O}\right)\right],
$$

with the Businger–Dyer forms $\psi_m = -5\,z/L_O$ (stable) and, with $x = (1 - 16\,z/L_O)^{1/4}$,

$$
\psi_m = 2\ln\frac{1 + x}{2} + \ln\frac{1 + x^2}{2} - 2\arctan x + \frac\pi2 \qquad \text{(unstable)}.
$$

**K.3 Bulk Richardson number.** Defined per layer in Section 3; negative values mean convective instability, and values above about 0.25 mean shear is too weak to sustain turbulence against stable stratification. It classifies stability and provides the starting estimate of $L_O$ for K.2.

**K.4 Fitting the profile.** For near-neutral conditions, $\bar u = \alpha\ln z + \beta$ gives $u_* = \kappa\alpha$ and $z_0 = e^{-\beta/\alpha}$. Because $z_0$ depends exponentially on the fitted intercept, its uncertainty is large, which is why S2 tests it only to an order of magnitude.

**K.5 Roughness lengths.** Typical values for comparison with land cover identified in the imagery: short grass ~0.01–0.05 m, crops ~0.05–0.2 m, farmland with hedgerows ~0.1–0.3 m, woodland ~0.5–1 m.

## Appendix L: Rigour and stretch mathematics

**L.1 Gauge freedom of the map datum.** Imagery fixes position relative to the map, so a rigid offset between map and world is unobservable from images alone. The offset is estimated separately from GPS-surveyed ground points and reported, so that map error is never mistaken for navigation error.

**L.2 Certifiable TRN fixes (stretch).** Writing rotation and scale as one complex number $z = s\,e^{i\psi}$, each landmark obeys $m_i = z\,u_i + t$, linear in $(z, t)$. Outliers are handled by truncated least squares,

$$
\min_{z,\,t}\ \sum_i \min\!\left(\frac{|m_i - z\,u_i - t|^2}{\sigma^2},\ \bar c^{\,2}\right),
$$

which becomes a quadratic programme in binary inlier variables. Its semidefinite relaxation is convex; if the relaxed solution is rank-1, it is a certified global optimum. The rank-1 ratio is logged per fix, in the same spirit as a losslessness certificate: relax, then verify the relaxation was exact. Post-flight only.

**L.3 Submodular landmark selection (stretch).** Choosing $k$ landmarks $S$ to maximise $\log\det\big(J_0 + \sum_{i \in S} J_i\big)$ is monotone submodular, so the greedy algorithm is guaranteed to reach at least $1 - 1/e$ of the optimum. Appendix C predicts greedy should favour landmarks near the image edges, which is a checkable consequence.


---

## References

- Johnson, A. E. et al. (2017). The Lander Vision System for Mars 2020 Entry Descent and Landing. AAS 17-036; see navigation parameter sources for source-access limits.
- Johnson, A. E. et al. Implementation of a map relative localization system for planetary landing. *Journal of Guidance, Control, and Dynamics.*
- Cheng, Y. et al. (2021). Making an onboard reference map from MRO/CTX imagery for Mars 2020 Lander Vision System. *Earth and Space Science.*
- Johnson, A. et al. (2007). Design through operation of an image-based velocity estimation system for Mars landing (DIMES). *International Journal of Computer Vision* 74(3).
- COBALT: terrestrial flight test of landing navigation using the Lander Vision System with Navigation Doppler Lidar. NASA Technical Reports Server.
- Maggio, D., Mario, C., Streetman, B., Steiner, T. and Carlone, L. (2023). Vision-based terrain relative navigation on high-altitude balloon and sub-orbital rocket. arXiv:2302.08011.
- Dever, C. et al. (2017). Guided-airdrop vision-based navigation. AIAA Aerodynamic Decelerator Systems Technology Conference.
- Building maps for terrain relative navigation using Blender: an open-source approach. NASA Technical Reports Server.
- Hartikainen, J. and Särkkä, S. (2010). Kalman filtering and smoothing solutions to temporal Gaussian process regression models.
- JPL Robotics. Mars 2020 Terrain Relative Navigation (architectural heritage, not CanSat performance evidence).
- Bird, M. K. et al. (2005). The vertical profile of winds on Titan. *Nature* 438, 800–802.
- Folkner, W. M. et al. (2006). Winds on Titan from ground-based tracking of the Huygens probe. *Journal of Geophysical Research: Planets.*
- Karkoschka, E. (2016). Titan's meridional wind profile and Huygens' orientation and swing inferred from the geometry of DISR imaging. *Icarus.*
- Lorenz, R. D. Wind shear and turbulence on Titan: Huygens analysis. arXiv:1704.05113.
- Stull, R. B. (1988). *An Introduction to Boundary Layer Meteorology.* Kluwer.
- MIL-F-8785C (1980). *Flying Qualities of Piloted Airplanes*; MIL-HDBK-1797 (1997). *Flying Qualities of Piloted Aircraft.* US Department of Defense.
- Townsend, A. A. (1976). *The Structure of Turbulent Shear Flow*, 2nd ed. Cambridge University Press.
- Blackadar, A. K. (1962). The vertical distribution of wind and turbulent exchange in a neutral atmosphere. *Journal of Geophysical Research* 67.
- Panofsky, H. A. and Dutton, J. A. (1984). *Atmospheric Turbulence.* Wiley.
- Businger, J. A. et al. (1971). Flux-profile relationships in the atmospheric surface layer. *Journal of the Atmospheric Sciences* 28.
- Rodgers, C. D. (2000). *Inverse Methods for Atmospheric Sounding: Theory and Practice.* World Scientific.
- Sampson, P. D. and Guttorp, P. (1992). Nonparametric estimation of nonstationary spatial covariance structure. *Journal of the American Statistical Association* 87.
- Solà, J. (2017). Quaternion kinematics for the error-state Kalman filter. arXiv:1711.02508.
- Bar-Shalom, Y., Li, X. R. and Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation.* Wiley.
- Fischler, M. A. and Bolles, R. C. (1981). Random sample consensus. *Communications of the ACM* 24.
- Yang, H. and Carlone, L. (2023). Certifiably optimal outlier-robust geometric perception: semidefinite relaxations and scalable global optimization. *IEEE Transactions on Pattern Analysis and Machine Intelligence* 45(3).
- Nemhauser, G. L., Wolsey, L. A. and Fisher, M. L. (1978). An analysis of approximations for maximizing submodular set functions. *Mathematical Programming* 14.
- ESERO-UK. UK CanSat Competition Guidelines 2025–2026.


Current implementation and hardware evidence: [BOM](flight_computer_bom.md), [design baseline](design_baseline.md), [navigation parameter sources](navigation_parameter_sources.md), [first-fix study](acquisition_first_fix.md), [wind study](wind_retrieval_end_to_end.md), [temperature study](temperature_probe_characterisation.md), [serial protocol](stm32_pi_link_protocol.md), [mass/power budget](mass_power_budget.md). Manufacturer and official regulatory links are collected in those documents; stock/price and source-access dates retain their original scope.
