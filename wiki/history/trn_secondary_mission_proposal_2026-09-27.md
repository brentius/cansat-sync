# Secondary Mission Proposal: Terrain-Relative Navigation

**A Lander Vision System at 300 m: GPS-denied navigation during parachute descent, and image-derived wind profiling of the lower atmosphere**

*Draft for team review, 27 September 2026*

---

## 1. Summary

During parachute descent, a downward-looking camera photographs the ground. The CanSat projects each image onto the ground plane, matches it against a satellite map stored onboard, and fuses the matches with its inertial sensors to work out where it is, in real time, without GPS. GPS is flown purely as ground truth.

The mission has two halves, matched to the two 35% judging categories.

- **Technical:** the navigation system follows the Lander Vision System (LVS) that landed Perseverance in 2021, scaled to a £400 CanSat and scored on the same metrics LVS reported: time to first fix and position error against truth.
- **Scientific:** the same imagery answers a question Huygens faced on Titan. Its Doppler radio experiment measured only one component of the wind; tracking features in its descent images recovered the other. We retrieve the full wind profile of the lowest 300 m of Earth's atmosphere from imagery alone, validate it against GPS Doppler, and combine it with the primary mission's temperature and pressure profile to measure the shear and stability of the boundary layer.

| Key number | Value | Basis |
| --- | --- | --- |
| Ejection altitude | 300–400 m | UK guidelines |
| Design descent rate | ~10.5 m/s | Guideline minimum 10 m/s; slower gives more images |
| Time under canopy | ~30 s | |
| Camera | 1280 × 800 global shutter, ~90° field of view, 5 Hz | Same field of view as Perseverance's LCAM |
| Ground footprint and resolution at 300 m | ~600 × 375 m, ~0.47 m/px | |
| Predicted position error at 300 m | 3–6 m ($1\sigma$) | Error budget, Section 12 |
| Requirement | 10 m | Compare LVS: few metres achieved against a 40 m requirement |
| Target time to first fix | ≤ 5 s after canopy settles | Compare LVS: 10 s |
| Wind retrieval target | ±0.5 m/s RMS in 20 m layers | Against GPS Doppler |
| Mass added | ~20 g | Camera and lens; fast thermistor with radiation shield |

---

## 2. Motivation and framing

### Why terrain-relative navigation

Before Perseverance, a Mars lander's position during descent came from propagating its IMU forward from the last ground-based fix before entry, an error that could reach kilometres. That forced mission planners to choose landing sites that were safe everywhere inside a large ellipse. Terrain-relative navigation removed the constraint: by matching descent images to an onboard map, the vehicle knew its position to within metres and could steer away from hazards. It is why Perseverance could be sent to Jezero, the most hazardous landing site ever attempted on Mars.

TRN is now baseline technology for future landers. JPL lists Mars Sample Return, Europa, Enceladus, Ceres, Titan and the Moon as candidate destinations for LVS-derived systems.

### What LVS does, and what we do

| | Mars 2020 LVS | This CanSat |
| --- | --- | --- |
| When it runs | Parachute descent | Parachute descent |
| Reference map | Orbital imagery processed to 12 m/px (coarse) and 6 m/px (fine) | Satellite imagery processed to ~2 m/px (coarse) and ~0.5 m/px (fine) |
| Image preparation | Project descent images onto the ground plane using altitude and attitude; resample to map scale and orientation | Same |
| Matching | Frequency-domain template correlation; coarse mode 15 landmarks over 3 images, fine mode up to 150 landmarks per image | Same structure, scaled to the H618 |
| Fusion | Landmarks fused with IMU for position, velocity and attitude | Error-state EKF fusing landmarks with IMU and barometer |
| Output used for | Safe Target Selection, then powered-descent divert | Live landing prediction; optional safe-target selection (Section 14) |
| Performance | Fix in 10 s; error of a few metres against a 40 m requirement | Target: fix in ≤ 5 s; error ≤ 10 m |

### Heritage and precedent

- **MER DIMES (2004):** estimated horizontal velocity during parachute descent from successive descent images, the first vision-based navigation on another planet.
- **Mars 2020 LVS (2021):** map-relative localisation during parachute descent, feeding Safe Target Selection.
- **COBALT (2017):** the LVS flown on Earth aboard Masten's Xodiac rocket to 500 m, navigating within metres of the GPS solution. This is the closest analogue to our mission: LVS-class TRN at a few hundred metres, scored against GPS truth.
- **Draper (2023):** satellite-map TRN on a high-altitude balloon and on Blue Origin's New Shepard, robust to rotation rates above 20°/s and cords swinging through the field of view.

### Where the analogy stops

Our map is Earth satellite imagery, our terrain is farmland rather than craters and dunes, and there is no divert: the CanSat descends passively. We claim a working, scaled implementation of the navigation half of the LVS–STS chain, evaluated against truth the way COBALT evaluated LVS. We do not claim Mars relevance beyond that.

---

## 3. Scientific value

### The question

Can a parachute probe measure the wind profile of an atmosphere from its own descent imagery, without radio tracking? And what does that profile, combined with temperature and pressure, tell us about the lowest 300 m of Earth's atmosphere on launch day?

### Why it matters

**For planetary probes.** Descent-probe winds are normally measured by radio Doppler, which needs an ultra-stable oscillator on the probe, a receiver elsewhere, and still sees only the line-of-sight component. On Huygens, the Doppler Wind Experiment's ultra-stable oscillator, tracked by the Cassini orbiter and large radio telescopes on Earth, gave Titan's zonal wind. The meridional component came from comparing descent images, which also showed the ride was far rougher than designed: swing often exceeded 20°/s and sometimes 40°/s against a 10°/s design limit. A camera is something a landing vehicle already carries for TRN, so image-derived winds come essentially for free. We test how well they work, with GPS Doppler as the independent check Huygens did not have for that component.

**For Earth's atmosphere.** The lowest few hundred metres, the atmospheric boundary layer, controls how pollutants, smoke and heat disperse and is where aviation meets wind shear. Routine profiles come from radiosondes launched from a handful of UK stations, typically twice a day. A descent probe samples exactly this layer, at the place and time of launch.

### Scientific hypotheses

- **S1 (method).** Horizontal winds derived from descent imagery alone agree with winds derived from GPS Doppler to within ±0.5 m/s RMS in 20 m layers from 300 m to 80 m. Because image-derived velocity errors scale with altitude, the disagreement shrinks towards the ground.
- **S2 (atmosphere).** In the surface layer (lowest ~50–100 m), the wind profile follows Monin–Obukhov similarity for the stability class implied by the primary mission's temperature lapse rate, and the fitted roughness length agrees in order of magnitude with the land cover upwind, identified in the CanSat's own images.
- **S3 (exploratory).** The probe's swing and spin, measured from IMU and imagery, increase with local wind shear, the effect that made Huygens' ride rougher than expected.
- **S4 (turbulence scaling).** Aviation turbulence models describe an aircraft flying horizontally; a falling probe samples the atmosphere vertically. Using a turbulence model derived for a vertical descent (Appendix I), we test whether, in the surface layer, the correlation of the horizontal wind between two heights depends on the *ratio* of the heights (eddies growing with height) rather than their difference, and whether measured turbulence intensities match the MIL-HDBK-1797 low-altitude model driven by the 6 m wind.

### The physics

The CanSat's horizontal velocity $\mathbf u$ lags the wind $\mathbf w$ by a length $\ell_\tau = v_d^2/g$ of altitude ($\approx 11$ m at 10.5 m/s), so the wind is recovered as

$$
\mathbf w(s) = \mathbf u(s) + \ell_\tau\,\frac{d\mathbf u}{ds},
$$

with $s$ the distance descended; in practice this is done inside a Kalman filter whose turbulence prior is a Dryden model adapted to a vertical descent (Appendix I), rather than by differentiating noisy data. Pendulum swing is removed using the attitude solution in both the image-derived and GPS-derived retrievals, so S1 compares like with like.

In the surface layer, similarity theory predicts

$$
\bar u(z) = \frac{u_*}{\kappa}\left[\ln\frac{z}{z_0} - \psi_m\!\left(\frac{z}{L_O}\right)\right],
$$

where $u_*$ is the friction velocity, $z_0$ the roughness length, $\kappa \approx 0.4$, and $\psi_m$ a stability correction depending on the Obukhov length $L_O$. Stability comes from the primary mission via potential temperature $\theta = T\,(p_0/p)^{R/c_p}$, and each layer's bulk Richardson number

$$
Ri_b = \frac{g}{\bar\theta}\,\frac{\Delta\theta\,\Delta z}{(\Delta\bar u)^2 + (\Delta\bar v)^2},
$$

which uses both missions' data: neither temperature nor wind alone determines whether a layer is turbulent.

### Is the data collection appropriate?

| Quantity | Needed | Provided |
| --- | --- | --- |
| Wind vertical resolution | 20 m layers | 5 Hz imagery at 10.5 m/s: ~2 m per sample, ~10 per layer |
| Wind accuracy | ±0.5 m/s | 0.3–1 m/s per sample, averaging to ~0.1–0.3 m/s per layer before correlated errors |
| Wind truth | Independent, better than 0.2 m/s | GPS Doppler, ~0.1 m/s |
| Temperature | ≤ 0.1 K relative precision, ≤ 1 s response | Fast bead thermistor in a radiation shield. A sensor inside the can lags by several seconds, smearing the profile over tens of metres, which would destroy the stability signal: over 300 m the dry-adiabatic lapse rate is only ~3 K |
| Pressure and altitude | ~0.5 m relative | BME280 |
| Near-surface reference | Wind at a known height | Ground anemometer at 6 m (20 ft), the reference height of the MIL low-altitude turbulence model, logged through the flight |
| External comparison | Independent profile | Forecast model profile and nearest radiosonde, where available |

### Originality

We have not found a CanSat that retrieved its wind profile from imagery rather than GPS, or one that validated an image-based retrieval against Doppler the way Huygens' two methods were compared. Linking the fitted roughness length to land cover seen by the probe itself uses the CanSat as both instrument and observer.

### The science survives technical failure

Image-derived winds can be computed after the flight from the logged imagery (Rung 0 of the ambition ladder), so S1 and S2 do not depend on live TRN working. If the imagery fails entirely, GPS-derived winds still deliver S2.

---

## 4. Mission statement, hypotheses and success levels

**Mission statement.** Demonstrate real-time, GPS-denied, map-relative navigation during CanSat parachute descent using the Mars 2020 LVS architecture, and use the same imagery to measure the wind profile of the lowest 300 m of the atmosphere, validated against GPS Doppler and combined with the primary mission's temperature and pressure to characterise the boundary layer.

### Scientific hypotheses

S1 (image-derived winds match Doppler), S2 (surface layer follows similarity theory; roughness matches observed land cover), S3 (swing grows with shear, exploratory) and S4 (turbulence scaling for a falling probe), as set out in Section 3.

### Technical hypotheses

- **T1 (navigation).** Onboard map-relative localisation, with no GPS in the navigation loop, achieves horizontal position error of 10 m or less against GPS truth throughout descent above 80 m, with a first fix within 5 s of canopy settling.
- **T2 (understood error).** The measured error follows the predicted error budget: attitude-dominated at altitude, map-dominated lower down. The filter's own covariance is consistent with its actual errors (normalised estimation error squared within its $\chi^2$ bounds).
- **T3 (GPS-denied prediction).** A landing prediction $(x, y, t)$ driven only by TRN navigation is statistically calibrated, and its accuracy is quantified against the same predictor driven by GPS and by IMU and barometer alone.
- **T4 (stretch: decision).** Given an onboard hazard map, the CanSat selects a safe, reachable landing target from its TRN solution, as Safe Target Selection did for Perseverance.

### Success levels

| Level | Criterion |
| --- | --- |
| Floor | Safe descent; imagery, IMU, GPS and temperature logs recovered; post-flight image-derived wind profile and boundary-layer analysis (S1, S2) |
| Minimum | Live onboard TRN fixes downlinked during descent |
| Nominal | T1, T2, S1 and S2 hold on the competition flight |
| Full | T3 and S4 hold: live, calibrated, GPS-denied $(x, y, t)$ prediction; turbulence scaling tested |
| Stretch | T4 and S3 |

---

## 5. Ambition ladder

Each rung builds on the one below and can be dropped without affecting it. Probabilities are team judgement, conditional on the launch happening, and should be revised after each test phase.

| Rung | What it adds | Main risk | Chance |
| --- | --- | --- | --- |
| 0. Offline reconstruction | Imagery registered to the map after landing; trajectory and image-derived winds without GPS, so S1 and S2 | Poor imagery | ~80% |
| 1. Live TRN | LVS-style coarse and fine matching onboard; live TRN track next to GPS track on the ground station | Compute, sync, matching on farmland | ~55% |
| 2. Rigour | Error budget, Cramér–Rao comparison, filter consistency, integrity monitoring | Mostly analysis; needs Rung 1 data | ~50% |
| 3. GPS-denied prediction | Live $(x, y, t)$ from TRN navigation; three-way comparison with GPS and IMU-only | Velocity quality from vision | ~45% |
| 4. Safe target selection | Onboard hazard map; safest reachable target chosen and downlinked | Only worth doing if Rung 1 is solid by January | ~40% |

**Deliberately out of scope:** onboard hazard detection from motion stereo (a research project in itself), learned feature matchers running on the flight computer (the H618 has no accelerator and they are too slow), and any steering (cut on risk grounds; Rung 4 is a decision demonstration, not a divert).

---

## 6. Rules compliance

- **Passive recovery.** A plain round canopy on a swivel; no actuators, so no motorised-descent question.
- **Descent rate.** Guidelines require at least 10 m/s and recommend 10–15 m/s. We design for ~10.5 m/s, the slow end, because every extra second is five more images. Measured descent-rate data will be brought to launch day, since the launch provider may replace any parachute it considers incorrectly sized.
- **Envelope.** The camera sits inside the can, looking through a window in the bottom cap; nothing extends beyond the can diameter.
- **Mass 300–350 g, 50 N parachute connection, 20 g launch load, 3 h powered, £400 budget:** see Sections 15 and 17.
- **Radio.** Telemetry packets are kept small (~40 bytes at 1 Hz) to respect the duty-cycle limit of the chosen 868 MHz sub-band.

---

## 7. Concept of operations

| Time | Altitude | Event |
| --- | --- | --- |
| Up to T − 3 h | Ground | Power on; GPS lock (truth only); map and launch-pad location loaded; camera exposure fixed; ground anemometer logging |
| Ascent | 0 → apogee | IMU, barometer and GPS logging; camera idle |
| T (ejection) | 300–400 m | Ejection detected; camera starts at 5 Hz; all frames logged |
| T + 0–3 s | | Canopy inflates; attitude filter settles; tilt reference from swing-averaged specific force |
| T + 3–8 s | ~330 → 280 m | Coarse mode: 3 images, wide search around launch pad, first fix |
| T + 8 s onward | 280 → ~80 m | Fine mode at 5 Hz; EKF fusion; live TRN and GPS positions, TRN covariance and $(x, y, t)$ prediction downlinked at 1 Hz |
| Below ~80 m | | Footprint too small for reliable map matching; frame-to-frame visual odometry from the last fix |
| T + ~30 s | Ground | Touchdown from accelerometer spike (also the truth for predicted touchdown time) |
| Post-flight | | Full-rate reprocessing; wind retrieval; results against S1–S3 and T1–T4 |

---

## 8. System architecture

1. **Camera:** global shutter, hardware-triggered so every frame is timestamped against the IMU clock to well under a millisecond.
2. **Attitude and inertial propagation:** 100 Hz IMU; barometer for altitude.
3. **Map-relative localisation:** image rectification, coarse and fine template correlation (Sections 10 and 11).
4. **Navigation filter:** error-state EKF fusing landmarks, IMU, barometer and visual odometry (Section 11).
5. **Integrity monitor:** $\chi^2$ gating of every landmark and fix, with a protection level downlinked alongside the position.
6. **Predictor:** wind and landing filter propagated to the ground, giving $(x, y, t)$ with covariance (Section 13).
7. **Safe target selection (stretch):** hazard cost map plus reachable disc (Section 14).
8. **Ground station:** live map showing the TRN track, GPS track, live error, protection level, and the predicted landing ellipse and time.

GPS never enters items 3–7. It is logged and downlinked only for comparison.

---

## 9. Map preparation

- **Source:** the most recent available satellite or aerial orthoimagery of the regional launch site, ideally from the same season.
- **Pyramid:** coarse (~2 m/px) and fine (~0.5 m/px) maps, mirroring LVS's two-resolution design.
- **Georeferencing check:** at least four ground points surveyed with GPS and compared against the map. Any offset would otherwise appear as a TRN error against GPS truth, so it is measured and reported separately.
- **Photometric preparation:** conversion to greyscale, contrast normalisation, and optionally matching on gradient images, which are more robust to the difference between summer imagery and a March launch.
- **Landmark selection:** landmarks are pre-selected where the map has strong, stable structure (field boundaries, hedgerows, tracks, copses, buildings), not crop texture, which changes with season. Repetitive tramlines are flagged as aliasing risks.

---

## 10. Map-relative localisation

### Rectification

Using the navigation filter's predicted pose, each descent image is projected onto the ground plane and resampled onto the map's grid, scale and orientation, exactly as LVS does. For the ground plane $Z = 0$, a camera with intrinsics $K$, world-to-camera rotation $R = [\mathbf r_1\ \mathbf r_2\ \mathbf r_3]$ and centre $\mathbf C$ images ground point $(X, Y)$ through the homography

$$
\tilde{\mathbf x} \sim K\,[\mathbf r_1\ \ \mathbf r_2\ \ \mathbf t]\begin{bmatrix}X\\ Y\\ 1\end{bmatrix}, \qquad \mathbf t = -R\,\mathbf C,
$$

so the rectified image is obtained by sampling the descent image at $K[\mathbf r_1\ \mathbf r_2\ \mathbf t]$ applied to each map-grid point. If the predicted pose were perfect, the rectified image would overlay the map exactly; any residual offset is the navigation error.

### Coarse mode (first fix)

- Three consecutive images; ~15 landmarks; 64 × 64 px templates on the 2 m/px map (about 128 m square).
- Search windows cover the prior uncertainty around the launch pad (±300 m).
- Frequency-domain normalised cross-correlation for each landmark, as in LVS.
- Landmark offsets must agree with each other across all three images; a consensus vote rejects aliased matches before the first fix is accepted.

### Fine mode (tracking)

- Every frame at 5 Hz; up to ~50 landmarks; 32 × 32 px templates on the 0.5 m/px map (16 m square); search windows of ±16 m around the predicted location.
- Sub-pixel peak location by quadratic fit; each landmark's covariance estimated from the curvature of the correlation peak, then calibrated against captive-carry data.
- Every landmark is passed to the filter individually, gated by its innovation (Section 11), rather than being collapsed into a single position fix.

### Low altitude

Below ~80 m the footprint (~160 × 100 m) usually contains only field interior. The system switches to frame-to-frame correlation of rectified images (visual odometry), propagating from the last good map fix.

---

## 11. Navigation filter

An error-state extended Kalman filter propagates position, velocity, attitude and IMU biases at 100 Hz from the IMU, with updates from:

- **Landmarks:** each matched map point $\mathbf m_i$ is observed at pixel $\mathbf z_i$ through the pinhole model, so every landmark constrains position *and* attitude:

$$
\mathbf z_i = \pi\big(K\,R\,(\mathbf m_i - \mathbf C)\big) + \boldsymbol\nu_i, \qquad \pi\big([a\ b\ c]^{\mathsf T}\big) = [a/c\ \ b/c]^{\mathsf T}
$$

- **Barometer:** height relative to launch-site pressure, converted to height above ground using the terrain elevation from the Environment Agency 1 m LiDAR model.
- **Visual odometry:** below ~80 m, horizontal displacement between consecutive rectified frames.
- **Gating:** every update passes a $\chi^2$ innovation test before acceptance; rejected landmarks are logged, not silently dropped.

### Two problems that make this hard, and how we handle them

**The pendulum fools the accelerometer.** Under a parachute, the CanSat swings with its suspension line, and the accelerometer measures specific force along that line. An ordinary attitude filter therefore believes it is level when it is actually tilted by 15°. Tilt must come from gyro integration, corrected by the landmark measurements above; the swing-averaged specific force is used only for initialisation.

**Tilt looks like translation.** A small tilt $\delta\theta$ moves the nadir point by $h\,\delta\theta$ and shifts the image almost exactly as a horizontal translation would. The two differ only through perspective: a landmark at angle $\alpha$ off the optical axis moves by $f\sec^2\!\alpha\,\delta\theta$ under tilt but by $f\,\delta x/h$ (independent of $\alpha$) under translation, leaving a separating residual proportional to $\tan^2\!\alpha$ (Appendix C). Landmarks near the image centre carry almost no tilt information; landmarks 45° off-axis carry a lot. This is why we fly a 90° lens, the same field of view as Perseverance's descent camera, and select landmarks spread across the whole image.

---

## 12. Error budget

Treating the dominant sources as independent, horizontal position error at height $h$ is approximately

$$
\sigma_p^2 \approx \underbrace{(h\,\sigma_\theta)^2}_{\text{attitude}} + \underbrace{\frac{(g\,\sigma_c)^2}{n}}_{\text{matching}} + \underbrace{\sigma_{\text{map}}^2}_{\text{georeferencing}} + \underbrace{(h\,\dot\theta\,\delta t)^2}_{\text{timing}} + \underbrace{(\delta z\,\tan\phi)^2}_{\text{terrain relief}}
$$

where $\sigma_\theta$ is residual attitude error after fusion, $g$ the map resolution, $\sigma_c$ the correlation peak precision in pixels, $n$ the number of landmarks, $\dot\theta$ the swing rate, $\delta t$ the camera–IMU timing error, $\delta z$ unmodelled terrain height and $\phi$ the off-nadir angle.

| Term | Assumption | At 300 m | At 100 m |
| --- | --- | --- | --- |
| Attitude | $\sigma_\theta$ = 0.3–1° after fusion | 1.6–5.2 m | 0.5–1.7 m |
| Matching | $g$ = 0.5 m, $\sigma_c$ = 0.2 px, $n$ = 50 | ~0.01 m | ~0.01 m |
| Georeferencing | To be measured against surveyed points | 2–3 m | 2–3 m |
| Timing | $\dot\theta$ = 40°/s, $\delta t$ = 0.5 ms (hardware trigger) | 0.1 m | ~0.03 m |
| Terrain relief | Hedges and gentle slopes, $\delta z$ ~ 2 m | ~1 m | ~1 m |
| **Total** | | **~3–6 m** | **~2.5–3.5 m** |

Three consequences drive the design:

1. **Attitude dominates at altitude, the map dominates low down.** T2 predicts exactly this crossover, which is a falsifiable shape, not just a number.
2. **Hardware triggering is mandatory.** USB camera timestamps jitter by several milliseconds; at $\delta t$ = 10 ms the timing term alone is ~2 m at 300 m. Keeping it under 1 m requires $\delta t \lesssim 5$ ms, and we target under 1 ms.
3. **Matching precision is irrelevant; map quality is not.** Effort goes into the map and the attitude solution, not into cleverer correlation.

**Motion blur:** at 0.5–1 ms exposure, a 40°/s swing smears less than a pixel; spin of 1 rev/s smears a few pixels at the image edge. March daylight allows short exposures.

### Rigour (Rung 2)

- **Cramér–Rao bound:** compute the best achievable pose covariance for the actual landmark geometry and pixel noise, and compare the filter's achieved error against it in simulation and on captive-carry data.
- **Consistency:** normalised estimation error squared across all tests, checked against $\chi^2$ bounds; an inconsistent filter is reported as such.
- **Integrity:** a horizontal protection level $\text{PL} = k\,\sigma_p$ downlinked with every position, with $k$ chosen for a stated risk. The result reported is how often the true error exceeded the protection level, the metric aviation navigation systems are certified on.

---

## 13. GPS-denied landing prediction (Rung 3)

The CanSat's horizontal drift follows the wind with a lag length $\ell_\tau = v_d^2/g \approx 11$ m at 10.5 m/s. A Gaussian-process wind model (the drop-Dryden turbulence model of Appendix I, implemented as a linear Kalman filter) reconstructs the wind profile from velocity and propagates it to the ground to predict the landing point with a covariance. The touchdown time follows from the density-dependent descent rate:

$$
t_{\text{impact}} = t_{\text{now}} + \int_0^{h_{\text{AGL}}} \frac{dh}{v_d(\rho(h))}, \qquad v_d(\rho) = \sqrt{\frac{2mg}{\rho\,C_D A}}
$$

The predictor runs three times in parallel on the same flight, fed by different velocity sources:

| Predictor | Velocity source | Expected velocity noise |
| --- | --- | --- |
| GPS | GPS Doppler | ~0.1 m/s |
| TRN | Navigation filter | ~0.3–1 m/s |
| Dead reckoning | IMU and barometer only; wind from prior | No wind information |

All three predict $(x, y, t)$ with covariance and are scored against the touchdown GPS fix and the accelerometer touchdown spike. The comparison shows directly what vision buys over inertial-only navigation and how close it gets to GPS. That is the argument that justified flying TRN on Perseverance, reproduced on one flight.

---

## 14. Safe target selection (Rung 4, stretch)

On Perseverance, the LVS position estimate fed Safe Target Selection, which picked the safest landing target that was still reachable given fuel limits; powered descent then flew there. We reproduce the decision without the divert:

- **Hazard map:** trees, hedgerows, water, roads, buildings and fences digitised from imagery and open map data, rasterised as a cost map.
- **Uncertainty-aware cost:** hazards are inflated by the current TRN position uncertainty, so a less certain vehicle chooses more conservative targets.
- **Reachability:** a disc of radius $R_{\text{div}}$ around the predicted landing point, representing a notional divert capability (for example, that of a steerable canopy with $L/D = 0.3$, radius $0.3\,h$).

$$
\mathbf p^\star = \arg\min_{\mathbf p\,\in\,\mathcal D(\hat{\mathbf p}_{\text{land}},\,R_{\text{div}})} \; c_\sigma(\mathbf p)
$$

The selected target is downlinked at 1 Hz. Evaluation: whether the chosen target is hazard-free on ground inspection, how stable the choice is over the descent, and whether TRN-driven and GPS-driven selections agree. This is presented honestly as a decision demonstration, not a landing.

---

## 15. Hardware

| Item | Role | Mass (est.) | Notes |
| --- | --- | --- | --- |
| Orange Pi Zero 2W (H618) | Flight computer | existing | Four A53 cores; TRN, filter and logging |
| Global-shutter camera, 1280 × 800 (OV9281-class) | Descent imagery | ~15 g | Module with external trigger or strobe output is essential |
| ~90° lens | Wide field of view | incl. above | Calibrated for distortion with a checkerboard |
| IMU + magnetometer | Inertial propagation | existing | Gyro quality matters most |
| BME280 | Primary mission pressure; barometric altitude; air density | existing | Too slow for the temperature profile inside the can |
| Fast bead thermistor + radiation shield | Primary mission temperature; stability for S2 | ~5 g | Response ≤ 1 s; mounted in the airflow, shaded |
| GPS (u-blox M10-class) | Truth only | existing | Never used by the navigation filter |
| 868 MHz LoRa radio | Telemetry | existing | |
| Plain round canopy on swivel | Recovery | TBC | Sized for ~10.5 m/s |

**Mass:** the secondary mission adds ~15 g for the camera, plus ~5 g for the thermistor and shield. Ground support equipment adds an anemometer on a mast. Total mass is confirmed by weighing against the 300–350 g limit.

**Power:** the camera is idle until ejection; the 3-hour requirement is verified by a bench run in flight configuration.

**Compute:** rectifying a frame and correlating ~50 small templates is estimated at 150–250 ms on one or two cores, enough for 5 Hz; profiled in Phase 0 before any optimisation. Coarse mode is heavier but runs once.

**Storage:** all frames are logged at full resolution (~5 MB/s, ~300 MB per flight) for post-flight reprocessing.

**Cost:** camera and lens add roughly £50–70 (to be confirmed), within the £400 limit.

---

## 16. Software

| Task | Rate | Notes |
| --- | --- | --- |
| IMU and barometer | 100 Hz | |
| Camera trigger and capture | 5 Hz | Trigger timestamped on the IMU clock |
| Rectification and matching | 5 Hz | Separate process, so a slow frame never blocks the filter |
| Navigation filter | 100 Hz propagation, 5 Hz updates | Error-state EKF with $\chi^2$ gating |
| Landing predictors (three) | 5 Hz | Linear Kalman filters |
| Safe target selection | 1 Hz | Stretch |
| Frame logging | 5 Hz | Raw greyscale |
| Telemetry | 1 Hz | TRN position, protection level, GPS position, predictions |
| Ground station | Live | TRN and GPS tracks, live error, ellipse, predicted touchdown time |
| Post-flight reprocessing | Offline | Full-rate TRN, smoothing, analysis |

Python with NumPy and OpenCV, with FFT correlation through NumPy or OpenCV's DFT. Anything too slow is profiled first and only then moved to C.

---

## 17. Test and verification

### Development campaign

| Phase | When | Tests | Gate |
| --- | --- | --- | --- |
| 0. Synthetic descents | October–November | Descent images rendered from the satellite map with simulated swing, spin, blur, noise and lighting change; end-to-end Monte Carlo; Cramér–Rao comparison | Filter consistent; timing fits 5 Hz |
| 1. Captive carry | November–December | CanSat hung under a drone on a 1.5 m line, flown at 50–120 m over farmland with induced swing; GPS truth; many passes per session | Error ≤ 10 m at 120 m against a satellite map; first fix ≤ 5 s |
| 2. Drops | January | Releases from 120 m for full descent dynamics (~10 s per drop) | Live fixes throughout |
| 3. Dress rehearsal | February | Full flight software; launch-site map prepared and georeferencing checked | Clean end-to-end run |

Captive carry mirrors how LVS was tested before launch: helicopter flights over representative terrain, with a gimbal reproducing the attitude rates expected under the parachute. It gives hours of realistic data at no risk to the CanSat.

### Kill criteria

- **After Phase 1:** error above 10 m or unreliable fixes over farmland. Fly Rung 0 (log everything, reconstruct post-flight) with GPS-driven prediction.
- **After Phase 2:** live pipeline unstable onboard. Run TRN in post-flight reprocessing only; keep live prediction on GPS.

### Compliance tests

| Requirement | Test | Consequence of failure |
| --- | --- | --- |
| Mass 300–350 g | Weigh flight configuration | Ballast or trim |
| Can envelope | Fit check with bottom window | Redesign camera mount |
| Parachute connection 50 N | Pull test on the hard point | Rework before any drop |
| 20 g launch load | Static load test of camera mount and lens | Reinforce mount |
| 3 h powered | Bench run | Larger battery |
| Descent rate ≥ 10 m/s | Measured in drops | Resize canopy |
| Radio range and duty cycle | Ground range test; airtime calculation | Smaller packets or different sub-band |

---

## 18. Risks and degradation

**Degradation chain:** live TRN with prediction → live TRN only → post-flight TRN reconstruction → GPS-only analysis.

| Risk | Likelihood | Impact | Mitigation | Degrades to |
| --- | --- | --- | --- | --- |
| Seasonal mismatch between map and launch-day terrain | Medium | High | Structure-based landmarks; gradient matching; most recent imagery | Post-flight reconstruction with heavier methods |
| Tramline aliasing | Medium | Medium | Consensus across landmarks and images; tight fine-mode windows | Gated fixes rejected |
| Attitude error under swing | Medium | High | Landmark-based attitude correction; wide field of view | Larger error, still reported |
| Camera–IMU timing | Medium | High | Hardware trigger | Post-flight time alignment |
| Compute overrun | Medium | Medium | Separate process; profiling; lower frame rate | Fewer fixes |
| Map georeferencing error | Medium | Medium | Surveyed ground points | Bias measured and reported |
| Unknown or changed launch site | Low | High | Maps prepared for all candidate sites | Map built on the day from latest imagery |
| Launch provider replaces canopy | Low | Low | Bring descent data | No effect on TRN |
| Weather cancellation | Medium | High | Captive-carry and drop data stand on their own | Results from test campaign |

---

## 19. Results analysis procedure

| Hypothesis | Data | Calculation | Validity checks | Success criterion |
| --- | --- | --- | --- | --- |
| S1 | Image-derived and GPS-derived wind profiles | RMS difference per 20 m layer vs altitude | Same swing correction and lag model in both | ≤ 0.5 m/s RMS; difference shrinks towards the ground |
| S2 | Wind profile; potential temperature profile; upwind imagery | Similarity-theory fit for $u_*$ and $z_0$; bulk Richardson number per layer; land-cover class upwind | Thermistor lag checked on ascent vs descent; fit residuals | Profile consistent with the stability class; $z_0$ within an order of magnitude of land-cover value |
| S4 | Retrieved wind profile; 6 m anemometer | Marginal-likelihood comparison of attached-eddy vs constant-scale models; fitted $(a, L_\infty)$; intensities vs MIL-HDBK-1797 and similarity predictions | Averaging kernels show which heights are resolved | Preferred model and parameters reported with uncertainties |
| S3 | Swing and spin amplitudes; shear per layer | Correlation of swing amplitude with shear | Separate canopy-induced oscillation from wind forcing | Exploratory: report effect size and uncertainty |
| T1 | Onboard TRN positions; GPS truth | Horizontal error vs altitude; time from canopy settling to first accepted fix | Georeferencing bias measured separately | Error ≤ 10 m above 80 m; first fix ≤ 5 s |
| T2 | TRN errors and filter covariances | Error vs altitude compared with the Section 12 budget; NEES against $\chi^2$ bounds; protection-level exceedance rate | Enough samples per altitude band | Crossover shape matches; filter consistent |
| T3 | Three predictors' outputs; touchdown fix and time | Prediction error vs altitude for each source; calibration of TRN-driven ellipses across all drops | Predictors share identical models apart from input | TRN-driven prediction calibrated; ranked between GPS and dead reckoning |
| T4 | Selected targets; hazard map; ground inspection | Hazard-free rate; decision stability; TRN vs GPS agreement | Hazard map checked on the ground | Selected targets hazard-free |

---

## 20. Team roles

| Role | Responsibilities |
| --- | --- |
| TRN lead | Map preparation, rectification, coarse and fine matching, synthetic descent renderer |
| Navigation and estimation lead | Error-state EKF, error budget, Cramér–Rao and consistency analysis, landing predictors |
| Systems, hardware and ground station lead | Camera integration and triggering, mass and power budgets, captive-carry rig, telemetry, ground-station display |
| Documentation and outreach lead | Reports, risk register, compliance evidence, outreach |

---

## 21. Schedule

Dates for PDR, CDR and the regional launch are to be confirmed against the 2026–27 calendar.

| Month | TRN | Navigation and estimation | Systems and ground station | Milestones |
| --- | --- | --- | --- | --- |
| October | Map pipeline; synthetic renderer; coarse mode | Filter in simulation | Camera and trigger selected; bottom-window mount | |
| November | Fine mode; profiling on the H618 | Error budget; Cramér–Rao analysis | Captive-carry rig | PDR |
| December | Captive-carry tuning | Consistency and integrity tuning | Telemetry and ground-station display | Phase 1 gate |
| January | Drop testing | Landing predictors (Rung 3) | Compliance tests | Phase 2 gate |
| February | Launch-site map | Flight software freeze | Integration | Dress rehearsal; CDR |
| March | Launch-day operations | Launch-day operations | Launch-day operations | Regional launch (date TBC); finalists notified early March |
| March–May | Full-rate reprocessing; map of the National Final site | Wind retrieval; analysis against S1–S3 and T1–T4 | Data archiving; final presentation | FDR; National Final (May, York area) |

---

## 22. How this maps to the assessment criteria

### Final judging (Guidelines, evaluation and scoring)

| Category | Weight | What they assess | Where this proposal delivers it |
| --- | --- | --- | --- |
| Technical achievement | 35% | Technical complexity and originality; performance of both missions; explaining failures | LVS-architecture TRN (T1–T3); degradation chain and kill criteria, so any failure comes with an explanation and a fallback result |
| Scientific value | 35% | Clear, well-founded scientific purpose; original use of the CanSat; appropriate data collection; understanding; reporting | S1–S3 with the physics stated (Section 3); data-appropriateness table; primary and secondary data combined in the Richardson number; figure plan below |
| Professional competencies | 20% | Teamwork, planning, adaptability, communication | Roles, schedule, kill criteria written in advance (adaptability by design) |
| Outreach | 10% | Communication to school and community | Descent footage on the satellite map and the LVS comparison are strong outreach material |

### PDR mark scheme, secondary mission

The top band requires the mission to be clearly justified with a well-thought hypothesis and a "why", with the most suitable hardware and software identified. Sections 2–4 give the why (planetary and terrestrial), explicit hypotheses in both categories, and Sections 15–16 justify hardware and software, including sensor limitations (the in-can temperature lag). The PDR itself also needs block diagrams for both missions and a Gantt chart, which this document describes but does not draw.

### Mathematics

Appendix E indexes every piece of mathematics in the mission, what it is used for, where in this document it is used, and which appendix derives it.

### Figure plan (technical reporting)

Every figure with labelled axes and SI units:

1. TRN horizontal error (m) against altitude (m), with the predicted error budget overlaid (T1, T2).
2. Wind speed (m/s) and direction (°) against altitude (m): image-derived and GPS-derived, with error bars (S1).
3. Image minus GPS wind difference (m/s) against altitude (m) (S1).
4. Mean wind speed (m/s) against ln(height), with similarity fit giving $u_*$ and $z_0$ (S2).
5. Potential temperature (K) against altitude (m), with bulk Richardson number per layer (S2).
6. Landing-prediction error (m) and touchdown-time error (s) against altitude for the three predictors (T3).
7. Descent frames mosaicked on the satellite map, with TRN and GPS tracks.
8. Averaging kernels (rows of $A$) against altitude (m), with degrees of freedom for signal, for image-derived and GPS-derived retrievals (S1).
9. Wind correlation against height ratio and against height difference, with the fitted eddy-scale profile $L(z)$ (m) (S4).

---

## 23. Open questions

- [ ] Which regional launch site, and the date and resolution of the best available imagery of it; the same for the National Final site near York
- [ ] Nearest radiosonde station and model profile source for each launch site
- [ ] Camera module with a usable external trigger or strobe output, and a 90° lens that fits the can
- [ ] Drone permissions for captive carry and drops over suitable farmland near school
- [ ] Whether the 300–350 g limit includes the parachute
- [ ] LoRa sub-band and packet size against duty-cycle limits

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

Every piece of mathematics in the mission, what it is for, where it is used, and where it is derived. **Core** is needed for the flight results; **Rigour** strengthens T2 and S1–S4 but is post-flight analysis; **Stretch** is done only if time allows; **Open** is unproven.

| Topic | Used for | Where used | Derived in | Status |
| --- | --- | --- | --- | --- |
| Pinhole camera; ground-plane homography | Rectifying descent images onto the map grid | Sec. 10; T1 | App. A | Core |
| Normalised cross-correlation via FFT; sub-pixel peaks; peak covariance | Landmark matching | Sec. 10; T1 | App. G | Core |
| Consensus (RANSAC) | Rejecting aliased matches (tramlines) | Sec. 10 | App. G | Core |
| Terminal velocity; hypsometric altitude | Descent-rate model; barometric height | Secs. 3, 11, 13 | App. F | Core |
| Lag length and inversion conditioning | Recovering wind from drift | Sec. 3; S1 | App. F | Core |
| Pendulum specific force | Why the accelerometer is blind to swing | Sec. 11 | App. F | Core |
| Error-state EKF; quaternion attitude; reprojection Jacobians | Navigation filter | Sec. 11; T1 | App. H | Core |
| $\chi^2$ innovation gating | Rejecting bad landmarks | Sec. 11 | App. H | Core |
| Tilt–translation separability | Lens choice; landmark placement | Sec. 11 | App. C | Core |
| Error budget; crossover altitude; timing requirement | Design; the T2 prediction | Sec. 12; T2 | Apps. B, H | Core |
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

which always points along the line. The lateral body-frame components are zero whatever the tilt, so accelerometer-derived tilt reads "level" throughout the swing. A real canopy–can system adds small residuals from canopy motion and line stretch, but not enough to recover tilt.

**F.5 Pendulum dynamics.** The swing angle obeys a driven, damped oscillator with $\omega_0^2 = g/L_p$:

$$
\ddot\theta + 2\zeta\omega_0\dot\theta + \omega_0^2\,\theta = F(t).
$$

With light forcing, $\theta \approx -\ddot\theta/\omega_0^2$, so tilt is observable from the gyro's angular acceleration even though the accelerometer is blind: a cross-check on the filter's attitude. For S3, the swing spectrum is the forcing spectrum filtered by the pendulum:

$$
S_\theta(\omega) = |H(\omega)|^2\,S_F(\omega), \qquad H(\omega) = \frac{1}{\omega_0^2 - \omega^2 + 2i\zeta\omega_0\omega}.
$$

**F.6 The path through the air is vertical.** Each layer moves at $U(z)$, and the probe drifts with the local wind. For linear shear $U = U_0 + Sz$ and $z = z_0 - v_d t$, the sampled point's horizontal offset in the frame of the layer the probe is in is

$$
\xi = \int_0^t \big[U(z(t')) - U(z(t))\big]\,dt' = S\,v_d\int_0^t (t - t')\,dt' = \frac{S\,\Delta z^2}{2\,v_d}.
$$

For $S = 0.05\ \text{s}^{-1}$ over 100 m this is ~24 m, and near the ground it is under a metre per 10 m of descent, so the sampling path is vertical to within a few percent.

**F.7 Frozen-field validity.** An eddy of size $L$ is crossed in $L/v_d$ and lives for about $L/\sigma$; the ratio $\sigma/v_d \approx 0.05$–$0.1$ is independent of $L$, so the frozen-field assumption holds at every height.

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

**H.4 Consistency.** The normalised estimation error squared $\varepsilon_k = \mathbf e_k^{\mathsf T}P_k^{-1}\mathbf e_k$, with $\mathbf e_k$ the error against GPS truth, averaged over $N$ independent runs, satisfies $N\bar\varepsilon_k \sim \chi^2_{nN}$ for a consistent filter; values outside the bounds mean the filter is over- or under-confident.

**H.5 Cramér–Rao bound.** For a single fix, the Fisher information from $n$ landmarks is $J = J_0 + \sum_i H_i^{\mathsf T}R_i^{-1}H_i$, and any unbiased estimator satisfies $\operatorname{Cov}(\hat{\mathbf x}) \succeq J^{-1}$. Comparing achieved error with $J^{-1}$ for the actual landmark geometry shows how close the system is to the best possible.

**H.6 Error budget.** The dominant terms (Section 12) cross at

$$
h^* = \frac{\sigma_{\text{map}}}{\sigma_\theta},
$$

~290 m for $\sigma_\theta = 0.5°$ and $\sigma_{\text{map}} = 2.5$ m. Reading $h^*$ off the flight's error-vs-altitude curve infers in-flight attitude accuracy. The timing requirement is $\delta t \lesssim 1\ \text{m}/(h\dot\theta) \approx 5$ ms (Appendix B).

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

**J.2 Landing prediction.** At each step the state is copied, $I$ reset to zero, and propagated to the ground with no velocity updates, applying the anemometer as a measurement at 6 m. The landing point is Gaussian with mean and covariance from the propagated state; touchdown time is

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

- Johnson, A. E. et al. (2017). The Lander Vision System for Mars 2020 Entry Descent and Landing.
- Johnson, A. E. et al. Implementation of a map relative localization system for planetary landing. *Journal of Guidance, Control, and Dynamics.*
- Cheng, Y. et al. (2021). Making an onboard reference map from MRO/CTX imagery for Mars 2020 Lander Vision System. *Earth and Space Science.*
- Johnson, A. et al. (2007). Design through operation of an image-based velocity estimation system for Mars landing (DIMES). *International Journal of Computer Vision* 74(3).
- COBALT: terrestrial flight test of landing navigation using the Lander Vision System with Navigation Doppler Lidar. NASA Technical Reports Server.
- Maggio, D., Mario, C., Streetman, B., Steiner, T. and Carlone, L. (2023). Vision-based terrain relative navigation on high-altitude balloon and sub-orbital rocket. arXiv:2302.08011.
- Dever, C. et al. (2017). Guided-airdrop vision-based navigation. AIAA Aerodynamic Decelerator Systems Technology Conference.
- Building maps for terrain relative navigation using Blender: an open-source approach. NASA Technical Reports Server.
- Hartikainen, J. and Särkkä, S. (2010). Kalman filtering and smoothing solutions to temporal Gaussian process regression models.
- JPL Robotics. Mars 2020 Terrain Relative Navigation.
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
