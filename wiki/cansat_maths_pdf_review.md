# Review of `cansat_maths.pdf`: maths re-check and comparison with the current mission

13 October 2026. Nothing here is a measurement: it is a check of derivations, arithmetic and consistency between documents and code. No simulation was rerun, and no source file, proposal or PDF was changed.

## 1. Bottom line

**The mathematics is right.** All 249 numbered equations were re-derived, and every stated number that could be recomputed was recomputed independently. One typographical error and a handful of small slips were found; none changes a result.

**The document is out of date against the mission.** It is the 30 September snapshot, written against the 27 September proposal. Three things follow:

1. Its worked numbers use a camera, descent rate and filter that are no longer the baseline (section 4.1 gives the same numbers at the current baseline).
2. Several of its conclusions have been overtaken by the 9 to 11 October simulation studies (section 4.3). The largest: the error budget's attitude/map crossover does not appear, and canopy glide, which the PDF's retrieval model leaves out, dominates the wind error.
3. The 13 October proposal still contains about a dozen statements that this PDF showed to be wrong or incomplete (section 4.4). The most concrete is a sign inconsistency in the turbulence coordinate (Appendix I.3 against I.6).

The flight code follows the PDF's conventions where it implements them. It departs in a few places, one of which the PDF explicitly warns against: `landing.py` adds the position covariance as an independent term (section 5).

## 2. What was reviewed, and how

The file is `C:\Users\b_yee23\Downloads\cansat_maths.pdf`: 52 pages, 249 numbered equations, dated 30 September 2026, SHA-256 `771cacde…1a4f`. It is byte-identical to the attachment audited on 1 October under the name `type_shit.pdf` ([audit](mathematics_simulation_audit.md)), so this is a second, independent pass over the same document.

The PDF's text layer drops Greek letters, so symbols were read from the LaTeX source. The body of [cansat_mathematics.tex](cansat_mathematics.tex) (sections 2 to 8) differs from the 30 September fragments in `sim/maths/` only by four "historical example" notes, so it is a faithful source for the PDF's equations. Numbers were cross-read against the PDF text.

| Part | Who checked | Method | Result |
|---|---|---|---|
| Sections 2 to 8, every displayed equation | Main reviewer | Re-derived by hand from the stated assumptions | All correct |
| Sections 2 to 4 numbers | Helper | 61 stated values recomputed; about 75 closed forms checked by a different route (own RK4, quadrature, finite differences) | 0 mismatches |
| Section 5 numbers and Jacobians | Helper | About 55 values and 40 identities; Jacobians by central differences, with deliberately wrong signs shown to fail; the "executed checks" table reproduced with the stated seed | 0 mismatches |
| Sections 6 to 9 | Second reviewer, independently | Re-derived; Kalman/RTS against a brute-force Gaussian solve (agreement 2e-14); every table entry in section 9 compared with `sim/maths/results/verification.json`; a separate 10,000-drop Monte Carlo with a different seed | 0 mismatches |
| Current mission documents and flight code | Two helpers, spot-checked by the main reviewer | Line-by-line comparison with file and line references | Sections 4 and 5 |

Limits of this review:

- Only one page (file page 34) was inspected as a rendered image, from the 1 October audit's retained copy. A rendering defect elsewhere would not have been seen, although the text layer shows no undefined references and the equation count (249) matches.
- The original 2,000-descent runs were not rerun; section 9 was checked against the retained result files and by an independent reimplementation.
- The MIL-F-8785C intensity coefficients and the cited papers were not re-read against primary sources.
- The Drive reference folder was listed on 13 October: it still holds only the 2025 regulations.

## 3. Maths findings

### 3.1 Errors and slips

| # | Where | Printed | Should be | Weight |
|---|---|---|---|---|
| 1 | Section 6.4, sentence after eq. (6.18), printed page 33 | Transfer function "(1 + √3, iω)/(1 + iω)²" | (1 + √3 iω)/(1 + iω)². A comma was typed where a thin space was meant. Still present at `wiki/cansat_mathematics.tex:2192` and `sim/maths/core_fragment.tex:91` | Typo; already recorded on 1 October, never fixed. The state-space model and spectrum around it are correct |
| 2 | Section 2.6 | Amplification "approximately 3.670" for ℓ = 11.2424 m at 20 m wavelength | 3.671 (the value is 3.6707). 3.670 is what the rounded ℓ = 11.24 m of section 7.1 gives | Last digit only |
| 3 | Sections 4.1 and 5.3.1 | Pitch blur 0.4468 / 0.8936 / 0.9362 px (centre / horizontal edge / corner) | Correct only for rotation about the axis parallel to the image's short side (flow along the long side), which the text does not say. About the other axis the same rate gives 0.447 px at the horizontal edge and 0.681 px at the corner | Unstated assumption; the printed values are the worse case |
| 4 | Section 5.4.1 | FFT arrays "must be padded … to the sum of image and template dimensions minus one" | That is needed only for partial-overlap shifts. For the valid windows the text then crops to, padding to the image size is enough | Wording |
| 5 | Section 9.5 | Grid "between approximately 299 and 1 m" | The lowest cell midpoint is 2.85 m (142 cells of 2.1 m) | Wording |
| 6 | Section 9.9 | "1.813 m/s", "17.275 %" | These are one-seed estimates from 4,000 samples; the exact values for that model are 1.849 m/s and 16.6 %. Within two standard errors, but five digits overstate the precision | Presentation |
| 7 | Section 10.2 | "Compilation was attempted and failed … no rendered page layout has been verified" | Stale: the sentence sits inside a compiled PDF. Section 9.1's "46 passing tests" is stale in the same way. Both are still in the current source (lines 2814 and 2588) | Stale text |
| 8 | Section 10.6, last page | Drive folder link | The link appears to run off the right-hand edge: the PDF's text layer stops eight characters short of the full address. Inferred from the extracted text, not seen on a rendered page | Layout |

### 3.2 True, but only under an assumption the text leaves out

- **Section 6.6, time form of the turbulence model.** It needs a known, positive descent speed. With an estimated or varying speed the model is no longer linear-Gaussian. Section 9 says "known constant"; the derivation does not.
- **Sections 7.7 and 7.10, smoother gain and cross-height recursion.** They need the predicted covariance to be invertible. It is in every model used here.
- **Section 9.11, coefficient-freezing test.** Truth and estimator share the same discretisation, so the test measures how the predicted spread changes, not error against a continuous truth. An independent continuous integration gives 14.0593 m against 14.0592 m, so the effect is negligible in this case.
- **Section 8.5, two-cell counterexample.** It works only for an intensity ratio above about 20 between the two cells (the text uses 100 and says the case is not physical).
- **Sections 5.6.1 and 5.11 pull in different directions.** The budget takes 0.3 to 1 degree of residual tilt from the proposal. The PDF's own tilt-information example (five landmarks spread across the image, 0.2 px noise) gives 8.96e6 rad⁻², which is 0.02 degrees. The text labels both as illustrations, but a reader could take the budget as the prediction. The closed-loop study later found 0.03 degrees.

### 3.3 Confirmed without change

Descent and canopy sizing; the tanh / ln cosh transient and the 29.31 s fall time; the hypsometric equation and its error propagation; the lag time v_d/g and its inversion; the pendulum equations and the gyro-acceleration counterexample; the shear offset S(Δz)²/2v_d; potential temperature and lapse rate; the Monin-Obukhov profile with displacement and lower-boundary terms; the unstable stability-function integral; the Richardson-number sensitivities and the Miles-Howard argument; footprint, optical flow and blur; storage, sampling and queue arithmetic; the LoRa airtime (82.176 ms); the camera model, homography and its determinant; the tilt/translation information; normalised cross-correlation, the sub-pixel vertex and its two-dimensional counterexample; RANSAC counts (25, 25, 108); error-state dynamics, discretisation, reprojection Jacobians with their signs, the Joseph update and the reset Jacobian; visual-odometry signs and shared-image covariance; the chi-square gate (11.618) and protection level (2.4477); the truncated-least-squares lifting and certificate conditions; the greedy bound; the transverse covariance, its spectrum, the two-state model and its stationary covariance; the warped coordinate; the retrieval model, Kalman filter and smoother; the layer-average covariance; the release-velocity marginal; touchdown-time spread (0.82 s); the fixed-endpoint theorem and both drift counterexamples.

All ten "corrections to the proposal" listed in the PDF's section 1.2 are mathematically correct.

## 4. Against the current mission scope

Sources: the 13 October [proposal](trn_secondary_mission_proposal.md), the [design baseline](design_baseline.md), the [BOM](flight_computer_bom.md), the [mass and power budget](mass_power_budget.md), and the 9 to 11 October studies. The current LaTeX source already carries "historical example" notes on the camera numbers; the PDF does not.

### 4.1 Inputs that have changed

"Current" optical values are for an ideal pinhole. The real lens is barrel-distorted (about 110 to 115 degrees horizontal is an inference, not a calibration), so they are provisional too.

| Quantity | PDF (30 Sep) | Current baseline | Same calculation at the current baseline |
|---|---|---|---|
| Camera | 1280 × 800, f = 640 px, 90° × 64.0° | IMX296 1456 × 1088, f = 724.6 px, 90.3° × 73.8° | Ground sample at 300 m: 0.469 → 0.414 m/px. Footprint: 600 × 375 → 603 × 450 m |
| Pitch blur, 40°/s, 1 ms (centre / edge / corner) | 0.447 / 0.894 / 0.936 px | Candidate exposure is now about 0.5 ms | 0.506 / 1.016 / 1.086 px at 1 ms; half that at 0.5 ms |
| Spin blur, 1 rev/s, 1 ms, corner | 4.74 px | | 5.71 px |
| Descent rate | 10.5 m/s throughout | Not frozen; drop-calibration near 11.0 m/s recommended | Lag time 1.071 → 1.122 s; lag length 11.24 → 12.34 m; amplification at 20 m wavelength 3.67 → 4.00; 300 m takes 28.6 → 27.3 s; samples per 20 m layer 9.5 → 9.1; shear offset over 100 m 23.8 → 22.7 m; lag/eddy crossover 22.5 → 24.7 m; touchdown-time spread 0.82 → 0.75 s |
| Start height | 300 m only | About 300 to 400 m | Up to 36 s and 16 layers above 80 m, against 11 |
| Vision below 80 m | "TRN" velocity at 5 Hz to the ground | T1 applies above 80 m; low-altitude visual odometry is not implemented | The PDF's 20.7 m "TRN" rows have no current counterpart |
| Image storage | 8-bit, 1.024 MB per frame, 5.12 MB/s | 9.9 MB/s packed RAW10, 15.8 MB/s in 16-bit | 30 s is 0.30 GB packed |
| Processing | "250 ms per frame", 50 landmarks at 5 Hz | Pi Zero 2 W untimed; 50 at 5 Hz is "an ambition" | No basis for a throughput statement either way |
| Radio | 40 bytes, SF7, 125 kHz, 4/5: 82.176 ms, 8.2176 %, called "not the selected configuration" | The same mode is now the candidate at 869.525 MHz, under a 10 % duty limit | Numbers unchanged. The PDF cites the SX1276 datasheet; the E22-900M22S is an SX1262. For this mode the SX126x formula gives the same airtime (recalled from the datasheet, not re-read) |
| Navigation filter | 15 error states | 17 (adds barometer bias and camera time offset), with iterated update, post-fit rejection and prior recovery | The PDF's 15-state consistency test does not apply as written |
| Matching term in the error budget | 0.5 m/px, 0.2 px, 50 landmarks: 0.014 m | 1 m/px, 12 landmarks: 0.058 m | Budget totals move by less than 1 mm |
| Velocity noise assumed in section 9 | GPS 0.1 m/s white; vision 0.6 m/s white, or correlated 0.85 per sample | Simulated filter output: 0.19 m/s per axis, lag-1 autocorrelation 0.93 at 5 Hz | The PDF's "correlated, tracked" case is the relevant one, with different numbers |
| Thermistor | 1 s response assumed: 10.5 m lag | 1 s is an unqualified goal for the GA10K3MCD1 | 11 m at 11 m/s |
| Reference wind height | 6.096 m (20 ft) | Proposal says "6 m (20 ft)" | Not adopted |
| Suspension length | Symbol only | 1.5 m in one study, 0.35 to 0.60 m lines in the budget | The pendulum frequency needs one agreed value |

### 4.2 Requirement wording

- **S1.** The PDF's 0.5 m/s, 20 m layers and 80 to 300 m match the proposal. The PDF asked for "component or vector" to be stated; the proposal now says per axis and also reports vector RMS. The PDF's 0.209 m/s is error against simulated truth, not the image-minus-GNSS difference that S1 now scores, and the PDF says so.
- **T1.** The PDF never claims 10 m or 5 s; it says neither is established. Consistent.
- **T2.** The proposal now scores position and velocity covariance. The PDF covers position only and has no velocity test.
- **T3.** The proposal compares TRN, GNSS and inertial-only predictors. The PDF's "prior" channel is not an inertial-only predictor, as its section 9.1 states.
- **S2, T4, timing.** The PDF's neutral-fit uncertainty, hypothetical divert radius and 4.77 ms timing limit are consistent with the current conditional wording and the 0.5 ms target.

### 4.3 Conclusions that later studies have overtaken

| PDF conclusion | Later result | Source |
|---|---|---|
| Attitude and map-datum terms cross near 286 m; budget of 2.7 to 6.1 m at 300 m | Error relative to the map is flat at 0.5 to 0.7 m from 300 m to 80 m. One image fixes tilt to 0.03 degrees. No crossover; the formula gives 480 to 1100 m with the simulated tilt | [Closed-loop study](navigation_closed_loop_simulation.md), lines 14 and 182 to 190 |
| The plain 15-state filter is the navigation model | With realistic errors the plain filter gives 22.7 m RMS; the hardened 17-state filter gives 0.61 m | Same study |
| An uncalibrated lateral drag ratio can cost about 0.5 m/s of wind | Canopy glide, absent from the PDF's retrieval model, can cost far more: 0.43 m/s per axis at a 5 degree trim, 1.26 at 12 degrees, 3.04 at 25 degrees | [End-to-end study](wind_retrieval_end_to_end.md) |
| Image and GPS winds can share a bias (stated qualitatively) | Quantified: the S1 difference reads 0.22 to 0.33 m/s in all five canopy classes, including those where the wind is wrong by 1 to 3 m/s | Same study |
| "Well-calibrated landing ellipses" under matched parameters (94 to 95 %) | Holds for a stable canopy (93 to 98 %). With a 12 degree trim the 95 % ellipse holds 62 to 84 % of touchdowns above 80 m | Same study |
| "First-fix reliability not established"; "no renderer, no flight software" | Synthetic first fix from 91 % of images on mixed farmland, 41 % on large fields; candidate flight code exists. Still no real imagery and nothing run on the Pi | [First-fix study](acquisition_first_fix.md) |

The PDF anticipated the first, fourth and fifth of these in general terms (non-unique crossover, shared bias, calibration conditional on the model). Glide appears only as a general trim-force term in section 2.5 (eq. 2.24 to 2.25); it is not in the retrieval model or in any experiment.

### 4.4 PDF corrections that the current proposal has not taken up

The proposal's appendices were kept "so equation reasoning remains traceable", but these statements are still printed as fact in the 13 October text.

| Proposal location | Still says | PDF's correction |
|---|---|---|
| I.3 and I.6 (lines 590, 615), J.1 (641) | ζ(z) = ∫^z dz′/L, which increases upward, yet ζ̇ = v_d/L > 0 during descent | Sign inconsistency. Use q(z) = ∫_z^{z_start} dξ/L, which increases with descent (PDF eq. 6.19) |
| I.1 (577) | Path "vertical to within a few percent" and frozen field "at every height" | Contradicts the proposal's own revised F.6 and F.7 (lines 504, 506) |
| K.2 (689 to 697) | Wind profile without displacement height or the lower-boundary term | PDF section 3.2 |
| K.3 (699) | "Above about 0.25 … shear is too weak to sustain turbulence"; "starting estimate of L_O" | The quarter threshold is a sufficient condition for linear stability of the gradient number, not a turbulence cutoff for a bulk number; a lapse rate does not fix L_O (PDF sections 3.4, 3.5) |
| Appendix C (417) | Residual is "the only information"; Fisher information "for tilt, given translation" scales with the spread of tan²α | The spread formula is for translation unknown. Given translation, the information is nonzero even with no spread. In two dimensions there is further information (PDF section 5.3) |
| G.4 (532) | "N ≈ 24" | 25 after rounding up; 24 trials give 0.998997, just short of 0.999. `firstfix.py:142` already rounds up |
| H.5 (561) | Bound with a prior stated for "any unbiased estimator" | Adding a prior does not give a frequentist bound (PDF section 5.6.2) |
| H.7 (571) | v ≈ (h/f)Δu/Δt, unsigned | Sign depends on the shift convention; descent adds scale change (PDF section 5.5.4) |
| J.2 (651) | "Propagated mean and covariance" | Landing covariance must be P_pp + P_II + P_pI + P_Ip (PDF section 8.1) |
| J.6 (675 to 681) and index row (448) | "Open question" | Partly answered: absolute landing covariance cannot grow under a fixed linear-Gaussian model; remaining-drift variance can, by counterexample. The realistic continuous case is still open |
| L.2 (711 to 715) | Binary switches give "a quadratic programme"; rank-1 means "certified global optimum" | The switched cost is cubic and needs a lifting; rank one alone is not a certificate (PDF section 5.8) |
| L.3 (717) | Greedy reaches 1 − 1/e of the optimum of log det; "should favour edge landmarks" | The guarantee is on the gain over the baseline, not the raw log-determinant; edge preference is not a theorem (PDF section 5.9) |
| F.2 (464 to 467) | "Mean temperature" | Log-pressure-weighted virtual temperature (PDF section 2.3) |
| I.7 (623) | "6 m (20 ft)" | 6.096 m |

### 4.5 Mathematics the mission now relies on that the PDF does not contain

- Canopy glide as a state, and lateral specific force used as an airspeed measurement (`windprofile.py`). This is now the main risk to the wind result.
- The two extra filter states (barometer bias, camera time offset), rate-dependent gyro noise, and the iterated update with post-fit rejection and prior recovery (`eskf.py`).
- The first fix with heading unknown: edge-strength correlation over position and 60 headings, pose refinement, two-image confirmation, and levelling from averaged specific force (`coarse.py`).
- A real-lens distortion model. The PDF says only "undistort first".
- Mean wind as an estimated log-law state with an anemometer prior; the drift table and its descent-rate rescaling (`windprofile.py`, `landing.py`).
- Frame time-stamping and cumulative inertial integrals across the serial link (`link.py`).
- The thermistor readout chain and its self-heating duty; mass, power and canopy sizing with numbers.

## 5. Against the implementations

Claims marked "helper" were checked numerically by a helper and not repeated by the main reviewer; the rest were confirmed by reading the cited lines.

| Topic | PDF | Code | Assessment |
|---|---|---|---|
| Frames, gravity sign, body-to-world rotation | Section 5.1 | `eskf.py:35, 99-107` | Same |
| Attitude error convention | Right/local, true minus nominal | `eskf.py:11-12, 226, 446` | Same |
| Reprojection Jacobians | H_p = −J R_cb Rᵀ, H_θ = +J R_cb [p_b]× | `eskf.py:135` | Same, signs included |
| Gate, Joseph update, protection level | Sections 5.5.3, 5.6.3, 5.7 | `eskf.py:42, 246, 432` | Same |
| RANSAC count | Rounded up | `firstfix.py:140-142` | Same |
| Turbulence model | F, c = [1, √3], P = I/4, L = 0.5 max(z, 1 m) | `windprofile.py:79-86, 126-128` | Same |
| Turbulence coordinate sign | q increases with descent (eq. 6.19) | ζ is measured up from the ground, as in the proposal, but the rate uses the absolute increment (`windprofile.py:152, 163`) | Same in effect: the proposal's sign slip (section 4.4) does not reach the code |
| Wind-model discretisation | Block exponential, Q = EΦᵀ (section 7.5) | `windprofile.py:164-170` | Same |
| Smoother, layer covariance, likelihood | Sections 7.7, 7.10, 7.12 | `windprofile.py:321, 415, 425-429` | Same |
| Touchdown-time spread | Section 8.2 | `landing.py:271` | Same (speed spread held as a fraction) |
| Independent variable | Distance descended | Time | Equivalent for constant descent speed |
| Process noise | Exact block-exponential form | Diagonal, density × dt (`eskf.py:196-202`) | Equivalent to first order at 100 Hz |
| Correlated velocity error | Per-sample correlation, 0.85 in the example | Continuous, 1.6 s default (`windprofile.py:74`), which is 0.88 per 5 Hz sample | Equivalent form. The navigation study quotes 0.93 beside the same 1.6 s, which an exponential law would not give |
| Lag time | v_d/g | `10.5/G0` hard-coded default (`windprofile.py:66`) | Same formula; the default is tied to 10.5 m/s |
| Release velocity | Joint prior with the wind | Independent prior, 15 m/s wide (`windprofile.py:70, 196-206`) | Different but safe: a wide prior treats it as unknown, which is the PDF's point |
| **Landing covariance** | P_pp + P_II + P_pI + P_Ip; "adding an independent position term also fails" | Position covariance added as an independent term (`landing.py:262-268`) | **The shortcut the PDF warns against.** Simulated coverage for a stable canopy is 93 to 98 %, so the effect looks small there, but the cross term has not been measured |
| Sub-pixel peak | Per-axis fit is biased when the peak is skewed (0.1 px in the example) | Per-axis fit (`coarse.py:338-349`) | Different; small beside a 1 m/px match |
| Landmark map error | Joint covariance or extra states for shared error | Independent, variance inflated tenfold in the simulation | Different; a pragmatic allowance, not the PDF's treatment |
| Reset after an iterated update | Reset Jacobian always applied; iteration not discussed | Applied only when there was a single pass (`eskf.py:427`) | Not covered by the PDF. A toy attitude-only check supports the code (helper) |
| Certified registration, greedy landmark selection, safe-target selection, peak-curvature covariance, visual odometry, averaging kernels | Sections 5.4.3, 5.5.4, 5.8 to 5.10, 7.3 | Not in `code/` | Not implemented (stretch or post-flight) |

## 6. Suggested actions

1. **Treat the PDF as a dated snapshot.** For anything design-facing, use the right-hand column of section 4.1, or reissue the document from the current source.
2. **Fix the source** (both `wiki/cansat_mathematics.tex` and `sim/maths/core_fragment.tex`): the transfer-function comma; 3.670 → 3.671; name the pitch axis; soften the padding sentence; correct the grid description; remove the stale "46 tests" and "compilation failed" sentences; break the Drive link.
3. **Carry the section 4.4 items into the proposal's appendices.** The I.3/I.6 sign and the I.1 contradiction are the ones a reviewer would notice first.
4. **Measure the landing cross-covariance once** in the existing Monte Carlo, then either add the term in `landing.py` or record why it is negligible.
5. **If this document is to remain the mission's maths reference, add the section 4.5 topics**, starting with glide and the lateral-force airspeed relation.
6. **Agree one suspension length and one reference-wind height** across the documents.

## 7. Evidence

Check scripts and their logs are in this session's scratch folder (`review/physics`, `review/navigation`, `review/core`, `review/implementation`, `review/main`). That folder is temporary; they have not been copied into the project.
