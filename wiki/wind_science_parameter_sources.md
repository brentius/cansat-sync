# Wind science: parameter sources

Collected 10 October 2026 for the wind-retrieval question "can the can's own lateral specific force measure (and remove) parachute glide bias?"; status cross-checked 13 October 2026. This is the research helper's extraction, in the same format as [navigation_parameter_sources.md](navigation_parameter_sources.md). Read the evidence codes before using a number: this session was badly limited by access (see "Access log"), so most rows are **[S]** or **[U]** and very few are **[P]**. This is dated source material, not a flight qualification or a measurement of the current CanSat. Current hardware is defined in [design_baseline.md](design_baseline.md) and the [flight computer BOM](flight_computer_bom.md); geometry and mass assumptions below are retained exactly for the derivations made on 10 October.

---

Evidence codes used in every row:
- **[P]** read directly in the primary document this session (PDF text extracted or page fetched)
- **[A]** read directly on the publisher/repository abstract or record page
- **[W]** read directly this session, but on a secondary page (e.g. Wikipedia, hobby site) that cites the primary
- **[S]** search-engine excerpt of the named source; the page itself could not be opened (403 / binary PDF / blocked)
- **[D]** derived by me from sourced or stated numbers (formula shown)
- **[U]** unverified recollection, not seen this session. Do not use without checking

### Access log (what was and was not actually read)

| Document | Outcome |
|---|---|
| DTIC (apps.dtic.mil): Knacke NWC TP 6575 (ADA247666), AEDC-TR-72-180 (AD0754920) | HTTP 403 on every attempt. archive.org mirrors blocked by the local proxy. NOT READ |
| AFFDL-TR-78-151 (everyspec.com) | Record page read; the file is a 96 MB download, not fetched. NOT READ |
| AMS journals (Hock & Franklin 1999 BAMS) | HTTP 403. NOT READ |
| u-blox MAX-M10S data sheet (u-blox.com and SparkFun copies) | PDF fetched but returned as compressed binary; no PDF text tool on this machine. Only search excerpts available |
| Bosch BMI088 data sheet | PDF is AES-encrypted; not readable this session. The accelerometer rows already in navigation_parameter_sources.md (read [P] on 8-9 Oct) are reused |
| NCAR ASPEN manual, Ingleby et al. AMT preprint, Jorgensen NASA report, Divaret et al. JFM 2014 | PDFs returned as binary or redirect loops. Search excerpts only |
| NCAR EOL "AVAPS Dropsondes" page | READ [P] |
| Wikipedia "Roughness length" (reproduces WMO-No. 8 table) | READ [W] |
| Martins & Wehmann 2024 abstract page | READ [A] |
| Nakka rocketry Appendix K (cross parachute) | READ [W] |

---

## 1. Side (normal) force on a short blunt cylinder in near-axial flow

**Historical calculation inputs (10 October study; not updated to the current budget):** d = 0.066 m, L = 0.115 m, L/d = 1.74, base area A_b = pi d^2/4 = 3.42e-3 m^2, planform area A_p = d L = 7.59e-3 m^2, V = 10.5 m/s, rho = 1.225 kg/m^3, m = 0.31 kg, Re_d = V d / nu = 10.5 x 0.066 / 1.5e-5 = 4.6e4 [D]. The current all-up expected mass is about 335 g, and the recommended drop-calibration target is about 11 m/s; those updated inputs have not been propagated through these derivations or the wind Monte Carlo. See [mass, power and descent budget](mass_power_budget.md).

Definition: k = F_side / (0.5 rho V v_side). With alpha = v_side / V and F_side = q A_b C_N, k = A_b C_N / alpha, so for a linear body k = A_b x C_N_alpha (per radian) [D].

What the simulation assumes: isotropic drag with CdS = 0.003 m^2 means k = 0.003 m^2, which is equivalent to C_N_alpha = 0.003 / 3.42e-3 = 0.88 /rad and an axial drag coefficient of 0.88 on base area [D]. It is the "drag vector stays aligned with the relative wind" model: C_N = C_D sin(alpha).

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Normal force of flat-faced cylinders, measured | Tested L/D 1.5 and 0.75 (others not visible), alpha 0-90 deg, subsonic and supersonic. Shortening from 1.5 to 0.75 raised axial force, lowered normal force, and the normal force became slightly negative at small alpha | Mach range not confirmed (subsonic lowest Mach unknown); Re far above ours | Rhudy & Baker, AEDC-TR-72-180 (1973), https://apps.dtic.mil/sti/pdfs/AD0754920.pdf [S]. Tables/figures NOT READ |
| Linearity at small angle (long cylinder) | Normal force linear in yaw angle below about 5 deg and dominated by lift; proportional to sin^2 for 10-90 deg | Long cylinder, Re about 2.4e4, not flat-faced short body | Divaret et al., J. Fluid Mech. 2014, https://perso.ensta-paris.fr/~doare/files/divaret_jfm2014.pdf [S] |
| Cross-flow drag coefficient of a circular cylinder | about 1.2 below Re about 3e5 | subcritical, infinite cylinder | MIT 2.154 lecture notes (via search) [S] |
| Slender-body potential slope | C_N_alpha = 2 /rad on base area | inviscid, pointed slender body; NOT established for a flat face with separated corners | Munk / Allen-Perkins, textbook result [U] |
| Jorgensen composite formula | C_N = (A_b/A) sin(2 alpha) cos(alpha/2) + eta C_dn (A_p/A) sin^2(alpha) | eta (finite-length factor) about 0.55 for L/d about 1.7 | Jorgensen, NASA TN D-6996 / TR R-474 [U]; the NTRS PDF fetched was unreadable scans |
| Axial drag coefficient, flat-faced cylinder in axial flow | about 0.85 at L/d = 2 (about 0.9 at L/d = 1, 1.17 for a disk) | subcritical Re, base-area reference | Hoerner, Fluid-Dynamic Drag, Ch. 3 [U]. Hoerner NOT READ this session |
| Centre of pressure | NOT FOUND. For a blunt flat-faced body the small-angle normal force is expected near the front face (destabilising about mid-length) | | AEDC-TR-72-180 has pitching-moment data, NOT READ; expectation is [U] |

### Bracket for k (side-force drag area) [D]

| Case | C_N_alpha (/rad, base area) | k (m^2) | Lateral accel per 1 m/s sideways airspeed | Basis |
|---|---|---|---|---|
| Low | 0 (possibly slightly negative) plus cross-flow term only | 5.0e-3 x alpha: 4.8e-4 at 1 m/s, 1.4e-3 at 3 m/s (quadratic, not linear) | 1.0 mg at 1 m/s; 9 mg total at 3 m/s | AEDC [S] trend toward zero/negative normal force for short flat-faced bodies; cross-flow k = eta C_dn A_p alpha with eta 0.55, C_dn 1.2 |
| Central (= simulation assumption) | 0.88 | 3.0e-3 | 6.3 mg | C_N = C_D sin(alpha) with C_D about 0.85-0.9 (Hoerner [U]). An assumption, not a measurement |
| High | 2.0 plus cross-flow term | 6.8e-3 (+0.5e-3 at 1 m/s, +1.4e-3 at 3 m/s) | 14.5 mg (15.5 mg at 1 m/s incl. cross-flow) | slender-body slope [U] plus Jorgensen cross-flow [U] |

Conversions: accel per m/s = 0.5 rho k V / m = 0.5 x 1.225 x k x 10.5 / 0.31; 1 mg = 9.81e-3 m/s^2 [D].

Assessment: 0.003 m^2 is the right order of magnitude as a mid-bracket value, but no source read this session supports a linear C_N_alpha of about 0.9 for a flat-faced L/d = 1.7 cylinder at Re 5e4. The one directly relevant data set (AEDC-TR-72-180) suggests the small-angle normal force of short flat-faced cylinders can be near zero or negative, so the gain could be anywhere from about 0 to about 2x the assumed value and may be nonlinear. The gain must be measured (wind tunnel, car-top rig, or a tilted drop) before the method is trusted. Reading the AEDC L/D = 1.5 curves is the single highest-value follow-up.

Derived side note [D]: in steady descent the total specific force is 1 g along true vertical, so a body-lateral reading of a_lat equals g sin(tilt of the sensor axis from the line of tension). 1 mg corresponds to 0.057 deg; 6.3 mg to 0.36 deg. The accelerometer-to-line-of-tension alignment (bridle symmetry, attachment offset) therefore has to be known or calibrated in flight to better than about 0.1 deg for a 1 m/s-class correction.

---

## 2. Parachute glide and oscillation

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Knacke Table 5-1/5-2 (NWC TP 6575, 1991) | NOT READ (DTIC 403, archive.org blocked) | | https://apps.dtic.mil/sti/pdfs/ADA247666.pdf |
| AFFDL-TR-78-151 (Ewing, Bixby, Knacke 1978) | NOT READ (96 MB file) | Tables 2.1-2.2 hold the same type data [U] | https://everyspec.com/USAF/USAF-General/AFFDL-TR-78-151_39040/ [A record page only] |
| Solid flat circular: oscillation, CD0 | +/-10 to +/-40 deg; CD0 0.75-0.80 | attributed to Knacke | Han Cheng et al., J. Industrial Textiles 2022, https://journals.sagepub.com/doi/10.1177/15280837211070023 [S]; matches my recollection of Knacke Table 5-1 [U] |
| "Square" parachutes: oscillation | +/-20 deg | as above | same [S] |
| Hemispherical / hemisflo: oscillation, CD0 | 10-15 deg; CD0 0.62-0.77 | search excerpt labels it hemisflo; I recall the same numbers for the solid hemispherical row | Tianwen-1 parachute paper, https://spj.science.org/doi/10.34133/2022/9805457 [S]; [U] for the Knacke attribution |
| Cross (cruciform): oscillation | about 3 deg (one design); a patent says 10-15 deg pendulum | conflict; Knacke value recalled as 0 to +/-3 deg | EUCASS recovery-system paper [S]; US patent 5839695 [S]; Knacke [U] |
| Cross: opening force coefficient | 1.2 (flat circular 1.7) | infinite mass | Nakka Appendix K citing Knacke, https://nakka-rocketry.net/Appendix-K.htm [W] |
| Cross: stability note | low canopy porosity leads to rotation; no spill hole needed | hobby author's statement | Nakka Appendix K [W] |
| Cross vs annular | cruciform showed higher drag coefficient and better stability than annular of equal projected area | student wind-tunnel study | DSTA (Singapore) report [S] |
| Conical | +/-10 to +/-30 deg; CD0 0.75-0.90 | | Knacke Table 5-1 [U] |
| Annular | below +/-6 deg; CD0 0.85-0.95 | | Knacke Table 5-1 [U] |
| Guide surface (ribbed / ribless) | 0 to +/-2 deg / 0 to +/-3 deg; CD0 0.28-0.42 / 0.30-0.34 | | Knacke Table 5-1 [U] |
| Flat ribbon / ringslot | 0 to +/-3 deg / 0 to +/-5 deg; CD0 0.45-0.50 / 0.56-0.65 | | Knacke Table 5-2 [U] |
| Ringsail / disk-gap-band | +/-5 to +/-10 deg / +/-10 to +/-15 deg | | Knacke Table 5-2 [U] |
| Cluster of three solid flat circulars | +/-7.1 deg | free-flight | NASA TN (1964) via search [S] |
| Trim angle of attack / glide ratio by type | NOT FOUND in any source reached. Knacke Sec. 5.2 discusses unstable canopies either oscillating or gliding at a stable trim angle [U] | | |
| Glide speed vs trim angle | v_glide = V_descent tan(alpha_trim): 1 m/s = 5.4 deg, 2 m/s = 10.8 deg, 3 m/s = 15.9 deg at 10.5 m/s | steady glide | [D] |
| Dropsonde parachute | NRD41: 20 x 20 cm; RD41: 30 x 30 cm (ribbon-delayed release). Construction not stated on the page; "square-cone" comes from Wikipedia | sonde 4.5 cm x 30.5 cm, 169 g | https://www.eol.ucar.edu/content/avaps-dropsondes [P]; Wikipedia "Dropsonde" [S] |
| Dropsonde parachute oscillation / glide figures | NOT FOUND | | |
| Spill hole on a model-rocket parachute | hole of 10% of canopy diameter vs 5%: drag-force standard deviation 3.9x larger for the 5% hole at 25 m/s; smaller hole gives higher drag | scale models, wind tunnel (UFMG); canopy type and size not in abstract | Martins & Wehmann, Rev. Bras. Fis. Tecnol. Apl. 11(2), 2024, https://periodicos.utfpr.edu.br/rbfta/article/view/18034 [A] |
| Small-scale (0.2-0.5 m) trim / glide data | NOT FOUND. NASA Langley tested 0.30 m cross, hemisflo and DGB models, but at Mach 0.3-1.2 in a forebody wake | | NTRS via search [S] |

Reading: every solid, low-porosity canopy type in the classic table has an oscillation band whose lower edge (10 deg) already exceeds the trim angle needed for a 1 m/s glide (5.4 deg). Only the slotted, guide-surface and cross types are quoted in the 0-3 deg band, equivalent to at most about 0.55 m/s at our descent rate [D]. These are large-canopy values; nothing found confirms them at 0.2-0.5 m.

---

## 3. What dropsonde and radiosonde processing does about parachute motion

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| xRD41 horizontal wind spec | range 0-200 m/s, uncertainty +/-0.5 m/s, resolution 0.01 m/s | NCAR sensor table | https://www.eol.ucar.edu/content/avaps-dropsondes [P] |
| Early GPS dropsonde wind accuracy | almost always better than 0.5 m/s, typically about 0.3 m/s | after GPS firmware fixes, 1997 | NOAA/AOML evaluation, https://www.aoml.noaa.gov/general/admin/fy97/jlf1q97.html [S] |
| Hock & Franklin 1999 | BAMS 80, 407-420. Wind resolution quoted by others as "order of 1 m/s" accuracy at 0.5 s sampling | paper NOT READ (403) | [S] via UWO thesis and NHC summary |
| Wind-finding equation | wind = sonde horizontal velocity minus an inertial term proportional to (fall speed / g) x d(sonde horizontal velocity)/dt | corrects the lag in responding to shear | Hock & Franklin 1999 [U] for the exact form; existence of the "inertial correction" in AVAPS processing confirmed by Ingleby et al. [S] |
| Field comparison | mean absolute wind-speed difference 1.5 m/s between RD41 and StreamSonde | Hurricane Nigel; two different instruments | NSF PAR record via search [S] |
| Sonde-to-parachute distance | about 1 m or less for a dropsonde vs 30-55 m string for a radiosonde, so dropsonde fall is smoother | | Ingleby et al., "On the quality of RS41 radiosonde descent data", AMT 2021 preprint, https://amt.copernicus.org/preprints/amt-2021-183/amt-2021-183.pdf [S] |
| Pendulum filtering (radiosonde) | descent wind errors depend strongly on the averaging window and are small when it matches the pendulum period; optimal descent filtering listed as future work | | Ingleby et al. author response [S] |
| ASPEN processing chain | dynamic correction of temperature and wind, u and v quality-controlled separately, then smoothing and sensor time-response correction | filter wavelengths NOT FOUND (manual PDF unreadable) | NOAA/HRD ASPEN document and EOL readme files via search [S]; manual at https://archive.eol.ucar.edu/docs/isf/facilities/software/aspen/Aspen%20Manual.pdf NOT READ |
| Numerical check of the wind equation | Hock-Franklin equations recover the mean profile; sonde velocity alone is inadequate in sharp shear; turbulence/shear-stress retrieval is sensitive to the low-pass filter and differentiation scheme | simulated sonde in prescribed wind | UWO thesis, https://ir.lib.uwo.ca/etd/529 [S] |
| Acknowledgement of steady glide bias | NOT FOUND in any source reached | | |
| Use of accelerometers, airspeed or attitude sensing to correct sonde winds | NOT FOUND in any source reached | absence of evidence only; the primary papers were not readable | |

Reading [D]: the operational inertial correction uses the time derivative of GPS velocity. A steady glide has zero horizontal acceleration, so that correction is blind to it by construction. This is consistent with the simulation finding and with no source mentioning glide. Using the body-frame lateral specific force instead would be a different observable; I found no published precedent.

---

## 4. u-blox MAX-M10S

Source A: "MAX-M10S Data sheet", u-blox, UBX-20035208. Revisions differ; the R06-era table and an earlier revision are both quoted below. https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf (current file metadata: created 2026-01-30). All rows are search excerpts; the PDF text could not be extracted this session, so table and page numbers are NOT confirmed.
Source B: "u-blox M10 SPG 5.10 Interface description", UBX-21035062 R02, 02-May-2022. https://cdn.sparkfun.com/assets/f/1/2/9/e/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf (NOT READ).

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Velocity accuracy | 0.05 m/s | 50% at 30 m/s for dynamic operation | A, performance table [S] |
| Operational limits | dynamics <= 4 g, altitude 80 000 m, velocity 500 m/s | assuming the Airborne 4 g platform | A [S] |
| Time pulse accuracy | 30 ns RMS, 60 ns 99% | | A [S] |
| Max navigation update rate, default configuration | 10 Hz GPS+GAL; 6 Hz GPS+GAL+GLO; 3 Hz GPS+GAL+BDS B1I; 8 Hz GPS+GAL+BDS B1C; 4 Hz GPS+GAL+BDS B1C+GLO | minimum 98% fix rate under typical conditions | A, later revision [S] |
| Max navigation update rate, high-performance configuration | 20 / 16 / 12 / 16 / 10 Hz for the same five combinations | needs a configuration change; higher baud rate and fewer messages advised | A, later revision [S] |
| Max navigation update rate, earlier revision | 10 Hz for most combinations, 5 Hz for GPS+GAL+BDS B1C+GLO | conflict with the row above; check the revision that matches the firmware on the purchased module | A, earlier revision (eicom-hosted copy) [S] |
| Dynamic platform models: horizontal velocity sanity limits | Airborne <1g: 100 m/s; <2g: 250 m/s; <4g: 500 m/s | quoted on a hobby forum from u-blox documentation, M8-era; verify against B or the M10 integration manual | arrmaforum post via search [S] |
| Airborne models: vertical velocity limit | 100 m/s for all three airborne models | M8 documentation | [U] |
| Configuration key | CFG-NAVSPG-DYNMODEL (values 6, 7, 8 for airborne <1g, <2g, <4g) | | B [U] for the values |
| Velocity latency / filtering under acceleration | NOT FOUND. The data sheet gives no velocity bandwidth or latency figure in any excerpt seen | the dynamic model sets the navigation filter's assumed process noise; no numbers published that I could find | |
| Heading accuracy, position accuracy | not extracted | | A NOT READ |

Reading [D]: 0.05 m/s (50%, steady 30 m/s) is small against a 1-3 m/s glide bias, so GNSS velocity noise is not the limiting term. The unknowns are filter lag during pendulum motion and the true update rate for the chosen constellation set (10 Hz needs at most GPS+GAL in the default mode).

---

## 5. Bosch BMI088 accelerometer

The data sheet PDF (BST-BMI088-DS001, rev 1.9) is encrypted and was not readable this session. The rows marked "nav file" were read [P] on 8-9 October and are in [navigation_parameter_sources.md](navigation_parameter_sources.md) Sec. 1 (Table 4, p. 9 of the data sheet); they are restated here only because the resolvability estimate depends on them. https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Zero-g offset | 20 mg typ | 25 C, +/-6 g range; product page says +/-20 mg typ over lifetime; one press article says up to +/-30 mg | nav file [P]; Bosch product page [S]; allaboutcircuits [S] |
| Offset temperature drift (TCO) | below 0.2 mg/K typ | | nav file [P]; product page and flyer agree [S] |
| Noise density | 160 ug/rtHz X,Y; 190 ug/rtHz Z | typ, +/-3 g, normal mode | nav file [P] |
| Noise density, other ranges | 175 ug/rtHz (range unstated) on product page; 230 ug/rtHz at +/-24 g in flyer | conflict | [S] |
| Cross-axis sensitivity | 0.5 % typ | | nav file [P] |
| Alignment error | 0.5 deg typ | | nav file [P] |
| Nonlinearity | 0.5 %FS typ | best-fit line, +/-3 g | nav file [P] |
| Sensitivity temperature drift | 0.002 %/K typ | | nav file [P] |
| Sensitivity tolerance | NOT FOUND (no such row in rev 1.9, per the earlier read) | | nav file [P] |
| Offset after soldering / board stress, long-term offset stability after calibration | NOT FOUND | | |

### Is a 1 mg lateral signal resolvable after a static calibration? [D]

| Error term | Size on the lateral axis | Removable by static calibration? |
|---|---|---|
| White noise | 160 ug/rtHz x sqrt(1 Hz) = 0.16 mg rms in a 1 Hz bandwidth; 0.05 mg in 0.1 Hz | not needed; averaging is enough |
| Zero-g offset | 20 mg typ | yes (six-position), but only at the calibration temperature |
| Offset temperature drift | 0.2 mg/K: 2 mg for a 10 K change between calibration and flight | only with a temperature sweep; otherwise this alone exceeds 1 mg |
| Cross-axis (0.5 %) of the 1 g axial load | 5 mg | yes, it is a fixed matrix term |
| Mounting/alignment 0.5 deg of the 1 g axial load | 8.7 mg | yes for sensor-to-board; the board-to-line-of-tension angle is not (see Sec. 1 note: 0.057 deg per mg) |
| Nonlinearity 0.5 %FS at +/-3 g | up to 15 mg full-scale bow; near 0 g lateral the local error is much smaller but unspecified | partly |

Conclusion: in this historical calculation noise is not the problem (0.05-0.16 mg); bias stability is. A 1 mg absolute lateral measurement needs temperature-compensated offsets (to 5 K or better, or a measured TCO) and knowledge of the tension direction to about 0.06 deg; neither is guaranteed by the data sheet. This establishes a candidate measurement and calibration burden only. A differential scheme (signal that reverses when the can rotates about its axis, so fixed offsets cancel) could relax both, since the can's spin modulates a wind-fixed glide direction in the body frame; no flight evidence or calibrated current-can force law exists.

---

## 6. Roughness length from a single short profile

| Parameter | Value | Conditions | Source |
|---|---|---|---|
| Terrain classification (z0) | 0.03 m open flat grass, few isolated obstacles; 0.10 m low crops, occasional large obstacles (x/H > 20); 0.25 m high crops, scattered obstacles (15 < x/H < 20); 0.5 m parkland/bushes, numerous obstacles (x/H about 10); 1.0 m suburb/forest | x = obstacle spacing, H = obstacle height | WMO-No. 8 Guide, p. I.5-13 (Davenport classification as revised by Wieringa), reproduced at https://en.wikipedia.org/wiki/Roughness_length [W] |
| UK farmland with hedgerows | 0.1-0.25 m by the table above, depending on hedge spacing; about 0.2 m for 8 m hedgerows 250 m apart, 0.1 m for 500 m spacing | wind-industry roughness-class table | [D] from the WMO table; Windographer roughness-class reference via search [S] |
| Wieringa 1993 | Boundary-Layer Meteorol. 63, 323-363; about fifty documented experiments; popular terrain tables underestimate z0 by about a factor of two | paper NOT READ | Springer abstract, https://link.springer.com/doi/10.1007/BF00705357 [S] |
| Spread of z0 estimates | discrepancies up to two orders of magnitude cited to Wieringa 1993 | | lidar-inference study on arXiv via search [S] |
| Averaging time | 95% credible interval on z0 shrinks by 46% on average going from 10 to 30 minute averages; flow integral timescale lies between the two | synthetic-observation inference, not field data | same arXiv study [S] |
| Averaging time, standard practice | 10 to 30 minute means; WMO standard wind average is 10 minutes | | Stull 1988, Kaimal & Finnigan 1994, WMO-No. 8 [U] |
| Number of levels / lowest level | scatter of fitted log-law parameters grows as the lowest measurement height rises; low-level data are essential | lidar calibration study | Wind Energy Science papers via search [S] |
| Fit range over obstacles | lower limit of the inertial sublayer about 2 (some say 3) times roughness-element height; a displacement height is needed | | Brutsaert & Kustas 1987, quoted via search [S] |
| Fit method | u* = kappa x slope of u vs ln z; z0 = exp(-intercept/slope); fitting u vs ln z and ln z vs u weight levels differently | neutral stability assumed | search excerpt of a review exchange [S] |
| Quoted uncertainty of z0 from a single profile | NOT FOUND as a number in any source reached | | |

Reading [D]: the can falls 300 m in about 29 s and crosses the layer where a log law could hold (roughly the lowest 50-100 m, above 2-3 hedge heights, say 10-20 m) in about 3-8 s, sampling each height once for about 0.1 s per metre. The literature's requirement is 10-30 minutes of averaging at each of several fixed levels. A single descent therefore gives one instantaneous realisation with gust-scale errors of order the turbulence intensity (not averaged at all), and the stability correction is unknown. A z0 from one profile should be reported, at most, as an order-of-magnitude class (0.03 / 0.1 / 0.25 / 0.5 m) with a factor of several uncertainty, and compared with the WMO class expected for the site; u* is somewhat better constrained than z0 because it depends on the slope only. The later [end-to-end simulation](wind_retrieval_end_to_end.md) found that its synthetic roughness fit improves when constrained by an independent ground anemometer near 6 m; that is a simulated method result, not an existing measurement or a substitute for a calibrated field reference.

---

## Caveats and items not sourced

1. No primary aerodynamic data for a flat-faced L/d about 1.7 cylinder was read. AEDC-TR-72-180 exists and covers the geometry (L/D 1.5 and 0.75 at least); its curves are the key missing input. The k bracket in Sec. 1 rests on recalled textbook models.
2. Knacke Table 5-1 and AFFDL-TR-78-151 were again not opened. Type-by-type oscillation values other than flat circular, square, hemisflo and one cross design are recollection [U].
3. No trim angle of attack or glide ratio by canopy type, and no small-scale (0.2-0.5 m) stability data, were found.
4. Hock & Franklin 1999, Franklin et al. 2003 and the ASPEN manual were not read; filter wavelengths and the exact wind equation are not sourced. No evidence found for or against any published use of sonde accelerometers.
5. MAX-M10S numbers are search excerpts with a revision conflict on update rate; dynamic-model limits are from a forum quote and recollection. Velocity latency is not published in anything seen.
6. BMI088 accelerometer sensitivity tolerance, post-solder offset shift and post-calibration bias stability are not in the data sheet as far as the earlier read found.
7. No quantitative uncertainty for z0 or u* from a single instantaneous profile was found.
