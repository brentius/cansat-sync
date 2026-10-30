# Mass, power and descent budget for the current hardware

Prepared 11 October 2026; current-status cross-check 13 October 2026. This is the current calculated budget against the hardware in [design_baseline.md](design_baseline.md) and the authoritative [flight computer BOM](flight_computer_bom.md): Pi Zero 2 W, STM32F405, BMI088, MS5611, GA10K3MCD1 probe circuit using ADS1115/OPA333/150 kΩ, IMX296 GS camera with Edmund #34-711 lens and #53-675 adapter, protected 2S pack, E22 radio and MAX-M10S. Model: `sim/systems/budget.py`; tests: `sim/systems/test_budget.py` (44 passing); run script: `sim/systems/run_budget.py`; results, tables and charts: `sim/systems/results/2026-10-11-budget/`. This replaces the 1 October sizing (`sim/systems/run_sizing.py`, `results/sizing.json`), which was written for an Orange Pi, an OV9281 camera and one 18650 cell and is left in place unchanged.

**Nothing in this report has been weighed or metered.** Every number is one of four kinds, and the tables say which:

| Tag | Meaning |
|---|---|
| measured | Weighed or metered on the real article. **There are none.** |
| datasheet / vendor | Manufacturer document or seller listing, with URL. The "basis" column says whether the page was opened this session (*primary read*), only quoted by a web search (*search excerpt*), remembered (*recalled*) or copied from the project BOM. |
| calculated | Arithmetic from stated inputs (for example board area × thickness × density). The inputs are often themselves assumptions. |
| assumed | An engineering allowance with no source. |

The 335 g expected mass and about 11 m/s recommended drop-calibration target are current budget outputs/recommendations, not physical measurements or frozen flight configuration. This report's canopy-area calculations below retain the earlier 10.5 m/s design point; they have not been rescaled to 11 m/s. The 2025-26 rules used here also need confirmation against the 2026-27 rules.

## 1. The answer

| Requirement (2025-26 rules) | Result | Margin | How sure |
|---|---|---|---|
| Mass 300-350 g | **Probably inside, not securely.** Expected 335 g; sum of typical values 324 g. | 15 g (4 %) below the limit at the expected value. 90 % of outcomes lie between 320 and 351 g. Chance of exceeding 350 g: **6 %** if item errors are independent, **15 %** if shell, caps and frame err together. Chance of being under 300 g: under 0.1 %. | Low. Of the 324 g typical, 89 g is assumed, 75 g is calculated from assumed dimensions, 144 g is from seller listings and 16 g from datasheets. No enclosure design exists; the three printed parts carry 55 % of the variance. If every item came out at its high value the total would be 431 g. |
| Powered for 3 h | **Yes in the model, with a large margin**, with Linux/service booted and the Pi powered but image processing idle throughout the wait. 3 h on the pad with the Pi idle, the flight and a 1 h beacon use 4.3 Wh of 19.6 Wh usable. | 78 % of usable energy left in the typical case; 45 % left with every load high and the pack cold and worn. | Medium. The conclusion survives a factor of two on any single load. The Pi and camera figures are other people's bench reports and an assumption, not measurements of this build. |
| Descent rate at least 10 m/s | **Not established; the historical 10.5 m/s sizing point is too close to the floor.** | A canopy that really does 10.5 m/s in standard air does 10.1 m/s on a cold, high-pressure March day. A canopy cut from a handbook drag coefficient and never dropped has a 6-21 % chance of falling below 10 m/s, depending on type. | Low until drop tests. Current recommendation: trim to about 11.0 m/s measured. |
| Peak currents | **Inside every limit checked**, smallest ratio 1.8 (3.3 V buck). | Section 5. | Medium. |

Three consequences:

1. **The mass problem is real but smaller than the status file's arithmetic suggested at the typical value, and worse at the high end.** `BOTS/PLANS/STATUS.md` estimated about 290 g typical and about 350 g high. With every item of the current design listed, the typical sum is 324 g and the all-high sum is 431 g. The old sizing had one carrier board and no ribbon link, allowed 5-12 g for the GPS module and its antenna together (the antenna alone is 13 g), and had no lens adapter, master switch or power connector.
2. **The battery is the cheapest place to find mass.** It is 100 g of a 324 g can and its 24 Wh is about 2 times (worst corner) to 5 times (typical) what three hours with the Pi idle, a flight and an hour's beacon need (section 5). The pack was sized for a 6 W average load; the estimate here is 1.3 W while waiting and 4.5 W for the 40 seconds of descent.
3. **No ballast is expected.** The old sizing predicted 92 g of ballast; that is gone.

## 2. Rules

Source: *UK CanSat Competition Guidelines 2025-2026*, section "CanSat Requirements", read on 11 October 2026 from the project Drive folder (file id `1LubKeC729i9NbmgHwG-wzu4gwhDO1PfJ`) through the Drive connector's text extraction. **The 2026-27 rules are not in the folder; everything below is last year's text and may change.**

| No. | Exact text |
|---|---|
| 1 | "All the components of the CanSat must fit inside a standard soda can (115 mm height and 66 mm diameter), with the exception of the parachute. An exemption can be made for radio antennas and GPS antennas, which can be mounted externally (on the top or bottom of the can, not on the sides), based on the design" |
| 2 | "The antennas, transducers and other elements of the CanSat cannot extend beyond the can's diameter until it has left the launch vehicle" |
| 3 | "The mass of the CanSat must be between 300 g and 350 g. CanSats that are lighter must take additional ballast with them to reach the 300 g mass limit required" |
| 5 | "The CanSat must be powered by a battery and/or solar panels. It must be possible for the systems to be switched on for three continuous hours" |
| 6 | "The battery must be easily accessible in case it has to be replaced or recharged in the field." |
| 7 | "The CanSat must have an easily accessible master power switch" |
| 8 | "The CanSat should have a recovery system, such as a parachute, which is able to be reused after launch. It is recommended to use bright coloured fabric, which will facilitate recovery of the CanSat after landing. It is forbidden to convert the CanSat into a motorised drone or motorised paraglider on ejection from the rocket." |
| 9 | "The parachute connection must be able to withstand up to 50N of force. The strength of the parachute must be tested, to give confidence that the system will operate nominally." |
| 10 | "A descent rate of around 10-15 ms-1 is recommended. At a minimum the descent speed should be 10 ms-1 but please be aware that the launch provider may request that the parachute is replaced by a smaller one. The launch provider checks all parachutes before launch and will insist on replacement of incorrectly sized parachutes (spare parachutes are provided at launches)." |
| 11 | "The CanSat must be able to withstand an acceleration of up to 20g for rocket launch" |
| 13 | "The total budget of the CanSat should not exceed £400. This does not include ground support equipment, such as laptops, power supplies, antennas. This includes the cost of the basic boxed kit if provided to your school, college or community group, which costs £50." |

Elsewhere the guidelines describe the launch as "around 10G to 20G of acceleration ... for a short 0.5 to 1.5 second thrust duration, followed by around a 5 to 7 second 'coast' before the rocket reaches between 300 – 400m peak altitude".

Reading of the rules against the project documents:

- **Does the parachute count toward 350 g?** The rules do not say. The parachute is excepted from the *dimension* rule (1) only; the mass rule (3) has no exception. This report counts it. If the organisers confirm it is excluded, the expected mass falls by about 14 g to 321.5 g and the chance of exceeding 350 g to 0.1 %. **Ask the organisers.**
- **Descent rate.** The proposal (section 6) says "Guidelines require at least 10 m/s and recommend 10-15 m/s". That agrees with rule 10, with one nuance: the wording is "should", and the only enforcement described is the launch provider replacing a canopy it judges wrongly sized, in the direction of *smaller* (faster). There is no stated upper limit; 15 m/s is the top of the recommended range.
- **Three hours.** The rule says it must be *possible* for the systems to be switched on for three continuous hours. It does not say at flight load. The BOM's "three hours of continuous operation" is consistent with it.
- **Budget.** The £50 kit cost counts toward the £400 if a kit was supplied; the BOM does not mention this. Cost is not assessed in this report.
- **Battery access (rule 6)** and **master switch (rule 7)** are layout requirements the enclosure does not yet address; the BOM puts a welded pack inside a closed can.
- **Antennas** may be mounted outside the can on the top or bottom. The BOM keeps both inside; the exemption is available if the top of the can gets crowded.
- No disagreement was found between the rules and the limits quoted in the BOM "Mission basis" (66 mm × 115 mm, 300-350 g, 20 g, 3 h, £400) or the proposal section 6 (50 N, 20 g, 3 h, £400).

## 3. Mass

### 3.1 Itemised bill

Values in grams. Source and basis as defined above. Full notes, URLs and formula inputs are in `results/2026-10-11-budget/mass_items.csv`.

| Item | Low | Typical | High | Source (basis) | Where the number comes from |
|---|---:|---:|---:|---|---|
| KeepPower 2S1P NCR18650GA pack | 97 | 100 | 106 | vendor (primary read) | Maker's page: "approx. 100 g" ([KeepPower](https://keeppower.com.cn/products_detail.php?id=863)). Cells are up to 48.0-49.5 g each (search excerpt). |
| Global Shutter Camera body | 30 | 34 | 41 | vendor (search excerpt) | 34 g bare, 41 g with C-CS adapter and dust cap ([Pimoroni](https://shop.pimoroni.com/en-eu/products/raspberry-pi-global-shutter-camera)). Low assumes the tripod mount is in the 34 g and comes off: unverified. |
| Printed shell, cylindrical wall | 21.2 | 32.1 | 44.9 | calculated | π(r_o² − r_i²)·L·ρ. Low: 64 mm OD, 100 mm, 1.0 mm wall, ASA 1.07 g/cm³. Typical: 65 mm, 105 mm, 1.2 mm, PETG 1.27. High: 65.5 mm, 110 mm, 1.6 mm, PETG. All dimensions assumed; there is no CAD. |
| Internal frame (battery cradle, Pi rail, camera mount) | 10 | 18 | 30 | assumed | Not designed. |
| Printed top and bottom caps | 9.2 | 15.1 | 23.0 | calculated | Two discs 1.5 / 2.0 / 3.0 mm thick less a 30 mm window opening; same materials as the shell. Assumed dimensions. |
| GNSS antenna, Taoglas AGGP.25F.07.0060A | 12 | 13 | 14 | datasheet (primary read) | "Weight 13 g" with 60 mm cable ([Taoglas](https://www.taoglas.com/datasheets/AGGP.25F.07.0060A.pdf)). |
| Lens, Edmund #34-711 | 6.5 | 12 | 18 | assumed | No mass found (page refused the request). Other 2.5 mm M12 lenses: 6.4 g Watec, 11.5 g Videology (search excerpts). This one is 18 mm long, 20 mm across. |
| MAIN PCB, bare | 9.2 | 10.0 | 11.2 | calculated | 44 × 64 mm (read from the KiCad board file), 1.6 mm, four copper layers at 35 µm and 40-80 % fill, FR4 1.80-1.95 g/cm³. |
| Raspberry Pi Zero 2 W | 9 | 10 | 11 | vendor (search excerpt) | Resellers list 9-10 g ([example](https://raspberrypi.dk/en/product/raspberry-pi-zero-2-w/?clang=en)). |
| RF PCB, bare | 5.1 | 9.4 | 10.7 | calculated | **No outline exists** (`cansat_radio.kicad_pcb` is empty). Assumed circular, 56 / 60 / 62 mm, 1.0 / 1.6 / 1.6 mm, two layers. |
| XT30 pair, master toggle switch, power wire | 6 | 9 | 13 | assumed | |
| C-to-M12 adapter #53-675 and lock nut | 4 | 8 | 13 | assumed | No published mass found. |
| Screws, standoffs, inserts | 4 | 8 | 15 | assumed | Counts and materials undefined. |
| MAIN-RF ribbon: two IDC sockets and cable | 4.5 | 7 | 10 | assumed | |
| MAIN PCB fitted parts and solder | 4.5 | 5.9 | 8.0 | assumed | 124 footprints, itemised by type in the code; includes the ~2.5 g shrouded header. |
| RF PCB fitted parts and solder | 3.6 | 5.0 | 6.5 | assumed | E22 1.5 ± 0.1 g (search excerpt), MAX-M10S, header, U.FL. |
| Adhesive, tape, foam, ties, heat-shrink | 2 | 5 | 10 | assumed | |
| Parachute canopy fabric | 2.5 | 4.1 | 9.2 | calculated | 0.055 / 0.070 / 0.110 m² × 38 / 45 / 60 g/m² × seam factor 1.2 / 1.3 / 1.4. Inputs assumed. |
| Swivel, ring, attachment hardware | 2 | 4 | 8 | assumed | Must hold 50 N. |
| Thermistor probe and shield | 1 | 3 | 6 | assumed | Shield not designed. |
| Parachute lines | 1.1 | 2.9 | 7.2 | calculated | 6-8 lines × 0.35-0.60 m × 0.5-1.5 g/m. Inputs assumed. |
| Pi, trigger and probe harnesses | 1.5 | 2.5 | 4 | assumed | |
| LoRa antenna, Taoglas FXP895 | 1.5 | 2 | 3 | datasheet (project BOM) | 2 g as the BOM quotes it; not re-read. |
| Bottom window | 1.3 | 1.9 | 4.5 | calculated | 30-40 mm disc, 1.5-3 mm, acrylic or polycarbonate. Assumed dimensions. |
| CSI cable | 1 | 1.5 | 2.5 | assumed | |
| Buzzer CMT-8504-100-SMT | 0.55 | 0.6 | 0.65 | datasheet (primary read) | "Weight 0.6" ([Same Sky](https://www.sameskydevices.com/product/resource/cmt-8504-100-smt-tr.pdf)). |
| microSD card | 0.25 | 0.3 | 0.5 | assumed | |
| **Straight sum** | **250.4** | **324.2** | **430.8** | | Parachute items (11 g typical) included. Without them: 244.8 / 313.2 / 406.4. |

Typical mass by kind of evidence: assumed 89.2 g, calculated 75.4 g, vendor 144.0 g, datasheet 15.6 g, **measured 0 g**.

The five heaviest items are the battery (100 g), camera body (34 g), shell (32 g), internal frame (18 g) and caps (15 g). Chart: `results/2026-10-11-budget/mass_items.png`.

### 3.2 Uncertainty

Each item is drawn from a **triangular distribution** on (low, typical, high). The reason for that choice: for most items all that is known is a floor, a best guess and a ceiling; the triangle is bounded, so it cannot produce a negative mass or a 200 g battery, and it carries the skew that most of these items have (the high side is further from typical than the low side). It is a statement of ignorance, not a measured spread. For independent items the mean and variance of the sum are known in closed form (mean = Σ(l+t+h)/3; variance = Σ(l²+t²+h²−lt−lh−th)/18), and the Monte Carlo reproduces both to within sampling error (tested).

| Case | Mean | σ | 5 % | 50 % | 95 % | P(> 350 g) | P(< 300 g) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Analytic, independent | 335.1 | 9.3 | | | | | |
| Monte Carlo, independent (200,000 draws) | 335.2 | 9.3 | 320.0 | 335.1 | 350.6 | 5.7 % | 0.004 % |
| Monte Carlo, shell + caps + frame fully correlated | 335.2 | 13.3 | 313.7 | 334.8 | 357.6 | 14.5 % | 0.10 % |
| Monte Carlo, parachute not counted | 321.5 | 9.0 | 306.8 | 321.4 | 336.4 | 0.08 % | 0.7 % |

The mean (335 g) is above the typical sum (324 g) because the ranges are skewed upward; 335 g is the better single estimate. The correlated row is the more honest one for the structure: one decision (wall thickness, filament, how much frame) moves all three printed parts the same way. Chart: `mass_total_distribution.png`.

Which items carry the uncertainty (share of the independent-sum variance):

| Item | σ (g) | Share |
|---|---:|---:|
| Printed shell | 4.8 | 27 % |
| Internal frame | 4.1 | 19 % |
| End caps | 2.8 | 9 % |
| Lens | 2.3 | 6 % |
| Camera body | 2.3 | 6 % |
| Fasteners | 2.3 | 6 % |
| Battery pack | 1.9 | 4 % |
| Lens adapter | 1.8 | 4 % |

The enclosure is 55 % of the variance and does not exist yet. The optical stack (camera body, lens, adapter) is another 16 %.

### 3.3 Does it breach the limit?

- **Typical values: no.** 324 g (sum) or 335 g (expected), inside 300-350 g.
- **High case: yes, plainly.** 431 g if everything is high. That corner is not credible as a whole (it needs 27 items all at their ceilings), but the 95th percentile is 351-358 g, which is over the limit.
- **A design decision that should be made now:** hold a mass allocation per subsystem and treat 350 g as a limit with no contingency left. A sensible target is 330 g expected with the options below held in reserve.

Mass-saving options. **All savings are assumed** and none has been checked for fit or function; they are not additive where they touch the same part.

| Option | Saving (g) | Cost or risk |
|---|---:|---|
| Board-level IMX296 M12 camera instead of the metal-bodied official camera, adapter and lock nut | 20-35 (typ. 28) | Removes the 34 g housing. Needs a module with a documented external-trigger input; the BOM chose the official camera for exactly that. |
| 2S pack of 18500-size cells (about 2 Ah, 14.7 Wh nominal) instead of 2S 18650 | 25-35 (typ. 30) | Still covers 3 h with the Pi idle in the worst corner (needs 13.0 Wh nominal); does not cover 3 h with the Pi working. Cell and pack sourcing unverified. |
| Passive ceramic patch on the RF board instead of the 13 g active patch antenna | 6-10 (typ. 8) | MAX-M10S has an internal LNA; needs ground-plane area and an RF check. |
| Mezzanine or flat-flex MAIN-RF link instead of shrouded headers, IDC sockets and ribbon | 6-11 (typ. 8) | PCB change on both boards while they are still open. |
| 1.0 mm boards instead of 1.6 mm | 5-7 (typ. 6) | Stiffness under 20 g; connector retention. |
| 1.0 mm ASA shell instead of 1.2 mm PETG | 8-14 (typ. 11) | Stiffness and the 20 g case. |
| Lighter M12 lens (6-7 g class) | 3-9 (typ. 5) | Optical requalification; also cheaper. |
| Slide switch instead of toggle; nylon fasteners | 3-8 (typ. 5) | Switch current rating. |

The first two together are worth about 55-60 g and would move the expected mass to roughly 280 g, at which point ballast returns and the design has real contingency. Either one alone moves the expected mass to about 305-307 g and the chance of exceeding 350 g to well under 1 %.

## 4. Power: loads

Loads at the regulated rails while active. Radio transmit power was not configured anywhere in `code/` when this budget was written; the candidate firmware now sets **+14 dBm** (`APP_RADIO_POWER_DBM` in `code/firmware/stm32/app/config.h`, 29 October cross-check). The BOM says to start at +14 dBm, and the range below runs from that to the module's +22 dBm maximum. Full table: `power_loads.csv`.

| Load | Rail | Low | Typical | High | Source (basis) |
|---|---|---:|---:|---:|---|
| STM32F405 at 168 MHz | 3.3 V | 40 mA | 60 mA | 93 mA | datasheet (recalled; ST's page quotes 238 µA/MHz = 40 mA) |
| BMI088 | 3.3 V | 5.0 mA | 5.2 mA | 5.5 mA | datasheet (recalled) |
| MS5611 at ~50 Hz | 3.3 V | 0.2 mA | 0.9 mA | 1.4 mA | datasheet (recalled) |
| ADS1115, OPA333, LDO, divider | 3.3 V | 0.2 mA | 0.3 mA | 0.5 mA | assumed |
| W25Q128 while programming | 3.3 V | 15 mA | 20 mA | 25 mA | datasheet (recalled); active 2-30 % of the time |
| LEDs, pull-ups, buffers | 3.3 V | 0.5 mA | 2 mA | 9 mA | calculated: (3.3 − 1.9) V / 330 Ω = 4.2 mA per LED |
| MAX-M10S tracking | 3.3 V | 6 mA | 8 mA | 10 mA | datasheet (search excerpt: 25 mW continuous tracking) |
| MAX-M10S acquisition | 3.3 V | 8 mA | 12 mA | 25 mA | datasheet (search excerpt, via a reseller summary) |
| Active GNSS antenna | 3.3 V | 8 mA | 10 mA | 13 mA | datasheet (primary read: 10 mA at 3.0 V) |
| E22 between packets | 3.3 V | 0.6 mA | 2 mA | 7 mA | datasheet (search excerpt: receive 6.8 mA) |
| E22 transmitting | 3.3 V | 45 mA | 60 mA | 119 mA | datasheet (search excerpt: 118-119 mA at +22 dBm; +14 dBm figure recalled) |
| Buzzer while sounding | 3.3 V | 92 mA | 110 mA | 138 mA | calculated: 0.5 × 3.3 V / 18, 15, 12 Ω coil (datasheet, primary read); coil inductance ignored, so high |
| Pi Zero 2 W idle, radios off | 5.1 V | 0.50 W | 0.65 W | 1.20 W | vendor (search excerpts of bench reports: 100, 120, 280 mA) |
| Pi Zero 2 W, four cores loaded | 5.1 V | 2.2 W | 2.8 W | 3.3 W | vendor (search excerpts: 500-600 mA) |
| Camera powered, not streaming | 5.1 V | 0.05 W | 0.2 W | 0.4 W | assumed: no published figure found |
| Camera streaming at 5 Hz | 5.1 V | 0.4 W | 0.8 W | 1.2 W | assumed: no published figure found |
| microSD writing frames | 5.1 V | 0.05 W | 0.15 W | 0.40 W | assumed |

**Radio duty cycle** comes from the flight code, not from an assumption: `lora.time_on_air` gives 82.2 ms for the 40-byte packet at SF7 / 125 kHz, so 8.2 % at 1 Hz in flight. On the pad `DownlinkScheduler` gives the same 1 s interval under a 10 % duty limit (the 869.4-869.65 MHz option in the BOM) and a 9.5 s interval (0.86 %) under a 1 % limit. The difference is 16 mW at the battery and does not matter to endurance. The firmware's PAD/LANDED interval (`APP_RADIO_GROUND_INTERVAL_S`) was 10 s until the 29 October cross-check and is now 1 s, matching this budget.

**Converters** (from the 2S input; best / typical / worst): TPS62130 to 5.1 V 95 / 92 / 88 %; TPS62162 to 3.3 V 90 / 87 / 82 %. These are **recalled from the datasheet efficiency curves**, which are images and could not be re-read this session; they need checking against the curves at 6-8.4 V in and the currents below. Series resistance is assumed: 0.03 / 0.05 / 0.10 Ω for the load switch and Pi harness (TPS22965 is 16 mΩ typical) and 0.06 / 0.10 / 0.16 Ω for switch, connector, fuse, reverse-protection MOSFET and wiring.

### Load by phase

"Low" pairs low loads with the best converters and pack; "high" pairs high loads with the worst of each. These are stacked corners, not probabilities. Table: `power_phases.csv`; chart: `endurance_by_phase.png`.

| Phase | 3.3 V rail (mA) | 5.1 V rail (W) | At the battery (W), low / typ / high | Hours if held in this phase, low / typ / high |
|---|---:|---:|---:|---:|
| Pad, Pi off | 65 / 94 / 150 | 0 | 0.24 / 0.36 / 0.60 | 97 / 55 / 24 |
| Pad, Pi idle, camera idle | 65 / 94 / 150 | 0.55 / 0.85 / 1.60 | 0.82 / 1.29 / 2.45 | 28 / 15 / 6.0 |
| Pad, Pi and camera working | 65 / 94 / 150 | 2.65 / 3.75 / 4.90 | 3.05 / 4.50 / 6.40 | 7.6 / 4.3 / 2.3 |
| Ascent (10 s) | 71 / 103 / 171 | 0.90 / 1.45 / 2.40 | 1.21 / 1.98 / 3.48 | 19 / 9.9 / 4.2 |
| Descent, camera at 5 Hz and matching (40 s) | 71 / 103 / 171 | 2.65 / 3.75 / 4.90 | 3.07 / 4.54 / 6.49 | 7.5 / 4.3 / 2.3 |
| Post-landing beacon, buzzer 10 % | 74 / 105 / 163 | 0 | 0.27 / 0.40 / 0.66 | 86 / 49 / 22 |

The 5.1 V rail (Pi, camera, card) is about 90 % of the descent load at the battery. Everything on the 3.3 V rail together is about a third of a watt.

## 5. Power: endurance and peak current

### Pack

KeepPower 2S1P NCR18650GA, from the maker's page (primary read): nominal 3500 mAh, **minimum 3350 mAh**, 7.2 V, maximum discharge 5.2 A, over-current protection 10-20 A, discharge −20 to +60 °C. Nominal energy 24.1 Wh at the minimum capacity. The reseller page the BOM links (3.3 Ah, 5 A) refused the request and was not read.

Derating, **all assumed** (best / typical / worst):

| Factor | Best | Typical | Worst | Basis |
|---|---:|---:|---:|---|
| Capacity (Ah) | 3.45 | 3.35 | 3.30 | maker's minimum as typical; reseller's figure as worst |
| Cold | 0.95 | 0.88 | 0.75 | March launch: 0-10 °C typical, worst near −10 °C. The cell datasheet has discharge curves at 25, 10, 0 and −10 °C at 4 A, but they are graphs that could not be read. These are generic lithium-ion figures for a 0.1-0.2 C load, not this cell's. |
| State of charge and wear | 1.00 | 0.97 | 0.90 | not topped up, self-discharge, cycle wear |
| Reserve before the protection board trips | 0.98 | 0.95 | 0.92 | shut the Pi down cleanly first |
| **Usable fraction** | 0.93 | 0.81 | 0.62 | product |
| **Usable energy (Wh)** | 23.1 | 19.6 | 14.8 | |

### Scenarios

Each is 3 h powered before launch, 10 s ascent, 40 s descent and a 1 h recovery beacon. Table: `power_scenarios.csv`.

| Scenario | Used (Wh), low / typ / high | Margin left, low / typ / high | Nominal pack energy that would be enough | Monte Carlo: P(runs out) |
|---|---:|---:|---:|---:|
| Pi on (idle) for the whole 3 h | 2.8 / 4.3 / 8.1 | 88 % / 78 % / 45 % | 3.0 / 5.3 / 13.0 Wh | 0 in 100,000 |
| Historical off alternative, switched on 10 minutes before launch — not selected flight policy | 1.1 / 1.7 / 2.9 | 95 % / 91 % / 81 % | 1.2 / 2.1 / 4.6 Wh | 0 in 100,000 |
| Pi and camera working flat out for the whole 3 h | 9.5 / 14.0 / 20.0 | 59 % / 29 % / **−35 %** | 10.1 / 17.2 / 32.1 Wh | 0.04 % |

After the first scenario the typical remaining energy would run the beacon for about 38 more hours.

**What breaks it.** Only one thing in this model: running the image pipeline at full load for the whole wait, with high-end loads, a cold worn pack and worst-case converters all at once (20.0 Wh needed, 14.8 Wh usable). The Monte Carlo, which does not stack every corner, gives the Pi-working case a median endurance of 4.1 h and a 1st percentile of 3.4 h. The selected software rule, clarified 28 October, is: **Linux and the flight service boot before loading; the Pi stays powered throughout the pad wait, with image processing idle until PREPARE/ascent.** With the Pi merely idle the worst corner still gives 6.0 h. The off alternative is retained as a historical numerical comparison, not the selected continuous-operation policy. Raw simulation outputs are unchanged; none of these figures establishes physical endurance.

Things outside the model that could break it: a Pi idle draw several times the published figures (for example Wi-Fi left on, a busy-wait in the flight software, HDMI enabled); a cell much colder than −10 °C; a pack that is not the one specified.

**The 5.1 V rail near empty.** The TPS62130 needs its input above about 5.2-5.3 V to hold 5.1 V at 1 A (assumed switch and inductor resistance of about 0.13 Ω). That is 2.6 V per cell, close to the pack's protection cut-off, so the Pi rail sags only in the last few percent of capacity, which the reserve factor already excludes.

### Peak current

Everything on at once at high values: radio transmitting at +22 dBm, buzzer at its peak (not mean) current, flash programming, GNSS acquiring, Pi on four cores with camera and card, and an **assumed** 1.5× allowance on the 5.1 V rail for bursts above bench averages. Pack current is taken at 6.0 V (3.0 V per cell), where it is highest. Table: `peak_currents.csv`.

| Check | Current | Limit | Limit / current |
|---|---:|---:|---:|
| 3.3 V rail against the TPS62162 (1 A) | 0.57 A | 1 A | 1.8 |
| 5.1 V rail against the TPS62130 (3 A) | 1.44 A | 3 A | 2.1 |
| 5.1 V rail against the TPS22965 (6 A, as the BOM states; not re-verified) | 1.44 A | 6 A | 4.2 |
| 5.1 V rail against three JST GH contacts (1 A each) | 1.44 A | 3 A | 2.1 |
| Pack current against the 5 A fuse | 1.91 A | 5 A | 2.6 |
| Pack current against the pack's 5.2 A rating | 1.91 A | 5.2 A | 2.7 |
| Pack current against the lowest protection trip (10 A) | 1.91 A | 10 A | 5.2 |

Nothing is close. Two observations, not defects: the 5 A fuse is about 2.6 times the worst credible load and almost equal to the pack's continuous rating, so a 3 A fuse would protect the harness better with no nuisance risk on these numbers; and inrush into the Pi rail's capacitance is not modelled (the load switch's rise-time capacitor is there for it).

## 6. Descent rate and canopy

The relations in `sim/systems/sizing.py` were checked and reused unchanged: C_d·S = 2mg/(ρv²), v = √(2mg/(ρ·C_d·S)), D₀ = √(4S/π). They are correct for steady vertical descent; the old script's results were right for the masses it was given. Mass here is the Monte Carlo all-up mass: mean 335 g, 5th-95th percentile 320-351 g. Density: 1.225 kg/m³ standard; 1.320 on a cold dense day (0 °C, 1035 hPa, sea level) and 1.185 on a warm thin one (15 °C, 980 hPa); the day's values are assumed, and site elevation is taken as sea level.

### Drag coefficients

| Canopy | C_d low / typ / high | Reference area | Source | Stability |
|---|---|---|---|---|
| Round, flat circular (the proposal's canopy) | 0.60 / 0.75 / 0.90 | fabric area | Knacke (1992) table 5-1 gives 0.75-0.80: **recalled, not re-read**. Widened to the range the old sizing used, because small hobby canopies with spill holes read lower. | Oscillates ±10-40° per Knacke (recalled). |
| Cruciform (cross) | 0.60 / 0.72 / 0.85 | fabric area of the cross | Knacke table 5-1, 0.60-0.85: recalled, not re-read. One CFD paper gives 0.91 (search excerpt). | 0 to ±3° (recalled). |
| Annular | 0.85 / 0.90 / 0.95 | fabric area | Knacke table 5-1, 0.85-0.95: recalled, not re-read. The same CFD paper gives 1.03 (search excerpt). | Under ±6° (recalled). |
| Square cone (RD41 dropsonde type) | 0.40 / 0.48 / 0.55 | area of the square mouth, with the body's drag included | **Calculated** from the figures quoted in `wiki/navigation_closed_loop_simulation.md` section 8: 323 g at 11 m/s under 0.30 × 0.30 m gives C_d = 0.475 at 1.225 kg/m³. The ±15 % range is assumed. | Sold as eliminating pendulum motion (as quoted in that report). |

The handbook could not be opened this session, so the three Knacke rows are remembered values and must be confirmed from the book before they go into a design review. The square-cone figure is the only one tied to a flying article of this mass and speed, and it is one data point.

### Area required

At standard density. Areas in cm²; the dimension is the diameter (round, annular), the arm length (cruciform, arms 0.3 as wide as long, assumed) or the side (square cone). Ranges are for C_d high to low. Full table: `canopy_sizing.csv`; chart: `canopy_area_vs_speed.png`.

| Canopy | Historical 10.5 m/s calc at 335 g | 10.0 m/s at 350 g (largest allowed) | 15 m/s at 300 g (smallest recommended) |
|---|---|---|---|
| Round | 649 (541-811); D = 0.287 m (0.262-0.321) | 747 (623-934); D = 0.308 m | 285 (237-356); D = 0.190 m |
| Cruciform | 676 (573-811); arm 0.364 m (0.335-0.399) | 778 (659-934); arm 0.391 m | 296 (251-356); arm 0.241 m |
| Annular | 541 (512-573); D = 0.262 m (0.255-0.270) | 623 (590-659); D = 0.282 m | 237 (225-251); D = 0.174 m |
| Square cone | 1014 (885-1217); side 0.318 m (0.297-0.349) | 1167 (1019-1401); side 0.342 m | 445 (388-534); side 0.211 m |

These are historical 10.5 m/s design-point calculations at the budget's 335 g expected mass, with handbook drag coefficients. The current recommendation is to size/trim from a measured target near 11 m/s; no area table at 11 m/s has been calculated here. The 10.5 m/s required drag area is C_d·S = 0.0487 m², up from 0.0472 m² in the 1 October sizing (325 g).

### Sensitivity

For a fixed canopy, speed goes as √(mass/density):

| Change | Effect on a 10.5 m/s descent |
|---|---|
| +10 g of mass | +0.16 m/s |
| 300 g / 320 g / 340 g / 350 g instead of 335 g | 9.93 / 10.26 / 10.58 / 10.73 m/s |
| +1 % air density | −0.05 m/s |
| Cold dense day (1.320 kg/m³) | **10.12 m/s** |
| Warm thin day (1.185 kg/m³) | 10.68 m/s |
| +1 % drag coefficient | −0.05 m/s |

What this means for the 10 m/s floor (`descent_speed_spread.csv`):

- **A canopy cut from the typical handbook coefficient and never dropped**, with the real coefficient, mass and launch-day density all unknown within their ranges: speed 9.7-11.3 m/s (5th-95th percentile) for the round canopy, with a 21 % chance of being under 10 m/s; cruciform 20 %; square cone 14 %; annular 6 % (its coefficient range is the narrowest, on recalled figures). Worst corners run from 9.0 to 12.2 m/s for the round canopy.
- **A canopy trimmed in drop tests to 10.5 m/s at standard density**: 10.0-10.7 m/s, 3 % chance of being under 10 m/s, all of it from cold dense air combined with a light can.
- **To stay above 10 m/s on the densest day** the canopy must measure at least 10.38 m/s at standard density with no measurement error, and about **11.0 m/s** if the figure comes from a single 100 m drop timed to 0.2 s (the old sizing's result, unchanged). A heavier can helps here: the same canopy falls 0.16 m/s faster per 10 g.

The archived September proposal's argument for 10.5 m/s was five more images per extra second. Trimming to 11.0 m/s costs about 1.3 s of a 300 m descent, or six or seven images out of about 140. The live 13 October proposal now labels 11 m/s as a drop-calibration recommendation and keeps the canopy unfrozen.

The canopy-type decision (`wiki/navigation_closed_loop_simulation.md` section 10, item 1) is not made here. What this adds: a stable canopy is not expensive in size. The annular canopy is the smallest for a given speed; the cruciform needs about the same fabric as the round one; the square cone needs a mouth about 32 cm across, close to the dropsonde's 30 cm. All of them pack into the space a 29 cm round canopy would.

## 7. Document-conflict register observed 11 October 2026

This table preserves what the source documents said when this budget was prepared; it is not a current conflict list or a fresh hardware audit. The live [13 October mission proposal](trn_secondary_mission_proposal.md) and [design baseline](design_baseline.md) reconcile several proposal rows, including the selected hardware, canopy status, optics, packet size and descent-rate recommendation. Other listed conflicts concern separate records or PCB artifacts and retain their stated dates and evidence. Keep this dated register for provenance; use the linked current sources for present decisions.

| # | Disagreement | Where |
|---|---|---|
| 1 | Flight computer: "Orange Pi Zero 2W (H618)" against Raspberry Pi Zero 2 W. | `wiki/trn_secondary_mission_proposal.md` section 15 against `wiki/flight_computer_bom.md` (header, offboard table). The old sizing uses the Orange Pi: 12.5 g and 0.94-2.4 W (`sim/systems/run_sizing.py`, `MASSES` and `LOADS`). |
| 2 | Camera: "Global-shutter camera, 1280 × 800 (OV9281-class), ~15 g", lens included, against the official IMX296 camera, 1456 × 1088, 34 g before lens, plus lens and adapter (about 54 g typical here). | Proposal section 15 against BOM offboard table and `wiki/sensor_bom.md` sections 1-2. Old sizing: "camera board, lens and holder" 15-25 g and an Arducam USB camera load. |
| 3 | Battery: one 18650 at 46-48 g with a 1S-to-5 V boost converter, against a 2S 18650 pack at about 100 g with two buck converters. | `run_sizing.py` (`MASSES`, `CELLS`, `CONVERTER_EFFICIENCY`) against BOM "Mission basis", power path and offboard table. The proposal names no battery (section 17 only says "Larger battery" as the remedy). |
| 4 | Pack capacity and current: "minimum 3.3 Ah, 5 A continuous" against the maker's "minimum 3350 mAh, maximum discharge 5.2 A, nominal 3500 mAh". | BOM offboard table and "Mass, power and cost checks" against the KeepPower product page. |
| 5 | Mass estimate: "the secondary mission adds ~15 g for the camera, plus ~5 g" (proposal section 15); typical 208.5 g and high 275.5 g (`results/sizing.json`); "about 290 g ... about 350 g" (`BOTS/PLANS/STATUS.md`, 9 October item 7, marked unverified); "Target about 330 g" (BOM "Mass, power and cost checks"). This report: 324 g typical sum, 335 g expected, 431 g all-high. | As listed. |
| 6 | Pi mass "10-15 g" against resellers' 9-10 g. | BOM "Mass, power and cost checks". |
| 7 | Power target: "≤6 W measured average target" and "roughly 6 W average load", against about 1.3 W waiting and 4.5 W in descent estimated here. Not an error, but the pack was sized on it. | BOM "Mission basis" and "Mass, power and cost checks". |
| 8 | MAIN board size: "44 × 65 mm", outline Y = 67.5-132.5 mm, holes at Y = 70.5 and 129.5 mm (38 × 59 mm pattern), BMI088 at (100, 95), "zero track segments". The saved board has a 44 × **64** mm outline (Y = 68-132), holes at Y = 71 and 129 (38 × 58 mm pattern), U2 at (100, 105) and two track or via objects. | `wiki/mainboard_placement.md` against `pcb/cansat_control/cansat_control.kicad_pcb` as read with pcbnew on 11 October. Any mechanical model built from the note will not fit the board. |
| 9 | RF board: described as a circular board with a full BOM, but the PCB file has no outline and no footprints. | BOM "Top RF PCB" against `pcb/cansat_radio/cansat_radio.kicad_pcb`. |
| 10 | MAIN-RF link: "Exact compatible MPN pair ... TBD" and a board-to-board connector, against a selected Würth 61202021621 shrouded header and ribbon harness. | BOM "PCB fabrication and inter-board hardware" and `wiki/flight_computer_connections.md` section 2, against the BOM's own "Saved MAIN completion" section and `wiki/mainboard_completion.md`. |
| 11 | Reference designators: U2/U3 bucks, U4 BMI088, U5 MS5611, U6 ADS1115, U11 GNSS, U12 radio, U13 flash, against the saved board's U7/U8 bucks, U2 BMI088, U3 MS5611, U4 ADS1115, U11 flash. | `wiki/flight_computer_connections.md` sections 1 and 3-8 against the board file; already noted in `wiki/sensor_bom.md` section 1. |
| 12 | Packet sizes need protocol context: downlink telemetry is 40 bytes with 82.2 ms airtime (8.2 % at 1 Hz); the STM32-to-Pi `Nav` message is a separate 32-byte payload/frame definition. | Current link specification [stm32_pi_link_protocol.md](stm32_pi_link_protocol.md) and `code/cansat/link.py`; downlink `code/cansat/telemetry.py` (`PACKET_SIZE`). The 32-byte link message does not replace the 40-byte radio telemetry packet. |
| 13 | Barometer BME280 against MS5611; "IMU + magnetometer" against BMI088 with the magnetometer optional and not fitted. | Proposal section 15 against BOM MAIN table and `sensor_bom.md` section 7. |
| 14 | Thermistor: "Response ≤ 1 s" against a probe whose 200 ms figure is in liquid and whose air response is unqualified. | Proposal section 15 against BOM offboard table and `sensor_bom.md` section 2. |
| 15 | Frame storage: "~5 MB/s, ~300 MB per flight" against 9.9-15.8 MB/s. | Proposal section 15 against BOM offboard table (microSD row). |
| 16 | Lens field of view: "around 90°" against "fisheye-like (about 110-115 degrees on this sensor)". | BOM offboard table against `BOTS/PLANS/STATUS.md` (9 October item 7). |
| 17 | Lens price: $87.40 against $96.00 on the maker's page now (search excerpt). | BOM "Mass, power and cost checks". |
| 18 | Canopy: "A plain round canopy on a swivel" against a recommendation to change to a stable type. | Proposal sections 6 and 15 against `wiki/navigation_closed_loop_simulation.md` section 10 item 1 (an open decision, not an error). |
| 19 | Radio loads in the old sizing are "ASSUMED from typical u-blox M10 and SX1276 datasheet currents"; the radio is an SX1262 module. | `run_sizing.py` `LOADS` against BOM RF table. |

## 8. What to weigh and measure first

In order of how much uncertainty each removes.

**Mass**

1. **Print one shell, two caps and a first frame, and weigh them** (55 % of the variance; an afternoon's work once there is a CAD model, which there is not). Before that, decide wall thickness and filament: that alone fixes most of it.
2. **Buy and weigh the optical stack**: camera body with and without the tripod mount and C-CS ring, the lens, the adapter and lock nut (16 %).
3. **Count and weigh the fasteners and standoffs** once the frame exists (6 %).
4. **Weigh the battery pack** as delivered, with its leads (4 %).
5. Weigh a bare MAIN board from the first fabrication batch, the ribbon set and the power harness (about 4 % together).
6. Ask the organisers whether the parachute counts toward 350 g (worth about 14 g of margin).

**Power**

1. **Pi Zero 2 W with the camera capturing at 5 Hz and the matcher running**, at the 5.1 V rail. It is about 90 % of the descent load and rests on other people's bench reports and an assumed camera figure.
2. **Pi idle in flight configuration** (radios off, headless), because it sets the 3-hour answer.
3. The whole 3.3 V rail with the radio at the chosen power, and the buzzer current at 3.3 V.
4. A 3-hour run of the complete system in a fridge or outdoors at 0-5 °C, logging pack voltage, followed by the flight profile. This replaces the assumed cold derating.
5. Converter efficiency at the measured loads, or read from the datasheet curves.

**Descent**

1. Drop the chosen canopy type from at least 25 m with the can at flight mass and derive terminal speed with `sizing.terminal_speed_from_drop` (the old sizing showed height/time under-reads badly from short drops).
2. Record air temperature and pressure at each drop and correct to standard density.
3. Trim to about 11.0 m/s at standard density, re-weigh the can, and bring the data to the launch.

## 9. Limits of this work

- No measurement of any kind. See the tag on every row.
- No mechanical design exists (`cad/` is empty), so the largest mass items after the battery and camera are calculated from assumed dimensions. Whether the 100 g pack, the 38 × 38 mm camera with an 18 mm lens, a 65 × 30 mm Pi and a 44 × 64 mm board fit together inside 66 × 115 mm is **not addressed here** and is at least as important as the mass.
- Several datasheets could not be read: the u-blox, Bosch, ST, TI and Winbond PDFs are compressed or image-based and the tools available could not extract their tables; the Edmund and reseller battery pages refused the request; Knacke's handbook is not online. Those values are marked "recalled" or "search excerpt". The Taoglas GNSS antenna, the buzzer and the KeepPower pack page were read directly.
- The triangular ranges are judgement. The probabilities that follow from them (6 %, 15 %) are a way of combining those judgements, not frequencies.
- Printed parts are taken as solid; real prints are a few percent lighter. Drilled holes, solder mask and plating are neglected on the PCBs.
- Cold derating is generic lithium-ion behaviour, not this cell's curve, and the pack's self-heating inside the can is ignored (which is conservative).
- Peak-current checks use steady-state high values with a 1.5× allowance on the Pi rail; start-up inrush and transmit-burst droop on the RF board are not modelled.
- The descent model is steady vertical fall of a point mass. Opening load (rule 9), glide and swing are covered in the old sizing and the navigation reports and were not redone.
- Cost against the £400 limit was not assessed.
- Rules are the 2025-26 edition.
