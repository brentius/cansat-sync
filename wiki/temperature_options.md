# Budget air-temperature sensing options

Checked 3 October 2026; supplier prices/stock are dated snapshots. The selected analogue probe path is a prototype and flight use remains conditional on calibration and installed air-response testing. The [flight-computer BOM](flight_computer_bom.md) is purchasing authority; see [design baseline](design_baseline.md). The total CanSat budget is **£400**, including the other electronics, optics, battery, boards and structure. FP07 at the user's approximately $200 quote is excluded from the baseline. Aim for roughly **£15–£20 for the assembled temperature probe and its readout**, subject to the delivered quote; this is a budget allocation, not a checked total.

The mission proposal asks for **≤0.1 K relative precision and ≤1 s response**. Catalogue thermal time constant means the time to reach 63.2% of a step; for a first-order sensor, 90% takes 2.303 times that value. Report both measured t63 and t90 rather than silently treating them as equivalent. Resolution, relative precision, absolute accuracy and installed thermal response are separate quantities.

## Practical choices

| Option | Checked purchase cost / availability | Suitability and limits |
|---|---|---|
| **TE GA10K3MCD1: budget analogue prototype** | [DigiKey UK](https://www.digikey.co.uk/en/products/detail/te-connectivity-measurement-specialties/GA10K3MCD1/5277239): **£10.39 ex VAT / £12.468 inc VAT**, single-unit purchase, 6,574 shown. JLC [C3153883](https://jlcpcb.com/partdetail/TEConnectivity-GA10K3MCD1/C3153883): **4**, $23.8646, Extended. Shipping excluded. | 10 kOhm wired probe; 0.5 mm maximum tip diameter and 152 mm fine leads make external mounting practical. Use the existing ADS1115/OPA333 circuit. TE specifies **200 ms in liquid**, not air; ≤1 s installed air response remains to be measured. |
| **TI TMP117AIDRVR: digital redesign** | JLC [C699536](https://jlcpcb.com/partdetail/TexasInstruments-TMP117AIDRVR/C699536): **36,347**, **$1.0177**, Extended. | I2C sensor on a tiny external sensor PCB or flex island. Removes the thermistor, ADS1115, OPA333, excitation PMOS and temperature-only LDO/support parts. Sensor price excludes carrier manufacture/assembly, harness and shield. Mounted air response still needs testing. |
| **Amphenol BR11KA104N: specified still-air time constant** | [Mouser UK](https://www.mouser.co.uk/en/ProductDetail/Amphenol-Advanced-Sensors/BR11KA104N?qs=1mbolxNpo8fkDH9jl5TmxQ%3D%3D): **£13.99 each at minimum 5**, 345 shown; **£69.95 ex VAT / £83.94 with 20% VAT** minimum purchase, before delivery. | 100 kOhm nominal miniature glass bead; BR11 family tau is **0.8 s in still air**. This is not 90% settling within 1 s. Loose ±25% resistance requires individual calibration. Very fine short leads make extension and strain relief harder than the wired TE probe. Do not budget just one unit's nominal price when you must buy five. |

Sources for performance: [TE probe specifications](https://www.te.com/en/product-GA10K3MCD1.html), [TE MCD family, including liquid-response condition](https://www.te.com/en/product-CAT-NTC0065.html), [TMP117 product specifications](https://www.ti.com/product/TMP117), [BR manufacturer datasheet](https://www.amphenol-sensors.com/hubfs/Documents/AAS-920-308B-Thermometrics-NTC-TypeBR-100714-web.pdf). Inventory/prices are snapshots, not reservations. The two wired probes are externally fitted assemblies; JLC stock does not mean their mounting in the airflow is an assembly service.

**Recommendation:** prototype the TE probe first if keeping the current analogue PCB. It is an affordable purchase with useful fine leads. Treat flight selection as conditional on calibration and response testing. If simplifying the temperature electronics before drawing them, prototype the TMP117 remote implementation instead; it has the strongest cost advantage in the IC BOM. Neither candidate has a demonstrated ≤1 s response in this CanSat's shield. BR11 is a fallback when its specified still-air tau and handling are worth the larger minimum spend.

## Keep the existing analogue implementation

Wire TEMP_EXC -> existing **150 kOhm, 0.1% reference** -> TEMP_RAW -> TE 10 kOhm probe -> GND. Keep OPA333, ADS1115, excitation switch and two-wire J_TEMP harness. The reference does not have to equal the sensor's nominal resistance; retaining 150 kOhm limits self-heating without purchasing a different PCB resistor.

At 25°C and 3.0 V excitation, calculated values are:

- Divider output: **0.1875 V**.
- Probe current: **18.75 uA**.
- Probe dissipation: **3.52 uW** while excitation is on.
- Worst-case probe dissipation over all positive resistances: **15 uW**, reached when the probe equals the 150 kOhm reference.

Continue ratiometric readout: R_NTC = 150 kOhm × V_AIN0 / (V_AIN1 - V_AIN0). Convert ADC codes to volts using each reading's PGA range; do not divide raw codes from different gain settings. Read AIN1 excitation with ±4.096 V full scale. AIN0 can use ±2.048 V for increased sensitivity if the worst-case expected divider voltage fits; widen if needed. Wait for completed conversions after mux/gain changes. Calibrate the actual probe's R(T), rather than reusing FP07 coefficients. The current design ceiling is ≤6% excitation duty. The 128 SPS / 10 Hz timing candidate discussed in the separate characterisation report exceeds that ceiling (about 19.41%); it is not an adopted schedule. The implemented candidate firmware uses 8 Hz paired reads at 860 SPS with 2300 µs settling, 5.0048% calculated duty ([design baseline](design_baseline.md), [flight software report](flight_software.md) §7); settling, self-heating and the hardware cutoff still need verification on the assembled board.

The temperature circuit's purchasing change is one externally sourced probe. Existing PCB MPNs remain valid; the reference stays 150 kOhm. This path minimises disruption to the current pin map.

## Digital implementation with TMP117

Use the installed stock symbol **`Sensor_Temperature:TMP117xxDRV`** and footprint **`Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm`**. Both were checked in the installed KiCad 10 libraries. Set the purchased MPN to TI TMP117AIDRVR.

Place the chip and a 100 nF bypass capacitor on a small external island, thermally separated from the Pi/board and shaded from radiation. Use the **3.3 V BARO/I2C1 bus** and its existing main-board pull-ups. ADD0 to GND selects 0x48, which does not conflict with MS5611 at 0x76. The old ADS1115 at 0x48 is on a different bus and can be omitted in the digital-only variant.

| TMP117 WSON pin | Connection |
|---:|---|
| 1 SCL | BARO_SCL -> STM32 PB6 |
| 2 GND | GND |
| 3 ALERT | No-connect when polling |
| 4 ADD0 | GND |
| 5 V+ | +3V3_AON; local 100 nF to GND |
| 6 SDA | BARO_SDA -> STM32 PB7 |

The stock symbol has six electrical pins; footprint pad 7 is the exposed thermal pad. For the rigid-board prototype, follow TI's best-accuracy recommendation to leave it **unsoldered**: omit its paste opening and leave that pad unconnected. For flex, TI allows soldering it for reliability; follow its instructions and calibrate the assembled part. It is not an omitted supply-ground pin. Source: [TMP117 datasheet, pins, supply and layout](https://www.ti.com/lit/ds/symlink/tmp117.pdf).

Digital harness J_TEMP_DIG contacts: **1 +3V3_AON, 2 GND, 3 BARO_SCL, 4 BARO_SDA**. This differs electrically from the two-wire analogue probe harness. Give the ports distinct labels and do not interchange the two probe assemblies.

Use TI's specified accuracy-test settings as the initial baseline: eight averages, 1 Hz conversion cycle and the stated pad/logic conditions. Faster single-conversion sampling can be used to characterise thermal response, but validate its noise and heating; do not assume every acquisition mode reproduces the advertised accuracy.

For the digital-only variant, remove MAIN U4 ADS1115, MAIN U5 OPA333, MAIN U6 temperature-only LDO, temperature excitation AO3401A, precision divider resistor, buffer RC, TEMP I2C/ready pull-ups and their dedicated capacitors. U1's TEMP_GATE_N, ADC_RDY_N and I2C2 GPIOs become spare. The reverse-protection AO3401A remains. Other shared-value BOM quantities must be reconciled against the final schematic, rather than removing whole resistor/capacitor rows. Add one TMP117 and its bypass capacitor/remote carrier.

## Other inexpensive implementations

- **Suspended miniature SMD NTC on fine wires:** a possible low-cost experimental probe using the same analogue readout. Keep the sensing body free of epoxy blobs, heatshrink and bulky carrier copper; add strain relief away from it. This is a construction approach to test, not a selected or qualified MPN. A standard SMD rating measured on a PCB cannot establish the response of this suspended version.
- **Generic metal-sheathed probe or large breakout:** convenient, but extra thermal mass and heat conducted from the CanSat can smear the profile. Low price and fast digital conversion do not establish air response.
- **Semitec FT thin-film thermistor:** manufacturer specifies about 1 s still-air tau, but the checked 103FT1005B5P2 listing is non-stocked with minimum 1,000. Excluded as a practical small-project purchase. [Manufacturer](https://www.semitec-global.com/products/thermistor_ft/), [checked distributor listing](https://www.mouser.fr/en/ProductDetail/Semitec/103FT1005B5P2?qs=wgO0AD0o1vv7nAoX9HOT3w%3D%3D).

## Qualify the complete probe, not just the chip

1. Calibrate at several temperatures spanning the expected flight range, using a reference thermometer whose uncertainty is small enough for the intended measurement. Ice point alone does not establish the full R(T) curve or slope accuracy.
2. Measure an **air-to-air** temperature step with the actual shield, fine leads and mount at representative airflow. Record t63/t90 and check measurement repeatability; a water-dunk test cannot establish air response.
3. Repeat with the Pi/converters running and with radiation exposure. Check heating conducted through leads, residual solar bias and whether airflow changes with swing/orientation.
4. Log at about 10 Hz for characterisation and report the measured time constant. A first-order lag has an approximate vertical smoothing length v × tau: at 10 m/s, tau = 1 s corresponds to about 10 m. Any post-flight lag correction must include noise amplification and uncertainty; it cannot prove an untested sensor meets the mission target.

The complete £400 budget remains to be reconciled, including PCB/extended-part charges, optics, shipping/VAT and mechanics. Removing FP07 corrects one large omission; it does not establish compliance by itself.
