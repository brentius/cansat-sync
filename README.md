# CANSAT

Local working folder for the CANSAT project, organised using ATLAS as the reference.

The current mission uses a Raspberry Pi Zero 2 W and STM32F405 to combine primary temperature/pressure telemetry with terrain-relative navigation during passive parachute descent and image-derived wind profiling. The selected hardware is defined by the [flight computer BOM](wiki/flight_computer_bom.md) and summarised in the [current design baseline](wiki/design_baseline.md).

The [all-board BOM summary](wiki/board_bom_summary.md) allocates the selected POWER, MAIN and RF boards, plus Pi, camera and shared offboard purchases. POWER contains supply electronics; MAIN contains the MCU, sensors and interfaces. The previous KiCad projects were cleared on 29 October 2026 to restart PCB design; the BOM and design notes remain reference material, and descriptions of the previous CAD are historical.

## File structure

```text
cansat/
├── pcb/                   Schematics, PCB layouts and parts libraries
├── code/                  Onboard software, ground software and data processing
├── cad/                   Mechanical designs and assembly models
├── sim/                   Simulations, analysis scripts and results
├── wiki/                  Mission proposals, documentation and reference notes
├── BOTS/
│   ├── AGENTS/            Local agent instructions
│   └── PLANS/             Architecture, plans, decisions and project status
├── README.md
├── LICENSE-HARDWARE
├── LICENSE-SOFTWARE
└── .gitignore
```

## Onboard software

The [flight software implementation report](wiki/flight_software.md) describes the
STM32 firmware, Pi service/navigation, design choices, verification and hardware
next steps. Sources are in `code/firmware/stm32`, `code/cansat`, `code/piapp` and
`code/deploy/pi`. Reproduce the combined host/cross-build gate with
`.venv/Scripts/python.exe code/tools/verify_flight.py`.

## Mission documentation

The [mission proposal](wiki/trn_secondary_mission_proposal.md) was rewritten on 13 October 2026 around the current BOM, reference interfaces and newer feasibility studies. It includes both mission software paths and distinguishes selected hardware, candidate code, targets and verified evidence. The [27 September proposal](wiki/history/trn_secondary_mission_proposal_2026-09-27.md) is retained as a historical source snapshot; it is not the current design. The original ChatGPT project source remains separate.

The standalone [mathematical derivations and verification document](wiki/cansat_mathematics.tex) covers the proposal's mathematics index, corrected assumptions, worked examples and synthetic experiments. It needs no external source files to compile and compiled successfully with the built-in editor on 13 October 2026. Its historical example inputs are preserved alongside current BOM context. The supplied 52-page PDF remains a separate historical attachment that has been audited.

The [simulation and feasibility audit](wiki/mathematics_simulation_audit.md) maps all 249 numbered PDF equations to checks and restrictions, with 76 tests reported passing in the retained audit, rendered-image experiments, navigation and atmospheric uncertainty ensembles, and processing budgets. It records a transfer-function typo and a corrected NCC memory problem. Numerical consistency is established under the tested assumptions; flight performance and current Pi Zero 2 W throughput remain unverified. Historical H618/OV9281 sizing is explicitly separated from the current baseline. Supporting code and retained results are in `sim/maths/`; project state is recorded in `BOTS/PLANS/`.

A [second review of that PDF](wiki/cansat_maths_pdf_review.md) on 13 October re-derived all 249 equations and recomputed the stated numbers: the mathematics is correct apart from one known typo and a few small slips. It also lists where the PDF's inputs and conclusions no longer match the current baseline, which of its corrections the live proposal has not yet taken up, and where the flight code departs from it. No simulation was rerun and no source was changed.

The [closed-loop navigation simulation](wiki/navigation_closed_loop_simulation.md) runs a candidate flight navigation filter (`code/cansat/eskf.py`, initialised from `code/cansat/firstfix.py` pose outputs) against a simulated parachute descent in `sim/nav/`: 109 scenarios of 160 descents each, an image-in-the-loop cross-check, a matcher-tolerance study and a canopy swing and glide study. In simulation the September proposal's plain filter loses lock under realistic errors, while the current proposal uses the hardened 17-error-state candidate, which holds about 0.6 m RMS relative to the simulated map. Canopy glide remains a major threat to wind retrieval. The acquisition-to-filter handoff is now implemented in `navigator.py`; the flight-software report covers its separate integrated evidence. All of it is synthetic: no real imagery, flight data or Pi timing yet. Inputs and their sources are in [navigation parameter sources](wiki/navigation_parameter_sources.md).

The [fine landmark tracking implementation and study](wiki/fine_landmark_tracking.md) adds the missing local image matcher in `code/cansat/fine.py`: prepared map templates, predicted-pose rectification, bounded spatial selection, ambiguity rejection and original-pixel measurements for the existing filter. `sim/nav/run_fine_mc.py` tests repeated image-to-filter corrections on procedural farmland with paired no-image controls and appearance/capture-limit stress cases. It is candidate software with synthetic verification; live acquisition/tracking orchestration is now implemented; real-image/Pi validation remains open.

The [end-to-end wind and landing study](wiki/wind_retrieval_end_to_end.md) carries that filter's output through to the mission's science products: a wind retrieval (`code/cansat/windprofile.py`) and a candidate landing predictor for the Pi Zero 2 W (`code/cansat/landing.py`), scored in `sim/nav/run_wind_mc.py` over five canopy behaviours. In simulation the simple drag-model retrieval meets its 0.5 m/s absolute-error target under a stable canopy and one with a 5 degree trim, and fails with a 12 or 25 degree trim or heavy swing, while the S1 comparison of image and GPS winds passes in every case. The current proposal therefore defines S1 as method agreement and treats absolute wind recovery and lateral-force correction as calibration-dependent. One descent's winds between 80 m and 300 m do not determine roughness length without the ground anemometer. All of it is synthetic and nothing has run on the flight hardware. Literature inputs and their evidence quality are in [wind science parameter sources](wiki/wind_science_parameter_sources.md).

The [first-fix study](wiki/acquisition_first_fix.md) covers the step before that filter can start: finding where the can is, and which way it faces, from one image with the heading unknown. `code/cansat/coarse.py` is flight code for the Pi Zero 2 W that does it in four stages and refuses instead of guessing; `sim/nav/run_acquisition_mc.py` scores it on procedural farmland (`sim/nav/farmland.py`) with rendered images and simulated descents. In simulation it gives a fix from about nine images in ten on well-hedged terrain and about four in ten on large fields, where it is also wrong more often, so the result depends on the launch site and needs real imagery of it. An independent review found seven defects, all fixed. All of it is synthetic and nothing has run on the Pi.

The [STM32-to-Pi link protocol](wiki/stm32_pi_link_protocol.md) defines the serial link between the two processors, which did not exist before: framing, twelve messages, the time base, and how camera frames get their timestamps. `code/cansat/link.py` is the Pi's end and the reference the STM32 firmware has to match, with test vectors in `code/tests/link_vectors.json`. It was checked in closed loop against the navigation Monte Carlo (`sim/nav/run_link_mc.py`). The C firmware and Pi service now exist; host/cross-build verification is recorded in the flight-software report. Nothing has been executed on either physical processor.

The [mass, power and descent budget](wiki/mass_power_budget.md) replaces the 1 October sizing, which was for hardware the project no longer uses. For the current design it expects about 335 g against the 350 g limit with a 6 to 15 % chance of exceeding it, a wide margin on three hours of power if the Pi idles while waiting, and a design descent rate too close to the 10 m/s floor. Nothing in it has been weighed or metered. The model is `sim/systems/budget.py`.

The [PDR checklist](BOTS/PLANS/PDR_FULL_MARKS_CHECKLIST.md) maps the supplied 36-point rubric and all available requirements. Those requirements are still from 2025-26; current-season rules/dates need confirmation. Completed analysis is documented separately from open firmware, physical packaging, calibration, target compute, mass/cost and flight qualification.

## Licenses

Following ATLAS, hardware designs in `pcb/` and `cad/`, and hardware circuit netlists in `sim/`, use CERN-OHL-S-2.0; software in `code/` and simulation scripts in `sim/` use GPL-3.0-only. See `LICENSE-HARDWARE` and `LICENSE-SOFTWARE`.
