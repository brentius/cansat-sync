# Documentation index

Start with the [design baseline](design_baseline.md) and [mission proposal](trn_secondary_mission_proposal.md). These describe the selected design and its evidence limits; study results are not hardware qualification.

## Hardware and packaging

- [Flight computer BOM](flight_computer_bom.md), [all-board BOM summary](board_bom_summary.md) and [sensor BOM](sensor_bom.md)
- [Connections and pin map](flight_computer_connections.md) and [STM32-Pi link protocol](stm32_pi_link_protocol.md)
- [Schematic/BOM review](schematic_bom_review.md), [mainboard completion](mainboard_completion.md) and [placement](mainboard_placement.md)
- [Passive package review](passive_package_review.md) and [mass, power and descent budget](mass_power_budget.md)
- [Temperature options](temperature_options.md) and [probe characterisation](temperature_probe_characterisation.md)

## Flight software

- [Implementation report: structure, choices, verification and next steps](flight_software.md)
- [STM32 build/driver guide](../code/firmware/stm32/README.md) and [Pi provisioning](../code/deploy/pi/README.md)

## Navigation and science studies

- [Acquisition and first fix](acquisition_first_fix.md) and [fine landmark tracking](fine_landmark_tracking.md)
- [Closed-loop navigation](navigation_closed_loop_simulation.md) and [navigation parameter sources](navigation_parameter_sources.md)
- [Wind retrieval and landing](wind_retrieval_end_to_end.md) and [wind science parameter sources](wind_science_parameter_sources.md)
- [Mathematics source](cansat_mathematics.tex), [simulation audit](mathematics_simulation_audit.md) and [PDF review](cansat_maths_pdf_review.md)

## Reviews, references and history

- [Example review assessment](example_review_assessment.md)
- [Reference material](references/README.md)
- [Historical proposal](history/README.md) and [archived working materials](history/working_materials/README.md)
- Local plans, decisions and execution records: `../BOTS/PLANS/` (local-only, ignored by Git)

Keep current documentation here, source references in `references/`, and superseded documents or editorial artifacts in `history/`. Keep simulation outputs with their original runs under `../sim/`; preserve raw data and logs.
