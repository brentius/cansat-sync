# Simulation index

These directories contain models, study runners, checks and retained outputs. Simulation results describe the tested assumptions; they do not establish measured hardware or flight performance. Existing output paths are retained so reports and reproducibility records continue to resolve.

| Directory | Subject | Study documentation |
| --- | --- | --- |
| `maths/` | Mathematical verification, feasibility and attachment audit | [Mathematics audit](../wiki/mathematics_simulation_audit.md) |
| `nav/` | Acquisition, navigation, image matching, link and wind studies | [Navigation](../wiki/navigation_closed_loop_simulation.md), [acquisition](../wiki/acquisition_first_fix.md), [fine tracking](../wiki/fine_landmark_tracking.md), [wind retrieval](../wiki/wind_retrieval_end_to_end.md) |
| `systems/` | Mass, power, descent sizing and flight-state studies | [System budget](../wiki/mass_power_budget.md) |
| `temperature/` | Probe response, readout, heating and synthetic calibration | [Probe characterisation](../wiki/temperature_probe_characterisation.md) |

Each study's `results/` directory contains retained artifacts. Read the associated report and any run manifest before comparing runs: parameters, software versions and assumptions may differ. Historical run directories are evidence, not disposable caches.

Put new runs in a distinct named or dated results directory. Preserve original inputs, seeds, manifests and logs; avoid overwriting a retained run. Use each runner's documented options rather than assuming a common command interface.
