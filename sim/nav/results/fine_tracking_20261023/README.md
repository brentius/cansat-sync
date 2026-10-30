# Fine landmark tracking study, 23 October 2026

Use **`full_corrected/`** for the retained experiment. `frames.csv` records each exposure, matcher acceptance, ESKF acceptance, rejections, reference errors and host timing. `summary.json` records the parameters, deterministic seeds, per-trial outcomes and aggregate measurements. `figures/audit.json` independently reconciles the retained CSV counts/RMS with the JSON and hashes the input/source files used for the figures.

All imagery, maps, motion and sensor measurements are synthetic. Six terrain/initial-error draws are paired across five conditions; frames and conditions from the same trial are correlated. Plot bands are the full observed range across six trials, not confidence intervals. Host timings are not Pi performance.

The `pilot/`, `benchmark/` and `full/` directories are retained **invalid development diagnostics**. Early runs used a different image setup; a later audit found the principal point had been binned from full image dimensions instead of the full-resolution principal point. They are not evidence for the intended camera or current implementation. Their raw files are preserved rather than replaced. The corrected camera is checked against `coarse.bin_camera` before the final run; its principal point is (181.625, 135.625) in a 364 x 272 frame.

From the repository root:

```powershell
.venv/Scripts/python.exe sim/nav/run_fine_mc.py --trials 6 --frames 139 --seed 20261023 --output sim/nav/results/fine_tracking_20261023/reproduced
.venv/Scripts/python.exe sim/nav/analyse_fine_mc.py --input sim/nav/results/fine_tracking_20261023/reproduced --output sim/nav/results/fine_tracking_20261023/reproduced_figures
```

Use a new output directory for each rerun to preserve the retained data. The script uses one numerical thread by default. Exact host timing is expected to vary; fixed seeds reproduce the numerical trial inputs. Full methods, conclusions and limitations are in [`wiki/fine_landmark_tracking.md`](../../../../wiki/fine_landmark_tracking.md).
