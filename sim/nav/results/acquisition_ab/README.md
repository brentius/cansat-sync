# First-fix acquisition: interim-pose comparison of 11 October 2026

SYNTHETIC. A separate run of `sim/nav/run_acquisition_mc.py`, 96 descents (576 images) per row, made to choose how the interim pose (stage 3) is found.

Code state: intermediate. At the time of this run the default interim pose was the four-parameter fit at the levelled tilt, the stage-4 search was 10 m, and the tilt check and the covariance factor of four were in place; the review's fixes were not. So in these files:

| Row | What it is |
| --- | --- |
| `baseline`, `style_arable` | interim pose by the fit at the levelled tilt (now `interim='similarity'`, not the default) |
| `interim_solver`, `interim_solver_arable` | interim pose by the six-parameter solver under relaxed checks (now the default) |
| `no_tilt_gate_arable` | as `style_arable` with the tilt check switched off; identical, because the check never fired |

Result: the solver gave more fixes on mixed terrain (90 % of images against 87 %) and no more wrong ones on large fields (8 % against 10 % of fixes over 10 m), so it stayed the default.
