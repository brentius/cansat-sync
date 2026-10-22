# First-fix acquisition Monte Carlo: the 36-scenario run of 11 October 2026

SYNTHETIC. Written by `sim/nav/run_acquisition_mc.py --trials 48 --workers 5` (57 minutes, laptop on mains), 11,664 images in 36 scenarios. This is the run the tables in `wiki/acquisition_first_fix.md` sections 4 to 7 are taken from.

Code state: `code/cansat/coarse.py` as it stood **before the independent review's fixes**. Compared with the code now in the repository it:

- took its stage-1 candidates as the best peak of each heading, not the best peaks of the whole search, and stopped at the first candidate that gave a fix, so it could not refuse a view that fits the map in two places;
- had no check of the fix's tilt against the levelled tilt;
- returned the solver's covariance plus 1 m², without the factor of four now applied (`sigma_h` and "within 2 sigma" here are on that basis);
- did not record the capture error.

The scenario names `strict_interim` (large fields) and `strict_interim_mixed` here are `strict_interim_arable` and `strict_interim` in the current script.

`sim/nav/results/acquisition/` holds five of these scenarios rerun on the final code, on the first 24 descents of each.
