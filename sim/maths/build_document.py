"""GPL-3.0-only. Consolidate reviewed derivations into ONE standalone TeX file.

No input/include, external bibliography, images or data files are needed to compile it.
"""
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]
data=json.loads((BASE/'results/verification.json').read_text(encoding='utf-8'))

front=r'''\documentclass[11pt,a4paper]{article}
\usepackage[margin=24mm,headheight=14pt]{geometry}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage{amsmath,amssymb,mathtools}
\usepackage{booktabs,longtable,array}
\usepackage{microtype}
\usepackage{xcolor}
\usepackage{enumitem}
\usepackage{fancyhdr}
\usepackage{pgfplots}
\pgfplotsset{compat=1.18}
\usepackage[hidelinks,breaklinks]{hyperref}
\definecolor{atlasblue}{RGB}{24,65,98}
\definecolor{atlasorange}{RGB}{183,91,31}
\hypersetup{pdftitle={CANSAT: mathematical derivations and verification},pdfauthor={CANSAT project}}
\pagestyle{fancy}
\fancyhf{}
\fancyhead[L]{\small CANSAT mathematics}
\fancyhead[R]{\small 30 September 2026}
\fancyfoot[C]{\thepage}
\setlength{\parindent}{0pt}
\setlength{\parskip}{5pt plus 1pt}
\setlength{\emergencystretch}{2em}
\setlist[itemize]{topsep=3pt,itemsep=2pt}
\setlist[enumerate]{topsep=3pt,itemsep=2pt}
\allowdisplaybreaks[2]
\numberwithin{equation}{section}
\setcounter{tocdepth}{1}
\newcommand{\R}{\mathbb R}
\newcommand{\E}{\operatorname E}
\newcommand{\Var}{\operatorname{Var}}
\newcommand{\Cov}{\operatorname{Cov}}
\newcommand{\tr}{\operatorname{tr}}
\newcommand{\diag}{\operatorname{diag}}
\begin{document}
\begin{titlepage}
\vspace*{16mm}
{\color{atlasblue}\Huge\bfseries CANSAT\\[4mm] Mathematical derivations\\[4mm] and verification\par}
\vspace{9mm}
{\Large A readable reference for terrain-relative navigation,\\wind retrieval and landing prediction\par}
\vspace{8mm}
{\large 30 September 2026\par}
\vspace{14mm}
\textbf{Purpose.} Derive the mathematics required by the project's secondary-mission proposal, explain what each equation means, state the conditions under which it is valid, and verify the results with independent numerical identities and synthetic experiments.

\textbf{Basis.} The local proposal dated 27 September 2026, including its mathematical index and Appendices A--L. This workspace contains no flight software, calibration dataset or measured descent logs. The connected CANSAT 2027 reference folder was inspected; its available regulations PDF describes the 2025--2026 competition. It is historical context, not confirmation of 2026--2027 rules.

\textbf{Evidence levels.} An algebraic identity can be proved within a model. A simulation checks implementation and consequences of that model. Agreement with the physical CanSat requires measurements. This document keeps those three statements separate.

\textbf{Deliverable.} This source contains the entire document, including plotted coordinates. It needs no external figures, bibliography, or included source files. Supporting scripts retain reproducible checks and synthetic data in the project, but are not required to compile this document.
\vfill
{\small Design inputs are identified as assumptions; no synthetic value is presented as a measured flight result.}
\end{titlepage}
\tableofcontents
\clearpage

\section{How to read this document}
\label{sec:guide}
The mission has a connected chain of mathematics. A camera and inertial sensors estimate the probe's motion; a dynamic response model converts that motion to an atmospheric wind profile; a statistical wind model propagates the remaining drift to predict landing. Pressure and temperature provide density, height and stability. A wrong assumption early in this chain can produce convincing but biased results later.

Each section explains the model, derives its equations, and gives numerical or limiting-case checks. For a first reading, follow the physical dynamics, then the camera and filter, then turbulence, retrieval, and prediction. The supporting-engineering section supplies quantities needed to make the data collection possible. The final sections distinguish completed verification from the experiments still needed.

\subsection{Conventions and units}
\begin{center}
\begin{tabular}{p{0.21\linewidth}p{0.65\linewidth}}
\toprule
Symbol & Meaning and convention\\
\midrule
$z,h$ & Height above the stated terrain datum, upward positive (m).\\
$s,v_d$ & Distance descended (m) and positive downward speed (m/s).\\
$\boldsymbol u,\boldsymbol w$ & Probe velocity and atmospheric wind, respectively (m/s).\\
$g$ or $g_0$ & Positive gravitational acceleration, $9.80665$ m/s$^2$ in examples.\\
$g(r)$ & A \emph{different} object: dimensionless transverse correlation.\\
$L,L_p,L_O,\ell$ & Turbulence decay scale, suspension length, Obukhov length and horizontal lag length (m).\\
$R_{wb},R_{cb}$ & Body-to-world and body-to-camera rotations.\\
$\theta$ & Potential temperature in atmospheric sections; an explicitly vector-valued local rotation error in navigation.\\
$P,\Sigma,S$ & Covariances, with squared units of the variables concerned.\\
$\Delta t,\Delta s$ & Time and descended-distance sample intervals, both positive.\\
\bottomrule
\end{tabular}
\end{center}
Convert degrees to radians before using $h\delta\theta$ or angular-rate products. Use absolute temperature in kelvin and pressure in pascals except where a pressure ratio uses consistent units. Logarithms of lengths must be ratios to a stated reference length.

\subsection{Important corrections to the proposal}
The following are results of the mathematical audit; they do not silently replace a flight design decision.
\begin{enumerate}
\item The lag $\ell=v_d^2/g$ follows from isotropic vector drag, not from vertical canopy force balance alone. A measured lateral response coefficient is needed to justify it.
\item The stated shear example gives a horizontal offset of $23.8$ m over $100$ m of descent. A vertical-path approximation is conditional, rather than accurate to a few percent at every scale.
\item The warped turbulence coordinate must increase with descent. A positive time derivative is inconsistent with an upward-increasing definition.
\item A body-axis accelerometer can be blind to swing under an ideal suspension, but gyro angular acceleration cannot determine arbitrary static tilt or unknown forcing.
\item A temperature lapse does not determine Obukhov length. The bulk Richardson number is not the gradient stability theorem, and $0.25$ is not a universal turbulence cutoff.
\item Tilt and translation are separated by landmark geometry. Placing every landmark at the same off-axis radius is not, by itself, enough.
\item An RMS or protection-level claim must state its dimension, truth uncertainty, correlations and probability model. A three-standard-deviation one-axis rule is not a two-dimensional integrity rule.
\item RANSAC's $w=0.5$, two-point, $p=0.999$ example needs $25$ iterations after rounding up. Truncated least squares requires a proper lifted formulation before claiming an SDP certificate.
\item The greedy guarantee applies to normalised log-determinant improvement. It does not justify a scale-dependent multiplicative claim about raw log determinant.
\item Remaining drift and absolute landing position have different covariance behaviour. The latter has a monotonic conditioning theorem under a fixed linear Gaussian model.
\end{enumerate}

\subsection{Verification approach}
Analytic derivations are accompanied by checks that use a different route: direct projection against homography, finite differences against analytic Jacobians, numerical ODE integration against closed forms, Fourier quadrature against a rational spectrum, and batch Gaussian conditioning against Kalman/RTS inference. A successful test is evidence for the checked identity, not for every untested implementation detail.

Lean certificates are not included. The ordinary linear algebra, calculus and probability identities are stated with their hypotheses and verified by independent calculations. The remaining uncertainty concerns unmeasured physical parameters, field representativeness, algorithmic assumptions and finite data; a formal certificate for algebra would not resolve those issues.
\clearpage
'''

def table_rows():
    r=[]
    for name in ['GPS','TRN','prior']:
        m=data['scenarios']['matched'][name]
        r.append(f"{name} & {m['wind_sample_rmse_mps']:.3f} & {m['mean_20m_layer_rmse_mps_80_to_300']:.3f} & {m['max_20m_layer_rmse_mps_80_to_300']:.3f}\\\\")
    return '\n'.join(r)

def landing_rows():
    r=[]
    for name in ['GPS','TRN','prior']:
        for m in data['scenarios']['matched'][name]['landing']:
            r.append(f"{name} & {m['height_m']:.1f} & {m['radial_rms_m']:.2f} & {m['predicted_axis_sd_m']:.2f} & {100*m['coverage95']:.2f}\\\\")
    return '\n'.join(r)

def coords(name):
    return ' '.join(f"({r['height_m']:.4f},{r['radial_rms_m']:.6f})" for r in data['scenarios']['matched'][name]['landing'])

def corrected_landing_rows():
    bad=data['scenarios']['correlated_vision_noise']['TRN']['landing']
    good=data['scenarios']['correlated_vision_noise_corrected']['TRN']['landing']
    return '\n'.join(f"{b['height_m']:.1f} & {100*b['coverage95']:.2f} & {100*g['coverage95']:.2f} & {g['mean_mahalanobis2']:.3f}\\\\" for b,g in zip(bad,good))

def layer_variance_rows():
    cases=[('GPS, white','matched','GPS'),('Vision, white','matched','TRN'),
           ('Coloured; ignored','correlated_vision_noise','TRN'),
           ('Coloured; tracked','correlated_vision_noise_corrected','TRN')]
    rows=[]
    for label,scenario,channel in cases:
        row=data['scenarios'][scenario][channel]['layer_uncertainty'][4]
        rows.append(f"{label} & {row['empirical_component_rmse_mps']:.3f} & {row['posterior_component_sd_mps']:.3f} & {row['diagonal_only_sd_mps']:.3f} & {100*row['coverage95']:.2f}\\\\")
    return '\n'.join(rows)

def initial_velocity_rows():
    labels=['Incorrectly fixed','Jointly estimated']
    return '\n'.join(f"{label} & {row['first_sample_rmse_mps']:.3f} & {row['first_sample_posterior_sd_mps']:.3f} & {100*row['first_sample_coverage95']:.2f}\\\\" for label,row in zip(labels,data['initial_velocity']['models']))

back=r'''
\section{Numerical verification and synthetic experiments}
\label{sec:verification}
\subsection{Reproducibility and the scope of the simulation}
All persistent calculations use Python and NumPy. The reference scripts are in \texttt{sim/maths}; \texttt{run\_verification.py} writes \texttt{results/verification.json}, a test report, and one synthetic profile CSV. The random seed is 20260930. There are 2000 independent synthetic descents per scenario, each with two independent horizontal axes. The suite currently has TESTCOUNT passing tests. The runs used Python PYTHONVERSION and NumPy NUMPYVERSION.

The mathematical reference implements only a small model, not onboard flight software. There is no terrain-image renderer or satellite-map matcher in these simulations. Projection and filter equations are checked numerically, but T1's match reliability, first-fix time and actual position accuracy have not been established. The ``TRN'' velocity channel below is a synthetic measurement channel with assumed noise; it is not a result from implemented terrain-relative navigation. Likewise the prior-only channel is not a complete inertial dead-reckoning implementation.

The synthetic atmosphere uses mean wind 5 m/s in each horizontal axis, fluctuation standard deviation 1 m/s, $L(z)=0.5\max(z,1\,\mathrm m)$, known constant downward speed 10.5 m/s, and initial horizontal position standard deviation 2 m per axis. The mean and covariance parameters are known to the matched estimator. Velocity measurements are at 5 Hz, with independent Gaussian component noise 0.1 m/s for the GPS-labelled channel and 0.6 m/s for the vision-labelled channel. The initial coupled turbulence/velocity covariance is the stationary solution at 300 m; the initial position error is independent. Ground regularisation is a scenario assumption.

The drift state is accumulated from the same uncertain initial position, rather than reset to a perfectly known current position. The endpoint predictor propagates the entire covariance and has no mast update. Post-flight wind retrieval uses an RTS smoother. This is a useful consistency test for the stated probabilistic system, while the nonlinear drag and coloured-noise experiments deliberately test departures from it.

\subsection{Independent identity checks}
\begin{longtable}{p{0.29\linewidth}p{0.61\linewidth}}
\toprule
Object & Independent verification\\
\midrule
\endfirsthead
\toprule Object & Independent verification\\\midrule\endhead
Plane homography & Direct 3D pinhole projection; homogeneous division; extrinsic and lever-arm cases.\\
Reprojection derivatives & Central finite differences of position and local rotation.\\
Attitude reset & Derivative of $\log(\exp(-a)\exp(a+\epsilon))$ checked against the right Jacobian.\\
Template matching & Valid-window direct NCC against zero-padded FFT correlation; contrast/offset invariance.\\
RANSAC and peak interpolation & Failure probability after ceiling the iteration count; recovery of a known quadratic peak.\\
Tilt information & Schur complement of the joint translation/tilt information; same-radius degeneracy and wide-field advantage.\\
Turbulence covariance & Lyapunov residual, closed-form matrix exponential and covariance identity; numerical Fourier quadrature.\\
Warped covariance & Positive eigenvalues on sample grids and equal-ratio correlation identity.\\
Stochastic discretisation & Block exponential against independently integrated transition and covariance ODEs; semigroup and PSD checks.\\
Wind inversion & Exact constant-input lag, limiting small/large lag, covariance/precision inverse equivalence.\\
Smoothing and likelihood & Direct batch GP posterior means/variances and log likelihood against two-state Kalman/RTS results.\\
Descent and atmosphere & Terminal transient against RK4, hydrostatic pressure forward/inverse, dry-adiabatic potential temperature, MOST derivative/integral checks.\\
Selection and robustness & Diminishing returns and greedy selection against exhaustive subsets; exhaustive finite-switch TLS example.\\
Probability and counterexamples & Gaussian radial protection probability; covariance counterexamples checked for PSD.\\
\bottomrule
\end{longtable}
The constant-step augmented transition and covariance agree with independently integrated ODEs within $2\times10^{-12}$ in the checked case. The direct GP and RTS posterior checks use a $2\times10^{-13}$ absolute tolerance. Their agreement verifies a different computational formulation of the same analytic model. Finite-grid PSD tests complement, but do not replace, the spectral and transformation proofs.

\subsection{Wind retrieval under matched assumptions}
The following are component errors against the simulated true wind. Layer errors average signed wind error over the sampled points within each 20 m layer first, then take RMS across runs. These equal-weight sample averages contain nine or ten samples per layer, not an exact continuous layer integral. ``Mean layer RMS'' is the mean of the eleven layer RMS values between 80 and 300 m; it is not a pooled pointwise RMS.
\begin{center}
\begin{tabular}{lrrr}
\toprule
Channel & Sample RMS & Mean layer RMS & Worst layer RMS\\
 & (m/s) & (m/s) & (m/s)\\
\midrule
WINDROWS
\bottomrule
\end{tabular}
\end{center}
For independent equal-quality axes the expected vector RMS is $\sqrt2$ times the component RMS. Thus the vision-labelled mean layer value corresponds to approximately 0.295 m/s vector RMS. This is error against simulated wind, not image-minus-GPS disagreement from a flight. It cannot establish S1 without real imagery, velocity error correlations, calibrated lag and truth uncertainty.

\subsection{Landing predictions under matched assumptions}
The RMS below is the two-dimensional distance to the synthetic touchdown point. Axis standard deviation is predicted by the estimator, and coverage is the fraction inside its nominal 95\% ellipse.
\begin{center}
\begin{tabular}{lrrrr}
\toprule
Channel & Height (m) & Radial RMS (m) & Axis SD (m) & Coverage (\%)\\
\midrule
LANDINGROWS
\bottomrule
\end{tabular}
\end{center}
At about 100 m the vision-labelled mean squared Mahalanobis distance is 2.003, close to the $\chi^2_2$ expectation of 2, and coverage is 95.15\%. The Monte Carlo standard error of nominal 95\% coverage is about 0.49 percentage points. This is evidence of internal calibration under known matched parameters. It is not evidence of real atmospheric calibration or a prediction of a specific competition landing error.

\begin{center}
\begin{tikzpicture}
\begin{axis}[width=.91\linewidth,height=67mm,
 xlabel={Height remaining (m)},ylabel={Synthetic radial RMS (m)},
 xmin=0,xmax=300,ymin=0,ymax=22,grid=major,
 legend style={at={(.03,.97)},anchor=north west,font=\small},
 tick label style={font=\small},label style={font=\small}]
\addplot[color=atlasblue,mark=*] coordinates {GPSCOORDS};\addlegendentry{GPS-labelled velocity}
\addplot[color=atlasorange,mark=square*] coordinates {TRNCOORDS};\addlegendentry{Vision-labelled velocity}
\addplot[color=black,dashed,mark=triangle*] coordinates {PRIORCOORDS};\addlegendentry{Prior only}
\end{axis}
\end{tikzpicture}
\end{center}
With no new information, the prior-only absolute endpoint covariance remains the same as height decreases. This is consistent with the fixed-endpoint theorem. Changing the remaining drift alone would answer a different covariance question.

\subsection{Averaging kernels reveal loss of independent information}
A separate piecewise-constant lag experiment on a 2.1 m grid between approximately 299 and 1 m uses the attached-eddy prior and $\ell=11.24$ m. It fixes the initial velocity perturbation to zero; it is a different boundary condition from the continuous-state descent Monte Carlo.
\begin{center}
\begin{tabular}{lrr}
\toprule
Noise model & DFS & Mean posterior SD (m/s)\\
\midrule
Independent GPS noise, 0.1 m/s & 43.12 & 0.226\\
Independent vision noise, 0.6 m/s & 14.98 & 0.402\\
Vision noise with known AR(1) correlation 0.85 & 12.52 & 0.572\\
\bottomrule
\end{tabular}
\end{center}
The correlated-noise row uses the full error covariance in the inverse. Correlation can sometimes improve particular high-frequency differences while reducing information in the mean or long-wave modes, so it is not a theorem that every correlation pattern reduces every DFS. These numbers apply to this chosen covariance and sampling grid.

\subsection{Deliberately mismatched sensor errors}
In a separate state-space experiment the true vision noise has the same marginal 0.6 m/s standard deviation but AR(1) correlation 0.85. The estimator incorrectly assumes independent noise. Near 100 m, coverage falls from 95.15\% to 81.6\%, and mean squared Mahalanobis distance rises from 2.003 to 3.499. Near 20.7 m, coverage is 50.65\%. The mean component layer RMS rises from 0.209 to 0.490 m/s.

The covariance underestimation is an outcome of a concrete misspecified model. It supports measuring temporal error covariance and incorporating it. Raising a protection multiplier after observing one test set is not a substitute for calibrating the underlying error model and evaluating fresh data.

\subsection{Verifying the remedy on exactly the same observations}
The added augmented-error filter uses the known $\rho=0.85$ and observation-error standard deviation 0.6 m/s. It is compared with the misspecified filter on the \emph{same} simulated trajectories and measurement realisations, not a new or easier dataset. The new error state has stationary prior variance $0.6^2$, and its innovation variance is $0.6^2(1-0.85^2)$. Its observation row adds that state to probe velocity, with no duplicate white observation variance.
\begin{center}
\begin{tabular}{rrrr}
\toprule
Height & Coverage: ignored & Coverage: tracked & Tracked mean $d^2$\\
(m) & (\%) & (\%) & \\
\midrule
CORRECTEDROWS
\bottomrule
\end{tabular}
\end{center}
Near 100 m the corrected coverage is 94.85\%, versus 81.6\% when the correlation is ignored. Near 20.7 m it is 95.10\%, versus 50.65\%. The corrected predictor's RMS remains larger than the independent-noise baseline because the observations contain less information about persistent velocity offsets. Honest covariance restores calibration; it does not turn coloured errors into independent measurements.

The corrected mean component layer RMS is 0.428 m/s; its pointwise wind RMS is 0.587 m/s. These are synthetic results with the error correlation known exactly. Estimating correlation from a short flight and allowing uncertainty in it is additional work. The result shows that accounting for correlation can fix the particular demonstrated failure, rather than proving the model adequate for all sensor errors.

\subsection{Checking layer uncertainty rather than only layer accuracy}
For the 160--180 m layer, the synthetic grid contains nine samples. The posterior standard deviation is computed from \emph{all} smoother cross-height covariances. ``Diagonal SD'' wrongly treats those posterior errors as independent and is included to quantify the mistake. Coverage is the fraction of 4000 independent component realisations (two axes for each of 2000 drops) within the nominal two-sided 95\% interval.
\begin{center}
\begin{tabular}{lrrrr}
\toprule
Channel & Actual RMS & Posterior SD & Diagonal SD & Coverage\\
 & (m/s) & (m/s) & (m/s) & (\%)\\
\midrule
LAYERVARIANCEROWS
\bottomrule
\end{tabular}
\end{center}
For the independent vision channel, the covariance calculation gives 0.215 m/s, close to the observed 0.219 m/s RMS, whereas dropping cross terms gives 0.113 m/s. With correctly tracked coloured errors, the figures are 0.434, 0.447 and 0.173 m/s. The GPS-labelled diagonal-only calculation instead gives a \emph{larger} value than the correct covariance calculation. This confirms that the omitted terms do not have a universally positive sign.

The full corrected model's layer coverages range from approximately 94.2\% to 95.6\% over the eleven bins. Coverage estimates fluctuate, and the eleven estimates from the same descents are correlated. They must not be treated as eleven independent experiments. Correct means and uncertainties for continuous physical layers also require a verified quadrature/overlap definition, rather than simply relabelling a sample average.

\subsection{Release-velocity uncertainty: an independently simulated boundary test}
An additional dense inversion uses the same 2.1 m lag grid and wind kernel, independent white velocity noise 0.1 m/s, and a zero-mean perturbation model $u_0=0.7w_1+\eta_0$ with independent $\eta_0$ standard deviation 0.8 m/s. The joint prior's minimum covariance eigenvalue is positive, approximately 0.0111 in its stated units. Generate wind and release velocity by that constructive rule; then generate observations through the lag operator. Compare the wrongly fixed $u_0=0$ model with the correctly marginalised joint model.
\begin{center}
\begin{tabular}{lrrr}
\toprule
Release condition & First-sample RMS & Posterior SD & 95\% coverage\\
 & (m/s) & (m/s) & (\%)\\
\midrule
INITIALROWS
\bottomrule
\end{tabular}
\end{center}
The first wind cell is represented at its midpoint, 1.05 m below the initial height; its velocity observation is at the cell's lower boundary, after 2.1 m of descent. Treating an uncertain release velocity as known makes that observation look like direct wind information. The wrong posterior states an SD of 0.198 m/s while actual RMS is 1.813 m/s. Its interval coverage is only 17.275\%. Joint marginalisation gives posterior SD 0.291 m/s, RMS 0.291 m/s and coverage 94.625\%. This is an explicit boundary-condition counterexample to assuming an exact initial velocity for convenience.

The unknown initial condition has no arbitrary mean bias in this experiment; the failure is a missing uncertainty and correlation. Across all wind samples, RMS changes from 0.319 to 0.241 m/s. The numerical model is illustrative, rather than an estimate of ejection dynamics. The continuous-state descent Monte Carlo already uses a coupled random initial velocity; these dense inverse results are a separate test of how to handle that boundary in a gridded retrieval.

\subsection{Independent nonlinear dynamics test of the lag assumption}
The forward calculation here is not the linear estimator's own simulator. It integrates a vector-quadratic horizontal drag equation with fixed vertical relative speed 10.5 m/s, mean wind 5 m/s, and smooth gust $2\sin(0.7t)$ m/s for 20 s using RK4 at 1 ms. A lateral coefficient ratio $r$ multiplies the horizontal force without changing the prescribed vertical force balance. Wind is recovered from the exact simulated acceleration, with no measurement noise.
\begin{center}
\begin{tabular}{rrr}
\toprule
Lateral coefficient ratio $r$ & Using $\tau=v_d/g$ & Using calibrated $\tau=(v_d/g)/r$\\
 & Wind RMS (m/s) & Wind RMS (m/s)\\
\midrule
0.5 & 0.5700 & 0.0112\\
1.0 & 0.0042 & 0.0042\\
2.0 & 0.4929 & 0.0008\\
\bottomrule
\end{tabular}
\end{center}
The residual after calibration comes from retaining a linear inverse for nonlinear drag. A wrong lateral coefficient alone can consume the nominal 0.5 m/s wind budget even with perfect velocity and acceleration. These coefficient ratios are sensitivity cases, not measured canopy parameters.

State-space lag mismatch is also tested by halving or doubling the true lag while retaining the nominal estimator. Mean vision component layer RMS becomes 0.230 and 0.311 m/s, respectively. The difference from the nonlinear experiment is expected: smoothing, forcing spectrum, prior regularisation and the error metric differ. Neither test permits treating the nominal lag as measured.

\subsection{Numerical convergence and ground-layer sensitivity}
Midpoint-frozen coefficients are tested with one, two and four substeps per 5 Hz interval. At about 100 m the vision prediction's axis SD is 4.8040006, 4.8042459 and 4.8043083 m, respectively. Halving the substep reduces the change by about a factor of four, consistent with second-order coefficient freezing in this example. One-to-four-substep change is about 0.0064\%, which is small relative to the assumed model uncertainty.

At four substeps, varying the scale floor from 0.5 to 2 m changes that same axis SD by less than $6\times10^{-6}$ m. This small influence occurs in the chosen 100 m forecast and lag model; it is not a claim that touchdown conditions or all near-ground retrievals are insensitive to boundary-layer physics. The example profiles and full sensitivity outputs are preserved, with their simulation labels.

\section{Coverage, unresolved questions, and engineering handoff}
\label{sec:handoff}
\subsection{Coverage of the proposal's mathematical index}
This table traces every topic in Appendix E. ``Derived'' means established with the stated hypotheses in this document, rather than experimentally validated. Stretch techniques are included mathematically even though they need not be flown.
\begin{longtable}{p{0.60\linewidth}p{0.28\linewidth}}
\toprule
Topic & Derivation location\\
\midrule
\endfirsthead
\toprule Topic & Derivation location\\\midrule\endhead
Pinhole camera; ground-plane homography & Navigation section\\
FFT NCC; sub-pixel peak; covariance; RANSAC & Navigation section\\
Terminal velocity; canopy sizing; hypsometric altitude & Physics section\\
Horizontal lag and inversion conditioning & Physics and retrieval sections\\
Pendulum specific force; forcing spectrum & Physics section\\
Error-state EKF; quaternion attitude; reset; reprojection derivatives & Navigation section\\
Innovation gating; NEES; information bound; protection level & Navigation section\\
Tilt--translation separation; timing; error budget; crossover & Navigation section\\
Shear path and frozen-field approximation & Physics section\\
Drop-Dryden transverse covariance; scale deformation; exact states & Section~\ref{sec:turbulence}\\
Kalman filtering; smoothing; Gaussian-process inverse & Section~\ref{sec:retrieval}\\
Averaging kernels; degrees of freedom; layer covariance & Section~\ref{sec:retrieval}\\
Landing position and time; Mahalanobis calibration & Section~\ref{sec:landing}\\
Marginal likelihood; model-comparison limitations & Section~\ref{sec:retrieval}\\
Potential temperature; MOST; Richardson number; roughness fit & Boundary-layer section\\
Pendulum response as a shear/turbulence diagnostic & Physics section\\
Landing uncertainty shrinkage & Section~\ref{sec:landing}: fixed-endpoint theorem; moving-drift counterexamples\\
Map-datum gauge; survey uncertainty & Navigation section\\
Truncated least squares; lifted QCQP and SDP certificate conditions & Navigation section\\
Submodular landmark selection and its normalised guarantee & Navigation section\\
Uncertainty-aware notional target selection & Navigation section\\
Footprint, blur, sample count, storage, compute, power, loads and radio & Engineering section\\
\bottomrule
\end{longtable}

\subsection{Completed and strongest evidence}
The proposal's mathematics has been assembled into a coherent reference with a consistent camera convention, stated physical assumptions, explicit derivations, corrected probability claims, and worked examples. The exact turbulence realisation, covariance positivity, descent time conversion, inverse lag, Gaussian conditioning, smoothing, and fixed-endpoint monotonicity have analytic derivations. The numerical suite verifies independent formulations and limiting cases, with retained synthetic outputs.

The strongest simulation result is conditional: matched known parameters yield well-calibrated landing ellipses and sub-0.5 m/s layer errors in the synthetic case. The strongest engineering caution is also quantified: unmodelled error correlation severely reduces coverage, and uncalibrated lateral response can produce order-0.5 m/s wind bias despite perfect motion measurements.

The built-in editor's compilation was attempted and failed before producing a PDF with the environment message ``Unable to find standard directories for platform.'' Source brace/environment checks, unique-label checks and reference checks are separate from compilation. No rendered page layout, line overflow or package availability has been verified. The saved standalone source is preserved for the editor or another LaTeX installation; no compiler or plugin was installed.

\subsection{Decisions and assumptions}
The original proposal has been preserved as source material. This document records corrections without adopting a new mission architecture or declaring a physically unmeasured model to be true. No large dependency, external service, flight rule or canopy design has been introduced. The numerical model uses NumPy, known Gaussian parameters, a stated near-ground scale floor, two independent horizontal axes, and labelled synthetic velocity channels. Certifiable registration is described with its necessary primal-feasibility and optimality conditions; no actual SDP flight fix or numerical global-optimality certificate is claimed.

\subsection{What still requires measurements or project decisions}
\begin{enumerate}
\item \textbf{Canopy dynamics.} Measure terminal speed, lateral response, deployment transient and pendulum motion. Fit $\tau_h$ or a lateral drag model with confidence intervals. A vertical descent-rate measurement alone does not select the horizontal model.
\item \textbf{Camera and inertial calibration.} Determine intrinsics, distortion, extrinsics, lever arms, exposure timing and timestamp uncertainty. Test map matches over representative terrain; measure failures, first-fix time, bias and temporal covariance.
\item \textbf{Truth quality.} Survey map control points with quantified uncertainty, and log independent position/velocity truth. GPS alone does not supply all fifteen navigation error-state components for a full-state NEES calculation.
\item \textbf{Atmospheric inference.} Measure thermistor response in flight-like airflow, pressure/density uncertainties, mast representativeness and relevant stability flux information or jointly identifiable MOST profiles. Assess displacement height and heterogeneous fetch before reporting roughness.
\item \textbf{Retrieval independence.} Quantify the shared attitude, lag and prior errors in image-versus-GPS wind comparisons. A common bias can cancel in their difference.
\item \textbf{Calibration across drops.} Evaluate prediction ellipses and time intervals on independent descents, with predeclared metrics and model tuning separate from validation. A single flight cannot test 95\% long-run landing coverage.
\item \textbf{Stretch certification.} Select the exact registration relaxation, bounds and solver tolerances before issuing an SDP certificate; rank appearance alone is insufficient. Safe-target reachability is hypothetical for a passive canopy.
\item \textbf{Requirements.} Confirm the current competition rules, actual radio configuration, mass convention, load cases, battery and processing budgets before asserting compliance.
\end{enumerate}

\subsection{Highest-value next action}
Conduct a short, time-synchronised suspension/drop calibration with image logging, IMU, pressure and independent velocity truth. Use it first to estimate canopy lateral response and sensor-error correlation, then rerun the same mathematical model with those measured uncertainties. This directly tests the two assumptions that dominate the demonstrated gap between an internally consistent simulation and a reliable wind/landing result. Preserve raw logs and calibration metadata so later navigation and atmosphere analysis can be repeated.

\subsection{Reference provenance}
The local secondary mission proposal is \texttt{wiki/trn\_secondary\_mission\_proposal.md}, draft 27 September 2026. Primary literature, manufacturer formulas and theorem sources are linked beside the relevant derivations. Derivations shown here are worked calculations for this project, not copied passages from those sources.

The connected reference folder is \url{https://drive.google.com/drive/folders/1ugYnjOalW-hIZqXO9nSJtGS6KleoJ3KW}. Its listed Control Systems and Design Reviews folders were empty when inspected. The fetched historical regulations were \url{https://drive.google.com/file/d/1LubKeC729i9NbmgHwG-wzu4gwhDO1PfJ/view}. No Drive document was edited. No recent Git history was available because this local folder was not a Git checkout.
\end{document}
'''

substitutions={'TESTCOUNT':str(data['tests_run']),'PYTHONVERSION':data['python'],'NUMPYVERSION':data['numpy'],
               'WINDROWS':table_rows(),'LANDINGROWS':landing_rows(),
               'CORRECTEDROWS':corrected_landing_rows(),'LAYERVARIANCEROWS':layer_variance_rows(),'INITIALROWS':initial_velocity_rows(),
               'GPSCOORDS':coords('GPS'),'TRNCOORDS':coords('TRN'),'PRIORCOORDS':coords('prior')}
for key,value in substitutions.items(): back=back.replace(key,value)
parts=[front]
for name in ['physics_fragment.tex','navigation_fragment.tex','core_fragment.tex']:
    parts.append((BASE/name).read_text(encoding='utf-8'))
parts.append(back)
target=ROOT/'wiki'/'cansat_mathematics.tex'
target.write_text('\n\n'.join(parts),encoding='utf-8')
print(f'Written standalone document: {target}; {target.stat().st_size} bytes')
