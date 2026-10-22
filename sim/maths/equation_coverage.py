"""GPL-3.0-only. Explicit traceability for all249 numbered PDF equations.

Each equation is assigned to a mathematical family and executable checks.
Family coverage is not a claim that every displayed proof line is separately
tested, or that a physical prescription/theorem was proved by simulation.
"""
import ast
from pathlib import Path
import re

# section, first, last, topic, tests, experiment, engineering restriction
GROUPS=[
 (2,1,5,'terminal drag, sizing and sensitivity','test_density_terminal_pressure_and_richardson_jacobians','atmosphere','drag area and mass are uncalibrated'),
 (2,6,12,'vertical transient and variable-speed time','test_terminal_descent_vs_ode test_time_height_speed_covariance','atmosphere','instantaneous deployment; quasi-steady density model conditional'),
 (2,13,19,'hydrostatic height and density','test_hypsometric_forward_inverse test_density_terminal_pressure_and_richardson_jacobians','atmosphere','hydrostatic dry-air assumptions; common pressure datum'),
 (2,20,23,'isotropic vector drag and lag','test_nonlinear_horizontal_drag_linear_limit test_constant_horizontal_lag_against_rk4','baseline/nonlinear_drag','terminal vertical balance alone does not calibrate lateral lag'),
 (2,24,26,'anisotropic canopy response','test_constant_horizontal_lag_against_rk4','baseline/nonlinear_drag','scenario coefficient ratios; no measured canopy law'),
 (2,27,28,'lag frequency response and sensor lever velocity','test_frequency_lag_response_and_averaging_kernel_eigenvalues test_lever_arm_and_pendulum_specific_force','atmosphere/shared_swing_wind_bias','flexible suspension geometry requires measurement'),
 (2,29,33,'pendulum force, forcing and transfer spectrum','test_lever_arm_and_pendulum_specific_force test_forced_pendulum_frequency_response test_pendulum_constant_forcing_counterexample','atmosphere/shared_swing_wind_bias','linear damping and fixed-pivot model assumed'),
 (2,34,38,'shear trajectory and frozen-field estimates','test_shear_path_and_lag_are_distinct','baseline/nonlinear_drag','crossing-time scaling is an estimate; not empirical validation'),
 (3,1,6,'potential temperature, buoyancy and sensor lag','test_dry_adiabatic_potential_temperature_invariant test_thermistor_ramp_and_inverse','atmosphere/thermistor','dry unsaturated parcel model; no thermistor calibration'),
 (3,7,14,'MOST closures and integrated profiles','test_most_integral_and_derivative test_most_lower_boundary_and_neutral_limit','atmosphere/roughness','empirical closures; homogeneous constant-flux conditions unverified'),
 (3,15,22,'Richardson sensitivity and stability theorem algebra','test_density_terminal_pressure_and_richardson_jacobians test_taylor_goldstein_transformation','atmosphere','transformation identity tested; Miles-Howard general proof is analytic, not a simulated physical cutoff'),
 (3,23,25,'flux-gradient stability relation','test_stable_most_gradient_relation_and_ill_conditioning','atmosphere','specified Dyer heat closure, gradient Ri not bulk Ri'),
 (3,26,28,'neutral GLS fit and roughness uncertainty','test_neutral_gls_and_roughness_gradient','atmosphere/roughness','known displacement and heights; no actual surface roughness identification'),
 (4,1,5,'camera footprint and optical flow blur','test_storage_and_corner_blur test_optical_flow_against_moving_point_projection','images','rectilinear lens; real photon signal and distortion unmeasured'),
 (4,6,7,'storage and correlated averaging','test_storage_and_corner_blur test_layer_variance_equal_correlation_closed_form','processing','8bit storage; no measured storage stalls'),
 (4,8,10,'energy, current and loading','test_energy_load_radio_and_binomial_limits','processing','illustrative energy/load inputs; no battery or rail measurements'),
 (4,11,14,'LoRa accounting and free-space link loss','test_lora_packet_example test_energy_load_radio_and_binomial_limits','processing/radio_sweep','no current-rule compliance or real link-range claim'),
 (5,1,3,'camera frames and lever arm','test_homography_with_extrinsics_and_lever test_right_error_reprojection_with_lever','images','intrinsics/extrinsics assumed known'),
 (5,4,6,'homography determinant and footprint','test_homography_determinant_and_datum_gauge test_renderer_ray_plane_and_interpolation','images','single plane fails with terrain relief'),
 (5,7,12,'timing, tilt geometry and information','test_optical_flow_against_moving_point_projection test_tilt_information_degeneracy','images','timing calibration and pose nuisance parameters required'),
 (5,13,17,'NCC FFT and integral normalization','test_ncc_integrals_against_direct_all_windows test_coarse_ncc_recovers_large_shift','processing/NCC','constant patches rejected; aliases remain possible'),
 (5,18,21,'subpixel peaks and photometric Fisher information','test_joint_quadratic_peak test_photometric_information_schur','images','curvature-to-covariance needs calibration'),
 (5,22,25,'RANSAC and two-point similarity','test_ransac_rounding test_parabola_and_ransac','images','success formula assumes independent valid minimal samples; renderer uses local peak gate rather than RANSAC'),
 (5,26,33,'right error and nonlinear ESKF linearization','test_eskf_dynamics_nonlinear_finite_difference test_quaternion_exact_propagation','linear_navigation','matched local-linear ensemble; not a full nonlinear flight ESKF'),
 (5,34,37,'15-state covariance discretization and attitude propagation','test_15state_discretization_covariance_ode test_quaternion_exact_propagation','linear_navigation','frozen coefficients and known noise density'),
 (5,38,41,'landmark reprojection derivatives','test_right_error_reprojection_with_lever test_pose_fit_from_independent_noiseless_projection','images','correct correspondences for local covariance'),
 (5,42,49,'navigation update, injection and reset','test_joint_correlated_update_against_batch_gaussian test_exact_reset_derivative','linear_navigation','no nonlinear injection in the linear ensemble; reset derivative checked independently'),
 (5,50,56,'VO signs, scaling and correlated update','test_downward_scale_correction_and_signed_velocity test_shared_image_velocity_temporal_covariance test_joint_correlated_update_against_batch_gaussian','baseline/correlated_vision_noise_corrected','known rotation/height in signed VO identity'),
 (5,57,59,'error budget and shared source covariance','test_truth_difference_and_landing_cross_covariance test_tilt_schur_information','images','design budget not independently additive to joint posterior'),
 (5,60,65,'Fisher bounds, consistency and imperfect truth','test_photometric_information_schur test_truth_difference_and_landing_cross_covariance','linear_navigation','CRLB general theorem analytic; known Gaussian noise and independent descents'),
 (5,66,68,'protection probability and empirical bound','test_gaussian_radial_PL_and_gating test_energy_load_radio_and_binomial_limits','images','conditional Gaussian PL does not cover wrong associations or systematic bias'),
 (5,69,75,'TLS lifting and SDP certificate conditions','test_TLS_quadratic_lift_exact_cases test_tls_enumeration_global_bound test_tls_lift_and_analytic_dual_certificate','test_feasibility','analytic zero-cost primal/dual numerical certificate exercised; no general noisy SDP solve or onboard certificate'),
 (5,76,81,'normalized logdet greedy selection','test_normalized_logdet_greedy','test_navigation_extra','fixed additive PSD information; theorem proof analytic'),
 (5,82,85,'expected hazard cost and notional control bound','test_expected_hazard_cost_and_control_bound','test_feasibility','half-plane case exercised; passive canopy has zero commanded divert'),
 (6,1,5,'transverse covariance and incompressibility','test_incompressible_tensor_divergence test_lyapunov_and_covariance','model_selection','isotropy, incompressibility and vertical path assumed'),
 (6,6,11,'Fourier positivity and physical spectrum','test_spectrum_fourier_integral test_spectral_variance_and_brownian_time_change','model_selection','PSD proof is spectral; finite grids are checks, not universal proof'),
 (6,12,18,'two-state SDE realization','test_lyapunov_and_covariance test_transfer_function_resolvent_not_comma','baseline','PDF transfer numerator contains comma typo; correct1+sqrt(3)*i*omega'),
 (6,19,23,'height warp and covariance deformation','test_warp_psd_ratio test_warp_integrals_and_intensity_units','model_selection','line deformation not proof of global3D turbulent field'),
 (6,24,25,'Brownian time change and ground limit','test_spectral_variance_and_brownian_time_change','baseline/sensitivity','positive scale floor is numerical scenario, not calibrated ground physics'),
 (6,26,26,'empirical MIL intensity prescription','test_warp_integrals_and_intensity_units','test_feasibility','feet/metres conversion checked; empirical coefficients not physically validated'),
 (7,1,4,'lag convolution, boundary memory and frequency','test_lag_limits_and_constant_wind test_frequency_lag_response_and_averaging_kernel_eigenvalues','baseline','known lag; uncertain initial velocity must be marginalised'),
 (7,5,10,'Gaussian inverse and covariance precision','test_retrieval_alternative_precision_and_dfs','baseline','joint Gaussian prior and noise; fixed hyperparameters'),
 (7,11,12,'averaging kernel and resolved modes','test_frequency_lag_response_and_averaging_kernel_eigenvalues','baseline/averaging_kernels','DFS not a count of independent20m bins'),
 (7,13,18,'continuous wind/velocity/drift and exact steps','test_discretization_independent_ode','baseline/sensitivity','coefficient-freezing checked by refinement'),
 (7,19,22,'Gaussian update and Joseph covariance','test_kalman_gp_and_rts_equivalence','baseline','fixed linear Gaussian model'),
 (7,23,25,'RTS smoothing and wind output','test_kalman_gp_and_rts_equivalence','baseline','offline future data used for smoothing only'),
 (7,26,29,'shared-image noise and S1 comparison','test_shared_image_velocity_temporal_covariance test_layer_variance_equal_correlation_closed_form','atmosphere/shared_swing_wind_bias','agreement does not exclude shared motion/prior bias'),
 (7,30,34,'stationary AR1 augmented-error remedy','test_ar1_stationarity_and_colored_batch_equivalence','baseline/correlated_vision_noise_corrected','known correlation; does not identify real temporal errors'),
 (7,35,42,'continuous layer average and smoother cross covariance','test_layer_overlap_integral_and_covariance test_ar1_stationarity_and_colored_batch_equivalence','baseline','baseline metrics remain sample averages; overlap identity separately exercised'),
 (7,43,48,'joint release-velocity marginalisation','test_initial_velocity_marginal_against_joint_gaussian','baseline/initial_velocity','synthetic valid joint prior; real boundary uncertainty unknown'),
 (7,49,49,'marginal likelihood and model comparison','test_kalman_gp_and_rts_equivalence','model_selection','fixed-model likelihood is not a fitted-model Bayes factor'),
 (8,1,1,'absolute landing covariance with cross terms','test_truth_difference_and_landing_cross_covariance','baseline','requires current-position uncertainty and all cross terms'),
 (8,2,4,'touchdown time integral and sensitivity','test_time_height_speed_covariance','atmosphere/touchdown_time','no measured density/vertical-wind/deployment uncertainty'),
 (8,5,5,'landing ellipse calibration','test_gaussian_radial_PL_and_gating','baseline','independent synthetic descents, not a measured coverage claim'),
 (8,6,6,'fixed endpoint covariance monotonicity','test_fixed_endpoint_covariance_conditioning_monotonic','baseline','theorem conditional on fixed model, endpoint and nested data'),
 (8,7,9,'remaining-drift counterexamples and positive prior integral','test_shrinkage_sufficient_condition','baseline/shrinkage_counterexample','negative lobes alone do not prove growing drift uncertainty'),
]


def inventory(text, unused_suite=None):
    tests={}
    for path in Path(__file__).parent.glob('test*.py'):
        tree=ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node,ast.FunctionDef) and node.name.startswith('test_'):
                tests[node.name]=path.name+':'+str(node.lineno)
    lookup={}
    for section,first,last,topic,names,experiment,restriction in GROUPS:
        for name in names.split():
            if name not in tests: raise ValueError('Missing executable check '+name)
        for i in range(first,last+1):
            key=f'{section}.{i}'
            if key in lookup: raise ValueError('Duplicate coverage '+key)
            lookup[key]=(topic,names,experiment,restriction)
    pages={}; heading=''; page=0
    for line in text.splitlines():
        if line.startswith('=== PAGE'): page=int(re.search(r'\d+',line)[0])
        if re.match(r'^[2-8]\.\d+\s+[A-Z]',line): heading=line
        for number in re.findall(r'\(([2-8]\.\d+)\)',line):
            if number not in pages: pages[number]=(page,heading)
    if set(pages)!=set(lookup): raise ValueError(f'Equation inventory mismatch: PDF-only{set(pages)-set(lookup)}, registry-only{set(lookup)-set(pages)}')
    rows=[]
    for number in sorted(pages,key=lambda x:tuple(map(int,x.split('.')))):
        topic,names,experiment,restriction=lookup[number]
        rows.append({'equation':number,'pdf_page':pages[number][0],'subsection':pages[number][1],
                     'family':topic,'evidence_level':'representative family check; physical validity conditional',
                     'checks':'; '.join(tests[name]+' '+name for name in names.split()),
                     'experiment':experiment,'restriction':restriction})
    return rows
