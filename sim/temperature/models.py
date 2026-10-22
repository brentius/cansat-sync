"""GPL-3.0-only. Design-stage models for the existing analogue air probe.

No flight/bench measurements. SI units except temperatures labelled _c.
Nominal TE R(T) data are literature values, not calibration of a purchased probe.
Thermal tau, lead coupling, radiation and excitation timing are assumptions.
"""
import math
from dataclasses import dataclass

import numpy as np

TE_SOURCE = ('https://www.te.com/commerce/DocumentDelivery/DDEController?'
             'Action=srchrtrv&DocFormat=pdf&DocLang=English&DocNm=GA10K3MCD1'
             '&DocType=Data+Sheet&PartCntxt=GA10K3MCD1')
# Selected entries, page 3, GA10K3MCD1 02/2018. Keep source values separate
# from computed fits. Restricted range is intentional; no extrapolation.
TE_T_C = np.array([-20., -10., 0., 10., 20., 25., 30., 40., 50.])
TE_R_OHM = np.array([97006.9, 55303.9, 32650.9, 19903.2, 12493.3,
                     10000., 8056.1, 5325., 3601.1])
TE_AIR_G_W_K = 0.3e-3  # datasheet typical still-air dissipation, NOT installed G
R_REF = 150000.0        # existing circuit, 0.1% reference resistor
V_EXC = 3.0


def _positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')


def beta_resistance(temp_c, r25=10000., beta=3976.):
    """Constant-beta approximation; useful for exposing its model bias."""
    t = np.asarray(temp_c, dtype=float) + 273.15
    if np.any(~np.isfinite(t)) or np.any(t <= 0):
        raise ValueError('temperature must be above absolute zero')
    _positive(r25, 'r25')
    _positive(beta, 'beta')
    return r25*np.exp(beta*(1/t - 1/298.15))


def beta_temperature(resistance, r25=10000., beta=3976.):
    r = np.asarray(resistance, dtype=float)
    if np.any(~np.isfinite(r)) or np.any(r <= 0):
        raise ValueError('resistance must be finite and positive')
    _positive(r25, 'r25')
    _positive(beta, 'beta')
    inverse_t = 1/298.15 + np.log(r/r25)/beta
    if np.any(inverse_t <= 0):
        raise ValueError('resistance outside beta model domain')
    return 1/inverse_t - 273.15


def fit_steinhart_hart(temp_c, resistance):
    """Fit 1/T=A+B ln(R)+C ln(R)^3; never claims physical calibration."""
    t, r = np.asarray(temp_c, float), np.asarray(resistance, float)
    if t.ndim != 1 or r.shape != t.shape or len(t) < 3:
        raise ValueError('need matching vectors with at least three points')
    if np.any(~np.isfinite(t)) or np.any(t <= -273.15) or np.any(~np.isfinite(r)) or np.any(r <= 0):
        raise ValueError('invalid calibration points')
    x = np.log(r)
    design = np.column_stack((np.ones(len(r)), x, x**3))
    coefficients, _, rank, _ = np.linalg.lstsq(design, 1/(t+273.15), rcond=None)
    if rank < 3:
        raise ValueError('calibration is rank deficient')
    return coefficients


def sh_temperature(resistance, coefficients):
    r = np.asarray(resistance, float)
    if np.any(~np.isfinite(r)) or np.any(r <= 0):
        raise ValueError('invalid resistance')
    x = np.log(r)
    a, b, c = coefficients
    inverse_t = a + b*x + c*x**3
    if np.any(~np.isfinite(inverse_t)) or np.any(inverse_t <= 0):
        raise ValueError('outside fitted model domain')
    return 1/inverse_t - 273.15


SH_COEFF = fit_steinhart_hart(TE_T_C[[0, 5, 8]], TE_R_OHM[[0, 5, 8]])


def nominal_resistance(temp_c):
    """Interpolate sourced ln(R) vs 1/T, only -20..50 C; not exact R(T)."""
    t = np.asarray(temp_c, float)
    if np.any(~np.isfinite(t)) or np.any(t < -20) or np.any(t > 50):
        raise ValueError('nominal table restricted to -20..50 C')
    return np.exp(np.interp(1/(t+273.15), (1/(TE_T_C+273.15))[::-1], np.log(TE_R_OHM)[::-1]))


def divider(resistance, v_exc=V_EXC, r_ref=R_REF):
    _positive(v_exc, 'excitation')
    _positive(r_ref, 'reference resistor')
    r = np.asarray(resistance, float)
    if np.any(~np.isfinite(r)) or np.any(r <= 0):
        raise ValueError('invalid resistance')
    current = v_exc/(r_ref+r)
    return current*r, current, current**2*r


def resistance_from_volts(v_ntc, v_exc, r_ref=R_REF):
    """Voltages must already reflect their individual PGA full-scale ranges."""
    _positive(r_ref, 'reference resistor')
    a, b = np.broadcast_arrays(np.asarray(v_ntc, float), np.asarray(v_exc, float))
    if np.any(~np.isfinite(a)) or np.any(~np.isfinite(b)) or np.any(a <= 0) or np.any(b <= a):
        raise ValueError('require 0 < V_ntc < V_exc')
    return r_ref*a/(b-a)


def adc_code(voltage, full_scale):
    """Signed ideal 16-bit ADC; refuse saturation instead of hiding it."""
    _positive(full_scale, 'full scale')
    v = np.asarray(voltage, float)
    code = np.rint(v/full_scale*32768)
    if np.any(~np.isfinite(v)) or np.any(code < -32768) or np.any(code > 32767):
        raise ValueError('ADC saturation')
    return code.astype(np.int16)


def code_volts(code, full_scale):
    _positive(full_scale, 'full scale')
    return np.asarray(code, float)*full_scale/32768


def readout_jacobian(v_ntc, v_exc, coefficients=SH_COEFF):
    """dT/d[V_ntc,V_exc], K/V, including ratiometric denominator."""
    r = float(resistance_from_volts(v_ntc, v_exc))
    t = float(sh_temperature(r, coefficients)) + 273.15
    _, b, c = coefficients
    dt_dlogr = -t*t*(b+3*c*math.log(r)**2)
    return dt_dlogr*np.array([1/v_ntc+1/(v_exc-v_ntc), -1/(v_exc-v_ntc)])


def pair_schedule(data_rate_sps, sample_hz, settle_s=0.001, bus_s=0.001,
                  startup_s=25e-6, rate_fraction=0.9):
    """Conservative candidate timing; readiness polling remains mandatory.

    TI rate tolerance -10%; 25 us single-shot wake. Two conversions per pair.
    settle_s and total bus_s are assumed, not measured electrical timings.
    """
    if data_rate_sps not in (8, 16, 32, 64, 128, 250, 475, 860):
        raise ValueError('unsupported ADS1115 rate')
    _positive(sample_hz, 'sample frequency')
    _positive(rate_fraction, 'rate fraction')
    if rate_fraction > 1:
        raise ValueError('rate fraction must be at most one for this conservative schedule')
    if any(not math.isfinite(x) or x < 0 for x in (settle_s, bus_s, startup_s)):
        raise ValueError('timings must be finite and non-negative')
    on_s = settle_s+bus_s+2*(startup_s+1/(data_rate_sps*rate_fraction))
    duty = on_s*sample_hz
    return dict(on_s=on_s, duty=duty, feasible=duty <= 1,
                maximum_pair_hz=1/on_s, conversion_midpoint_gap_s=startup_s+1/(data_rate_sps*rate_fraction))


def periodic_heating(power_w, conductance_w_k, tau_s, period_s, on_s):
    """Exact steady-cycle rise for constant power on/off; K, not measurements."""
    for value, name in ((conductance_w_k, 'conductance'), (tau_s, 'tau'), (period_s, 'period')):
        _positive(value, name)
    if not math.isfinite(power_w) or power_w < 0 or not 0 <= on_s <= period_s:
        raise ValueError('invalid pulse')
    rise_on = power_w/conductance_w_k
    peak = rise_on*(-math.expm1(-on_s/tau_s))/(-math.expm1(-period_s/tau_s))
    return dict(peak_k=peak, trough_k=peak*math.exp(-(period_s-on_s)/tau_s),
                mean_k=rise_on*on_s/period_s)


@dataclass(frozen=True)
class Thermal:
    tau_s: float = 1.0             # ASSUMED effective time constant, no air measurement
    air_g_w_k: float = TE_AIR_G_W_K # SOURCED typical still air; sensitivity required
    lead_g_w_k: float = 0.0        # ASSUMED
    radiation_w: float = 0.0       # ASSUMED net absorbed heat

    def __post_init__(self):
        _positive(self.tau_s, 'tau')
        _positive(self.air_g_w_k, 'air conductance')
        if not math.isfinite(self.lead_g_w_k) or self.lead_g_w_k < 0 or not math.isfinite(self.radiation_w):
            raise ValueError('invalid thermal parameter')

    @property
    def heat_capacity_j_k(self):
        return self.tau_s*(self.air_g_w_k+self.lead_g_w_k)


def thermal_trace(time_s, air_c, body_c, on, config=Thermal(), initial_c=None):
    """RK4 nonlinear electrothermal model with ZOH inputs over each interval.

    C dT/dt = Gair(Tair-T)+Glead(Tbody-T)+P_ntc(T,on)+Qrad.
    Samples on[k] apply on [time[k],time[k+1]); pulse edges must be in time_s.
    on=0/1 resolves pulses; intermediate values approximate duty-averaged heat.
    A tiny-bead one-node model cannot establish multi-node installed behaviour.
    """
    t = np.asarray(time_s, float)
    if t.ndim != 1 or len(t) < 2 or np.any(~np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('need increasing finite times')
    air, body, excitation = [np.broadcast_to(np.asarray(x, float), t.shape) for x in (air_c, body_c, on)]
    if any(np.any(~np.isfinite(x)) for x in (air, body, excitation)) or np.any((excitation < 0) | (excitation > 1)):
        raise ValueError('invalid forcing')
    y = np.empty_like(t)
    y[0] = air[0] if initial_c is None else initial_c
    if not math.isfinite(y[0]) or not -20 <= y[0] <= 50:
        raise ValueError('initial probe temperature outside nominal table domain')
    capacity = config.heat_capacity_j_k
    for k, dt in enumerate(np.diff(t)):
        if dt > config.tau_s/4:
            raise ValueError('thermal integration interval too large; refine timestamps')
        def rhs(temp):
            power = float(divider(nominal_resistance(temp))[2])*excitation[k]
            return (config.air_g_w_k*(air[k]-temp)+config.lead_g_w_k*(body[k]-temp)
                    +config.radiation_w+power)/capacity
        a = rhs(y[k])
        b = rhs(y[k]+dt*a/2)
        c = rhs(y[k]+dt*b/2)
        d = rhs(y[k]+dt*c)
        y[k+1] = y[k]+dt*(a+2*b+2*c+d)/6
    return y
