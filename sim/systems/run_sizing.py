"""GPL-3.0-only. Evaluates the sizing models and writes results/sizing.json.

Provenance tags on inputs:
  RULE      UK CanSat Guidelines 2025-26 (2026-27 edition not yet published)
  PROPOSAL  wiki/trn_secondary_mission_proposal.md
  SOURCED   published figure, source named alongside
  ASSUMED   engineering assumption awaiting a measurement
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'code'))
import sizing as sz  # noqa: E402
from cansat import lora  # noqa: E402
from cansat.telemetry import PACKET_SIZE  # noqa: E402

# RULE
MASS_MIN, MASS_MAX = 0.300, 0.350
SPEED_FLOOR = 10.0
CONNECTION_FORCE = 50.0
POWERED_HOURS = 3.0
RELEASE_HEIGHTS = (300.0, 400.0)
# PROPOSAL
DESIGN_SPEED = 10.5
FRAME_RATE = 5.0
G = sz.G0
MAP_FLOOR = 80.0
# ASSUMED: sea-level launch site in March, 0 to 15 degC, 980 to 1035 hPa.
RHO_STD = 1.225
RHO_MIN = sz.air_density(98000.0, 288.15)
RHO_MAX = sz.air_density(103500.0, 273.15)
# ASSUMED: drag coefficients on nominal area for candidate canopy types.
# Recalled from Knacke, Parachute Recovery Systems Design Manual (1992),
# table 5-1; not re-read this session, so treat as a range to be measured.
CD_RANGE = (0.60, 0.75, 0.90)
OPENING_CX = 1.7


def canopy():
    out = {'density': {'standard': RHO_STD, 'march_min': RHO_MIN, 'march_max': RHO_MAX}}
    out['diameter_m'] = [
        {'speed': v, 'mass_kg': m, 'cd': cd,
         'cd_s_m2': sz.drag_area(m, v, RHO_STD),
         'diameter_m': sz.nominal_diameter(sz.drag_area(m, v, RHO_STD), cd)}
        for v in (10.5, 11.0, 12.0) for m in (MASS_MIN, 0.325, MASS_MAX) for cd in CD_RANGE]

    # A canopy cut from a handbook Cd, before any drop test.
    cd_s = sz.drag_area(0.325, DESIGN_SPEED, RHO_STD)
    out['paper_design_spread'] = {
        'basis': 'cut for 10.5 m/s assuming Cd = 0.75 at 325 g, standard density',
        'speed_if_cd_0.90_mass_300g_dense_air': sz.descent_speed(MASS_MIN, RHO_MAX, cd_s*0.90/0.75),
        'speed_if_cd_0.60_mass_350g_thin_air': sz.descent_speed(MASS_MAX, RHO_MIN, cd_s*0.60/0.75),
    }

    # A canopy trimmed until a drop test reads the design speed at standard density.
    drop_sigma = sz.drop_test_relative_sigma(100.0, 100.0/DESIGN_SPEED, 2.0, 0.2)
    out['measured_design_spread'] = {
        'basis': 'trimmed to a measured 10.5 m/s at standard density, same mass on launch day',
        'speed_dense_air': DESIGN_SPEED*math.sqrt(RHO_STD/RHO_MAX),
        'speed_thin_air': DESIGN_SPEED*math.sqrt(RHO_STD/RHO_MIN),
        'single_drop_relative_sigma': drop_sigma,
        'target_for_floor_one_drop': sz.target_speed_for_floor(SPEED_FLOOR, RHO_STD, RHO_MAX, drop_sigma),
        'target_for_floor_five_drops': sz.target_speed_for_floor(SPEED_FLOOR, RHO_STD, RHO_MAX, drop_sigma/math.sqrt(5)),
        'mass_lever': 'speed scales as sqrt(mass): 350 g instead of 325 g adds %.2f%%'
                      % (100*(math.sqrt(0.350/0.325) - 1)),
    }

    out['drop_test'] = {
        'distance_to_95pct_terminal_m': sz.distance_to_fraction(DESIGN_SPEED, 0.95),
        'distance_to_99pct_terminal_m': sz.distance_to_fraction(DESIGN_SPEED, 0.99),
        'naive_speed_from_15m_drop': 15.0/_fall_time(DESIGN_SPEED, 15.0),
        'naive_speed_from_100m_drop': 100.0/_fall_time(DESIGN_SPEED, 100.0),
    }

    out['opening'] = {
        'steady_drag_n': 0.325*G,
        'airspeed_reaching_50n': sz.speed_for_opening_force(CONNECTION_FORCE, cd_s, RHO_STD, OPENING_CX),
        'force_at_20_mps': sz.opening_force(cd_s, RHO_STD, 20.0, OPENING_CX),
        'force_at_30_mps': sz.opening_force(cd_s, RHO_STD, 30.0, OPENING_CX),
        'fifty_newtons_in_g_at_325g': CONNECTION_FORCE/(0.325*G),
    }
    return out




def _fall_time(terminal_speed, height):
    low, high = 0.0, 1e3
    for _ in range(200):
        mid = (low + high)/2
        if sz.fall_distance(terminal_speed, mid) < height:
            low = mid
        else:
            high = mid
    return (low + high)/2


def timeline():
    return [{'release_m': h, 'speed': v, **sz.descent_timeline(h, v, FRAME_RATE, MAP_FLOOR)}
            for h in RELEASE_HEIGHTS for v in (10.0, 10.5, 11.0, 12.0, 15.0)]


# Loads in watts at the regulated rail: (low, typical, high).
LOADS = {
    # SOURCED: geerlingguy/sbc-reviews issue 33, measured at the wall on a 2 GB
    # board: 0.94 W minimal image, 1.12 W idle, 2.40 W stress-ng. High idle is
    # an unverified vendor-article figure kept as a pessimistic bound.
    'computer_idle': (0.94, 1.12, 2.2),
    'computer_busy': (2.4, 2.4, 4.0),
    # SOURCED: Arducam OV9281 USB board, 5 V, 200 mA maximum. Idle is ASSUMED.
    'camera_idle': (0.1, 0.3, 1.0),
    'camera_streaming': (0.5, 0.8, 1.0),
    # ASSUMED from typical u-blox M10 and SX1276 datasheet currents.
    'gps': (0.03, 0.10, 0.15),
    'radio_average': (0.01, 0.03, 0.05),
    'sensors': (0.01, 0.03, 0.08),
    'storage': (0.01, 0.05, 0.30),
}
PAD = ('computer_idle', 'camera_idle', 'gps', 'radio_average', 'sensors', 'storage')
FLIGHT = ('computer_busy', 'camera_streaming', 'gps', 'radio_average', 'sensors', 'storage')
CONVERTER_EFFICIENCY = (0.92, 0.88, 0.80)   # ASSUMED 1S to 5 V boost: best, typical, worst
# ASSUMED cell masses and capacities from typical datasheets.
CELLS = {
    'LiPo pouch 2000 mAh': (2000, 38),
    '18650 3000 mAh': (3000, 46),
    '18650 3500 mAh': (3500, 48),
    '21700 5000 mAh': (5000, 68),
    '2 x 18650 3000 mAh': (6000, 92),
}


def energy():
    def total(names, column):
        return sum(LOADS[n][column] for n in names)
    cases = {}
    for label, column in (('low', 0), ('typical', 1), ('high', 2)):
        pad, flight = total(PAD, column), total(FLIGHT, column)
        eta = CONVERTER_EFFICIENCY[column]
        cases[label] = {
            'pad_load_w': pad, 'flight_load_w': flight, 'converter_efficiency': eta,
            'battery_power_pad_w': pad/eta,
            'required_mah_for_3h': sz.required_capacity_mah(pad, POWERED_HOURS, eta),
            'peak_battery_current_a_at_3v3': flight/eta/3.3,
        }
    cells = {}
    for name, (mah, grams) in CELLS.items():
        usable = sz.battery_energy_wh(mah)
        cells[name] = {'mass_g': grams, 'usable_wh': usable,
                       **{'hours_' + label: sz.endurance_hours(usable, total(PAD, column), CONVERTER_EFFICIENCY[column])
                          for label, column in (('low', 0), ('typical', 1), ('high', 2))}}
    return {'loads_w': LOADS, 'cases': cases, 'cells': cells}


# Component masses in grams: (low, typical, high). SOURCED where noted, else ASSUMED.
MASSES = {
    'flight computer (SOURCED 12.5 g, CNX Software)': (12.5, 12.5, 12.5),
    'camera board, lens and holder': (15, 20, 25),
    'GPS module and antenna': (5, 8, 12),
    'radio module and antenna': (3, 5, 8),
    'IMU, barometer, thermistor and shield': (3, 6, 10),
    'carrier board, connectors and wiring': (15, 20, 30),
    'power conversion and switch': (3, 5, 8),
    'battery, one 18650': (46, 47, 48),
    'structure, printed shell and frame': (45, 60, 80),
    'window and fasteners': (7, 10, 17),
    'canopy, lines and swivel': (10, 15, 25),
}


def mass():
    totals = [sum(v[i] for v in MASSES.values()) for i in range(3)]
    return {'components_g': MASSES,
            'total_g': dict(zip(('low', 'typical', 'high'), totals)),
            'ballast_to_reach_300g': dict(zip(('low', 'typical', 'high'), (300 - t for t in totals))),
            'headroom_to_350g': dict(zip(('low', 'typical', 'high'), (350 - t for t in totals)))}


def radio():
    rows = []
    for sf in (7, 8, 9, 10):
        for bw in (125e3, 250e3):
            airtime = lora.time_on_air(PACKET_SIZE, sf, bw)
            rows.append({
                'sf': sf, 'bw_khz': bw/1e3, 'airtime_ms': airtime*1e3,
                'duty_at_1hz': lora.duty_cycle(airtime, 1.0),
                'min_interval_10pct_s': lora.min_interval(airtime, 0.10),
                'min_interval_1pct_s': lora.min_interval(airtime, 0.01),
                'sensitivity_dbm': lora.sensitivity_dbm(sf, bw),
                # ASSUMED: 0 dBi both ends and 20 dB for a tumbling can's
                # pattern nulls, polarisation mismatch and ground effects.
                'margin_db_1km_14dbm_868': lora.link_margin_db(1000, 868e6, 14, sf, bw, other_loss_db=20),
                'margin_db_2km_14dbm_868': lora.link_margin_db(2000, 868e6, 14, sf, bw, other_loss_db=20),
                'margin_db_2km_10dbm_433': lora.link_margin_db(2000, 433.9e6, 10, sf, bw, other_loss_db=20),
            })
    airtime = lora.time_on_air(PACKET_SIZE, 7, 125e3)
    flight_reserve = 60*airtime
    hourly_budget = 0.01*3600
    return {
        'packet_bytes': PACKET_SIZE, 'table': rows,
        'one_percent_band_schedule': {
            'hourly_airtime_budget_s': hourly_budget,
            'reserve_for_60s_at_1hz_s': flight_reserve,
            'pad_packets_per_hour': (hourly_budget - flight_reserve)/airtime,
            'pad_interval_s': 3600/((hourly_budget - flight_reserve)/airtime),
        },
    }


def exposure():
    rows = []
    for label, lux in (('heavy overcast', 2000), ('overcast', 10000), ('clear, sun at 35 deg', 60000)):
        for exposure_s in (0.5e-3, 1e-3):
            electrons = sz.photoelectrons(lux, 0.15, 2.4, exposure_s)
            rows.append({'sky': label, 'scene_lux': lux, 'exposure_ms': exposure_s*1e3,
                         'electrons': electrons, 'snr': sz.shot_noise_snr(electrons),
                         # A 20 % reflectance step between neighbouring fields.
                         'contrast_to_noise_20pct': 0.2*electrons/math.sqrt(electrons + 25)})
    return {'assumptions': 'reflectance 0.15, f/2.4, transmission 0.85, QE 0.5, 3 um pixel, '
                           '1e16 photons/m2/s/lux, read noise 5 e-', 'table': rows}


def camera_link():
    width, height = 1280, 800
    return {
        'raw_y8_mb_per_s': width*height*FRAME_RATE/1e6,
        'yuy2_over_usb_mb_per_s': width*height*2*FRAME_RATE/1e6,
        'usb2_high_speed_practical_mb_per_s': 35,   # ASSUMED practical bulk/isochronous ceiling
        'focal_length_mm_for_90deg_hfov': 1280*3e-3/2/math.tan(math.radians(45)),
        'focal_length_mm_for_70deg_hfov': 1280*3e-3/2/math.tan(math.radians(35)),
        'footprint_width_m_at_300m_90deg': 2*300*math.tan(math.radians(45)),
        'footprint_width_m_at_300m_70deg': 2*300*math.tan(math.radians(35)),
        'gsd_m_at_300m_70deg': 2*300*math.tan(math.radians(35))/width,
        'ram_buffer_mb_40s': width*height*FRAME_RATE*40/1e6,
    }


def main():
    results = {'canopy': canopy(), 'timeline': timeline(), 'energy': energy(), 'mass': mass(),
               'radio': radio(), 'exposure': exposure(), 'camera_link': camera_link()}
    out = HERE/'results'
    out.mkdir(exist_ok=True)
    (out/'sizing.json').write_text(json.dumps(results, indent=1))
    print(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
