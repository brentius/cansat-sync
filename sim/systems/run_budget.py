"""GPL-3.0-only. Evaluates budget.py and writes results/2026-10-11-budget/.

Outputs: budget.json (everything), CSV tables, and four PNG charts drawn with
sim/nav/plotting.py. Nothing here is measured; see the source and basis
columns. The older results/sizing.json is not touched.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent/'nav'))
import budget as b  # noqa: E402
import plotting  # noqa: E402
import sizing as sz  # noqa: E402

OUT = HERE/'results'/'2026-10-11-budget'

RULES = {
    'document': 'UK CanSat Competition Guidelines 2025-2026, section "CanSat Requirements"',
    'drive_file_id': '1LubKeC729i9NbmgHwG-wzu4gwhDO1PfJ',
    'read': '2026-10-11, through the Google Drive connector (text extraction of the PDF)',
    'edition_note': 'The 2026-27 guidelines are not in the Drive folder; these are last year\'s rules.',
    'quotes': {
        '1 dimensions': 'All the components of the CanSat must fit inside a standard soda can (115 mm height and '
                        '66 mm diameter), with the exception of the parachute. An exemption can be made for radio '
                        'antennas and GPS antennas, which can be mounted externally (on the top or bottom of the '
                        'can, not on the sides), based on the design',
        '3 mass': 'The mass of the CanSat must be between 300 g and 350 g. CanSats that are lighter must take '
                  'additional ballast with them to reach the 300 g mass limit required',
        '5 power': 'The CanSat must be powered by a battery and/or solar panels. It must be possible for the '
                   'systems to be switched on for three continuous hours',
        '6 battery access': 'The battery must be easily accessible in case it has to be replaced or recharged in the field.',
        '7 switch': 'The CanSat must have an easily accessible master power switch',
        '8 recovery': 'The CanSat should have a recovery system, such as a parachute, which is able to be reused '
                      'after launch. It is recommended to use bright coloured fabric, which will facilitate '
                      'recovery of the CanSat after landing.',
        '9 connection': 'The parachute connection must be able to withstand up to 50N of force. The strength of '
                        'the parachute must be tested, to give confidence that the system will operate nominally.',
        '10 descent rate': 'A descent rate of around 10-15 ms-1 is recommended. At a minimum the descent speed '
                           'should be 10 ms-1 but please be aware that the launch provider may request that the '
                           'parachute is replaced by a smaller one. The launch provider checks all parachutes '
                           'before launch and will insist on replacement of incorrectly sized parachutes (spare '
                           'parachutes are provided at launches).',
        '11 acceleration': 'The CanSat must be able to withstand an acceleration of up to 20g for rocket launch',
        '13 budget': 'The total budget of the CanSat should not exceed £400. This does not include ground support '
                     'equipment, such as laptops, power supplies, antennas. This includes the cost of the basic '
                     'boxed kit if provided to your school, college or community group, which costs £50.',
    },
}


def write_csv(name, header, rows):
    with open(OUT/name, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def r(x, n=3):
    return round(float(x), n)


def tables(mass, power, power_mc, peak, descent):
    write_csv('mass_items.csv',
              ['key', 'item', 'low_g', 'typical_g', 'high_g', 'mean_g', 'sigma_g', 'source', 'basis',
               'counts_as_parachute', 'url', 'note', 'inputs'],
              [[i['key'], i['name'], r(i['low'], 2), r(i['typical'], 2), r(i['high'], 2), r(i['mean'], 2),
                r(i['sigma'], 2), i['source'], i['basis'], i['parachute'], i['url'], i['note'],
                json.dumps(i['inputs']) if i['inputs'] else ''] for i in mass['items']])
    write_csv('mass_variance_shares.csv', ['key', 'item', 'sigma_g', 'share_of_variance', 'source'],
              [[v['key'], v['name'], r(v['sigma_g'], 2), r(v['share'], 4), v['source']]
               for v in mass['variance_shares']])
    write_csv('mass_savings.csv', ['option', 'low_g', 'typical_g', 'high_g', 'basis (all assumed)'],
              [[s['option'], s['low'], s['typical'], s['high'], s['basis']] for s in mass['savings_g']])
    write_csv('power_loads.csv',
              ['key', 'load', 'rail', 'low_w', 'typical_w', 'high_w', 'low_ma', 'typical_ma', 'high_ma',
               'source', 'basis', 'url', 'note'],
              [[i['key'], i['name'], i['inputs']['rail'], r(i['low'], 4), r(i['typical'], 4), r(i['high'], 4),
                *[r(v/(b.V_3V3 if i['inputs']['rail'] == '3V3' else b.V_5V1)*1000, 1)
                  for v in (i['low'], i['typical'], i['high'])],
                i['source'], i['basis'], i['url'], i['note']] for i in power['loads']])
    rows = []
    for name, cases in power['phases'].items():
        for case, v in cases.items():
            rows.append([name, b.PHASE_LABEL[name], case, r(v['rail_3v3_ma'], 1), r(v['rail_3v3_w']),
                         r(v['rail_5v1_ma'], 1), r(v['rail_5v1_w']), r(v['battery_w']),
                         r(v['battery_ma_at_7v2'], 1), r(v['endurance_h'], 2)])
    write_csv('power_phases.csv',
              ['phase', 'label', 'corner', 'rail_3v3_ma', 'rail_3v3_w', 'rail_5v1_ma', 'rail_5v1_w',
               'battery_w', 'battery_ma_at_7v2', 'endurance_h_continuous'], rows)
    rows = []
    for name, cases in power['scenarios'].items():
        for case, v in cases.items():
            rows.append([name, case, r(v['used_wh']), r(v['usable_wh']), r(v['margin_wh']),
                         r(v['margin_fraction']), r(v['nominal_wh_needed'], 2),
                         r(v['extra_beacon_h_after_scenario'], 1)])
        m = power_mc['scenarios'][name]['margin_wh']
        rows.append([name, 'monte carlo p05 / p50 / p95 margin_wh', r(m['p05']), r(m['p50']), r(m['p95']),
                     'P(margin<0)=%g' % m['p_below_0'], '', ''])
    write_csv('power_scenarios.csv',
              ['scenario', 'corner', 'used_wh', 'usable_wh', 'margin_wh', 'margin_fraction',
               'nominal_wh_needed', 'extra_beacon_h'], rows)
    write_csv('peak_currents.csv', ['check', 'current_a', 'limit_a', 'limit_over_current', 'limit_source'],
              [[c['check'], r(c['current_a']), c['limit_a'], r(c['ratio_limit_over_current'], 2),
                c['limit_source']] for c in peak['checks']])
    rows = []
    for key, c in descent['canopies'].items():
        for row in c['sizing']:
            rows.append([key, c['label'], c['reference'], *c['cd'], row['speed_mps'], row['mass'],
                         r(row['mass_g'], 1), r(row['cd_s_m2'], 5), r(row['area_m2_cd_high'], 4),
                         r(row['area_m2_cd_typical'], 4), r(row['area_m2_cd_low'], 4),
                         r(row['dimension_m_cd_high']), r(row['dimension_m_cd_typical']),
                         r(row['dimension_m_cd_low'])])
    write_csv('canopy_sizing.csv',
              ['canopy', 'label', 'reference_area', 'cd_low', 'cd_typical', 'cd_high', 'speed_mps', 'mass',
               'mass_g', 'cd_s_m2', 'area_m2_at_cd_high', 'area_m2_at_cd_typical', 'area_m2_at_cd_low',
               'dimension_m_at_cd_high', 'dimension_m_at_cd_typical', 'dimension_m_at_cd_low'], rows)
    rows = []
    for key, c in descent['canopies'].items():
        for label, s in (('cut from handbook Cd, never dropped', c['paper_design_speed_mps']),
                         ('trimmed to 10.5 m/s at standard density', c['trimmed_speed_mps'])):
            rows.append([key, label, r(c['paper_design_area_m2'], 4), r(c['paper_design_dimension_m']),
                         r(s['p05'], 2), r(s['p50'], 2), r(s['p95'], 2), r(s['p_below_10'], 4), r(s['p_above_15'], 4)])
    write_csv('descent_speed_spread.csv',
              ['canopy', 'basis', 'area_m2', 'dimension_m', 'speed_p05', 'speed_p50', 'speed_p95',
               'p_below_10', 'p_above_15'], rows)


def charts(mass, total, grouped_total, power, descent):
    made = []
    items = sorted(mass['items'], key=lambda i: -i['typical'])[:12]
    short = {'battery_pack': 'Battery', 'camera_body': 'Camera', 'shell': 'Shell', 'internal_frame': 'Frame',
             'end_caps': 'Caps', 'gnss_antenna': 'GNSS ant', 'lens': 'Lens', 'main_pcb_bare': 'MAIN PCB',
             'pi_zero_2w': 'Pi Zero', 'rf_pcb_bare': 'RF PCB', 'power_harness': 'Pwr harness',
             'lens_adapter': 'Adapter', 'fasteners': 'Fasteners', 'rf_ribbon': 'Ribbon',
             'main_pcb_parts': 'MAIN parts', 'sundries': 'Sundries', 'rf_pcb_parts': 'RF parts'}
    made.append(plotting.bar_chart(
        OUT/'mass_items.png', [short.get(i['key'], i['key']) for i in items],
        [{'label': 'low', 'values': [i['low'] for i in items]},
         {'label': 'typical', 'values': [i['typical'] for i in items]},
         {'label': 'high', 'values': [i['high'] for i in items]}],
        'Twelve heaviest items, current hardware', 'mass (g)',
        subtitle='Estimates only: vendor, datasheet, calculated or assumed. Nothing has been weighed.',
        size=(1300, 620), value_labels=False))
    grid = np.linspace(290, 400, 221)
    made.append(plotting.line_chart(
        OUT/'mass_total_distribution.png',
        [{'x': grid, 'y': [100*np.mean(total > g) for g in grid], 'label': 'items independent'},
         {'x': grid, 'y': [100*np.mean(grouped_total > g) for g in grid],
          'label': 'shell, caps and frame move together', 'style': 'dash'}],
        'Chance that the all-up mass exceeds a given value', 'mass (g)', 'probability of exceeding (%)',
        subtitle='Monte Carlo, triangular low/typical/high per item, parachute included; no item has been weighed',
        vlines=[(b.MASS_MIN_G, '300 g minimum', None), (b.MASS_MAX_G, '350 g limit', None)], ylim=(0, 100)))
    names = list(power['phases'])
    made.append(plotting.bar_chart(
        OUT/'endurance_by_phase.png', [b.PHASE_LABEL[n] for n in names],
        [{'label': 'low loads, best pack', 'values': [power['phases'][n]['low']['endurance_h'] for n in names]},
         {'label': 'typical', 'values': [power['phases'][n]['typical']['endurance_h'] for n in names]},
         {'label': 'high loads, cold worn pack', 'values': [power['phases'][n]['high']['endurance_h'] for n in names]}],
        'Hours the pack lasts if the can stayed in one phase', 'endurance (h)',
        subtitle='2S1P NCR18650GA pack, derated for cold; loads from datasheets, bench reports and assumptions, none measured',
        hlines=[(b.POWERED_HOURS, '3 h rule', None)], logy=True, size=(1300, 620)))
    speeds = np.linspace(9.0, 16.0, 71)
    m = descent['design_mass_g']['mean']/1000
    series = []
    for i, (key, c) in enumerate(descent['canopies'].items()):
        lo, typ, hi = c['cd']
        cds = np.array([sz.drag_area(m, v, b.RHO_STD) for v in speeds])
        series.append({'x': speeds, 'y': cds/typ*1e4, 'label': '%s, Cd %.2f (%.2f-%.2f)' % (c['label'], typ, lo, hi),
                       'band': (cds/hi*1e4, cds/lo*1e4), 'color': plotting.PALETTE[i]})
    made.append(plotting.line_chart(
        OUT/'canopy_area_vs_speed.png', series,
        'Canopy reference area needed at %.0f g, standard density' % (m*1000), 'steady descent speed (m/s)',
        'reference area (cm2)',
        subtitle='Bands span the drag-coefficient range; coefficients are handbook or calculated values, not drop-tested',
        vlines=[(b.SPEED_MIN, '10 m/s minimum', None), (b.DESIGN_SPEED, '10.5 design', None),
                (b.SPEED_RECOMMENDED[1], '15 recommended top', None)], legend='bottom'))
    return [str(Path(p).name) for p in made]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    mass, keys, total = b.mass_budget()
    grouped_total = b.sample_items(b.mass_items(), len(total), 20261011, correlate_groups=True).sum(axis=1)
    power = b.power_budget()
    power_1pct = b.power_budget(duty_limit=0.01)
    power_mc = b.power_monte_carlo()
    peak = b.peak_currents()
    descent = b.descent_budget(total)
    results = {
        'provenance': {
            'generated_by': 'sim/systems/run_budget.py', 'date': '2026-10-11',
            'status': 'ESTIMATE. No mass has been weighed and no current has been metered.',
            'tags': {'source': list(b.SOURCES),
                     'basis': ['primary read', 'search excerpt', 'recalled', 'project BOM', 'derived']},
            'urls': b.URL},
        'rules': RULES, 'mass': mass, 'power': power,
        'power_at_1pct_duty_limit': {'radio': power_1pct['radio'],
                                     'pad_pi_off': power_1pct['phases']['pad_pi_off'],
                                     'scenarios': power_1pct['scenarios']},
        'power_monte_carlo': power_mc, 'peak_currents': peak, 'descent': descent}
    tables(mass, power, power_mc, peak, descent)
    results['charts'] = charts(mass, total, grouped_total, power, descent)
    (OUT/'budget.json').write_text(json.dumps(results, indent=1), encoding='utf-8')

    s, mc = mass['straight_sum_g'], mass['monte_carlo_independent']
    print('mass straight sum low/typ/high: %.1f / %.1f / %.1f g' % (s['low'], s['typical'], s['high']))
    print('mass Monte Carlo: mean %.1f g, sigma %.1f g, P(>350 g) %.3f, P(<300 g) %.5f'
          % (mc['mean'], mc['sigma'], mc['p_above_350'], mc['p_below_300']))
    print('  structure correlated: P(>350 g) %.3f' % mass['monte_carlo_structure_correlated']['p_above_350'])
    for name, cases in power['phases'].items():
        print('%-12s battery W %.2f / %.2f / %.2f, endurance h %.1f / %.1f / %.1f' % (
            name, *[cases[c]['battery_w'] for c in b.CASES], *[cases[c]['endurance_h'] for c in b.CASES]))
    for name, cases in power['scenarios'].items():
        print('%-22s used Wh %.2f / %.2f / %.2f of usable %.1f / %.1f / %.1f' % (
            name, *[cases[c]['used_wh'] for c in b.CASES], *[cases[c]['usable_wh'] for c in b.CASES]))
    print('wrote', OUT)


if __name__ == '__main__':
    main()
