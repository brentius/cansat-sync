"""GPL-3.0-only. Mass, power/endurance and descent budgets for the current hardware.

Current hardware means wiki/flight_computer_bom.md and wiki/sensor_bom.md as of
8 October 2026: STM32F405 MAIN board, top RF board (MAX-M10S + E22-900M22S),
Raspberry Pi Zero 2 W, Raspberry Pi Global Shutter Camera with a 2.5 mm M12
lens, KeepPower 2S1P NCR18650GA pack, thermistor probe and buzzer. It replaces
the 1 October sizing in run_sizing.py, which is left untouched.

NumPy and the standard library only. Every line item carries low / typical /
high values and a source tag:

  measured    weighed or metered on the real article. NONE EXIST YET.
  datasheet   manufacturer document (URL given)
  vendor      seller or reseller listing (URL given)
  calculated  from a stated formula; the inputs are kept in Item.inputs
  assumed     engineering allowance awaiting a measurement

and a basis saying how the figure reached this file:

  primary read    the source page or PDF text was opened this session
  search excerpt  quoted by a web search result; the page itself was not opened
  recalled        remembered datasheet figure, not re-read this session
  project BOM     copied from wiki/flight_computer_bom.md, which cites a source
  derived         arithmetic on the inputs given

Nothing here is a measurement of the CanSat.
"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for _p in (str(HERE), str(HERE.parents[1]/'code')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import sizing as sz  # noqa: E402
from cansat import atmosphere, lora  # noqa: E402
from cansat.downlink import DownlinkScheduler  # noqa: E402
from cansat.telemetry import PACKET_SIZE  # noqa: E402

SOURCES = ('measured', 'datasheet', 'vendor', 'calculated', 'assumed')
CASES = ('low', 'typical', 'high')

# --- Rules: UK CanSat Competition Guidelines 2025-26, "CanSat Requirements" ---
# (Drive file 1LubKeC729i9NbmgHwG-wzu4gwhDO1PfJ, read 11 October 2026. The
# 2026-27 edition is not in the Drive folder.)
MASS_MIN_G, MASS_MAX_G = 300.0, 350.0        # requirement 3
CAN_DIAMETER_MM, CAN_HEIGHT_MM = 66.0, 115.0  # requirement 1
POWERED_HOURS = 3.0                           # requirement 5
SPEED_MIN = 10.0                              # requirement 10, "at a minimum"
SPEED_RECOMMENDED = (10.0, 15.0)              # requirement 10, "recommended"
CONNECTION_FORCE_N = 50.0                     # requirement 9
LAUNCH_G = 20.0                               # requirement 11
BUDGET_GBP = 400.0                            # requirement 13
DESIGN_SPEED = 10.5                           # wiki/trn_secondary_mission_proposal.md section 6

URL = {
    'keeppower': 'https://keeppower.com.cn/products_detail.php?id=863',
    'illumn': 'https://illumn.com/batteries/18650-keeppower-2s1p-battery-pack-ncr18650ga-3500mah-7-2v.html',
    'ncr18650ga': 'https://www.imrbatteries.com/content/sanyo_ncr18650ga.pdf',
    'gs_camera': 'https://shop.pimoroni.com/en-eu/products/raspberry-pi-global-shutter-camera',
    'pi_zero_mass': 'https://raspberrypi.dk/en/product/raspberry-pi-zero-2-w/?clang=en',
    'pi_zero_power_cnx': 'https://www.cnx-software.com/2021/12/09/raspberry-pi-zero-2-w-power-consumption/',
    'pi_zero_power_geerling': 'https://www.jeffgeerling.com/blog/2021/disabling-cores-reduce-pi-zero-2-ws-power-consumption-half/',
    'pi_zero_power_toms': 'https://www.tomshardware.com/uk/reviews/raspberry-pi-zero-2-w-review',
    'aggp25': 'https://www.taoglas.com/datasheets/AGGP.25F.07.0060A.pdf',
    'fxp895': 'https://www.taoglas.com/datasheets/FXP895.07.0200C.pdf',
    'buzzer': 'https://www.sameskydevices.com/product/resource/cmt-8504-100-smt-tr.pdf',
    'e22_mass': 'https://www.tindie.com/products/ebyteiot/e22-900m22s-sx1262-900mhz-new-lora-module/',
    'e22_manual': 'https://cdn.hackaday.io/files/1614916909230944/E22_Usermanual_EN_v1.10.pdf',
    'e22_jlc': 'https://jlcpcb.com/partdetail/Chengdu_Ebyte_ElecTech-E22900M22S/C411293',
    'max_m10s': 'https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf',
    'stm32f405': 'https://www.st.com/resource/en/datasheet/stm32f405rg.pdf',
    'bmi088': 'https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf',
    'ms5611': 'https://www.te.com/en/product-MS561101BA03-50.html',
    'w25q128': 'https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf',
    'tps62130': 'https://www.ti.com/lit/ds/symlink/tps62130.pdf',
    'tps62160': 'https://www.ti.com/lit/ds/symlink/tps62160.pdf',
    'tps22965': 'https://www.ti.com/lit/ds/symlink/tps22965.pdf',
    'lens': 'https://www.edmundoptics.com/p/25mm-fl-high-resolution-f2-micro-video-lens/3949/',
    'lens_comparable': 'https://www.watec.co.jp/English/eng_spec/M2520BC-12.eng.pdf',
    'camera_power_comparable': 'https://hackaday.io/project/184754/log/221313',
    'cross_annular_cfd': 'https://semarakilmu.com.my/journals/index.php/CFD_Letters/article/download/10061/5906/69390',
}


# --- Line items and their statistics -------------------------------------------

class Item:
    """One budget line: low <= typical <= high in `unit`, with provenance."""

    def __init__(self, key, name, low, typ, high, source, note, unit='g', basis='', url='',
                 inputs=None, group=None, parachute=False):
        if source not in SOURCES:
            raise ValueError('unknown source tag %r for %s' % (source, key))
        if not low <= typ <= high:
            raise ValueError('need low <= typical <= high for %s' % key)
        if source in ('datasheet', 'vendor') and not url:
            raise ValueError('%s is tagged %s and needs a URL' % (key, source))
        if source == 'calculated' and not inputs:
            raise ValueError('%s is tagged calculated and needs its inputs' % key)
        self.key, self.name = key, name
        self.low, self.typ, self.high = float(low), float(typ), float(high)
        self.source, self.note, self.unit = source, note, unit
        self.basis, self.url, self.inputs = basis, url, inputs or {}
        self.group, self.parachute = group, parachute

    @property
    def values(self):
        return self.low, self.typ, self.high

    @property
    def mean(self):
        return triangular_mean(*self.values)

    @property
    def variance(self):
        return triangular_variance(*self.values)

    def as_dict(self):
        return {'key': self.key, 'name': self.name, 'unit': self.unit, 'low': self.low,
                'typical': self.typ, 'high': self.high, 'mean': self.mean,
                'sigma': math.sqrt(self.variance), 'source': self.source, 'basis': self.basis,
                'url': self.url, 'inputs': self.inputs, 'group': self.group,
                'parachute': self.parachute, 'note': self.note}


def triangular_mean(low, mode, high):
    return (low + mode + high)/3


def triangular_variance(low, mode, high):
    return (low*low + mode*mode + high*high - low*mode - low*high - mode*high)/18


def triangular_ppf(u, low, mode, high):
    """Inverse CDF of the triangular distribution; u may be an array in [0, 1]."""
    u = np.asarray(u, float)
    span = high - low
    if span <= 0:
        return np.full(u.shape, float(low))
    split = (mode - low)/span
    with np.errstate(invalid='ignore'):
        left = low + np.sqrt(u*span*(mode - low))
        right = high - np.sqrt((1 - u)*span*(high - mode))
    return np.where(u < split, left, right)


def totals(items):
    """Straight sums of the low, typical and high columns."""
    return {case: sum(it.values[i] for it in items) for i, case in enumerate(CASES)}


def analytic_moments(items):
    """Mean and variance of the sum for INDEPENDENT triangular items."""
    return sum(it.mean for it in items), sum(it.variance for it in items)


def sample_items(items, n, seed=0, correlate_groups=False):
    """(n, len(items)) triangular draws.

    Independent by default. With correlate_groups, items sharing a group name
    use one uniform draw (fully correlated), which is how one design choice,
    e.g. wall thickness and filament, moves several structural parts together.
    """
    rng = np.random.default_rng(seed)
    u = rng.random((n, len(items)))
    if correlate_groups:
        first = {}
        for j, it in enumerate(items):
            if it.group is not None:
                u[:, j] = u[:, first.setdefault(it.group, j)]
    return np.column_stack([triangular_ppf(u[:, j], *it.values) for j, it in enumerate(items)])


def summarise(samples, low_limit=None, high_limit=None):
    out = {'n': int(len(samples)), 'mean': float(np.mean(samples)),
           'sigma': float(np.std(samples, ddof=1)),
           **{'p%02d' % q: float(np.percentile(samples, q)) for q in (1, 5, 25, 50, 75, 95, 99)}}
    if low_limit is not None:
        out['p_below_%g' % low_limit] = float(np.mean(samples < low_limit))
    if high_limit is not None:
        out['p_above_%g' % high_limit] = float(np.mean(samples > high_limit))
    return out


def variance_shares(items):
    """Each item's share of the independent-sum variance, largest first."""
    total = sum(it.variance for it in items)
    rows = [(it.key, it.name, math.sqrt(it.variance), it.variance/total if total else 0.0, it.source)
            for it in items]
    return sorted(rows, key=lambda r: -r[3])


# --- Mass: formulae -------------------------------------------------------------

FR4_DENSITY = (1.80, 1.85, 1.95)   # g/cm3, assumed range for glass-epoxy laminate
COPPER_DENSITY = 8.96              # g/cm3
# g/cm3, generic filament figures (assumed; check the reel's data sheet).
FILAMENT_DENSITY = {'ASA': 1.07, 'PLA': 1.24, 'PETG': 1.27}
WINDOW_DENSITY = {'acrylic': 1.19, 'polycarbonate': 1.20}


def pcb_mass_g(area_cm2, thickness_mm, copper_layers, copper_um=35.0, copper_fraction=0.6,
               laminate_density=1.85):
    """Bare board: laminate at full thickness plus the excess density of the copper.

    Solder mask, plating in holes and drilled-out material are neglected.
    """
    laminate = area_cm2*thickness_mm/10*laminate_density
    copper_cm3 = area_cm2*copper_layers*copper_um*1e-4*copper_fraction
    return laminate + copper_cm3*(COPPER_DENSITY - laminate_density)


def disc_area_cm2(diameter_mm):
    return math.pi*(diameter_mm/20)**2


def tube_mass_g(outer_diameter_mm, length_mm, wall_mm, density):
    """Solid-walled cylindrical shell (no end faces)."""
    ro = outer_diameter_mm/2
    return math.pi*(ro*ro - (ro - wall_mm)**2)*length_mm/1000*density


def disc_mass_g(diameter_mm, thickness_mm, density, hole_diameter_mm=0.0):
    return (disc_area_cm2(diameter_mm) - disc_area_cm2(hole_diameter_mm))*thickness_mm/10*density


def canopy_fabric_mass_g(area_m2, areal_density_g_m2, seam_factor):
    return area_m2*areal_density_g_m2*seam_factor


def lines_mass_g(count, length_m, linear_density_g_m):
    return count*length_m*linear_density_g_m


# Board outline read from pcb/cansat_control/cansat_control.kicad_pcb (Edge.Cuts
# rectangle 78..122 x 68..132 mm, 4 copper layers, 1.6 mm) with pcbnew on
# 11 October 2026, read-only. wiki/mainboard_placement.md says 44 x 65 mm; the
# high case uses that. pcb/cansat_radio/cansat_radio.kicad_pcb has no outline and no
# footprints, so the RF board diameter is an assumption.
MAIN_PCB_MM = (44.0, 64.0)
MAIN_PCB_NOTE_MM = (44.0, 65.0)
RF_PCB_DIAMETER_MM = (56.0, 60.0, 62.0)


def mass_items():
    """The itemised mass bill, in grams. Nothing in it has been weighed."""
    a_main = MAIN_PCB_MM[0]*MAIN_PCB_MM[1]/100
    a_main_hi = MAIN_PCB_NOTE_MM[0]*MAIN_PCB_NOTE_MM[1]/100
    main = (pcb_mass_g(a_main, 1.6, 4, 35, 0.4, FR4_DENSITY[0]),
            pcb_mass_g(a_main, 1.6, 4, 35, 0.6, FR4_DENSITY[1]),
            pcb_mass_g(a_main_hi, 1.6, 4, 35, 0.8, FR4_DENSITY[2]))
    rf = (pcb_mass_g(disc_area_cm2(RF_PCB_DIAMETER_MM[0]), 1.0, 2, 35, 0.5, FR4_DENSITY[0]),
          pcb_mass_g(disc_area_cm2(RF_PCB_DIAMETER_MM[1]), 1.6, 2, 35, 0.7, FR4_DENSITY[1]),
          pcb_mass_g(disc_area_cm2(RF_PCB_DIAMETER_MM[2]), 1.6, 2, 35, 0.85, FR4_DENSITY[2]))
    shell = (tube_mass_g(64.0, 100.0, 1.0, FILAMENT_DENSITY['ASA']),
             tube_mass_g(65.0, 105.0, 1.2, FILAMENT_DENSITY['PETG']),
             tube_mass_g(65.5, 110.0, 1.6, FILAMENT_DENSITY['PETG']))
    caps = (2*disc_mass_g(64.0, 1.5, FILAMENT_DENSITY['ASA']) - disc_mass_g(30.0, 1.5, FILAMENT_DENSITY['ASA']),
            2*disc_mass_g(65.0, 2.0, FILAMENT_DENSITY['PETG']) - disc_mass_g(30.0, 2.0, FILAMENT_DENSITY['PETG']),
            2*disc_mass_g(65.5, 3.0, FILAMENT_DENSITY['PETG']) - disc_mass_g(30.0, 3.0, FILAMENT_DENSITY['PETG']))
    window = (disc_mass_g(30.0, 1.5, WINDOW_DENSITY['acrylic']),
              disc_mass_g(32.0, 2.0, WINDOW_DENSITY['acrylic']),
              disc_mass_g(40.0, 3.0, WINDOW_DENSITY['polycarbonate']))
    fabric = (canopy_fabric_mass_g(0.055, 38.0, 1.2), canopy_fabric_mass_g(0.070, 45.0, 1.3),
              canopy_fabric_mass_g(0.110, 60.0, 1.4))
    lines = (lines_mass_g(6, 0.35, 0.5), lines_mass_g(8, 0.45, 0.8), lines_mass_g(8, 0.60, 1.5))
    I = Item
    return [
        I('main_pcb_bare', 'MAIN PCB, bare board', *main, 'calculated',
          'area x thickness x FR4 density + copper excess; holes and mask neglected', basis='derived',
          inputs={'outline_mm': [MAIN_PCB_MM, MAIN_PCB_MM, MAIN_PCB_NOTE_MM], 'thickness_mm': 1.6,
                  'copper_layers': 4, 'copper_um': 35, 'copper_fraction': [0.4, 0.6, 0.8],
                  'fr4_g_cm3': FR4_DENSITY, 'outline_source': 'pcbnew read of Edge.Cuts; high uses the placement note'}),
        I('main_pcb_parts', 'MAIN PCB, fitted parts and solder', 4.5, 5.9, 8.0, 'assumed',
          '124 footprints: LQFP-64 ~0.35 g, two 4x4 mm inductors ~0.5 g, shrouded 2x10 header ~2.5 g, '
          'three JST GH ~0.9 g, about 90 chip passives ~0.5 g, other ICs and solder ~1.2 g; buzzer listed separately'),
        I('buzzer', 'Buzzer CMT-8504-100-SMT', 0.55, 0.6, 0.65, 'datasheet',
          'datasheet weight 0.6 g', basis='primary read', url=URL['buzzer']),
        I('rf_pcb_bare', 'RF PCB, bare board (circular)', *rf, 'calculated',
          'no outline exists in the KiCad file; diameter and thickness are assumptions', basis='derived',
          inputs={'diameter_mm': RF_PCB_DIAMETER_MM, 'thickness_mm': [1.0, 1.6, 1.6], 'copper_layers': 2,
                  'copper_um': 35, 'copper_fraction': [0.5, 0.7, 0.85], 'fr4_g_cm3': FR4_DENSITY}),
        I('rf_pcb_parts', 'RF PCB, fitted parts and solder', 3.6, 5.0, 6.5, 'assumed',
          'E22-900M22S 1.5 +/- 0.1 g (Ebyte storefront, search excerpt), MAX-M10S ~0.6 g, 2x10 header ~2.5 g, U.FL and passives ~0.4 g'),
        I('rf_ribbon', 'MAIN-RF ribbon: two 20-way IDC sockets and cable', 4.5, 7.0, 10.0, 'assumed',
          'two 2.54 mm IDC sockets ~2.2 g each plus about 60 mm of 20-way ribbon at ~45 g/m'),
        I('pi_zero_2w', 'Raspberry Pi Zero 2 W, unheadered', 9.0, 10.0, 11.0, 'vendor',
          'resellers list 9-10 g; the BOM text says 10-15 g', basis='search excerpt', url=URL['pi_zero_mass']),
        I('microsd', 'microSD card', 0.25, 0.3, 0.5, 'assumed', 'typical card mass'),
        I('camera_body', 'Global Shutter Camera body', 30.0, 34.0, 41.0, 'vendor',
          '34 g bare, 41 g with C-CS adapter and dust cap (reseller listings); low assumes the tripod '
          'mount is included in the 34 g and can be removed, which is unverified',
          basis='search excerpt', url=URL['gs_camera']),
        I('lens', 'Lens, Edmund #34-711 2.5 mm f/2 M12', 6.5, 12.0, 18.0, 'assumed',
          'no mass found for this lens (page returned HTTP 403); other 2.5 mm M12 lenses are 6.4 g '
          '(Watec) and 11.5 g (Videology); this one is 18 mm long and 20 mm across'),
        I('lens_adapter', 'C-to-M12 adapter #53-675 and lock nut', 4.0, 8.0, 13.0, 'assumed',
          'no published mass found; aluminium ring of C-mount diameter'),
        I('csi_cable', 'CSI cable, 150 mm Zero type', 1.0, 1.5, 2.5, 'assumed', 'flat flex cable'),
        I('battery_pack', 'KeepPower 2S1P NCR18650GA pack', 97.0, 100.0, 106.0, 'vendor',
          'maker says "approx. 100 g"; two cells at up to 48.0-49.5 g each (search excerpt) plus '
          'protection board, tabs, wrap and leads', basis='primary read', url=URL['keeppower']),
        I('power_harness', 'XT30 pair, master toggle switch, power wire', 6.0, 9.0, 13.0, 'assumed',
          'XT30 pair ~2 g, miniature toggle ~4-5 g, 20 AWG leads'),
        I('signal_harness', 'Pi, trigger and probe harnesses (JST GH)', 1.5, 2.5, 4.0, 'assumed',
          'eight plus two plus two 28 AWG wires, housings and crimps'),
        I('gnss_antenna', 'GNSS antenna Taoglas AGGP.25F.07.0060A', 12.0, 13.0, 14.0, 'datasheet',
          'datasheet weight 13 g with its 60 mm cable', basis='primary read', url=URL['aggp25']),
        I('lora_antenna', 'LoRa antenna Taoglas FXP895 with 200 mm coax', 1.5, 2.0, 3.0, 'datasheet',
          '2 g as quoted in the project BOM from the datasheet', basis='project BOM', url=URL['fxp895']),
        I('probe_shield', 'Thermistor probe and printed radiation shield', 1.0, 3.0, 6.0, 'assumed',
          'probe under 0.5 g; shield geometry not designed'),
        I('fasteners', 'Screws, standoffs and inserts', 4.0, 8.0, 15.0, 'assumed',
          'about twelve M2 screws and four to eight standoffs; nylon at the low end, brass and steel at the high'),
        I('shell', 'Printed shell, cylindrical wall', *shell, 'calculated',
          'pi (ro^2 - ri^2) L rho, printed solid; no CAD model exists', basis='derived', group='structure',
          inputs={'outer_diameter_mm': [64.0, 65.0, 65.5], 'length_mm': [100, 105, 110],
                  'wall_mm': [1.0, 1.2, 1.6], 'material': ['ASA', 'PETG', 'PETG'],
                  'density_g_cm3': [1.07, 1.27, 1.27]}),
        I('end_caps', 'Printed top and bottom caps', *caps, 'calculated',
          'two discs less a 30 mm window opening', basis='derived', group='structure',
          inputs={'diameter_mm': [64.0, 65.0, 65.5], 'thickness_mm': [1.5, 2.0, 3.0],
                  'window_hole_mm': 30, 'density_g_cm3': [1.07, 1.27, 1.27]}),
        I('internal_frame', 'Internal frame: battery cradle, Pi rail, camera mount', 10.0, 18.0, 30.0,
          'assumed', 'not designed; must carry a 100 g pack and a 50 g camera stack at 20 g', group='structure'),
        I('window', 'Bottom window', *window, 'calculated', 'disc area x thickness x density', basis='derived',
          inputs={'diameter_mm': [30, 32, 40], 'thickness_mm': [1.5, 2.0, 3.0],
                  'material': ['acrylic', 'acrylic', 'polycarbonate']}),
        I('canopy_fabric', 'Parachute canopy fabric', *fabric, 'calculated',
          'area x areal density x seam factor', basis='derived', parachute=True,
          inputs={'area_m2': [0.055, 0.070, 0.110], 'ripstop_g_m2': [38, 45, 60], 'seam_factor': [1.2, 1.3, 1.4]}),
        I('canopy_lines', 'Parachute lines', *lines, 'calculated', 'count x length x linear density',
          basis='derived', parachute=True,
          inputs={'count': [6, 8, 8], 'length_m': [0.35, 0.45, 0.60], 'g_per_m': [0.5, 0.8, 1.5]}),
        I('swivel_attachment', 'Swivel, ring and attachment hardware', 2.0, 4.0, 8.0, 'assumed',
          'must hold 50 N', parachute=True),
        I('sundries', 'Adhesive, tape, foam, cable ties, heat-shrink', 2.0, 5.0, 10.0, 'assumed',
          'always more than expected'),
    ]


# Mass-saving options: (name, low, typical, high saving in g, basis). All ASSUMED.
MASS_SAVINGS = [
    ('Board-level IMX296 M12 camera in place of the metal-bodied camera, adapter and lock nut', 20.0, 28.0, 35.0,
     'removes the 34 g housing and the adapter; needs a module with a documented trigger input'),
    ('2S pack of 18500-size cells (about 2 Ah) in place of the 2S 18650 pack', 25.0, 30.0, 35.0,
     'about 70 g and 14.7 Wh nominal; covers 3 h with the Pi idle, not 3 h with it working; cell choice unverified'),
    ('Passive ceramic patch on the RF board in place of the 13 g active patch', 6.0, 8.0, 10.0,
     'MAX-M10S has an internal LNA; needs ground-plane area and an RF check'),
    ('Mezzanine or flat-flex link in place of shrouded headers, IDC sockets and ribbon', 6.0, 8.0, 11.0,
     'about 2.5 g a header plus the ribbon set'),
    ('1.0 mm boards instead of 1.6 mm', 5.0, 6.0, 7.0, '37.5 % of the laminate on both boards'),
    ('Shell wall 1.0 mm in ASA instead of 1.2 mm PETG', 8.0, 11.0, 14.0, 'stiffness and 20 g check needed'),
    ('Lighter M12 lens (plastic barrel, 6-7 g class)', 3.0, 5.0, 9.0, 'optical performance must be re-qualified'),
    ('Slide switch instead of toggle; nylon fasteners', 3.0, 5.0, 8.0, 'check switch current rating'),
]


def mass_budget(n=200000, seed=20261011):
    items = mass_items()
    keys = [it.key for it in items]
    body = [it for it in items if not it.parachute]
    mean, var = analytic_moments(items)
    draws = sample_items(items, n, seed)
    grouped = sample_items(items, n, seed, correlate_groups=True)
    not_chute = np.array([not it.parachute for it in items])
    total = draws.sum(axis=1)
    out = {
        'items': [it.as_dict() for it in items],
        'straight_sum_g': totals(items),
        'straight_sum_without_parachute_g': totals(body),
        'analytic_independent': {'mean_g': mean, 'sigma_g': math.sqrt(var)},
        'monte_carlo_independent': summarise(total, MASS_MIN_G, MASS_MAX_G),
        'monte_carlo_structure_correlated': summarise(grouped.sum(axis=1), MASS_MIN_G, MASS_MAX_G),
        'monte_carlo_without_parachute': summarise(draws[:, not_chute].sum(axis=1), MASS_MIN_G, MASS_MAX_G),
        'variance_shares': [{'key': k, 'name': nm, 'sigma_g': s, 'share': f, 'source': src}
                            for k, nm, s, f, src in variance_shares(items)],
        'by_source_typical_g': {src: sum(it.typ for it in items if it.source == src) for src in SOURCES},
        'distribution': 'triangular(low, typical, high) per item, independent unless stated',
        'savings_g': [{'option': o, 'low': a, 'typical': b, 'high': c, 'basis': why}
                      for o, a, b, c, why in MASS_SAVINGS],
    }
    return out, keys, total


# --- Power: loads ---------------------------------------------------------------

V_3V3, V_5V1 = 3.3, 5.1
RADIO_DUTY_LIMIT = 0.10      # 869.4-869.65 MHz alternative named in the BOM; 0.01 elsewhere
BEACON_BUZZER_DUTY = 0.10    # ASSUMED: 0.1 s chirp each second after landing
BUZZER_COIL_OHM = (18.0, 15.0, 12.0)   # datasheet coil resistance; low current first


def packet_airtime_s():
    """Airtime of one telemetry packet at the code's default SF7 / 125 kHz."""
    return lora.time_on_air(PACKET_SIZE)


def radio_duty(flying, duty_limit=RADIO_DUTY_LIMIT):
    """Transmit duty cycle that code/cansat/downlink.py would produce."""
    airtime = packet_airtime_s()
    sched = DownlinkScheduler(airtime, duty_limit)
    return lora.duty_cycle(airtime, sched.flight_interval if flying else sched.ground_interval)


def _ma(key, name, low, typ, high, source, note, **kw):
    """A 3.3 V rail load given in mA, stored in watts."""
    return Item(key, name, low*V_3V3/1000, typ*V_3V3/1000, high*V_3V3/1000, source, note, unit='W',
                inputs={'rail': '3V3', 'mA': [low, typ, high], **kw.pop('inputs', {})}, **kw)


def _w(key, name, low, typ, high, source, note, **kw):
    return Item(key, name, low, typ, high, source, note, unit='W',
                inputs={'rail': '5V1', **kw.pop('inputs', {})}, **kw)


def load_items():
    """Loads at the regulated rails, in watts while active. None has been metered."""
    buzz = tuple(0.5*V_3V3/r*1000 for r in BUZZER_COIL_OHM)   # 50 % square wave into the coil
    rows = [
        _ma('mcu', 'STM32F405 running at 168 MHz', 40, 60, 93, 'datasheet',
            'typical 40 mA with peripherals off (ST quotes 238 uA/MHz) to 87-93 mA with all on',
            basis='recalled', url=URL['stm32f405']),
        _ma('imu', 'BMI088 accelerometer and gyro', 5.0, 5.2, 5.5, 'datasheet',
            'gyro 5 mA, accelerometer 0.15 mA', basis='recalled', url=URL['bmi088']),
        _ma('baro', 'MS5611 at about 50 Hz, OSR 4096', 0.2, 0.9, 1.4, 'datasheet',
            '1.4 mA while converting; duty set by the sample rate', basis='recalled', url=URL['ms5611']),
        _ma('temp_chain', 'ADS1115, OPA333, 3.0 V LDO, divider', 0.2, 0.3, 0.5, 'assumed',
            'sum of datasheet quiescent currents, not re-read'),
        _ma('flash_write', 'W25Q128 while programming', 15, 20, 25, 'datasheet',
            'page-program current; standby is microamps', basis='recalled', url=URL['w25q128']),
        _ma('leds_misc', 'LEDs, pull-ups, UART buffers', 0.5, 2.0, 9.0, 'calculated',
            'two LEDs at (3.3 - 1.9) V / 330 ohm = 4.2 mA each; duty 0 to 100 %', basis='derived',
            inputs={'led_mA_each': 4.2}),
        _ma('gnss_track', 'MAX-M10S tracking', 6, 8, 10, 'datasheet',
            'data sheet overview quotes 25 mW continuous tracking (7.6 mA at 3.3 V)',
            basis='search excerpt', url=URL['max_m10s']),
        _ma('gnss_acq', 'MAX-M10S acquisition', 8, 12, 25, 'datasheet',
            'reseller summary of the data sheet gives 6-25 mA over tracking state and constellations',
            basis='search excerpt', url=URL['max_m10s']),
        _ma('gnss_antenna', 'Active GNSS antenna LNA', 8, 10, 13, 'datasheet',
            '5 mA at 1.8 V, 10 mA at 3.0 V, 23 mA at 5.5 V; fed from VCC_RF through 10 ohm',
            basis='primary read', url=URL['aggp25']),
        _ma('radio_idle', 'E22 between packets', 0.6, 2.0, 7.0, 'datasheet',
            'standby to continuous receive (module receive 6.8 mA per JLC listing)',
            basis='search excerpt', url=URL['e22_jlc']),
        _ma('radio_tx', 'E22 transmitting', 45, 60, 119, 'datasheet',
            '118-119 mA at +22 dBm (Ebyte manual, JLC listing); SX1262 about 45 mA at +14 dBm (recalled). '
            'No transmit power is configured in code/; the BOM says start at +14 dBm',
            basis='search excerpt', url=URL['e22_manual']),
        _ma('buzzer_on', 'Buzzer while sounding (50 % drive)', *buzz, 'calculated',
            '0.5 x 3.3 V / coil resistance; coil inductance at 4 kHz neglected, so this is high',
            basis='derived', url=URL['buzzer'], inputs={'coil_ohm': BUZZER_COIL_OHM}),
        _w('pi_idle', 'Pi Zero 2 W idle, radios off', 0.50, 0.65, 1.20, 'vendor',
           'published bench figures: 100 mA (Geerling), 120 mA (CNX), 280 mA (Tom\'s Hardware) at 5 V',
           basis='search excerpt', url=URL['pi_zero_power_cnx']),
        _w('pi_busy', 'Pi Zero 2 W, four cores loaded', 2.2, 2.8, 3.3, 'vendor',
           'published bench figures: 500 mA (Geerling) to 580-600 mA (Tom\'s Hardware) at 5 V',
           basis='search excerpt', url=URL['pi_zero_power_geerling']),
        _w('cam_idle', 'Camera powered, not streaming', 0.05, 0.2, 0.4, 'assumed',
           'no published figure found for the IMX296 module'),
        _w('cam_stream', 'Camera streaming at 5 Hz', 0.4, 0.8, 1.2, 'assumed',
           'no published figure found; a different Pi camera added about 0.65 W on a Zero W (search excerpt)'),
        _w('sd_write', 'microSD writing frames', 0.05, 0.15, 0.40, 'assumed', 'about 10 MB/s of raw frames'),
    ]
    return {it.key: it for it in rows}


# Phase -> {load: fraction of time active}. Radio duty is filled in by phases().
_PHASE_BASE = {'mcu': 1, 'imu': 1, 'baro': 1, 'temp_chain': 1, 'leds_misc': 1, 'gnss_antenna': 1}


def phases(duty_limit=RADIO_DUTY_LIMIT):
    ground, flight = radio_duty(False, duty_limit), radio_duty(True, duty_limit)

    def radio(d):
        return {'radio_tx': d, 'radio_idle': 1 - d}
    pad = {**_PHASE_BASE, 'gnss_track': 1, 'flash_write': 0.05, **radio(ground)}
    return {
        'pad_pi_off': pad,
        'pad_pi_idle': {**pad, 'pi_idle': 1, 'cam_idle': 1},
        'pad_pi_busy': {**pad, 'pi_busy': 1, 'cam_stream': 1, 'sd_write': 1},
        'ascent': {**_PHASE_BASE, 'gnss_acq': 1, 'flash_write': 0.3, **radio(flight),
                   'pi_idle': 1, 'cam_stream': 1},
        'descent': {**_PHASE_BASE, 'gnss_acq': 1, 'flash_write': 0.3, **radio(flight),
                    'pi_busy': 1, 'cam_stream': 1, 'sd_write': 1},
        'beacon': {**_PHASE_BASE, 'gnss_track': 1, 'flash_write': 0.02, **radio(ground),
                   'buzzer_on': BEACON_BUZZER_DUTY},
    }


PHASE_LABEL = {
    'pad_pi_off': 'Pad, Pi off', 'pad_pi_idle': 'Pad, Pi idle', 'pad_pi_busy': 'Pad, Pi and camera working',
    'ascent': 'Ascent', 'descent': 'Descent, 5 Hz matching', 'beacon': 'Post-landing beacon'}

# Converter efficiency from the 2S input, (best, typical, worst). DATASHEET
# curves, RECALLED: the curves are images and were not re-read this session.
ETA_5V1 = (0.95, 0.92, 0.88)   # TPS62130 to 5.1 V, 0.1-1 A
ETA_3V3 = (0.90, 0.87, 0.82)   # TPS62162 to 3.3 V, about 0.1 A
# ASSUMED series resistances, ohm (best, typical, worst).
R_PI_PATH = (0.03, 0.05, 0.10)   # TPS22965 16 mohm typical + GH contacts + harness
R_INPUT = (0.06, 0.10, 0.16)     # switch, XT30, 5 A fuse, AO3401A (<= 60 mohm), wiring
V_PACK_NOMINAL = 7.2
V_PACK_LOW = 6.0                 # 3.0 V per cell: where the load current is highest


def rail_power(phase, loads, column):
    """(P at 3.3 V rail, P at 5.1 V rail) in watts; column 0/1/2 or sampled dict key -> array."""
    p33 = p51 = 0.0
    for key, duty in phase.items():
        it = loads[key]
        value = it.values[column] if isinstance(column, int) else column[key]
        if it.inputs['rail'] == '3V3':
            p33 = p33 + duty*value
        else:
            p51 = p51 + duty*value
    return p33, p51


def battery_power(p33, p51, eta33, eta51, r_pi=0.0, r_in=0.0, v_pack=V_PACK_NOMINAL):
    """Power drawn from the pack terminals for given rail loads (scalars or arrays)."""
    p51 = p51 + (p51/V_5V1)**2*r_pi
    p = p33/eta33 + p51/eta51
    # Input-path loss: solve p_total = p + (p_total/v)^2 r for the smaller root.
    r = np.asarray(r_in, float)
    disc = np.maximum(1 - 4*r*p/v_pack**2, 0.0)
    return np.where(r > 0, v_pack**2*(1 - np.sqrt(disc))/(2*np.where(r > 0, r, 1.0)), p)


def phase_battery_power(name, column, duty_limit=RADIO_DUTY_LIMIT):
    """Pack power in a phase for the low (0), typical (1) or high (2) corner."""
    p33, p51 = rail_power(phases(duty_limit)[name], load_items(), column)
    return battery_power(p33, p51, ETA_3V3[column], ETA_5V1[column], R_PI_PATH[column], R_INPUT[column])


# --- Power: battery ------------------------------------------------------------

# KeepPower 2S1P NCR18650GA: nominal 3500 mAh, minimum 3350 mAh, 7.2 V, 5.2 A
# maximum discharge, over-current protection 10-20 A, discharge -20 to +60 degC
# (maker's page, primary read). The BOM quotes 3.3 Ah and 5 A from the reseller.
PACK = {
    'capacity_ah': (3.45, 3.35, 3.30),     # best, typical, worst
    'nominal_v': V_PACK_NOMINAL,
    'max_discharge_a': 5.2,
    'protection_trip_a': (10.0, 20.0),
    'fuse_a': 5.0,
    # ASSUMED derating factors (best, typical, worst). The cell data sheet has
    # discharge curves at 25, 10, 0 and -10 degC at 4 A, but they are graphs
    # that could not be read this session; these are generic Li-ion figures
    # for a 0.1-0.2 C load at 0 to 10 degC, with the worst case near -10 degC.
    'cold': (0.95, 0.88, 0.75),
    'charge_and_age': (1.00, 0.97, 0.90),  # not full at switch-on, self-discharge, cycle wear
    'cutoff_reserve': (0.98, 0.95, 0.92),  # stop before the protection board trips
}


def usable_fraction(column):
    return PACK['cold'][column]*PACK['charge_and_age'][column]*PACK['cutoff_reserve'][column]


def usable_energy_wh(column):
    return sz.battery_energy_wh(PACK['capacity_ah'][column]*1000, PACK['nominal_v'], usable_fraction(column))


def nominal_energy_wh(column=1):
    return PACK['capacity_ah'][column]*PACK['nominal_v']


ASCENT_S, DESCENT_S, BEACON_H, PI_LEAD_H = 10.0, 40.0, 1.0, 10/60
SCENARIOS = {
    'pi_on_throughout': [('pad_pi_idle', POWERED_HOURS), ('ascent', ASCENT_S/3600),
                         ('descent', DESCENT_S/3600), ('beacon', BEACON_H)],
    'pi_on_late': [('pad_pi_off', POWERED_HOURS - PI_LEAD_H), ('pad_pi_idle', PI_LEAD_H),
                   ('ascent', ASCENT_S/3600), ('descent', DESCENT_S/3600), ('beacon', BEACON_H)],
    'pi_working_throughout': [('pad_pi_busy', POWERED_HOURS), ('ascent', ASCENT_S/3600),
                              ('descent', DESCENT_S/3600), ('beacon', BEACON_H)],
}


def scenario_energy_wh(name, column, duty_limit=RADIO_DUTY_LIMIT):
    return sum(float(phase_battery_power(ph, column, duty_limit))*hours for ph, hours in SCENARIOS[name])


def power_budget(duty_limit=RADIO_DUTY_LIMIT):
    """Corner-case loads, endurance and scenario margins.

    'low' pairs the low loads with the best efficiency and best battery;
    'high' pairs the high loads with the worst of each. They are stacked
    corners, not probabilities; see power_monte_carlo for those.
    """
    loads, ph = load_items(), phases(duty_limit)
    out = {'loads': [it.as_dict() for it in loads.values()],
           'radio': {'packet_bytes': PACKET_SIZE, 'airtime_s': packet_airtime_s(), 'duty_limit': duty_limit,
                     'duty_flight': radio_duty(True, duty_limit), 'duty_ground': radio_duty(False, duty_limit),
                     'duty_ground_at_1pct_limit': radio_duty(False, 0.01)},
           'converter_efficiency': {'tps62130_5v1': ETA_5V1, 'tps62162_3v3': ETA_3V3},
           'pack': {**PACK, 'nominal_wh': [nominal_energy_wh(c) for c in range(3)],
                    'usable_fraction': [usable_fraction(c) for c in range(3)],
                    'usable_wh': [usable_energy_wh(c) for c in range(3)]},
           'phases': {}, 'scenarios': {}}
    for name in ph:
        rows = {}
        for c, case in enumerate(CASES):
            p33, p51 = rail_power(ph[name], loads, c)
            pb = float(phase_battery_power(name, c, duty_limit))
            rows[case] = {'rail_3v3_w': p33, 'rail_3v3_ma': p33/V_3V3*1000, 'rail_5v1_w': p51,
                          'rail_5v1_ma': p51/V_5V1*1000, 'battery_w': pb,
                          'battery_ma_at_7v2': pb/V_PACK_NOMINAL*1000,
                          'endurance_h': usable_energy_wh(c)/pb}
        out['phases'][name] = rows
    for name in SCENARIOS:
        rows = {}
        for c, case in enumerate(CASES):
            used, usable = scenario_energy_wh(name, c, duty_limit), usable_energy_wh(c)
            beacon = float(phase_battery_power('beacon', c, duty_limit))
            rows[case] = {'used_wh': used, 'usable_wh': usable, 'margin_wh': usable - used,
                          'margin_fraction': (usable - used)/usable,
                          'nominal_wh_needed': used/usable_fraction(c),
                          'extra_beacon_h_after_scenario': (usable - used)/beacon}
        out['scenarios'][name] = rows
    return out


def power_monte_carlo(n=100000, seed=20261012, duty_limit=RADIO_DUTY_LIMIT):
    """Triangular draws of every load, efficiency and derating factor, independent."""
    rng = np.random.default_rng(seed)
    loads, ph = load_items(), phases(duty_limit)

    def tri(best_typ_worst):
        lo, hi = min(best_typ_worst), max(best_typ_worst)
        return triangular_ppf(rng.random(n), lo, best_typ_worst[1], hi)
    draw = {k: triangular_ppf(rng.random(n), *it.values) for k, it in loads.items()}
    eta33, eta51, r_pi, r_in = tri(ETA_3V3), tri(ETA_5V1), tri(R_PI_PATH), tri(R_INPUT)
    usable = tri(PACK['capacity_ah'])*PACK['nominal_v']*tri(PACK['cold'])*tri(PACK['charge_and_age']) \
        * tri(PACK['cutoff_reserve'])
    power = {name: battery_power(*rail_power(ph[name], loads, draw), eta33, eta51, r_pi, r_in) for name in ph}
    out = {'n': n, 'usable_wh': summarise(usable), 'phases': {}, 'scenarios': {}}
    for name, p in power.items():
        out['phases'][name] = {'battery_w': summarise(p),
                               'endurance_h': summarise(usable/p, low_limit=POWERED_HOURS)}
    for name, steps in SCENARIOS.items():
        used = sum(power[phname]*hours for phname, hours in steps)
        out['scenarios'][name] = {'used_wh': summarise(used),
                                  'margin_wh': summarise(usable - used, low_limit=0.0)}
    return out


def peak_currents():
    """Worst simultaneous draw against each limit. High column, everything on at once."""
    loads = load_items()
    on33 = ('mcu', 'imu', 'baro', 'temp_chain', 'flash_write', 'leds_misc', 'gnss_acq', 'gnss_antenna', 'radio_tx')
    i33 = sum(loads[k].high for k in on33)/V_3V3 + V_3V3/min(BUZZER_COIL_OHM)   # buzzer at its peak, not mean
    p51 = sum(loads[k].high for k in ('pi_busy', 'cam_stream', 'sd_write'))
    i51 = p51/V_5V1
    i51_transient = 1.5*i51   # ASSUMED allowance for CPU and SD bursts above the bench averages
    pb = float(battery_power(i33*V_3V3, i51_transient*V_5V1, ETA_3V3[2], ETA_5V1[2], R_PI_PATH[2],
                             R_INPUT[2], V_PACK_LOW))
    ib = pb/V_PACK_LOW
    limits = [
        ('3.3 V rail vs TPS62162 1 A', i33, 1.0, 'datasheet'),
        ('5.1 V rail vs TPS62130 3 A', i51_transient, 3.0, 'datasheet'),
        ('5.1 V rail vs TPS22965 load switch (6 A per the BOM)', i51_transient, 6.0, 'datasheet'),
        ('5.1 V rail vs three JST GH contacts at 1 A', i51_transient, 3.0, 'datasheet'),
        ('Pack current at 6.0 V vs 5 A fuse', ib, PACK['fuse_a'], 'datasheet'),
        ('Pack current at 6.0 V vs 5.2 A pack rating', ib, PACK['max_discharge_a'], 'vendor'),
        ('Pack current at 6.0 V vs 10 A lowest protection trip', ib, PACK['protection_trip_a'][0], 'vendor'),
    ]
    return {'rail_3v3_a': i33, 'rail_5v1_a_average_high': i51, 'rail_5v1_a_with_transient': i51_transient,
            'battery_a_at_6v0': ib, 'battery_w_peak': pb,
            'checks': [{'check': name, 'current_a': cur, 'limit_a': lim, 'ratio_limit_over_current': lim/cur,
                        'limit_source': src} for name, cur, lim, src in limits]}


# --- Descent and canopy --------------------------------------------------------

RHO_STD = atmosphere.air_density(101325.0, 288.15)
RHO_WARM_THIN = atmosphere.air_density(98000.0, 288.15)    # ASSUMED: 15 degC, 980 hPa
RHO_COLD_DENSE = atmosphere.air_density(103500.0, 273.15)  # ASSUMED: 0 degC, 1035 hPa, sea level

# Drag coefficient (low, typical, high) on the stated reference area.
CANOPIES = {
    'round_flat': {
        'label': 'Round, flat circular', 'cd': (0.60, 0.75, 0.90), 'shape': 'disc',
        'reference': 'nominal (fabric) area',
        'source': 'Knacke 1992 table 5-1 gives 0.75-0.80; RECALLED, not re-read. Widened to the range '
                  'run_sizing.py already uses because small hobby canopies with spill holes read lower.',
        'stability': 'oscillates +/-10 to 40 degrees per Knacke (recalled); this is the proposal canopy'},
    'cruciform': {
        'label': 'Cruciform (cross)', 'cd': (0.60, 0.72, 0.85), 'shape': 'cross',
        'reference': 'fabric area of the cross',
        'source': 'Knacke 1992 table 5-1 gives 0.60-0.85; RECALLED, not re-read. One CFD paper '
                  '(search excerpt) reports 0.91.',
        'stability': '0 to +/-3 degrees per Knacke (recalled)'},
    'annular': {
        'label': 'Annular', 'cd': (0.85, 0.90, 0.95), 'shape': 'disc',
        'reference': 'nominal (fabric) area',
        'source': 'Knacke 1992 table 5-1 gives 0.85-0.95; RECALLED, not re-read. One CFD paper '
                  '(search excerpt) reports 1.03.',
        'stability': 'under +/-6 degrees per Knacke (recalled)'},
    'square_cone': {
        'label': 'Square cone (RD41 dropsonde type)', 'cd': (0.40, 0.48, 0.55), 'shape': 'square',
        'reference': 'area of the square mouth, sonde body drag included',
        'source': 'CALCULATED from the figures quoted in wiki/navigation_closed_loop_simulation.md '
                  'section 8: 323 g at 11 m/s under a 0.30 x 0.30 m canopy gives Cd = 0.476 at '
                  '1.225 kg/m3. The +/-15 % range is ASSUMED.',
        'stability': 'sold as eliminating pendulum motion (NCAR, as quoted in the project report)'},
}
CROSS_ARM_RATIO = 0.30   # ASSUMED arm width / arm length for the cruciform


def rd41_cd():
    """Square-cone Cd on mouth area implied by the dropsonde figures in the project report."""
    return sz.drag_area(0.323, 11.0, RHO_STD)/0.30**2


def canopy_area(mass_kg, speed, rho, cd):
    return sz.drag_area(mass_kg, speed, rho)/cd


def canopy_dimension(shape, area_m2):
    """Diameter of a disc, side of a square, or arm length of a cross, in metres."""
    if shape == 'disc':
        return math.sqrt(4*area_m2/math.pi)
    if shape == 'square':
        return math.sqrt(area_m2)
    if shape == 'cross':
        return math.sqrt(area_m2/(2*CROSS_ARM_RATIO - CROSS_ARM_RATIO**2))
    raise ValueError(shape)


def speed_sensitivity(mass_kg, speed, rho):
    """Partial derivatives of steady descent speed for a fixed canopy."""
    return {'per_gram_mps': speed/(2*mass_kg)/1000, 'per_10g_mps': speed/(2*mass_kg)/100,
            'per_percent_density_mps': -speed/200, 'per_percent_cd_mps': -speed/200,
            'per_kg_m3_mps': -speed/(2*rho)}


def descent_budget(mass_samples_g, n_seed=20261013):
    mass_samples_kg = np.asarray(mass_samples_g, float)/1000
    m_typ = float(np.mean(mass_samples_kg))
    m05, m95 = (float(np.percentile(mass_samples_kg, q)) for q in (5, 95))
    rng = np.random.default_rng(n_seed)
    n = len(mass_samples_kg)
    rho = rng.uniform(RHO_WARM_THIN, RHO_COLD_DENSE, n)
    out = {'density_kg_m3': {'standard': RHO_STD, 'warm_thin': RHO_WARM_THIN, 'cold_dense': RHO_COLD_DENSE},
           'design_mass_g': {'mean': m_typ*1000, 'p05': m05*1000, 'p95': m95*1000},
           'rd41_cd_on_mouth_area': rd41_cd(),
           'sensitivity_at_design': speed_sensitivity(m_typ, DESIGN_SPEED, RHO_STD),
           'cold_day': {'speed_if_trimmed_to_10.5_at_standard': DESIGN_SPEED*math.sqrt(RHO_STD/RHO_COLD_DENSE),
                        'speed_warm_thin': DESIGN_SPEED*math.sqrt(RHO_STD/RHO_WARM_THIN),
                        'trim_target_for_10_floor_one_drop': sz.target_speed_for_floor(
                            SPEED_MIN, RHO_STD, RHO_COLD_DENSE, sz.drop_test_relative_sigma(100.0, 100.0/DESIGN_SPEED, 2.0, 0.2)),
                        'trim_target_for_10_floor_no_error': sz.target_speed_for_floor(SPEED_MIN, RHO_STD, RHO_COLD_DENSE, 0.0)},
           'mass_lever': [{'mass_g': m, 'speed_mps': DESIGN_SPEED*math.sqrt(m/1000/m_typ)}
                          for m in (300.0, 320.0, 340.0, 350.0, 360.0)],
           'canopies': {}}
    for key, c in CANOPIES.items():
        lo, typ, hi = c['cd']
        rows = []
        for speed in (SPEED_MIN, DESIGN_SPEED, 12.0, SPEED_RECOMMENDED[1]):
            for label, mass in (('300 g', 0.300), ('mean', m_typ), ('350 g', 0.350)):
                areas = [canopy_area(mass, speed, RHO_STD, cd) for cd in (hi, typ, lo)]
                rows.append({'speed_mps': speed, 'mass': label, 'mass_g': mass*1000,
                             'cd_s_m2': sz.drag_area(mass, speed, RHO_STD),
                             'area_m2_cd_high': areas[0], 'area_m2_cd_typical': areas[1],
                             'area_m2_cd_low': areas[2],
                             'dimension_m_cd_high': canopy_dimension(c['shape'], areas[0]),
                             'dimension_m_cd_typical': canopy_dimension(c['shape'], areas[1]),
                             'dimension_m_cd_low': canopy_dimension(c['shape'], areas[2])})
        # A canopy cut from the typical Cd for the design speed at the mean mass,
        # then flown with the real Cd, mass and launch-day density unknown.
        area = canopy_area(m_typ, DESIGN_SPEED, RHO_STD, typ)
        cd = triangular_ppf(rng.random(n), lo, typ, hi)
        v = np.sqrt(2*mass_samples_kg*sz.G0/(rho*cd*area))
        # The same canopy after trimming on drop tests to 10.5 m/s at standard
        # density: Cd is then known and only mass and density move the speed.
        v_trim = DESIGN_SPEED*np.sqrt(mass_samples_kg/m_typ*RHO_STD/rho)
        out['canopies'][key] = {
            **c, 'sizing': rows, 'paper_design_area_m2': area,
            'paper_design_dimension_m': canopy_dimension(c['shape'], area),
            'paper_design_corner_speeds_mps': {
                'slowest': sz.descent_speed(m05, RHO_COLD_DENSE, hi*area),
                'fastest': sz.descent_speed(m95, RHO_WARM_THIN, lo*area)},
            'paper_design_speed_mps': {**summarise(v, SPEED_MIN, SPEED_RECOMMENDED[1])},
            'trimmed_speed_mps': {**summarise(v_trim, SPEED_MIN, SPEED_RECOMMENDED[1])}}
    return out
