"""GPL-3.0-only. Checks of the SYNTHETIC farmland world generator.

Simulation test code, not flight software. These checks show the generator
does what its parameters say; they say nothing about real imagery.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import farmland as F  # noqa: E402

NULL = dict(albedo_change=0.0, tramline_change=0.0, sun_flight=(180.0, 55.0))
_cache = {}


def world(seed=11, size_m=800.0, res_m=1.0, style='mixed', **season):
    key = (seed, size_m, res_m, style, tuple(sorted(season.items())))
    if key not in _cache:
        _cache[key] = F.generate(seed, size_m, res_m, style, F.Season(**season) if season else None)
    return _cache[key]


class Contract(unittest.TestCase):
    def test_shapes_dtypes_and_kinds(self):
        w = world()
        self.assertEqual(w.map.shape, (800, 800))
        for a, dt in ((w.map, np.float32), (w.flight, np.float32), (w.height, np.float32),
                      (w.field_id, np.int32), (w.kind, np.uint8)):
            self.assertEqual(a.shape, w.map.shape)
            self.assertEqual(a.dtype, dt)
        self.assertTrue(np.isfinite(w.map).all() and np.isfinite(w.flight).all())
        self.assertGreaterEqual(w.map.min(), 0)
        self.assertLessEqual(w.map.max(), 255)
        self.assertLessEqual(w.kind.max(), F.KIND_WATER)
        self.assertEqual((F.KIND_FIELD, F.KIND_HEDGE, F.KIND_TREE, F.KIND_ROAD, F.KIND_BUILDING, F.KIND_WATER),
                         (0, 1, 2, 3, 4, 5))
        self.assertTrue((w.field_id[w.kind != F.KIND_FIELD] == -1).all())
        self.assertTrue((w.height[w.kind == F.KIND_FIELD] == 0).all())
        self.assertTrue((w.height[w.kind == F.KIND_ROAD] == 0).all())
        self.assertTrue((w.height[w.kind == F.KIND_HEDGE] > 0).all())
        self.assertEqual(w.field_id.max(), len(w.fields) - 1)
        counts = np.bincount(w.field_id[w.field_id >= 0], minlength=len(w.fields))
        self.assertTrue((counts > 0).all())
        np.testing.assert_allclose(w.fields['area_m2'], counts*w.res**2)

    def test_default_size(self):
        w = F.generate(3, style='open')
        self.assertEqual(w.map.shape, (3200, 3200))
        self.assertEqual(w.res, 0.5)
        self.assertAlmostEqual(w.origin[0], -799.75)

    def test_grid_convention(self):
        for size, res in ((800.0, 1.0), (300.0, 0.5), (301.0, 1.0)):
            w = world(size_m=size, res_m=res, style='pasture')
            rows, cols = w.map.shape
            self.assertAlmostEqual(w.origin[0] + (cols - 1)/2*w.res, 0.0)
            self.assertAlmostEqual(w.origin[1] + (rows - 1)/2*w.res, 0.0)
            east, north = w.axes()
            self.assertAlmostEqual(east[0], w.origin[0])
            self.assertGreater(north[1], north[0])

    def test_north_increases_with_row_and_shadow_length(self):
        h = np.zeros((200, 200), np.float32)
        h[100, 100] = 10.0
        south = F.shadow_factor(h, 0.5, (180.0, 45.0))      # sun in the south: shadow to the north
        rows = np.nonzero(south[:, 100] > 0)[0]
        self.assertGreater(rows.min(), 100)
        self.assertAlmostEqual((rows.max() - 100)*0.5, 10.0/math.tan(math.radians(45)), delta=0.6)
        self.assertEqual(south[:100].max(), 0)
        east = F.shadow_factor(h, 0.5, (90.0, 30.0))         # sun in the east: shadow to the west
        cols = np.nonzero(east[100] > 0)[0]
        self.assertLess(cols.max(), 100)
        self.assertAlmostEqual((100 - cols.min())*0.5, 10.0/math.tan(math.radians(30)), delta=0.6)

    def test_field_centroid_uses_the_grid_convention(self):
        w = world()
        east, north = w.axes()
        big = np.argsort(-w.fields['area_m2'])[:5]
        for i in big:
            if w.fields['truncated'][i]:
                continue
            rr, cc = np.nonzero(w.field_id == i)
            self.assertAlmostEqual(east[cc].mean(), w.fields['east'][i], delta=30.0)  # copses and woods carved out of a field move its pixel centroid
            self.assertAlmostEqual(north[rr].mean(), w.fields['north'][i], delta=30.0)  # copses and woods carved out of a field move its pixel centroid

    def test_deterministic(self):
        a = F.generate(5, 400.0, 1.0, 'pasture')
        b = F.generate(5, 400.0, 1.0, 'pasture')
        for name in ('map', 'flight', 'height', 'field_id', 'kind', 'fields'):
            self.assertTrue(np.array_equal(getattr(a, name), getattr(b, name)), name)
        c = F.generate(6, 400.0, 1.0, 'pasture')
        self.assertFalse(np.array_equal(a.map, c.map))

    def test_meta_says_synthetic(self):
        w = world()
        self.assertIn('SYNTHETIC', w.meta['statement'])
        self.assertTrue(w.meta['synthetic'])
        self.assertEqual((w.meta['seed'], w.meta['style']), (11, 'mixed'))
        self.assertEqual(w.meta['style_parameters'], F.asdict(F.STYLES['mixed']))
        for row in F.PARAMETERS:
            self.assertEqual(len(row), 4)
            self.assertTrue(row[2] in ('assumed', 'source', 'source + assumed'), row)
            if 'source' in row[2]:
                self.assertIn('http', row[3])

    def test_styles_exist(self):
        for name in ('arable', 'mixed', 'pasture', 'open', 'tramline'):
            self.assertEqual(F.STYLES[name].name, name)


class SeasonBehaviour(unittest.TestCase):
    def test_null_season_gives_identical_rasters(self):
        for style in ('mixed', 'pasture'):
            w = world(style=style, **NULL)
            self.assertTrue(np.array_equal(w.map, w.flight))
            self.assertFalse(w.fields['changed'].any())

    def test_map_and_structure_do_not_depend_on_season(self):
        a, b = world(**NULL), world()
        for name in ('map', 'height', 'field_id', 'kind'):
            self.assertTrue(np.array_equal(getattr(a, name), getattr(b, name)), name)
        self.assertFalse(np.array_equal(b.map, b.flight))

    def test_changed_fractions(self):
        w = world(style='pasture', size_m=1600.0, albedo_change=0.3, tramline_change=0.5)
        n = len(w.fields)
        self.assertEqual(w.fields['albedo_changed'].sum(), round(0.3*n))
        self.assertEqual(w.fields['tram_changed'].sum(), round(0.5*n))
        same = ~w.fields['albedo_changed']
        np.testing.assert_array_equal(w.fields['map_mean'][same], w.fields['flight_mean'][same])

    def test_field_mean_correlation(self):
        keep = world(style='pasture', size_m=1600.0, albedo_change=0.0)
        redo = world(style='pasture', size_m=1600.0, albedo_change=1.0)
        out = {}
        for name, w in (('0', keep), ('1', redo)):
            big = w.fields['area_m2'] > 2000
            self.assertGreater(big.sum(), 80)
            out[name] = np.corrcoef(F.field_means(w, w.map)[0][big], F.field_means(w, w.flight)[0][big])[0, 1]
        print(f'\nper-field mean correlation map/flight: albedo_change=0 {out["0"]:.3f}, =1 {out["1"]:.3f}')
        self.assertGreater(out['0'], 0.97)
        self.assertLess(abs(out['1']), 0.3)

    def test_structure_persists_when_every_field_changes(self):
        for style in ('mixed', 'pasture', 'open', 'tramline'):
            w = world(style=style, size_m=1600.0, albedo_change=1.0)
            got = {s: F.gradient_correlation(w.map, w.flight, w.res, s) for s in (0.0, 1.0, 2.0, 4.0, 8.0)}
            print(f'\ngradient-magnitude correlation, albedo_change=1, {style} (1 m/px): '
                  + ', '.join(f'{s:g} m: {v:.3f}' for s, v in got.items()))
            if style in ('mixed', 'pasture'):
                for s in (1.0, 2.0, 4.0):
                    self.assertGreater(got[s], 0.2, (style, s))
            else:
                self.assertGreater(got[2.0], 0.05, style)

    def test_gain_offset_gamma(self):
        base = world(**NULL)
        w = world(gain=0.8, offset=12.0, gamma=1.3, **NULL)
        want = np.clip(255*(base.map/255.0)**1.3*0.8 + 12.0, 0, 255)
        np.testing.assert_allclose(w.flight, want, atol=1e-2)
        self.assertTrue(np.array_equal(w.map, base.map))

    def test_sun_moves_the_shadow_only(self):
        w = world(albedo_change=0.0, tramline_change=0.0, sun_flight=(90.0, 30.0))
        diff = np.abs(w.map - w.flight) > 0.5
        self.assertGreater(diff.mean(), 0.002)
        self.assertLess(diff.mean(), 0.25)
        # every changed pixel is a tall thing or lies within reach of a shadow
        reach = max(F.shadow_factor(w.height, w.res, (180.0, 55.0)).max(), 1)
        lit = (F.shadow_factor(w.height, w.res, (180.0, 55.0)) > 0) | \
              (F.shadow_factor(w.height, w.res, (90.0, 30.0)) > 0) | (w.height > 0)
        self.assertEqual(reach, 1)
        self.assertTrue((lit | ~diff).all())

    def test_leaf_off_touches_only_vegetation_and_its_shadow(self):
        a = world(**NULL)
        b = world(leaf_off=1.0, **NULL)
        veg = (a.kind == F.KIND_HEDGE) | (a.kind == F.KIND_TREE)
        self.assertGreater(b.flight[veg].mean(), a.flight[veg].mean() + 5)
        far = (F.shadow_factor(a.height, a.res, (180.0, 55.0)) == 0) & ~veg
        self.assertTrue(np.array_equal(a.flight[far], b.flight[far]))


class Statistics(unittest.TestCase):
    def test_field_area_and_count_per_style(self):
        for name, st in F.STYLES.items():
            areas = np.concatenate([world(seed=s, size_m=1600.0, style=name).fields['area_full_m2']
                                    for s in (11, 12)])/1e4
            w = world(size_m=1600.0, style=name)
            median = float(np.median(areas))
            print(f'\n{name}: {len(w.fields)} fields in 256 ha, parcel area median {median:.1f} ha '
                  f'(stated {st.field_area_ha:g}), p10 {np.percentile(areas, 10):.1f}, p90 {np.percentile(areas, 90):.1f}')
            # fields that reach into the raster are size-biased towards large ones
            self.assertGreater(median, 0.75*st.field_area_ha)
            self.assertLess(median, 1.9*st.field_area_ha)
            expected = 256.0/(1.1*st.field_area_ha)
            self.assertGreater(len(w.fields), 0.5*expected)
            self.assertLess(len(w.fields), 3.0*expected + 8)
            self.assertTrue((w.fields['area_full_m2'] >= 0.99*st.min_field_ha*1e4).all())

    def test_hedge_area_fraction_matches_parameters(self):
        got = {}
        for name, st in F.STYLES.items():
            w = world(size_m=1600.0, style=name)
            got[name] = w.meta['kind_fraction']['hedge']
            # boundary length per unit area of a tiling by squares of the mean
            # field area is 2/sqrt(area); a fraction p_hedge of it carries a hedge
            mean_area = 1.1*st.field_area_ha*1e4
            expect = st.p_hedge*st.hedge_width_m*2/math.sqrt(mean_area)
            print(f'\n{name}: hedge fraction {got[name]:.4f}, estimate from parameters {expect:.4f}, '
                  f'tree {w.meta["kind_fraction"]["tree"]:.4f}, road {w.meta["kind_fraction"]["road"]:.4f}')
            if name != 'open':      # a dozen boundaries, 15 % hedged: too few to bound from below
                self.assertGreater(got[name], 0.4*expect)
            self.assertLess(got[name], 2.2*expect + 0.004)
        self.assertGreater(got['pasture'], got['mixed'])
        self.assertGreater(got['mixed'], got['open'])

    def test_tramline_spacing_measured_from_raster(self):
        w = world(seed=4, size_m=1200.0, res_m=0.5, style='tramline')
        f = w.fields
        self.assertTrue(np.isin(f['tram_spacing_m'], F.STYLES['tramline'].tramline_spacings).all())
        self.assertTrue(((f['tram_gauge_m'] >= 1.8) & (f['tram_gauge_m'] <= 2.0)).all())
        rows = []
        for i in np.argsort(-f['area_m2']):
            if f['area_m2'][i] < 2.5e4 or f['tram_contrast_map'][i] == 0:
                continue
            got = F.measure_tramline_spacing(w, i)
            if not math.isnan(got):
                rows.append((float(f['tram_spacing_m'][i]), got))
        print('\ntramline spacing recorded -> measured (m): ' + ', '.join(f'{a:g}->{b:g}' for a, b in rows))
        self.assertGreaterEqual(len(rows), 4)
        good = sum(abs(a - b) <= 0.75 for a, b in rows)
        self.assertGreaterEqual(good, math.ceil(0.85*len(rows)))
        # flight day: phase redrawn, spacing kept unless the field was re-bearinged
        flight = [(float(f['tram_spacing_flight_m'][i]),
                   F.measure_tramline_spacing(w, i, w.flight, f['tram_bearing_flight_deg'][i]))
                  for i in np.argsort(-f['area_m2'])[:4] if f['tram_contrast_flight'][i] != 0]
        self.assertTrue(any(abs(a - b) <= 0.75 for a, b in flight))

    def test_tramline_fraction(self):
        for name in ('pasture', 'arable'):
            w = world(size_m=1600.0, style=name)
            frac = (w.fields['tram_contrast_map'] != 0).mean()
            self.assertAlmostEqual(frac, F.STYLES[name].tramline_fraction, delta=0.17)

    def test_radiometry(self):
        for name in F.STYLES:
            w = world(size_m=1600.0, style=name)
            s = F.statistics(w)
            print(f'\n{name}: between-field SD {s["between_field_sd"]:.1f}, within-field SD '
                  f'{s["within_field_sd_median"]:.1f}, hedge-field {s["hedge_minus_field"]:.1f}, '
                  f'road-field {s["road_minus_field"]:.1f}')
            self.assertGreater(w.meta['kind_fraction']['road'], 0.001)
            self.assertLess(s['hedge_minus_field'], -30)
            self.assertGreater(s['within_field_sd_median'], 2)
            self.assertLess(s['within_field_sd_median'], 20)
            cover = np.array(F.STYLES[name].map_cover)
            p = cover[:, 2]/cover[:, 2].sum()
            mix_sd = math.sqrt(p @ (cover[:, 1]**2 + cover[:, 0]**2) - (p @ cover[:, 0])**2)
            self.assertGreater(s['between_field_sd'], 0.5*mix_sd)
            self.assertLess(s['between_field_sd'], 1.6*mix_sd)


if __name__ == '__main__':
    unittest.main(verbosity=1)
