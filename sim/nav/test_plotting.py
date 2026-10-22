"""GPL-3.0-only. Checks of the Pillow plotting helper: valid PNGs, gaps, log axes, axis direction."""
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
from PIL import Image

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotting as pl  # noqa: E402

RED, BLUE = (255, 0, 0), (0, 0, 255)


def pixels(im, rgb):
    """(row, col) arrays of pixels exactly equal to rgb."""
    a = np.asarray(im.convert('RGB'))
    return np.nonzero(np.all(a == rgb, axis=2))


def is_red(im, x, y):
    r, g, b = im.convert('RGB').getpixel((int(round(x)), int(round(y))))
    return r > 200 and g < 90 and b < 90


class Base(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)

    def check_png(self, path, size):
        with Image.open(path) as im:
            self.assertEqual(im.format, 'PNG')
            self.assertEqual(im.size, tuple(size))
            im.load()
            a = np.asarray(im.convert('RGB'))
        # Text is clamped 2 px inside the image, so the outermost ring stays white.
        for edge in (a[0], a[-1], a[:, 0], a[:, -1]):
            self.assertTrue((edge == 255).all())


class TestTicks(unittest.TestCase):
    def test_unit_range(self):
        self.assertEqual(pl.nice_ticks(0, 1), [0.0, 0.2, 0.4, 0.6, 0.8, 1.0])

    def test_small_offset_range(self):
        t = pl.nice_ticks(0.03, 0.47)
        self.assertTrue(3 <= len(t) <= 9)
        self.assertTrue(all(0.03 <= v <= 0.47 for v in t))
        self.assertTrue(np.allclose(np.diff(t), t[1] - t[0]))
        self.assertIn(round(t[1] - t[0], 12), (0.05, 0.1, 0.02))

    def test_negative_to_positive(self):
        t = pl.nice_ticks(-12, 345)
        self.assertIn(0.0, t)
        self.assertTrue(all(-12 <= v <= 345 for v in t))
        step = t[1] - t[0]
        self.assertIn(step, (20.0, 50.0, 100.0))
        self.assertTrue(4 <= len(t) <= 10)

    def test_log_decades_and_minor(self):
        self.assertEqual(pl.nice_ticks(1e-3, 2e2, log=True), [1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0])
        m = pl.log_minor_ticks(1e-3, 2e2)
        self.assertIn(2e-3, m)
        self.assertIn(5e-3, m)
        self.assertIn(5.0, m)
        self.assertIn(2e2, m)
        self.assertNotIn(5e2, m)
        self.assertNotIn(1.0, m)

    def test_no_float_noise_in_labels(self):
        for lo, hi in [(0, 1), (0.03, 0.47), (0.1, 0.4), (-12, 345), (0, 0.0007)]:
            for s in pl.fmt_ticks(pl.nice_ticks(lo, hi)):
                self.assertLessEqual(len(s), 8, s)
        self.assertEqual(pl.fmt_ticks([0.1, 0.2, 0.30000000000000004]), ['0.1', '0.2', '0.3'])
        self.assertEqual(pl.fmt_ticks([-0.0, 0.5]), ['0.0', '0.5'])
        self.assertEqual(pl.fmt_ticks([1e-3, 1e-2, 1.0, 1e3, 1e5], log=True),
                         ['0.001', '0.01', '1', '1000', '100000'])

    def test_limits(self):
        lo, hi = pl.nice_limits(0.0, 0.93)
        self.assertEqual(lo, 0.0)                  # non-negative data keeps a zero floor
        self.assertGreaterEqual(hi, 0.93)
        self.assertEqual(pl.nice_limits(2e-3, 150, log=True), (1e-3, 1e3))
        lo, hi = pl.nice_limits(5, 5)              # degenerate range still has width
        self.assertLess(lo, hi)

    def test_degenerate_inputs(self):
        self.assertEqual(pl.nice_ticks(float('nan'), 1), [])
        self.assertEqual(pl.nice_ticks(-1, 5, log=True), [])
        self.assertEqual(pl.nice_ticks(1, 1), [1.0])


class TestTransform(unittest.TestCase):
    def test_linear_and_inverted(self):
        f = pl.make_transform(0, 300, 0, 100)
        self.assertAlmostEqual(float(f(0)), 0.0)
        self.assertAlmostEqual(float(f(300)), 100.0)
        g = pl.make_transform(0, 300, 0, 100, invert=True)
        self.assertAlmostEqual(float(g(300)), 0.0)
        self.assertLess(float(g(300)), float(g(0)))      # larger x is further left

    def test_log(self):
        f = pl.make_transform(1e-3, 1e3, 600, 0, log=True)     # y style: lo at the bottom
        self.assertAlmostEqual(float(f(1e-3)), 600.0)
        self.assertAlmostEqual(float(f(1.0)), 300.0)
        self.assertAlmostEqual(float(f(1e3)), 0.0)


class TestLineChart(Base):
    def test_writes_png_of_requested_size(self):
        x = np.linspace(0, 10, 50)
        for size in [(1100, 620), (500, 320), (900, 700)]:
            p = pl.line_chart(self.dir/'a.png', [dict(x=x, y=np.sin(x), label='sin')],
                              'T', 'x', 'y', subtitle='sub', size=size)
            self.assertEqual(Path(p), self.dir/'a.png')
            self.check_png(p, size)

    def test_all_styles_and_legends(self):
        x = np.arange(20.0)
        ser = [dict(x=x, y=x**1.2, label=f'series {i}', style=s, band=(x**1.2 - 1, x**1.2 + 1))
               for i, s in enumerate(['line', 'dash', 'points', 'line+points'])]
        for lg in ('right', 'bottom', None):
            p = self.dir/f'l_{lg}.png'
            pl.line_chart(p, ser, 'T', 'x', 'y', legend=lg, size=(700, 450))
            self.check_png(p, (700, 450))

    def test_nan_gaps_do_not_raise_and_break_the_line(self):
        ser = [dict(x=[0, 1, 2, 3, 4], y=[1, 1, np.nan, 1, 1], label='r', color='#ff0000',
                    band=(np.array([0, 0, np.nan, 0, 0.0]), np.array([2, 2, 2, 2, np.inf])))]
        im, info = pl._render_line(ser, 'T', 'x', 'y', '', (0, 4), (0, 2), (700, 420), False, False,
                                   (), (), None, False)
        L, T, R, B = info['rect']
        px = lambda x, y: (L + float(info['tx'](x)), T + float(info['ty'](y)))  # noqa: E731
        self.assertTrue(is_red(im, *px(0.5, 1)))
        self.assertTrue(is_red(im, *px(3.5, 1)))
        self.assertFalse(is_red(im, *px(2.0, 1)))       # the gap itself
        self.assertFalse(is_red(im, *px(2.2, 1)))

    def test_inf_nan_everywhere(self):
        x = np.array([0, 1, np.nan, 3, np.inf, 5.0])
        y = np.array([np.nan, np.inf, -np.inf, 1, 2, np.nan])
        ser = [dict(x=x, y=y, label='a'),
               dict(x=[], y=[], label='empty'),
               dict(x=[1.0], y=[1.0], label='single'),
               dict(x=[1, 2], y=[np.nan, np.nan], label='allnan', style='points')]
        p = pl.line_chart(self.dir/'n.png', ser, 'T', 'x', 'y', size=(600, 400))
        self.check_png(p, (600, 400))
        p = pl.line_chart(self.dir/'e.png', [], 'No series', 'x', 'y', size=(600, 400))
        self.check_png(p, (600, 400))

    def test_log_axes_skip_non_positive(self):
        x = np.array([-1.0, 0.0, 1.0, 10.0, 100.0])
        y = np.array([5.0, -2.0, 0.0, 1e-3, 1e2])
        ser = [dict(x=x, y=y, label='mixed', band=(y - 1, y + 1)),
               dict(x=x, y=-np.ones(5), label='all non-positive')]
        for kw in (dict(logy=True), dict(logx=True), dict(logx=True, logy=True)):
            p = pl.line_chart(self.dir/'log.png', ser, 'T', 'x', 'y', size=(600, 400),
                              hlines=[(-3, 'neg', None), (10, 'ten', None)],
                              vlines=[(0, 'zero', None), (5, 'five', None)], **kw)
            self.check_png(p, (600, 400))
        im, info = pl._render_line(ser, 'T', 'x', 'y', '', None, None, (600, 400), True, True,
                                   (), (), None, False)
        self.assertEqual(info['ylim'], (1e-3, 1e3))     # decade limits from the positive data
        self.assertEqual(info['xlim'], (1.0, 100.0))

    def test_invert_x_puts_larger_x_on_the_left(self):
        ser = [dict(x=[0.0], y=[1.0], style='points', color='#0000ff', label='low', width=3),
               dict(x=[300.0], y=[1.0], style='points', color='#ff0000', label='high', width=3)]
        got = {}
        for inv in (False, True):
            im, info = pl._render_line(ser, 'T', 'x', 'y', '', (-50, 350), (0, 2), (700, 420), False,
                                       False, (), (), None, inv)
            r_row, r_col = pixels(im, RED)
            b_row, b_col = pixels(im, BLUE)
            self.assertGreater(r_col.size, 0)
            self.assertGreater(b_col.size, 0)
            got[inv] = (r_col.mean(), b_col.mean())
            L = info['rect'][0]
            self.assertAlmostEqual(r_col.mean(), L + float(info['tx'](300.0)), delta=1.5)
        self.assertGreater(got[False][0], got[False][1])      # normal: 300 right of 0
        self.assertLess(got[True][0], got[True][1])           # inverted: 300 left of 0

    def test_legend_right_does_not_overlap_plot(self):
        ser = [dict(x=[0, 1], y=[0, 1], label='a fairly long legend label ' + str(i)) for i in range(5)]
        for size in [(1100, 620), (500, 400)]:
            im, info = pl._render_line(ser, 'T', 'x', 'y', '', None, None, size, False, False,
                                       (), (), 'right', False)
            self.assertEqual(info['mode'], 'right')
            self.assertGreater(info['legend'][0], info['rect'][2])
            self.assertLessEqual(info['legend'][2], size[0])

    def test_legend_bottom_below_axis_labels(self):
        ser = [dict(x=[0, 1], y=[0, 1], label=f'series number {i}') for i in range(9)]
        im, info = pl._render_line(ser, 'T', 'x', 'y', '', None, None, (700, 450), False, False,
                                   (), (), 'bottom', False)
        self.assertEqual(info['mode'], 'bottom')
        self.assertGreater(info['legend'][1], info['rect'][3] + 20)
        self.assertLessEqual(info['legend'][3], 450)

    def test_too_many_series_for_side_legend_falls_back_to_bottom(self):
        ser = [dict(x=[0, 1], y=[0, 1], label=f'series {i}') for i in range(30)]
        im, info = pl._render_line(ser, 'T', 'x', 'y', '', None, None, (800, 400), False, False,
                                   (), (), 'right', False)
        self.assertEqual(info['mode'], 'bottom')

    def test_long_text_stays_inside_image(self):
        long = 'a very long title that cannot possibly fit ' * 6
        ser = [dict(x=[0, 1], y=[0, 1], label=long)]
        p = pl.line_chart(self.dir/'long.png', ser, long, long, long, subtitle=long, size=(500, 350),
                          hlines=[(0.5, long, None)], vlines=[(0.5, long, None)])
        self.check_png(p, (500, 350))

    def test_band_is_behind_lines(self):
        x = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
        ser = [dict(x=x, y=[1, 1, 1, 1, 1], band=(x*0, x*0 + 2), color='#ff0000', width=4)]
        im, info = pl._render_line(ser, 'T', 'x', 'y', '', (0, 4), (0, 2), (700, 420), False, False,
                                   (), (), None, False)
        L, T, R, B = info['rect']
        x_, y_ = L + float(info['tx'](2)), T + float(info['ty'](1))
        self.assertTrue(is_red(im, x_, y_))              # line colour is pure, not tinted by the band
        r, g, b = im.getpixel((int(L + float(info['tx'](2.25))), int(T + float(info['ty'](1.25)))))  # off-grid
        self.assertTrue(r > 240 and 150 < g < 235)       # band is a light tint away from the line

    def test_repeat_calls_are_identical(self):
        ser = [dict(x=np.arange(30.0), y=np.arange(30.0)**1.5, label='a', band=(np.zeros(30), np.ones(30)*50))]
        a = pl.line_chart(self.dir/'1.png', ser, 'T', 'x', 'y', size=(600, 400), hlines=[(20, 'h', None)])
        b = pl.line_chart(self.dir/'2.png', ser, 'T', 'x', 'y', size=(600, 400), hlines=[(20, 'h', None)])
        with Image.open(a) as ia, Image.open(b) as ib:
            self.assertTrue(np.array_equal(np.asarray(ia), np.asarray(ib)))

    def test_font_fallback(self):
        with mock.patch.object(pl, 'FONT_REG', 'C:/nope/missing.ttf'), \
                mock.patch.object(pl, 'FONT_BOLD', 'C:/nope/missing-bold.ttf'):
            p = pl.line_chart(self.dir/'f.png', [dict(x=[0, 1], y=[0, 1], label='a')], 'T', 'x', 'y',
                              size=(500, 320))
        self.check_png(p, (500, 320))

    def test_creates_missing_directory(self):
        p = pl.line_chart(self.dir/'sub'/'dir'/'a.png', [dict(x=[0, 1], y=[0, 1])], 'T', 'x', 'y',
                          size=(400, 300))
        self.check_png(p, (400, 300))

    def test_bad_inputs_degrade_gracefully(self):
        ser = [dict(x=[0, 1, 2], y=[1, 2], color='not-a-colour', style='weird', label='a'),
               dict(y=[1, 2, 3], label='no x')]
        p = pl.line_chart(self.dir/'b.png', ser, 'T', 'x', 'y', size=(500, 320),
                          xlim=(float('nan'), None), ylim=(5, 5), hlines=[(float('inf'), 'x', None), ('a',)])
        self.check_png(p, (500, 320))
        with self.assertRaises(ValueError):
            pl.line_chart(self.dir/'tiny.png', ser, 'T', 'x', 'y', size=(50, 50))


class TestBarChart(Base):
    CATS = ['Calm', 'Gusty wind', 'A really rather long category name that must wrap', 'x']

    def groups(self):
        return [dict(label='A', values=[1, 2, 3, 4]), dict(label='B', values=[2, np.nan, 1, 0.5]),
                dict(label='C', values=[0.5, 1, np.nan, np.nan], color='#123456')]

    def test_writes_png(self):
        for size in [(1100, 620), (600, 400)]:
            p = pl.bar_chart(self.dir/'b.png', self.CATS, self.groups(), 'T', 'y', subtitle='s',
                             size=size, hlines=[(2.5, 'limit', None)])
            self.check_png(p, size)

    def test_nan_marked_na_and_bars_drawn(self):
        g = [dict(label='A', values=[3.0, np.nan], color='#ff0000'),
             dict(label='B', values=[np.nan, 3.0], color='#0000ff')]
        im, info = pl._render_bar(['one', 'two'], g, 'T', 'y', '', (0, 4), (700, 420), (), False, True)
        L, T, R, B = info['rect']
        rows, cols = pixels(im, RED)
        self.assertGreater(rows.size, 500)                   # a bar exists
        self.assertLess(cols.mean(), (L + R)/2)              # category one is on the left half
        rows, cols = pixels(im, BLUE)
        self.assertGreater(cols.mean(), (L + R)/2)
        # the 'n/a' text sits just above the baseline where the bar is missing
        base = int(T + float(info['ty'](0)))
        patch = np.asarray(im.convert('L'))[base - 20:base - 1, int(L + (R - L)*0.12):int(L + (R - L)*0.38)]
        self.assertTrue((patch < 200).any())

    def test_no_value_labels_and_negative_values(self):
        g = [dict(label='A', values=[-2, 3, -1]), dict(label='B', values=[1, -4, 2])]
        for vl in (True, False):
            p = pl.bar_chart(self.dir/'neg.png', ['a', 'b', 'c'], g, 'T', 'y', value_labels=vl,
                             size=(600, 400))
            self.check_png(p, (600, 400))

    def test_log_axis_skips_non_positive(self):
        g = [dict(label='A', values=[1e-3, 0.0, -1.0, 50.0]), dict(label='B', values=[np.nan, 10, 1e2, 2e3])]
        p = pl.bar_chart(self.dir/'log.png', ['a', 'b', 'c', 'd'], g, 'T', 'y', logy=True,
                         hlines=[(-1, 'bad', None), (5, 'ok', None)], size=(700, 420))
        self.check_png(p, (700, 420))
        im, info = pl._render_bar(['a', 'b', 'c', 'd'], g, 'T', 'y', '', None, (700, 420), (), True, True)
        self.assertEqual(info['ylim'][0], 1e-3)
        self.assertGreaterEqual(info['ylim'][1], 2e3)

    def test_degenerate_inputs(self):
        for cats, groups in [([], []), (['a'], []), (['a', 'b'], [dict(label='x', values=[])]),
                             (['a'], [dict(label='x', values=[np.nan])]),
                             (['a', 'b'], [dict(label='x', values=[1, 2, 3, 4])]),
                             (['a'], [dict(values=[0.0])])]:
            p = pl.bar_chart(self.dir/'d.png', cats, groups, 'T', 'y', size=(500, 320))
            self.check_png(p, (500, 320))

    def test_many_bars_use_rotated_or_shrunk_labels(self):
        cats = [f'c{i}' for i in range(14)]
        g = [dict(label=f'g{j}', values=list(np.linspace(1234.5, 99999.9, 14))) for j in range(4)]
        p = pl.bar_chart(self.dir/'many.png', cats, g, 'T', 'y', size=(900, 520))
        self.check_png(p, (900, 520))

    def test_legend_beside_plot_not_over_bars(self):
        im, info = pl._render_bar(['a', 'b'], self.groups(), 'T', 'y', '', None, (700, 420), (), False, True)
        self.assertEqual(info['mode'], 'right')
        self.assertGreater(info['legend'][0], info['rect'][2])

    def test_value_labels_share_decimals(self):
        self.assertEqual(pl.fmt_values([0.8, 1.9, 3.6, 2.0]), ['0.8', '1.9', '3.6', '2.0'])
        self.assertEqual(pl.fmt_values([12.34, 5.0]), ['12.3', '5.0'])
        self.assertEqual(pl.fmt_values([1234.5, 2e5]), ['1.23e3', '2.00e5'])


class TestPanel(Base):
    def test_tiles_images(self):
        paths = []
        for i, (w, h) in enumerate([(200, 100), (200, 100), (200, 100)]):
            p = self.dir/f'i{i}.png'
            Image.new('RGB', (w, h), (255, 0, 0)).save(p)
            paths.append(p)
        out = pl.panel(self.dir/'p.png', paths, columns=2, padding=12)
        with Image.open(out) as im:
            self.assertEqual(im.size, (2*200 + 3*12, 2*100 + 3*12))
            im.load()
            self.assertEqual(im.convert('RGB').getpixel((12 + 5, 12 + 5)), RED)
            # third image sits in row 2 column 1; the fourth cell stays white
            self.assertEqual(im.convert('RGB').getpixel((12 + 5, 12 + 100 + 12 + 5)), RED)
            self.assertEqual(im.convert('RGB').getpixel((12 + 200 + 12 + 5, 12 + 100 + 12 + 5)), (255, 255, 255))

    def test_mixed_sizes_and_columns(self):
        paths = []
        for i, (w, h) in enumerate([(120, 80), (60, 200), (90, 90), (40, 40)]):
            p = self.dir/f'm{i}.png'
            Image.new('RGBA', (w, h), (0, 0, 255, 255)).save(p)
            paths.append(p)
        for cols, expect in [(1, (120 + 16, 80 + 200 + 90 + 40 + 5*8)), (4, (120 + 60 + 90 + 40 + 5*8, 200 + 16)),
                             (9, (120 + 60 + 90 + 40 + 5*8, 200 + 16))]:
            out = pl.panel(self.dir/'p.png', paths, columns=cols, padding=8)
            with Image.open(out) as im:
                self.assertEqual(im.size, expect, cols)

    def test_accepts_chart_output_and_rejects_empty(self):
        a = pl.line_chart(self.dir/'a.png', [dict(x=[0, 1], y=[0, 1])], 'T', 'x', 'y', size=(300, 200))
        b = pl.bar_chart(self.dir/'b.png', ['a'], [dict(label='g', values=[1])], 'T', 'y', size=(300, 200))
        out = pl.panel(self.dir/'p.png', [a, b], columns=2)
        with Image.open(out) as im:
            self.assertEqual(im.size, (600 + 36, 200 + 24))
        with self.assertRaises(ValueError):
            pl.panel(self.dir/'x.png', [])


class TestDemo(Base):
    def test_demo_writes_panel(self):
        out = pl._demo(self.dir/'demo.png')
        with Image.open(out) as im:
            self.assertGreater(im.width, 1000)
            self.assertGreater(im.height, 700)


if __name__ == '__main__':
    unittest.main()
