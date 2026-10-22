"""GPL-3.0-only. Small PNG plotting helper for machines without matplotlib.

Pillow draws and NumPy handles the data. Three entry points:

  line_chart   lines, dashes, markers and translucent bands on linear or log axes
  bar_chart    grouped vertical bars on a linear or log axis, NaN shown as "n/a"
  panel        tile existing PNG files into one figure

Charts are rendered at 2x and downsampled with LANCZOS. Plot content is drawn on
its own layer the size of the plot area, so nothing can spill over the axes. Text
is measured with ImageDraw.textbbox and shifted or shrunk to stay inside the
image. There is no module state: every call builds its own canvas.

Running the module writes results/plotting_demo.png next to it.
"""
import math
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageColor, ImageDraw, ImageFont

SS = 2                                      # supersampling factor
FONT_REG = 'C:/Windows/Fonts/arial.ttf'
FONT_BOLD = 'C:/Windows/Fonts/arialbd.ttf'
# Okabe-Ito based, colour-blind safe; the yellow is darkened to read on white.
PALETTE = ('#0072B2', '#D55E00', '#009E73', '#CC79A7',
           '#E69F00', '#56B4E9', '#4D4D4D', '#B59B00')
INK, GREY, FRAME = '#222222', '#666666', '#555555'
GRID, GRID_MINOR, REF = '#E2E2E2', '#F0F0F0', '#B22222'
SWATCH = 24                                 # legend swatch width, output px


# ----------------------------------------------------------------- axis maths

def nice_step(span, target=6):
    """1-2-5 step giving about `target` intervals across `span`."""
    if not (span > 0 and math.isfinite(span)):
        return 1.0
    e = math.floor(math.log10(span/max(target, 1)))
    cands = [m*10.0**k for k in (e - 1, e, e + 1) for m in (1, 2, 5)]
    return min(cands, key=lambda s: (abs(math.log(span/s/max(target, 1))), -s))


def nice_limits(lo, hi, target=6, log=False, pad=0.03):
    """Outward-rounded axis limits: decades on log axes, half 1-2-5 steps otherwise.

    Non-negative data never gets a negative lower limit (likewise for non-positive).
    """
    if hi < lo:
        lo, hi = hi, lo
    if log:
        if not (lo > 0 and hi > 0 and math.isfinite(hi)):
            return 1.0, 10.0
        a = math.floor(math.log10(lo) + 1e-9)
        b = math.ceil(math.log10(hi) - 1e-9)
        return float(f'1e{a}'), float(f'1e{max(b, a + 1)}')
    nonneg, nonpos = lo >= 0, hi <= 0
    span = hi - lo
    if span <= 0:
        span = abs(hi)*0.2 or 1.0
        lo, hi = lo - span/2, hi + span/2
    else:
        lo, hi = lo - pad*span, hi + pad*span
    step = nice_step(hi - lo, target)/2
    nlo = math.floor(lo/step + 1e-9)*step
    nhi = math.ceil(hi/step - 1e-9)*step
    if nonneg:
        nlo = max(nlo, 0.0)
    if nonpos:
        nhi = min(nhi, 0.0)
    if nhi <= nlo:
        nhi = nlo + step
    return float(f'{nlo:.12g}'), float(f'{nhi:.12g}')


def nice_ticks(lo, hi, target=6, log=False):
    """Major tick values inside [lo, hi]: 1-2-5 multiples, or decades when `log`."""
    if hi < lo:
        lo, hi = hi, lo
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return []
    if log:
        if not (lo > 0 and hi > 0):
            return []
        a, b = math.ceil(math.log10(lo) - 1e-9), math.floor(math.log10(hi) + 1e-9)
        k = max(1, math.ceil((b - a + 1)/(max(target, 4) + 2)))   # thin very wide ranges
        return [float(f'1e{e}') for e in range(math.ceil(a/k)*k, b + 1, k)]
    step = nice_step(hi - lo, target)
    i0, i1 = math.ceil(lo/step - 1e-9), math.floor(hi/step + 1e-9)
    return [float(f'{i*step:.12g}') for i in range(i0, i1 + 1)]    # .12g strips 0.30000000000000004


def log_minor_ticks(lo, hi):
    """Minor ticks at 2 and 5 times each decade inside [lo, hi]."""
    if not (0 < lo < hi and math.isfinite(hi)) or hi/lo > 1e40:
        return []
    out = []
    for e in range(math.floor(math.log10(lo)) - 1, math.ceil(math.log10(hi)) + 1):
        for m in (2, 5):
            v = float(f'{m}e{e}')
            if lo*(1 - 1e-9) <= v <= hi*(1 + 1e-9):
                out.append(v)
    return out


def _sci(s):
    return s.replace('e+0', 'e').replace('e-0', 'e-').replace('e+', 'e')


def _ndec(v):
    for d in range(10):
        if abs(round(v, d) - v) <= 1e-9*max(1.0, abs(v)):
            return d
    return 10


def fmt_ticks(ticks, log=False):
    """Tick labels with a common number of decimals and no float noise."""
    if not len(ticks):
        return []
    if log:
        return [_sci(f'{v:g}') for v in ticks]
    if any(abs(v) >= 1e6 or 0 < abs(v) < 1e-4 for v in ticks):
        return [_sci(f'{v:.3g}') for v in ticks]
    d = min(max(_ndec(v) for v in ticks), 8)
    out = [f'{v:.{d}f}' for v in ticks]
    return [f'{0:.{d}f}' if float(t) == 0 else t for t in out]      # no '-0.0'


def fmt_values(vals):
    """Bar labels: 3 significant digits, with one shared decimal count when values are alike."""
    vals = [float(v) for v in vals]
    if any(abs(v) >= 1e5 or 0 < abs(v) < 1e-3 for v in vals):
        return [_sci(f'{v:.2e}') for v in vals]
    ds = [len(s.split('.')[1]) if '.' in s and 'e' not in s else 0
          for s in (f'{v:.3g}' for v in vals)]
    if ds and max(ds) - min(ds) <= 1:
        return [f'{v:.{max(ds)}f}' for v in vals]
    return [f'{v:.{d}f}' for v, d in zip(vals, ds)]


def make_transform(lo, hi, p0, p1, log=False, invert=False):
    """Return f(value) -> pixel, mapping [lo, hi] onto p0..p1 (p0 at lo unless `invert`)."""
    a, b = (math.log10(lo), math.log10(hi)) if log else (float(lo), float(hi))
    span = (b - a) or 1.0

    def f(v):
        v = np.asarray(v, float)
        with np.errstate(all='ignore'):
            t = ((np.log10(v) if log else v) - a)/span
        return p0 + (1.0 - t if invert else t)*(p1 - p0)
    return f


# ------------------------------------------------------------------- drawing

def _rgb(c, fallback='#000000'):
    try:
        return ImageColor.getrgb(c)[:3]
    except (ValueError, TypeError, AttributeError):
        return ImageColor.getrgb(fallback)[:3]


def _load_font(px, bold):
    try:
        return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, px)
    except OSError:
        try:
            return ImageFont.load_default(px)
        except TypeError:
            return ImageFont.load_default()


class _Canvas:
    """Supersampled RGB canvas. Methods take output pixels; `.d` draws in SS pixels."""

    def __init__(self, size):
        self.w, self.h = int(size[0]), int(size[1])
        if self.w < 160 or self.h < 120:
            raise ValueError('size must be at least 160 x 120')
        self.img = Image.new('RGB', (self.w*SS, self.h*SS), 'white')
        self.d = ImageDraw.Draw(self.img, 'RGBA')
        self._fonts = {}

    def font(self, px, bold=False):
        key = (int(round(px*SS)), bool(bold))
        if key not in self._fonts:
            self._fonts[key] = _load_font(*key)
        return self._fonts[key]

    def _bbox(self, s, f, x=0, y=0, anchor='ls'):
        return self.d.textbbox((x, y), s, font=f, anchor=anchor)

    def width(self, s, f):
        l, _, r, _ = self._bbox(s, f)
        return (r - l)/SS

    def cap(self, f):
        """Cap height."""
        return -self._bbox('H', f)[1]/SS

    def th(self, f):
        """Cap top to descender bottom."""
        _, t, _, b = self._bbox('Hg', f)
        return (b - t)/SS

    def box(self, x, y, s, f, anchor='ls', bounds=None):
        """Ink box of `s` anchored at (x, y), shifted to lie inside `bounds`."""
        bx0, by0, bx1, by1 = bounds or (2, 2, self.w - 2, self.h - 2)
        l, t, r, b = (v/SS for v in self._bbox(s, f, x*SS, y*SS, anchor))
        dx = dy = 0.0
        if r > bx1:
            dx = bx1 - r
        if l + dx < bx0:
            dx = bx0 - l
        if b > by1:
            dy = by1 - b
        if t + dy < by0:
            dy = by0 - t
        return (l + dx, t + dy, r + dx, b + dy), (dx, dy)

    def text(self, x, y, s, f, fill, anchor='ls', bounds=None):
        (l, t, r, b), (dx, dy) = self.box(x, y, s, f, anchor, bounds)
        self.d.text(((x + dx)*SS, (y + dy)*SS), s, font=f, fill=fill, anchor=anchor)
        return l, t, r, b

    def vtext(self, cx, y, s, f, fill, valign='m', bounds=None):
        """Text rotated to read bottom-to-top, centred on cx; y is its centre, bottom or top."""
        l, t, r, b = self._bbox(s, f)
        m = Image.new('L', (int(r - l) + 4, int(b - t) + 4), 0)
        ImageDraw.Draw(m).text((2 - l, 2 - t), s, font=f, fill=255, anchor='ls')
        m = m.rotate(90, expand=True)
        mw, mh = m.size
        bx0, by0, bx1, by1 = (v*SS for v in (bounds or (2, 2, self.w - 2, self.h - 2)))
        x0 = cx*SS - mw/2
        y0 = {'m': y*SS - mh/2, 'b': y*SS - mh, 't': y*SS}[valign]
        x0 = max(bx0, min(x0, bx1 - mw))
        y0 = max(by0, min(y0, by1 - mh))
        self.img.paste(Image.new('RGB', m.size, _rgb(fill)), (int(round(x0)), int(round(y0))), m)
        return x0/SS, y0/SS, (x0 + mw)/SS, (y0 + mh)/SS

    def output(self):
        return self.img.resize((self.w, self.h), Image.Resampling.LANCZOS)


def _save(im, path):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    im.save(p, 'PNG')
    return path


def _ellipsize(cv, s, f, maxw):
    if cv.width(s, f) <= maxw:
        return s
    s = s.rstrip('.')
    while len(s) > 1 and cv.width(s + '...', f) > maxw:
        s = s[:-1]
    return s.rstrip() + '...'


def _fit(cv, s, size, bold, maxw, min_size=8):
    """Font and string that fit in maxw: shrink first, then cut with an ellipsis."""
    s = str(s).replace('\n', ' ')
    sz = size
    f = cv.font(sz, bold)
    while cv.width(s, f) > maxw and sz > min_size:
        sz -= 0.5
        f = cv.font(sz, bold)
    return f, _ellipsize(cv, s, f, maxw)


def _wrap(cv, s, size, bold, maxw, max_lines=3, min_size=8):
    """Font and lines for `s` wrapped on spaces into maxw; shrink, then cut, when needed."""
    words = str(s).split() or ['']
    sz = size
    while True:
        f = cv.font(sz, bold)
        lines, cur = [], ''
        for w in words:
            t = (cur + ' ' + w).strip()
            if cur and cv.width(t, f) > maxw:
                lines.append(cur)
                cur = w
            else:
                cur = t
        lines.append(cur)
        ok = len(lines) <= max_lines and all(cv.width(x, f) <= maxw for x in lines)
        if ok or sz <= min_size:
            break
        sz -= 0.5
    if not ok:
        over = len(lines) > max_lines
        lines = lines[:max_lines]
        if over:
            lines[-1] += '...'
        lines = [_ellipsize(cv, x, f, maxw) for x in lines]
    return f, lines


def _header(cv, title, subtitle):
    """Draw title and subtitle top-left; return the y where the plot may start."""
    ft, t = _fit(cv, title, 21, True, cv.w - 36, 11)
    y = 14 + cv.cap(ft)
    cv.text(18, y, t, ft, INK)
    y += cv.th(ft) - cv.cap(ft)
    if subtitle:
        fs, s = _fit(cv, subtitle, 13.5, False, cv.w - 36, 9)
        y += 5 + cv.cap(fs)
        cv.text(18, y, s, fs, GREY)
        y += cv.th(fs) - cv.cap(fs)
    return y + 16


def _runs(ok):
    """(start, stop) index pairs of contiguous True runs."""
    if not ok.any():
        return []
    d = np.diff(np.concatenate(([0], ok.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def _decimate(px, py):
    """Drop consecutive points that land on the same half pixel; return a tuple list."""
    p = np.column_stack((px, py))
    q = np.round(p*2)
    keep = np.ones(len(p), bool)
    keep[1:] = np.any(q[1:] != q[:-1], axis=1)
    keep[-1] = True
    return [tuple(v) for v in p[keep].tolist()]


def _dash_runs(pts, on, off):
    """The 'on' pieces (point lists) of a dash pattern walked along a polyline."""
    pieces, cur, drawing, left = [], [], True, on
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg == 0:
            continue
        if drawing and not cur:
            cur = [(x0, y0)]
        t = 0.0
        while seg - t > left:
            t += left
            p = (x0 + (x1 - x0)*t/seg, y0 + (y1 - y0)*t/seg)
            if drawing:
                pieces.append(cur + [p])
                cur, drawing, left = [], False, off
            else:
                cur, drawing, left = [p], True, on
        left -= seg - t
        if drawing:
            cur.append((x1, y1))
    if len(cur) > 1:
        pieces.append(cur)
    return pieces


def _stroke(ld, px, py, col, w, dash):
    """Polyline through finite (px, py), broken at NaN/inf; w in SS pixels."""
    for a, b in _runs(np.isfinite(px) & np.isfinite(py)):
        pts = _decimate(px[a:b], py[a:b])
        if len(pts) < 2:
            x, y = pts[0]
            ld.ellipse([x - w/2, y - w/2, x + w/2, y + w/2], fill=col)
            continue
        for piece in (_dash_runs(pts, 8*SS, 5*SS) if dash else [pts]):
            if len(piece) > 1:
                ld.line(piece, fill=col, width=max(1, int(round(w))), joint='curve')


def _markers(ld, px, py, col, r):
    ok = np.isfinite(px) & np.isfinite(py)
    for x, y in zip(px[ok].tolist(), py[ok].tolist()):
        ld.ellipse([x - r, y - r, x + r, y + r], fill=col)


def _px(f, v):
    """Transform to SS pixels, clipped to a sane range (NaN is kept)."""
    return np.clip(f(v)*SS, -1e5, 1e5)


# -------------------------------------------------------------------- legend

def _swatch(cv, x, yc, style, col, band):
    d, x0, x1, y = cv.d, x*SS, (x + SWATCH)*SS, yc*SS
    if band or style == 'bar':
        d.rectangle([x0, y - 6*SS, x1, y + 6*SS], fill=col + ((255,) if style == 'bar' else (60,)))
    if style in ('line', 'line+points'):
        d.line([(x0, y), (x1, y)], fill=col, width=int(2.2*SS))
    elif style == 'dash':
        for p in _dash_runs([(x0, y), (x1, y)], 6*SS, 4*SS):
            d.line(p, fill=col, width=int(2.2*SS))
    if style in ('points', 'line+points'):
        xm = (x0 + x1)/2
        d.ellipse([xm - 3*SS, y - 3*SS, xm + 3*SS, y + 3*SS], fill=col)


def _legend_plan(cv, items, mode, avail):
    """Measure a legend: a column ('right', labels wrapped) or flowing rows ('bottom')."""
    f = cv.font(12.5)
    lh = cv.th(f)*1.18
    if mode == 'right':
        maxt = max(70, min(cv.w*0.24, max(cv.width(it['label'], f) for it in items) + 1))
        ents, h = [], 0
        for it in items:
            fl, lines = _wrap(cv, it['label'], 12.5, False, maxt, 3, 9)
            eh = max(len(lines)*lh, 14)
            ents.append((it, fl, lines, h))
            h += eh + 8
        w = SWATCH + 8 + max(cv.width(x, e[1]) for e in ents for x in e[2])
        return dict(mode=mode, ents=ents, w=w, h=h - 8, lh=lh)
    rows, row, x = [], [], 0.0
    for it in items:
        fl, t = _fit(cv, it['label'], 12.5, False, avail - SWATCH - 8, 9)
        ew = SWATCH + 8 + cv.width(t, fl)
        if row and x + ew > avail:
            rows.append(row)
            row, x = [], 0.0
        row.append((it, fl, t, x))
        x += ew + 22
    rows.append(row)
    rh = max(cv.th(f), 14) + 8
    return dict(mode=mode, rows=rows, w=avail, h=len(rows)*rh - 8, rh=rh)


def _legend_draw(cv, plan, x, y):
    """Draw a planned legend with its top-left at (x, y); return its rectangle."""
    if plan['mode'] == 'right':
        for it, f, lines, oy in plan['ents']:
            cap = cv.cap(f)
            _swatch(cv, x, y + oy + cap/2, it['style'], it['col'], it['band'])
            for i, ln in enumerate(lines):
                cv.text(x + SWATCH + 8, y + oy + cap + i*plan['lh'], ln, f, INK)
    else:
        for r, row in enumerate(plan['rows']):
            for it, f, t, ox in row:
                base = y + r*plan['rh'] + cv.cap(f)
                _swatch(cv, x + ox, base - cv.cap(f)/2, it['style'], it['col'], it['band'])
                cv.text(x + ox + SWATCH + 8, base, t, f, INK)
    return x, y, x + plan['w'], y + plan['h']


# --------------------------------------------------------------- shared parts

def _limits(user, vals, log, pad=0.03):
    """Axis limits: user values where given and valid, otherwise nice limits of the data."""
    v = np.asarray(vals, float).ravel()
    v = v[np.isfinite(v) & ((v > 0) if log else True)]
    lo, hi = nice_limits(v.min(), v.max(), 6, log, pad) if v.size else ((1.0, 10.0) if log else (0.0, 1.0))
    if user is not None:
        for i, u in enumerate(user):
            if u is not None and math.isfinite(u) and (u > 0 or not log):
                lo, hi = (float(u), hi) if i == 0 else (lo, float(u))
    if hi < lo:
        lo, hi = hi, lo
    if hi == lo:
        hi = lo*10 if log else lo + 1.0
    return lo, hi


def _axis_ticks(lo, hi, target, log):
    """(major, minor, labelled values, label strings); log axes with < 2 decades label minors."""
    target = int(min(max(target, 3), 10))
    major = nice_ticks(lo, hi, target, log)
    if not log:
        return major, [], major, fmt_ticks(major)
    minor = log_minor_ticks(lo, hi) if hi/lo <= 1e12 else []
    lab = sorted(major + minor) if len(major) < 2 else major
    return major, minor, lab, fmt_ticks(lab, True)


def _refs(refs, log):
    """Normalise (value, label[, color]) reference lines; drop unusable values."""
    out = []
    for r in refs or ():
        try:
            v = float(r[0])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(v) and (v > 0 or not log):
            out.append((v, str(r[1]) if len(r) > 1 else '', _rgb(r[2] if len(r) > 2 else None, REF)))
    return out


def _free(box, placed):
    return all(box[2] <= p[0] or box[0] >= p[2] or box[3] <= p[1] or box[1] >= p[3] for p in placed)


def _ref_label(cv, rect, cands, label, col, placed, maxw):
    """Place a reference-line label (fitted to maxw) at the best candidate (x, y, anchor).

    Best is: inside the plot without being shifted and clear of earlier labels; then
    merely clear of earlier labels; then the first candidate.
    """
    f, label = _fit(cv, label, 11.5, False, maxw, 8)
    L, T, R, B = rect
    bounds = (L + 2, T + 2, R - 2, B - 2)
    scored = []
    for x, y, anc in cands:
        box, shift = cv.box(x, y, label, f, anc, bounds)
        scored.append((_free(box, placed) + (shift == (0, 0)) * 2, -len(scored), x, y, anc, box))
    _, _, x, y, anc, box = max(scored, key=lambda s: s[:2])
    pad = 2
    cv.d.rectangle([(box[0] - pad)*SS, (box[1] - pad)*SS, (box[2] + pad)*SS, (box[3] + pad)*SS],
                   fill=(255, 255, 255, 215))
    cv.text(x, y, label, f, col, anc, bounds)
    placed.append(box)


def _draw_refs(cv, rect, hl, vl, tx, ty, xr, yr):
    """Labels for reference lines (the dashes themselves go on the plot layer)."""
    L, T, R, B = rect
    placed = []
    for v, label, col in hl:
        if label and yr[0] <= v <= yr[1]:
            y = T + float(ty(v))
            _ref_label(cv, rect, [(R - 6, y - 4, 'rs'), (R - 6, y + 4, 'rt'),
                                  (L + 6, y - 4, 'ls'), (L + 6, y + 4, 'lt')], label, col, placed,
                       R - L - 16)
    for v, label, col in vl:
        if label and xr[0] <= v <= xr[1]:
            x = L + float(tx(v))
            c = []
            for k in range(6):
                c += [(x + 5, T + 6 + k*18, 'lt'), (x - 5, T + 6 + k*18, 'rt')]
            _ref_label(cv, rect, c, label, col, placed, max(x - L, R - x) - 12)


def _ref_dashes(ld, pw, ph, hl, vl, tx, ty, xr, yr):
    for v, _, col in hl:
        if yr[0] <= v <= yr[1]:
            y = float(ty(v))*SS
            for p in _dash_runs([(0, y), (pw*SS, y)], 7*SS, 4*SS):
                ld.line(p, fill=col, width=int(1.6*SS))
    for v, _, col in vl:
        if xr[0] <= v <= xr[1]:
            x = float(tx(v))*SS
            for p in _dash_runs([(x, 0), (x, ph*SS)], 7*SS, 4*SS):
                ld.line(p, fill=col, width=int(1.6*SS))


def _thin(positions, widths, gap=8):
    """Smallest stride so labels at the given pixel centres do not touch; returns kept indices."""
    order = sorted(range(len(positions)), key=lambda i: positions[i])
    for k in range(1, max(len(order), 2)):
        kept = order[::k]
        if all(positions[b] - positions[a] >= (widths[a] + widths[b])/2 + gap
               for a, b in zip(kept[:-1], kept[1:])):
            return kept
    return order[:1]


def _yaxis(cv, rect, ty, ticks, labs, strs, ylabel, f_tick, f_lab):
    L, T, R, B = rect
    cap = cv.cap(f_tick)
    for v in ticks:
        y = T + float(ty(v))
        cv.d.line([(L - 5)*SS, y*SS, L*SS, y*SS], fill=_rgb(FRAME), width=SS)
    for v, s in zip(labs, strs):
        cv.text(L - 9, T + float(ty(v)) + cap/2, s, f_tick, INK, 'rs')
    if ylabel:
        f, s = _fit(cv, ylabel, 14, False, B - T, 9)
        cv.vtext(14 + cv.th(f)/2, (T + B)/2, s, f, INK, 'm')


def _frame(cv, rect):
    L, T, R, B = rect
    cv.d.rectangle([L*SS, T*SS, R*SS, B*SS], outline=_rgb(FRAME), width=SS)


def _grid_y(ld, pw, ph, ty, major, minor):
    for v in minor:
        y = round(float(ty(v))*SS)
        ld.line([(0, y), (pw*SS, y)], fill=_rgb(GRID_MINOR), width=SS)
    for v in major:
        y = round(float(ty(v))*SS)
        ld.line([(0, y), (pw*SS, y)], fill=_rgb(GRID), width=SS)


def _grid_x(ld, pw, ph, tx, major, minor):
    for v in minor:
        x = round(float(tx(v))*SS)
        ld.line([(x, 0), (x, ph*SS)], fill=_rgb(GRID_MINOR), width=SS)
    for v in major:
        x = round(float(tx(v))*SS)
        ld.line([(x, 0), (x, ph*SS)], fill=_rgb(GRID), width=SS)


def _legend_mode(legend, items):
    return legend if legend in ('right', 'bottom') and items else None


# ---------------------------------------------------------------- line chart

def _prep(s, i, logx, logy):
    """Clean one series dict: float arrays of equal length, unusable values -> NaN."""
    y = np.asarray(s.get('y', []), float).ravel()
    x = np.asarray(s['x'], float).ravel() if s.get('x') is not None else np.arange(len(y), dtype=float)
    n = min(len(x), len(y))
    x, y = x[:n], y[:n]

    def clean(a, log):
        with np.errstate(invalid='ignore'):
            return np.where(np.isfinite(a) & ((a > 0) if log else True), a, np.nan)

    band = None
    if s.get('band') is not None:
        lo, hi = (np.full(n, np.nan) for _ in range(2))
        for dst, src in zip((lo, hi), s['band']):
            a = np.asarray(src, float).ravel()[:n]
            dst[:len(a)] = a
        band = (np.where(np.isfinite(lo), lo, np.nan), clean(hi, logy))
    style = s.get('style', 'line')
    return dict(x=clean(x, logx), y=clean(y, logy), band=band,
                label=str(s.get('label') or ''), style=style if style in
                ('line', 'dash', 'points', 'line+points') else 'line',
                col=_rgb(s.get('color'), PALETTE[i % len(PALETTE)]),
                width=float(s.get('width') or 2.0))


def _render_line(series, title, xlabel, ylabel, subtitle, xlim, ylim, size, logy, logx,
                 hlines, vlines, legend, invert_x):
    cv = _Canvas(size)
    W, H = cv.w, cv.h
    ser = [_prep(s, i, logx, logy) for i, s in enumerate(series)]
    hl, vl = _refs(hlines, logy), _refs(vlines, logx)
    xv = [s['x'] for s in ser] + [np.array([v for v, _, _ in vl])]
    yv = ([s['y'] for s in ser] + [b for s in ser if s['band'] for b in s['band']]
          + [np.array([v for v, _, _ in hl])])
    xr = _limits(xlim, np.concatenate(xv), logx)
    yr = _limits(ylim, np.concatenate(yv), logy)

    f_tick, f_lab = cv.font(12), cv.font(14)
    top = _header(cv, title, subtitle)
    items = [dict(label=s['label'], col=s['col'], style=s['style'], band=s['band'] is not None)
             for s in ser if s['label']]
    mode = _legend_mode(legend, items)
    bottom0 = 9 + cv.th(f_tick) + (8 + cv.th(f_lab) if xlabel else 0) + 12
    plan = _legend_plan(cv, items, 'right', 0) if mode == 'right' else None
    if plan and plan['h'] > H - top - bottom0 - 40:
        mode, plan = 'bottom', None                     # too many entries for a side column
    right = 16 + plan['w'] + 20 if plan else 18
    if mode == 'bottom':
        bottom0 += _legend_plan(cv, items, 'bottom', W - 120)['h'] + 14

    # Provisional geometry only sets the tick density; ticks are then fixed.
    pw0, ph0 = W - 90 - right, H - top - bottom0
    xt = _axis_ticks(xr[0], xr[1], pw0/90, logx)
    yt = _axis_ticks(yr[0], yr[1], ph0/55, logy)
    xw = [cv.width(s, f_tick) for s in xt[3]] or [0]
    yw = [cv.width(s, f_tick) for s in yt[3]] or [0]
    left = max(14 + (cv.th(f_lab) + 8 if ylabel else 0) + max(yw) + 13, xw[0]/2 + 10)
    right = max(right, xw[-1]/2 + 10)
    L, R = int(round(left)), W - int(round(right))
    T = int(round(top))
    legend_h = 0
    if mode == 'bottom':
        plan = _legend_plan(cv, items, 'bottom', R - L)
        legend_h = plan['h'] + 14
    B = H - int(round(9 + cv.th(f_tick) + (8 + cv.th(f_lab) if xlabel else 0) + 12 + legend_h))
    R, B = max(R, L + 60), max(B, T + 60)
    pw, ph = R - L, B - T
    rect = (L, T, R, B)

    tx = make_transform(xr[0], xr[1], 0, pw, logx, invert_x)
    ty = make_transform(yr[0], yr[1], ph, 0, logy)
    layer = Image.new('RGB', (pw*SS, ph*SS), 'white')
    ld = ImageDraw.Draw(layer, 'RGBA')
    _grid_y(ld, pw, ph, ty, yt[0], yt[1])
    _grid_x(ld, pw, ph, tx, xt[0], xt[1])

    for s in ser:                                       # bands first so they sit behind every line
        if not s['band']:
            continue
        lo, hi = s['band']
        if logy:
            lo = np.where(lo <= 0, yr[0]/100, lo)       # non-positive lower edge runs off the bottom
        px, pl, ph_ = _px(tx, s['x']), _px(ty, lo), _px(ty, hi)
        for a, b in _runs(np.isfinite(px) & np.isfinite(pl) & np.isfinite(ph_)):
            if b - a > 1:
                poly = (list(zip(px[a:b].tolist(), ph_[a:b].tolist()))
                        + list(zip(px[a:b][::-1].tolist(), pl[a:b][::-1].tolist())))
                ld.polygon(poly, fill=s['col'] + (58,))
    _ref_dashes(ld, pw, ph, hl, vl, tx, ty, xr, yr)
    for s in ser:
        px, py = _px(tx, s['x']), _px(ty, s['y'])
        w = s['width']*SS
        if s['style'] in ('line', 'dash', 'line+points'):
            _stroke(ld, px, py, s['col'], w, s['style'] == 'dash')
        if s['style'] in ('points', 'line+points'):
            _markers(ld, px, py, s['col'], max(2.5, 1.5*s['width'])*SS)
    cv.img.paste(layer, (L*SS, T*SS))
    _frame(cv, rect)

    # x axis: ticks, labels (thinned if crowded), title
    for v in xt[0]:
        x = L + float(tx(v))
        cv.d.line([x*SS, B*SS, x*SS, (B + 5)*SS], fill=_rgb(FRAME), width=SS)
    for v in xt[1]:
        x = L + float(tx(v))
        cv.d.line([x*SS, B*SS, x*SS, (B + 3)*SS], fill=_rgb(FRAME), width=SS)
    pos = [L + float(tx(v)) for v in xt[2]]
    wid = [cv.width(s, f_tick) for s in xt[3]]
    base = B + 9 + cv.cap(f_tick)
    for i in _thin(pos, wid) if pos else []:
        cv.text(pos[i], base, xt[3][i], f_tick, INK, 'ms')
    if xlabel:
        f, s = _fit(cv, xlabel, 14, False, pw, 9)
        cv.text((L + R)/2, B + 9 + cv.th(f_tick) + 8 + cv.cap(f), s, f, INK, 'ms')
    _yaxis(cv, rect, ty, yt[0], yt[2], yt[3], ylabel, f_tick, f_lab)
    _draw_refs(cv, rect, hl, vl, tx, ty, xr, yr)
    lrect = None
    if plan:
        lrect = _legend_draw(cv, plan, R + 16, T) if mode == 'right' else \
            _legend_draw(cv, plan, L, H - legend_h + 4)
    return cv.output(), dict(rect=rect, tx=tx, ty=ty, xlim=xr, ylim=yr, legend=lrect, mode=mode)


def line_chart(path, series, title, xlabel, ylabel, subtitle='', xlim=None, ylim=None,
               size=(1100, 620), logy=False, logx=False, hlines=(), vlines=(), legend='right',
               invert_x=False):
    """Write a line chart PNG and return `path`.

    series  list of {'x', 'y', 'label', 'color', 'style', 'width', 'band'}; style is
            'line' | 'dash' | 'points' | 'line+points'; band is (y_low, y_high).
            NaN/inf (and non-positive values on a log axis) break the line.
    hlines / vlines  (value, label, color) dashed reference lines.
    legend  'right' | 'bottom' | None.  invert_x: x decreases left to right.
    """
    im, _ = _render_line(series, title, xlabel, ylabel, subtitle, xlim, ylim, size, logy, logx,
                         hlines, vlines, legend, invert_x)
    return _save(im, path)


# ----------------------------------------------------------------- bar chart

def _render_bar(categories, groups, title, ylabel, subtitle, ylim, size, hlines, logy, value_labels):
    cv = _Canvas(size)
    W, H = cv.w, cv.h
    cats = [str(c) for c in categories]
    n = max(len(cats), 1)
    gs = []
    for i, g in enumerate(groups):
        v = np.full(n, np.nan)
        a = np.asarray(g.get('values', []), float).ravel()[:n]
        v[:len(a)] = a
        with np.errstate(invalid='ignore'):
            v = np.where(np.isfinite(v) & ((v > 0) if logy else True), v, np.nan)
        gs.append(dict(v=v, label=str(g.get('label') or ''), col=_rgb(g.get('color'), PALETTE[i % len(PALETTE)])))
    hl = _refs(hlines, logy)
    allv = np.concatenate([g['v'] for g in gs] + [np.array([v for v, _, _ in hl])])
    allv = allv[np.isfinite(allv)]
    if logy:
        yr = _limits(ylim, allv*(1.8 if value_labels else 1.0), True)     # headroom for labels
    else:
        yr = _limits(ylim, np.append(allv, 0.0), False, pad=0.10)         # bars always include 0

    f_tick, f_lab = cv.font(12), cv.font(14)
    top = _header(cv, title, subtitle)
    items = [dict(label=g['label'], col=g['col'], style='bar', band=False) for g in gs if g['label']]
    mode = _legend_mode('right', items if len(gs) > 1 else [])
    plan = _legend_plan(cv, items, 'right', 0) if mode else None
    if plan and plan['h'] > H - top - 150:
        mode, plan = 'bottom', None
    right = 16 + plan['w'] + 20 if plan else 18
    yt = _axis_ticks(yr[0], yr[1], (H - top - 110)/55, logy)
    yw = [cv.width(s, f_tick) for s in yt[3]] or [0]
    L = int(round(14 + (cv.th(f_lab) + 8 if ylabel else 0) + max(yw) + 13))
    R = W - int(round(right))
    pw = max(R - L, 60)
    R = L + pw
    slot = pw/n
    wrapped = [_wrap(cv, c, 13, False, slot - 8, 3, 9) for c in cats]
    nlines = max([len(w[1]) for w in wrapped] or [1])
    lh = cv.th(cv.font(13))*1.15
    legend_h = 0
    if mode == 'bottom':
        plan = _legend_plan(cv, items, 'bottom', pw)
        legend_h = plan['h'] + 14
    T = int(round(top))
    B = max(H - int(round(10 + cv.cap(cv.font(13)) + (nlines - 1)*lh + 14 + legend_h)), T + 60)
    ph = B - T
    rect = (L, T, R, B)

    ty = make_transform(yr[0], yr[1], ph, 0, logy)
    base_v = yr[0] if logy else (0.0 if yr[0] <= 0 <= yr[1] else (yr[0] if yr[0] > 0 else yr[1]))
    yb = float(ty(base_v))
    layer = Image.new('RGB', (pw*SS, ph*SS), 'white')
    ld = ImageDraw.Draw(layer, 'RGBA')
    _grid_y(ld, pw, ph, ty, yt[0], yt[1])
    m = max(len(gs), 1)
    bar_w = slot*0.8/m
    marks = []                                          # (cx, v) per bar for labels after the paste
    for j, g in enumerate(gs):
        for i, v in enumerate(g['v']):
            cx = slot*(i + 0.5) + (j - (m - 1)/2)*bar_w
            if not np.isfinite(v):
                marks.append((cx, None, g))
                continue
            yv = float(ty(v))
            x0, x1 = cx - bar_w/2 + 0.75, cx + bar_w/2 - 0.75
            y0, y1 = sorted((yb, yv))
            ld.rectangle([x0*SS, max(y0, -5)*SS, x1*SS, min(y1, ph + 5)*SS], fill=g['col'])
            marks.append((cx, v, g))
    ld.line([(0, yb*SS), (pw*SS, yb*SS)], fill=_rgb(FRAME), width=SS)
    _ref_dashes(ld, pw, ph, hl, [], None, ty, None, yr)
    cv.img.paste(layer, (L*SS, T*SS))
    _frame(cv, rect)

    for i, (f, lines) in enumerate(wrapped):            # category labels
        for k, ln in enumerate(lines):
            cv.text(L + slot*(i + 0.5), B + 10 + cv.cap(f) + k*lh, ln, f, INK, 'ms')
    _yaxis(cv, rect, ty, yt[0], yt[2], yt[3], ylabel, f_tick, f_lab)

    present = [(cx, v) for cx, v, _ in marks if v is not None]
    strs = fmt_values([v for _, v in present]) if value_labels else []
    fs = 11.5
    while fs > 8 and max([cv.width(s, cv.font(fs)) for s in strs] or [0]) > bar_w + 2:
        fs -= 0.5
    f_val, f_na = cv.font(fs), cv.font(10.5)
    rot = max([cv.width(s, f_val) for s in strs] or [0]) > bar_w + 2   # too wide: stand them up
    inner = (L + 2, T + 2, R - 2, B - 2)
    for cx, v, _ in marks:
        if v is None:
            cv.text(L + cx, T + yb - 4, 'n/a', f_na, GREY, 'ms', inner)
    for (cx, v), s in zip(present, strs):
        yv, up = T + float(ty(v)), v >= base_v
        if rot:
            cv.vtext(L + cx, yv - 3 if up else yv + 3, s, f_val, INK, 'b' if up else 't', inner)
        else:
            cv.text(L + cx, yv - 4 if up else yv + 4 + cv.cap(f_val), s, f_val, INK, 'ms', inner)
    _draw_refs(cv, rect, hl, [], None, ty, None, yr)
    lrect = None
    if plan:
        lrect = _legend_draw(cv, plan, R + 16, T) if mode == 'right' else \
            _legend_draw(cv, plan, L, H - legend_h + 4)
    return cv.output(), dict(rect=rect, ty=ty, ylim=yr, legend=lrect, mode=mode)


def bar_chart(path, categories, groups, title, ylabel, subtitle='', ylim=None,
              size=(1100, 620), hlines=(), logy=False, value_labels=True):
    """Write a grouped vertical bar chart PNG and return `path`.

    groups  list of {'label', 'values', 'color'}, one value per category. NaN (and
            non-positive values on a log axis) are skipped and marked "n/a".
    """
    im, _ = _render_bar(categories, groups, title, ylabel, subtitle, ylim, size, hlines, logy,
                        value_labels)
    return _save(im, path)


# --------------------------------------------------------------------- panel

def _open_rgb(src):
    if isinstance(src, Image.Image):
        im = src
    else:
        with Image.open(src) as f:
            im = f.copy()
    if im.mode in ('RGBA', 'LA', 'P'):
        im = im.convert('RGBA')
        bg = Image.new('RGBA', im.size, 'white')
        return Image.alpha_composite(bg, im).convert('RGB')
    return im.convert('RGB')


def panel(path, images, columns=2, padding=12):
    """Tile existing PNG files (paths) into one white PNG, row by row; return `path`."""
    ims = [_open_rgb(p) for p in images]
    if not ims:
        raise ValueError('panel needs at least one image')
    cols = max(1, min(int(columns), len(ims)))
    rows = -(-len(ims)//cols)
    pad = int(padding)
    cw = [max(ims[i].width for i in range(c, len(ims), cols)) for c in range(cols)]
    rh = [max(im.height for im in ims[r*cols:(r + 1)*cols]) for r in range(rows)]
    out = Image.new('RGB', (pad + sum(cw) + pad*cols, pad + sum(rh) + pad*rows), 'white')
    for i, im in enumerate(ims):
        r, c = divmod(i, cols)
        x = pad + sum(cw[:c]) + pad*c + (cw[c] - im.width)//2
        y = pad + sum(rh[:r]) + pad*r + (rh[r] - im.height)//2
        out.paste(im, (x, y))
    return _save(out, path)


# ---------------------------------------------------------------------- demo

def _demo(out_path):
    h = np.linspace(300, 0, 301)
    k = (300 - h)/300
    rng = np.random.default_rng(3)
    eskf = 0.9 + 3.4*k**1.4 + 0.12*np.sin(h/9)
    spread = 0.25 + 0.9*k
    deadreck = 1.2 + 6.5*k**1.1 + 0.2*np.cos(h/7)
    deadreck[150:172] = np.nan                          # a dropout shows as a gap
    baro = 2.1 + 2.2*k + 0.15*rng.standard_normal(h.size)
    with tempfile.TemporaryDirectory() as tmp:
        a = line_chart(
            Path(tmp)/'a.png',
            [dict(x=h, y=eskf, label='ESKF mean', band=(eskf - spread, eskf + spread)),
             dict(x=h, y=baro, label='Baro-aided', style='dash', color='#009E73'),
             dict(x=h, y=deadreck, label='Dead reckoning (gap = dropout)', color='#D55E00')],
            'Position error during descent', 'Altitude (m)', 'Position error (m)',
            subtitle='Median over 200 runs, band = 1 sigma; x axis runs from 300 m down to 0 m',
            ylim=(0, None), size=(820, 520), hlines=[(5.0, 'requirement 5 m', '#B22222')],
            invert_x=True, legend='right')
        t = np.linspace(0, 60, 61)
        cov = 2e2*np.exp(-t/6.5) + 2e-3
        b = line_chart(
            Path(tmp)/'b.png',
            [dict(x=t, y=cov, label='Covariance trace', style='line+points'),
             dict(x=t, y=0.05*cov[::-1] + 0.005, label='Reversed', style='dash'),
             dict(x=t, y=np.where(t % 20 < 3, -1.0, 3*np.exp(-t/25)), label='Skipped when <= 0', style='points')],
            'Filter convergence', 'Time since release (s)', 'Variance (m^2)',
            subtitle='Log y axis: non-positive values are skipped, decades labelled',
            size=(820, 520), logy=True, vlines=[(20, 'deploy', '#4D4D4D')], legend='bottom')
        c = bar_chart(
            Path(tmp)/'c.png', ['Calm', 'Gusty wind', 'High spin rate', 'Wind shear with long label'],
            [dict(label='Baseline', values=[0.8, 1.9, 3.6, 2.4]),
             dict(label='Tuned filter', values=[0.5, 1.1, 2.0, np.nan]),
             dict(label='Tuned + baro', values=[0.4, 0.9, 1.6, 1.2])],
            'RMS position error by scenario', 'RMS error (m)',
            subtitle='Missing result shown as n/a', size=(820, 520),
            hlines=[(2.5, 'limit 2.5 m', '#B22222')])
        return panel(out_path, [a, b, c], columns=2, padding=14)


if __name__ == '__main__':
    sys.dont_write_bytecode = True
    out = Path(__file__).resolve().parent/'results'/'plotting_demo.png'
    print(_demo(out))
