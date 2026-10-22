"""GPL-3.0-only. Synthetic English-farmland world for the navigation simulations.

SYNTHETIC simulation code, not flight software. It replaces the six-sinusoid
test terrain with something that has the structure an image matcher meets over
UK farmland: irregular fields, hedgerows with gaps and trees, roads, tracks,
farm buildings, woods, ponds, tramlines, shadows, and a map/flight pair that
share the permanent structure but differ by a Season. Nothing here was fitted
to a real aerial image. It cannot validate anything about real imagery; it only
gives the matcher a harder and more relevant synthetic test than sinusoids.

How the world is built

  layout    a large rotated square is split recursively by jittered straight
            lines until each piece is below a log-normally drawn target area.
            The pieces are convex 4-6 sided fields with locally shared
            orientation. A smooth low-amplitude warp bends every boundary.
  boundary  every split line is a road, a track, a hedge, a fence/ditch margin
            or a bare crop-change line. Hedge width and height vary along the
            line and have gateways and gappy runs.
  objects   boundary trees, in-field trees, copses, whole-parcel woods, ponds,
            farmyards with gabled buildings.
  fields    mean level + soil patches (tens of metres, permanent) + per-pixel
            texture + drill stripes + tramlines (wheel-track pairs repeated at
            the sprayer width, with a headland circuit).
  light     a height raster casts shadows of length height/tan(elevation) away
            from the sun, and tall things are shaded by their slope to the sun.
  season    a fraction of fields have their mean level redrawn, a fraction have
            their tramlines redrawn, the sun moves, and a global gain/offset/
            gamma is applied to the flight raster.

Grid convention (a contract with the renderer and matcher; do not change):
raster[row, col] is the value at east = origin[0] + col*res,
north = origin[1] + row*res. North INCREASES with row. World (0, 0) is the
centre of the raster. Sun azimuth is degrees clockwise from north, towards the
sun; bearings are degrees clockwise from north in [0, 180).

Known simplifications are listed in LIMITATIONS and returned in World.meta.
Where each number comes from (named source or assumption) is in PARAMETERS.
"""
import math
from dataclasses import asdict, dataclass, field

import numpy as np

KIND_FIELD, KIND_HEDGE, KIND_TREE, KIND_ROAD, KIND_BUILDING, KIND_WATER = range(6)
KIND_NAMES = ('field', 'hedge', 'tree', 'road', 'building', 'water')

B_NONE, B_HEDGE, B_FENCE, B_ROAD, B_TRACK = range(5)

CHUNK = 256
F32 = np.float32

SYNTHETIC_STATEMENT = (
    'SYNTHETIC procedural terrain. Not derived from, fitted to, or validated '
    'against any real aerial image. Results on it say nothing certain about '
    'real imagery.')

LIMITATIONS = (
    'Fields are convex pieces of a recursive straight-line subdivision; real '
    'fields include concave shapes, curved ancient boundaries and streams.',
    'The ground is flat: height holds only hedges, trees and buildings, so '
    'there is no relief, no relief shadow and no parallax from terrain.',
    'Grey levels and contrasts are assumptions, not measured from orthophotos.',
    'Field level, tramline presence and land use are drawn independently; '
    'real pasture stays pasture and has no tramlines.',
    'A changed field keeps the same permanent soil-patch pattern (rescaled).',
    'Shadows are cast only by objects inside the raster and are a fixed '
    'fractional darkening; there is no sky-light model and no cloud shadow.',
    'Fine texture is per pixel, so its scale follows res_m.',
    'No villages, rivers, railways, pylons, livestock, vehicles or bales.',
)

# Median leaf area / split target area, measured over 30 layouts per style
# (0.572 to 0.588). Dividing by it makes Style.field_area_ha the median field.
SPLIT_TO_MEDIAN = 0.585

# (parameter, value used, basis, reference or remark). Basis is 'source' only
# when the cited page states the number; 'derived' is my arithmetic on sourced
# numbers; everything else is 'assumed'.
PARAMETERS = (
    ('tramline spacing', '24 m and 36 m dominant; 12, 18, 30 m also drawn', 'source + assumed',
     'profi.co.uk says UK tramline widths are mostly 24 m or 36 m '
     '(https://profi.co.uk/test-centre/spreading-granular-fertiliser-wider-spread-wide-gain-acres). '
     'The 12/18/30 m values and all the weights are assumed.'),
    ('tramline wheel gauge', '1.8 to 2.0 m', 'source',
     'Hardi Alpha Varitrack track width adjustable from 1.80 m to 2.25 m '
     '(https://www.fwi.co.uk/machinery/hardi-launches-a-smaller-alpha-self-propelled-with-stepless-track-width-adjustment); '
     'the brief gives 1.8 to 2 m.'),
    ('wheel-track width', 'about 0.45 m (Gaussian, sigma 0.19 m)', 'assumed',
     'The same article quotes 270 mm row-crop tyres; the visible rut is assumed wider.'),
    ('tramline contrast', 'median 22 grey levels (42 in the tramline style), 80 % darker than crop', 'assumed', ''),
    ('tramline headland', 'one circuit half a spacing inside the boundary', 'assumed', ''),
    ('hedge width', 'median 2.0 m (2.5 m pasture), log-normal sd 0.3', 'source + assumed',
     'Countryside Survey 2000: about 68 % of hedges 1 to 2 m wide '
     '(https://nora.nerc.ac.uk/id/eprint/4798/1/M03_hedgerows_final_report.pdf). Spread assumed.'),
    ('hedge height', 'median 2.2 m (2.6 m pasture), log-normal sd 0.3', 'source + assumed',
     'UKCEH/Natural England 2007-2023 comparison: most hedges now taller than 2 m '
     '(reported via https://hedgelink.org.uk/?p=5058). Spread assumed.'),
    ('hedge gaps', 'about 10 % of hedge length missing, plus gateways every ~220 m', 'source + assumed',
     'Countryside Survey 2000: gaps very rarely exceed 10 % of length '
     '(https://nora.nerc.ac.uk/id/eprint/4798/1/M03_hedgerows_final_report.pdf). The open and tramline styles '
     'deliberately use gappier hedges. '
     'Gateway spacing and the gappy-run model are assumed.'),
    ('boundary trees', '0.4 to 2.2 per 100 m of hedge, crown diameter 4 to 12 m, height 9 to 19 m', 'assumed',
     'Crown range from the brief; no density source found.'),
    ('field area', 'median 9 ha arable, 4 ha mixed, 2 ha pasture, 30 ha open, 10 ha tramline', 'assumed',
     'No published mean field size found. Crude anchor (derived, not sourced): about 2.6 million RPA land '
     'parcels in England (https://www.data.gov.uk/dataset/ed6f8d0e-df4f-4027-b4e3-ee43d7cebf35/rpa-parcel-points-england) '
     'over roughly 9 million ha of farmland gives about 3.5 ha per parcel. Regional contrast is supported only '
     'qualitatively: East of England is 80 % arable, the west mostly pasture '
     '(https://www.gov.uk/government/statistics/agricultural-facts-england-regional-profiles/agricultural-facts-summary).'),
    ('field shape', 'convex, almost all 4-sided, split jitter 4 to 10 deg, boundary warp 3 to 7 m', 'assumed', ''),
    ('boundary type mix', 'hedge / fence-or-ditch margin / bare crop line, per style', 'assumed', ''),
    ('road and track width', 'lane 3.2 to 5.5 m, track 2.4 to 3.6 m, verge 1 to 2.5 m', 'assumed', ''),
    ('buildings', '12-32 m by 8-18 m, eaves 4-6.5 m, ridge +1.5-3 m, yard 50-75 m by 38-50 m', 'assumed', ''),
    ('woods and ponds', 'copse radius 15-45 m, canopy about 14 m; pond radius 5-16 m', 'assumed', ''),
    ('grey levels', 'fields 92-172 by cover, hedge 58, tree 55, wood 52, road 120, track 168 or 88, '
     'roof 185 or 75, yard 150, water 32, verge 108', 'assumed',
     'No orthophoto radiometry source found. These are guesses on a 0..255 scale.'),
    ('within-field variation', 'patches sd 6 at 18 and 60 m scales, per-pixel texture sd 3.5, drill stripes sd 1.5',
     'assumed', ''),
    ('shadow depth', '55 % darkening in full shadow, 0.5 m soft edge', 'assumed', ''),
    ('sun positions', 'map (180, 55) deg, flight (150, 35) deg', 'assumed', 'Defaults from the brief.'),
)


@dataclass
class Season:
    """How flight day differs from the day the map image was taken."""
    albedo_change: float = 0.7     # fraction of fields whose mean level is redrawn independently
    tramline_change: float = 1.0   # fraction of fields whose tramline pattern is redrawn
    sun_map: tuple = (180.0, 55.0)     # azimuth, elevation (deg), summer survey
    sun_flight: tuple = (150.0, 35.0)  # March, late morning
    gain: float = 1.0
    offset: float = 0.0
    gamma: float = 1.0
    leaf_off: float = 0.0          # 0 hedges/trees as in the map, 1 fully bare on flight day
    tramline_rebearing: float = 0.1  # of the fields whose tramlines are redrawn, fraction that follow another edge


@dataclass
class Style:
    """One landscape type. Lengths in metres, areas in hectares, levels 0..255."""
    name: str = 'mixed'
    field_area_ha: float = 4.0       # median of the split target (log-normal)
    field_area_sigma: float = 0.45
    min_field_ha: float = 0.4
    split_jitter_deg: float = 7.0
    warp_m: float = 5.0              # amplitude of each boundary-warp term
    p_hedge: float = 0.6             # boundary type mix (the rest are bare crop-change lines)
    p_fence: float = 0.2
    hedge_width_m: float = 2.0
    hedge_height_m: float = 2.2
    gappy_fraction: float = 0.2      # hedges with long gaps
    gateway_spacing_m: float = 220.0
    trees_per_100m: float = 1.2      # boundary trees per 100 m of hedge
    field_trees_per_km2: float = 6.0
    n_roads: int = 1
    n_tracks: int = 2
    n_farms: int = 1
    copses_per_km2: float = 1.5
    wood_parcel_prob: float = 0.04   # small parcels that are woodland
    ponds_per_km2: float = 1.5
    tramline_fraction: float = 0.6   # fields with visible tramlines
    tramline_spacings: tuple = (12.0, 18.0, 24.0, 30.0, 36.0)
    tramline_weights: tuple = (0.1, 0.15, 0.45, 0.15, 0.15)
    tramline_contrast: float = 22.0  # median unresolved-track contrast, grey levels
    patch_sd: float = 6.0            # within-field soil/moisture patches
    fine_sd: float = 3.5             # per-pixel texture
    stripe_sd: float = 1.5           # drill-bout stripes
    shadow_depth: float = 0.55       # fractional darkening in full shadow
    # (mean, sd, weight) of field level on map day and on flight day
    map_cover: tuple = ((100, 10, .35), (172, 14, .2), (140, 12, .1), (115, 8, .25), (125, 18, .1))
    flight_cover: tuple = ((105, 10, .4), (92, 16, .2), (150, 14, .1), (115, 8, .3))


STYLES = {
    'arable': Style(name='arable', field_area_ha=9.0, p_hedge=0.5, p_fence=0.2,
                    trees_per_100m=0.8, n_tracks=2, tramline_fraction=0.85,
                    wood_parcel_prob=0.03, copses_per_km2=1.0, ponds_per_km2=1.0,
                    map_cover=((100, 10, .45), (172, 14, .25), (140, 12, .1), (115, 8, .1), (125, 18, .1)),
                    flight_cover=((105, 10, .5), (92, 16, .25), (150, 14, .1), (115, 8, .15))),
    'mixed': Style(name='mixed'),
    'pasture': Style(name='pasture', field_area_ha=2.0, min_field_ha=0.25, split_jitter_deg=10.0,
                     warp_m=7.0, p_hedge=0.8, p_fence=0.15, hedge_width_m=2.5, hedge_height_m=2.6,
                     trees_per_100m=2.2, field_trees_per_km2=20.0, n_tracks=3, n_farms=2,
                     copses_per_km2=3.0, wood_parcel_prob=0.06, ponds_per_km2=3.0,
                     tramline_fraction=0.15, patch_sd=7.0,
                     map_cover=((115, 9, .7), (160, 12, .15), (100, 10, .1), (125, 18, .05)),
                     flight_cover=((115, 9, .8), (105, 10, .12), (92, 16, .08))),
    'open': Style(name='open', field_area_ha=30.0, min_field_ha=4.0, split_jitter_deg=4.0,
                  warp_m=3.0, p_hedge=0.15, p_fence=0.25, gappy_fraction=0.6,
                  trees_per_100m=0.4, field_trees_per_km2=1.0, n_tracks=1, n_farms=0,
                  copses_per_km2=0.4, wood_parcel_prob=0.0, ponds_per_km2=0.4,
                  tramline_fraction=0.9, tramline_weights=(0.0, 0.05, 0.45, 0.2, 0.3),
                  map_cover=((100, 10, .5), (172, 14, .3), (140, 12, .1), (125, 18, .1)),
                  flight_cover=((105, 10, .6), (92, 16, .3), (150, 14, .1))),
    'tramline': Style(name='tramline', field_area_ha=10.0, min_field_ha=1.5, p_hedge=0.2, p_fence=0.3,
                      gappy_fraction=0.5, trees_per_100m=0.4, field_trees_per_km2=2.0,
                      n_tracks=1, n_farms=0, copses_per_km2=0.5, wood_parcel_prob=0.0,
                      ponds_per_km2=0.5, tramline_fraction=1.0, tramline_contrast=42.0,
                      tramline_weights=(0.15, 0.2, 0.45, 0.1, 0.1), patch_sd=4.0,
                      map_cover=((104, 6, .7), (112, 6, .3)),
                      flight_cover=((104, 6, .7), (112, 6, .3))),
}


@dataclass
class World:
    res: float
    origin: tuple
    map: np.ndarray
    flight: np.ndarray
    height: np.ndarray
    field_id: np.ndarray
    kind: np.ndarray
    fields: np.ndarray
    meta: dict = field(default_factory=dict)

    def axes(self):
        """East and north coordinates (m) of the pixel centres."""
        rows, cols = self.map.shape
        return (self.origin[0] + self.res*np.arange(cols),
                self.origin[1] + self.res*np.arange(rows))


FIELD_DTYPE = np.dtype([
    ('map_mean', 'f4'), ('flight_mean', 'f4'),
    ('tram_bearing_deg', 'f4'), ('tram_spacing_m', 'f4'), ('tram_gauge_m', 'f4'),
    ('tram_contrast_map', 'f4'), ('tram_contrast_flight', 'f4'),
    ('tram_phase_map_m', 'f4'), ('tram_phase_flight_m', 'f4'),
    ('tram_bearing_flight_deg', 'f4'), ('tram_spacing_flight_m', 'f4'),
    ('area_m2', 'f4'), ('area_full_m2', 'f4'), ('truncated', '?'),
    ('east', 'f4'), ('north', 'f4'),
    ('albedo_changed', '?'), ('tram_changed', '?'), ('changed', '?')])


# ---------------------------------------------------------------- helpers

def _warp_terms(rng, amp):
    terms = []
    for _ in range(2):
        axis = []
        for i in range(3):
            lam = rng.uniform(350.0, 900.0)
            th = rng.uniform(0.0, 2*math.pi)
            a = amp*rng.uniform(0.5, 1.0)/math.sqrt(i + 1.0)
            axis.append((a, 2*math.pi*math.cos(th)/lam, 2*math.pi*math.sin(th)/lam,
                         rng.uniform(0.0, 2*math.pi)))
        terms.append(axis)
    return terms


def _warp(terms, x, y):
    out = []
    for axis in terms:
        w = 0.0
        for a, kx, ky, ph in axis:
            w = w + F32(a)*np.sin(F32(kx)*x + F32(ky)*y + F32(ph))
        out.append(w)
    return out


def _unwarp(terms, u, v):
    """World point whose warped coordinates are (u, v)."""
    x, y = float(u), float(v)
    for _ in range(8):
        wx, wy = _warp(terms, np.float64(x), np.float64(y))
        x, y = u - float(wx), v - float(wy)
    return x, y


def _value_noise(rng, n, cell_px):
    """Smooth unit-variance noise with features about cell_px across."""
    m = int(math.ceil(n/cell_px)) + 4
    g = rng.standard_normal((m, m), dtype=F32)
    x = np.arange(n, dtype=np.float64)/cell_px
    i = x.astype(np.intp)
    f = x - i
    # cubic B-spline weights: smooth, with no visible lattice
    w = [((1 - f)**3/6).astype(F32), ((3*f**3 - 6*f**2 + 4)/6).astype(F32),
         ((-3*f**3 + 3*f**2 + 3*f + 1)/6).astype(F32), (f**3/6).astype(F32)]
    rows = sum(g[i + k]*w[k][:, None] for k in range(4))
    out = rows[:, i]*w[0][None, :]
    for k in range(1, 4):
        out += rows[:, i + k]*w[k][None, :]
    out -= out.mean()
    out /= out.std()
    return out


def _fine_noise(rng, n):
    a = rng.standard_normal((n, n), dtype=F32)
    a[1:] += F32(0.5)*a[:-1]
    a[:, 1:] += F32(0.5)*a[:, :-1]
    a *= F32(1/1.25)
    return a


def _box1d(a, w):
    """Box filter of width w samples along the last axis (edge replicated)."""
    w = max(1, int(w))
    if w == 1:
        return a
    pad = np.concatenate((np.repeat(a[..., :1], w//2, -1), a,
                          np.repeat(a[..., -1:], w - w//2, -1)), -1)
    c = np.cumsum(pad, -1, dtype=np.float64)
    return ((c[..., w:] - c[..., :-w])/w).astype(F32)


def _poly_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5*float(np.sum(x*np.roll(y, -1) - y*np.roll(x, -1)))


def _clip(poly, eids, p0, n, sid):
    """Part of a convex polygon with (x - p0).n >= 0; the cut edge gets id sid."""
    d = (poly - p0) @ n
    out, oe = [], []
    m = len(poly)
    for i in range(m):
        j = (i + 1) % m
        if d[i] >= 0:
            out.append(poly[i])
            oe.append(eids[i])
            if d[j] < 0:
                out.append(poly[i] + (poly[j] - poly[i])*(d[i]/(d[i] - d[j])))
                oe.append(sid)
        elif d[j] >= 0:
            out.append(poly[i] + (poly[j] - poly[i])*(d[i]/(d[i] - d[j])))
            oe.append(eids[i])
    return np.array(out), oe


def _layout(rng, st, half, size_m):
    """Recursive subdivision. Returns leaves [(poly, edge ids)] and split records."""
    th = rng.uniform(0.0, math.pi/2)
    c, s = math.cos(th), math.sin(th)
    rot = np.array([[c, -s], [s, c]])
    root = half*np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]) @ rot.T
    splits = [dict(p0=np.zeros(2), t=np.array([1., 0.]), a=root[0], b=root[0], depth=-1)]
    leaves = []
    stack = [(root, [0, 0, 0, 0], 0)]
    jit = math.radians(st.split_jitter_deg)
    while stack:
        poly, eids, depth = stack.pop()
        area = _poly_area(poly)
        target = st.field_area_ha*1e4/SPLIT_TO_MEDIAN*math.exp(st.field_area_sigma*rng.standard_normal())
        # the first two cuts (the roads) pass within 0.3 of the raster size of the centre
        f = rng.uniform(0.36, 0.64) if depth > 1 else 0.5 + rng.uniform(-0.3, 0.3)*size_m/(2*half)
        ang = float(np.clip(jit*rng.standard_normal(), -0.4, 0.4))
        if depth > 0 and (area <= target or depth > 24):
            leaves.append((poly, eids))
            continue
        e = np.roll(poly, -1, 0) - poly
        length = np.hypot(e[:, 0], e[:, 1])
        a = e[int(np.argmax(length))]/length.max()
        perp = np.array([-a[1], a[0]])
        if np.ptp(poly @ perp) > np.ptp(poly @ a):
            a = perp
        ca, sa = math.cos(ang), math.sin(ang)
        nrm = np.array([ca*a[0] - sa*a[1], sa*a[0] + ca*a[1]])
        proj = poly @ a
        cen = poly.mean(0)
        p0 = cen + (proj.min() + f*np.ptp(proj) - cen @ a)*a
        sid = len(splits)
        pa, ea = _clip(poly, eids, p0, nrm, sid)
        pb, eb = _clip(poly, eids, p0, -nrm, sid)
        if depth > 0 and (len(pa) < 3 or len(pb) < 3
                          or min(_poly_area(pa), _poly_area(pb)) < st.min_field_ha*1e4):
            leaves.append((poly, eids))
            continue
        k = ea.index(sid)
        splits.append(dict(p0=p0, t=np.array([-nrm[1], nrm[0]]), a=pa[k], b=pa[(k + 1) % len(pa)],
                           depth=depth))
        stack.append((pb, eb, depth + 1))
        stack.append((pa, ea, depth + 1))
    return leaves, splits


def _window(xs, ys, x0, x1, y0, y1):
    c0, c1 = int(np.searchsorted(xs, x0)), int(np.searchsorted(xs, x1, 'right'))
    r0, r1 = int(np.searchsorted(ys, y0)), int(np.searchsorted(ys, y1, 'right'))
    if c1 <= c0 or r1 <= r0:
        return None
    return slice(r0, r1), slice(c0, c1)


def _blob(xs, ys, cx, cy, radius, lumps):
    """Window and normalised radius (<1 inside) of a lumpy disc."""
    rmax = radius*1.6
    win = _window(xs, ys, cx - rmax, cx + rmax, cy - rmax, cy + rmax)
    if win is None:
        return None, None
    x = (xs[win[1]] - cx).astype(F32)[None, :]
    y = (ys[win[0]] - cy).astype(F32)[:, None]
    r = np.hypot(x, y)
    if lumps:
        th = np.arctan2(y, x)
        scale = 1.0
        for k, (a, ph) in enumerate(lumps):
            scale = scale + F32(a)*np.sin(F32(k + 2)*th + F32(ph))
        r = r/(F32(radius)*scale)
    else:
        r = r/F32(radius)
    return win, r


def _rect(xs, ys, cx, cy, tx, ty, la, lb):
    """Window and local (along, across) coordinates of a rotated rectangle."""
    rad = 0.5*math.hypot(la, lb)
    win = _window(xs, ys, cx - rad, cx + rad, cy - rad, cy + rad)
    if win is None:
        return None, None, None
    x = (xs[win[1]] - cx).astype(F32)[None, :]
    y = (ys[win[0]] - cy).astype(F32)[:, None]
    return win, x*F32(tx) + y*F32(ty), -x*F32(ty) + y*F32(tx)


def _shifted(a, dr, dc):
    """out[r, c] = a[r + dr, c + dc], edges replicated."""
    n0, n1 = a.shape
    r = np.clip(np.arange(n0) + dr, 0, n0 - 1)
    c = np.clip(np.arange(n1) + dc, 0, n1 - 1)
    if dc == 0:
        return a[r]
    if dr == 0:
        return a[:, c]
    return a[r][:, c]


def _sun_vector(sun):
    az, el = math.radians(sun[0]), math.radians(sun[1])
    return math.sin(az), math.cos(az), math.tan(el)


def shadow_factor(height, res, sun):
    """0 in sunlight, 1 in full shadow. A point is shadowed when something
    between it and the sun rises above the sun ray: shadow length is
    height/tan(elevation), pointing away from the sun."""
    se, sn, tan_el = _sun_vector(sun)
    n0, n1 = height.shape
    occ = height.copy()
    rr, cc = np.nonzero(height > 0)
    hv = height[rr, cc]
    step = 0.7*res
    k = 1
    while len(hv):
        t = k*step
        drop = F32(t*tan_el)
        keep = hv > drop
        if not keep.all():
            rr, cc, hv = rr[keep], cc[keep], hv[keep]
            if not len(hv):
                break
        r2 = rr - int(round(t*sn/res))
        c2 = cc - int(round(t*se/res))
        ok = (r2 >= 0) & (r2 < n0) & (c2 >= 0) & (c2 < n1)
        r2, c2 = r2[ok], c2[ok]
        occ[r2, c2] = np.maximum(occ[r2, c2], hv[ok] - drop)
        k += 1
    occ -= height
    occ *= F32(1/0.5)
    np.clip(occ, 0, 1, out=occ)
    return occ


def _illuminate(rad, height, kind, res, sun, depth, relief=0.22):
    """Slope shading of tall things, then cast shadows. In place."""
    se, sn, _ = _sun_vector(sun)
    k = max(1.0, 1.0/res)
    dr, dc = int(round(k*sn)), int(round(k*se))
    if dr == 0 and dc == 0:
        dr = 1
    base = res*math.hypot(dr, dc)
    g = _shifted(height, dr, dc) - _shifted(height, -dr, -dc)
    g *= F32(1/(2*base*1.5))
    np.tanh(g, out=g)
    g *= F32(-relief)
    g += 1
    tall = (kind == KIND_HEDGE) | (kind == KIND_TREE) | (kind == KIND_BUILDING)
    np.multiply(rad, g, out=rad, where=tall)
    del g
    sh = shadow_factor(height, res, sun)
    sh *= F32(-depth)
    sh += 1
    rad *= sh


# ------------------------------------------------------------ field layer

def _field_layer(fid, xs, ys, bdist, margin, patch, fine, fine2, fp, stripe_tab, res, margin_grey):
    """Radiance of field pixels (zeros elsewhere)."""
    n = fid.shape[0]
    out = np.zeros((n, n), F32)
    if len(fp['mean']) == 0:
        return out
    x = xs.astype(F32)[None, :]
    sig_w = 0.19                      # wheel-track profile, about 0.45 m wide
    sig = math.sqrt(sig_w**2 + (0.42*res)**2)   # widened by the pixel footprint
    amp = F32(sig_w/sig)
    inv = F32(-0.5/sig**2)
    for r0 in range(0, n, CHUNK):
        sl = slice(r0, min(n, r0 + CHUNK))
        f = fid[sl]
        m = f >= 0
        fi = np.where(m, f, 0)
        y = ys[sl].astype(F32)[:, None]
        c = x*fp['nx'][fi] + y*fp['ny'][fi]
        sp = fp['sp'][fi]
        hg = F32(0.5)*fp['g'][fi]
        t = c - fp['ph'][fi] + F32(0.5)*sp
        t -= sp*np.floor(t/sp)
        t -= F32(0.5)*sp
        np.abs(t, out=t)
        t -= hg
        d = bdist[sl]
        e = np.abs(d - F32(0.5)*sp)
        e -= hg
        prof = np.where(d > F32(0.5)*sp, np.exp(inv*t*t), F32(0))
        np.maximum(prof, np.exp(inv*e*e), out=prof)
        nz = fine[sl] if fine2 is None else np.where(fp['new'][fi], fine2[sl], fine[sl])
        si = np.floor(c/fp['sw'][fi]).astype(np.intp) & 1023
        rad = (fp['mean'][fi] + fp['pamp'][fi]*patch[sl] + fp['famp'][fi]*nz
               + (amp*fp['con'][fi])*prof + fp['samp'][fi]*stripe_tab[fp['srow'][fi], si])
        mg = margin[sl]
        rad[mg] = F32(0.5)*rad[mg] + F32(0.5*margin_grey)
        out[sl] = np.where(m, rad, F32(0))
    return out


def _draw_levels(rng, cover, n):
    cover = np.asarray(cover, float)
    which = rng.choice(len(cover), size=n, p=cover[:, 2]/cover[:, 2].sum())
    return (cover[which, 0] + cover[which, 1]*rng.standard_normal(n)).astype(F32)


def _bearing(d):
    return math.degrees(math.atan2(d[0], d[1])) % 180.0


def _tram_normal(bearing_deg):
    b = np.radians(bearing_deg)
    return np.cos(b).astype(F32), (-np.sin(b)).astype(F32)


# --------------------------------------------------------------- generate

def generate(seed, size_m=1600.0, res_m=0.5, style='mixed', season=None):
    """Build a World. Deterministic in (seed, size_m, res_m, style, season).

    The layout and the map raster depend only on (seed, size_m, res_m, style):
    changing the Season changes only the flight raster and the flight columns
    of World.fields.
    """
    st = STYLES[style] if isinstance(style, str) else style
    season = Season() if season is None else season
    res = float(res_m)
    n = int(round(size_m/res))
    x0 = -(n - 1)/2*res
    xs = x0 + res*np.arange(n)
    ys = xs.copy()
    seed = int(seed)
    rng = np.random.default_rng([seed, 1])
    rng_noise = np.random.default_rng([seed, 2])
    rng_field = np.random.default_rng([seed, 3])
    rng_season = np.random.default_rng([seed, 4])

    # ---- layout in warped coordinates
    terms = _warp_terms(rng, st.warp_m)
    pad = 2.0*sum(t[0] for t in terms[0]) + 2*res
    half = 0.5*size_m + max(250.0, 2.5*math.sqrt(st.field_area_ha*1e4)) + pad
    leaves, splits = _layout(rng, st, half, size_m)
    ns_k = len(splits)
    span = half*math.sqrt(2.0) + 4.0
    ns = int(2*span) + 2

    xr = xs.astype(F32)[None, :]
    yr = ys.astype(F32)[:, None]
    wx, wy = _warp(terms, xr, yr)
    U = (xr + wx).astype(F32)
    V = (yr + wy).astype(F32)
    del wx, wy

    leaf = np.full((n, n), -1, np.int32)
    bdist = np.zeros((n, n), F32)
    bid = np.zeros((n, n), np.int16)
    for li, (poly, eids) in enumerate(leaves):
        win = _window(xs, ys, poly[:, 0].min() - pad, poly[:, 0].max() + pad,
                      poly[:, 1].min() - pad, poly[:, 1].max() + pad)
        if win is None:
            continue
        uw, vw = U[win], V[win]
        best = np.full(uw.shape, np.inf, F32)
        bestid = np.zeros(uw.shape, np.int16)
        nxt = np.roll(poly, -1, 0)
        for a, b, eid in zip(poly, nxt, eids):
            ex, ey = b - a
            ln = math.hypot(ex, ey)
            if ln < 1e-9:
                continue
            d = (uw - F32(a[0]))*F32(-ey/ln) + (vw - F32(a[1]))*F32(ex/ln)
            m = d < best
            best = np.where(m, d, best)
            bestid[m] = eid
        inside = best >= -1e-3
        if inside.any():
            leaf[win][inside] = li
            bdist[win][inside] = np.maximum(best[inside], 0)
            bid[win][inside] = bestid[inside]

    # ---- boundary attributes
    btype = np.zeros(ns_k, np.int8)
    depth = np.array([s['depth'] for s in splits])
    u3 = rng.uniform(size=ns_k)
    btype[u3 < st.p_hedge] = B_HEDGE
    btype[(u3 >= st.p_hedge) & (u3 < st.p_hedge + st.p_fence)] = B_FENCE
    btype[0] = B_NONE
    if st.n_roads >= 1 and ns_k > 1:
        btype[1] = B_ROAD
    d1 = np.nonzero(depth == 1)[0]
    if st.n_roads >= 2 and len(d1):
        btype[d1[0]] = B_ROAD
    cand = np.nonzero((depth >= 2) & (depth <= 4) & (btype != B_ROAD))[0]
    if len(cand) and st.n_tracks:
        btype[rng.choice(cand, size=min(st.n_tracks, len(cand)), replace=False)] = B_TRACK

    is_road, is_track = btype == B_ROAD, btype == B_TRACK
    road_hw = np.where(is_road, rng.uniform(1.6, 2.75, ns_k),
                       np.where(is_track, rng.uniform(1.2, 1.8, ns_k), 0.0)).astype(F32)
    verge = np.where(is_road, rng.uniform(1.0, 2.5, ns_k),
                     np.where(is_track, rng.uniform(0.3, 1.0, ns_k), 0.0)).astype(F32)
    width = st.hedge_width_m*np.exp(0.3*rng.standard_normal(ns_k))
    side_hedge = (is_road & (rng.uniform(size=ns_k) < 0.8)) | (is_track & (rng.uniform(size=ns_k) < 0.4))
    extent = np.where(btype == B_HEDGE, 0.5*width, np.where(side_hedge, width, 0.0))
    hgt = st.hedge_height_m*np.exp(0.3*rng.standard_normal(ns_k))
    margin_w = np.where(btype == B_HEDGE, rng.uniform(0.5, 2.0, ns_k),
                        np.where(btype == B_FENCE, rng.uniform(0.6, 1.4, ns_k), 0.0)).astype(F32)
    road_rad = np.where(is_road, rng.normal(120, 8, ns_k),
                        np.where(rng.uniform(size=ns_k) < 0.7, rng.normal(168, 14, ns_k),
                                 rng.normal(88, 8, ns_k))).astype(F32)
    hedge_rad = rng.normal(58, 6, ns_k).astype(F32)
    gappy = (rng.uniform(size=ns_k) < st.gappy_fraction) & (extent > 0)

    hw_tab = np.zeros((ns_k, ns), F32)
    hh_tab = np.zeros((ns_k, ns), F32)
    has = np.nonzero(extent > 0)[0]
    if len(has):
        nz = rng.standard_normal((len(has), ns), dtype=F32)
        wmod = _box1d(nz, 8)*F32(math.sqrt(8))
        hmod = _box1d(rng.standard_normal((len(has), ns), dtype=F32), 12)*F32(math.sqrt(12))
        gmod = _box1d(rng.standard_normal((len(has), ns), dtype=F32), 40)*F32(math.sqrt(40))
        w_tab = extent[has, None].astype(F32)*np.clip(1 + F32(0.35)*wmod, 0.35, 2.0)
        w_tab[gappy[has][:, None] & (gmod < -0.1)] = 0
        for row, k in enumerate(has):
            s = splits[k]
            sa, sb = sorted((float(s['a'] @ s['t']), float(s['b'] @ s['t'])))
            for _ in range(rng.poisson(max(sb - sa, 0.0)/st.gateway_spacing_m)):
                g0 = int(rng.uniform(sa, sb) + span)
                w_tab[row, max(g0, 0):g0 + int(rng.integers(4, 7))] = 0
        hw_tab[has] = w_tab
        hh_tab[has] = hgt[has, None].astype(F32)*np.clip(1 + F32(0.25)*hmod, 0.4, 1.8)
        del nz, wmod, hmod, gmod, w_tab
    tx = np.array([s['t'][0] for s in splits], F32)
    ty = np.array([s['t'][1] for s in splits], F32)

    # ---- permanent rasters
    kind = np.zeros((n, n), np.uint8)
    height = np.zeros((n, n), F32)
    perm = np.zeros((n, n), F32)
    fid = np.empty((n, n), np.int32)
    margin = np.zeros((n, n), bool)
    verge_grey, margin_grey = 108.0, 100.0
    for r0 in range(0, n, CHUNK):
        sl = slice(r0, min(n, r0 + CHUNK))
        b = bid[sl].astype(np.intp)
        d = bdist[sl]
        si = np.clip(U[sl]*tx[b] + V[sl]*ty[b] + F32(span), 0, ns - 1).astype(np.intp)
        inner = road_hw[b]
        mid = inner + verge[b]
        outer = mid + hw_tab[b, si]
        road = d < inner
        hd = (d >= mid) & (d < outer)
        off = d < outer
        kind[sl][road] = KIND_ROAD
        kind[sl][hd] = KIND_HEDGE
        height[sl][hd] = hh_tab[b, si][hd]
        p = perm[sl]
        p[off] = F32(verge_grey)
        p[road] = road_rad[b][road]
        p[hd] = hedge_rad[b][hd]
        fid[sl] = np.where(off, -1, leaf[sl])
        margin[sl] = (d >= outer) & (d < outer + margin_w[b])
    del U, V, bid, leaf

    crown = _value_noise(rng_noise, n, max(2.0, 4.0/res))
    open_px = lambda win: ((kind[win] == KIND_FIELD) & (fid[win] >= 0)) | (kind[win] == KIND_HEDGE)  # noqa: E731

    # whole-parcel woods
    n_wood_parcels = 0
    for li, (poly, _) in enumerate(leaves):
        if rng.uniform() < st.wood_parcel_prob and _poly_area(poly) < 4e4:
            win = _window(xs, ys, poly[:, 0].min() - pad, poly[:, 0].max() + pad,
                          poly[:, 1].min() - pad, poly[:, 1].max() + pad)
            if win is None:
                continue
            m = fid[win] == li
            if not m.any():
                continue
            n_wood_parcels += 1
            kind[win][m] = KIND_TREE
            height[win][m] = np.clip(14 + F32(1.2)*crown[win][m], 4, 24)
            perm[win][m] = F32(rng.normal(52, 4))*(1 + F32(0.08)*crown[win][m])
            fid[win][m] = -1
            margin[win][m] = False

    area_km2 = (size_m/1000.0)**2

    def lumps():
        return [(rng.uniform(0.05, 0.2), rng.uniform(0, 2*math.pi)) for _ in range(4)]

    n_copses = n_ponds = 0
    for _ in range(rng.poisson(st.copses_per_km2*area_km2)):
        cx, cy = rng.uniform(xs[0], xs[-1], 2)
        rad, lm, grey = rng.uniform(15, 45), lumps(), rng.normal(52, 4)
        win, r = _blob(xs, ys, cx, cy, rad, lm)
        if win is None:
            continue
        m = (r < 1) & open_px(win)
        if m.any():
            n_copses += 1
            kind[win][m] = KIND_TREE
            height[win][m] = np.clip((13 + F32(1.2)*crown[win][m])*np.sqrt(np.clip(1.15 - r[m], 0, 1)), 2, 24)
            perm[win][m] = F32(grey)*(1 + F32(0.08)*crown[win][m])
            fid[win][m] = -1
            margin[win][m] = False
    for _ in range(rng.poisson(st.ponds_per_km2*area_km2)):
        cx, cy = rng.uniform(xs[0], xs[-1], 2)
        rad, lm, grey = rng.uniform(5, 16), lumps(), rng.normal(32, 4)
        win, r = _blob(xs, ys, cx, cy, rad, lm)
        if win is None:
            continue
        m = (r < 1) & open_px(win) & (kind[win] == KIND_FIELD)
        if m.any():
            n_ponds += 1
            kind[win][m] = KIND_WATER
            perm[win][m] = F32(grey)
            fid[win][m] = -1
            margin[win][m] = False

    # farmyards and buildings
    n_buildings = 0
    roads = np.nonzero(is_road)[0]
    for _ in range(st.n_farms):
        if len(roads):
            k = int(rng.choice(roads))
            s = splits[k]
            t = s['t']
            nrm = np.array([-t[1], t[0]])
            along = -float(s['p0'] @ t) + rng.uniform(-0.35, 0.35)*size_m
            side = 1.0 if rng.uniform() < 0.5 else -1.0
            cu = s['p0'] + t*along + side*nrm*(road_hw[k] + verge[k] + extent[k] + 26.0)
        else:
            th = rng.uniform(0, math.pi)
            t = np.array([math.cos(th), math.sin(th)])
            cu = rng.uniform(-0.35, 0.35, 2)*size_m
        cx, cy = _unwarp(terms, cu[0], cu[1])
        la, lb = rng.uniform(50, 75), rng.uniform(38, 50)
        win, a, b = _rect(xs, ys, cx, cy, t[0], t[1], la, lb)
        if win is not None:
            m = (np.abs(a) < la/2) & (np.abs(b) < lb/2) & (kind[win] != KIND_ROAD)
            kind[win][m] = KIND_ROAD
            perm[win][m] = F32(rng.normal(150, 8))
            height[win][m] = 0
            fid[win][m] = -1
            margin[win][m] = False
        for _ in range(int(rng.integers(3, 7))):
            ba, bb = rng.uniform(12, 32), rng.uniform(8, 18)
            oa, ob = rng.uniform(-0.5, 0.5)*(la - ba), rng.uniform(-0.5, 0.5)*(lb - bb)
            tt = t if rng.uniform() < 0.6 else np.array([-t[1], t[0]])
            eave, ridge, grey = rng.uniform(4, 6.5), rng.uniform(1.5, 3.0), \
                (rng.normal(185, 20) if rng.uniform() < 0.65 else rng.normal(75, 10))
            bx = cx + oa*t[0] - ob*t[1]
            by = cy + oa*t[1] + ob*t[0]
            win, a, b = _rect(xs, ys, bx, by, tt[0], tt[1], ba, bb)
            if win is None:
                continue
            m = (np.abs(a) < ba/2) & (np.abs(b) < bb/2)
            if m.any():
                n_buildings += 1
                kind[win][m] = KIND_BUILDING
                height[win][m] = (eave + ridge*(1 - np.abs(b)/(bb/2)))[m]
                perm[win][m] = F32(grey)
                fid[win][m] = -1
                margin[win][m] = False

    # trees: along hedges, then in fields
    n_trees = 0

    def tree(cx, cy, need_open):
        nonlocal n_trees
        rad = rng.uniform(2.0, 6.0)
        top = rng.uniform(6.0, 9.0) + 1.6*rad
        lm, grey = lumps()[:3], rng.normal(55, 6)
        if not (xs[0] - rad < cx < xs[-1] + rad and ys[0] - rad < cy < ys[-1] + rad):
            return
        win, r = _blob(xs, ys, cx, cy, rad, lm)
        if win is None:
            return
        m = (r < 1) & (kind[win] != KIND_BUILDING)
        if need_open:
            j = int(np.clip(round((cx - xs[0])/res), 0, n - 1))
            i = int(np.clip(round((cy - ys[0])/res), 0, n - 1))
            if kind[i, j] != KIND_FIELD or fid[i, j] < 0:
                return
        if not m.any():
            return
        h = F32(top)*(F32(0.5) + F32(0.5)*np.sqrt(np.clip(1 - r*r, 0, 1)))
        m &= h > height[win]
        n_trees += 1
        kind[win][m] = KIND_TREE
        height[win][m] = h[m]
        perm[win][m] = F32(grey)
        fid[win][m] = -1
        margin[win][m] = False

    for k in np.nonzero(extent > 0)[0]:
        s = splits[k]
        seg = s['b'] - s['a']
        ln = float(np.hypot(*seg))
        nrm = np.array([-s['t'][1], s['t'][0]])
        for _ in range(rng.poisson(ln*st.trees_per_100m/100.0)):
            if btype[k] == B_HEDGE:
                off = rng.normal(0, 0.5)
            else:
                off = (1.0 if rng.uniform() < 0.5 else -1.0)*(road_hw[k] + verge[k] + 0.5*extent[k])
            pu = s['a'] + seg*rng.uniform() + nrm*off
            if abs(pu[0]) > 0.5*size_m + pad + 8 or abs(pu[1]) > 0.5*size_m + pad + 8:
                continue
            tree(*_unwarp(terms, pu[0], pu[1]), False)
    for _ in range(rng.poisson(st.field_trees_per_km2*area_km2)):
        cx, cy = rng.uniform(xs[0], xs[-1], 2)
        tree(cx, cy, True)

    # ---- compact field ids and per-field geometry
    counts = np.bincount(fid[fid >= 0], minlength=len(leaves))
    present = np.nonzero(counts > 0)[0]
    nf = len(present)
    remap = np.full(len(leaves) + 1, -1, np.int32)
    remap[present] = np.arange(nf, dtype=np.int32)
    fid = remap[fid]
    del remap

    rec = np.zeros(nf, FIELD_DTYPE)
    alt_bearing = np.zeros(nf, F32)
    edge_c = np.zeros(nf)
    lim = 0.5*size_m - pad
    for i, li in enumerate(present):
        poly = leaves[li][0]
        e = np.roll(poly, -1, 0) - poly
        length = np.hypot(e[:, 0], e[:, 1])
        order = np.argsort(-length)
        b0 = _bearing(e[order[0]])
        rec['tram_bearing_deg'][i] = b0
        alt = b0
        for j in order[1:]:
            bj = _bearing(e[j])
            if min(abs(bj - b0), 180 - abs(bj - b0)) > 25:
                alt = bj
                break
        alt_bearing[i] = alt
        br = math.radians(b0)
        edge_c[i] = poly[order[0]] @ np.array([math.cos(br), -math.sin(br)])
        rec['area_full_m2'][i] = _poly_area(poly)
        rec['truncated'][i] = bool((np.abs(poly) > lim).any())
        nx_ = np.roll(poly, -1, 0)
        cr = poly[:, 0]*nx_[:, 1] - nx_[:, 0]*poly[:, 1]
        cen = ((poly + nx_)*cr[:, None]).sum(0)/(3*cr.sum())
        rec['east'][i], rec['north'][i] = _unwarp(terms, cen[0], cen[1])
    rec['area_m2'] = counts[present]*res*res

    # ---- noise rasters and map-day field parameters
    patch = _value_noise(rng_noise, n, 60.0/res)
    patch *= F32(0.8)
    patch += F32(0.6)*_value_noise(rng_noise, n, 18.0/res)
    fine = _fine_noise(rng_noise, n)
    stripe_tab = rng_noise.standard_normal((64, 1024), dtype=F32)

    has_tram = rng_field.uniform(size=nf) < st.tramline_fraction
    spacing = rng_field.choice(np.array(st.tramline_spacings, F32), size=nf,
                               p=np.array(st.tramline_weights)/np.sum(st.tramline_weights))
    gauge = rng_field.uniform(1.8, 2.0, nf).astype(F32)
    con_mag = st.tramline_contrast*np.exp(0.35*rng_field.standard_normal(nf))
    con_sign = np.where(rng_field.uniform(size=nf) < 0.8, -1.0, 1.0)
    con_map = np.where(has_tram, con_mag*con_sign, 0.0).astype(F32)
    mean_map = _draw_levels(rng_field, st.map_cover, nf)
    pamp_map = (st.patch_sd*np.exp(0.4*rng_field.standard_normal(nf))).astype(F32)
    famp_map = (st.fine_sd*np.exp(0.3*rng_field.standard_normal(nf))).astype(F32)
    samp_map = (st.stripe_sd*np.exp(0.5*rng_field.standard_normal(nf))).astype(F32)
    sw = rng_field.choice(np.array([3.0, 4.0, 6.0], F32), size=nf)
    srow = rng_field.integers(0, 64, nf)
    phase_map = np.mod(edge_c + 0.5*spacing, spacing).astype(F32)

    nxm, nym = _tram_normal(rec['tram_bearing_deg'])
    fp = dict(mean=mean_map, pamp=pamp_map, famp=famp_map, nx=nxm, ny=nym, sp=spacing,
              ph=phase_map, g=gauge, con=con_map, samp=samp_map, sw=sw, srow=srow)
    field_map = _field_layer(fid, xs, ys, bdist, margin, patch, fine, None, fp, stripe_tab, res, margin_grey)

    # ---- flight-day field parameters
    def pick(frac):
        chosen = np.zeros(nf, bool)
        chosen[rng_season.permutation(nf)[:int(round(float(np.clip(frac, 0, 1))*nf))]] = True
        return chosen

    alb = pick(season.albedo_change)
    trm = pick(season.tramline_change)
    mean_f = np.where(alb, _draw_levels(rng_season, st.flight_cover, nf), mean_map).astype(F32)
    pamp_f = np.where(alb, st.patch_sd*np.exp(0.4*rng_season.standard_normal(nf)), pamp_map).astype(F32)
    samp_f = np.where(alb, st.stripe_sd*np.exp(0.5*rng_season.standard_normal(nf)), samp_map).astype(F32)
    srow_f = np.where(alb, rng_season.integers(0, 64, nf), srow)
    new_has = rng_season.uniform(size=nf) < st.tramline_fraction
    new_con = np.where(new_has, st.tramline_contrast*np.exp(0.35*rng_season.standard_normal(nf))
                       * np.where(rng_season.uniform(size=nf) < 0.8, -1.0, 1.0), 0.0)
    con_f = np.where(trm, new_con, con_map).astype(F32)
    rebear = trm & (rng_season.uniform(size=nf) < season.tramline_rebearing)
    bearing_f = np.where(rebear, alt_bearing, rec['tram_bearing_deg']).astype(F32)
    new_sp = rng_season.choice(np.array(st.tramline_spacings, F32), size=nf,
                               p=np.array(st.tramline_weights)/np.sum(st.tramline_weights))
    spacing_f = np.where(rebear, new_sp, spacing).astype(F32)
    phase_f = np.where(trm, rng_season.uniform(size=nf)*spacing_f, phase_map).astype(F32)

    if alb.any() or trm.any():
        nxf, nyf = _tram_normal(bearing_f)
        fine2 = _fine_noise(rng_season, n) if alb.any() else None
        fpf = dict(mean=mean_f, pamp=pamp_f, famp=famp_map, nx=nxf, ny=nyf, sp=spacing_f,
                   ph=phase_f, g=gauge, con=con_f, samp=samp_f, sw=sw, srow=srow_f, new=alb)
        field_flight = _field_layer(fid, xs, ys, bdist, margin, patch, fine, fine2, fpf,
                                    stripe_tab, res, margin_grey)
        del fine2
    else:
        field_flight = field_map.copy()
    del bdist, margin, patch

    rec['map_mean'], rec['flight_mean'] = mean_map, mean_f
    rec['tram_spacing_m'], rec['tram_gauge_m'] = spacing, gauge
    rec['tram_contrast_map'], rec['tram_contrast_flight'] = con_map, con_f
    rec['tram_phase_map_m'], rec['tram_phase_flight_m'] = phase_map, phase_f
    rec['tram_bearing_flight_deg'], rec['tram_spacing_flight_m'] = bearing_f, spacing_f
    rec['albedo_changed'], rec['tram_changed'], rec['changed'] = alb, trm, alb | trm

    # ---- compose: permanent surfaces, texture, light
    tex = np.array([0.05, 0.12, 0.12, 0.03, 0.02, 0.02], F32)[kind]
    tex *= fine
    tex += 1
    perm *= tex
    del tex, fine
    is_field = fid >= 0
    world_map = np.where(is_field, field_map, perm)
    del field_map
    world_flight = np.where(is_field, field_flight, perm)
    del field_flight, is_field
    lo = float(np.clip(season.leaf_off, 0, 1))
    if lo > 0:
        veg = (kind == KIND_HEDGE) | (kind == KIND_TREE)
        world_flight[veg] = (1 - 0.45*lo)*world_flight[veg] + 0.45*lo*F32(105.0)
        del veg
    del perm, crown
    _illuminate(world_map, height, kind, res, season.sun_map, st.shadow_depth)
    _illuminate(world_flight, height, kind, res, season.sun_flight, st.shadow_depth*(1 - 0.5*lo))
    np.clip(world_map, 0, 255, out=world_map)
    np.clip(world_flight, 0, 255, out=world_flight)
    if season.gamma != 1.0:
        world_flight = (F32(255)*(world_flight/F32(255))**F32(season.gamma)).astype(F32)
    if season.gain != 1.0 or season.offset != 0.0:
        world_flight = world_flight*F32(season.gain) + F32(season.offset)
        np.clip(world_flight, 0, 255, out=world_flight)

    kind_fraction = np.bincount(kind.ravel(), minlength=6)/kind.size
    meta = dict(
        statement=SYNTHETIC_STATEMENT, synthetic=True, seed=seed, style=st.name,
        size_m=float(size_m), res_m=res, shape=(n, n),
        style_parameters=asdict(st), season=asdict(season),
        counts=dict(fields=int(nf), splits=int(ns_k - 1), trees=int(n_trees), copses=int(n_copses),
                    wood_parcels=int(n_wood_parcels), ponds=int(n_ponds), buildings=int(n_buildings),
                    boundaries_by_type={name: int((btype[1:] == code).sum()) for name, code in
                                        (('crop_line', B_NONE), ('hedge', B_HEDGE), ('fence', B_FENCE),
                                         ('road', B_ROAD), ('track', B_TRACK))}),
        kind_fraction={name: float(v) for name, v in zip(KIND_NAMES, kind_fraction)},
        kind_names=KIND_NAMES, limitations=LIMITATIONS, parameters=PARAMETERS,
        conventions='raster[row, col] at east=origin[0]+col*res, north=origin[1]+row*res; '
                    'north increases with row; sun azimuth and bearings clockwise from north')
    return World(res=res, origin=(x0, x0), map=world_map, flight=world_flight, height=height,
                 field_id=fid, kind=kind, fields=rec, meta=meta)


# ------------------------------------------------- measurement and previews

def _smooth(a, sigma_px):
    """Approximate Gaussian blur: three box filters along each axis."""
    if sigma_px <= 0:
        return a
    w = max(1, int(round(math.sqrt(4*sigma_px**2 + 1))))
    if w == 1:
        return a
    for _ in range(3):
        a = _box1d(a, w)
        a = _box1d(a.T, w).T
    return a


def gradient_correlation(a, b, res, sigma_m):
    """Pearson correlation of the gradient-magnitude images after smoothing."""
    out = []
    for img in (a, b):
        s = _smooth(img, sigma_m/res)
        gy, gx = np.gradient(s)
        out.append(np.hypot(gx, gy).ravel())
    return float(np.corrcoef(out[0], out[1])[0, 1])


def field_means(world, raster):
    """Measured mean and SD of a raster inside each field."""
    m = world.field_id >= 0
    ids = world.field_id[m]
    v = raster[m].astype(np.float64)
    nf = len(world.fields)
    cnt = np.maximum(np.bincount(ids, minlength=nf), 1)
    mean = np.bincount(ids, v, nf)/cnt
    var = np.bincount(ids, v*v, nf)/cnt - mean**2
    return mean, np.sqrt(np.maximum(var, 0))


def measure_tramline_spacing(world, index, raster=None, bearing_deg=None):
    """Tramline spacing (m) of one field from the raster alone: the image is
    averaged along the recorded bearing and the first strong autocorrelation
    peak of the resulting cross-profile between 6 and 45 m is returned."""
    raster = world.map if raster is None else raster
    b = math.radians(world.fields['tram_bearing_deg'][index] if bearing_deg is None else bearing_deg)
    rr, cc = np.nonzero(world.field_id == index)
    r0, c0 = rr.mean(), cc.mean()
    step = 0.25
    q = np.arange(-90.0, 90.0 + step, step)
    a = np.arange(-40.0, 40.5, 1.0)
    east = (q[None, :]*math.cos(b) + a[:, None]*math.sin(b))/world.res + c0
    north = (-q[None, :]*math.sin(b) + a[:, None]*math.cos(b))/world.res + r0
    n0, n1 = raster.shape
    i0, j0 = np.floor(north).astype(int), np.floor(east).astype(int)
    ok = (i0 >= 0) & (i0 < n0 - 1) & (j0 >= 0) & (j0 < n1 - 1)
    i0, j0 = np.clip(i0, 0, n0 - 2), np.clip(j0, 0, n1 - 2)
    fy, fx = north - i0, east - j0
    val = (raster[i0, j0]*(1 - fy)*(1 - fx) + raster[i0, j0 + 1]*(1 - fy)*fx
           + raster[i0 + 1, j0]*fy*(1 - fx) + raster[i0 + 1, j0 + 1]*fy*fx)
    for di in (0, 1):
        for dj in (0, 1):
            ok &= world.field_id[i0 + di, j0 + dj] == index
    cnt = ok.sum(0)
    good = cnt >= 20
    if good.sum() < 45/step:
        return float('nan')
    prof = np.where(good, (val*ok).sum(0)/np.maximum(cnt, 1), 0.0)
    w = int(6/step)
    wsum = np.convolve(good.astype(float), np.ones(w), 'same')
    trend = np.convolve(prof, np.ones(w), 'same')/np.maximum(wsum, 1)
    hp = np.where(good, prof - trend, 0.0)
    lags = np.arange(int(6/step), int(45/step) + 1)
    ac = np.array([np.sum(hp[k:]*hp[:-k])/max(np.sum(good[k:] & good[:-k]), 1) for k in lags])
    hi = int(np.nonzero(ac >= 0.8*ac.max())[0][0])
    while hi + 1 < len(ac) and ac[hi + 1] > ac[hi]:
        hi += 1
    return float(lags[hi]*step)


def statistics(world):
    """Measured properties of a generated world (all from the rasters)."""
    f = world.fields
    whole = f[~f['truncated']]
    mm, ms = field_means(world, world.map)
    fm, _ = field_means(world, world.flight)
    big = f['area_m2'] > 2000
    kinds = {name: float(world.map[world.kind == k].mean()) if (world.kind == k).any() else None
             for k, name in enumerate(KIND_NAMES)}
    field_level = float(world.map[world.field_id >= 0].mean())
    area = whole['area_full_m2']/1e4
    pct = (lambda p: float(np.percentile(area, p))) if len(area) else (lambda p: None)
    return dict(
        n_fields=int(len(f)), n_whole_fields=int(len(whole)),
        field_area_ha=dict(median=pct(50), p10=pct(10), p90=pct(90),
                           mean=float(area.mean()) if len(area) else None),
        mean_visible_field_area_ha=float(f['area_m2'].mean()/1e4),
        kind_fraction=world.meta['kind_fraction'],
        between_field_sd=float(mm[big].std()), within_field_sd_median=float(np.median(ms[big])),
        mean_level_by_kind=kinds, field_mean_level=field_level,
        hedge_minus_field=None if kinds['hedge'] is None else kinds['hedge'] - field_level,
        road_minus_field=None if kinds['road'] is None else kinds['road'] - field_level,
        field_mean_correlation_map_flight=float(np.corrcoef(mm[big], fm[big])[0, 1]) if big.sum() > 2 else None,
        gradient_correlation={f'{s:g} m': gradient_correlation(world.map, world.flight, world.res, s)
                              for s in (0.0, 1.0, 2.0, 4.0, 8.0)},
        raster_correlation=float(np.corrcoef(world.map.ravel(), world.flight.ravel())[0, 1]),
        fields_with_tramlines_map=int((f['tram_contrast_map'] != 0).sum()),
        tramline_spacing_counts={f'{s:g}': int(((f['tram_spacing_m'] == s) & (f['tram_contrast_map'] != 0)).sum())
                                 for s in np.unique(f['tram_spacing_m'])})


KIND_COLOURS = np.array([[196, 204, 150], [40, 90, 40], [20, 130, 60], [120, 120, 125],
                         [200, 60, 50], [50, 90, 200]], np.uint8)


def _peak_memory_mb():
    try:
        import ctypes
        from ctypes import wintypes

        class Counters(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD), ('faults', wintypes.DWORD)] + [
                (name, ctypes.c_size_t) for name in (
                    'peak_wss', 'wss', 'qpp', 'qp', 'qpnp', 'qnp', 'pagefile', 'peak_pagefile')]
        c = Counters()
        c.cb = ctypes.sizeof(c)
        k32 = ctypes.WinDLL('kernel32')
        k32.GetCurrentProcess.restype = wintypes.HANDLE
        psapi = ctypes.WinDLL('psapi')
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD]
        if psapi.GetProcessMemoryInfo(k32.GetCurrentProcess(), ctypes.byref(c), c.cb):
            return c.peak_wss/2**20
    except Exception:
        pass
    return None


def save_previews(world, folder, stem):
    """North-up PNGs: 2 m/px overviews and a 400 m full-resolution crop."""
    from PIL import Image
    n = world.map.shape[0]
    k = max(1, int(round(2.0/world.res)))
    m = (n//k)*k

    def small(a):
        return np.flipud(a[:m, :m].reshape(m//k, k, m//k, k).mean((1, 3)))
    Image.fromarray(small(world.map).astype(np.uint8)).save(folder/f'{stem}_map.png')
    Image.fromarray(small(world.flight).astype(np.uint8)).save(folder/f'{stem}_flight.png')
    Image.fromarray(np.flipud(KIND_COLOURS[world.kind[::k, ::k]])).save(folder/f'{stem}_kind.png')
    h = int(round(200.0/world.res))
    c = n//2
    lo, hi = max(c - h, 0), min(c + h, n)
    crop = np.hstack((np.flipud(world.map[lo:hi, lo:hi]), np.flipud(world.flight[lo:hi, lo:hi])))
    Image.fromarray(crop.astype(np.uint8)).save(folder/f'{stem}_crop400m_map_flight.png')


def main(seed=2027):
    """Previews and measured statistics for every style (default season, plus
    the all-fields-changed season for the structure-persistence figures)."""
    import json
    import time
    from pathlib import Path
    folder = Path(__file__).resolve().parent/'results'/'farmland'
    folder.mkdir(parents=True, exist_ok=True)
    summary = dict(statement=SYNTHETIC_STATEMENT, seed=seed, size_m=1600.0, res_m=0.5,
                   parameters=[dict(parameter=p, value=v, basis=b, reference=r) for p, v, b, r in PARAMETERS],
                   limitations=LIMITATIONS, styles={})
    for name in STYLES:
        t0 = time.perf_counter()
        world = generate(seed, style=name)
        seconds = time.perf_counter() - t0
        peak = _peak_memory_mb()
        save_previews(world, folder, name)
        stats = statistics(world)
        tram = [i for i in np.argsort(-world.fields['area_m2'])[:12]
                if world.fields['tram_contrast_map'][i] != 0 and world.fields['area_m2'][i] > 2e4]
        stats['tramline_spacing_check'] = [
            dict(field=int(i), recorded_m=float(world.fields['tram_spacing_m'][i]),
                 measured_m=measure_tramline_spacing(world, i)) for i in tram]
        del world
        changed = generate(seed, style=name, season=Season(albedo_change=1.0))
        s1 = statistics(changed)
        del changed
        summary['styles'][name] = dict(
            generate_seconds=round(seconds, 1), peak_memory_mb_after_generate=peak,
            style_parameters=asdict(STYLES[name]), default_season=stats,
            all_fields_changed=dict(field_mean_correlation_map_flight=s1['field_mean_correlation_map_flight'],
                                    gradient_correlation=s1['gradient_correlation'],
                                    raster_correlation=s1['raster_correlation']))
        print(name, f'{seconds:.1f} s', 'peak MB', peak, flush=True)
        (folder/'summary.json').write_text(json.dumps(summary, indent=1, default=str))


if __name__ == '__main__':
    main()
