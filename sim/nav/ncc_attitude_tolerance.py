"""GPL-3.0-only. SYNTHETIC measurement of NCC landmark-matcher tolerance to predicted-pose error.

Usage: python sim/nav/ncc_attitude_tolerance.py
Writes sim/nav/results/ncc_attitude_tolerance.{json,csv}. Only NumPy is required.
SI units, upward world z, camera looks along its +z; angles in degrees at the interfaces.

A camera frame is rendered from the TRUE pose, rectified onto the map grid with a PREDICTED
pose that is wrong by one controlled error at a time (heading, tilt, height or horizontal
position), and landmarks are located by normalised cross-correlation with the existing gates.
A match is scored against truth as a (map point, original image pixel) pair: the pixel is
back-projected through the TRUE pose and must land within 1.5 map pixels of the map point.
That is independent of the predicted pose being wrong, which is the point of the experiment.

Everything here is synthetic: feasibility.terrain is a sum of six sinusoids, not farmland.
These are verification models, not flight software.
"""
import csv
import json
import math
import platform
import sys
import time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True  # importing must not add .pyc files under sim/maths
sys.path.insert(0, str(HERE.parent/'maths'))
from feasibility import terrain, render_image, camera_pixels, bilinear, quadratic_peak  # noqa: E402
from models import ncc_fft, rotation  # noqa: E402

OUT = HERE/'results'
DATE = '2026-10-08'
SEED = 20261008
SCENES_PER_HEIGHT = 24
HEIGHTS = (300., 150., 80.)
NOISE_SD = 2.           # grey levels, as sim/maths/run_feasibility.py
MAX_TILT_DEG = 10.      # true tilt magnitude is uniform on [0, MAX_TILT_DEG]
NADIR = np.diag([1., -1., -1.])
# Sony IMX296, 3.45 um pixels, 2.5 mm lens: 2.5e-3/3.45e-6 = 724.6 px, taken as 725 px.
CAMERA = {'name': 'Sony IMX296 1456x1088, 2.5 mm lens, FULL resolution (no binning)',
          'size': (1088, 1456), 'focal_px': 725.}
# Ray tangents of the 3x3 landmark grid; the same +/-0.4h, +/-0.23h layout as rendered_navigation().
GRID = [(a, b) for a in (-.4, 0., .4) for b in (-.23, 0., .23)]
SCORE_GATE, GAP_GATE, GAP_EXCLUSION_PX = .5, .04, 3   # feasibility.image_matches
CORRECT_MAP_PX = 1.5
MIN_CELL = 20           # fewest samples for a fraction to be used in a threshold

# template_from='map': map template located in the rectified image (existing fine pipeline).
# template_from='image': rectified-image template located in the map. This is the only layout in
# which a 128 m template with a wide search fits: the footprint at 300 m is only 602 x 450 m.
# texture_stretch=s matches against terrain(x/s, y/s): the same texture with every wavelength
# multiplied by s. COARSE_TEX4 is therefore NOT the baseline terrain; it is the coarse matcher on
# a texture whose correlation length in 2 m map pixels equals that of the baseline terrain in
# 0.5 m map pixels, used to test the pixel scaling law free of under-sampling.
CONFIGS = {
    'FINE': {'template_px': 32, 'scale_m': .5, 'search_m': 16., 'template_from': 'map',
             'heights': HEIGHTS, 'sweeps': ('yaw', 'tilt', 'height', 'position')},
    'COARSE': {'template_px': 64, 'scale_m': 2., 'search_m': 64., 'template_from': 'image',
               'heights': HEIGHTS, 'sweeps': ('yaw', 'tilt', 'height', 'position')},
    'COARSE_TEX4': {'template_px': 64, 'scale_m': 2., 'search_m': 64., 'template_from': 'image',
                    'texture_stretch': 4., 'heights': (300.,),
                    'sweeps': ('yaw', 'tilt', 'height', 'position'),
                    'extra': {'position': (48., 60., 72., 96.)}},
    'COARSE_WIDE': {'template_px': 64, 'scale_m': 2., 'search_m': 300., 'template_from': 'image',
                    'heights': (300.,), 'sweeps': ('yaw',), 'scenes': 12},
}
SWEEPS = {
    'yaw': {'unit': 'deg', 'values': (0., 1., 2., 3., 5., 7.5, 10., 15., 20., 30., 45., 90., 180.)},
    'tilt': {'unit': 'deg', 'values': (0., .25, .5, 1., 2., 3., 5., 8.)},
    'height': {'unit': 'percent', 'values': (0., 1., 2., 5., 10., 20.)},
    'position': {'unit': 'm', 'values': (0., 2., 4., 8., 12., 16., 24.)},
}
LIMITS = [
    'SYNTHETIC. feasibility.terrain is a sum of six sinusoids with wavelengths of about 3.4 to '
    '17 m, not farmland imagery. NCC tolerance depends on the texture correlation length, so the '
    'absolute angles will differ on real maps; the scaling (tolerance ~ correlation length / '
    'template half-width, both in map pixels) is what should carry over.',
    'A sum of sinusoids is quasi-periodic: the two strongest components (two thirds of the '
    'variance) repeat exactly on a lattice of about 16 x 10 m, and each sinusoid equals its own '
    '180 deg rotation up to a shift. Secondary correlation peaks, and therefore every false-match '
    'rate reported here, are properties of this texture, not estimates for real imagery.',
    'On the 2 m/px grid the baseline terrain is under-sampled (1/e correlation length about 1.4 '
    'map pixels). A sub-pixel misalignment alone then lowers the true peak below the texture '
    'repeats inside the search window, so the COARSE and COARSE_WIDE rows measure that artefact, '
    'not attitude tolerance. COARSE_TEX4 (wavelengths x4) is the well-sampled coarse case.',
    'Flat ground, pinhole camera, principal point at the image centre, no lens distortion, no '
    'blur, no photometric change, no map/season mismatch, 2 grey-level white noise only.',
    'Maps and rectified images are point-sampled (bilinear) with no anti-alias filter.',
    'One error type at a time; real first-fix errors are combined (yaw + tilt + position).',
    'Landmarks are a fixed 3x3 pixel grid projected with the PREDICTED pose (what a real system '
    'can do), not pre-selected high-contrast map features. Landmarks within a frame share one '
    'pose error, so the nine results per frame are not independent samples.',
    'Thresholds are read from a coarse list of tested error levels and 24 frames per height '
    '(12 for COARSE_WIDE); the 90 % level is the largest TESTED level, and the 50 % point is a '
    'linear interpolation between tested levels.',
    'Per-landmark gates only (score, peak gap, quadratic peak). No multi-landmark consensus or '
    'innovation gating, which the proposal relies on to reject false matches.',
]


def attitude(yaw, tilt, tilt_dir):
    """Camera-to-world rotation: nadir camera tilted about a horizontal world axis, then headed."""
    axis = np.array([math.cos(tilt_dir), math.sin(tilt_dir), 0.])
    return rotation([0., 0., yaw])@rotation(tilt*axis)@NADIR


def make_scene(seed, height, index, camera=CAMERA):
    """True pose, fixed error directions/signs and the image noise for one trial."""
    rng = np.random.default_rng([seed, int(round(height)), index])
    scene = {'height_m': float(height), 'index': index,
             'pos': np.r_[rng.uniform(-1000., 1000., 2), height],
             'yaw': rng.uniform(-math.pi, math.pi),
             'tilt': math.radians(rng.uniform(0., MAX_TILT_DEG)),
             'tilt_dir': rng.uniform(0., 2*math.pi),
             'yaw_sign': float(rng.choice([-1., 1.])), 'height_sign': float(rng.choice([-1., 1.])),
             'tilt_error_dir': rng.uniform(0., 2*math.pi),
             'position_dir': rng.uniform(0., 2*math.pi)}
    scene['r'] = attitude(scene['yaw'], scene['tilt'], scene['tilt_dir'])
    scene['noise'] = rng.normal(size=camera['size'])*NOISE_SD
    return scene


def scene_image(scene, camera=CAMERA, stretch=1.):
    """Noisy frame of terrain(x/stretch, y/stretch) from the true pose.

    By similarity this is the unstretched terrain seen from pos/stretch with the same attitude.
    """
    return render_image(scene['pos']/stretch, scene['r'], camera['size'], camera['focal_px'])+scene['noise']


def predicted_pose(scene, kind, value):
    """True pose with exactly one controlled error. Yaw is about the world vertical through the
    camera; tilt is about a horizontal world axis; height is a fraction; position is horizontal."""
    pos, r = scene['pos'].copy(), scene['r']
    if kind == 'yaw':
        r = rotation([0., 0., scene['yaw_sign']*math.radians(value)])@r
    elif kind == 'tilt':
        d = scene['tilt_error_dir']
        r = rotation(math.radians(value)*np.array([math.cos(d), math.sin(d), 0.]))@r
    elif kind == 'height':
        pos[2] *= 1+scene['height_sign']*value/100
    elif kind == 'position':
        d = scene['position_dir']
        pos[:2] += value*np.array([math.cos(d), math.sin(d)])
    else:
        raise ValueError('unknown error kind '+kind)
    return pos, r


def ground_to_pixel(xy, pos, r, focal, size):
    """Ground-plane (z=0) metres to image pixels; NaN behind the camera."""
    xy = np.atleast_2d(np.asarray(xy, dtype=float))
    uv, depth = camera_pixels(np.column_stack((xy, np.zeros(len(xy)))), pos, r, focal, size)
    return np.where(depth[:, None] > 0, uv, np.nan)


def pixel_to_ground(uv, pos, r, focal, size):
    """Inverse of camera_pixels for the plane z=0; NaN for rays at or above the horizon."""
    uv = np.atleast_2d(np.asarray(uv, dtype=float))
    rays = np.column_stack(((uv[:, 0]-size[1]/2)/focal, (uv[:, 1]-size[0]/2)/focal,
                            np.ones(len(uv))))@r.T
    down = rays[:, 2] < 0
    distance = np.where(down, -pos[2]/np.where(down, rays[:, 2], -1.), np.nan)
    return pos[:2]+distance[:, None]*rays[:, :2]


def patch_to_map(index_xy, centre, size, scale):
    """Rectified-patch pixel index (x, y) to map metres; index (size-1)/2 is the patch centre."""
    return np.asarray(centre, dtype=float)[:2]+(np.asarray(index_xy, dtype=float)-(size-1)/2)*scale


def map_to_patch(xy, centre, size, scale):
    return (np.asarray(xy, dtype=float)-np.asarray(centre, dtype=float)[:2])/scale+(size-1)/2


def patch_grid(centre, size, scale):
    index = np.stack(np.meshgrid(np.arange(size), np.arange(size)), axis=-1)
    return patch_to_map(index, centre, size, scale)


def rectify(image, centre, pos, r, size, scale, focal):
    """As feasibility.rectify_patch (same grid), plus NaN for points behind the camera."""
    xy = patch_grid(centre, size, scale).reshape(-1, 2)
    uv, depth = camera_pixels(np.column_stack((xy, np.zeros(len(xy)))), pos, r, focal, image.shape)
    patch = bilinear(image, uv.reshape(size, size, 2))
    return np.where(depth.reshape(size, size) > 0, patch, np.nan)


def map_patch(centre, size, scale, stretch=1.):
    """Stored-map stand-in: terrain point-sampled on the map grid around centre."""
    xy = patch_grid(centre, size, scale)/stretch
    return terrain(xy[..., 0], xy[..., 1])


def landmark_pixels(camera=CAMERA):
    rows, cols = camera['size']
    return np.array([(cols/2+a*camera['focal_px'], rows/2+b*camera['focal_px']) for a, b in GRID])


def match_landmark(image, landmark, pos, r, cfg, focal):
    """Locate one landmark with the predicted pose (pos, r). No truth is used here.

    Returns the correspondence as map_xy (map metres) and uv (original image pixel), the latter
    obtained through the same predicted-pose mapping that was used to rectify, plus the score
    surface. Score index `half` on each axis is zero offset from the predicted location.
    """
    t, scale, stretch = cfg['template_px'], cfg['scale_m'], cfg.get('texture_stretch', 1.)
    half = int(round(cfg['search_m']/scale))
    big = t+2*half
    landmark = np.asarray(landmark, dtype=float)[:2]
    if cfg['template_from'] == 'map':
        search = rectify(image, landmark, pos, r, big, scale, focal)
        if not np.all(np.isfinite(search)):
            return {'in_view': False, 'accepted': False}
        template = map_patch(landmark, t, scale, stretch)
    else:
        template = rectify(image, landmark, pos, r, t, scale, focal)
        if not np.all(np.isfinite(template)):
            return {'in_view': False, 'accepted': False}
        search = map_patch(landmark, big, scale, stretch)
    scores = ncc_fft(search, template)
    iy, ix = np.unravel_index(np.nanargmax(scores), scores.shape)
    other = scores.copy()
    g = GAP_EXCLUSION_PX
    other[max(0, iy-g):iy+g+1, max(0, ix-g):ix+g+1] = np.nan
    score = float(scores[iy, ix])
    gap = float(score-np.nanmax(other))
    peak = quadratic_peak(scores)
    located = np.array([ix, iy], dtype=float) if peak is None else peak
    shift = (located-half)*scale
    if cfg['template_from'] == 'map':
        map_xy, image_xy = landmark, landmark+shift
    else:
        map_xy, image_xy = landmark+shift, landmark
    uv = ground_to_pixel(image_xy, pos, r, focal, image.shape)[0]
    return {'in_view': True, 'score': score, 'gap': gap, 'peak_ok': peak is not None,
            'accepted': bool(peak is not None and score > SCORE_GATE and gap > GAP_GATE),
            'shift': shift, 'map_xy': map_xy, 'uv': uv, 'scores': scores, 'half': half}


def evaluate(image, true_pose, pred_pose, cfg, camera=CAMERA):
    """Match the nine footprint landmarks of one frame and score them against truth.

    Landmarks are the map points under a fixed 3x3 pixel grid according to the PREDICTED pose.
    ideal_shift is the offset at which a perfect matcher would report the peak; truth_in_window
    says whether that offset is reachable (at least one map pixel inside the search window).
    peak_correct ignores the gates; correct requires acceptance as well.
    """
    size, focal = camera['size'], camera['focal_px']
    (tpos, tr), (ppos, pr) = true_pose, pred_pose
    grid = landmark_pixels(camera)
    landmarks = pixel_to_ground(grid, ppos, pr, focal, size)
    limit = cfg['search_m']-cfg['scale_m']
    records = []
    for j, landmark in enumerate(landmarks):
        if cfg['template_from'] == 'map':
            seen = ground_to_pixel(landmark, tpos, tr, focal, size)
            ideal = pixel_to_ground(seen, ppos, pr, focal, size)[0]-landmark
        else:
            ideal = pixel_to_ground(grid[j], tpos, tr, focal, size)[0]-landmark
        record = match_landmark(image, landmark, ppos, pr, cfg, focal)
        record.update(landmark=j, ideal_shift=ideal,
                      truth_in_window=bool(np.max(np.abs(ideal)) <= limit))
        if record['in_view']:
            scores, half = record.pop('scores'), record.pop('half')
            error = pixel_to_ground(record['uv'], tpos, tr, focal, size)[0]-record['map_xy']
            pixel = record['uv']-ground_to_pixel(record['map_xy'], tpos, tr, focal, size)[0]
            record.update(ground_error_m=float(np.linalg.norm(error)),
                          pixel_error_px=float(np.linalg.norm(pixel)))
            record['peak_correct'] = bool(record['ground_error_m'] <= CORRECT_MAP_PX*cfg['scale_m'])
            record['correct'] = bool(record['accepted'] and record['peak_correct'])
            if record['truth_in_window']:
                # NCC where the truth is: best of the 3x3 scores around the nearest whole pixel.
                ix, iy = np.rint(ideal/cfg['scale_m']).astype(int)+half
                record['ncc_at_truth'] = float(np.nanmax(scores[iy-1:iy+2, ix-1:ix+2]))
        records.append(record)
    return records


def summarise(records):
    """Cell statistics. Fractions are of landmarks whose patch lay wholly inside the image."""
    view = [x for x in records if x['in_view']]
    window = [x for x in view if x['truth_in_window']]
    accepted = [x for x in view if x['accepted']]
    correct = [x for x in accepted if x['correct']]
    false = [x for x in accepted if not x['correct']]

    def ratio(a, b):
        return len(a)/len(b) if b else None

    def mean(key, group):
        return float(np.mean([x[key] for x in group])) if group else None

    def rms(key):
        return float(np.sqrt(np.mean([x[key]**2 for x in correct]))) if correct else None
    return {'landmarks_attempted': len(records), 'in_view': len(view), 'truth_in_window': len(window),
            'accepted_frac': ratio(accepted, view), 'correct_frac': ratio(correct, view),
            'false_frac': ratio(false, view),
            'mean_peak_ncc': mean('score', view),
            'rms_ground_error_correct_m': rms('ground_error_m'),
            'rms_pixel_error_correct_px': rms('pixel_error_px'),
            'correct_frac_truth_in_window': ratio([x for x in window if x['correct']], window),
            'peak_correct_frac': ratio([x for x in view if x['peak_correct']], view),
            'peak_correct_frac_truth_in_window': ratio([x for x in window if x['peak_correct']], window),
            'mean_ncc_at_truth': mean('ncc_at_truth', window),
            'correct_frac_of_attempted': ratio(correct, records),
            'false_frac_of_accepted': ratio(false, accepted),
            'accepted': len(accepted), 'correct': len(correct), 'false': len(false),
            'false_with_truth_in_window': len([x for x in false if x['truth_in_window']]),
            'rejected_low_score': len([x for x in view if not x['score'] > SCORE_GATE]),
            'rejected_low_gap': len([x for x in view if not x['gap'] > GAP_GATE]),
            'rejected_bad_peak': len([x for x in view if not x['peak_ok']]),
            'mean_peak_gap': mean('gap', view)}


def thresholds(levels, fractions):
    """Largest tested level with >=90 % at it and every smaller level, and linear crossings.

    fractions may contain None (too few samples); the curve is cut at the first None.
    'censored' means the curve never fell below the level inside the usable tested range.
    """
    pairs = []
    for level, fraction in sorted(zip(levels, fractions), key=lambda p: p[0]):
        if fraction is None:
            break
        pairs.append((float(level), float(fraction)))
    result = {'tested_to': pairs[-1][0] if pairs else None}
    for name, cut in (('90', .9), ('50', .5)):
        tested = crossing = None
        censored = bool(pairs) and pairs[0][1] >= cut
        for k, (level, fraction) in enumerate(pairs):
            if fraction >= cut:
                tested = level
                continue
            if k:
                l0, f0 = pairs[k-1]
                crossing = l0+(f0-cut)/(f0-fraction)*(level-l0)
            censored = False
            break
        result.update({'tol'+name+'_tested': tested, 'e'+name+'_interp': crossing,
                       'censored'+name: censored})
    return result


def autocorrelation(scale, n=1024, max_lag=128):
    """Unbiased sample autocorrelation of terrain on an n x n grid at `scale` m/px.

    Returned as acf[ly+max_lag, lx+max_lag] for integer pixel lags, normalised to 1 at zero.
    """
    xy = np.arange(n)*scale
    field = terrain(*np.meshgrid(xy, xy))
    field = field-field.mean()
    raw = np.fft.irfft2(abs(np.fft.rfft2(field, (2*n, 2*n)))**2, (2*n, 2*n))
    lags = np.arange(-max_lag, max_lag+1)
    acf = raw[np.ix_(lags % (2*n), lags % (2*n))]/np.outer(n-abs(lags), n-abs(lags))
    return acf/acf[max_lag, max_lag]


def correlation_length(acf, level=math.exp(-1), directions=72):
    """First lag radius (pixels of the acf grid) at which the ACF falls to `level`, per direction."""
    centre = (len(acf)-1)//2
    radii = np.linspace(0., centre-1, 16*(centre-1)+1)
    lengths = []
    for angle in np.arange(directions)*math.pi/directions:   # the ACF is point-symmetric
        uv = centre+radii[:, None]*np.array([math.cos(angle), math.sin(angle)])
        values = bilinear(acf, uv)
        below = np.flatnonzero(values <= level)
        if not len(below):
            lengths.append(math.inf)
            continue
        k = below[0]
        lengths.append(radii[k-1]+(values[k-1]-level)/(values[k-1]-values[k])*(radii[k]-radii[k-1]))
    return np.array(lengths)


def warp_matrix(kind, value):
    """In-plane warp of rectified content about a landmark for a yaw (deg) or height (%) error."""
    if kind == 'yaw':
        a = math.radians(value)
        return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return (1+value/100)*np.eye(2)


def smear_correlation(acf, acf_scale, template_px, scale, warp):
    """Zero-shift ensemble NCC predicted for a template warped about its centre.

    A point x of the template is displaced by (warp-I)x, so the expected normalised correlation
    is the texture autocorrelation averaged over those displacements. For a small rotation psi
    they fill a square of half-width W*psi; for a scale error eps, half-width W*eps. acf_scale is
    metres per ACF lag of the texture being matched. Returns None if the displacements leave the
    stored ACF lags.
    """
    xy = patch_grid((0., 0.), template_px, scale).reshape(-1, 2)
    lag = xy@(warp-np.eye(2)).T/acf_scale+(len(acf)-1)/2
    values = bilinear(acf, lag)
    return float(values.mean()) if np.all(np.isfinite(values)) else None


def pure_warp_ncc(template_px, scale, warp, rng, count=100, stretch=1.):
    """Measured zero-shift NCC between a finite map template and the same ground warped about
    the template centre (no camera, no noise, no search). Mean and SD over random locations."""
    offsets = patch_grid((0., 0.), template_px, scale).reshape(-1, 2)
    values = []
    for centre in rng.uniform(-1000., 1000., (count, 2)):
        a = terrain(*((centre+offsets)/stretch).T)
        b = terrain(*((centre+offsets@warp.T)/stretch).T)
        values.append(np.corrcoef(a, b)[0, 1])
    return float(np.mean(values)), float(np.std(values))


def texture_analysis(seed=SEED, names=('FINE', 'COARSE', 'COARSE_TEX4')):
    """Correlation length of terrain and the W*psi / W*eps predictions for each matcher."""
    rng = np.random.default_rng([seed, 99])
    reference = .5                                    # m per lag of the reference ACF
    acf = autocorrelation(reference)                  # lags out to +/-64 m
    length_m = correlation_length(acf)*reference
    result = {'definition': 'radius at which the 2-D autocorrelation of terrain first falls to 1/e, '
                            'over 72 directions (unbiased FFT estimate on a 1024 x 1024 grid)',
              'acf_reference': {'scale_m': reference, 'grid': 1024, 'max_lag_m': 128*reference},
              'correlation_length_m': {'min': float(length_m.min()), 'mean': float(length_m.mean()),
                                       'max': float(length_m.max())}, 'matchers': {}}
    for name in names:
        cfg = CONFIGS[name]
        t, scale, stretch = cfg['template_px'], cfg['scale_m'], cfg.get('texture_stretch', 1.)
        half_px = t/2
        # The same estimate made directly on this matcher's map grid (coarsely sampled if short).
        grid_length = correlation_length(autocorrelation(scale/stretch, max_lag=32))
        length_px = float(length_m.mean()*stretch/scale)

        def predicted(kind, value):
            return smear_correlation(acf, reference*stretch, t, scale, warp_matrix(kind, value))

        def crossing(kind, upper, level=SCORE_GATE, steps=400):
            previous = (0., 1.)
            for value in np.linspace(0., upper, steps+1)[1:]:
                p = predicted(kind, value)
                if p is None:
                    return None
                if p < level:
                    return float(previous[0]+(previous[1]-level)/(previous[1]-p)*(value-previous[0]))
                previous = (value, p)
            return None
        tables = {}
        for kind in ('yaw', 'height'):
            rows = []
            for value in SWEEPS[kind]['values']:
                mean, sd = pure_warp_ncc(t, scale, warp_matrix(kind, value), rng, stretch=stretch)
                smear = (2*math.sin(math.radians(value)/2) if kind == 'yaw' else value/100)*half_px
                rows.append({'error': value, 'edge_smear_map_px': smear,
                             'predicted_ncc_acf_smear': predicted(kind, value),
                             'pure_warp_zero_shift_ncc_mean': mean, 'pure_warp_zero_shift_ncc_sd': sd})
            tables[kind] = rows
        result['matchers'][name] = {
            'texture_stretch': stretch,
            'template_half_width_map_px': half_px, 'template_half_width_m': half_px*scale,
            'correlation_length_map_px': length_px,
            'correlation_length_on_grid_map_px': {'min': float(grid_length.min()),
                                                  'mean': float(grid_length.mean()),
                                                  'max': float(grid_length.max())},
            'simple_rule_yaw_deg': math.degrees(length_px/half_px),
            'simple_rule_height_percent': 100*length_px/half_px,
            'acf_smear_ncc_half_yaw_deg': crossing('yaw', 60.),
            'acf_smear_ncc_half_height_percent': crossing('height', 100.),
            'yaw': tables['yaw'], 'height': tables['height']}
    return result


def levels(cfg, sweep):
    return tuple(SWEEPS[sweep]['values'])+tuple(cfg.get('extra', {}).get(sweep, ()))


def run(scenes_per_height=SCENES_PER_HEIGHT, configs=None, camera=CAMERA, seed=SEED, progress=False):
    """All cells. Returns (records by (config, height, sweep, error), scene descriptions)."""
    configs = CONFIGS if configs is None else configs
    cells = {}
    scenes = []
    for height in HEIGHTS:
        for index in range(scenes_per_height):
            scene = make_scene(seed, height, index, camera)
            true_pose = (scene['pos'], scene['r'])
            scenes.append({'height_m': height, 'index': index, 'position_m': scene['pos'].tolist(),
                           'yaw_deg': math.degrees(scene['yaw']), 'tilt_deg': math.degrees(scene['tilt']),
                           'tilt_dir_deg': math.degrees(scene['tilt_dir']),
                           'yaw_error_sign': scene['yaw_sign'], 'height_error_sign': scene['height_sign'],
                           'tilt_error_dir_deg': math.degrees(scene['tilt_error_dir']),
                           'position_error_dir_deg': math.degrees(scene['position_dir'])})
            images = {}
            for name, cfg in configs.items():
                if height not in cfg['heights'] or index >= cfg.get('scenes', scenes_per_height):
                    continue
                stretch = cfg.get('texture_stretch', 1.)
                if stretch not in images:
                    images[stretch] = scene_image(scene, camera, stretch)
                image = images[stretch]
                zero = evaluate(image, true_pose, true_pose, cfg, camera)
                for sweep in cfg['sweeps']:
                    for value in levels(cfg, sweep):
                        records = zero if value == 0 else evaluate(
                            image, true_pose, predicted_pose(scene, sweep, value), cfg, camera)
                        cells.setdefault((name, height, sweep, value), []).extend(records)
            if progress:
                print(f'  height {height:.0f} m scene {index+1}/{scenes_per_height}', flush=True)
    return cells, scenes


def cell_rows(cells):
    rows = []
    for (name, height, sweep, value), records in cells.items():
        row = {'config': name, 'height_m': height, 'sweep': sweep, 'error': value,
               'unit': SWEEPS[sweep]['unit'], 'scenes': len(records)//len(GRID)}
        row.update(summarise(records))
        rows.append(row)
    return rows


def threshold_rows(rows):
    """Per (config, height, sweep): thresholds with the gates ('gated') and for the raw NCC peak
    ('peak'), each on all in-view landmarks and on the subset whose true location was still
    inside the search window (which isolates decorrelation from displacement)."""
    out = []
    for name, height, sweep in dict.fromkeys((x['config'], x['height_m'], x['sweep']) for x in rows):
        subset = sorted((x for x in rows if (x['config'], x['height_m'], x['sweep']) == (name, height, sweep)),
                        key=lambda x: x['error'])
        errors = [x['error'] for x in subset]
        entry = {'config': name, 'height_m': height, 'sweep': sweep, 'unit': SWEEPS[sweep]['unit'],
                 'in_view_at_zero': subset[0]['in_view'], 'landmarks_attempted': subset[0]['landmarks_attempted']}
        for label, key, count in (('gated', 'correct_frac', 'in_view'),
                                  ('gated_truth_in_window', 'correct_frac_truth_in_window', 'truth_in_window'),
                                  ('peak', 'peak_correct_frac', 'in_view'),
                                  ('peak_truth_in_window', 'peak_correct_frac_truth_in_window', 'truth_in_window')):
            entry[label] = thresholds(errors, [x[key] if x[count] >= MIN_CELL else None for x in subset])
        entry['ncc_at_truth_half_crossing'] = thresholds(
            errors, [x['mean_ncc_at_truth'] if x['truth_in_window'] >= MIN_CELL else None for x in subset])['e50_interp']
        worst = max((x for x in subset if x['false_frac'] is not None), key=lambda x: x['false_frac'], default=None)
        entry['max_false_frac'] = worst['false_frac'] if worst else None
        entry['max_false_frac_at'] = worst['error'] if worst else None
        out.append(entry)
    return out


def write_csv(path, records):
    keys = list(dict.fromkeys(k for row in records for k in row))
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(records)


def fmt(value, spec='.3f', missing='   n/a'):
    return missing if value is None else format(value, spec)


def print_tables(rows, limits):
    for name, height, sweep in dict.fromkeys((x['config'], x['height_m'], x['sweep']) for x in rows):
        subset = [x for x in rows if (x['config'], x['height_m'], x['sweep']) == (name, height, sweep)]
        print(f"\n{name} h={height:.0f} m, {sweep} error [{subset[0]['unit']}]  (SYNTHETIC, {subset[0]['scenes']} frames)")
        print('   error  view  accept correct  false   ncc    rms_m | inwin corr|in  peak|in ncc@true')
        for x in sorted(subset, key=lambda x: x['error']):
            print(f"  {x['error']:6.2f} {x['in_view']:5d}  {fmt(x['accepted_frac'])}  {fmt(x['correct_frac'])}  "
                  f"{fmt(x['false_frac'])}  {fmt(x['mean_peak_ncc'])}  {fmt(x['rms_ground_error_correct_m'])} | "
                  f"{x['truth_in_window']:5d}  {fmt(x['correct_frac_truth_in_window'])}   "
                  f"{fmt(x['peak_correct_frac_truth_in_window'])}  {fmt(x['mean_ncc_at_truth'])}")
    print('\nThresholds: tol90 = largest tested level with >=90 % accepted-and-correct; e50 by interpolation')
    print('  (gated = existing gates; in-window = only landmarks whose truth stayed inside the search window;')
    print('   peak = raw NCC maximum at the right place, gates ignored; a trailing + means not reached)')
    for x in limits:
        def pair(label):
            t = x[label]
            return (f"{fmt(t['tol90_tested'], '.2f')}{'+' if t['censored90'] else ' '}/"
                    f"{fmt(t['e50_interp'], '.2f')}{'+' if t['censored50'] else ' '}")
        print(f"  {x['config']:12s} h={x['height_m']:3.0f} {x['sweep']:8s} [{x['unit']:7s}] view0={x['in_view_at_zero']:3d} "
              f"gated {pair('gated')} | in-window {pair('gated_truth_in_window')} | peak in-window "
              f"{pair('peak_truth_in_window')} | max false {fmt(x['max_false_frac'])} at {fmt(x['max_false_frac_at'], '.2f')}")


def clean(value):
    """JSON-safe copy: tuples to lists, NumPy scalars to Python, non-finite floats to None."""
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(value) else None
    return value


def main():
    start = time.perf_counter()
    OUT.mkdir(parents=True, exist_ok=True)
    print('SYNTHETIC NCC pose-error tolerance; camera: '+CAMERA['name'], flush=True)
    texture = texture_analysis()
    cells, scenes = run(progress=True)
    rows = cell_rows(cells)
    limits = threshold_rows(rows)
    print_tables(rows, limits)
    measured = {(x['config'], x['height_m'], x['sweep']): x for x in limits}
    for name, entry in texture['matchers'].items():
        entry['measured'] = [
            {'height_m': h, 'sweep': sweep, 'unit': SWEEPS[sweep]['unit'],
             'gated_e50_truth_in_window': measured[(name, h, sweep)]['gated_truth_in_window']['e50_interp'],
             'gated_tol90_truth_in_window': measured[(name, h, sweep)]['gated_truth_in_window']['tol90_tested'],
             'peak_e50_truth_in_window': measured[(name, h, sweep)]['peak_truth_in_window']['e50_interp'],
             'ncc_at_truth_half_crossing': measured[(name, h, sweep)]['ncc_at_truth_half_crossing'],
             'tested_to_truth_in_window': measured[(name, h, sweep)]['gated_truth_in_window']['tested_to']}
            for h in CONFIGS[name]['heights'] for sweep in ('yaw', 'height')]
    first_fix = []
    for x in limits:
        if x['sweep'] == 'yaw' and x['config'].startswith('COARSE') and x['in_view_at_zero'] >= MIN_CELL:
            tol = x['gated']['tol90_tested']
            first_fix.append({'config': x['config'], 'height_m': x['height_m'],
                              'yaw_tol90_tested_deg': tol, 'yaw_e90_interp_deg': x['gated']['e90_interp'],
                              'yaw_hypotheses_360_over_2tol': math.ceil(360/(2*tol)) if tol else None,
                              'note': None if tol else 'gated success is below 90 % at the smallest non-zero '
                                                       'tested yaw error (or at zero), so no count is defined'})
    print('\nTexture: 1/e correlation length %.2f m (min %.2f, max %.2f over direction)' % (
        texture['correlation_length_m']['mean'], texture['correlation_length_m']['min'],
        texture['correlation_length_m']['max']))
    for name, entry in texture['matchers'].items():
        print(f"  {name}: W={entry['template_half_width_map_px']:.0f} px, l={entry['correlation_length_map_px']:.2f} px, "
              f"rule l/W: yaw {entry['simple_rule_yaw_deg']:.1f} deg, height {entry['simple_rule_height_percent']:.1f} %; "
              f"ACF-smear NCC=0.5 at yaw {fmt(entry['acf_smear_ncc_half_yaw_deg'], '.1f')} deg, height "
              f"{fmt(entry['acf_smear_ncc_half_height_percent'], '.1f')} %")
        for x in entry['measured']:
            print(f"    measured h={x['height_m']:.0f} {x['sweep']:6s}: gated tol90 {fmt(x['gated_tol90_truth_in_window'], '.2f')}, "
                  f"gated e50 {fmt(x['gated_e50_truth_in_window'], '.2f')}, peak e50 {fmt(x['peak_e50_truth_in_window'], '.2f')}, "
                  f"NCC@truth=0.5 at {fmt(x['ncc_at_truth_half_crossing'], '.2f')} [{x['unit']}] (truth-in-window subset)")
        for kind in ('yaw', 'height'):
            for x in entry[kind]:
                print(f"    {kind:6s} {x['error']:6.2f}: smear {x['edge_smear_map_px']:6.2f} px, predicted "
                      f"{fmt(x['predicted_ncc_acf_smear'])}, pure-warp {x['pure_warp_zero_shift_ncc_mean']:.3f}")
    for x in first_fix:
        print(f"  first fix {x['config']} h={x['height_m']:.0f}: gated yaw tol90 {fmt(x['yaw_tol90_tested_deg'], '.1f')} deg -> "
              f"{x['yaw_hypotheses_360_over_2tol']} hypotheses")
    runtime = time.perf_counter()-start
    report = {
        'label': 'SYNTHETIC', 'date': DATE, 'seed': SEED, 'runtime_s': runtime,
        'python': platform.python_version(), 'numpy': np.__version__, 'host': platform.platform(),
        'camera': CAMERA, 'heights_m': HEIGHTS, 'scenes_per_height': SCENES_PER_HEIGHT,
        'noise_gray_sd': NOISE_SD, 'true_tilt_max_deg': MAX_TILT_DEG, 'true_yaw': 'uniform on +/-180 deg',
        'landmark_grid_ray_tangents': GRID, 'configs': CONFIGS, 'sweeps': SWEEPS,
        'gates': {'score_above': SCORE_GATE, 'peak_gap_above': GAP_GATE,
                  'gap_exclusion_px': GAP_EXCLUSION_PX, 'peak': 'interior concave quadratic (feasibility.quadratic_peak)'},
        'definitions': {
            'landmarks': 'map points under a fixed 3x3 pixel grid according to the PREDICTED pose (9 per frame)',
            'in_view': 'the rectified patch (search patch for FINE, template for COARSE*) lies wholly inside the image',
            'fractions': 'of in-view landmarks unless the name says otherwise',
            'correct': 'accepted, and the matched image pixel back-projected through the TRUE pose lies within '
                       f'{CORRECT_MAP_PX} map pixels of the matched map point',
            'false': 'accepted but not correct',
            'peak_correct': 'as correct but ignoring the gates (is the raw NCC maximum at the right place)',
            'truth_in_window': 'the true offset is at least one map pixel inside the search window',
            'mean_ncc_at_truth': 'best NCC in the 3x3 scores around the true offset, truth-in-window landmarks only',
            'errors': 'yaw about the world vertical through the camera, tilt about a horizontal world axis, '
                      'height as a fraction of true height, position horizontal; random sign/direction per '
                      'frame, fixed across levels (paired)',
            'thresholds': f'tol90_tested is the largest tested level with >=90 % at it and all smaller levels; '
                          f'e50/e90 are linear interpolations; cells with fewer than {MIN_CELL} samples cut the curve'},
        'cells': rows, 'thresholds': limits, 'texture': texture, 'first_fix_yaw_search': first_fix,
        'scenes': scenes, 'limits': LIMITS}
    (OUT/'ncc_attitude_tolerance.json').write_text(json.dumps(clean(report), indent=1, allow_nan=False),
                                                   encoding='utf-8')
    write_csv(OUT/'ncc_attitude_tolerance.csv', rows)
    print('\nLimits:')
    for line in LIMITS:
        print('  - '+line)
    print(f'\nRuntime {runtime:.1f} s; wrote {OUT}', flush=True)


if __name__ == '__main__':
    main()
