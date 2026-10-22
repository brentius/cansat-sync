"""GPL-3.0-only. Synthetic engineering models for the attached maths reference.

Only NumPy is required. SI units, upward world z, right/local rotation errors.
These are verification models, not flight software or calibrated physical laws.
"""
import math
import numpy as np
from models import G0, rotation, skew, sym, discretize, ncc_fft
from test_navigation_extra import right_jacobian, rotation_log


def eskf_matrices(r, force, omega, densities=(.03, .002, .001, .0001)):
    """15 errors [p,v,theta,ba,bg]; densities are assumed sqrt(PSD), SI."""
    f = np.zeros((15, 15)); g = np.zeros((15, 12))
    f[:3, 3:6] = np.eye(3)
    f[3:6, 6:9] = -r@skew(force); f[3:6, 9:12] = -r
    f[6:9, 6:9] = -skew(omega); f[6:9, 12:15] = -np.eye(3)
    g[3:6, :3] = -r; g[6:9, 3:6] = -np.eye(3)
    g[9:12, 6:9] = np.eye(3); g[12:15, 9:12] = np.eye(3)
    d = g@np.diag(np.repeat(np.square(densities), 3))@g.T
    return f, d


def gaussian_update(mean, p, y, h, v, cross=None):
    """Vector/ensemble update, including state/observation noise cross terms."""
    cross = np.zeros((len(p), len(v))) if cross is None else cross
    s = sym(h@p@h.T+h@cross+cross.T@h.T+v)
    cov = p@h.T+cross
    gain = np.linalg.solve(s, cov.T).T
    return mean+gain@(y-h@mean), sym(p-gain@cov.T), s


def camera_pixels(points, pos, r, focal=320., size=(400, 640)):
    """No lever arm in rendered-image experiment; tested separately in old suite."""
    pc = (points-pos)@r
    uv = focal*pc[:, :2]/pc[:, 2, None]+np.array([size[1]/2, size[0]/2])
    return uv, pc[:, 2]


def pose_jacobian(points, pos, r, focal=320.):
    pb = (points-pos)@r
    rows = []
    for x, y, z in pb:
        j = np.array([[focal/z, 0., -focal*x/z**2], [0., focal/z, -focal*y/z**2]])
        rows.append(np.column_stack((-j@r.T, j@skew([x, y, z]))))
    return np.concatenate(rows)


def fit_pose(points, observed, pos, r, pixel_sd=.5, focal=320., iterations=8):
    """Local Gauss-Newton reprojection fit with fixed accepted correspondences.

    Reports local likelihood covariance. Does not model datum or association error.
    """
    pos = pos.copy(); r = r.copy()
    for _ in range(iterations):
        predicted, depths = camera_pixels(points, pos, r, focal)
        if np.any(depths <= 0): raise ValueError('pose fit placed point behind camera')
        h = pose_jacobian(points, pos, r, focal)
        delta = np.linalg.lstsq(h, (observed-predicted).reshape(-1), rcond=None)[0]
        if np.linalg.norm(delta[3:]) > .3 or np.linalg.norm(delta[:3]) > 50:
            raise ValueError('pose increment outside local convergence region')
        pos += delta[:3]; r = r@rotation(delta[3:])
        if np.linalg.norm(delta) < 1e-8: break
    h = pose_jacobian(points, pos, r, focal)
    info = h.T@h/pixel_sd**2
    if np.linalg.matrix_rank(info) < 6: raise ValueError('unobservable pose')
    return pos, r, np.linalg.solve(info, np.eye(6))


def terrain(x, y, repetitive=False):
    """Continuous procedural radiance, generated independently of camera warp."""
    if repetitive:
        return 128+45*np.cos(2*np.pi*x/8)+20*np.cos(2*np.pi*y/8)
    result = np.full(np.broadcast_shapes(np.shape(x), np.shape(y)), 128.)
    for a, kx, ky, phase in [(24,.35,.11,.3),(19,-.17,.57,1.2),
                             (14,.81,.23,2.3),(12,-.43,.93,.9),
                             (9,1.73,-.61,2.1),(7,.71,1.51,.7)]:
        result += a*np.sin(kx*x+ky*y+phase)
    return result


def surface_height(x, y, amplitude):
    return amplitude*np.sin(x/17)*np.cos(y/23)


def render_image(pos, r, size=(400, 640), focal=320., relief=0., repetitive=False):
    """Ray/height-field intersections, rather than a ground-plane homography.

    Fixed-point intersection is safe for the shallow analytic relief used here;
    no occlusion model. Pixel centres follow the document's integer convention.
    """
    yy, xx = np.indices(size)
    rays = np.stack(((xx-size[1]/2)/focal, (yy-size[0]/2)/focal, np.ones(size)), axis=-1)@r.T
    distance = -pos[2]/rays[..., 2]
    for _ in range(8 if relief else 0):
        x = pos[0]+distance*rays[..., 0]; y = pos[1]+distance*rays[..., 1]
        distance = (surface_height(x, y, relief)-pos[2])/rays[..., 2]
    x = pos[0]+distance*rays[..., 0]; y = pos[1]+distance*rays[..., 1]
    return terrain(x, y, repetitive)


def bilinear(image, uv):
    """Returns NaN beyond the interpolation domain; never wraps image borders."""
    x, y = uv[..., 0], uv[..., 1]
    valid = (x>=0)&(y>=0)&(x<image.shape[1]-1)&(y<image.shape[0]-1)
    xi = np.clip(np.floor(x).astype(int), 0, image.shape[1]-2)
    yi = np.clip(np.floor(y).astype(int), 0, image.shape[0]-2)
    dx, dy = x-xi, y-yi
    result = ((1-dx)*(1-dy)*image[yi, xi]+dx*(1-dy)*image[yi, xi+1]
              +(1-dx)*dy*image[yi+1, xi]+dx*dy*image[yi+1, xi+1])
    return np.where(valid, result, np.nan)


def rectify_patch(image, centre, pos, r, size=64, scale=.5, focal=320.):
    xy = (np.arange(size)-(size-1)/2)*scale
    xx, yy = np.meshgrid(xy+centre[0], xy+centre[1])
    points = np.stack((xx, yy, np.zeros_like(xx)), axis=-1)
    uv, _ = camera_pixels(points.reshape(-1, 3), pos, r, focal, image.shape)
    return bilinear(image, uv.reshape(size, size, 2))


def quadratic_peak(scores):
    """Joint 2-D quadratic fit at an interior maximum; offset in (x,y)."""
    if not np.any(np.isfinite(scores)): return None
    y, x = np.unravel_index(np.nanargmax(scores), scores.shape)
    if x in (0, scores.shape[1]-1) or y in (0, scores.shape[0]-1): return None
    yy, xx = np.mgrid[-1:2, -1:2]
    design = np.column_stack((np.ones(9), xx.ravel(), yy.ravel(),
                             (xx*xx).ravel()/2, (xx*yy).ravel(), (yy*yy).ravel()/2))
    coeff = np.linalg.lstsq(design, scores[y-1:y+2, x-1:x+2].ravel(), rcond=None)[0]
    hess = np.array([[coeff[3], coeff[4]], [coeff[4], coeff[5]]])
    if np.linalg.eigvalsh(hess).max() >= -1e-8: return None
    delta = -np.linalg.solve(hess, coeff[1:3])
    if np.max(abs(delta)) > 1: return None
    return np.array([x, y])+delta


def image_matches(image, landmarks, pos, r, repetitive=False, scale=.5):
    """Fine-mode demonstration: 32 px templates, +/-8m search at .5m/px.

    Uses original-image observations obtained from the predicted rectification.
    The narrower window than the proposal is explicit; the benchmark uses its
    full +/-16m window. A peak-gap gate rejects obvious periodic ambiguity.
    """
    records = []; observed = []; points = []
    xy = (np.arange(32)-15.5)*scale
    for j, landmark in enumerate(landmarks):
        patch = rectify_patch(image, landmark, pos, r, scale=scale)
        record = {'landmark':j, 'accepted':False, 'reason':'out_of_view'}
        if np.all(np.isfinite(patch)):
            xx, yy = np.meshgrid(xy+landmark[0], xy+landmark[1])
            scores = ncc_fft(patch, terrain(xx, yy, repetitive))
            peak = quadratic_peak(scores)
            iy, ix = np.unravel_index(np.nanargmax(scores), scores.shape)
            other = scores.copy()
            other[max(0,iy-3):iy+4, max(0,ix-3):ix+4] = np.nan
            gap = float(scores[iy,ix]-np.nanmax(other))
            record.update(score=float(scores[iy,ix]), gap=gap, reason='ambiguous_or_boundary')
            if peak is not None and scores[iy,ix]>.5 and gap>.04:
                shift = (peak-16)*scale
                uv, _ = camera_pixels(np.array([landmark+np.r_[shift,0.]]), pos, r)
                observed.append(uv[0]); points.append(landmark)
                record.update(accepted=True, reason='accepted', shift_x_m=float(shift[0]), shift_y_m=float(shift[1]))
        records.append(record)
    return np.array(points), np.array(observed), records


def queue_simulation(service_seconds, arrival_interval=.2, workers=1):
    """FIFO scheduling model; service is assumed, not measured board timing."""
    ends = np.zeros(workers); starts = []; finishes = []
    for j, service in enumerate(service_seconds):
        worker = int(np.argmin(ends)); start = max(j*arrival_interval, ends[worker])
        ends[worker] = start+service; starts.append(start); finishes.append(ends[worker])
    arrival = np.arange(len(service_seconds))*arrival_interval
    return np.array(starts)-arrival, np.array(finishes)-arrival


def overlap_weights(edges, low, high):
    if high <= low or np.any(np.diff(edges)<=0): raise ValueError('ordered positive-width cells required')
    weights = np.maximum(np.minimum(edges[1:], high)-np.maximum(edges[:-1], low), 0)/(high-low)
    if not np.isclose(weights.sum(), 1): raise ValueError('grid does not cover layer')
    return weights


def tls_certificate(points, targets, caps):
    """Construct a valid analytic SDP dual bound, without installing a solver.

    x=[1,q(4),w(n),v(4n)]. Q objective, A_i binary equalities. With
    y_i=-cap_i the slack is sums of residual squares + cap_i*(1-w_i)^2,
    hence PSD and lower bound 0. Tight on noiseless all-inlier data only.
    """
    n = len(points); dim = 5+5*n; qmat = np.zeros((dim, dim)); slack = qmat.copy()
    for i, ((ux,uy), m, cap) in enumerate(zip(points, targets, caps)):
        a = np.array([[ux,-uy,1,0],[uy,ux,0,1.]])
        residual = np.zeros((2,dim)); residual[:,5+i] = m
        residual[:,5+n+4*i:5+n+4*i+4] = -a
        qmat += residual.T@residual
        qmat[0,0] += cap; qmat[0,5+i] -= cap/2; qmat[5+i,0] -= cap/2
        binary = np.zeros_like(qmat); binary[5+i,5+i] = 1
        binary[0,5+i] = binary[5+i,0] = -.5
        slack += cap*binary
    return qmat, sym(qmat+slack), 0.


def lift_tls(q, switches):
    return np.r_[1., q, switches, np.concatenate([w*q for w in switches])]
