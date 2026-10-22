"""GPL-3.0-only. Auditable mathematical reference models, not flight software.

SI units; z increases upwards and s increases downwards. Only NumPy is needed.
"""
import math
import numpy as np

F = np.array([[0., 1.], [-1., -2.]])
B = np.array([[0.], [1.]])
C = np.array([1., math.sqrt(3.)])
P_INF = np.eye(2) / 4
G0 = 9.80665


def sym(a):
    return (a + a.T) / 2


def expm(a):
    """Scaling/squaring with a converged Taylor series for small reference matrices."""
    a = np.asarray(a, dtype=float)
    scale = max(0, int(math.ceil(math.log2(max(np.linalg.norm(a, 1), 1.)))))
    b = a / 2**scale
    out = np.eye(len(a))
    term = out.copy()
    for k in range(1, 100):
        term = term @ b / k
        out += term
        if np.linalg.norm(term, 1) < 2e-16 * np.linalg.norm(out, 1):
            break
    else:
        raise ArithmeticError('matrix exponential did not converge')
    for _ in range(scale):
        out = out @ out
    return out


def discretize(a, diffusion, ds):
    """Exact for constant coefficients: Van Loan block exponential."""
    n = len(a)
    block = np.zeros((2*n, 2*n))
    block[:n, :n] = a
    block[:n, n:] = diffusion
    block[n:, n:] = -a.T
    e = expm(block * ds)
    phi = e[:n, :n]
    return phi, sym(e[:n, n:] @ phi.T)


def psd_root(p):
    vals, vecs = np.linalg.eigh(sym(p))
    if vals.min() < -1e-9 * max(1., vals.max()):
        raise ArithmeticError('non-PSD covariance')
    return vecs @ np.diag(np.sqrt(np.maximum(vals, 0.)))


def stationary_cov(a, diffusion):
    n = len(a)
    # C-order vectorization: vec_C(AP+PA^T)=(A⊗I+I⊗A)vec_C(P).
    op = np.kron(a, np.eye(n)) + np.kron(np.eye(n), a)
    return sym(np.linalg.solve(op, -diffusion.reshape(-1)).reshape(n, n))


def transverse(r):
    r = np.abs(np.asarray(r))
    return (1 - r/2) * np.exp(-r)


def warp(z, a=.5, linf=None):
    z = np.asarray(z)
    if np.any(z <= 0) or a <= 0:
        raise ValueError('warp needs positive height and scale')
    return np.log(z) / a + (0 if linf is None else z / linf)


def wind_cov(z, sigma=1., a=.5, linf=None):
    q = warp(z, a, linf)
    sigma = np.broadcast_to(sigma, len(q))
    return sigma[:, None]*sigma[None, :]*transverse(q[:, None]-q[None, :])


def lag_matrix(n, ds, ell):
    """w_j constant on cell j; u sampled at end of cell, u_initial=0."""
    decay = math.exp(-ds / ell)
    k = np.zeros((n, n))
    for i in range(n):
        k[i, :i+1] = (1-decay) * decay**np.arange(i, -1, -1)
    return k


def retrieval(k, sa, se):
    sy = k @ sa @ k.T + se
    gain = np.linalg.solve(sy, k @ sa).T
    post = sym(sa - gain @ k @ sa)
    return gain, gain @ k, post


def state_model(length, ell, vd, sigma=1.):
    """[x1,x2,u,I] in descended distance; I is drift in metres."""
    if min(length, ell, vd) <= 0:
        raise ValueError('positive scales required')
    a = np.zeros((4, 4))
    a[:2, :2] = F / length
    a[2, :2] = sigma * C / ell
    a[2, 2] = -1 / ell
    a[3, 2] = 1 / vd
    d = np.zeros((4, 4))
    d[:2, :2] = B @ B.T / length
    return a, d


def step_model(length, ell, vd, ds, mu=0., sigma=1.):
    a, d = state_model(length, ell, vd, sigma)
    phi, q = discretize(a, d, ds)
    aug = np.zeros((5, 5))
    aug[:4, :4] = a
    aug[2, 4] = mu / ell
    b = expm(aug * ds)[:4, 4]
    return phi, q, b


def kalman_update(mean, cov, y, h, noise):
    """Means may be (state, ensemble). Covariance is shared across ensembles."""
    h = np.atleast_2d(h)
    noise = np.atleast_2d(noise)
    s = h @ cov @ h.T + noise
    gain = np.linalg.solve(s, h @ cov).T
    innovation = np.atleast_1d(y) - (h @ mean).squeeze(axis=0)
    if mean.ndim == 2:
        updated = mean + gain @ innovation[None, :]
    else:
        updated = mean + gain @ np.atleast_1d(innovation)
    eye = np.eye(len(cov)) - gain @ h
    p = sym(eye @ cov @ eye.T + gain @ noise @ gain.T)
    return updated, p, innovation, s


def augment_ar1_step(phi, q, b, rho, noise_sd):
    """Append a stationary AR(1) measurement-error state in m/s.

    rho is per observation interval (not a continuous-time decay rate).
    Observation y=u+e has no additional white noise unless supplied separately.
    """
    if abs(rho) >= 1 or noise_sd <= 0:
        raise ValueError('stationary AR1 requires |rho|<1 and positive SD')
    n=len(phi)
    augmented_phi=np.zeros((n+1,n+1))
    augmented_phi[:n,:n]=phi; augmented_phi[n,n]=rho
    augmented_q=np.zeros_like(augmented_phi)
    augmented_q[:n,:n]=q; augmented_q[n,n]=noise_sd**2*(1-rho**2)
    return augmented_phi, augmented_q, np.r_[b,0.]


def rts_smooth(filtered_means, filtered_covs, predicted_means, predicted_covs, phis):
    """Linear-Gaussian RTS smoother, including ensemble-valued means."""
    ms=[m.copy() for m in filtered_means]
    ps=[p.copy() for p in filtered_covs]
    gains=[None]*len(phis)
    for i in range(len(phis)-1,-1,-1):
        j=np.linalg.solve(predicted_covs[i+1],phis[i]@filtered_covs[i]).T
        gains[i]=j
        ms[i]=filtered_means[i]+j@(ms[i+1]-predicted_means[i+1])
        ps[i]=sym(filtered_covs[i]+j@(ps[i+1]-predicted_covs[i+1])@j.T)
    return ms,ps,gains


def smoother_cross_cov(i,j,covs,gains):
    """Cov(X_i,X_j | all observations) = J_i ... J_(j-1) P_j^s."""
    if i>j:
        return smoother_cross_cov(j,i,covs,gains).T
    product=np.eye(len(covs[i]))
    for k in range(i,j): product=product@gains[k]
    return product@covs[j]


def layer_posterior_variance(indices, weights, output, covs, gains):
    """Variance of a weighted linear output; retains all cross-height terms."""
    indices=list(indices); weights=np.asarray(weights)
    output=np.asarray(output)
    total=0.
    for a,i in enumerate(indices):
        total+=weights[a]**2*(output@covs[i]@output)
        for b,j in enumerate(indices[a+1:],start=a+1):
            total+=2*weights[a]*weights[b]*(output@smoother_cross_cov(i,j,covs,gains)@output)
    return float(total)


def retrieval_with_initial_velocity(k,b,sa,cross,var_initial,se):
    """Marginal wind posterior for y=K w+b u0+noise, with Cov(w,u0)=cross."""
    b=np.asarray(b); cross=np.asarray(cross)
    kcross=k@cross
    sy=k@sa@k.T+se+var_initial*np.outer(b,b)+np.outer(kcross,b)+np.outer(b,kcross)
    swy=sa@k.T+np.outer(cross,b)
    gain=np.linalg.solve(sy,swy.T).T
    return gain,sym(sa-gain@swy.T)


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(v):
    angle = np.linalg.norm(v)
    vsk = skew(v)
    if angle < 1e-10:
        return np.eye(3) + vsk + vsk @ vsk / 2
    return np.eye(3) + math.sin(angle)/angle*vsk + (1-math.cos(angle))/angle**2*(vsk@vsk)


def project(point, pos, rbw, rcb, fx=640., fy=640., cx=640., cy=400.):
    pc = rcb @ rbw.T @ (point - pos)
    if pc[2] <= 0:
        raise ValueError('point behind camera')
    return np.array([fx*pc[0]/pc[2]+cx, fy*pc[1]/pc[2]+cy])


def reprojection_jacobians(point, pos, rbw, rcb, fx=640., fy=640.):
    pb = rbw.T @ (point-pos)
    x, y, z = rcb @ pb
    j = np.array([[fx/z, 0, -fx*x/z**2], [0, fy/z, -fy*y/z**2]])
    return -j @ rcb @ rbw.T, j @ rcb @ skew(pb)


def ncc_fft(image, template):
    """Valid linear NCC with FFT numerator and integral-image normalization.

    Constant patches yield NaN. Memory is O(padded image pixels), rather than
    O(valid shifts * template pixels); do not materialize sliding windows.
    """
    image = np.asarray(image, dtype=float)
    template = np.asarray(template, dtype=float)
    if image.ndim != 2 or template.ndim != 2 or not np.all(np.isfinite(image)) or not np.all(np.isfinite(template)):
        raise ValueError('NCC needs finite two-dimensional arrays')
    ih, iw = image.shape
    th, tw = template.shape
    if min(th, tw) < 1 or th > ih or tw > iw:
        raise ValueError('nonempty template must fit inside image')
    shape = tuple(1 << (n-1).bit_length() for n in (ih+th-1, iw+tw-1))
    # Remove a global DC offset to reduce cancellation in variance sums.
    image = image-image.mean()
    tc = template-template.mean()
    raw = np.fft.irfft2(np.fft.rfft2(image, shape)*np.conj(np.fft.rfft2(tc, shape)), s=shape)
    def window_sums(a):
        integral = np.pad(a.cumsum(axis=0).cumsum(axis=1), ((1, 0), (1, 0)))
        return integral[th:, tw:]-integral[:-th, tw:]-integral[th:, :-tw]+integral[:-th, :-tw]
    sums, squares = window_sums(image), window_sums(image*image)
    energy = np.maximum(squares-sums*sums/(th*tw), 0.)
    # Integral subtraction can leave roundoff at a mathematically constant patch.
    tolerance = 64*np.finfo(float).eps*np.maximum(squares, np.max(squares))
    energy[energy <= tolerance] = 0.
    den = np.sqrt(energy*np.sum(tc**2))
    return np.divide(raw[:ih-th+1, :iw-tw+1], den, out=np.full(den.shape, np.nan), where=den>0)


def rk4(fun, y, dt, steps):
    y = np.array(y, dtype=float)
    for i in range(steps):
        t = i * dt
        k1 = fun(t, y)
        k2 = fun(t+dt/2, y+dt*k1/2)
        k3 = fun(t+dt/2, y+dt*k2/2)
        k4 = fun(t+dt, y+dt*k3)
        y += dt*(k1+2*k2+2*k3+k4)/6
    return y
