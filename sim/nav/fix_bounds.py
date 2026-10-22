"""GPL-3.0-only. Single-image pose bounds and the proposal's error budget, evaluated.

Cramér-Rao bound for one image (proposal Appendix H.5): with n landmarks at
known map positions, the Fisher information for [position, attitude] is
J = sum H_i^T R_i^-1 H_i, and no unbiased single-image estimate can beat J^-1.
Comparing that bound with and without attitude knowledge quantifies how well
perspective separates tilt from translation (Appendix C) for the camera we
actually plan to fly, and what the section 12 budget's h*sigma_theta term
means once the landmarks themselves are allowed to correct attitude.

Analytic and SYNTHETIC: geometry and assumed noise only, no images.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'code'))
from cansat.eskf import LEVEL, Camera, exp_so3  # noqa: E402

DEG = math.pi/180
IMAGE = (1456, 1088)
FOCAL = 2.5e-3/3.45e-6


def camera():
    return Camera(FOCAL, FOCAL, IMAGE[0]/2, IMAGE[1]/2, IMAGE[0], IMAGE[1])


def grid_pixels(fraction=1.0, cells=(6, 5), margin=16.0):
    """Cell-centre pixels of the landmark selection grid used by the Monte Carlo."""
    w, h = IMAGE
    x0, x1 = w/2*(1 - fraction) + margin, w/2*(1 + fraction) - margin
    y0, y1 = h/2*(1 - fraction) + margin, h/2*(1 + fraction) - margin
    xs = x0 + (np.arange(cells[0]) + 0.5)*(x1 - x0)/cells[0]
    ys = y0 + (np.arange(cells[1]) + 0.5)*(y1 - y0)/cells[1]
    return np.array([(x, y) for y in ys for x in xs])


def ground_points(cam, pixels, p, r):
    """Where the pixels' rays meet the plane z = 0."""
    rays = cam.ray(pixels) @ r.T
    origin = p + r @ cam.lever
    return origin + rays*(-origin[2]/rays[:, 2])[:, None]


def information(cam, points, p, r, pixel_floor=0.3, ground_noise=0.15, map_var=None):
    """6 x 6 Fisher information for [dp, dtheta] from one image."""
    _, depth, jp, jt = cam.jacobians(points, p, r)
    info = np.zeros((6, 6))
    for i in range(len(points)):
        h = np.hstack((jp[i], jt[i]))
        noise = np.eye(2)*(pixel_floor**2 + (ground_noise*cam.fx/depth[i])**2)
        if map_var is not None:
            noise = noise + (jp[i]*np.asarray(map_var)) @ jp[i].T
        info += h.T @ np.linalg.solve(noise, h)
    return info


def horizontal_sigma(info, attitude_sigma=None):
    """Root-sum-square east/north position bound.

    attitude_sigma: None for no outside attitude knowledge, 0 for perfectly
    known attitude, otherwise an independent prior of that 1-sigma (rad) on
    each attitude axis, as left by gyro propagation from the previous image.
    """
    if attitude_sigma == 0:
        cov = np.linalg.inv(info[:3, :3])
    else:
        j = info.copy()
        if attitude_sigma is not None:
            j[3:, 3:] += np.eye(3)/attitude_sigma**2
        cov = np.linalg.inv(j)
    return math.sqrt(cov[0, 0] + cov[1, 1])


def attitude_sigma_from_image(info):
    """Tilt (rss of the two horizontal axes) and yaw bound in degrees, position unknown."""
    cov = np.linalg.inv(info)
    return math.sqrt(cov[3, 3] + cov[4, 4])/DEG, math.sqrt(cov[5, 5])/DEG


def budget(height, attitude_sigma_deg, map_sigma=2.5, resolution=0.5, peak_px=0.2, landmarks=50,
           swing_rate_dps=40.0, timing_s=0.5e-3, relief=2.0, off_nadir_deg=25.0):
    """The five terms of proposal section 12, in metres, and their root sum square."""
    terms = {
        'attitude': height*attitude_sigma_deg*DEG,
        'matching': resolution*peak_px/math.sqrt(landmarks),
        'georeferencing': map_sigma,
        'timing': height*swing_rate_dps*DEG*timing_s,
        'relief': relief*math.tan(off_nadir_deg*DEG),
    }
    terms['total'] = math.sqrt(sum(v*v for v in terms.values()))
    return terms


def table(heights=(300.0, 250.0, 200.0, 150.0, 100.0, 80.0), tilt_deg=8.0):
    cam = camera()
    r = LEVEL @ exp_so3(np.array([tilt_deg*DEG, 0.0, 0.0]))
    rows = []
    for h in heights:
        p = np.array([0.0, 0.0, h])
        row = {'height_m': h}
        for label, fraction in (('full', 1.0), ('half', 0.5), ('third', 1/3)):
            pts = ground_points(cam, grid_pixels(fraction), p, r)
            info = information(cam, pts, p, r)
            tilt, yaw = attitude_sigma_from_image(info)
            row[label] = {
                'position_sigma_free_attitude_m': horizontal_sigma(info),
                'position_sigma_attitude_prior_0p1deg_m': horizontal_sigma(info, 0.1*DEG),
                'position_sigma_known_attitude_m': horizontal_sigma(info, 0),
                'tilt_sigma_deg': tilt, 'yaw_sigma_deg': yaw,
                # What section 12's attitude term would give for that tilt.
                'height_times_tilt_sigma_m': h*tilt*DEG,
            }
        pts = ground_points(cam, grid_pixels(1.0), p, r)
        row['full_with_1m_relief_m'] = horizontal_sigma(information(cam, pts, p, r, map_var=(0.09, 0.09, 1.0)))
        row['full_with_5m_relief_m'] = horizontal_sigma(information(cam, pts, p, r, map_var=(0.09, 0.09, 25.0)))
        rows.append(row)
    return rows


def main():
    rows = table()
    out = HERE/'results'/'nav_mc'
    out.mkdir(parents=True, exist_ok=True)
    report = {'label': 'SYNTHETIC, analytic', 'camera': {'image_px': IMAGE, 'focal_px': FOCAL},
              'assumptions': '30 landmarks on a 6 x 5 image grid, 0.3 px floor plus 0.15 m on the ground, '
                             'camera tilted 8 degrees, flat ground, known correspondences',
              'single_image_bounds': rows,
              'proposal_budget': {str(h): {str(a): budget(h, a) for a in (0.3, 0.5, 1.0)} for h in (300.0, 100.0)}}
    (out/'fix_bounds.json').write_text(json.dumps(report, indent=1))
    print('height | full FOV: free / 0.1deg prior / known attitude | tilt sigma | half FOV free | third FOV free | relief 1 m | 5 m')
    for row in rows:
        f = row['full']
        print('%5.0f m | %6.3f / %6.3f / %6.3f m | %.4f deg | %7.3f m | %7.3f m | %6.3f | %6.3f' % (
            row['height_m'], f['position_sigma_free_attitude_m'], f['position_sigma_attitude_prior_0p1deg_m'],
            f['position_sigma_known_attitude_m'], f['tilt_sigma_deg'], row['half']['position_sigma_free_attitude_m'],
            row['third']['position_sigma_free_attitude_m'], row['full_with_1m_relief_m'], row['full_with_5m_relief_m']))


if __name__ == '__main__':
    main()
