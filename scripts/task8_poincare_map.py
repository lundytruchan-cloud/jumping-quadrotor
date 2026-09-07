#!/usr/bin/env python3
"""Task 8: reproduce the paper's Fig. 6C Poincare map and stable region.

The map is a direct port of the authors' ``run_this_0.m`` (hopping height
0.5 m): ``theta_z|k+1/theta_z|k`` as a function of the landing velocity
angle ``theta_z|k`` and the landing attitude ``phi|k``.  The stable region
is ``ratio < 1`` (plus the trivial fixed point at the origin).  The script
also cross-checks the map against ``variables_results_05.mat`` and writes
the grid CSV + figure used in the task-8 report.
"""

import argparse
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import scipy.io

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))

from hopcopter_model.params import AUTHORS_PARAMS  # noqa: E402
from hopcopter_model.poincare import (  # noqa: E402
    fixed_point_locus,
    poincare_ratio_map,
    stability_region,
)

G = 9.81
DATA = REPO / 'data' / 'task8' / 'AerodynamicStabilizer'


def compare_with_authors(m, authors_mat):
    """Max absolute difference against the authors' saved grid (deg)."""
    d = scipy.io.loadmat(authors_mat)
    X = d['X']
    N = d['next_landing_theta']
    # Authors: rows = alpha (body-to-velocity angle), cols = theta_z.
    alphas_deg = np.rad2deg(d['Y'][:, 0])
    thetas_deg = np.rad2deg(d['X'][0, :])
    phi_deg = thetas_deg[None, :] - alphas_deg[:, None]
    our = m['next_theta_z_deg']
    phi_idx = np.round(phi_deg - m['phi_deg'][0, 0]).astype(int)
    theta_idx = np.round(
        thetas_deg[None, :] - m['theta_z_deg'][0, 0]).astype(int)
    valid = (
        (phi_idx >= 0) & (phi_idx < m['phi_deg'].shape[0])
        & (theta_idx >= 0) & (theta_idx < m['theta_z_deg'].shape[1])
    )
    if not valid.all():
        return float('nan'), 'grid does not cover the authors grid'
    our_sampled = our[phi_idx, theta_idx]
    diff = np.abs(our_sampled - np.rad2deg(N))
    return float(np.nanmax(diff)), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=str(REPO / 'docs' / 'data'))
    ap.add_argument('--author-mat', default=str(DATA / 'variables_results_05.mat'))
    args = ap.parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    thetas = np.arange(0.0, 41.0, 1.0)
    # phi = theta_z - alpha with theta_z in [0, 40] and alpha in [-40, 55]
    # (the authors' grid) spans [-55, 80].
    phis = np.arange(-55.0, 81.0, 1.0)
    v_land = np.sqrt(2.0 * G * 0.5)
    m = poincare_ratio_map(thetas, phis, v_land, AUTHORS_PARAMS)
    stable = stability_region(m['ratio'], m['theta_z_deg'], m['phi_deg'])

    # Fixed-point locus (authors' fminbnd search, alpha in deg).
    alphas = list(range(-2, 2)) + list(range(8, 36))
    crosses = fixed_point_locus(alphas, v_land, AUTHORS_PARAMS)
    locus_theta = crosses
    locus_phi = crosses - np.asarray(alphas, dtype=float)

    # Compare with the authors' saved results (same 0.5 m grid).
    max_diff, err = compare_with_authors(m, args.author_mat)
    if err:
        print('author comparison skipped: {}'.format(err))
    else:
        print('max |ours - authors| on the Fig. 6C grid: {:.4f} deg'
              .format(max_diff))

    # CSV grid (long format).
    rows = np.column_stack([
        m['theta_z_deg'].ravel(),
        m['phi_deg'].ravel(),
        m['next_theta_z_deg'].ravel(),
        m['ratio'].ravel(),
        stable.ravel().astype(int),
    ])
    grid_csv = out_dir / 'task8_poincare_map.csv'
    header = 'theta_z_deg,phi_deg,next_theta_z_deg,ratio,stable'
    np.savetxt(grid_csv, rows, delimiter=',', header=header,
               comments='', fmt='%.8f')

    locus_csv = out_dir / 'task8_poincare_fixed_points.csv'
    np.savetxt(locus_csv, np.column_stack([alphas, locus_theta, locus_phi]),
               delimiter=',', header='alpha_deg,theta_z_star_deg,phi_star_deg',
               comments='', fmt='%.8f')

    # Figure 6C style.
    ratio_clip = np.clip(m['ratio'], 0.0, 3.0)
    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    cf = ax.contourf(m['theta_z_deg'], m['phi_deg'], ratio_clip,
                     levels=np.linspace(0.0, 3.0, 15),
                     cmap='summer', extend='max')
    # Stability boundary: ratio = 1.
    ax.contour(m['theta_z_deg'], m['phi_deg'], ratio_clip,
               levels=[1.0], colors='white', linewidths=1.6)
    # Controller-only (phi = 0) and stabilizer-only (phi = theta_z) lines.
    ax.plot(thetas, np.zeros_like(thetas), 'r--', linewidth=1.6,
            label=r'$\phi_k = 0$ (controller only)')
    ax.plot(thetas, thetas, 'b--', linewidth=1.6,
            label=r'$\phi_k = \theta_{z,k}$ (stabilizer only)')
    # Fixed-point locus.
    ax.plot(locus_theta, locus_phi, 'k-', linewidth=2.0,
            label='fixed-point locus')
    ax.set_xlim(0, 40)
    ax.set_ylim(-10, 35)
    ax.set_xlabel(r'$\theta_{z}|_{k}$ (deg)')
    ax.set_ylabel(r'$\phi|_{k}$ (deg)')
    ax.set_title('Poincare map at 0.5 m hopping height '
                 '(stable region ratio < 1)')
    cb = fig.colorbar(cf, ax=ax)
    cb.set_label(r'$\theta_{z}|_{k+1} / \theta_{z}|_{k}$')
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    png = out_dir / 'task8_poincare_fig6c.png'
    fig.savefig(png, dpi=150)
    print('saved: {}, {}'.format(grid_csv, png))

    # Summary of the stable region.
    between = (m['phi_deg'] >= 0.0) & (m['phi_deg'] <= m['theta_z_deg'])
    frac_stable_between = float(np.mean(stable[between]))
    frac_stable_all = float(np.mean(stable))
    print('stable fraction (full grid): {:.3f}'.format(frac_stable_all))
    print('stable fraction between phi=0 and phi=theta_z: {:.3f}'
          .format(frac_stable_between))
    with open(out_dir / 'task8_poincare_summary.txt', 'w') as f:
        f.write('hopping_height_m 0.5\n')
        f.write('max_abs_diff_vs_authors_deg {:.6f}\n'.format(max_diff))
        f.write('stable_fraction_full_grid {:.4f}\n'.format(frac_stable_all))
        f.write('stable_fraction_between_dashed_lines {:.4f}\n'
                .format(frac_stable_between))
    return 0


if __name__ == '__main__':
    sys.exit(main())
