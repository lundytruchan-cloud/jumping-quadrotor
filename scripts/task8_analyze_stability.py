#!/usr/bin/env python3
"""Task 8: analyse the authors' StabilityTests data (paper Fig. 6C/S9).

For every landing of the three strategies (controller only = no_damper,
stabilizer only = no_controller, combined = with_damper) the script
computes the landing velocity direction angle ``theta_z``, the signed
landing attitude ``phi`` (paper Eq. 1-2), the lateral drift and the
number of hops before divergence.  The results are written as CSVs and
overlaid on the reproduced Poincare map (Fig. 6C style).
"""

import argparse
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))

from hopcopter_model.evaluate import (  # noqa: E402
    body_z_world,
    detect_jump_crossings,
    load_mocap,
    select_jumps,
)
from hopcopter_model.params import AUTHORS_PARAMS  # noqa: E402
from hopcopter_model.poincare import poincare_ratio_map  # noqa: E402

G = 9.81

STRATEGY_MAP = {
    'with_damper': 'combined',
    'no_damper': 'controller_only',
    'no_controller': 'stabilizer_only',
}
COLORS = {
    'combined': 'white',
    'controller_only': 'red',
    'stabilizer_only': 'blue',
}


def angle_deg(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return np.rad2deg(np.arctan2(np.linalg.norm(np.cross(a, b)),
                                 np.dot(a, b)))


def _slope_pre_impact(t, x, idx):
    """Pre-impact velocity: linear fit of the samples before touchdown.

    The crossing sample itself can sit at the bottom of the landing bounce
    (vertical velocity fitted across the impact is corrupted), so the
    landing velocity is estimated from the 4 samples strictly before the
    crossing.
    """
    idx = int(idx)
    lo = max(0, idx - 3)
    window = slice(lo, idx + 1)
    xm = np.vstack([t[window], np.ones(idx + 1 - lo)])
    slope = np.linalg.lstsq(xm.T, x[window], rcond=None)[0]
    return float(slope[0])


def analyze_file(path, strategy, out_rows):
    mocap = load_mocap(path)
    detect = detect_jump_crossings(mocap)
    pairs = select_jumps(detect)
    z_b = body_z_world(mocap)
    e3 = np.array([0.0, 0.0, 1.0])
    t = mocap.t

    hops = []
    for ld_i, _to_i in pairs:
        idx = int(detect['index'][ld_i])
        t_ld = float(t[idx])
        v = np.array([
            _slope_pre_impact(t, mocap.x, idx),
            _slope_pre_impact(t, mocap.y, idx),
            _slope_pre_impact(t, mocap.z, idx),
        ])
        zb = z_b[idx]
        speed = float(np.linalg.norm(v))
        if speed < 1e-6:
            continue
        theta_z = angle_deg(e3, -v)
        theta_ld = angle_deg(zb, -v)
        tilt = angle_deg(zb, e3)
        phi = tilt * (1.0 if theta_z >= theta_ld else -1.0)
        hops.append({
            'file': path.stem,
            'strategy': strategy,
            't_LD': t_ld,
            'theta_z_deg': theta_z,
            'phi_deg': phi,
            'tilt_deg': tilt,
            'lateral_speed': float(np.hypot(v[0], v[1])),
            'landing_speed': speed,
        })
        out_rows.append(hops[-1])

    return hops


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir',
                    default=str(REPO / 'data' / 'task8' / 'StabilityTests'))
    ap.add_argument('--out-dir', default=str(REPO / 'docs' / 'data'))
    args = ap.parse_args()
    data_dir = Path(args.data_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    out_rows = []
    summaries = []
    for group, strategy in STRATEGY_MAP.items():
        group_dir = data_dir / group
        if not group_dir.is_dir():
            print('missing group dir: {}'.format(group_dir))
            continue
        for mat in sorted(group_dir.glob('*.mat')):
            hops = analyze_file(mat, strategy, out_rows)
            max_tilt = max((h['tilt_deg'] for h in hops), default=0.0)
            max_theta = max((h['theta_z_deg'] for h in hops), default=0.0)
            summaries.append({
                'file': mat.stem,
                'strategy': strategy,
                'landings': len(hops),
                'max_theta_z_deg': max_theta,
                'max_tilt_deg': max_tilt,
                'max_lateral_speed': max(
                    (h['lateral_speed'] for h in hops), default=0.0),
            })
            print('{:28s} {:16s} landings={:3d} max_theta_z={:5.1f} deg '
                  'max_tilt={:5.1f} deg'.format(
                      mat.stem, strategy, len(hops), max_theta, max_tilt))

    # Pair consecutive landings per file for the next-cycle ratio.
    ratio_rows = []
    for i, h in enumerate(out_rows):
        if (i + 1 < len(out_rows)
                and out_rows[i + 1]['file'] == h['file']
                and out_rows[i + 1]['strategy'] == h['strategy']):
            ratio = out_rows[i + 1]['theta_z_deg'] / max(
                h['theta_z_deg'], 1e-9)
            ratio_rows.append({**h, 'ratio_next': ratio})
        else:
            ratio_rows.append({**h, 'ratio_next': float('nan')})

    hop_csv = out_dir / 'task8_stability_hops.csv'
    if ratio_rows:
        keys = list(ratio_rows[0].keys())
        with open(hop_csv, 'w', newline='') as f:
            import csv
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(ratio_rows)

    sum_csv = out_dir / 'task8_stability_summary.csv'
    if summaries:
        import csv
        with open(sum_csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(summaries[0].keys()))
            w.writeheader()
            w.writerows(summaries)

    # Overlay the landing states on the Poincare map (Fig. 6C style).
    thetas = np.arange(0.0, 41.0, 1.0)
    phis = np.arange(-55.0, 81.0, 1.0)
    m = poincare_ratio_map(thetas, phis, np.sqrt(2.0 * G * 0.5),
                           AUTHORS_PARAMS)
    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    ax.contourf(m['theta_z_deg'], m['phi_deg'],
                np.clip(m['ratio'], 0.0, 3.0),
                levels=np.linspace(0.0, 3.0, 15), cmap='summer')
    ax.contour(m['theta_z_deg'], m['phi_deg'], np.clip(m['ratio'], 0.0, 3.0),
               levels=[1.0], colors='white', linewidths=1.6)
    ax.plot(thetas, np.zeros_like(thetas), 'r--', linewidth=1.4)
    ax.plot(thetas, thetas, 'b--', linewidth=1.4)
    for strategy, color in COLORS.items():
        pts = [(h['theta_z_deg'], h['phi_deg']) for h in ratio_rows
               if h['strategy'] == strategy]
        if pts:
            pts = np.asarray(pts)
            ax.plot(pts[:, 0], pts[:, 1], 'o', markersize=4,
                    markerfacecolor=color, markeredgecolor='black',
                    markeredgewidth=0.4,
                    label=strategy.replace('_', ' '))
    ax.set_xlim(0, 40)
    ax.set_ylim(-10, 35)
    ax.set_xlabel(r'$\theta_{z}|_{k}$ (deg)')
    ax.set_ylabel(r'$\phi|_{k}$ (deg)')
    ax.set_title('StabilityTests landing states on the Poincare map')
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    png = out_dir / 'task8_stability_fig6c_overlay.png'
    fig.savefig(png, dpi=150)
    print('saved: {}, {}, {}'.format(hop_csv, sum_csv, png))
    return 0


if __name__ == '__main__':
    sys.exit(main())
