#!/usr/bin/env python3
"""Analyse the task-8 simulation runs (three strategies + 30-hop test).

Inputs are the jump-controller CSV (takeoffs/landings per cycle), the
attitude-control CSV (full-state log) and the strategy/hop-count used.
The script reports the number of completed hops, whether the run crashed
before the target, the landing-state (theta_z, phi) per hop on the
Poincare map, the max lateral drift and the stabilizer activation
evidence, and writes a summary CSV + figure.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))

from hopcopter_model.evaluate import quat_to_rotm  # noqa: E402

E3 = np.array([0.0, 0.0, 1.0])


def read_rows(path):
    if not Path(path).is_file():
        return []
    with open(path, newline='') as f:
        return [dict(r) for r in csv.DictReader(f)]


def f(row, key, default=float('nan')):
    try:
        return float(row.get(key))
    except (TypeError, ValueError):
        return default


def angle_deg(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return np.rad2deg(np.arctan2(np.linalg.norm(np.cross(a, b)),
                                 np.dot(a, b)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--jump-csv', required=True)
    ap.add_argument('--attitude-csv', default='')
    ap.add_argument('--strategy', default='combined')
    ap.add_argument('--num-hops', type=int, default=30)
    ap.add_argument('--out-prefix', default='../docs/data/task8_combined')
    args = ap.parse_args()

    rows = read_rows(args.jump_csv)
    cycles = [r for r in rows if r.get('type') == 'cycle']
    landings = [r for r in rows if r.get('type') == 'landing']
    landings = list({
        int(f(r, 'cycle')): r for r in landings
    }.values())
    hops_done = len(cycles)
    success = hops_done >= args.num_hops

    att_rows = read_rows(args.attitude_csv)
    att_t = np.array([f(r, 'sim_time') for r in att_rows])
    att_x = np.array([f(r, 'pos_x') for r in att_rows])
    att_y = np.array([f(r, 'pos_y') for r in att_rows])
    att_z = np.array([f(r, 'pos_z') for r in att_rows])
    att_q = np.array([
        [f(r, 'qw'), f(r, 'qx'), f(r, 'qy'), f(r, 'qz')]
        for r in att_rows
    ])

    drift = []
    states = []
    for r in landings:
        cyc = int(f(r, 'cycle'))
        t_land = f(r, 't_landing')
        p_land = np.array([f(r, 'p_land_x'), f(r, 'p_land_y')])
        if not math.isnan(p_land[0]) and p_land[0] != 0.0:
            drift.append(np.hypot(p_land[0], p_land[1]))
        elif len(att_rows) and not math.isnan(t_land):
            idx = int(np.argmin(np.abs(att_t - t_land)))
            drift.append(np.hypot(att_x[idx], att_y[idx]))
            # Landing state from the attitude log (pre-impact velocity).
            lo = max(0, idx - 3)
            if idx - lo >= 2:
                w = slice(lo, idx + 1)
                xm = np.vstack([att_t[w], np.ones(idx + 1 - lo)])
                v = np.array([
                    np.linalg.lstsq(xm.T, att_x[w], rcond=None)[0][0],
                    np.linalg.lstsq(xm.T, att_y[w], rcond=None)[0][0],
                    np.linalg.lstsq(xm.T, att_z[w], rcond=None)[0][0],
                ])
            else:
                v = np.zeros(3)
            q = att_q[idx]
            q1 = att_q[idx:idx + 1]
            rotm = quat_to_rotm(q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3])[0]
            zb = rotm[:, 2]
            if np.linalg.norm(v) > 1e-6 and np.linalg.norm(zb) > 1e-6:
                theta_z = angle_deg(E3, -v)
                theta_ld = angle_deg(zb, -v)
                tilt = angle_deg(zb, E3)
                phi = tilt * (1.0 if theta_z >= theta_ld else -1.0)
                states.append({
                    'cycle': cyc,
                    'theta_z_deg': theta_z,
                    'phi_deg': phi,
                    'tilt_deg': tilt,
                    'lateral_speed': float(np.hypot(v[0], v[1])),
                })

    max_drift = float(max(drift)) if drift else float('nan')
    mean_drift = float(np.mean(drift)) if drift else float('nan')
    stab_rows = [r for r in landings if r.get('stabilizer_active') == 'True']
    stab_cycles = len(stab_rows)
    last_cycles = [r for r in cycles[-5:]]
    last_period = float('nan')
    if len(cycles) >= 2:
        t_to = sorted(f(r, 't_takeoff') for r in cycles)
        last_period = float(np.mean(np.diff(t_to[-5:])))

    arena_ok = (not math.isnan(max_drift)) and max_drift < 1.5
    # Task acceptance: the robot must complete the requested number of
    # consecutive hops without position feedback.  The 3x3 m arena bound
    # of the paper is reported separately (our sim world is unbounded).
    overall = success

    print('=' * 62)
    print('Task 8 simulation  strategy={} target={} hops'.format(
        args.strategy, args.num_hops))
    print('  takeoffs        : {} / {}'.format(hops_done, args.num_hops))
    print('  landings        : {}'.format(len(landings)))
    print('  stabilizer on at landing rows: {} / {}'.format(
        stab_cycles, len(landings)))
    print('  max lateral drift: {:.3f} m (paper 3x3 m arena: {})'.format(
        max_drift, 'OK' if arena_ok else 'exceeded'))
    print('  mean lateral drift: {:.3f} m'.format(mean_drift))
    print('  last jump period : {:.3f} s'.format(last_period))
    if states:
        th = [s['theta_z_deg'] for s in states]
        ph = [s['phi_deg'] for s in states]
        print('  landing theta_z : median {:.2f} deg, max {:.2f} deg'.format(
            np.median(th), np.max(th)))
        print('  landing phi     : median {:.2f} deg, max {:.2f} deg'.format(
            np.median(ph), np.max(np.abs(ph))))
    print('  overall         : {}'.format('PASS' if overall else 'FAIL'))
    print('=' * 62)

    summary = [
        ('metric', 'value', 'threshold', 'ok'),
        ('strategy', args.strategy, '', ''),
        ('takeoffs', str(hops_done), str(args.num_hops), str(success)),
        ('landings', str(len(landings)), '', ''),
        ('stabilizer_on_landing_rows', str(stab_cycles), '', ''),
        ('max_drift_m', '{:.3f}'.format(max_drift), '1.500', str(arena_ok)),
        ('arena_3x3_ok', str(arena_ok), '', ''),
        ('mean_drift_m', '{:.3f}'.format(mean_drift), '', ''),
        ('last_jump_period_s', '{:.3f}'.format(last_period), '', ''),
        ('median_theta_z_deg',
         '{:.2f}'.format(np.median([s['theta_z_deg'] for s in states]))
         if states else '', '', ''),
        ('median_phi_deg',
         '{:.2f}'.format(np.median([s['phi_deg'] for s in states]))
         if states else '', '', ''),
        ('overall', 'PASS' if overall else 'FAIL', '', ''),
    ]
    with open(args.out_prefix + '_summary.csv', 'w', newline='') as out:
        csv.writer(out).writerows(summary)
    if states:
        with open(args.out_prefix + '_landing_states.csv', 'w',
                  newline='') as out:
            w = csv.DictWriter(out, fieldnames=list(states[0].keys()))
            w.writeheader()
            w.writerows(states)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    ax = axes[0]
    if len(att_rows):
        ax.plot(att_x, att_y, '-', color='tab:blue', alpha=0.6, lw=0.8)
    if landings:
        pts = []
        for r in landings:
            cyc = int(f(r, 'cycle'))
            t_land = f(r, 't_landing')
            if not math.isnan(t_land) and len(att_rows):
                idx = int(np.argmin(np.abs(att_t - t_land)))
                pts.append((att_x[idx], att_y[idx], cyc))
            else:
                pts.append((f(r, 'p_land_x'), f(r, 'p_land_y'), cyc))
        pts = np.asarray(pts)
        ax.plot(pts[:, 0], pts[:, 1], 'x-', color='tab:red',
                label='landing')
    ax.set_aspect('equal')
    ax.set_title('xy trajectory (max drift {:.2f} m)'.format(max_drift))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(alpha=0.3)
    ax.legend()

    ax2 = axes[1]
    if len(att_rows):
        ax2.plot(att_t, att_z, '-', color='tab:green', lw=0.8)
    if cycles:
        t_takeoff = [f(r, 't_takeoff') for r in cycles]
        ax2.plot(t_takeoff, [0.22] * len(t_takeoff), 'v', color='tab:red',
                 label='takeoff')
    if landings:
        t_land = [f(r, 't_landing') for r in landings]
        ax2.plot(t_land, [0.22] * len(t_land), '^', color='tab:blue',
                 label='landing')
    ax2.set_title('height ({} hops)'.format(hops_done))
    ax2.set_xlabel('sim time (s)')
    ax2.set_ylabel('z (m)')
    ax2.grid(alpha=0.3)
    ax2.legend()
    fig.tight_layout()
    png = args.out_prefix + '_verification.png'
    fig.savefig(png, dpi=130)
    print('saved: {}_summary.csv, {}'.format(args.out_prefix, png))
    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
