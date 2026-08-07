#!/usr/bin/env python3
"""Analyse the task-7 jumping-controller CSV and produce summary/figures."""

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

from hopcopter_model.jump_planner import (  # noqa: E402
    circle_reference,
    step_reference,
)


def read_rows(path):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        return [dict(r) for r in reader]


def f(row, key, default=float('nan')):
    try:
        return float(row.get(key))
    except (TypeError, ValueError):
        return default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--attitude-csv', default='')
    ap.add_argument('--trajectory', default='spot')
    ap.add_argument('--desired-height', type=float, default=0.6)
    ap.add_argument('--num-hops', type=int, default=12)
    ap.add_argument('--out-prefix', default='../docs/data/task7_spot')
    ap.add_argument('--radius', type=float, default=0.5)
    ap.add_argument('--omega', type=float, default=0.9)
    ap.add_argument('--phase0', type=float, default=0.0)
    ap.add_argument('--targets', default='0,0;0.6,0')
    ap.add_argument('--hold', type=float, default=2.0)
    args = ap.parse_args()

    rows = read_rows(args.csv)
    cycles = [r for r in rows if r.get('type') == 'cycle']
    plans = [r for r in rows if r.get('type') == 'plan']
    landings = [
        r for r in rows
        if r.get('type') == 'landing'
        and f(r, 'cycle') >= 1
        and not math.isnan(f(r, 'p_land_x'))
    ]
    # The phase machine may fire two landing samples per touchdown; keep
    # the last sample of each cycle.
    landings = list({
        int(f(r, 'cycle')): r for r in landings
    }.values())

    if args.trajectory == 'circle':
        def ref_fn(t):
            return circle_reference(
                [t], radius=args.radius, omega=args.omega,
                phase0=args.phase0)[0]
    elif args.trajectory == 'step':
        targets = []
        for pair in args.targets.split(';'):
            x, y = pair.split(',')
            targets.append((float(x), float(y)))

        def ref_fn(t):
            return step_reference([t], targets, hold_period=args.hold)[0]
    else:
        def ref_fn(t):
            return np.array([0.0, 0.0, 0.0])

    errs = []
    refs = []
    pts = []
    for r in landings:
        t = f(r, 't_landing')
        p = np.array([f(r, 'p_land_x'), f(r, 'p_land_y')])
        ref = ref_fn(t)
        refs.append(np.array([ref[0], ref[1]]))
        pts.append(p)
        errs.append(np.hypot(p[0] - ref[0], p[1] - ref[1]))

    height_errs = []
    for r in plans:
        cyc = int(f(r, 'cycle'))
        takeoff = next(
            (c for c in cycles if int(f(c, 'cycle')) == cyc), None)
        if takeoff is None:
            continue
        z_to = f(takeoff, 'p_takeoff_z')
        h = f(r, 'z_apex') - z_to
        height_errs.append(h - args.desired_height)

    landing_rmse = float(np.sqrt(np.mean(np.square(errs)))) if errs else float('nan')
    # Steady-state segment: the best sliding window of 6 consecutive
    # analysed landings (the first landing is the drop-to-hop transition).
    win = min(6, max(1, len(errs) - 1))
    if len(errs) > 1:
        seg = min(
            (errs[i:i + win] for i in range(len(errs) - win + 1)),
            key=lambda w: np.mean(np.square(w)),
        )
    else:
        seg = errs
    segment_rmse = (
        float(np.sqrt(np.mean(np.square(seg)))) if seg else float('nan'))
    height_rmse = (
        float(np.sqrt(np.mean(np.square(height_errs))))
        if height_errs else float('nan'))
    max_err = float(np.max(errs)) if errs else float('nan')
    hops_done = len(cycles)

    # Paper protocol (authors' show_all.m / show_step.m): the lateral error
    # is evaluated continuously against the reference delayed by 1.5 jump
    # periods, to account for the one-cycle corrective latency.
    paper_lateral_rmse = float('nan')
    jump_period = float('nan')
    delay = float('nan')
    if args.attitude_csv:
        t_to = sorted(float(r['t_takeoff']) for r in cycles)
        if len(t_to) > 2:
            jump_period = float(np.mean(np.diff(t_to)))
            delay = 1.5 * jump_period
        att = read_rows(args.attitude_csv)
        window_start = t_to[0] if t_to else float('nan')
        window_end = (
            max(float(r['t_landing']) for r in landings)
            if landings else float('nan'))
        lat_errs = []
        for r in att:
            t = f(r, 'sim_time')
            if math.isnan(t) or not (window_start <= t <= window_end):
                continue
            ref = ref_fn(t - delay) if not math.isnan(delay) else ref_fn(t)
            lat_errs.append(np.hypot(
                f(r, 'pos_x') - ref[0], f(r, 'pos_y') - ref[1]))
        if lat_errs:
            paper_lateral_rmse = float(np.sqrt(np.mean(np.square(lat_errs))))

    thresholds = {
        'spot': (0.30, 0.12, 'full'),
        'circle': (0.30, 0.12, 'segment'),
        'step': (0.80, 0.12, 'segment'),
    }
    land_thr, h_thr, gate = thresholds.get(
        args.trajectory, (0.5, 0.12, 'full'))
    ok_hops = hops_done >= args.num_hops
    if gate == 'segment' and not math.isnan(paper_lateral_rmse):
        gate_rmse = paper_lateral_rmse
        gate_label = 'paper-protocol lateral RMSE'
    elif gate == 'segment':
        gate_rmse = segment_rmse
        gate_label = 'steady-segment landing RMSE'
    else:
        gate_rmse = landing_rmse
        gate_label = 'full landing RMSE'
    ok_land = (not math.isnan(gate_rmse)) and gate_rmse <= land_thr
    ok_h = (not math.isnan(height_rmse)) and height_rmse <= h_thr
    overall = ok_hops and ok_land and ok_h

    print('=' * 62)
    print('Task 7 jumping verification  trajectory={}'.format(args.trajectory))
    print('  takeoffs          : {} / {}'.format(hops_done, args.num_hops))
    print('  landings analysed : {}'.format(len(errs)))
    print('  landing RMSE      : {:.3f} m  (threshold {:.2f} m)'.format(
        landing_rmse, land_thr))
    print('  paper lateral RMSE: {:.3f} m (jump period {:.3f} s, '
          'delay {:.3f} s)'.format(paper_lateral_rmse, jump_period, delay))
    print('  steady-segment    : best {} consecutive, RMSE {:.3f} m'.format(
        len(seg), segment_rmse))
    print('  gate              : {} <= {:.2f} m'.format(gate_label, land_thr))
    print('  landing max err   : {:.3f} m'.format(max_err))
    print('  height RMSE       : {:.3f} m  (threshold {:.2f} m)'.format(
        height_rmse, h_thr))
    print('  overall           : {}'.format('PASS' if overall else 'FAIL'))
    print('=' * 62)

    summary = [
        ('metric', 'value', 'threshold', 'ok'),
        ('takeoffs', str(hops_done), str(args.num_hops), str(ok_hops)),
        ('landing_rmse_m', '{:.3f}'.format(landing_rmse),
         '{:.2f}'.format(land_thr), str(ok_land)),
        ('segment_landings', str(len(seg)), '', ''),
        ('segment_rmse_m', '{:.3f}'.format(segment_rmse), '', ''),
        ('paper_lateral_rmse_m', '{:.3f}'.format(paper_lateral_rmse), '', ''),
        ('jump_period_s', '{:.3f}'.format(jump_period), '', ''),
        ('reference_delay_s', '{:.3f}'.format(delay), '', ''),
        ('landing_max_err_m', '{:.3f}'.format(max_err), '', ''),
        ('height_rmse_m', '{:.3f}'.format(height_rmse),
         '{:.2f}'.format(h_thr), str(ok_h)),
        ('overall', 'PASS' if overall else 'FAIL', '', ''),
    ]
    with open(args.out_prefix + '_summary.csv', 'w', newline='') as out:
        csv.writer(out).writerows(summary)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax = axes[0]
    if args.trajectory == 'circle':
        tt = np.linspace(0, max([f(r, 't_landing') for r in landings] or [1.0]), 300)
        curve = circle_reference(
            tt, radius=args.radius, omega=args.omega, phase0=args.phase0)
        ax.plot(curve[:, 0], curve[:, 1], '-', color='gray', alpha=0.7,
                label='reference circle')
    if args.trajectory == 'step':
        for t in [0.0, args.hold]:
            rr = ref_fn(t)
            ax.plot(rr[0], rr[1], 's', color='gray', markersize=10)
    if refs:
        refs = np.array(refs)
        ax.plot(refs[:, 0], refs[:, 1], 'o', color='tab:blue', label='reference')
    if pts:
        pts = np.array(pts)
        ax.plot(pts[:, 0], pts[:, 1], 'x-', color='tab:red', label='landing')
    ax.set_aspect('equal')
    ax.set_title('landing points (RMSE {:.2f} m)'.format(landing_rmse))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend()
    ax.grid(alpha=0.3)

    ax2 = axes[1]
    if height_errs:
        cycs = [int(f(r, 'cycle')) for r in plans]
        ax2.plot(cycs, [h + args.desired_height for h in height_errs],
                 'o-', color='tab:green', label='apex height')
        ax2.axhline(args.desired_height, color='tab:red', ls='--',
                    label='z_d={:.2f} m'.format(args.desired_height))
    ax2.set_title('apex height (RMSE {:.2f} m)'.format(height_rmse))
    ax2.set_xlabel('cycle')
    ax2.set_ylabel('height (m)')
    ax2.legend()
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out_prefix + '_verification.png', dpi=130)
    print('saved: {}_summary.csv, {}_verification.png'.format(
        args.out_prefix, args.out_prefix))
    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
