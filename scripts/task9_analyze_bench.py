#!/usr/bin/env python3
"""Task-9 benchmark analysis: Fig.4A period-height curve, agility, duty cycle.

Paper-side curve follows the authors' ``hopping_agility.m`` (AgilityEvaluation
package): seven time windows, jump height h = apex - trough, hopping
frequency f = 1/diff(apex times), agility nu = 2*h*f.

Simulation-side curve comes from the task-9 Gazebo spot-hopping runs
(``docs/data/task9_h{height}_jump.csv``); the duty cycle comes from the
commanded rotor speeds in the attitude logs, assuming rotor power
P ~ sum(speed^3) (momentum-theory scaling).
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
import scipy.io
from scipy.signal import find_peaks

REPO = Path(__file__).resolve().parents[1]

PAPER_WINDOWS = [
    (55.5, 68.0), (45.0, 55.5), (34.0, 45.0), (25.0, 34.0),
    (16.9, 25.0), (9.4, 16.9), (4.44, 9.4),
]


def fval(row, key, default=float('nan')):
    try:
        return float(row.get(key))
    except (TypeError, ValueError):
        return default


def read_rows(path):
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


def paper_curve(mat_path):
    """Port of ``hopping_agility.m`` (authors' Zenodo package)."""
    m = scipy.io.loadmat(mat_path)
    t = np.asarray(m['Abs_time']).ravel()
    z = np.asarray(m['b1_z']).ravel()
    rows = []
    for i, (a, b) in enumerate(PAPER_WINDOWS):
        idx = np.where((t >= a) & (t <= b))[0]
        seg_t, seg_z = t[idx], z[idx]
        pks, _ = find_peaks(seg_z)
        pks2, _ = find_peaks(-seg_z)
        if len(pks) > len(pks2):
            pks = pks[:-1]
        elif len(pks) < len(pks2):
            pks2 = pks2[:-1]
        if len(pks) < 2:
            continue
        apex = seg_z[pks]
        trough = seg_z[pks2]
        h = apex - trough
        t_apex = seg_t[pks]
        periods = np.diff(t_apex)
        freqs = 1.0 / periods
        h_mean = float(np.mean(h))
        h_sd = float(np.sqrt(np.sum((h - h_mean) ** 2) / len(h)))
        f_mean = float(np.mean(freqs))
        f_sd = float(np.sqrt(np.sum((freqs - f_mean) ** 2) / len(freqs)))
        t_mean = float(np.mean(periods))
        t_sd = float(np.std(periods))
        rows.append({
            'source': 'paper',
            'window': i + 1,
            'h_mean_m': h_mean,
            'h_sd_m': h_sd,
            'T_mean_s': t_mean,
            'T_sd_s': t_sd,
            'f_mean_hz': f_mean,
            'f_sd_hz': f_sd,
            'nu_2hf_m_s': 2.0 * h_mean * f_mean,
            'nu_2h_T_m_s': 2.0 * h_mean / t_mean,
            'n_hops': int(len(h)),
        })
    return rows


def sim_curve(jump_csv):
    """Period/height/agility from one task-9 spot run."""
    rows = read_rows(jump_csv)
    cycles = [r for r in rows if r.get('type') == 'cycle']
    plans = {int(fval(r, 'cycle')): r for r in rows
             if r.get('type') == 'plan' and not math.isnan(fval(r, 'z_apex'))}
    takeoffs = [(int(fval(r, 'cycle')), fval(r, 't_takeoff'),
                 fval(r, 'p_takeoff_z')) for r in cycles]
    takeoffs.sort()
    h_by_cycle = {}
    for cyc, t_to, z_to in takeoffs:
        plan = plans.get(cyc)
        if plan is not None:
            h_by_cycle[cyc] = fval(plan, 'z_apex') - z_to

    n = len(takeoffs)
    if n < 3:
        return None
    to_times = [t for _, t, _ in takeoffs]
    periods = np.diff(to_times)
    # Steady cycles: drop the drop-to-hop transition (cycle 1).
    steady_cycles = [c for c, _, _ in takeoffs if c >= 2]
    h_vals = np.array([h_by_cycle[c] for c in steady_cycles
                       if c in h_by_cycle], dtype=float)
    steady_mask = [(takeoffs[k][0] >= 2 and takeoffs[k + 1][0] >= 2)
                   for k in range(len(periods))]
    T_vals = np.array([p for p, ok in zip(periods, steady_mask) if ok],
                      dtype=float)
    if len(h_vals) < 2 or len(T_vals) < 2:
        return None
    h_mean = float(np.mean(h_vals))
    h_sd = float(np.std(h_vals))
    t_mean = float(np.mean(T_vals))
    t_sd = float(np.std(T_vals))
    freqs = 1.0 / T_vals
    f_mean = float(np.mean(freqs))
    return {
        'source': 'sim',
        'window': 0,
        'h_mean_m': h_mean,
        'h_sd_m': h_sd,
        'T_mean_s': t_mean,
        'T_sd_s': t_sd,
        'f_mean_hz': f_mean,
        'f_sd_hz': float(np.std(freqs)),
        'nu_2hf_m_s': 2.0 * h_mean * f_mean,
        'nu_2h_T_m_s': 2.0 * h_mean / t_mean,
        'n_hops': len(h_vals),
    }


def power_metrics(att_csv, t_start, t_end):
    """Mean rotor power proxies from the attitude log over [t_start, t_end]."""
    p_omega2 = []
    p_thrust = []
    n = 0
    for r in read_rows(att_csv):
        t = fval(r, 'sim_time')
        if math.isnan(t) or t < t_start or t > t_end:
            continue
        speeds = [fval(r, 'speed{}'.format(i)) for i in range(4)]
        forces = [fval(r, 'f{}'.format(i)) for i in range(4)]
        p_omega2.append(sum(max(0.0, s) ** 3 for s in speeds))
        p_thrust.append(sum(max(0.0, f) ** 1.5 for f in forces))
        n += 1
    if n == 0:
        return None
    return {
        'n_samples': n,
        'P_omega3_mean': float(np.mean(p_omega2)),
        'P_thrust15_mean': float(np.mean(p_thrust)),
    }


def thrust_duty_metrics(att_csv, t_start, t_end, mass=0.0348, g=9.80665):
    """Paper duty ratio D = mean(sum(f_i)) / mg (Eq. in Endurance section).

    Also returns the fraction of samples with sum(f_i) < 0.11*mg, matching
    the paper's statement that >85% of the time is spent with minimal
    attitude-control thrust outside the powered climb.
    """
    mg = mass * g
    thrusts = []
    low = 0
    for r in read_rows(att_csv):
        t = fval(r, 'sim_time')
        if math.isnan(t) or t < t_start or t > t_end:
            continue
        f = sum(max(0.0, fval(r, 'f{}'.format(i))) for i in range(4))
        thrusts.append(f / mg)
        if f < 0.11 * mg:
            low += 1
    if not thrusts:
        return None
    return {
        'D_mean': float(np.mean(thrusts)),
        'frac_lt_011mg': low / len(thrusts),
        'n_samples': len(thrusts),
    }


def hover_metrics(att_csv):
    rows = read_rows(att_csv)
    hover_rows = [r for r in rows if r.get('mode') == 'hover']
    if len(hover_rows) < 10:
        return None
    t0 = min(fval(r, 'sim_time') for r in hover_rows)
    t1 = max(fval(r, 'sim_time') for r in hover_rows)
    return power_metrics(att_csv, t0 + 1.0, t1 - 0.5)


def load_task7_rmse():
    def summary(prefix):
        path = REPO / 'docs' / 'data' / '{}_summary.csv'.format(prefix)
        if not path.exists():
            return None
        d = {r['metric']: r['value'] for r in read_rows(path)}
        return {
            'paper_lateral_rmse_m': float(d.get('paper_lateral_rmse_m', 'nan')),
            'landing_rmse_m': float(d.get('landing_rmse_m', 'nan')),
            'height_rmse_m': float(d.get('height_rmse_m', 'nan')),
            'jump_period_s': float(d.get('jump_period_s', 'nan')),
        }
    return {'circle': summary('task7_circle'),
            'step': summary('task7_step')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default=str(REPO / 'docs' / 'data'))
    ap.add_argument('--paper-mat', default=str(
        REPO / 'data' / 'task9' / 'AgilityEvaluation' / 'AgilityEvaluation'
        / '20230411_121902.mat'))
    ap.add_argument('--heights', default='0.6,0.8,1.0,1.2,1.4,1.63')
    ap.add_argument('--out-prefix', default=str(REPO / 'docs' / 'data'
                                                / 'task9'))
    args = ap.parse_args()

    out = Path(args.out_prefix)
    out.parent.mkdir(parents=True, exist_ok=True)
    data_dir = Path(args.data_dir)

    # 1) Paper curve.
    paper = paper_curve(args.paper_mat)
    with open(str(out) + '_paper_period_height.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(paper[0].keys()))
        w.writeheader()
        w.writerows(paper)

    # 2) Simulation curve.
    sim_rows = []
    for h in args.heights.split(','):
        jump_csv = data_dir / 'task9_h{}_jump.csv'.format(h)
        if not jump_csv.exists():
            print('[task9] missing {}'.format(jump_csv))
            continue
        r = sim_curve(jump_csv)
        if r is None:
            print('[task9] too few steady hops in {}'.format(jump_csv))
            continue
        r['window'] = h
        sim_rows.append(r)
    if not sim_rows:
        print('[task9] no simulation runs found')
        return 1

    with open(str(out) + '_sim_period_height.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(sim_rows[0].keys()))
        w.writeheader()
        w.writerows(sim_rows)

    # 3) Duty cycle (rotor-power proxies).
    hover = hover_metrics(data_dir / 'task9_hover.csv')
    duty_rows = []
    duty_thrust_rows = []
    for r in sim_rows:
        att_csv = data_dir / 'task9_h{}_attitude.csv'.format(r['window'])
        jump_csv = data_dir / 'task9_h{}_jump.csv'.format(r['window'])
        if not att_csv.exists() or hover is None:
            continue
        rows = read_rows(jump_csv)
        takeoffs = sorted(
            fval(x, 't_takeoff') for x in rows if x.get('type') == 'cycle')
        landings = sorted(
            fval(x, 't_landing') for x in rows
            if x.get('type') == 'landing' and not math.isnan(fval(x, 't_landing')))
        if len(takeoffs) < 3 or not landings:
            continue
        full = power_metrics(att_csv, takeoffs[0], max(landings))
        steady = power_metrics(att_csv, takeoffs[1], max(landings))
        if full is None:
            continue
        d_full = thrust_duty_metrics(att_csv, takeoffs[0], max(landings))
        d_steady = thrust_duty_metrics(att_csv, takeoffs[1], max(landings))
        duty_rows.append({
            'height_m': r['h_mean_m'],
            'setpoint_m': r['window'],
            'P_hover_omega3': hover['P_omega3_mean'],
            'P_hop_full_omega3': full['P_omega3_mean'],
            'duty_full': full['P_omega3_mean'] / hover['P_omega3_mean'],
            'endurance_multiplier_full': hover['P_omega3_mean'] / full['P_omega3_mean'],
            'P_hop_steady_omega3': (steady['P_omega3_mean']
                                    if steady else float('nan')),
            'duty_steady': ((steady['P_omega3_mean'] / hover['P_omega3_mean'])
                            if steady else float('nan')),
            'P_hop_full_thrust15': full['P_thrust15_mean'],
            'duty_full_thrust15': (full['P_thrust15_mean']
                                   / hover['P_thrust15_mean']),
            'n_samples': full['n_samples'],
        })
        if d_full is not None:
            duty_thrust_rows.append({
                'height_m': r['h_mean_m'],
                'setpoint_m': r['window'],
                'D_full': d_full['D_mean'],
                'D_steady': (d_steady['D_mean'] if d_steady else float('nan')),
                'frac_lt_011mg_full': d_full['frac_lt_011mg'],
                'frac_lt_011mg_steady': (d_steady['frac_lt_011mg']
                                         if d_steady else float('nan')),
            })
    if duty_rows:
        with open(str(out) + '_duty.csv', 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(duty_rows[0].keys()))
            w.writeheader()
            w.writerows(duty_rows)
    if duty_thrust_rows:
        with open(str(out) + '_duty_thrust.csv', 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(duty_thrust_rows[0].keys()))
            w.writeheader()
            w.writerows(duty_thrust_rows)

    # 4) Task-7 trajectory RMSE summary (paper-protocol values).
    rmse = load_task7_rmse()

    # 5) Figures.
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    ph = np.asarray([r['h_mean_m'] for r in paper])
    pt = np.asarray([r['T_mean_s'] for r in paper])
    ax.errorbar(ph, pt, xerr=[r['h_sd_m'] for r in paper],
                yerr=[r['T_sd_s'] for r in paper], fmt='o-', ms=5,
                color='tab:red', capsize=3, label='paper (author data)')
    sh = np.asarray([r['h_mean_m'] for r in sim_rows])
    st = np.asarray([r['T_mean_s'] for r in sim_rows])
    ax.errorbar(sh, st, xerr=[r['h_sd_m'] for r in sim_rows],
                yerr=[r['T_sd_s'] for r in sim_rows], fmt='s-', ms=5,
                color='tab:blue', capsize=3, label='simulation (Gazebo)')
    hgrid = np.linspace(0.4, 2.1, 100)
    ax.plot(hgrid, 2.0 * np.sqrt(2.0 * hgrid / 9.80665), '--', lw=1,
            color='gray', label=r'$2\sqrt{2h/g}$ (ballistic lower bound)')
    ax.set_xlabel('jump height $h$ (m)')
    ax.set_ylabel('jump period $T$ (s)')
    ax.set_title('Fig. 4A reproduction: jumping period vs height')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(str(out) + '_fig4a_period_height.png', dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    ax.errorbar(ph, [2.0 * r['h_mean_m'] * r['f_mean_hz'] for r in paper],
                xerr=[r['h_sd_m'] for r in paper], fmt='o-', ms=5,
                color='tab:red', capsize=3, label='paper (2hf)')
    ax.errorbar(sh, [r['nu_2hf_m_s'] for r in sim_rows],
                xerr=[r['h_sd_m'] for r in sim_rows], fmt='s-', ms=5,
                color='tab:blue', capsize=3, label='simulation (2hf)')
    ax.axhline(2.38, color='tab:red', ls=':', lw=1.2,
               label='paper best 2.38 m/s @ 1.63 m')
    ax.set_xlabel('jump height $h$ (m)')
    ax.set_ylabel(r'agility $\nu = 2hf$ (m/s)')
    ax.set_title('Fig. 4B equivalent: hopping agility')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(str(out) + '_fig4b_agility.png', dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    if duty_thrust_rows:
        hh = [r['height_m'] for r in duty_thrust_rows]
        ax.plot(hh, [r['D_full'] for r in duty_thrust_rows], 'o-',
                color='tab:blue',
                label=r'simulation $D=\overline{\Sigma f_i}/mg$')
        ax.axhline(0.28, color='tab:red', ls='--',
                   label='paper D = 0.28')
        ax.axhline(1.0, color='gray', ls=':', label='hover reference')
    ax.set_xlabel('jump height $h$ (m)')
    ax.set_ylabel(r'duty ratio $D$ (net thrust / $mg$)')
    ax.set_title('Endurance duty ratio (paper definition, simulation)')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(str(out) + '_duty.png', dpi=140)
    plt.close(fig)

    # 6) Text summary.
    lines = []
    lines.append('=' * 70)
    lines.append('Task 9 benchmark summary')
    lines.append('=' * 70)
    lines.append('')
    lines.append('Paper Fig.4A (author hopping_agility.m reprocessed):')
    lines.append('  window | h_mean (m) | T_mean (s) | f_mean (Hz) | nu=2hf (m/s)')
    for r in paper:
        lines.append('   {:<5} | {:8.3f} | {:8.3f} | {:8.3f} | {:8.3f}'.format(
            r['window'], r['h_mean_m'], r['T_mean_s'], r['f_mean_hz'],
            r['nu_2hf_m_s']))
    lines.append('')
    lines.append('Simulation (steady cycles >= 2):')
    lines.append('  setpoint | h_mean (m) | T_mean (s) | f_mean (Hz) | nu=2hf (m/s)')
    for r in sim_rows:
        lines.append('   {:>8} | {:8.3f} | {:8.3f} | {:8.3f} | {:8.3f}'.format(
            r['window'], r['h_mean_m'], r['T_mean_s'], r['f_mean_hz'],
            r['nu_2hf_m_s']))
    lines.append('')
    if hover is not None and duty_rows:
        lines.append('Hover reference P_omega3 = {:.4e} (rel. 1.0)'.format(
            hover['P_omega3_mean']))
        for r in duty_rows:
            lines.append('  h={:.3f} m: duty_full={:.3f}, endurance x{:.2f}, '
                         'duty_steady={:.3f}'.format(
                             r['height_m'], r['duty_full'],
                             r['endurance_multiplier_full'], r['duty_steady']))
    lines.append('')
    if duty_thrust_rows:
        lines.append('Paper duty ratio D = mean(sum(f_i))/mg (endurance test '
                     'definition):')
        for r in duty_thrust_rows:
            lines.append('  h={:.3f} m: D_full={:.3f}, D_steady={:.3f}, '
                         'frac(Σf<0.11mg)={:.3f}'.format(
                             r['height_m'], r['D_full'], r['D_steady'],
                             r['frac_lt_011mg_full']))
    lines.append('')
    lines.append('Trajectory RMSE (task-7 data, paper protocol):')
    for name, d in rmse.items():
        if d:
            lines.append('  {}: paper-lateral {:.3f} m, landing {:.3f} m, '
                         'height {:.3f} m'.format(
                             name, d['paper_lateral_rmse_m'],
                             d['landing_rmse_m'], d['height_rmse_m']))
    summary_txt = '\n'.join(lines) + '\n'
    with open(str(out) + '_summary.txt', 'w', encoding='utf-8') as fh:
        fh.write(summary_txt)
    print(summary_txt)
    print('saved: {}_*.csv/png/txt'.format(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
