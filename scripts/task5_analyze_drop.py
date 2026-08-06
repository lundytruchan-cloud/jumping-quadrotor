#!/usr/bin/env python3
"""Analyze the Gazebo free-fall drop CSV against the paper drop test.

Metrics (paper targets, +/-20%):
  * support time     ~32 ms   (leg compressed: q < -1 mm)
  * leg stretch     4.7-5.5 cm (elastic element total stretch = l_p + (l0 - l))
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / 'src'))

from hopcopter_model.params import PAPER_PARAMS  # noqa: E402
from hopcopter_model.stance import stance_transform  # noqa: E402

G = 9.81
L0 = PAPER_PARAMS.l0
LP = PAPER_PARAMS.l_p
PAPER_SUPPORT_MS = 32.0
PAPER_STRETCH_CM = 4.7
PAPER_STRETCH_MAX_CM = 5.5
TOL = 0.20
COMPRESSION_EPS = 0.001


def load_csv(path):
    t, foot_z, q, q_dot, force, phase = [], [], [], [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            t.append(float(row['sim_time']))
            foot_z.append(float(row['foot_z']))
            q.append(float(row['leg_q']))
            q_dot.append(float(row['leg_qdot']))
            force.append(float(row['spring_force']))
            phase.append(row['phase'])
    return (
        np.asarray(t), np.asarray(foot_z), np.asarray(q),
        np.asarray(q_dot), np.asarray(force), phase,
    )


def first_support_segment(support):
    """Return indices of the first contiguous support segment."""
    idx = np.flatnonzero(support)
    if len(idx) == 0:
        return idx
    split = np.flatnonzero(np.diff(idx) > 5)  # >5 ms gap starts a new bounce
    if len(split) == 0:
        return idx
    return idx[: int(split[0]) + 1]


def support_segments(support, t, max_gap_samples=5):
    """Return (start_t, end_t) of every support segment in the record."""
    idx = np.flatnonzero(support)
    if len(idx) == 0:
        return []
    segments = []
    start = idx[0]
    for i in range(1, len(idx)):
        if idx[i] - idx[i - 1] > max_gap_samples:
            segments.append((float(t[start]), float(t[idx[i - 1]])))
            start = idx[i]
    segments.append((float(t[start]), float(t[idx[-1]])))
    return segments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', required=True, help='leg state CSV')
    parser.add_argument(
        '--out-prefix', default=str(REPO_ROOT / 'docs/data/task5_drop'),
        help='output prefix for plot/summary',
    )
    args = parser.parse_args()

    t, foot_z, q, q_dot, force, phase = load_csv(args.csv)
    com = foot_z + L0 + q
    support = q < -COMPRESSION_EPS
    idx = first_support_segment(support)
    if len(idx) < 3:
        raise SystemExit('No support segment found in CSV (q < -1mm).')

    t_ld, t_to = t[idx[0]], t[idx[-1]]
    support_ms = (t_to - t_ld) * 1000.0
    q_min = float(np.min(q[idx]))
    l_min = L0 + q_min
    compression_cm = (L0 - l_min) * 100.0
    stretch_cm = (L0 + LP - l_min) * 100.0

    # Impact speed: joint velocity right after contact (massless-foot
    # approximation) equals the free-fall impact speed; a CoM linear fit over
    # the last 50 ms before contact is kept as a cross-check.
    v_impact = float(abs(q_dot[idx[0]])) if len(idx) else float('nan')
    pre = (t >= t_ld - 0.05) & (t < t_ld - 0.003)
    if np.count_nonzero(pre) >= 2:
        v_impact_fit = float(-np.polyfit(t[pre], com[pre], 1)[0])
    else:
        v_impact_fit = float('nan')
    v_impact_theory = float(np.sqrt(2.0 * G * (com[0] - L0)))

    model = stance_transform(0.0, v_impact, PAPER_PARAMS) if v_impact > 0 else None
    if model is not None:
        model_support_ms = model.t_TO * 1000.0
        model_stretch_cm = (L0 + LP - model.l_t1) * 100.0
    else:
        model_support_ms = float('nan')
        model_stretch_cm = float('nan')

    support_ok = abs(support_ms - PAPER_SUPPORT_MS) <= TOL * PAPER_SUPPORT_MS
    stretch_ok = (
        PAPER_STRETCH_CM * (1.0 - TOL) <= stretch_cm
        <= PAPER_STRETCH_MAX_CM * (1.0 + TOL)
    )

    print('=== Task 5 drop summary ===')
    print('first support segment: t_LD={:.4f}s t_TO={:.4f}s'.format(t_ld, t_to))
    print('support time        : {:.2f} ms (paper ~32 ms, +/-20%: 25.6-38.4) {}'
          .format(support_ms, 'PASS' if support_ok else 'FAIL'))
    print('leg stretch (l0+lp-l): {:.2f} cm (paper 4.7-5.5 cm, +/-20%) {}'
          .format(stretch_cm, 'PASS' if stretch_ok else 'FAIL'))
    print('geometric compression: {:.2f} cm'.format(compression_cm))
    print('impact speed (joint/theory/fit): {:.2f} / {:.2f} / {:.2f} m/s'
          .format(v_impact, v_impact_theory, v_impact_fit))
    print('model prediction: support {:.2f} ms, stretch {:.2f} cm'
          .format(model_support_ms, model_stretch_cm))

    summary_rows = [
        ('drop_height_m', float(com[0] - L0)),
        ('impact_speed_m_s', v_impact),
        ('impact_speed_fit_m_s', v_impact_fit),
        ('impact_speed_theory_m_s', v_impact_theory),
        ('support_time_ms', support_ms),
        ('paper_support_time_ms', PAPER_SUPPORT_MS),
        ('support_tolerance_pct', TOL * 100.0),
        ('support_pass', support_ok),
        ('leg_stretch_cm', stretch_cm),
        ('leg_compression_cm', compression_cm),
        ('paper_stretch_range_cm', '{}-{}'.format(
            PAPER_STRETCH_CM, PAPER_STRETCH_MAX_CM)),
        ('stretch_pass', stretch_ok),
        ('model_support_time_ms', model_support_ms),
        ('model_leg_stretch_cm', model_stretch_cm),
        ('t_LD_s', t_ld),
        ('t_TO_s', t_to),
    ]
    summary_path = args.out_prefix + '_summary.csv'
    with open(summary_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'value'])
        writer.writerows(summary_rows)
    print('summary ->', summary_path)

    # Plot the full record so every bounce is visible; the first support
    # segment (used for the paper comparison) is marked explicitly.
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    segments = support_segments(support, t)
    for seg_start, seg_end in segments:
        axes[0].axvspan(
            seg_start, seg_end, color='tab:orange', alpha=0.25,
            label='support' if seg_start == segments[0][0] else None)
    axes[0].plot(t, com, label='body CoM')
    axes[0].plot(t, foot_z, label='foot')
    axes[0].axvline(t_ld, color='tab:red', ls='--', lw=1)
    axes[0].axvline(t_to, color='tab:red', ls='--', lw=1)
    axes[0].set_ylabel('height (m)')
    axes[0].legend(loc='upper right')

    stretch = (L0 + LP - (L0 + q)) * 100.0
    axes[1].plot(t, stretch, color='tab:green')
    axes[1].axhline(4.7, color='gray', ls=':', lw=1)
    axes[1].axhline(5.5, color='gray', ls=':', lw=1)
    axes[1].set_ylabel('leg stretch (cm)')
    axes[1].set_ylim(0.0, max(6.0, float(np.max(stretch)) * 1.1))

    phase_num = np.asarray([
        ['AERIAL', 'LANDING', 'SUPPORT', 'TAKEOFF'].index(p)
        for p in phase])
    axes[2].plot(t, phase_num, drawstyle='steps-post', color='tab:blue')
    axes[2].plot(t, force, color='tab:red', alpha=0.7)
    axes[2].set_yticks([0, 1, 2, 3])
    axes[2].set_yticklabels(['AERIAL', 'LANDING', 'SUPPORT', 'TAKEOFF'])
    axes[2].set_ylabel('phase / force (N)')
    axes[2].set_xlabel('sim time (s)')
    fig.suptitle(
        'Task 5 drop: support {:.1f} ms, stretch {:.1f} cm'.format(
            support_ms, stretch_cm),
    )
    fig.tight_layout()
    plot_path = args.out_prefix + '_verification.png'
    fig.savefig(plot_path, dpi=150)
    print('plot ->', plot_path)

    if not (support_ok and stretch_ok):
        raise SystemExit('FAIL: metrics outside the paper +/-20% band.')


if __name__ == '__main__':
    main()
