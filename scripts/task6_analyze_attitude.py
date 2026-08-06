#!/usr/bin/env python3
"""Analyze task-6 attitude control telemetry against acceptance metrics.

Checks:
  * hover: mean |roll/pitch| < 2 deg, bounded drift, no divergence
  * steps: settle within 5 s, steady-state error < 1.5 deg
  * zero-thrust: motor speeds ~0 and the vehicle enters a ballistic fall
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

REPO_ROOT = Path(__file__).resolve().parents[1]

DEG = 180.0 / math.pi
HOVER_MEAN_MAX = 2.0          # deg
HOVER_MAX_MAX = 8.0           # deg
HOVER_DRIFT_MAX = 0.30        # m
SETTLE_TIME_MAX = 5.0         # s
STEP_BAND = 1.15              # deg (settling band)
STEP_STEADY_MAX = 1.5         # deg
STEP_OVERSHOOT_MAX = 20.0     # deg
ZERO_SPEED_MAX = 5.0          # rad/s
ZERO_DROP_MIN = 0.05          # m


def quaternion_to_rpy(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return (0.0, 0.0, 0.0)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    sin_p = 2.0 * (w * y - z * x)
    pitch = (math.copysign(math.pi / 2.0, sin_p)
             if abs(sin_p) >= 1.0 else math.asin(sin_p))
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def as_array(rows, key):
    return np.asarray([float(r[key]) for r in rows], dtype=float)


def settle_time(t, error, band, start):
    """First time (after `start`) the error enters and stays in the band."""
    mask = t >= start
    if np.count_nonzero(mask) == 0:
        return float('nan')
    for i in np.flatnonzero(mask):
        if i + 1 >= len(t):
            return float('nan')
        window = np.abs(error[i:]) < band
        if np.all(window[: max(1, len(window) // 2)]):
            return float(t[i])
    return float('nan')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', required=True, help='controller telemetry CSV')
    parser.add_argument(
        '--out-prefix',
        default=str(REPO_ROOT / 'docs/data/task6_attitude'),
        help='output prefix for plots and summary',
    )
    parser.add_argument(
        '--hover-duration', type=float, default=5.0,
        help='hover phase length (matches the driver schedule)',
    )
    parser.add_argument(
        '--step-duration', type=float, default=5.0,
        help='per-step phase length',
    )
    args = parser.parse_args()

    rows = load_csv(args.csv)
    if len(rows) < 10:
        raise SystemExit('CSV too short: {} rows'.format(len(rows)))

    t = as_array(rows, 'sim_time')
    mode = [r['mode'] for r in rows]
    roll = as_array(rows, 'roll')
    pitch = as_array(rows, 'pitch')
    yaw = as_array(rows, 'yaw')
    erot = np.sqrt(
        as_array(rows, 'erot_x') ** 2
        + as_array(rows, 'erot_y') ** 2
        + as_array(rows, 'erot_z') ** 2)
    speeds = np.column_stack([
        as_array(rows, 'speed{}'.format(i)) for i in range(4)])
    pos_z = as_array(rows, 'pos_z')
    pos_x = as_array(rows, 'pos_x')
    pos_y = as_array(rows, 'pos_y')

    # Reference angles for plotting (from the setpoint quaternion).
    ref = np.array([
        quaternion_to_rpy(float(r['r_ref_x']), float(r['r_ref_y']),
                          float(r['r_ref_z']), float(r['r_ref_w']))
        for r in rows
    ])

    ref_roll = ref[:, 0]
    ref_pitch = ref[:, 1]
    ref_yaw = ref[:, 2]

    def first_time(mask, after=-math.inf):
        idx = np.flatnonzero(mask & (t > after))
        return float(t[idx[0]]) if len(idx) else float('nan')

    roll_start = first_time(np.abs(ref_roll) > 0.01)
    pitch_start = first_time(np.abs(ref_pitch) > 0.01, after=roll_start)
    yaw_start = first_time(np.abs(ref_yaw) > 0.01, after=pitch_start)
    level_start = first_time(
        (np.abs(ref_roll) < 0.01) & (np.abs(ref_pitch) < 0.01)
        & (np.abs(ref_yaw) < 0.01),
        after=yaw_start + 0.5)
    zero_start = first_time(
        np.asarray([r == 'zero_thrust' for r in mode]),
        after=level_start)

    if not math.isnan(roll_start):
        hover_end = roll_start - 0.5
        phase_starts = [roll_start, pitch_start, yaw_start, level_start]
        phase_names = ['roll', 'pitch', 'yaw', 'level']
        phase_ends = phase_starts[1:] + [zero_start]
        step_duration = args.step_duration
    else:
        hover_end = args.hover_duration
        step = args.step_duration
        phase_starts = [
            hover_end, hover_end + step, hover_end + 2 * step,
            hover_end + 3 * step,
        ]
        phase_names = ['roll', 'pitch', 'yaw', 'level']
        phase_ends = phase_starts[1:] + [hover_end + 4 * step]
        zero_start = hover_end + 4 * step

    results = []

    def add(metric, value, target, passed):
        results.append((metric, value, target, passed))

    # --- Hover ---
    m = (t >= 0.5) & (t < hover_end - 0.5) & ~np.isnan(pos_z)
    if np.count_nonzero(m) > 10:
        mean_roll = np.mean(np.abs(roll[m])) * DEG
        mean_pitch = np.mean(np.abs(pitch[m])) * DEG
        max_err = np.max(erot[m]) * DEG
        z_drift = float(pos_z[m][-1] - pos_z[m][0])
        xy_drift = float(
            math.sqrt((pos_x[m][-1] - pos_x[m][0]) ** 2
                      + (pos_y[m][-1] - pos_y[m][0]) ** 2))
        hover_ok = (
            mean_roll < HOVER_MEAN_MAX and mean_pitch < HOVER_MEAN_MAX
            and max_err < HOVER_MAX_MAX
            and abs(z_drift) < HOVER_DRIFT_MAX
            and xy_drift < HOVER_DRIFT_MAX)
        add('hover_mean_roll_deg', mean_roll, HOVER_MEAN_MAX, mean_roll < HOVER_MEAN_MAX)
        add('hover_mean_pitch_deg', mean_pitch, HOVER_MEAN_MAX, mean_pitch < HOVER_MEAN_MAX)
        add('hover_max_att_err_deg', max_err, HOVER_MAX_MAX, max_err < HOVER_MAX_MAX)
        add('hover_z_drift_m', z_drift, HOVER_DRIFT_MAX, abs(z_drift) < HOVER_DRIFT_MAX)
        add('hover_xy_drift_m', xy_drift, HOVER_DRIFT_MAX, xy_drift < HOVER_DRIFT_MAX)
        add('hover_pass', 1 if hover_ok else 0, 1, hover_ok)
    else:
        add('hover_pass', 0, 1, False)

    # --- Attitude steps (roll / pitch / yaw / return level) ---
    axes = {'roll': 0, 'pitch': 1, 'yaw': 2}
    step_ok = True
    for phase, start, end in zip(phase_names, phase_starts, phase_ends):
        if math.isnan(start) or math.isnan(end):
            step_ok = False
            continue
        window = (t >= start + 0.2) & (t < end)
        if np.count_nonzero(window) < 10:
            step_ok = False
            continue
        if phase == 'level':
            error = erot[window]
        else:
            axis = axes[phase]
            euler = [roll, pitch, yaw][axis]
            error = euler[window] - ref[window][:, axis]
        ts = t[window]
        settle = settle_time(ts, error, STEP_BAND * math.pi / 180.0, ts[0])
        steady = float(np.mean(np.abs(error[-int(max(1, len(error) * 0.2)):])))
        max_abs = float(np.max(np.abs(error)))
        settled = (settle - start) <= SETTLE_TIME_MAX
        steady_ok = steady * DEG < STEP_STEADY_MAX
        overshoot_ok = max_abs * DEG < STEP_OVERSHOOT_MAX
        phase_ok = settled and steady_ok and overshoot_ok
        step_ok = step_ok and phase_ok
        add('step_{}_settle_s'.format(phase), settle - start if not math.isnan(settle) else float('nan'),
            SETTLE_TIME_MAX, settled)
        add('step_{}_steady_err_deg'.format(phase), steady * DEG,
            STEP_STEADY_MAX, steady_ok)
        add('step_{}_max_err_deg'.format(phase), max_abs * DEG,
            STEP_OVERSHOOT_MAX, overshoot_ok)
        add('step_{}_pass'.format(phase), 1 if phase_ok else 0, 1, phase_ok)

    # --- Zero thrust / ballistic ---
    mz = np.asarray([r == 'zero_thrust' for r in mode])
    if np.count_nonzero(mz) > 10:
        max_speed = float(np.max(np.abs(speeds[mz])))
        z_start = float(np.nanmean(pos_z[mz][: max(1, np.count_nonzero(mz) // 10)]))
        z_end = float(np.nanmean(pos_z[mz][-max(1, np.count_nonzero(mz) // 10):]))
        z_drop = z_start - z_end
        zero_ok = (max_speed < ZERO_SPEED_MAX and z_drop > ZERO_DROP_MIN
                   and np.count_nonzero(~np.isnan(pos_z[mz])) > 10)
        add('zero_max_motor_speed_rad_s', max_speed, ZERO_SPEED_MAX,
            max_speed < ZERO_SPEED_MAX)
        add('zero_z_drop_m', z_drop, ZERO_DROP_MIN, z_drop > ZERO_DROP_MIN)
        add('zero_pass', 1 if zero_ok else 0, 1, zero_ok)
    else:
        add('zero_pass', 0, 1, False)
        zero_ok = False

    overall = all(
        r[3] for r in results
        if r[0].endswith('pass') or r[0] in (
            'hover_mean_roll_deg', 'hover_mean_pitch_deg',
            'hover_max_att_err_deg', 'hover_z_drift_m', 'hover_xy_drift_m'))
    overall = overall and step_ok and zero_ok

    # --- Plots ---
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    for start in phase_starts:
        for ax in (axs[0, 0], axs[0, 1], axs[1, 1]):
            ax.axvline(start, color='gray', lw=0.6, alpha=0.6)
    axs[0, 0].plot(t, roll * DEG, label='roll')
    axs[0, 0].plot(t, pitch * DEG, label='pitch')
    axs[0, 0].plot(t, yaw * DEG, label='yaw')
    axs[0, 0].plot(t, ref[:, 0] * DEG, '--', color='C0', lw=1.0, alpha=0.7)
    axs[0, 0].plot(t, ref[:, 1] * DEG, '--', color='C1', lw=1.0, alpha=0.7)
    axs[0, 0].plot(t, ref[:, 2] * DEG, '--', color='C2', lw=1.0, alpha=0.7)
    axs[0, 0].set_ylabel('angle [deg]')
    axs[0, 0].legend(loc='upper right', fontsize=8)
    axs[0, 0].set_title('Euler angles vs setpoints')

    axs[0, 1].plot(t, erot * DEG)
    axs[0, 1].set_ylabel('attitude error [deg]')
    axs[0, 1].set_title('Rotation error norm')

    axs[1, 0].plot(t, speeds)
    axs[1, 0].set_ylabel('motor speed [rad/s]')
    axs[1, 0].set_xlabel('sim time [s]')
    axs[1, 0].set_title('Motor commands')

    axs[1, 1].plot(t, pos_z, label='z')
    axs[1, 1].plot(t, pos_x, label='x')
    axs[1, 1].plot(t, pos_y, label='y')
    axs[1, 1].set_ylabel('position [m]')
    axs[1, 1].set_xlabel('sim time [s]')
    axs[1, 1].legend(loc='upper right', fontsize=8)
    axs[1, 1].set_title('Position (zero-thrust -> ballistic fall)')
    fig.tight_layout()

    plot_path = args.out_prefix + '_verification.png'
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    summary_path = args.out_prefix + '_summary.csv'
    with open(summary_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'value', 'target', 'pass'])
        for metric, value, target, passed in results:
            writer.writerow([
                metric,
                '{:.6f}'.format(value) if isinstance(value, float) else value,
                '{:.6f}'.format(target) if isinstance(target, float) else target,
                'PASS' if passed else 'FAIL',
            ])

    print('=== Task 6 attitude control summary ===')
    for metric, value, target, passed in results:
        if isinstance(value, float):
            value_str = '{:.4f}'.format(value)
        else:
            value_str = str(value)
        print('{:<32s} {:<12s} target {:<10s} {}'.format(
            metric, value_str,
            '{:.4f}'.format(target) if isinstance(target, float) else str(target),
            'PASS' if passed else 'FAIL'))
    print('overall: {}'.format('PASS' if overall else 'FAIL'))
    print('plot: {}'.format(plot_path))
    print('summary: {}'.format(summary_path))
    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
