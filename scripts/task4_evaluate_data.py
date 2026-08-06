#!/usr/bin/env python3
"""Process the Hopcopter mocap record with the ``evaluate_data.m`` pipeline.

Steps (ported from the authors' MATLAB script):

1. load the Qualisys record;
2. compute the foot position via a -0.22 m body-z offset;
3. detect t_LD / t_TO by foot-height threshold crossings (0.03 m);
4. compute landing/takeoff velocities by central differences;
5. fit the ground plane normal per jump, project vectors onto the plane and
   compute theta_LD, body-pitch change delta_psi and takeoff velocity angle
   theta_TO;
6. predict delta_psi and theta_TO with the stance model (Eqs. 12-21) and
   report RMSE / max errors.

The default ``--window`` reproduces the 57-jump selection that the authors
chose interactively in ``evaluate_data.m`` (verified: values match the
packaged ``data_output/saved_data.mat`` to ~0.002 deg). Use ``--full`` to
process the whole record instead.

Outputs (default ``docs/data/``):
  task4_mocap_overview.png  foot height, crossings and selected window
  task4_prediction_scatter.png  measured vs predicted delta_psi and theta_TO
  task4_jumps.csv           per-jump measured/predicted table
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from hopcopter_model.evaluate import (
    analyze_jumps,
    central_diff,
    detect_jump_crossings,
    foot_z,
    load_mocap,
    rmse,
)
from hopcopter_model.params import AUTHORS_PARAMS, PAPER_PARAMS


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--mat",
        default=str(ROOT / "data/zenodo/StanceDynamics/StanceDynamics/data/20220502_140314.mat"),
    )
    ap.add_argument("--out-dir", default=str(ROOT / "docs/data"))
    ap.add_argument(
        "--window",
        nargs=2,
        type=float,
        default=[26.0, 61.0],
        help="landing-time window (authors' 57-jump selection)",
    )
    ap.add_argument("--full", action="store_true", help="process the whole record")
    ap.add_argument(
        "--params",
        choices=["paper", "authors"],
        default="paper",
        help="model parameters (paper: k/m=5.00e3, f_c/m=12.7; authors: MATLAB code values)",
    )
    args = ap.parse_args()

    params = PAPER_PARAMS if args.params == "paper" else AUTHORS_PARAMS
    window = None if args.full else args.window
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    mocap = load_mocap(args.mat)
    jumps = analyze_jumps(mocap, params, window=window)

    r_dpsi = rmse(jumps, "delta_psi", "model_delta_psi")
    r_tto = rmse(jumps, "theta_TO", "model_theta_TO")
    e_dpsi = np.abs(jumps["err_delta_psi"])
    e_tto = np.abs(jumps["err_theta_TO"])

    print(f"jumps analysed      : {len(jumps)}")
    print(f"landing speed range : {jumps['landing_speed'].min():.3f} - {jumps['landing_speed'].max():.3f} m/s")
    print(f"theta_LD range      : {jumps['theta_landing'].min():.3f} - {jumps['theta_landing'].max():.3f} deg")
    print(f"RMSE delta_psi      : {r_dpsi:.3f} deg (max {e_dpsi.max():.3f})")
    print(f"RMSE theta_TO       : {r_tto:.3f} deg (max {e_tto.max():.3f})")

    # Per-jump table.
    header = ",".join(jumps.dtype.names)
    rows = "\n".join(",".join(f"{v:.9g}" for v in row) for row in jumps)
    (out / "task4_jumps.csv").write_text(header + "\n" + rows, encoding="utf-8")

    # Mocap overview: foot height, crossings, selected window.
    zf = foot_z(mocap)
    det = detect_jump_crossings(mocap)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(mocap.t, mocap.z, lw=0.8)
    axes[0].set_ylabel("z (m)")
    axes[0].set_title("Mocap body height")
    axes[1].plot(mocap.t, zf, lw=0.8, label="foot z")
    axes[1].axhline(0.03, color="k", ls="--", lw=0.8, label="threshold")
    axes[1].plot(det["t_cross"], np.full(len(det["t_cross"]), 0.03), "rx", ms=4)
    axes[1].set_ylabel("z (m)")
    axes[1].set_title("Foot height with landing/takeoff crossings")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[2].plot(mocap.t, central_diff(mocap.t, mocap.z), lw=0.8, label="dz/dt")
    axes[2].plot(mocap.t[det["index"]], det["vz"], "rx", ms=4, label="crossings")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("dz/dt (m/s)")
    axes[2].set_title("Vertical velocity")
    axes[2].legend(loc="upper right", fontsize=8)
    if window is not None:
        for ax in axes:
            ax.axvspan(window[0], window[1], color="C3", alpha=0.08)
    fig.tight_layout()
    fig.savefig(out / "task4_mocap_overview.png", dpi=160)
    plt.close(fig)

    # Prediction scatter.
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.6))
    for ax, (meas, pred, title, rms) in zip(
        axes,
        [
            ("delta_psi", "model_delta_psi", r"$\Delta\psi$ (°)", r_dpsi),
            ("theta_TO", "model_theta_TO", r"$\theta_{TO}$ (°)", r_tto),
        ],
    ):
        lo = min(jumps[meas].min(), jumps[pred].min()) - 1
        hi = max(jumps[meas].max(), jumps[pred].max()) + 1
        ax.plot([lo, hi], [lo, hi], "k--", lw=0.8, label="1:1")
        ax.plot(jumps[meas], jumps[pred], "o", ms=5, mfc="C0", mec="k", mew=0.5, alpha=0.85)
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
        ax.set_xlabel(f"measured {title}")
        ax.set_ylabel(f"model {title}")
        ax.set_title(f"{title}  RMSE = {rms:.2f}°")
        ax.legend(loc="lower right", fontsize=8)
        ax.grid(alpha=0.3)
    fig.suptitle("Stance model prediction vs measured (paper parameters)")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / "task4_prediction_scatter.png", dpi=160)
    plt.close(fig)

    print(f"figures and CSV written to {out}")


if __name__ == "__main__":
    main()
