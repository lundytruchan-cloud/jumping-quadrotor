#!/usr/bin/env python3
"""Reproduce Hopcopter Fig. 2C: stance-phase landing-to-takeoff mapping.

Ports the mapping loop of the authors' ``run_this.m``:

* original grid: theta_LD in [0, 25] deg, |p_dot_LD| in [1, 3.5] m/s
  (0.5 deg / 0.05 m/s steps, matching the published figure);
* extended grid (task requirement): theta_LD in [0, 45] deg,
  |p_dot_LD| in [0.1, 4.5] m/s;
* measured jumps (57, the authors' selection) overlaid as scatter points.

Outputs (default ``docs/data/``):
  task4_fig2c_mapping.png      original-grid contour map with measured points
  task4_fig2c_mapping_ext.png  extended-grid contour map
  task4_mapping_grid.csv       original-grid mapping table
  task4_mapping_grid_ext.csv   extended-grid mapping table
  task4_jumps.csv              measured vs predicted per-jump table
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

from hopcopter_model.evaluate import analyze_jumps, load_mocap
from hopcopter_model.params import PAPER_PARAMS
from hopcopter_model.stance import stance_mapping_grid


def _contour(ax, x, y, z, levels, cmap):
    cf = ax.contourf(x, y, z, levels=levels, cmap=cmap)
    ax.contour(x, y, z, levels=levels, colors="w", linewidths=0.4, alpha=0.7)
    return cf


def plot_mapping(theta_ls_deg, speeds, out_png, title_suffix, jumps=None, cmaps=("turbo", "turbo", "winter")):
    theta_l = np.deg2rad(theta_ls_deg)
    g = stance_mapping_grid(theta_l, speeds, PAPER_PARAMS)
    delta_psi = np.rad2deg(g["delta_psi"])
    theta_to = np.rad2deg(g["theta_TO"])
    v_to = g["v_t"]

    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.9))
    panels = [
        (delta_psi, r"$\Delta\psi$ (°)", "turbo"),
        (theta_to, r"$\theta_{TO}$ (°)", "turbo"),
        (v_to, r"$\|\dot{\mathbf{p}}_{TO}\|$ (m/s)", "winter"),
    ]
    for ax, (z, title, cmap) in zip(axes, panels):
        levels = np.linspace(np.nanmin(z), np.nanmax(z), 11)
        if cmap == "winter":
            levels = np.linspace(np.nanmin(z), np.nanmax(z), 12)
        cf = _contour(ax, speeds, theta_ls_deg, z, levels, cmap)
        fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.04)
        ax.set_title(title, fontsize=11)
        ax.set_xlim(speeds[0], speeds[-1])
        ax.set_ylim(theta_ls_deg[0], theta_ls_deg[-1])
        ax.set_xticks(np.arange(np.ceil(speeds[0]), speeds[-1] + 1e-9, 0.5))
        ax.set_xlabel(r"$\|\dot{\mathbf{p}}(t_{LD})\|$ (m/s)")
        if ax is axes[0]:
            ax.set_ylabel(r"$\theta_{LD}$ (°)")
        if jumps is not None:
            ax.scatter(
                jumps["landing_speed"],
                jumps["theta_landing"],
                s=18,
                facecolor="none",
                edgecolor="k",
                linewidths=0.8,
                zorder=5,
                label="measured",
            )
    if jumps is not None:
        axes[0].legend(loc="upper left", fontsize=9, framealpha=0.9)
    fig.suptitle(f"Stance-phase landing-to-takeoff map{title_suffix}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    return g


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--mat",
        default=str(ROOT / "data/zenodo/StanceDynamics/StanceDynamics/data/20220502_140314.mat"),
    )
    ap.add_argument("--out-dir", default=str(ROOT / "docs/data"))
    ap.add_argument("--window", nargs=2, type=float, default=[26.0, 61.0], help="landing-time window (authors' selection)")
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Original run_this.m grid (Fig. 2C).
    theta_ls = np.arange(0, 25 + 0.5, 0.5)
    speeds = np.arange(1, 3.5 + 0.05, 0.05)
    g = plot_mapping(theta_ls, speeds, out / "task4_fig2c_mapping.png", " (original grid)")

    # Extended grid requested by the task.
    theta_ls_ext = np.arange(0, 45 + 0.5, 0.5)
    speeds_ext = np.arange(0.1, 4.5 + 0.05, 0.05)
    g_ext = plot_mapping(
        theta_ls_ext, speeds_ext, out / "task4_fig2c_mapping_ext.png", " (extended grid)"
    )

    # Measured jumps (same 57-jump window as the authors).
    jumps = None
    mat_path = Path(args.mat)
    if mat_path.exists():
        jumps = analyze_jumps(load_mocap(mat_path), PAPER_PARAMS, window=args.window)
        header = ",".join(jumps.dtype.names)
        rows = "\n".join(",".join(f"{v:.9g}" for v in row) for row in jumps)
        (out / "task4_jumps.csv").write_text(header + "\n" + rows, encoding="utf-8")
        # Re-plot original grid with measured overlay.
        plot_mapping(
            theta_ls,
            speeds,
            out / "task4_fig2c_mapping.png",
            " (original grid)",
            jumps=jumps,
        )

    def save_csv(name, theta_ls_deg, speeds, grid):
        theta_2d, speed_2d = np.meshgrid(theta_ls_deg, speeds, indexing="ij")
        names = ["theta_LD_deg", "speed_m_s", "delta_psi_deg", "theta_TO_deg", "v_TO_m_s"]
        data = np.column_stack(
            [
                theta_2d.ravel(),
                speed_2d.ravel(),
                np.rad2deg(grid["delta_psi"]).ravel(),
                np.rad2deg(grid["theta_TO"]).ravel(),
                grid["v_t"].ravel(),
            ]
        )
        np.savetxt(out / name, data, header=",".join(names), delimiter=",", comments="", fmt="%.9g")

    save_csv("task4_mapping_grid.csv", theta_ls, speeds, g)
    save_csv("task4_mapping_grid_ext.csv", theta_ls_ext, speeds_ext, g_ext)

    print(f"figures and tables written to {out}")
    if jumps is not None:
        print(f"measured jumps used for overlay: {len(jumps)}")


if __name__ == "__main__":
    main()
