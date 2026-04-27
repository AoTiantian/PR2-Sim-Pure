#!/usr/bin/env python3
"""Plot whole-body PR2 admittance response: XYZ wrench, arm EE, and mobile base.

This complements ``plot_arm_response.py`` for demos where both the arm and the
mobile base respond to an external wrench.  The input CSV is produced by
``pr2_arm_force_injector`` and should include ``base_x``, ``base_y``, and
``base_yaw`` columns in addition to force and end-effector pose columns.
"""
from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

AXES = ("x", "y", "z")
COLORS = {"x": "#ff4d6d", "y": "#2bd576", "z": "#4dabf7"}


def load_rows(path: str) -> list[dict[str, str]]:
    if not os.path.isfile(path):
        raise SystemExit(f"CSV not found: {path}")
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"CSV has no rows: {path}")
    required = {"timestamp", "pos_x", "pos_y", "pos_z", "force_x", "force_y", "force_z", "base_x", "base_y", "base_yaw"}
    missing = required - set(rows[0])
    if missing:
        raise SystemExit(f"CSV missing columns: {sorted(missing)}")
    return rows


def arr(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.asarray([float(r[key]) for r in rows], dtype=float)


def force_window(force: np.ndarray, t: np.ndarray, threshold: float = 0.5) -> tuple[float | None, float | None]:
    active = np.max(np.abs(force), axis=1) > threshold
    if not active.any():
        return None, None
    idx = np.flatnonzero(active)
    return float(t[idx[0]]), float(t[min(idx[-1] + 1, len(t) - 1)])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--save", required=True)
    ap.add_argument("--baseline-skip-samples", type=int, default=60)
    ap.add_argument("--title", default="PR2 Whole-Body Amplified XYZ Force Demo: Arm + Mobile Base Motion")
    args = ap.parse_args()

    rows = load_rows(args.csv)
    t = arr(rows, "timestamp")
    t -= t[0]
    idx = min(max(args.baseline_skip_samples, 0), len(t) - 1)
    force = np.vstack([arr(rows, f"force_{a}") for a in AXES]).T
    pos = {a: (arr(rows, f"pos_{a}") - arr(rows, f"pos_{a}")[idx]) * 1000.0 for a in AXES}
    base = {a: (arr(rows, f"base_{a}") - arr(rows, f"base_{a}")[idx]) * 1000.0 for a in "xy"}
    yaw = (arr(rows, "base_yaw") - arr(rows, "base_yaw")[idx]) * 180.0 / np.pi
    onset, release = force_window(force, t)

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axs = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig.suptitle(args.title, fontsize=16, fontweight="bold")
    for ax in axs:
        if onset is not None and release is not None:
            ax.axvspan(onset, release, color="#ffe066", alpha=0.25, label="force on" if ax is axs[0] else None)
            ax.axvline(onset, color="0.3", ls="--", lw=1)
            ax.axvline(release, color="0.3", ls=":", lw=1)
        ax.grid(True, alpha=0.28)

    for i, axis in enumerate(AXES):
        axs[0].plot(t, force[:, i], color=COLORS[axis], lw=1.8, label=f"F{axis.upper()}")
    axs[0].set_ylabel("Force [N]")
    axs[0].legend(ncol=4, loc="upper right")
    axs[0].set_title("Simultaneous 3D external wrench")

    for axis in AXES:
        axs[1].plot(t, pos[axis], color=COLORS[axis], lw=1.9, label=f"EE Δ{axis.upper()}")
    axs[1].set_ylabel("EE displacement [mm]")
    axs[1].legend(ncol=3, loc="upper right")
    axs[1].set_title("Arm end-effector compliant displacement")

    axs[2].plot(t, base["x"], color="#12b886", lw=2, label="base ΔX [mm]")
    axs[2].plot(t, base["y"], color="#15aabf", lw=2, label="base ΔY [mm]")
    ax2 = axs[2].twinx()
    ax2.plot(t, yaw, color="#f76707", lw=1.7, label="base yaw [deg]")
    axs[2].set_ylabel("Base translation [mm]")
    ax2.set_ylabel("Base yaw [deg]")
    axs[2].set_xlabel("Time [s]")
    axs[2].set_title("Mobile-base admittance response")
    lines, labels = axs[2].get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    axs[2].legend(lines + lines2, labels + labels2, ncol=3, loc="upper right")

    summary = (
        f"EE peak: {max(np.max(np.abs(v)) for v in pos.values()):.1f} mm\n"
        f"Base XY peak: {max(np.max(np.abs(v)) for v in base.values()):.1f} mm\n"
        f"Yaw peak: {np.max(np.abs(yaw)):.2f} deg"
    )
    axs[2].text(0.012, 0.96, summary, transform=axs[2].transAxes, va="top", bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.save, dpi=210, bbox_inches="tight")
    print(args.save)


if __name__ == "__main__":
    main()
