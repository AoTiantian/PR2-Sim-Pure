#!/usr/bin/env python3
"""Plot PR2 force-input / displacement-output response curves from raw CSV logs.

The preferred input is the CSV produced by ``pr2_arm_force_injector`` after the
admittance debug columns were added.  Legacy logs that only contain force and
end-effector pose still work, but the admittance-output and execution-command
panels will explicitly say that those channels are missing.

Five panels are generated:
  1. external force input;
  2. admittance-module output, i.e. desired displacement/velocity;
  3. actual end-effector response, relative to the baseline pose;
  4. execution-layer command summary, i.e. qdot/tau command norms;
  5. tail zoom of the actual end-effector response for settle/stability checks.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

AXES = ("x", "y", "z")
COLORS = {"x": "#d62728", "y": "#2ca02c", "z": "#1f77b4"}


@dataclass(frozen=True)
class EventWindow:
    onset: float | None
    release: float | None


def _one_dim_structured(data: np.ndarray) -> np.ndarray:
    if data.size == 0:
        return data
    if data.shape == ():
        return data.reshape(1)
    return data


def load_csv(path: str) -> np.ndarray:
    if not os.path.isfile(path):
        sys.exit(f"file not found: {path}")
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    data = _one_dim_structured(data)
    if data.size == 0:
        sys.exit(f"empty CSV: {path}")
    required = {"timestamp", "pos_x", "pos_y", "pos_z", "force_x", "force_y", "force_z"}
    names = set(data.dtype.names or ())
    missing = sorted(required - names)
    if missing:
        sys.exit(f"CSV missing required columns: {', '.join(missing)}")
    return data


def series(data: np.ndarray, name: str, default: float | None = None) -> np.ndarray | None:
    names = set(data.dtype.names or ())
    if name in names:
        return np.asarray(data[name], dtype=float)
    if default is None:
        return None
    return np.full(len(data), default, dtype=float)


def time_axis(data: np.ndarray) -> np.ndarray:
    t = np.asarray(data["timestamp"], dtype=float)
    return t - float(t[0])


def baseline_relative(values: np.ndarray, baseline_index: int) -> np.ndarray:
    idx = min(max(baseline_index, 0), len(values) - 1)
    return values - float(values[idx])


def force_window(data: np.ndarray, t: np.ndarray, threshold: float = 0.5) -> EventWindow:
    force_mag = np.zeros(len(t), dtype=float)
    for axis in AXES:
        force_mag = np.maximum(force_mag, np.abs(data[f"force_{axis}"]))
    active = force_mag > threshold
    if not np.any(active):
        return EventWindow(onset=None, release=None)
    active_idx = np.flatnonzero(active)
    onset = float(t[active_idx[0]])
    release_idx = active_idx[-1] + 1
    release = float(t[release_idx]) if release_idx < len(t) else float(t[active_idx[-1]])
    return EventWindow(onset=onset, release=release)


def annotate_force_window(ax: plt.Axes, window: EventWindow) -> None:
    if window.onset is not None:
        ax.axvline(window.onset, color="0.35", linestyle="--", linewidth=1.0, alpha=0.7)
        ax.text(window.onset, 0.98, " force on", transform=ax.get_xaxis_transform(),
                va="top", ha="left", fontsize=8, color="0.25")
    if window.release is not None:
        ax.axvline(window.release, color="0.35", linestyle=":", linewidth=1.0, alpha=0.7)
        ax.text(window.release, 0.88, " force off", transform=ax.get_xaxis_transform(),
                va="top", ha="left", fontsize=8, color="0.25")
    if window.onset is not None and window.release is not None and window.release > window.onset:
        ax.axvspan(window.onset, window.release, color="0.85", alpha=0.25, zorder=0)


def add_missing_panel(ax: plt.Axes, message: str) -> None:
    ax.text(0.5, 0.5, message, ha="center", va="center", transform=ax.transAxes,
            fontsize=10, bbox=dict(boxstyle="round", facecolor="#fff3cd", edgecolor="#c9a227"))
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)


def plot_response(
    data: np.ndarray,
    title: str,
    save: str | None,
    baseline_skip_samples: int,
    tail_window_s: float,
) -> None:
    t = time_axis(data)
    window = force_window(data, t)

    fig = plt.figure(figsize=(15, 13))
    fig.suptitle(title, fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(5, 1, figure=fig, hspace=0.42)

    # 1. External force input.
    ax_force = fig.add_subplot(gs[0, 0])
    for axis in AXES:
        ax_force.plot(t, data[f"force_{axis}"], label=f"F{axis.upper()}",
                      color=COLORS[axis], linewidth=1.4)
    annotate_force_window(ax_force, window)
    ax_force.set_title("1) External wrench input")
    ax_force.set_ylabel("Force [N]")
    ax_force.grid(True, alpha=0.35)
    ax_force.legend(loc="upper right", ncol=3, fontsize=8)

    # 2. Admittance output.
    ax_adm = fig.add_subplot(gs[1, 0], sharex=ax_force)
    if all(series(data, f"adm_disp_{axis}") is not None for axis in AXES):
        for axis in AXES:
            adm_disp = series(data, f"adm_disp_{axis}")
            assert adm_disp is not None
            ax_adm.plot(t, adm_disp * 1000.0, label=f"Δx_des {axis.upper()}",
                        color=COLORS[axis], linewidth=1.4)
        if all(series(data, f"adm_vel_{axis}") is not None for axis in AXES):
            ax_vel = ax_adm.twinx()
            for axis in AXES:
                adm_vel = series(data, f"adm_vel_{axis}")
                assert adm_vel is not None
                ax_vel.plot(t, adm_vel * 1000.0, linestyle="--", alpha=0.45,
                            color=COLORS[axis], linewidth=1.0)
            ax_vel.set_ylabel("Desired velocity [mm/s]", color="0.35")
            ax_vel.tick_params(axis="y", labelcolor="0.35")
        annotate_force_window(ax_adm, window)
        ax_adm.set_ylabel("Desired disp [mm]")
        ax_adm.grid(True, alpha=0.35)
        ax_adm.legend(loc="upper right", ncol=3, fontsize=8)
    else:
        add_missing_panel(ax_adm, "CSV has no adm_disp_* / adm_vel_* columns;\nrerun simulation with the updated logger.")
    ax_adm.set_title("2) Admittance module output (desired motion)")

    # 3. Actual EE response.
    ax_ee = fig.add_subplot(gs[2, 0], sharex=ax_force)
    ee_rel: dict[str, np.ndarray] = {}
    for axis in AXES:
        rel = baseline_relative(np.asarray(data[f"pos_{axis}"], dtype=float), baseline_skip_samples)
        ee_rel[axis] = rel
        ax_ee.plot(t, rel * 1000.0, label=f"Δp {axis.upper()}",
                   color=COLORS[axis], linewidth=1.4)
    annotate_force_window(ax_ee, window)
    ax_ee.set_title("3) Actual end-effector response")
    ax_ee.set_ylabel("EE displacement [mm]")
    ax_ee.grid(True, alpha=0.35)
    ax_ee.legend(loc="upper right", ncol=3, fontsize=8)

    # 4. Execution-layer commands.
    ax_cmd = fig.add_subplot(gs[3, 0], sharex=ax_force)
    plotted_cmd = False
    qdot_norm = series(data, "qdot_cmd_norm")
    tau_norm = series(data, "tau_norm")
    tau_max = series(data, "tau_max_abs")
    if qdot_norm is not None:
        ax_cmd.plot(t, qdot_norm, label="||qdot_cmd|| [rad/s]", color="#9467bd", linewidth=1.3)
        plotted_cmd = True
    if tau_norm is not None:
        ax_tau = ax_cmd.twinx()
        ax_tau.plot(t, tau_norm, label="||tau|| [Nm]", color="#ff7f0e", linewidth=1.2)
        if tau_max is not None:
            ax_tau.plot(t, tau_max, label="max |tau_i| [Nm]", color="#8c564b", linestyle="--", linewidth=1.1)
        ax_tau.set_ylabel("Torque command [Nm]", color="#8c564b")
        ax_tau.tick_params(axis="y", labelcolor="#8c564b")
        ax_tau.legend(loc="upper right", fontsize=8)
        plotted_cmd = True
    if plotted_cmd:
        annotate_force_window(ax_cmd, window)
        ax_cmd.set_ylabel("Velocity cmd norm")
        ax_cmd.grid(True, alpha=0.35)
        ax_cmd.legend(loc="upper left", fontsize=8)
    else:
        add_missing_panel(ax_cmd, "CSV has no qdot_cmd_norm / tau_norm / tau_max_abs columns;\ncommand-layer oscillation cannot be judged.")
    ax_cmd.set_title("4) Execution-layer command")

    # 5. Tail zoom.
    ax_tail = fig.add_subplot(gs[4, 0])
    tail_start = max(0.0, float(t[-1]) - tail_window_s)
    if window.release is not None:
        tail_start = max(tail_start, float(window.release))
    tail_mask = t >= tail_start
    if not np.any(tail_mask):
        tail_mask = np.ones(len(t), dtype=bool)
    for axis in AXES:
        ax_tail.plot(t[tail_mask], ee_rel[axis][tail_mask] * 1000.0,
                     label=f"tail Δp {axis.upper()}", color=COLORS[axis], linewidth=1.5)
    ax_tail.set_title("5) Tail zoom for settle check")
    ax_tail.set_xlabel("Time [s]")
    ax_tail.set_ylabel("EE displacement [mm]")
    ax_tail.grid(True, alpha=0.35)
    ax_tail.legend(loc="upper right", ncol=3, fontsize=8)

    summary = []
    for axis in AXES:
        tail = ee_rel[axis][tail_mask] * 1000.0
        summary.append(
            f"{axis}: final={tail[-1]:+.2f} mm, std={np.std(tail):.2f} mm, range={np.ptp(tail):.2f} mm"
        )
    ax_tail.text(0.01, 0.98, "\n".join(summary), transform=ax_tail.transAxes,
                 va="top", ha="left", fontsize=8,
                 bbox=dict(boxstyle="round", facecolor="white", alpha=0.75))

    for ax in (ax_force, ax_adm, ax_ee, ax_cmd):
        ax.tick_params(labelbottom=False)

    if save:
        os.makedirs(os.path.dirname(save) or ".", exist_ok=True)
        plt.savefig(save, dpi=160, bbox_inches="tight")
        print(f"saved plot to {save}")
        plt.close(fig)
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="PR2 force-input / displacement-output response plot")
    parser.add_argument("--csv", required=True, help="raw simulation CSV log path")
    parser.add_argument("--title", default="PR2 Arm Force Input / Displacement Output Response")
    parser.add_argument("--save", default=None, help="PNG output path; show a window if omitted")
    parser.add_argument(
        "--baseline-skip-samples",
        type=int,
        default=0,
        help="samples to skip before choosing the displacement baseline; acceptance often uses 60",
    )
    parser.add_argument("--tail-window-s", type=float, default=3.0, help="tail zoom window length")
    args = parser.parse_args()

    data = load_csv(args.csv)
    plot_response(data, args.title, args.save, args.baseline_skip_samples, args.tail_window_s)


if __name__ == "__main__":
    main()
