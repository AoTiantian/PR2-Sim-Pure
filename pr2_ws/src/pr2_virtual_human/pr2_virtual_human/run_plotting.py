"""Per-run plots generated automatically beside each recorded dataset.

Each run produces exactly two figures:

1. ``trajectory_6d.png``     -- 6D desired vs actual trajectory
   (position xyz + orientation rotvec xyz).
2. ``wrench_6d.png``         -- 6D applied wrench (forces xyz + torques xyz).
   For ``human_robot`` runs the human and robot wrench components are
   overlaid on the same axes; for ``human_only`` only the human wrench
   exists.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_rcParams = {
    "figure.titlesize": 15,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
}


def _values(rows: list[dict[str, float | str]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=np.float64)


def _hold_boundary(time: np.ndarray, rows: list[dict[str, float | str]]) -> float | None:
    for t, row in zip(time, rows):
        if str(row.get("state", "")) == "hold":
            return float(t)
    return None


def _style_axis(ax: plt.Axes) -> None:
    ax.grid(alpha=0.3, linewidth=0.6)
    ax.tick_params(direction="in")


def _mark_hold(ax: plt.Axes, hold_t: float | None) -> None:
    if hold_t is None:
        return
    ax.axvline(hold_t, color="0.6", linewidth=0.9, linestyle=":", zorder=0)


def _plot_6d(
    time: np.ndarray,
    titles: list[str],
    ylabels: list[str],
    series: list[list[tuple[np.ndarray, dict]]],
    suptitle: str,
    hold_t: float | None,
    ncols_per_row: int = 3,
) -> plt.Figure:
    """Draw a 2x3 grid; ``series[i]`` holds (y, style) pairs for subplot i."""
    fig, axes = plt.subplots(
        2, ncols_per_row, figsize=(16, 9), sharex=True, constrained_layout=True
    )
    for i in range(2 * ncols_per_row):
        ax = axes[i // ncols_per_row, i % ncols_per_row]
        _mark_hold(ax, hold_t)
        for y, style in series[i]:
            ax.plot(time, y, **style)
        ax.set_title(titles[i])
        ax.set_ylabel(ylabels[i])
        _style_axis(ax)
        handles, labels = ax.get_legend_handles_labels()
        if labels:
            ax.legend(loc="upper right")
    for col in range(ncols_per_row):
        axes[-1, col].set_xlabel("time (s)")
    fig.suptitle(suptitle)
    return fig


def save_run_plots(
    rows: list[dict[str, float | str]], condition: str, run_dir: Path
) -> list[Path]:
    """Write the two standard plots for one run; no A/B comparison is performed."""
    for key, value in _rcParams.items():
        plt.rcParams[key] = value

    time = _values(rows, "time") - _values(rows, "time")[0]
    hold_t = _hold_boundary(time, rows)
    label = "Human only" if condition == "human_only" else "Human + robot"
    outputs: list[Path] = []

    # --- Plot 1: 6D trajectory, desired vs actual -------------------------
    titles = ["Position X", "Position Y", "Position Z", "Orientation X", "Orientation Y", "Orientation Z"]
    ylabels = ["x (m)", "y (m)", "z (m)", "rotvec x (rad)", "rotvec y (rad)", "rotvec z (rad)"]
    desired_style = {"color": "0.45", "linestyle": "--", "linewidth": 1.8, "label": "desired"}
    actual_style = {"color": "tab:blue", "linewidth": 1.6, "label": "actual"}
    series = [
        [
            (_values(rows, f"desired_position_{axis}"), desired_style),
            (_values(rows, f"actual_position_{axis}"), actual_style),
        ]
        for axis in "xyz"
    ] + [
        [
            (_values(rows, f"desired_rotvec_{axis}"), desired_style),
            (_values(rows, f"actual_rotvec_{axis}"), actual_style),
        ]
        for axis in "xyz"
    ]
    fig = _plot_6d(
        time,
        titles,
        ylabels,
        series,
        f"{label}: 6D trajectory, desired vs actual",
        hold_t,
    )
    trajectory_path = run_dir / "trajectory_6d.png"
    fig.savefig(trajectory_path, dpi=200)
    plt.close(fig)
    outputs.append(trajectory_path)

    # --- Plot 2: 6D wrench (forces + torques) -----------------------------
    titles = ["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"]
    ylabels = ["Fx (N)", "Fy (N)", "Fz (N)", "Tx (N·m)", "Ty (N·m)", "Tz (N·m)"]
    human_style = {"color": "tab:blue", "linewidth": 1.6, "label": "human"}
    robot_style = {"color": "tab:orange", "linewidth": 1.6, "label": "robot"}
    series = [
        [(_values(rows, f"human_force_{axis}"), human_style)] for axis in "xyz"
    ] + [
        [(_values(rows, f"human_task_torque_{axis}"), human_style)] for axis in "xyz"
    ]
    if condition == "human_robot":
        robot_series = [
            (_values(rows, f"robot_force_{axis}"), robot_style) for axis in "xyz"
        ] + [
            (_values(rows, f"robot_torque_{axis}"), robot_style) for axis in "xyz"
        ]
        for i, (y, _style) in enumerate(robot_series):
            if np.any(np.isfinite(y)):
                series[i].append(robot_series[i])
    fig = _plot_6d(
        time,
        titles,
        ylabels,
        series,
        f"{label}: 6D applied wrench (force & torque)",
        hold_t,
    )
    wrench_path = run_dir / "wrench_6d.png"
    fig.savefig(wrench_path, dpi=200)
    plt.close(fig)
    outputs.append(wrench_path)

    return outputs
