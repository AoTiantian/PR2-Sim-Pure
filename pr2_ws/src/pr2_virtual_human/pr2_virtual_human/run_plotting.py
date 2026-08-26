"""Per-run plots generated automatically beside each recorded dataset."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _values(rows: list[dict[str, float | str]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=np.float64)


def save_run_plots(
    rows: list[dict[str, float | str]], condition: str, run_dir: Path
) -> list[Path]:
    """Write plots for one run only; no A/B comparison is performed."""
    time = _values(rows, "time") - _values(rows, "time")[0]
    label = "Human only" if condition == "human_only" else "Human + robot"
    outputs: list[Path] = []

    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, constrained_layout=True)
    for row, axis in enumerate("xyz"):
        axes[row, 0].plot(time, _values(rows, f"desired_position_{axis}"), "--", label="desired")
        axes[row, 0].plot(time, _values(rows, f"actual_position_{axis}"), label="actual")
        axes[row, 0].set_ylabel(f"{axis} (m)")
        axes[row, 1].plot(time, _values(rows, f"desired_rotvec_{axis}"), "--", label="desired")
        axes[row, 1].plot(time, _values(rows, f"actual_rotvec_{axis}"), label="actual")
        axes[row, 1].set_ylabel(f"rotvec {axis} (rad)")
        for column in range(2):
            axes[row, column].grid(alpha=0.25)
    axes[0, 0].set_title(f"{label}: desired vs actual position")
    axes[0, 1].set_title(f"{label}: desired vs actual orientation")
    axes[0, 0].legend()
    axes[0, 1].legend()
    axes[-1, 0].set_xlabel("tracking time (s)")
    axes[-1, 1].set_xlabel("tracking time (s)")
    trajectory_path = run_dir / "trajectory_6d.png"
    fig.savefig(trajectory_path, dpi=200)
    plt.close(fig)
    outputs.append(trajectory_path)

    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, constrained_layout=True)
    for row, axis in enumerate("xyz"):
        axes[row, 0].plot(time, _values(rows, f"human_force_{axis}"))
        axes[row, 0].set_ylabel(f"F{axis} (N)")
        axes[row, 1].plot(time, _values(rows, f"human_task_torque_{axis}"))
        axes[row, 1].set_ylabel(f"task torque {axis} (Nm)")
        for column in range(2):
            axes[row, column].grid(alpha=0.25)
    axes[0, 0].set_title(f"{label}: applied human endpoint force")
    axes[0, 1].set_title(f"{label}: applied human task torque")
    axes[-1, 0].set_xlabel("tracking time (s)")
    axes[-1, 1].set_xlabel("tracking time (s)")
    human_wrench_path = run_dir / "human_applied_wrench_6d.png"
    fig.savefig(human_wrench_path, dpi=200)
    plt.close(fig)
    outputs.append(human_wrench_path)

    if condition == "human_robot":
        robot_force = np.column_stack(
            [_values(rows, f"robot_force_{axis}") for axis in "xyz"]
        )
        robot_torque = np.column_stack(
            [_values(rows, f"robot_torque_{axis}") for axis in "xyz"]
        )
        if np.any(np.isfinite(np.column_stack((robot_force, robot_torque)))):
            fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, constrained_layout=True)
            for row, axis in enumerate("xyz"):
                axes[row, 0].plot(time, robot_force[:, row])
                axes[row, 0].set_ylabel(f"F{axis} (N)")
                axes[row, 1].plot(time, robot_torque[:, row])
                axes[row, 1].set_ylabel(f"torque {axis} (Nm)")
                for column in range(2):
                    axes[row, column].grid(alpha=0.25)
            axes[0, 0].set_title("Measured robot force (tare-referenced)")
            axes[0, 1].set_title("Measured robot torque (tare-referenced)")
            axes[-1, 0].set_xlabel("tracking time (s)")
            axes[-1, 1].set_xlabel("tracking time (s)")
            robot_wrench_path = run_dir / "robot_measured_wrench_6d.png"
            fig.savefig(robot_wrench_path, dpi=200)
            plt.close(fig)
            outputs.append(robot_wrench_path)

    return outputs
