"""Validate and compare one human-only run with one human-robot run."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _load(run_dir: Path):
    with (run_dir / "run_manifest.json").open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    with (run_dir / "metrics.json").open(encoding="utf-8") as stream:
        metrics = json.load(stream)
    with (run_dir / "history.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return manifest, metrics, rows


def _values(rows, key):
    return np.asarray([float(row[key]) for row in rows], dtype=np.float64)


def compare(human_only_dir: Path, human_robot_dir: Path, output_dir: Path) -> None:
    ho_manifest, ho_metrics, ho = _load(human_only_dir)
    hr_manifest, hr_metrics, hr = _load(human_robot_dir)
    if ho_manifest["condition"] != "human_only" or hr_manifest["condition"] != "human_robot":
        raise ValueError("run conditions are not human_only and human_robot")
    if ho_manifest["config_sha256"] != hr_manifest["config_sha256"]:
        raise ValueError("refusing to compare runs with different configuration hashes")
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "config_sha256": ho_manifest["config_sha256"],
        "human_only": ho_metrics,
        "human_robot": hr_metrics,
    }
    with (output_dir / "comparison_metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)

    fig, axes = plt.subplots(6, 2, figsize=(12, 14), sharex="col", constrained_layout=True)
    for column, (label, rows) in enumerate((("Human only", ho), ("Human + robot", hr))):
        time = _values(rows, "time") - _values(rows, "time")[0]
        for row, axis in enumerate("xyz"):
            axes[row, column].plot(time, _values(rows, f"desired_position_{axis}"), "--", label="desired")
            axes[row, column].plot(time, _values(rows, f"actual_position_{axis}"), label="actual")
            axes[row, column].set_ylabel(f"{axis} (m)")
            axes[row, column].grid(alpha=0.25)
            axes[row + 3, column].plot(
                time, _values(rows, f"desired_rotvec_{axis}"), "--", label="desired"
            )
            axes[row + 3, column].plot(
                time, _values(rows, f"actual_rotvec_{axis}"), label="actual"
            )
            axes[row + 3, column].set_ylabel(f"rotvec {axis} (rad)")
            axes[row + 3, column].grid(alpha=0.25)
        axes[0, column].set_title(label)
        axes[0, column].legend()
        axes[3, column].legend()
        axes[-1, column].set_xlabel("tracking time (s)")
    fig.savefig(output_dir / "trajectory_6d.png", dpi=200)
    plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True, constrained_layout=True)
    for label, rows in (("Human only", ho), ("Human + robot", hr)):
        time = _values(rows, "time") - _values(rows, "time")[0]
        for row, axis in enumerate("xyz"):
            axes[row, 0].plot(time, _values(rows, f"human_force_{axis}"), label=label)
            axes[row, 1].plot(
                time, _values(rows, f"human_task_torque_{axis}"), label=label
            )
            axes[row, 0].set_ylabel(f"F{axis} (N)")
            axes[row, 1].set_ylabel(f"task torque {axis} (Nm)")
            axes[row, 0].grid(alpha=0.25)
            axes[row, 1].grid(alpha=0.25)
    axes[0, 0].set_title("Applied human endpoint force")
    axes[0, 1].set_title("Applied human task torque")
    axes[0, 0].legend()
    axes[0, 1].legend()
    axes[-1, 0].set_xlabel("tracking time (s)")
    axes[-1, 1].set_xlabel("tracking time (s)")
    fig.savefig(output_dir / "human_effort.png", dpi=200)
    fig.savefig(output_dir / "human_wrench_6d.png", dpi=200)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True, constrained_layout=True)
    for label, rows in (("Human only", ho), ("Human + robot", hr)):
        time = _values(rows, "time") - _values(rows, "time")[0]
        for row, axis in enumerate("xyz"):
            axes[row].plot(time, _values(rows, f"desired_rotvec_{axis}"), "--", alpha=0.8)
            axes[row].plot(time, _values(rows, f"actual_rotvec_{axis}"), label=label)
            axes[row].set_ylabel(f"rot {axis} (rad)")
            axes[row].grid(alpha=0.25)
    axes[0].legend()
    axes[-1].set_xlabel("tracking time (s)")
    fig.savefig(output_dir / "board_attitude.png", dpi=200)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True, constrained_layout=True)
    time = _values(hr, "time") - _values(hr, "time")[0]
    for axis in "xyz":
        axes[0].plot(time, _values(hr, f"robot_force_{axis}"), label=axis)
        axes[1].plot(time, _values(hr, f"robot_torque_{axis}"), label=axis)
    axes[0].set_ylabel("tare-referenced force (N)")
    axes[1].set_ylabel("tare-referenced torque (Nm)")
    axes[1].set_xlabel("tracking time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    fig.savefig(output_dir / "robot_assistance.png", dpi=200)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("human_only_run", type=Path)
    parser.add_argument("human_robot_run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    compare(args.human_only_run, args.human_robot_run, args.output)


if __name__ == "__main__":
    main()
