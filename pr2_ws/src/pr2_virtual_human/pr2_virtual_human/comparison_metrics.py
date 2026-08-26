"""Offline metrics for one transport-comparison run."""

from __future__ import annotations

from typing import Any

import numpy as np


def _series(rows: list[dict[str, float | str]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=np.float64)


def _axis_metric(rows, prefix: str, function) -> dict[str, float]:
    return {axis: float(function(_series(rows, f"{prefix}_{axis}"))) for axis in "xyz"}


def compute_run_metrics(rows: list[dict[str, float | str]], limits: dict[str, np.ndarray]) -> dict[str, Any]:
    if len(rows) < 2:
        raise ValueError("at least two samples are required")
    times = _series(rows, "time")
    dt = np.diff(times)
    if np.any(dt <= 0.0):
        raise ValueError("sample times must be strictly increasing")
    position_rmse = _axis_metric(rows, "position_error", lambda x: np.sqrt(np.mean(x * x)))
    orientation_rmse = _axis_metric(rows, "orientation_error", lambda x: np.sqrt(np.mean(x * x)))
    force_rms = _axis_metric(rows, "human_force", lambda x: np.sqrt(np.mean(x * x)))
    task_torque_rms = _axis_metric(rows, "human_task_torque", lambda x: np.sqrt(np.mean(x * x)))
    force = np.column_stack([_series(rows, f"human_force_{axis}") for axis in "xyz"])
    task_torque = np.column_stack([_series(rows, f"human_task_torque_{axis}") for axis in "xyz"])
    position = np.column_stack([_series(rows, f"actual_position_{axis}") for axis in "xyz"])
    orientation = np.column_stack([_series(rows, f"actual_rotvec_{axis}") for axis in "xyz"])
    linear_velocity = np.diff(position, axis=0) / dt[:, None]
    angular_velocity = np.diff(orientation, axis=0) / dt[:, None]
    power = np.sum(force[:-1] * linear_velocity + task_torque[:-1] * angular_velocity, axis=1)
    force_limit = np.asarray(limits["force"], dtype=np.float64)
    torque_limit = np.asarray(limits["task_torque"], dtype=np.float64)
    saturation = np.logical_or(
        np.any(np.isclose(np.abs(force), force_limit, rtol=0.0, atol=1.0e-8), axis=1),
        np.any(np.isclose(np.abs(task_torque), torque_limit, rtol=0.0, atol=1.0e-8), axis=1),
    )
    return {
        "samples": len(rows),
        "duration_sec": float(times[-1] - times[0]),
        "position_rmse_m": position_rmse,
        "orientation_rmse_rad": orientation_rmse,
        "human_force_rms_n": force_rms,
        "human_force_peak_n": _axis_metric(rows, "human_force", lambda x: np.max(np.abs(x))),
        "human_task_torque_rms_nm": task_torque_rms,
        "human_task_torque_peak_nm": _axis_metric(rows, "human_task_torque", lambda x: np.max(np.abs(x))),
        "human_wrench_saturation_fraction": float(np.mean(saturation)),
        "human_signed_work_j": float(np.sum(power * dt)),
        "human_absolute_work_j": float(np.sum(np.abs(power) * dt)),
        "all_required_values_finite": bool(
            np.all(np.isfinite(np.column_stack((force, task_torque, position, orientation))))
        ),
    }
