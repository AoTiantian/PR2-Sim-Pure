"""Validated configuration loader for the transport comparison experiment."""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .experiment_state import ExperimentTiming
from .human_impedance_6d import HumanImpedanceConfig
from .trajectory_6d import TrajectoryConfig


def _vec3(mapping: dict[str, Any], key: str) -> np.ndarray:
    value = np.asarray(mapping[key], dtype=np.float64)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{key} must be a finite 3-vector")
    return value


def _resolve_trajectory_block(trajectory: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """Select the active preset; returns (effective block, robot overrides)."""
    presets = trajectory.get("presets")
    if not isinstance(presets, dict) or not presets:
        return trajectory, {}
    active = str(trajectory.get("active", "sinusoid"))
    if active not in presets:
        raise ValueError(f"unknown trajectory scenario: {active}")
    preset = dict(presets[active] or {})
    robot_overrides = dict(preset.pop("robot_overrides", {}))
    return preset, robot_overrides


@dataclass(frozen=True)
class ComparisonConfig:
    source_path: Path
    raw: dict[str, Any]
    timing: ExperimentTiming
    trajectory: TrajectoryConfig
    human_impedance: HumanImpedanceConfig
    hand_offset: np.ndarray
    publish_rate_hz: float
    output_root: str
    frame_id: str
    human_only_model: str
    human_robot_model: str
    robot_overrides: dict[str, Any]

    @property
    def config_hash(self) -> str:
        canonical = json.dumps(self.raw, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def load_comparison_config(path: str | Path) -> ComparisonConfig:
    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict) or int(raw.get("schema_version", -1)) != 1:
        raise ValueError("unsupported comparison config schema")
    experiment = raw["experiment"]
    trajectory_raw = raw["trajectory"]
    impedance = raw["human_impedance"]
    models = raw["models"]
    timing = ExperimentTiming(
        settle_sec=float(experiment["settle_sec"]),
        tracking_sec=float(experiment["tracking_sec"]),
        hold_sec=float(experiment["hold_sec"]),
    )
    trajectory_block, robot_overrides = _resolve_trajectory_block(trajectory_raw)
    if "tracking_sec" in trajectory_block:
        # Scenario presets may lengthen the tracking phase; timing and the
        # trajectory duration must stay coupled.
        timing = replace(timing, tracking_sec=float(trajectory_block["tracking_sec"]))
    if "waypoints" in trajectory_block:
        waypoints = np.asarray(trajectory_block["waypoints"], dtype=np.float64)
        waypoints[:, 2] = np.deg2rad(waypoints[:, 2])
        segment_durations = trajectory_block.get("segment_durations_sec")
        trajectory_config = TrajectoryConfig(
            duration_sec=timing.tracking_sec,
            cycles=1.0,
            position_amplitude=np.zeros(3),
            orientation_amplitude=np.zeros(3),
            mode="waypoints",
            waypoints=waypoints,
            segment_durations_sec=(
                np.asarray(segment_durations, dtype=np.float64)
                if segment_durations is not None
                else None
            ),
        )
    else:
        trajectory_config = TrajectoryConfig(
            duration_sec=timing.tracking_sec,
            cycles=float(trajectory_block["cycles"]),
            position_amplitude=_vec3(trajectory_block, "position_amplitude_m"),
            orientation_amplitude=_vec3(trajectory_block, "orientation_amplitude_rad"),
        )
    return ComparisonConfig(
        source_path=source,
        raw=raw,
        timing=timing,
        trajectory=trajectory_config,
        human_impedance=HumanImpedanceConfig(
            linear_stiffness=_vec3(impedance, "linear_stiffness"),
            linear_damping=_vec3(impedance, "linear_damping"),
            angular_stiffness=_vec3(impedance, "angular_stiffness"),
            angular_damping=_vec3(impedance, "angular_damping"),
            force_limit=_vec3(impedance, "force_limit"),
            task_torque_limit=_vec3(impedance, "task_torque_limit"),
        ),
        hand_offset=_vec3(experiment, "human_hand_offset"),
        publish_rate_hz=max(float(experiment["publish_rate_hz"]), 1.0),
        output_root=str(experiment["output_root"]),
        frame_id=str(experiment["frame_id"]),
        human_only_model=str(models["human_only"]),
        human_robot_model=str(models["human_robot"]),
        robot_overrides=robot_overrides,
    )

