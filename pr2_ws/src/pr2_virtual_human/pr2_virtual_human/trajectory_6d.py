"""One deterministic six-dimensional trajectory shared by both conditions."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .spatial import matrix_to_quaternion, quaternion_to_matrix, rotvec_to_matrix, so3_left_jacobian


@dataclass(frozen=True)
class TrajectoryConfig:
    duration_sec: float
    cycles: float
    position_amplitude: np.ndarray
    orientation_amplitude: np.ndarray
    mode: str = "sinusoid"
    waypoints: np.ndarray | None = None  # rows [dx, dy, yaw_rad], anchor frame
    segment_durations_sec: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.duration_sec <= 0.0 or self.cycles <= 0.0:
            raise ValueError("trajectory duration and cycles must be positive")
        for name, value in (("position_amplitude", self.position_amplitude), ("orientation_amplitude", self.orientation_amplitude)):
            array = np.asarray(value, dtype=np.float64)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite 3-vector")
            object.__setattr__(self, name, array)
        if self.mode not in ("sinusoid", "waypoints"):
            raise ValueError("trajectory mode must be sinusoid or waypoints")
        if self.mode == "waypoints":
            waypoints = np.asarray(self.waypoints, dtype=np.float64)
            if waypoints.ndim != 2 or waypoints.shape[1] != 3 or waypoints.shape[0] < 2:
                raise ValueError("waypoints must be an (N,3) array of [dx, dy, yaw_rad] with N >= 2")
            if not np.all(np.isfinite(waypoints)):
                raise ValueError("waypoints must be finite")
            if not np.allclose(waypoints[0], 0.0):
                raise ValueError("the first waypoint must be [0, 0, 0] (the latched anchor)")
            object.__setattr__(self, "waypoints", waypoints)
            if self.segment_durations_sec is not None:
                durations = np.asarray(self.segment_durations_sec, dtype=np.float64)
                if durations.shape != (waypoints.shape[0] - 1,) or np.any(durations <= 0.0):
                    raise ValueError("segment_durations_sec must be positive with len == waypoints - 1")
                object.__setattr__(self, "segment_durations_sec", durations)


@dataclass(frozen=True)
class TrajectorySample:
    position: np.ndarray
    linear_velocity: np.ndarray
    linear_acceleration: np.ndarray
    quaternion: np.ndarray
    angular_velocity: np.ndarray
    relative_rotvec: np.ndarray


class Trajectory6D:
    def __init__(self, config: TrajectoryConfig, initial_position: np.ndarray, initial_quaternion: np.ndarray) -> None:
        self.config = config
        self.initial_position = np.asarray(initial_position, dtype=np.float64).copy()
        self.initial_quaternion = np.asarray(initial_quaternion, dtype=np.float64).copy()
        if self.initial_position.shape != (3,) or self.initial_quaternion.shape != (4,):
            raise ValueError("initial pose must contain position[3] and quaternion[4]")
        self._initial_rotation = quaternion_to_matrix(self.initial_quaternion)

    @staticmethod
    def _smooth_phase(u: float) -> tuple[float, float, float]:
        u = float(np.clip(u, 0.0, 1.0))
        value = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
        first = 30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4
        second = 60.0 * u - 180.0 * u ** 2 + 120.0 * u ** 3
        return value, first, second

    def sample(self, elapsed_sec: float) -> TrajectorySample:
        duration = self.config.duration_sec
        u = float(np.clip(elapsed_sec / duration, 0.0, 1.0))
        phase, phase_u, phase_uu = self._smooth_phase(u)
        omega_scale = 2.0 * math.pi * self.config.cycles
        theta = omega_scale * phase
        theta_dot = omega_scale * phase_u / duration
        theta_ddot = omega_scale * phase_uu / (duration * duration)

        harmonics = np.array([1.0, 2.0, 1.0])
        angles = harmonics * theta
        angle_dots = harmonics * theta_dot
        angle_ddots = harmonics * theta_ddot
        amplitudes = self.config.position_amplitude
        offset = amplitudes * np.sin(angles)
        velocity = amplitudes * np.cos(angles) * angle_dots
        acceleration = amplitudes * (np.cos(angles) * angle_ddots - np.sin(angles) * angle_dots ** 2)

        orientation_amplitudes = self.config.orientation_amplitude
        rotvec = orientation_amplitudes * np.sin(angles)
        rotvec_dot = orientation_amplitudes * np.cos(angles) * angle_dots
        relative_rotation = rotvec_to_matrix(rotvec)
        target_rotation = self._initial_rotation @ relative_rotation
        angular_velocity_body0 = so3_left_jacobian(rotvec) @ rotvec_dot
        angular_velocity_world = self._initial_rotation @ angular_velocity_body0
        return TrajectorySample(
            position=self.initial_position + offset,
            linear_velocity=velocity,
            linear_acceleration=acceleration,
            quaternion=matrix_to_quaternion(target_rotation),
            angular_velocity=angular_velocity_world,
            relative_rotvec=rotvec,
        )


class WaypointTrajectory6D:
    """Anchor-relative planar waypoint sequence traversed with per-segment
    quintic smooth-phase profiles (zero velocity/acceleration at every
    waypoint).  Waypoint rows are ``[dx, dy, yaw_rad]`` in the anchor's world
    frame; the board stays level and only yaw changes relative to the anchor.
    """

    # Yaw-to-translation weighting for automatic segment durations (m per rad).
    _YAW_EQUIV_METERS_PER_RAD = 0.4

    def __init__(self, config: TrajectoryConfig, initial_position: np.ndarray, initial_quaternion: np.ndarray) -> None:
        if config.mode != "waypoints" or config.waypoints is None:
            raise ValueError("WaypointTrajectory6D requires mode='waypoints' with waypoints")
        waypoints = np.asarray(config.waypoints, dtype=np.float64)
        self.initial_position = np.asarray(initial_position, dtype=np.float64).copy()
        self.duration = float(config.duration_sec)
        self._initial_rotation = quaternion_to_matrix(initial_quaternion)

        positions = self.initial_position + np.column_stack(
            [waypoints[:, 0], waypoints[:, 1], np.zeros(len(waypoints))]
        )
        rotvecs = np.column_stack(
            [np.zeros(len(waypoints)), np.zeros(len(waypoints)), waypoints[:, 2]]
        )
        position_delta = np.diff(positions, axis=0)
        rotvec_delta = np.diff(rotvecs, axis=0)

        if config.segment_durations_sec is not None:
            durations = np.asarray(config.segment_durations_sec, dtype=np.float64)
        else:
            lengths = np.linalg.norm(position_delta, axis=1) + (
                self._YAW_EQUIV_METERS_PER_RAD * np.abs(rotvec_delta[:, 2])
            )
            lengths = np.where(lengths > 1.0e-9, lengths, 1.0e-9)
            durations = self.duration * lengths / lengths.sum()
        self._segment_duration = durations
        self._segment_start_time = np.concatenate(([0.0], np.cumsum(durations)[:-1]))
        self._start_position = positions[:-1]
        self._position_delta = position_delta
        self._start_rotvec = rotvecs[:-1]
        self._rotvec_delta = rotvec_delta

    def sample(self, elapsed_sec: float) -> TrajectorySample:
        t = float(np.clip(elapsed_sec, 0.0, self.duration))
        index = int(np.searchsorted(self._segment_start_time, t, side="right")) - 1
        index = int(np.clip(index, 0, len(self._segment_duration) - 1))
        segment_duration = max(float(self._segment_duration[index]), 1.0e-9)
        u = (t - float(self._segment_start_time[index])) / segment_duration
        phase, phase_dot, phase_ddot = Trajectory6D._smooth_phase(u)

        position = self._start_position[index] + self._position_delta[index] * phase
        velocity = self._position_delta[index] * (phase_dot / segment_duration)
        acceleration = self._position_delta[index] * (phase_ddot / segment_duration**2)

        rotvec = self._start_rotvec[index] + self._rotvec_delta[index] * phase
        rotvec_dot = self._rotvec_delta[index] * (phase_dot / segment_duration)
        rotation = self._initial_rotation @ rotvec_to_matrix(rotvec)
        angular_velocity_world = self._initial_rotation @ (
            so3_left_jacobian(rotvec) @ rotvec_dot
        )
        return TrajectorySample(
            position=position,
            linear_velocity=velocity,
            linear_acceleration=acceleration,
            quaternion=matrix_to_quaternion(rotation),
            angular_velocity=angular_velocity_world,
            relative_rotvec=rotvec,
        )


def build_trajectory(
    config: TrajectoryConfig, position: np.ndarray, quaternion: np.ndarray
) -> Trajectory6D | WaypointTrajectory6D:
    """Create the trajectory object selected by ``config.mode``."""
    if config.mode == "waypoints":
        return WaypointTrajectory6D(config, position, quaternion)
    return Trajectory6D(config, position, quaternion)

