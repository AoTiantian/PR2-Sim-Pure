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

    def __post_init__(self) -> None:
        if self.duration_sec <= 0.0 or self.cycles <= 0.0:
            raise ValueError("trajectory duration and cycles must be positive")
        for name, value in (("position_amplitude", self.position_amplitude), ("orientation_amplitude", self.orientation_amplitude)):
            array = np.asarray(value, dtype=np.float64)
            if array.shape != (3,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must be a finite 3-vector")
            object.__setattr__(self, name, array)


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

