"""Minimal human endpoint impedance with no robot-aware compensation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .spatial import orientation_error_world
from .trajectory_6d import TrajectorySample


def _vec3(name: str, value: np.ndarray) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return array


@dataclass(frozen=True)
class HumanImpedanceConfig:
    linear_stiffness: np.ndarray
    linear_damping: np.ndarray
    angular_stiffness: np.ndarray
    angular_damping: np.ndarray
    force_limit: np.ndarray
    task_torque_limit: np.ndarray

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, _vec3(name, getattr(self, name)))
        if np.any(self.linear_stiffness < 0.0) or np.any(self.linear_damping < 0.0):
            raise ValueError("linear impedance gains must be non-negative")
        if np.any(self.angular_stiffness < 0.0) or np.any(self.angular_damping < 0.0):
            raise ValueError("angular impedance gains must be non-negative")
        if np.any(self.force_limit <= 0.0) or np.any(self.task_torque_limit <= 0.0):
            raise ValueError("wrench limits must be positive")


@dataclass(frozen=True)
class HumanWrench:
    force: np.ndarray
    task_torque: np.ndarray
    position_error: np.ndarray
    orientation_error: np.ndarray


class HumanImpedance6D:
    """Task impedance only.

    There is deliberately no robot state, load split, gravity feed-forward,
    lever-arm cancellation, level hold, filter, or force-allocation input.
    """

    def __init__(self, config: HumanImpedanceConfig) -> None:
        self.config = config

    def evaluate(
        self,
        target: TrajectorySample,
        actual_position: np.ndarray,
        actual_linear_velocity: np.ndarray,
        actual_quaternion: np.ndarray,
        actual_angular_velocity: np.ndarray,
    ) -> HumanWrench:
        position = _vec3("actual_position", actual_position)
        velocity = _vec3("actual_linear_velocity", actual_linear_velocity)
        angular_velocity = _vec3("actual_angular_velocity", actual_angular_velocity)
        position_error = target.position - position
        velocity_error = target.linear_velocity - velocity
        orientation_error = orientation_error_world(actual_quaternion, target.quaternion)
        angular_velocity_error = target.angular_velocity - angular_velocity
        force = np.clip(
            self.config.linear_stiffness * position_error + self.config.linear_damping * velocity_error,
            -self.config.force_limit,
            self.config.force_limit,
        )
        task_torque = np.clip(
            self.config.angular_stiffness * orientation_error + self.config.angular_damping * angular_velocity_error,
            -self.config.task_torque_limit,
            self.config.task_torque_limit,
        )
        return HumanWrench(force, task_torque, position_error, orientation_error)


def wrench_at_body_com(
    endpoint_force_world: np.ndarray,
    task_torque_world: np.ndarray,
    endpoint_offset_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map endpoint wrench to body COM without cancelling its natural moment."""
    force = _vec3("endpoint_force_world", endpoint_force_world)
    task_torque = _vec3("task_torque_world", task_torque_world)
    offset = _vec3("endpoint_offset_world", endpoint_offset_world)
    natural_moment = np.cross(offset, force)
    return force.copy(), natural_moment + task_torque, natural_moment

