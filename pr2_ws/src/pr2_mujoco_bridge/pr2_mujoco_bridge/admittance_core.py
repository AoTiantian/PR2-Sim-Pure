from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _as_array(values: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    return np.asarray(values, dtype=np.float64)


def apply_deadband(vec: np.ndarray, threshold: np.ndarray) -> np.ndarray:
    out = vec.copy()
    mask = np.abs(out) < threshold
    out[mask] = 0.0
    return out


def clip_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm or max_norm <= 0.0:
        return vec
    return vec * (max_norm / norm)


def solve_dls_velocity(
    jacobian: np.ndarray,
    ee_velocity: np.ndarray,
    damping_lambda: float,
) -> np.ndarray:
    jac = _as_array(jacobian)
    vel = _as_array(ee_velocity)
    reg = (damping_lambda ** 2) * np.eye(jac.shape[0], dtype=np.float64)
    return jac.T @ np.linalg.solve(jac @ jac.T + reg, vel)


def limit_joint_velocity_near_limits(
    q: np.ndarray,
    qdot_des: np.ndarray,
    qmin: np.ndarray,
    qmax: np.ndarray,
    margin: float,
) -> np.ndarray:
    limited = _as_array(qdot_des).copy()
    q_arr = _as_array(q)
    qmin_arr = _as_array(qmin)
    qmax_arr = _as_array(qmax)
    for i in range(len(limited)):
        if qmax_arr[i] <= qmin_arr[i]:
            continue
        if q_arr[i] < qmin_arr[i] + margin and limited[i] < 0.0:
            limited[i] *= max(0.0, (q_arr[i] - qmin_arr[i]) / margin)
        elif q_arr[i] > qmax_arr[i] - margin and limited[i] > 0.0:
            limited[i] *= max(0.0, (qmax_arr[i] - q_arr[i]) / margin)
    return limited


def rotate_world_vector_to_body(
    world_vector: np.ndarray,
    rot_world_body: np.ndarray,
) -> np.ndarray:
    return _as_array(rot_world_body).T @ _as_array(world_vector)


def rotate_body_vector_to_world(
    body_vector: np.ndarray,
    rot_world_body: np.ndarray,
) -> np.ndarray:
    return _as_array(rot_world_body) @ _as_array(body_vector)


def project_end_effector_wrench_to_base(
    ee_pos_in_base: np.ndarray,
    ee_force_in_base: np.ndarray,
    planar_scale: float,
    yaw_scale: float,
) -> np.ndarray:
    pos = _as_array(ee_pos_in_base)
    force = _as_array(ee_force_in_base)
    out = np.zeros(6, dtype=np.float64)
    out[0] = force[0] * planar_scale
    out[1] = force[1] * planar_scale
    out[5] = (pos[0] * force[1] - pos[1] * force[0]) * yaw_scale
    return out


@dataclass(frozen=True)
class AxisGains:
    mass: np.ndarray
    damping: np.ndarray
    stiffness: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "mass", _as_array(self.mass))
        object.__setattr__(self, "damping", _as_array(self.damping))
        object.__setattr__(self, "stiffness", _as_array(self.stiffness))


@dataclass(frozen=True)
class AdmittanceState:
    displacement: np.ndarray
    velocity: np.ndarray

    @classmethod
    def zeros(cls, dim: int) -> "AdmittanceState":
        return cls(
            displacement=np.zeros(dim, dtype=np.float64),
            velocity=np.zeros(dim, dtype=np.float64),
        )

    def __post_init__(self) -> None:
        object.__setattr__(self, "displacement", _as_array(self.displacement))
        object.__setattr__(self, "velocity", _as_array(self.velocity))

    def step(
        self,
        force: np.ndarray,
        gains: AxisGains,
        dt: float,
    ) -> "AdmittanceState":
        force_arr = _as_array(force)
        mass = np.maximum(gains.mass, 1e-6)
        accel = (force_arr - gains.damping * self.velocity - gains.stiffness * self.displacement) / mass
        velocity = self.velocity + accel * dt
        displacement = self.displacement + velocity * dt
        return AdmittanceState(displacement=displacement, velocity=velocity)
