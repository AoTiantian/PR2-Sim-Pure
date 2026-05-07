from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

try:
    import mujoco
except Exception as _exc:  # pragma: no cover
    mujoco = None  # type: ignore


@dataclass(frozen=True)
class DofIndex:
    """Reduced DOF indexing for whole-body QP.

    We use a 10-DOF reduced velocity vector:
      u = [qdot_arm(7), vx, vy, wz]
    """

    arm_vadr: np.ndarray  # (7,)
    base_free_vadr0: int  # start dofadr for free joint (size 6)

    @property
    def base_vadr_vx(self) -> int:
        return int(self.base_free_vadr0 + 0)

    @property
    def base_vadr_vy(self) -> int:
        return int(self.base_free_vadr0 + 1)

    @property
    def base_vadr_wz(self) -> int:
        return int(self.base_free_vadr0 + 5)


def _require_mujoco() -> None:
    if mujoco is None:
        raise RuntimeError(
            "MuJoCo python bindings are not available. "
            "Ensure `mujoco` is installed in the environment."
        )


def find_joint_dofadr(model, joint_names: Sequence[str]) -> np.ndarray:
    """Return dofadr indices (vadr) for a list of 1-DoF joints."""
    _require_mujoco()
    out: list[int] = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(jn))
        if jid < 0:
            raise ValueError(f"joint not found: {jn}")
        vadr = int(model.jnt_dofadr[jid])
        out.append(vadr)
    return np.array(out, dtype=np.int32)


def find_base_freejoint_vadr0(model, freejoint_name: str = "base_free") -> int:
    """Return the starting dofadr (vadr0) for the base free joint."""
    _require_mujoco()
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(freejoint_name))
    if jid < 0:
        raise ValueError(f"base free joint not found: {freejoint_name}")
    if int(model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise ValueError(f"joint '{freejoint_name}' is not a free joint")
    return int(model.jnt_dofadr[jid])


def make_reduced_dof_index(
    model,
    arm_joint_names: Sequence[str],
    base_freejoint_name: str = "base_free",
) -> DofIndex:
    arm_vadr = find_joint_dofadr(model, arm_joint_names)
    if arm_vadr.shape != (7,):
        raise ValueError(f"expected 7 arm joints, got {arm_vadr.shape}")
    base_vadr0 = find_base_freejoint_vadr0(model, base_freejoint_name)
    return DofIndex(arm_vadr=arm_vadr, base_free_vadr0=base_vadr0)


def get_full_mass_matrix(model, data) -> np.ndarray:
    """Return full mass matrix M (nv, nv)."""
    _require_mujoco()
    # MuJoCo stores M in a sparse format in data.qM; mj_fullM expands it.
    M = np.zeros((model.nv, model.nv), dtype=np.float64)
    mujoco.mj_fullM(model, M, data.qM)
    return M


def get_bias_forces(model, data) -> np.ndarray:
    """Return bias generalized forces h (nv,)."""
    _require_mujoco()
    # data.qfrc_bias is valid after mj_forward/mj_step.
    return np.array(data.qfrc_bias, dtype=np.float64)


def get_arm_bias_forces(model, data, arm_vadr: Sequence[int]) -> np.ndarray:
    h = get_bias_forces(model, data)
    return h[np.array(list(arm_vadr), dtype=np.int32)]


def get_ee_jacobian_6xn(model, data, ee_body_id: int) -> np.ndarray:
    """Return full 6×nv body Jacobian (rows: [Jp; Jr])."""
    _require_mujoco()
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacBody(model, data, jacp, jacr, int(ee_body_id))
    return np.vstack([jacp, jacr])


def get_ee_jacobian_6x10(
    model,
    data,
    ee_body_id: int,
    dof: DofIndex,
) -> np.ndarray:
    """Return reduced 6×10 Jacobian with columns [arm_7, base_vx, base_vy, base_wz]."""
    j6nv = get_ee_jacobian_6xn(model, data, ee_body_id)
    cols = list(map(int, dof.arm_vadr)) + [dof.base_vadr_vx, dof.base_vadr_vy, dof.base_vadr_wz]
    return j6nv[:, np.array(cols, dtype=np.int32)]

