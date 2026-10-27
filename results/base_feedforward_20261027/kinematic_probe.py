"""Isolated instantaneous kinematics probe; never launches or commands the robot.

Synthetic base references test allocation only, not intent inference or dynamics.
Stage 1 reproduces the current default Cartesian/regularization objective.
Stage 2 locks its achieved six-dimensional twist while preferring a base twist.
"""
from pathlib import Path
import hashlib
import json

import mujoco
import numpy as np
import osqp
import scipy.sparse as sp
import yaml

from pr2_wbc_admittance_control.pr2_dynamics_utils import (
    get_ee_jacobian_6x10,
    make_reduced_dof_index,
)


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
config = ROOT / "pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml"
scene = ROOT / "unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml"
cfg = yaml.safe_load(config.read_text())
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)
for name, value in json.loads(cfg["robot"]["initial_qpos_json"]).items():
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[model.jnt_qposadr[jid]] = value
mujoco.mj_forward(model, data)
arm = [f"l_{part}_joint" for part in (
    "shoulder_pan", "shoulder_lift", "upper_arm_roll", "elbow_flex",
    "forearm_roll", "wrist_flex", "wrist_roll",
)]
indices = make_reduced_dof_index(model, arm)
ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_gripper_tool_frame")
J = get_ee_jacobian_6x10(model, data, ee, indices)
E = np.eye(10)[7:]


def solve(P, q, A, lower, upper):
    qp = osqp.OSQP()
    qp.setup(P=sp.csc_matrix(P), q=q, A=sp.csc_matrix(A), l=lower, u=upper,
             verbose=False, eps_abs=1e-9, eps_rel=1e-9, max_iter=50000,
             polishing=True)
    result = qp.solve()
    if result.info.status_val != 1:
        raise RuntimeError(result.info.status)
    return result.x


rows = []
hold = np.zeros(6)
moving = np.array([0.0, 0.03, 0.0, 0.0, 0.0, 0.04])
for name, task, reference, arm_limit in [
    ("hold_base_x", hold, [0.05, 0, 0], 10.0),
    ("hold_base_y", hold, [0, 0.05, 0], 10.0),
    ("hold_base_yaw", hold, [0, 0, 0.1], 10.0),
    ("moving_task", moving, [0.04, 0.02, 0.08], 10.0),
    ("restricted_arm", hold, [0.2, 0.2, 0.4], 0.01),
]:
    reference = np.array(reference, dtype=float)
    limit = np.array([arm_limit] * 7 + [1.0, 1.0, 1.5])
    # No new base objective in stage 1; same default W_ee=I and W_reg=.01I.
    u0 = solve(J.T @ J + 0.01 * np.eye(10), -J.T @ task,
               np.eye(10), -limit, limit)
    achieved = J @ u0
    # Secondary base tracking, with a small positive-definite tie breaker.
    epsilon = 1e-6
    u1 = solve(E.T @ E + epsilon * np.eye(10),
               -E.T @ reference - epsilon * u0,
               np.vstack([J, np.eye(10)]),
               np.r_[achieved, -limit], np.r_[achieved, limit])
    # Deliberately naive post-QP addition, for an algebraic comparison only.
    naive = u0.copy()
    naive[7:] += reference
    row = dict(case=name, synthetic_task_twist=task.tolist(),
               requested_base_world=reference.tolist(),
               baseline_u=u0.tolist(), allocated_u=u1.tolist(),
               baseline_task_residual=(achieved-task).tolist(),
               allocated_base_world=u1[7:].tolist(),
               task_twist_change=(J @ (u1-u0)).tolist(),
               naive_post_qp_task_twist_change=(J @ (naive-u0)).tolist(),
               bound_violation=float(np.maximum(abs(u1)-limit, 0).max()))
    assert np.max(abs(J @ (u1-u0))) < 1e-7
    assert row["bound_violation"] < 1e-7
    rows.append(row)

sources = [config, scene,
           ROOT / "unitree_mujoco/unitree_robots/pr2/robot_pr2_board_grasp.xml",
           ROOT / "pr2_ws/src/pr2_wbc_admittance_control/pr2_wbc_admittance_control/pr2_qp_whole_body_admittance.py"]
result = dict(scope="One configured initial posture; instantaneous kinematics only. "
                   "Synthetic task/base references; not closed-loop friction-grasp validation. "
                   "No-wrench base-freeze logic is deliberately outside this allocation probe.",
              jacobian_rank=int(np.linalg.matrix_rank(J)),
              nullspace_dimension=10-int(np.linalg.matrix_rank(J)),
              source_hashes={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sources},
              cases=rows)
(OUT / "kinematic_probe.json").write_text(json.dumps(result, indent=2))
for row in rows:
    print(row["case"], "base=", np.round(row["allocated_base_world"], 6),
          "max_task_change=", max(abs(x) for x in row["task_twist_change"]))
