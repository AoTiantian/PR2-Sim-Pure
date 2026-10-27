"""Known base-path feedforward with angular admittance priority.

This is an instantaneous allocation experiment, not a robot controller.
It deliberately permits nominal endpoint translation, unlike kinematic_probe.
"""
from pathlib import Path
import json

import mujoco
import numpy as np
import osqp
import scipy.sparse as sp
import yaml

from pr2_wbc_admittance_control.pr2_dynamics_utils import (
    get_ee_jacobian_6x10, make_reduced_dof_index,
)
from pr2_virtual_human.trajectory_6d import TrajectoryConfig, build_trajectory


ROOT = Path(__file__).resolve().parents[2]
cfg = yaml.safe_load((ROOT / "pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml").read_text())
model = mujoco.MjModel.from_xml_path(str(ROOT / "unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml"))
data = mujoco.MjData(model)
for name, value in json.loads(cfg["robot"]["initial_qpos_json"]).items():
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[model.jnt_qposadr[jid]] = value
mujoco.mj_forward(model, data)
arm = [f"l_{x}_joint" for x in (
    "shoulder_pan", "shoulder_lift", "upper_arm_roll", "elbow_flex",
    "forearm_roll", "wrist_flex", "wrist_roll",
)]
ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_gripper_tool_frame")
J = get_ee_jacobian_6x10(model, data, ee, make_reduced_dof_index(model, arm))
Jp, Jr = J[:3], J[3:]
E = np.eye(10)[7:]


def solve(P, q, A, lo, hi):
    qp = osqp.OSQP()
    qp.setup(P=sp.csc_matrix(P), q=q, A=sp.csc_matrix(A), l=lo, u=hi,
             eps_abs=1e-9, eps_rel=1e-9, max_iter=50000, verbose=False,
             polishing=True)
    r = qp.solve()
    if r.info.status_val != 1:
        raise RuntimeError(r.info.status)
    return r.x


def allocate(v_adm, base_reference, arm_limit=10.0):
    limits = np.array([arm_limit] * 7 + [1.0, 1.0, 1.5])
    # First priority: best achievable original angular-admittance velocity.
    ua = solve(Jr.T @ Jr + 1e-12 * np.eye(10), -Jr.T @ v_adm[3:],
               np.eye(10), -limits, limits)
    angular_achieved = Jr @ ua
    # Nominal transport translation plus original force-induced translation.
    # Base yaw enters this linear velocity via the base-to-EE lever arm.
    linear_reference = v_adm[:3] + Jp[:, 7:] @ base_reference
    P = Jp.T @ Jp + E.T @ E + 0.01 * np.eye(10)
    q = -Jp.T @ linear_reference - E.T @ base_reference
    A = np.vstack([np.eye(10), Jr])
    u = solve(P, q, A, np.r_[-limits, angular_achieved],
              np.r_[limits, angular_achieved])
    assert max(abs(Jr @ u - angular_achieved)) < 1e-7
    assert max(np.maximum(abs(u) - limits, 0)) < 1e-7
    return u, linear_reference, angular_achieved


# A separately specified robot path, anchored at the robot, not a subscription
# to the virtual human's desired hand pose. Same geometry as the S22 test task.
path_cfg = TrajectoryConfig(
    duration_sec=24.0, cycles=1.0,
    position_amplitude=np.zeros(3), orientation_amplitude=np.zeros(3),
    mode="waypoints", waypoints=np.array([[0, 0, 0], [.3, -.5, 0], [-.3, -1, 0]]),
)
base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
path = build_trajectory(path_cfg, data.xpos[base_id], np.array([1., 0, 0, 0]))
rows = []
for name, v_adm, b_ref, limit in [
    ("zero_wrench_base_x", np.zeros(6), np.array([.05, 0, 0]), 10.0),
    ("zero_wrench_base_yaw", np.zeros(6), np.array([0, 0, .1]), 10.0),
    ("angular_admittance_nonzero", np.array([0., 0, 0, .02, -.01, .03]),
     np.array([.03, -.04, .1]), 10.0),
    ("limited_arm", np.array([0., 0, 0, .02, -.01, .03]),
     np.array([.3, -.3, .5]), .01),
]:
    u, linear_reference, angular_achieved = allocate(v_adm, b_ref, limit)
    rows.append(dict(case=name, synthetic_admittance_twist=v_adm.tolist(),
                     requested_base_world=b_ref.tolist(), allocated_u=u.tolist(),
                     endpoint_twist=(J @ u).tolist(),
                     linear_task_reference=linear_reference.tolist(),
                     primary_angular_residual=(angular_achieved-v_adm[3:]).tolist(),
                     secondary_angular_change=(Jr @ u-angular_achieved).tolist()))
samples = []
for t in np.linspace(0, 24, 49):
    s = path.sample(float(t))
    ref = np.r_[s.linear_velocity[:2], 0.0]
    u, _, angular = allocate(np.zeros(6), ref)
    samples.append(dict(time=float(t), base_path_position=s.position.tolist(),
                        base_reference=ref.tolist(), allocated_u=u.tolist(),
                        endpoint_twist=(J @ u).tolist(), angular_target=angular.tolist()))
result = dict(
    scope="Instantaneous kinematics at one initial configuration, without time integration, "
          "plant, ROS, contact dynamics, actual force input or post-QP filtering. "
          "Path samples are independent allocations at this same configuration, not a rollout.",
    architecture="Known robot path provides planar nominal translation; original angular "
                 "admittance is the primary task. Linear admittance is a correction about "
                 "the nominal motion, not the old absolute world-velocity task.",
    cases=rows, path_samples_at_fixed_configuration=samples,
)
out = Path(__file__).with_suffix(".json")
out.write_text(json.dumps(result, indent=2))
for r in rows:
    print(r["case"], "EE twist=", np.round(r["endpoint_twist"], 6),
          "angular allocation change=", max(abs(x) for x in r["secondary_angular_change"]))
print("Saved", out)
