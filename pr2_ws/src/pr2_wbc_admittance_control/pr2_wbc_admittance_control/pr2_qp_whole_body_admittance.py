#!/usr/bin/env python3
"""
统一 QP 全身导纳控制（移动底座 + 左臂 7 轴，不含躯干）。

## 数学形式（速度层 QP，10-DOF）

决策变量（Reduced velocity）：
  u = [ qdot_arm(7), vx, vy, wz ]^T  ∈ R^10

末端任务速度由导纳律给出（与 `pr2_arm_admittance_controller` 同形）：
  v_des = B^{-1}( F_ext - K * dx )
其中 v_des 为 6D twist（linear xyz + angular xyz），dx 为相对基准位姿的 6D 小量误差。

QP（加权最小二乘 + box 约束）：
  minimize  || J_ee(u) - v_des ||^2_{W_ee} + || u ||^2_{W_reg}
  subject to u_min ≤ u ≤ u_max

实现上写成标准二次型：
  1/2 u^T P u + q^T u
  s.t.  l ≤ A u ≤ r
其中
  P = J^T W_ee J + W_reg
  q = - J^T W_ee v_des
  A = I
  l = u_min, r = u_max

## 输出接口（与现有 WBC 管线对齐）
发布：
  - `wbc/reference/cmd_vel` (Twist)            : 底座速度 (vx,vy,wz)
  - `wbc/reference/joint_command` (JointState): 左臂关节速度（velocity 字段）

下游可继续复用 `pr2_wbc_coordinator` 汇总并输出到仿真。
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import mujoco
import numpy as np
import osqp
import scipy.sparse as sp
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

from .pr2_dynamics_utils import DofIndex, get_ee_jacobian_6x10, make_reduced_dof_index


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _apply_deadzone(v: float, dz: float) -> float:
    if abs(v) <= dz:
        return 0.0
    return v - dz if v > 0.0 else v + dz


def _quat_wxyz_to_rotmat(q_wxyz: np.ndarray) -> np.ndarray:
    mat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(mat, q_wxyz)
    return mat.reshape(3, 3)


def _orientation_error_world(cur_q_wxyz: np.ndarray, tgt_q_wxyz: np.ndarray) -> np.ndarray:
    """Small-angle orientation error vector (world frame)."""
    rc = _quat_wxyz_to_rotmat(cur_q_wxyz)
    rt = _quat_wxyz_to_rotmat(tgt_q_wxyz)
    return 0.5 * (
        np.cross(rc[:, 0], rt[:, 0])
        + np.cross(rc[:, 1], rt[:, 1])
        + np.cross(rc[:, 2], rt[:, 2])
    )


@dataclass
class LatchState:
    base_pose: Optional[PoseStamped] = None
    latched: bool = False
    wrench_was_active: bool = False


class Pr2QpWholeBodyAdmittance(Node):
    def __init__(self) -> None:
        super().__init__("pr2_qp_whole_body_admittance")

        # #region agent log
        self._dbg_log_path = "/workspace/.cursor/debug-a24b67.log"
        self._debug_trace = os.environ.get("PR2_DEBUG_TRACE", "0").lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        self._dbg_last_mono = 0.0
        self._dbg_prev_wrench_active: Optional[bool] = None
        self._dbg_last_cmdvel_mono = 0.0

        def _dbg_write(hypothesis_id: str, message: str, data: dict) -> None:
            if not self._debug_trace:
                return
            try:
                import json as _json

                payload = {
                    "sessionId": "a24b67",
                    "runId": os.environ.get("DEBUG_RUN_ID", "qp_whole_body"),
                    "hypothesisId": hypothesis_id,
                    "location": "pr2_qp_whole_body_admittance.py",
                    "message": message,
                    "data": data,
                    "timestamp": int(time.time() * 1000),
                }
                with open(self._dbg_log_path, "a", encoding="utf-8") as f:
                    f.write(_json.dumps(payload, ensure_ascii=False) + "\n")
            except Exception:
                pass

        self._dbg_write = _dbg_write
        self._dbg_write("H0_DebugInit", "qp init", {"pid": int(os.getpid())})
        # #endregion agent log

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        )
        self.declare_parameter("end_effector_body", "l_gripper_tool_frame")
        self.declare_parameter("arm_joint_names", [
            "l_shoulder_pan_joint",
            "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint",
            "l_elbow_flex_joint",
            "l_forearm_roll_joint",
            "l_wrist_flex_joint",
            "l_wrist_roll_joint",
        ])
        self.declare_parameter("base_freejoint_name", "base_free")

        self.declare_parameter("input_wrench_topic", "wbc/whole_body_wrench")
        self.declare_parameter("ee_pose_topic", "ee_pose")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("state_joint_topic", "state/joint_states")

        self.declare_parameter("output_cmd_vel_topic", "wbc/reference/cmd_vel")
        self.declare_parameter("output_joint_command_topic", "wbc/reference/joint_command")
        # Back-compat with existing logger/plots
        self.declare_parameter("output_cartesian_velocity_topic", "arm_cartesian_velocity")
        self.declare_parameter("base_pose_latched_topic", "base_pose_latched")

        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("wrench_lpf_alpha", 0.15)

        self.declare_parameter("damping_linear", [320.0, 320.0, 400.0])
        self.declare_parameter("stiffness_linear", [260.0, 260.0, 320.0])
        self.declare_parameter("force_deadzone", [0.8, 0.8, 0.8])
        self.declare_parameter("force_despring_thresh", [20.0, 20.0, 30.0])
        # NOTE: z-hold typically needs more authority against gravity than x/y.
        self.declare_parameter("max_linear_velocity", [0.25, 0.25, 0.8])

        # Stronger "hold" gains used when wrench is inactive.
        # This makes the robot behave like a position hold at rest, while still using
        # the same admittance form (F term simply becomes zero).
        self.declare_parameter("hold_damping_linear", [320.0, 320.0, 180.0])
        self.declare_parameter("hold_stiffness_linear", [260.0, 260.0, 1200.0])

        # Virtual mass for second-order admittance: M dv/dt + B v = F - K dx (per axis).
        self.declare_parameter("mass_linear", [5.0, 5.0, 5.0])
        self.declare_parameter("mass_angular", [0.5, 0.5, 0.5])
        # If true, zero integrated admittance velocity when wrench becomes inactive (no coast-down).
        self.declare_parameter("reset_admittance_velocity_on_wrench_drop", False)

        self.declare_parameter("freeze_orientation", True)
        self.declare_parameter("damping_angular", [24.0, 24.0, 18.0])
        self.declare_parameter("stiffness_angular", [35.0, 35.0, 20.0])
        self.declare_parameter("torque_deadzone", [0.08, 0.08, 0.08])
        self.declare_parameter("max_angular_velocity", [0.35, 0.35, 0.45])

        self.declare_parameter("wrench_activate_force_norm", 0.5)
        self.declare_parameter("wrench_activate_torque_norm", 0.05)
        self.declare_parameter("hold_until_wrench_active", True)

        # QP weights and limits
        self.declare_parameter("W_ee", [1.0] * 6)
        self.declare_parameter("W_reg", [1e-2] * 10)
        self.declare_parameter("arm_max_joint_velocity_rad_s", 10.0)
        self.declare_parameter("base_max_linear_vel", 1.0)
        self.declare_parameter("base_max_angular_vel", 1.5)
        # Smooth published base cmd_vel to avoid caster hunting under small forces.
        self.declare_parameter("cmd_vel_lpf_alpha", 0.25)
        self.declare_parameter("cmd_vel_dir_hold_vplanar_min", 0.05)
        # Per-axis scale on base velocity output before publishing (world frame [vx, vy, wz]).
        # Compensates for anisotropic base tracking due to caster dynamics.
        self.declare_parameter("cmd_vel_world_scale", [1.0, 1.0, 1.0])

        # debug
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("debug_residual_topic", "qp/debug_residual")

        model_path = str(self.get_parameter("model_path").value)
        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        ee_body = str(self.get_parameter("end_effector_body").value)
        self._ee_body_id = int(mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body))
        if self._ee_body_id < 0:
            raise RuntimeError(f"end_effector_body not found: {ee_body}")

        self._arm_joints: List[str] = list(self.get_parameter("arm_joint_names").value)
        self._dof: DofIndex = make_reduced_dof_index(
            self._model,
            arm_joint_names=self._arm_joints,
            base_freejoint_name=str(self.get_parameter("base_freejoint_name").value),
        )

        hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(hz, 1.0)

        self._alpha = float(self.get_parameter("wrench_lpf_alpha").value)
        self._alpha = _clamp(self._alpha, 0.0, 1.0)

        self._b_lin = np.array(list(self.get_parameter("damping_linear").value), dtype=np.float64)
        self._k_lin = np.array(list(self.get_parameter("stiffness_linear").value), dtype=np.float64)
        self._b_lin_hold = np.array(list(self.get_parameter("hold_damping_linear").value), dtype=np.float64)
        self._k_lin_hold = np.array(list(self.get_parameter("hold_stiffness_linear").value), dtype=np.float64)
        self._dz_f = np.array(list(self.get_parameter("force_deadzone").value), dtype=np.float64)
        self._despring_thresh = np.array(list(self.get_parameter("force_despring_thresh").value), dtype=np.float64)
        self._vmax_lin = np.array(list(self.get_parameter("max_linear_velocity").value), dtype=np.float64)

        self._freeze_ori = bool(self.get_parameter("freeze_orientation").value)
        self._b_ang = np.array(list(self.get_parameter("damping_angular").value), dtype=np.float64)
        self._k_ang = np.array(list(self.get_parameter("stiffness_angular").value), dtype=np.float64)
        self._dz_t = np.array(list(self.get_parameter("torque_deadzone").value), dtype=np.float64)
        self._vmax_ang = np.array(list(self.get_parameter("max_angular_velocity").value), dtype=np.float64)

        self._m_lin = np.array(list(self.get_parameter("mass_linear").value), dtype=np.float64)
        self._m_ang = np.array(list(self.get_parameter("mass_angular").value), dtype=np.float64)
        self._reset_adm_vel_on_wrench_drop = bool(
            self.get_parameter("reset_admittance_velocity_on_wrench_drop").value
        )
        self._v_adm = np.zeros(6, dtype=np.float64)

        self._act_force_norm = float(self.get_parameter("wrench_activate_force_norm").value)
        self._act_torque_norm = float(self.get_parameter("wrench_activate_torque_norm").value)
        self._hold_until_wrench = bool(self.get_parameter("hold_until_wrench_active").value)

        self._W_ee = np.diag(np.array(list(self.get_parameter("W_ee").value), dtype=np.float64))
        wreg = np.array(list(self.get_parameter("W_reg").value), dtype=np.float64)
        if wreg.shape != (10,):
            raise RuntimeError("W_reg must have len=10")
        self._W_reg = np.diag(wreg)

        self._arm_vel_lim = float(self.get_parameter("arm_max_joint_velocity_rad_s").value)
        self._base_v_lim = float(self.get_parameter("base_max_linear_vel").value)
        self._base_w_lim = float(self.get_parameter("base_max_angular_vel").value)
        self._cmd_vel_alpha = float(self.get_parameter("cmd_vel_lpf_alpha").value)
        self._cmd_vel_alpha = _clamp(self._cmd_vel_alpha, 0.0, 1.0)
        self._cmd_vel_dir_hold_vmin = float(self.get_parameter("cmd_vel_dir_hold_vplanar_min").value)
        self._cmd_vel_dir_hold_vmin = max(0.0, self._cmd_vel_dir_hold_vmin)
        self._cmd_vel_world_scale = np.array(list(self.get_parameter("cmd_vel_world_scale").value), dtype=np.float64)
        if self._cmd_vel_world_scale.shape != (3,):
            raise RuntimeError("cmd_vel_world_scale must have len=3")

        # Filter state for published cmd_vel (base frame).
        self._cmd_vel_vx_f = 0.0
        self._cmd_vel_vy_f = 0.0

        # subscriptions
        self._latest_wrench: Optional[WrenchStamped] = None
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_odom: Optional[Odometry] = None
        self._latest_state: Dict[str, tuple[float, float]] = {}

        self.create_subscription(WrenchStamped, str(self.get_parameter("input_wrench_topic").value), self._cb_wrench, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("ee_pose_topic").value), self._cb_pose, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._cb_odom, 10)
        self.create_subscription(JointState, str(self.get_parameter("state_joint_topic").value), self._cb_state_joint, 10)

        # publishers
        self._pub_cmd_vel = self.create_publisher(Twist, str(self.get_parameter("output_cmd_vel_topic").value), 10)
        self._pub_joint = self.create_publisher(JointState, str(self.get_parameter("output_joint_command_topic").value), 10)
        self._pub_debug = self.create_publisher(JointState, str(self.get_parameter("debug_residual_topic").value), 10)
        self._publish_debug = bool(self.get_parameter("publish_debug").value)
        self._pub_cart_vel = self.create_publisher(
            TwistStamped, str(self.get_parameter("output_cartesian_velocity_topic").value), 10
        )
        self._pub_latched = self.create_publisher(
            Bool, str(self.get_parameter("base_pose_latched_topic").value), 10
        )

        self._latch = LatchState()
        self._wrench_filtered = np.zeros(6, dtype=np.float64)

        # OSQP instance (warm-start)
        self._osqp = osqp.OSQP()
        self._osqp_inited = False
        self._A = sp.eye(10, format="csc")
        # Use a fixed sparsity pattern for P: full upper-triangular (incl diagonal).
        self._P_pattern = sp.triu(sp.csc_matrix(np.ones((10, 10), dtype=np.float64))).tocsc()

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(f"qp whole-body admittance ready @ {hz:.1f}Hz")

    def _cb_wrench(self, msg: WrenchStamped) -> None:
        self._latest_wrench = msg

    def _cb_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _cb_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _cb_state_joint(self, msg: JointState) -> None:
        for n, q, dq in zip(msg.name, msg.position, msg.velocity):
            self._latest_state[str(n)] = (float(q), float(dq))

    def _sync_state_to_mujoco(self) -> bool:
        """Write state/joint + odom into MuJoCo qpos/qvel. Returns True if ok."""
        if self._latest_odom is None:
            return False

        # base free joint by name (requires robot_pr2.xml has named freejoint)
        jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
        if jid < 0:
            return False
        qadr = int(self._model.jnt_qposadr[jid])
        vadr = int(self._model.jnt_dofadr[jid])

        p = self._latest_odom.pose.pose.position
        q = self._latest_odom.pose.pose.orientation
        self._data.qpos[qadr + 0] = float(p.x)
        self._data.qpos[qadr + 1] = float(p.y)
        self._data.qpos[qadr + 2] = float(p.z)
        # MuJoCo free joint quaternion is wxyz
        self._data.qpos[qadr + 3] = float(q.w)
        self._data.qpos[qadr + 4] = float(q.x)
        self._data.qpos[qadr + 5] = float(q.y)
        self._data.qpos[qadr + 6] = float(q.z)

        tw = self._latest_odom.twist.twist
        self._data.qvel[vadr + 0] = float(tw.linear.x)
        self._data.qvel[vadr + 1] = float(tw.linear.y)
        self._data.qvel[vadr + 2] = float(tw.linear.z)
        self._data.qvel[vadr + 3] = float(tw.angular.x)
        self._data.qvel[vadr + 4] = float(tw.angular.y)
        self._data.qvel[vadr + 5] = float(tw.angular.z)

        # arm joints
        ok = True
        for jn, vadr_j in zip(self._arm_joints, list(self._dof.arm_vadr)):
            if jn not in self._latest_state:
                ok = False
                continue
            qj, dqj = self._latest_state[jn]
            jid_j = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid_j < 0:
                ok = False
                continue
            qadr_j = int(self._model.jnt_qposadr[jid_j])
            self._data.qpos[qadr_j] = float(qj)
            self._data.qvel[int(vadr_j)] = float(dqj)

        mujoco.mj_forward(self._model, self._data)
        return ok

    def _tick(self) -> None:
        if self._latest_pose is None or self._latest_wrench is None:
            return

        if not self._sync_state_to_mujoco():
            return

        # Latch a reference EE pose once, then always hold around it.
        wr_raw_msg = np.array(
            [
                float(self._latest_wrench.wrench.force.x),
                float(self._latest_wrench.wrench.force.y),
                float(self._latest_wrench.wrench.force.z),
                float(self._latest_wrench.wrench.torque.x),
                float(self._latest_wrench.wrench.torque.y),
                float(self._latest_wrench.wrench.torque.z),
            ],
            dtype=np.float64,
        )

        # Interpret wrench frame.
        # - ee_pose and Jacobians are in odom/world frame (as published by ee_pose_publisher).
        # - For user convenience we support providing wrench in base_link frame.
        #   If so, rotate it into odom/world using current base yaw from odom.
        wr_raw = wr_raw_msg.copy()
        wrench_frame = str(self._latest_wrench.header.frame_id)
        wrench_rotated = False
        if wrench_frame == "base_link":
            try:
                qb = self._latest_odom.pose.pose.orientation
                qw, qx, qy, qz = float(qb.w), float(qb.x), float(qb.y), float(qb.z)
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw_b = float(math.atan2(siny_cosp, cosy_cosp))
                cy, sy = float(math.cos(yaw_b)), float(math.sin(yaw_b))

                fx_b, fy_b = float(wr_raw[0]), float(wr_raw[1])
                tx_b, ty_b = float(wr_raw[3]), float(wr_raw[4])
                # base -> world: v_w = Rz(yaw) v_b
                wr_raw[0] = cy * fx_b - sy * fy_b
                wr_raw[1] = sy * fx_b + cy * fy_b
                wr_raw[3] = cy * tx_b - sy * ty_b
                wr_raw[4] = sy * tx_b + cy * ty_b
                wrench_rotated = True
            except Exception:
                wrench_rotated = False

        # Decide "active/inactive" based on the *raw* wrench message (after frame rotation).
        # Rationale: We still low-pass filter the wrench for smooth v_des, but we want the
        # inactive edge (force removed) to be immediate, to avoid post-release drift caused by
        # filter residual.
        f_raw = wr_raw[:3]
        t_raw = wr_raw[3:]
        wrench_is_active = (np.linalg.norm(f_raw) >= self._act_force_norm) or (np.linalg.norm(t_raw) >= self._act_torque_norm)

        # LPF update. When wrench is inactive, hard-reset filter to raw (typically zeros) to
        # eliminate residual force.
        if not bool(wrench_is_active):
            self._wrench_filtered = wr_raw.copy()
        else:
            self._wrench_filtered = self._alpha * wr_raw + (1.0 - self._alpha) * self._wrench_filtered

        f = self._wrench_filtered[:3]
        t = self._wrench_filtered[3:]

        # #region agent log
        prev_active = self._dbg_prev_wrench_active
        if (prev_active is None) or (bool(prev_active) != bool(wrench_is_active)):
            self._dbg_prev_wrench_active = bool(wrench_is_active)
            # Frame debugging: record pose/wrench frames and current base yaw (about +z).
            try:
                _q = self._latest_pose.pose.orientation
                qw, qx, qy, qz = float(_q.w), float(_q.x), float(_q.y), float(_q.z)
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                _yaw = float(math.atan2(siny_cosp, cosy_cosp))
            except Exception:
                _yaw = float("nan")
            # Also compute base yaw from odom (the yaw we actually use to rotate cmd_vel).
            try:
                _qb = self._latest_odom.pose.pose.orientation if self._latest_odom is not None else self._latest_pose.pose.orientation
                qw, qx, qy, qz = float(_qb.w), float(_qb.x), float(_qb.y), float(_qb.z)
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                _yaw_b_edge = float(math.atan2(siny_cosp, cosy_cosp))
            except Exception:
                _yaw_b_edge = float("nan")
            # For intuition: express filtered force xyz in base frame using odom yaw.
            try:
                cy, sy = float(math.cos(_yaw_b_edge)), float(math.sin(_yaw_b_edge))
                fx_w, fy_w, fz_w = float(self._wrench_filtered[0]), float(self._wrench_filtered[1]), float(self._wrench_filtered[2])
                fx_b = cy * fx_w + sy * fy_w
                fy_b = -sy * fx_w + cy * fy_w
                f_b = [fx_b, fy_b, fz_w]
            except Exception:
                f_b = [float("nan"), float("nan"), float("nan")]
            self._dbg_write(
                "H9_WrenchEdgeDrop",
                "wrench_active edge",
                {
                    "wrench_active": bool(wrench_is_active),
                    "wrench_frame_id": str(self._latest_wrench.header.frame_id),
                    "pose_frame_id": str(self._latest_pose.header.frame_id),
                    "wrench_rotated_base_to_odom": bool(wrench_rotated),
                    "w_raw_msg": [float(x) for x in wr_raw_msg],
                    "base_yaw_rad": _yaw,
                    "base_yaw_odom_rad": float(_yaw_b_edge),
                    "f_norm": float(np.linalg.norm(f)),
                    "tau_norm": float(np.linalg.norm(t)),
                    "w_raw": [float(x) for x in wr_raw],
                    "w_filt": [float(x) for x in self._wrench_filtered],
                    "f_filt_base_frame_xyz": [float(x) for x in f_b],
                    "act_force_norm": float(self._act_force_norm),
                    "act_torque_norm": float(self._act_torque_norm),
                },
            )
        # #endregion agent log

        # Always latch a reference pose the first time we can.
        # With stiffness K>0: hold uses v from M dv/dt + B v = -K dx when F=0.
        # With K=0 and mass-damper: v decays via M dv/dt + B v = 0 when F=0.
        prev_wrench_was_active = bool(self._latch.wrench_was_active)
        if (not self._latch.latched) or (self._latch.base_pose is None):
            self._latch.base_pose = self._latest_pose
            self._latch.latched = True
            self._dbg_write(
                "H1_LatchAndTopics",
                "latched base pose",
                {"ros_t": float(self.get_clock().now().nanoseconds * 1e-9), "reason": "initial_latch"},
            )
        elif prev_wrench_was_active and (not bool(wrench_is_active)):
            self._latch.base_pose = self._latest_pose
            if self._reset_adm_vel_on_wrench_drop:
                self._v_adm[:] = 0.0
            self._dbg_write(
                "H1_LatchAndTopics",
                "updated base pose on wrench drop",
                {"ros_t": float(self.get_clock().now().nanoseconds * 1e-9), "reason": "wrench_drop_latch"},
            )
        self._latch.wrench_was_active = bool(wrench_is_active)

        # publish latch state for logger/validator
        self._pub_latched.publish(Bool(data=bool(self._latch.latched)))

        if not self._latch.latched or self._latch.base_pose is None:
            return

        # dx (linear) from ee_pose feedback (same frame)
        cur = self._latest_pose.pose
        base = self._latch.base_pose.pose
        dx_lin = np.array(
            [
                float(cur.position.x) - float(base.position.x),
                float(cur.position.y) - float(base.position.y),
                float(cur.position.z) - float(base.position.z),
            ],
            dtype=np.float64,
        )

        # dx (angular): small-angle error (world frame), or zero if frozen
        if self._freeze_ori:
            dx_ang = np.zeros(3, dtype=np.float64)
        else:
            cur_q = np.array([float(cur.orientation.w), float(cur.orientation.x), float(cur.orientation.y), float(cur.orientation.z)], dtype=np.float64)
            base_q = np.array([float(base.orientation.w), float(base.orientation.x), float(base.orientation.y), float(base.orientation.z)], dtype=np.float64)
            dx_ang = _orientation_error_world(cur_q, base_q)

        # Apply deadzone per axis
        w = self._wrench_filtered.copy()
        for i in range(3):
            w[i] = _apply_deadzone(float(w[i]), float(self._dz_f[i]))
            w[3 + i] = _apply_deadzone(float(w[3 + i]), float(self._dz_t[i]))

        # Admittance (mass-damper): M dv/dt + B v = F - K dx  =>  v += dt * (F - K dx - B v) / M
        dt = float(self._dt)
        v_des = np.zeros(6, dtype=np.float64)
        v_des_raw = np.zeros(6, dtype=np.float64)
        v_sat_lin = [False, False, False]
        # Gain selection:
        # We want "hold" behavior to remain the baseline even when an external wrench is applied.
        # Otherwise switching to weaker active gains can cause a sudden drop (notably in Z against gravity).
        #
        # Strategy: per-axis, if that axis force is effectively zero (after deadzone), keep hold gains.
        # If that axis is being actively driven by force, use the active gains.
        b_lin = self._b_lin.copy()
        k_lin = self._k_lin.copy()
        for i in range(3):
            if abs(float(w[i])) <= 1e-9:
                b_lin[i] = self._b_lin_hold[i]
                k_lin[i] = self._k_lin_hold[i]
            elif abs(float(self._wrench_filtered[i])) >= float(self._despring_thresh[i]):
                k_lin[i] = 0.0
        for i in range(3):
            m = max(float(self._m_lin[i]), 1e-6)
            b = max(float(b_lin[i]), 1e-6)
            f_net = float(w[i]) - float(k_lin[i]) * float(dx_lin[i])
            self._v_adm[i] += dt * (f_net - b * self._v_adm[i]) / m
            v_des_raw[i] = self._v_adm[i]
            v_des[i] = _clamp(float(v_des_raw[i]), -float(self._vmax_lin[i]), float(self._vmax_lin[i]))
            self._v_adm[i] = float(v_des[i])
            v_sat_lin[i] = bool(abs(float(v_des_raw[i])) > float(self._vmax_lin[i]) + 1e-12)
        if self._freeze_ori:
            self._v_adm[3:6] = 0.0
        else:
            v_sat_ang = [False, False, False]
            for i in range(3):
                m = max(float(self._m_ang[i]), 1e-6)
                b = max(float(self._b_ang[i]), 1e-6)
                f_net = float(w[3 + i]) - float(self._k_ang[i]) * float(dx_ang[i])
                self._v_adm[3 + i] += dt * (f_net - b * self._v_adm[3 + i]) / m
                v_des_raw[3 + i] = self._v_adm[3 + i]
                v_des[3 + i] = _clamp(float(v_des_raw[3 + i]), -float(self._vmax_ang[i]), float(self._vmax_ang[i]))
                self._v_adm[3 + i] = float(v_des[3 + i])
                v_sat_ang[i] = bool(abs(float(v_des_raw[3 + i])) > float(self._vmax_ang[i]) + 1e-12)

        # #region agent log
        now_m = time.monotonic()
        if (any(v_sat_lin) or (not self._freeze_ori and any(v_sat_ang))) and (now_m - self._dbg_last_mono > 0.5):
            # share throttle with existing logs
            self._dbg_last_mono = now_m
            self._dbg_write(
                "H7_VDesSaturation",
                "v_des clamped by vmax",
                {
                    "wrench_active": bool(wrench_is_active),
                    "dx_lin": [float(x) for x in dx_lin],
                    "w_filt": [float(x) for x in self._wrench_filtered],
                    "v_des_raw": [float(x) for x in v_des_raw],
                    "v_des": [float(x) for x in v_des],
                    "vmax_lin": [float(x) for x in self._vmax_lin],
                    "sat_lin": v_sat_lin,
                },
            )
        # #endregion agent log

        # Build reduced Jacobian J (6x10)
        J = get_ee_jacobian_6x10(self._model, self._data, self._ee_body_id, self._dof)

        # QP matrices
        P = J.T @ self._W_ee @ J + self._W_reg
        q = -(J.T @ self._W_ee @ v_des)

        # Box constraints
        umin = np.array(
            [-self._arm_vel_lim] * 7 + [-self._base_v_lim, -self._base_v_lim, -self._base_w_lim],
            dtype=np.float64,
        )
        umax = np.array(
            [+self._arm_vel_lim] * 7 + [+self._base_v_lim, +self._base_v_lim, +self._base_w_lim],
            dtype=np.float64,
        )

        # When wrench is inactive, freeze base to avoid wheel spinning.
        # This keeps "hold position" purely on the arm, matching user expectation.
        freeze_base_no_wrench = True
        if freeze_base_no_wrench and (not wrench_is_active):
            umin[7:] = 0.0
            umax[7:] = 0.0

        # #region agent log
        # Root-cause logs for "torque-only causes translation drift / jump".
        # Goal: determine whether drift is from (a) v_des linear generated by hold gains,
        # (b) base motion participation, or (c) latch update edges.
        try:
            import numpy as _np
            now_m = time.monotonic()
            f_raw_n = float(_np.linalg.norm(f_raw))
            t_raw_n = float(_np.linalg.norm(t_raw))
            # Log only when torque is present, and throttle.
            if (t_raw_n >= float(self._act_torque_norm)) and (now_m - self._dbg_last_mono > 0.20):
                self._dbg_last_mono = now_m
                # Solve preview quantities already available here.
                self._dbg_write(
                    "H_TorqueOnlyDrift",
                    "torque-only debug snapshot",
                    {
                        "wrench_active": bool(wrench_is_active),
                        "f_raw_norm": f_raw_n,
                        "t_raw_norm": t_raw_n,
                        "w_raw": [float(x) for x in wr_raw],
                        "w_filt": [float(x) for x in self._wrench_filtered],
                        "dx_lin": [float(x) for x in dx_lin],
                        "dx_ang": [float(x) for x in dx_ang],
                        "k_lin": [float(x) for x in k_lin],
                        "b_lin": [float(x) for x in b_lin],
                        "v_des": [float(x) for x in v_des],
                        "freeze_base_no_wrench": bool(freeze_base_no_wrench),
                        "base_box_is_zero": bool(abs(float(umin[7])) < 1e-12 and abs(float(umax[7])) < 1e-12),
                        "umin_base": [float(x) for x in umin[7:]],
                        "umax_base": [float(x) for x in umax[7:]],
                    },
                )
        except Exception:
            pass
        # #endregion agent log

        # #region agent log
        # Snapshot right after wrench activates to explain "sudden drop" (typically v_des.z jump or saturation).
        if bool(wrench_is_active) and (prev_active is False):
            self._dbg_write(
                "H9_WrenchEdgeDrop",
                "post-activation snapshot",
                {
                    "dx_lin": [float(x) for x in dx_lin],
                    "dx_ang": [float(x) for x in dx_ang],
                    "b_lin": [float(x) for x in b_lin],
                    "k_lin": [float(x) for x in k_lin],
                    "v_des_raw": [float(x) for x in v_des_raw],
                    "v_des": [float(x) for x in v_des],
                    "v_sat_lin": [bool(x) for x in v_sat_lin],
                    "vmax_lin": [float(x) for x in self._vmax_lin],
                    "umin_base": [float(x) for x in umin[7:]],
                    "umax_base": [float(x) for x in umax[7:]],
                },
            )
        # #endregion agent log

        # #region agent log
        now_m = time.monotonic()
        if (now_m - self._dbg_last_mono > 0.5) and (not wrench_is_active):
            self._dbg_last_mono = now_m
            self._dbg_write(
                "H8_NoWrenchHoldBehavior",
                "no-wrench hold solve inputs",
                {
                    "dx_lin": [float(x) for x in dx_lin],
                    "dx_lin_norm": float(np.linalg.norm(dx_lin)),
                    "w_filt": [float(x) for x in self._wrench_filtered],
                    "v_des_raw": [float(x) for x in v_des_raw],
                    "v_des": [float(x) for x in v_des],
                    "umin_base": [float(x) for x in umin[7:]],
                    "umax_base": [float(x) for x in umax[7:]],
                    "umin_arm": [float(x) for x in umin[:7]],
                    "umax_arm": [float(x) for x in umax[:7]],
                },
            )
        # #endregion agent log

        # OSQP setup/update (fixed P sparsity pattern to avoid wrong updates)
        P_sym = (P + P.T) * 0.5

        # Fill Px in CSC data order for the fixed upper-tri pattern.
        Px = np.empty_like(self._P_pattern.data, dtype=np.float64)
        k = 0
        indptr = self._P_pattern.indptr
        indices = self._P_pattern.indices
        for col in range(10):
            for pidx in range(indptr[col], indptr[col + 1]):
                row = indices[pidx]
                Px[k] = float(P_sym[row, col])
                k += 1

        if not self._osqp_inited:
            P_init = sp.csc_matrix((Px.copy(), indices.copy(), indptr.copy()), shape=(10, 10))
            self._osqp.setup(P=P_init, q=q, A=self._A, l=umin, u=umax, verbose=False, warm_start=True)
            self._osqp_inited = True
        else:
            self._osqp.update(Px=Px, q=q, l=umin, u=umax)

        res = self._osqp.solve()
        if res.x is None or res.info.status_val not in (1, 2):  # solved / solved inaccurate
            self._dbg_write(
                "H2_QPSolve",
                "qp failed",
                {"status": str(res.info.status), "status_val": int(res.info.status_val)},
            )
            return
        u = np.array(res.x, dtype=np.float64)

        # Apply per-axis scale compensation on base velocity (world frame).
        u_scaled = u.copy()
        u_scaled[7] *= float(self._cmd_vel_world_scale[0])
        u_scaled[8] *= float(self._cmd_vel_world_scale[1])
        u_scaled[9] *= float(self._cmd_vel_world_scale[2])

        # Publish outputs
        # NOTE: MuJoCo free-joint velocities (used by Jacobians) are in world/odom frame,
        # but the sim consumes /cmd_vel as base-frame planar velocity (see pr2_sim_ros.py).
        # Rotate (vx, vy) from odom/world into base frame before publishing so that
        # the realized world motion matches the QP solution.
        try:
            _q_ee = self._latest_pose.pose.orientation
            _q_b = self._latest_odom.pose.pose.orientation if self._latest_odom is not None else _q_ee

            def _yaw_from_q(_q) -> float:
                qw, qx, qy, qz = float(_q.w), float(_q.x), float(_q.y), float(_q.z)
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                return float(math.atan2(siny_cosp, cosy_cosp))

            _yaw_ee = _yaw_from_q(_q_ee)
            _yaw_b = _yaw_from_q(_q_b)
            # Use base yaw from odometry for world->base rotation. Using EE yaw will
            # rotate cmd_vel incorrectly when the wrist/EE orientation differs from base.
            _yaw = _yaw_b
            cy, sy = float(math.cos(_yaw)), float(math.sin(_yaw))
            vx_w, vy_w = float(u_scaled[7]), float(u_scaled[8])
            # world/odom -> base: v_b = Rz(-yaw) v_w
            vx_cmd = cy * vx_w + sy * vy_w
            vy_cmd = -sy * vx_w + cy * vy_w
        except Exception:
            _yaw = float("nan")
            _yaw_ee = float("nan")
            _yaw_b = float("nan")
            vx_w = vy_w = float("nan")
            vx_cmd = float(u_scaled[7])
            vy_cmd = float(u_scaled[8])
        tw = Twist()
        tw.linear.x = float(vx_cmd)
        tw.linear.y = float(vy_cmd)
        tw.angular.z = float(u_scaled[9])

        # Smooth cmd_vel direction/magnitude (base frame) to avoid caster hunting when
        # QP direction oscillates under small forces.
        vx_raw = float(tw.linear.x)
        vy_raw = float(tw.linear.y)
        vplan_raw = float(math.hypot(vx_raw, vy_raw))
        dir_raw = float(math.atan2(vy_raw, vx_raw)) if vplan_raw > 1e-9 else None
        vx_out, vy_out = vx_raw, vy_raw
        if bool(wrench_is_active) and (vplan_raw > 1e-9):
            # Component-domain LPF: avoids "direction memory" and prevents sudden sign flips in vx/vy.
            a = float(self._cmd_vel_alpha)
            vx_f = (1.0 - a) * float(self._cmd_vel_vx_f) + a * vx_raw
            vy_f = (1.0 - a) * float(self._cmd_vel_vy_f) + a * vy_raw
            # Under very small commanded planar speed, hold the filtered direction but shrink magnitude.
            if vplan_raw < float(self._cmd_vel_dir_hold_vmin):
                vx_f *= 0.5
                vy_f *= 0.5
            vx_out = float(vx_f)
            vy_out = float(vy_f)
            self._cmd_vel_vx_f = float(vx_f)
            self._cmd_vel_vy_f = float(vy_f)

        tw.linear.x = float(vx_out)
        tw.linear.y = float(vy_out)
        self._pub_cmd_vel.publish(tw)

        # #region agent log
        # Detect "base stops while arm still moves" during constant wrench.
        try:
            now_m = time.monotonic()
            if bool(wrench_is_active) and (now_m - float(self._dbg_last_mono) > 0.2):
                self._dbg_last_mono = float(now_m)
                base_plan = float(math.hypot(vx_out, vy_out))
                arm_peak = float(np.max(np.abs(u[:7]))) if u is not None else float("nan")
                try:
                    v_ach_local = (J @ u).astype(np.float64)
                    v_ach_list = [float(x) for x in v_ach_local.tolist()]
                except Exception:
                    v_ach_list = None
                self._dbg_write(
                    "H12_BaseStopWhileArmMoves",
                    "base/arm activity under wrench",
                    {
                        "sim_time": float(self._data.time),
                        "base_latched": bool(self._latch.latched),
                        "wrench_frame_id": str(self._latest_wrench.header.frame_id) if self._latest_wrench is not None else None,
                        "w_raw_msg": [
                            float(self._latest_wrench.wrench.force.x),
                            float(self._latest_wrench.wrench.force.y),
                            float(self._latest_wrench.wrench.force.z),
                        ] if self._latest_wrench is not None else None,
                        "cmd_vel_pub_base": [float(vx_out), float(vy_out), float(u[9])],
                        "cmd_planar": float(base_plan),
                        "u_base_world": [float(u[7]), float(u[8]), float(u[9])],
                        "arm_peak_abs_qdot": float(arm_peak),
                        "v_des": [float(x) for x in v_des.tolist()],
                        "v_ach": v_ach_list,
                        "obj": float(res.info.obj_val) if hasattr(res, "info") else None,
                    },
                )
        except Exception:
            pass
        # #endregion agent log

        # #region agent log
        # When base "hunts", first determine whether cmd_vel direction itself oscillates.
        try:
            now_m = time.monotonic()
            if bool(wrench_is_active) and (now_m - float(self._dbg_last_cmdvel_mono) > 0.1):
                self._dbg_last_cmdvel_mono = float(now_m)
                vplan = float(math.hypot(vx_cmd, vy_cmd))
                ang = float(math.atan2(vy_cmd, vx_cmd)) if vplan > 1e-9 else float("nan")
                self._dbg_write(
                    "H11_QPCmdVelDir",
                    "qp cmd_vel published (base frame)",
                    {
                        "sim_time": float(self._data.time),
                        "wrench_active": bool(wrench_is_active),
                        "cmd_vel_base_raw": [float(vx_raw), float(vy_raw), float(u[9])],
                        "cmd_vel_base_pub": [float(vx_out), float(vy_out), float(u[9])],
                        "vplanar_raw": float(vplan_raw),
                        "dir_atan2_raw": float(dir_raw) if dir_raw is not None else None,
                        "vplanar_pub": float(math.hypot(vx_out, vy_out)),
                        "dir_atan2_pub": float(math.atan2(vy_out, vx_out)) if math.hypot(vx_out, vy_out) > 1e-9 else None,
                        "alpha": float(self._cmd_vel_alpha),
                        "dir_hold_vmin": float(self._cmd_vel_dir_hold_vmin),
                        "u_base_world": [float(u[7]), float(u[8]), float(u[9])],
                    },
                )
        except Exception:
            pass
        # #endregion agent log

        js = JointState()
        js.name = list(self._arm_joints)
        js.velocity = [float(x) for x in u[:7]]
        self._pub_joint.publish(js)

        # Publish cartesian velocity for logger (use achieved twist J*u)
        v_ach = J @ u
        tws = TwistStamped()
        tws.header.stamp = self.get_clock().now().to_msg()
        tws.header.frame_id = str(self._latest_pose.header.frame_id)
        tws.twist.linear.x = float(v_ach[0])
        tws.twist.linear.y = float(v_ach[1])
        tws.twist.linear.z = float(v_ach[2])
        tws.twist.angular.x = float(v_ach[3])
        tws.twist.angular.y = float(v_ach[4])
        tws.twist.angular.z = float(v_ach[5])
        self._pub_cart_vel.publish(tws)

        if self._publish_debug:
            dbg = JointState()
            dbg.name = ["qp_obj"]
            dbg.effort = [float(res.info.obj_val)]
            self._pub_debug.publish(dbg)

        # #region agent log
        # Decompose achieved EE twist into arm vs base contributions to explain
        # "偏移但不弯折" (often base dominates in y).
        try:
            v_arm = J[:, :7] @ u[:7]
            v_base = J[:, 7:] @ u[7:]
            now_m = time.monotonic()
            if now_m - self._dbg_last_mono > 0.5:
                # NOTE: _dbg_last_mono is also used above; shared throttling is OK for coarse logs.
                # Get actual joint velocities from latest state for comparison with QP commands
                try:
                    actual_qdot = np.array(
                        [float(self._latest_state[jn][1]) for jn in self._arm_joints],
                        dtype=np.float64,
                    )
                    v_arm_actual = J[:, :7] @ actual_qdot
                    arm_track = [float(actual_qdot[i]) / float(u[i]) if abs(float(u[i])) > 0.005 else float("nan")
                                 for i in range(7)]
                except Exception:
                    actual_qdot = None
                    v_arm_actual = None
                    arm_track = None
                self._dbg_write(
                    "H_WBCTracking",
                    "arm velocity cmd vs actual",
                    {
                        "wrench_active": bool(wrench_is_active),
                        "v_des_xyz": [float(v_des[i]) for i in range(3)],
                        "v_ach_xyz": [float(v_ach[i]) for i in range(3)],
                        "v_arm_xyz": [float(v_arm[i]) for i in range(3)],
                        "v_base_xyz": [float(v_base[i]) for i in range(3)],
                        "u_arm": [float(u[i]) for i in range(7)],
                        "u_base_world": [float(u[7]), float(u[8]), float(u[9])],
                        "qdot_actual": [float(x) for x in actual_qdot] if actual_qdot is not None else None,
                        "v_arm_actual_xyz": [float(v_arm_actual[i]) for i in range(3)] if v_arm_actual is not None else None,
                        "arm_track_ratio": arm_track,
                    },
                )
        except Exception:
            pass
        # #endregion agent log

        # One-shot spike capture for debugging: huge u or huge achieved EE speed.
        if float(np.max(np.abs(u))) > 5.0 or float(np.linalg.norm(v_ach[:3])) > 2.0:
            self._dbg_write(
                "H4_SpikeCapture",
                "spike detected",
                {
                    "w_raw": [float(x) for x in wr_raw],
                    "w_filt": [float(x) for x in self._wrench_filtered],
                    "wrench_active": bool(wrench_is_active),
                    "v_des": [float(x) for x in v_des],
                    "v_ach": [float(x) for x in v_ach],
                    "u": [float(x) for x in u],
                    "J_col_norms": [float(np.linalg.norm(J[:, i])) for i in range(10)],
                    "obj": float(res.info.obj_val),
                    "status": str(res.info.status),
                },
            )

        now_m = time.monotonic()
        if now_m - self._dbg_last_mono > 0.5:
            self._dbg_last_mono = now_m
            # Frame check: QP's base velocities are in the same frame as J and ee_pose (typically odom/world),
            # while /cmd_vel is consumed as base-frame planar velocity in the sim mapping.
            # Log both "as published" and "rotated into base frame" using current yaw.
            try:
                _q = self._latest_pose.pose.orientation
                qw, qx, qy, qz = float(_q.w), float(_q.x), float(_q.y), float(_q.z)
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                _yaw = float(math.atan2(siny_cosp, cosy_cosp))
                cy, sy = float(math.cos(_yaw)), float(math.sin(_yaw))
                vx_w, vy_w = float(u[7]), float(u[8])
                # world/odom -> base: v_b = Rz(-yaw) v_w
                vx_b = cy * vx_w + sy * vy_w
                vy_b = -sy * vx_w + cy * vy_w
            except Exception:
                _yaw = float("nan")
                vx_w = vy_w = vx_b = vy_b = float("nan")
            self._dbg_write(
                "H3_PublishAndSignals",
                "tick summary",
                {
                    "wrench_active": bool(wrench_is_active),
                    "latched": bool(self._latch.latched),
                    "pose_frame_id": str(self._latest_pose.header.frame_id),
                    "base_yaw_rad": _yaw,
                    "yaw_ee_rad": _yaw_ee,
                    "yaw_base_odom_rad": _yaw_b,
                    "yaw_used": "odom",
                    "cmd_vel_world_solution": [float(u[7]), float(u[8]), float(u[9])],
                    "cmd_vel_published": [float(vx_cmd), float(vy_cmd), float(u[9])],
                    "cmd_vel_rotated_base_xy": [vx_b, vy_b],
                    "v_des_norm": float(np.linalg.norm(v_des)),
                    "u_arm_peak": float(np.max(np.abs(u[:7]))),
                    "u_base": [float(u[7]), float(u[8]), float(u[9])],
                    "obj": float(res.info.obj_val),
                },
            )


def main() -> None:
    rclpy.init()
    node = Pr2QpWholeBodyAdmittance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
