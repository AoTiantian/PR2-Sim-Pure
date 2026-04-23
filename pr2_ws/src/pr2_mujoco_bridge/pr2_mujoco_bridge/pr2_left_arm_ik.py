#!/usr/bin/env python3
"""
PR2 左臂数值拟运动学（IK）节点。

输入：
  - /ik_target_pose (geometry_msgs/PoseStamped): 末端目标位姿（默认世界坐标系）
  - /joint_states   (sensor_msgs/JointState): 当前关节状态
输出：
  - /joint_commands (sensor_msgs/JointState): 仅发布 effort，用于驱动 *_tau 力矩电机

说明：
  - 本节点用 MuJoCo 模型计算雅可比（阻尼最小二乘 DLS）。
  - 先求关节目标增量 dq，再用 PD 产生力矩命令。
  - 建议与 pr2_mujoco_sim 同时运行时设置 demo_motion:=false，避免控制冲突。
"""

from __future__ import annotations

import math
from typing import Dict, List

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _quat_xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=np.float64)


def _quat_wxyz_to_rotmat(q_wxyz: np.ndarray) -> np.ndarray:
    mat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(mat, q_wxyz)
    return mat.reshape(3, 3)


def _orientation_error_world(cur_q_wxyz: np.ndarray, tgt_q_wxyz: np.ndarray) -> np.ndarray:
    """3D 姿态误差向量（世界系），用于小角度迭代 IK。"""
    rc = _quat_wxyz_to_rotmat(cur_q_wxyz)
    rt = _quat_wxyz_to_rotmat(tgt_q_wxyz)
    return 0.5 * (
        np.cross(rc[:, 0], rt[:, 0])
        + np.cross(rc[:, 1], rt[:, 1])
        + np.cross(rc[:, 2], rt[:, 2])
    )


class Pr2LeftArmIk(Node):
    def __init__(self) -> None:
        super().__init__("pr2_left_arm_ik")

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        )
        self.declare_parameter("end_effector_body", "l_gripper_tool_frame")
        self.declare_parameter("target_topic", "ik_target_pose")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("use_gravity_compensation", True)
        self.declare_parameter("control_rate_hz", 100.0)

        self.declare_parameter("use_orientation", True)
        self.declare_parameter("position_weight", 1.0)
        self.declare_parameter("orientation_weight", 0.35)
        self.declare_parameter("damping_lambda", 0.08)
        self.declare_parameter("step_size", 0.35)
        self.declare_parameter("max_step_rad", 0.06)

        self.declare_parameter("torque_kp", 150.0)
        self.declare_parameter("torque_kd", 20.0)
        self.declare_parameter("max_torque", 60.0)
        self.declare_parameter("require_odom_sync", True)
        self.declare_parameter("state_timeout_sec", 0.25)
        self.declare_parameter("tau_rate_limit", 400.0)  # Nm/s
        self.declare_parameter("tau_lpf_alpha", 0.35)
        # Initial joint pose (joint_name:angle_rad pairs as JSON string) applied from t=0
        # until the first external target arrives.  Prevents arm from free-falling before latch.
        # Example: '{"l_shoulder_pan_joint":0.4,"l_shoulder_lift_joint":-0.2,"l_elbow_flex_joint":-0.8}'
        self.declare_parameter("initial_joint_pose_json", "")
        # velocity_tracking mode parameters
        self.declare_parameter("control_mode", "pose_tracking")
        # "pose_tracking"    = existing behavior (PoseStamped → DLS IK → PD → torque)
        # "velocity_tracking" = new: TwistStamped → J⁺·ẋ_cmd → integrate → PD → torque
        self.declare_parameter("cartesian_velocity_topic", "arm_cartesian_velocity")
        self.declare_parameter("velocity_cmd_timeout_sec", 0.15)
        self.declare_parameter("velocity_integration_gain", 1.0)
        self.declare_parameter("velocity_sync_alpha", 0.3)
        # Uniform scale on q̇_cmd so no joint exceeds this |rad/s| (velocity_tracking only).
        self.declare_parameter("max_joint_velocity_rad_s", 12.0)
        # Per-joint Kp/Kd scale factors as JSON {joint_name: scale}.
        # Wrist/forearm roll joints have small inertia; lower scale prevents oscillation.
        # Default: forearm_roll=0.25, wrist_flex=0.4, wrist_roll=0.25
        self.declare_parameter(
            "joint_kp_scale_json",
            '{"l_forearm_roll_joint":0.25,"l_wrist_flex_joint":0.4,"l_wrist_roll_joint":0.25}',
        )
        # Soft joint limit margin (rad): q_next is clamped to initial_joint_pose ± this value.
        # Prevents shoulder/elbow from drifting to degenerate configurations during Cartesian IK.
        # Roll joints (continuous, no gravity coupling) are excluded automatically.
        # 0 = disabled.  Default 0.35 rad catches the observed shoulder_lift drift.
        self.declare_parameter("joint_soft_limit_margin", 0.35)

        self.declare_parameter(
            "controlled_joints",
            [
                "l_shoulder_pan_joint",
                "l_shoulder_lift_joint",
                "l_upper_arm_roll_joint",
                "l_elbow_flex_joint",
                "l_forearm_roll_joint",
                "l_wrist_flex_joint",
                "l_wrist_roll_joint",
            ],
        )

        model_path = self.get_parameter("model_path").value
        ee_body_name = self.get_parameter("end_effector_body").value
        self._target_topic = self.get_parameter("target_topic").value
        self._joint_state_topic = self.get_parameter("joint_state_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._joint_command_topic = self.get_parameter("joint_command_topic").value
        self._use_gravity_compensation = bool(self.get_parameter("use_gravity_compensation").value)
        hz = float(self.get_parameter("control_rate_hz").value)
        self._ctrl_hz = max(hz, 1e-3)

        self._use_orientation = bool(self.get_parameter("use_orientation").value)
        self._w_pos = float(self.get_parameter("position_weight").value)
        self._w_ori = float(self.get_parameter("orientation_weight").value)
        self._lam = float(self.get_parameter("damping_lambda").value)
        self._alpha = float(self.get_parameter("step_size").value)
        self._max_step = float(self.get_parameter("max_step_rad").value)

        self._kp = float(self.get_parameter("torque_kp").value)
        self._kd = float(self.get_parameter("torque_kd").value)
        self._max_torque = float(self.get_parameter("max_torque").value)
        self._require_odom_sync = bool(self.get_parameter("require_odom_sync").value)
        self._state_timeout = float(self.get_parameter("state_timeout_sec").value)
        self._tau_rate_limit = float(self.get_parameter("tau_rate_limit").value)
        self._tau_lpf_alpha = max(
            0.0, min(1.0, float(self.get_parameter("tau_lpf_alpha").value))
        )

        self._controlled_joints: List[str] = list(
            self.get_parameter("controlled_joints").value
        )

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name
        )
        self._base_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        self._base_free_qadr = None
        self._base_free_vadr = None
        if self._base_body_id >= 0:
            for jid in range(int(self._model.njnt)):
                jtype = int(self._model.jnt_type[jid])
                jbody = int(self._model.jnt_bodyid[jid])
                if (
                    jtype == int(mujoco.mjtJoint.mjJNT_FREE)
                    and jbody == self._base_body_id
                ):
                    self._base_free_qadr = int(self._model.jnt_qposadr[jid])
                    self._base_free_vadr = int(self._model.jnt_dofadr[jid])
                    break

        if self._ee_body_id < 0:
            raise RuntimeError(f"end_effector_body 不存在: {ee_body_name}")

        # joint 映射（仅支持 1DoF 铰链/滑动关节）
        self._joint_ids: List[int] = []
        self._qadr: List[int] = []
        self._vadr: List[int] = []
        self._qmin: List[float] = []
        self._qmax: List[float] = []
        self._joint_to_tau_limit: Dict[str, float] = {}
        self._model_joint_addr: Dict[str, tuple[int, int]] = {}

        # Build model joint address map for all 1DoF joints, so FK uses full robot state.
        for jid in range(int(self._model.njnt)):
            jtype = int(self._model.jnt_type[jid])
            if jtype not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                continue
            jn = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if not jn:
                continue
            self._model_joint_addr[str(jn)] = (
                int(self._model.jnt_qposadr[jid]),
                int(self._model.jnt_dofadr[jid]),
            )

        for jn in self._controlled_joints:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"受控关节不存在: {jn}")
            jtype = int(self._model.jnt_type[jid])
            if jtype not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise RuntimeError(f"关节 {jn} 不是 1DoF，当前实现不支持")
            self._joint_ids.append(jid)
            self._qadr.append(int(self._model.jnt_qposadr[jid]))
            self._vadr.append(int(self._model.jnt_dofadr[jid]))
            self._qmin.append(float(self._model.jnt_range[jid][0]))
            self._qmax.append(float(self._model.jnt_range[jid][1]))

        # 由执行器配置读取每个关节的力矩上限（优先 *_tau 执行器）
        for a in range(int(self._model.nu)):
            an = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
            if not an.endswith("_tau"):
                continue
            jid = int(self._model.actuator_trnid[a, 0])
            jn = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if not jn:
                continue
            lim = float(max(abs(self._model.actuator_ctrlrange[a][0]), abs(self._model.actuator_ctrlrange[a][1])))
            self._joint_to_tau_limit[jn] = lim

        self._joint_state_pos: Dict[str, float] = {}
        self._joint_state_vel: Dict[str, float] = {}
        self._t_joint_state = self.get_clock().now()
        self._odom_pose = None
        self._odom_twist = None
        self._t_odom = self.get_clock().now()
        self._target_pos = None
        self._target_quat_wxyz = None
        self._external_target_received = False

        # Parse initial joint pose JSON
        import json as _json
        _raw = str(self.get_parameter("initial_joint_pose_json").value).strip()
        self._initial_joint_pose: Dict[str, float] = {}
        if _raw:
            try:
                self._initial_joint_pose = {k: float(v) for k, v in _json.loads(_raw).items()}
                self.get_logger().info(f"initial_joint_pose_json 已加载: {self._initial_joint_pose}")
            except Exception as e:
                self.get_logger().warn(f"initial_joint_pose_json 解析失败: {e}")
        self._joint_soft_limit_margin = float(self.get_parameter("joint_soft_limit_margin").value)

        import json as _json2
        _raw_scale = str(self.get_parameter("joint_kp_scale_json").value).strip()
        self._joint_kp_scale: Dict[str, float] = {}
        try:
            self._joint_kp_scale = {k: float(v) for k, v in _json2.loads(_raw_scale).items()}
        except Exception:
            pass

        self._tau_prev = np.zeros(len(self._controlled_joints), dtype=np.float64)
        self._tau_initialized = False

        # velocity_tracking mode state
        self._control_mode = str(self.get_parameter("control_mode").value)
        self._vel_cmd_timeout = float(self.get_parameter("velocity_cmd_timeout_sec").value)
        self._vel_integration_gain = float(self.get_parameter("velocity_integration_gain").value)
        self._vel_sync_alpha = float(np.clip(self.get_parameter("velocity_sync_alpha").value, 0.0, 1.0))
        self._max_joint_vel_rad_s = float(self.get_parameter("max_joint_velocity_rad_s").value)
        self._last_qdot_scale_warn_ns = 0
        self._cart_vel_cmd: np.ndarray | None = None
        self._t_cart_vel = self.get_clock().now()
        self._q_integrated: np.ndarray | None = None

        self._pub_cmd = self.create_publisher(JointState, self._joint_command_topic, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(PoseStamped, self._target_topic, self._on_target_pose, 10)
        if self._control_mode == "velocity_tracking":
            from geometry_msgs.msg import TwistStamped as _TwistStamped
            self.create_subscription(
                _TwistStamped,
                str(self.get_parameter("cartesian_velocity_topic").value),
                self._on_cartesian_velocity,
                10,
            )
        self.create_timer(1.0 / self._ctrl_hz, self._on_timer)

        self.get_logger().info(
            "IK 节点启动: "
            f"mode={self._control_mode}, "
            f"target={self._target_topic}, joints={self._controlled_joints}, "
            f"ee_body={ee_body_name}, gravity_comp={self._use_gravity_compensation}"
            + (
                f", max_joint_vel={self._max_joint_vel_rad_s:.2f} rad/s"
                if self._control_mode == "velocity_tracking"
                else ""
            )
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_state_pos[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self._joint_state_vel[name] = float(msg.velocity[i])
        self._t_joint_state = self.get_clock().now()

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._external_target_received = True
        self._target_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        q_xyzw = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        n = np.linalg.norm(q_xyzw)
        if n < 1e-9:
            q_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            q_xyzw /= n
        self._target_quat_wxyz = _quat_xyzw_to_wxyz(q_xyzw)

    def _on_cartesian_velocity(self, msg) -> None:
        self._cart_vel_cmd = np.array([
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
            float(msg.twist.angular.x),
            float(msg.twist.angular.y),
            float(msg.twist.angular.z),
        ], dtype=np.float64)
        self._t_cart_vel = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_pose = msg.pose.pose
        self._odom_twist = msg.twist.twist
        self._t_odom = self.get_clock().now()

    def _is_fresh(self, t_msg) -> bool:
        age = (self.get_clock().now() - t_msg).nanoseconds * 1e-9
        return age <= self._state_timeout

    def _publish_effort(self, tau: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._controlled_joints)
        msg.effort = [float(x) for x in tau]
        self._pub_cmd.publish(msg)

    def _build_model_state_from_joint_states(self) -> bool:
        # 用接收到的 joint_states 回填 MuJoCo qpos/qvel（全关节），确保 FK 与仿真一致。
        missing = [jn for jn in self._controlled_joints if jn not in self._joint_state_pos]
        if missing:
            return False

        for jn, q in self._joint_state_pos.items():
            addr = self._model_joint_addr.get(jn)
            if addr is None:
                continue
            qadr, vadr = addr
            self._data.qpos[qadr] = q
            self._data.qvel[vadr] = self._joint_state_vel.get(jn, 0.0)

        # Sync base free-joint from /odom if available, to align world-frame FK.
        if (
            self._base_free_qadr is not None
            and self._base_free_vadr is not None
            and self._odom_pose is not None
        ):
            p = self._odom_pose.position
            q = self._odom_pose.orientation
            qnorm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
            if qnorm < 1e-9:
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
            else:
                qw, qx, qy, qz = q.w / qnorm, q.x / qnorm, q.y / qnorm, q.z / qnorm

            qadr = self._base_free_qadr
            self._data.qpos[qadr + 0] = float(p.x)
            self._data.qpos[qadr + 1] = float(p.y)
            self._data.qpos[qadr + 2] = float(p.z)
            self._data.qpos[qadr + 3] = float(qw)
            self._data.qpos[qadr + 4] = float(qx)
            self._data.qpos[qadr + 5] = float(qy)
            self._data.qpos[qadr + 6] = float(qz)

            if self._odom_twist is not None:
                vadr = self._base_free_vadr
                tw = self._odom_twist
                self._data.qvel[vadr + 0] = float(tw.linear.x)
                self._data.qvel[vadr + 1] = float(tw.linear.y)
                self._data.qvel[vadr + 2] = float(tw.linear.z)
                self._data.qvel[vadr + 3] = float(tw.angular.x)
                self._data.qvel[vadr + 4] = float(tw.angular.y)
                self._data.qvel[vadr + 5] = float(tw.angular.z)

        mujoco.mj_forward(self._model, self._data)
        return True

    def _solve_dls(self, e6: np.ndarray, j6n: np.ndarray) -> np.ndarray:
        # 加权（位置/姿态）
        w = np.array(
            [self._w_pos, self._w_pos, self._w_pos, self._w_ori, self._w_ori, self._w_ori],
            dtype=np.float64,
        )
        if not self._use_orientation:
            w[3:] = 0.0
        ew = w * e6
        jw = w[:, None] * j6n

        jj_t = jw @ jw.T
        reg = (self._lam ** 2) * np.eye(6, dtype=np.float64)
        dq = jw.T @ np.linalg.solve(jj_t + reg, ew)
        return dq

    def _compute_qdot_from_cartesian_vel(self, xdot_cmd: np.ndarray) -> np.ndarray:
        """q̇_cmd = J⁺·ẋ_cmd，DLS 伪逆，复用 mj_jacBody 基础设施。"""
        jacp = np.zeros((3, self._model.nv), dtype=np.float64)
        jacr = np.zeros((3, self._model.nv), dtype=np.float64)
        mujoco.mj_jacBody(self._model, self._data, jacp, jacr, self._ee_body_id)
        jcols = np.array(self._vadr, dtype=np.int32)
        j6n = np.vstack([jacp[:, jcols], jacr[:, jcols]])  # (6, n_joints)
        lam2 = self._lam ** 2
        jjt = j6n @ j6n.T
        qdot = j6n.T @ np.linalg.solve(jjt + lam2 * np.eye(6, dtype=np.float64), xdot_cmd)
        return qdot

    def _uniformly_scale_qdot_cmd(self, qdot: np.ndarray) -> np.ndarray:
        """If any |q̇_i| exceeds max_joint_velocity_rad_s, scale entire vector uniformly."""
        lim = self._max_joint_vel_rad_s
        if lim <= 0.0 or qdot.size == 0:
            return qdot
        peak = float(np.max(np.abs(qdot)))
        if peak <= lim + 1e-12:
            return qdot
        scale = lim / peak
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_qdot_scale_warn_ns > 2_000_000_000:
            self.get_logger().warn(
                f"velocity_tracking: scaling q̇_cmd by {scale:.4f} "
                f"(peak |q̇|={peak:.3f} rad/s > max_joint_velocity_rad_s={lim:.3f})"
            )
            self._last_qdot_scale_warn_ns = now_ns
        return qdot * scale

    def _on_timer(self) -> None:
        if self._control_mode == "velocity_tracking":
            self._on_timer_velocity_tracking()
            return

        # Before any external target arrives, hold the arm at the initial joint pose
        # using joint-space PD + gravity compensation to prevent free-fall before latch.
        if (not self._external_target_received) and self._initial_joint_pose:
            if not self._joint_state_pos:
                return
            if not self._build_model_state_from_joint_states():
                return
            q_cur = np.array([self._joint_state_pos.get(jn, 0.0) for jn in self._controlled_joints], dtype=np.float64)
            q_vel = np.array([self._joint_state_vel.get(jn, 0.0) for jn in self._controlled_joints], dtype=np.float64)
            q_tgt = np.array([self._initial_joint_pose.get(jn, q_cur[i]) for i, jn in enumerate(self._controlled_joints)], dtype=np.float64)
            kp_scales = np.array([self._joint_kp_scale.get(jn, 1.0) for jn in self._controlled_joints], dtype=np.float64)
            tau = (self._kp * kp_scales) * (q_tgt - q_cur) - (self._kd * kp_scales) * q_vel
            if self._use_gravity_compensation:
                tau += np.array([float(self._data.qfrc_bias[v]) for v in self._vadr], dtype=np.float64)
            tau_clip = np.minimum(
                np.array([self._joint_to_tau_limit.get(jn, self._max_torque) for jn in self._controlled_joints], dtype=np.float64),
                self._max_torque,
            )
            tau = np.clip(tau, -tau_clip, tau_clip)
            if self._tau_initialized:
                tau = self._tau_lpf_alpha * tau + (1.0 - self._tau_lpf_alpha) * self._tau_prev
            self._tau_prev = tau.copy()
            self._tau_initialized = True
            self._publish_effort(tau)
            return

        if self._target_pos is None or self._target_quat_wxyz is None:
            return
        if not self._is_fresh(self._t_joint_state):
            if self._tau_initialized:
                self._publish_effort(self._tau_prev)
            return
        if self._require_odom_sync and not self._is_fresh(self._t_odom):
            if self._tau_initialized:
                self._publish_effort(self._tau_prev)
            return
        if not self._build_model_state_from_joint_states():
            if self._tau_initialized:
                self._publish_effort(self._tau_prev)
            return

        cur_pos = np.asarray(self._data.xpos[self._ee_body_id, :], dtype=np.float64)
        cur_q = np.asarray(self._data.xquat[self._ee_body_id, :], dtype=np.float64)

        e_pos = self._target_pos - cur_pos
        e_ori = _orientation_error_world(cur_q, self._target_quat_wxyz)
        e6 = np.concatenate([e_pos, e_ori])

        jacp = np.zeros((3, self._model.nv), dtype=np.float64)
        jacr = np.zeros((3, self._model.nv), dtype=np.float64)
        mujoco.mj_jacBody(self._model, self._data, jacp, jacr, self._ee_body_id)

        jcols = np.array(self._vadr, dtype=np.int32)
        j6n = np.vstack([jacp[:, jcols], jacr[:, jcols]])

        dq = self._solve_dls(e6, j6n)
        dq = np.clip(self._alpha * dq, -self._max_step, self._max_step)

        q_cur = np.array([self._joint_state_pos[jn] for jn in self._controlled_joints], dtype=np.float64)
        q_vel = np.array([self._joint_state_vel.get(jn, 0.0) for jn in self._controlled_joints], dtype=np.float64)
        q_next = np.clip(q_cur + dq, np.array(self._qmin), np.array(self._qmax))

        # Soft joint limits: clamp q_next to initial_joint_pose ± margin.
        # This prevents shoulder/elbow from drifting into degenerate configurations
        # (e.g. shoulder_lift going to -0.52 rad while IK tries to correct X error).
        # Roll joints are excluded: they are continuous with no gravity coupling.
        if self._joint_soft_limit_margin > 0.0 and self._initial_joint_pose:
            for i, jn in enumerate(self._controlled_joints):
                if "roll" in jn:
                    continue
                if jn not in self._initial_joint_pose:
                    continue
                q_init = self._initial_joint_pose[jn]
                soft_lo = max(self._qmin[i], q_init - self._joint_soft_limit_margin)
                soft_hi = min(self._qmax[i], q_init + self._joint_soft_limit_margin)
                q_next[i] = float(np.clip(q_next[i], soft_lo, soft_hi))

        # 关节空间 PD + 重力补偿 -> 力矩命令，交给 pr2_mujoco_sim 的 *_tau 执行器
        # Per-joint scaling reduces PD gains for light wrist/forearm joints to prevent oscillation.
        kp_scales = np.array([self._joint_kp_scale.get(jn, 1.0) for jn in self._controlled_joints], dtype=np.float64)
        tau = (self._kp * kp_scales) * (q_next - q_cur) - (self._kd * kp_scales) * q_vel
        if self._use_gravity_compensation:
            # qfrc_bias = gravity + Coriolis; adding it cancels these forces
            # so PD only needs to handle tracking error, not fight gravity.
            tau_bias = np.array(
                [float(self._data.qfrc_bias[vadr]) for vadr in self._vadr],
                dtype=np.float64,
            )
            tau = tau + tau_bias

        # Per-joint actuator limit takes precedence; max_torque is a global safety cap only.
        tau_lims = np.array(
            [self._joint_to_tau_limit.get(jn, self._max_torque) for jn in self._controlled_joints],
            dtype=np.float64,
        )
        if self._max_torque > 0:
            tau_lims = np.minimum(tau_lims, self._max_torque)
        tau = np.clip(tau, -tau_lims, tau_lims)

        if self._tau_initialized and self._tau_rate_limit > 0.0:
            dt = 1.0 / self._ctrl_hz
            dmax = self._tau_rate_limit * dt
            dtau = np.clip(tau - self._tau_prev, -dmax, dmax)
            tau = self._tau_prev + dtau

        if self._tau_initialized:
            tau = self._tau_lpf_alpha * tau + (1.0 - self._tau_lpf_alpha) * self._tau_prev

        self._tau_prev = tau.copy()
        self._tau_initialized = True
        self._publish_effort(tau)

    def _on_timer_velocity_tracking(self) -> None:
        dt = 1.0 / self._ctrl_hz

        # Pre-command: hold at initial joint pose (same logic as pose_tracking pre-latch)
        if self._cart_vel_cmd is None and self._initial_joint_pose:
            if not self._joint_state_pos:
                return
            if not self._build_model_state_from_joint_states():
                return
            q_cur = np.array([self._joint_state_pos.get(jn, 0.0) for jn in self._controlled_joints], dtype=np.float64)
            q_vel = np.array([self._joint_state_vel.get(jn, 0.0) for jn in self._controlled_joints], dtype=np.float64)
            q_tgt = np.array([self._initial_joint_pose.get(jn, q_cur[i]) for i, jn in enumerate(self._controlled_joints)], dtype=np.float64)
            kp_scales = np.array([self._joint_kp_scale.get(jn, 1.0) for jn in self._controlled_joints], dtype=np.float64)
            tau = (self._kp * kp_scales) * (q_tgt - q_cur) - (self._kd * kp_scales) * q_vel
            if self._use_gravity_compensation:
                tau += np.array([float(self._data.qfrc_bias[v]) for v in self._vadr], dtype=np.float64)
            tau_clip = np.minimum(
                np.array([self._joint_to_tau_limit.get(jn, self._max_torque) for jn in self._controlled_joints], dtype=np.float64),
                self._max_torque,
            )
            tau = np.clip(tau, -tau_clip, tau_clip)
            if self._tau_initialized:
                tau = self._tau_lpf_alpha * tau + (1.0 - self._tau_lpf_alpha) * self._tau_prev
            self._tau_prev = tau.copy()
            self._tau_initialized = True
            self._publish_effort(tau)
            return

        if not self._is_fresh(self._t_joint_state):
            if self._tau_initialized:
                self._publish_effort(self._tau_prev)
            return
        if self._require_odom_sync and not self._is_fresh(self._t_odom):
            if self._tau_initialized:
                self._publish_effort(self._tau_prev)
            return
        if not self._build_model_state_from_joint_states():
            if self._tau_initialized:
                self._publish_effort(self._tau_prev)
            return

        q_cur = np.array([self._joint_state_pos[jn] for jn in self._controlled_joints], dtype=np.float64)
        q_vel = np.array([self._joint_state_vel.get(jn, 0.0) for jn in self._controlled_joints], dtype=np.float64)

        # --- cc_ws-style synchronization ---
        # Every cycle, re-anchor _q_integrated to real q_cur (like synchronizeJointPositions).
        # This prevents open-loop drift: the integrator can never stray far from the real robot.
        # We keep a small "carry-over" delta so the PD doesn't see a sudden jump.
        if self._q_integrated is None:
            self._q_integrated = q_cur.copy()
        else:
            # Blend: pull _q_integrated toward q_cur by sync_alpha each cycle.
            # sync_alpha=1.0 → full re-anchor every cycle (maximum stability, no memory).
            # sync_alpha=0.0 → pure open-loop integration (original, unstable).
            # 0.3 keeps ~1 cycle of "intent" while preventing runaway drift.
            alpha_sync = self._vel_sync_alpha
            self._q_integrated = (1.0 - alpha_sync) * self._q_integrated + alpha_sync * q_cur

        # Integrate q̇_cmd if velocity command is fresh
        vel_age = (self.get_clock().now() - self._t_cart_vel).nanoseconds * 1e-9
        if self._cart_vel_cmd is not None and vel_age <= self._vel_cmd_timeout:
            qdot_raw = self._compute_qdot_from_cartesian_vel(self._cart_vel_cmd)
            qdot_cmd = self._uniformly_scale_qdot_cmd(qdot_raw)
            self._q_integrated = self._q_integrated + qdot_cmd * dt * self._vel_integration_gain

        # Hard joint limits
        self._q_integrated = np.clip(self._q_integrated, np.array(self._qmin), np.array(self._qmax))

        # Soft joint limits (same as pose_tracking)
        if self._joint_soft_limit_margin > 0.0 and self._initial_joint_pose:
            for i, jn in enumerate(self._controlled_joints):
                if "roll" in jn:
                    continue
                if jn not in self._initial_joint_pose:
                    continue
                q_init = self._initial_joint_pose[jn]
                soft_lo = max(self._qmin[i], q_init - self._joint_soft_limit_margin)
                soft_hi = min(self._qmax[i], q_init + self._joint_soft_limit_margin)
                self._q_integrated[i] = float(np.clip(self._q_integrated[i], soft_lo, soft_hi))

        # PD + gravity compensation → torque (identical to pose_tracking path)
        kp_scales = np.array([self._joint_kp_scale.get(jn, 1.0) for jn in self._controlled_joints], dtype=np.float64)
        tau = (self._kp * kp_scales) * (self._q_integrated - q_cur) - (self._kd * kp_scales) * q_vel
        if self._use_gravity_compensation:
            tau_bias = np.array([float(self._data.qfrc_bias[vadr]) for vadr in self._vadr], dtype=np.float64)
            tau = tau + tau_bias

        tau_lims = np.array([self._joint_to_tau_limit.get(jn, self._max_torque) for jn in self._controlled_joints], dtype=np.float64)
        if self._max_torque > 0:
            tau_lims = np.minimum(tau_lims, self._max_torque)
        tau = np.clip(tau, -tau_lims, tau_lims)

        if self._tau_initialized and self._tau_rate_limit > 0.0:
            dmax = self._tau_rate_limit * dt
            dtau = np.clip(tau - self._tau_prev, -dmax, dmax)
            tau = self._tau_prev + dtau

        if self._tau_initialized:
            tau = self._tau_lpf_alpha * tau + (1.0 - self._tau_lpf_alpha) * self._tau_prev

        self._tau_prev = tau.copy()
        self._tau_initialized = True
        self._publish_effort(tau)


def main() -> None:
    rclpy.init()
    node = Pr2LeftArmIk()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Launch global-shutdown can race with spin and trigger take_message conversion errors.
        if rclpy.ok():
            raise exc
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
