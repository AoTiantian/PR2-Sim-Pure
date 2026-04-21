#!/usr/bin/env python3
"""PR2 左臂末端笛卡尔空间顺应控制节点。

控制流程：
  外部力(WrenchStamped) -> 顺应动力学(M*a+D*v=F) -> 虚拟EE速度
    -> DLS逆运动学 -> 期望关节速度 -> 积分期望关节位置
    -> PD力矩控制 -> /joint_commands (effort)

参数 active_axes 控制哪些笛卡尔轴响应力：
  Phase 1: [1, 0, 0]  仅X轴
  Phase 2: [1, 1, 1]  全三轴
"""

from __future__ import annotations
from typing import List

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmAdmittanceNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_admittance")

        self.declare_parameter("model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml")
        self.declare_parameter("ee_body_name", "l_gripper_tool_frame")
        self.declare_parameter("controlled_joints", [
            "l_shoulder_pan_joint", "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint", "l_elbow_flex_joint",
            "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint",
        ])
        self.declare_parameter("active_axes", [1, 0, 0])
        self.declare_parameter("mass_x", 2.0)
        self.declare_parameter("mass_y", 2.0)
        self.declare_parameter("mass_z", 2.0)
        self.declare_parameter("damping_x", 100.0)
        self.declare_parameter("damping_y", 100.0)
        self.declare_parameter("damping_z", 100.0)
        self.declare_parameter("control_frequency", 100.0)
        self.declare_parameter("dls_lambda", 0.15)
        self.declare_parameter("ee_vel_max", 0.06)     # m/s, EE velocity saturation
        self.declare_parameter("qdot_des_max", 0.4)   # rad/s, joint velocity saturation
        self.declare_parameter("torque_kp", 0.0)    # position gain (0 = velocity-servo mode)
        self.declare_parameter("torque_kd", 50.0)  # velocity bandwidth K [rad/s]: τ = τ_bias + M(q)*K*Δqdot
        self.declare_parameter("max_torque", 60.0)
        self.declare_parameter("wrench_topic", "wbc/arm/external_wrench")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("ee_pose_topic", "wbc/arm/ee_pose_log")

        model_path = self.get_parameter("model_path").value
        ee_body_name = self.get_parameter("ee_body_name").value
        self._joints: List[str] = list(self.get_parameter("controlled_joints").value)
        active_raw = list(self.get_parameter("active_axes").value)
        self._active_mask = np.array(
            [float(v) for v in active_raw[:3]], dtype=np.float64)

        self._M = np.array([
            self.get_parameter("mass_x").value,
            self.get_parameter("mass_y").value,
            self.get_parameter("mass_z").value,
        ], dtype=np.float64)
        self._D = np.array([
            self.get_parameter("damping_x").value,
            self.get_parameter("damping_y").value,
            self.get_parameter("damping_z").value,
        ], dtype=np.float64)

        hz = float(self.get_parameter("control_frequency").value)
        self._dt = 1.0 / max(hz, 1.0)
        self._lam = float(self.get_parameter("dls_lambda").value)
        self._ee_vel_max = float(self.get_parameter("ee_vel_max").value)
        self._qdot_des_max = float(self.get_parameter("qdot_des_max").value)
        self._kp = float(self.get_parameter("torque_kp").value)
        self._kd = float(self.get_parameter("torque_kd").value)
        self._max_torque = float(self.get_parameter("max_torque").value)

        wrench_topic = self.get_parameter("wrench_topic").value
        cmd_topic = self.get_parameter("joint_command_topic").value
        pose_topic = self.get_parameter("ee_pose_topic").value

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
        if self._ee_body_id < 0:
            raise RuntimeError(f"末端体不存在: {ee_body_name}")

        self._qadr: List[int] = []
        self._vadr: List[int] = []
        self._qmin: List[float] = []
        self._qmax: List[float] = []
        for jn in self._joints:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"关节不存在: {jn}")
            self._qadr.append(int(self._model.jnt_qposadr[jid]))
            self._vadr.append(int(self._model.jnt_dofadr[jid]))
            self._qmin.append(float(self._model.jnt_range[jid, 0]))
            self._qmax.append(float(self._model.jnt_range[jid, 1]))

        self._tau_limits = np.full(len(self._joints), self._max_torque)
        for a in range(self._model.nu):
            an = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
            if not an.endswith("_tau"):
                continue
            jid = int(self._model.actuator_trnid[a, 0])
            jn = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, jid) or ""
            if jn in self._joints:
                idx = self._joints.index(jn)
                lim = float(max(
                    abs(self._model.actuator_ctrlrange[a, 0]),
                    abs(self._model.actuator_ctrlrange[a, 1])))
                self._tau_limits[idx] = lim

        # 所有带 qpos 地址的 1DOF 关节映射（用于同步完整模型状态以获取正确的重力/Jacobian）
        self._all_joint_qadr = {}
        self._all_joint_vadr = {}
        for jid in range(self._model.njnt):
            jname = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if not jname:
                continue
            jtype = int(self._model.jnt_type[jid])
            # 仅处理 1DOF 关节（HINGE / SLIDE）
            if jtype in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                self._all_joint_qadr[jname] = int(self._model.jnt_qposadr[jid])
                self._all_joint_vadr[jname] = int(self._model.jnt_dofadr[jid])

        self._ee_vel = np.zeros(3)
        self._f_ext = np.zeros(3)
        self._q_cur = np.zeros(len(self._joints))
        self._qdot_cur = np.zeros(len(self._joints))
        self._q_des = np.zeros(len(self._joints))  # used only when kp > 0
        self._full_state = {}  # name -> (pos, vel)
        self._initialized = False
        self._q_dot_des = np.zeros(len(self._joints))

        self._pub_cmd = self.create_publisher(JointState, cmd_topic, 10)
        self._pub_pose = self.create_publisher(PoseStamped, pose_topic, 10)
        self.create_subscription(
            WrenchStamped, wrench_topic, self._wrench_cb, 10)
        self.create_subscription(
            JointState, "joint_states", self._joint_state_cb, 20)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"臂顺应控制器启动: active_axes={active_raw}, "
            f"M={self._M.tolist()}, D={self._D.tolist()}"
        )

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self._f_ext[0] = msg.wrench.force.x
        self._f_ext[1] = msg.wrench.force.y
        self._f_ext[2] = msg.wrench.force.z

    def _joint_state_cb(self, msg: JointState) -> None:
        # 保存完整关节状态
        for i, name in enumerate(msg.name):
            p = float(msg.position[i]) if i < len(msg.position) else 0.0
            v = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
            self._full_state[name] = (p, v)
        # 提取受控 7 关节
        for k, jn in enumerate(self._joints):
            if jn in self._full_state:
                self._q_cur[k], self._qdot_cur[k] = self._full_state[jn]
        if not self._initialized and all(j in self._full_state for j in self._joints):
            self._q_des = self._q_cur.copy()
            self._initialized = True
            self.get_logger().info('初始化完成，q_des = q_cur')

    def _control_loop(self) -> None:
        if not self._initialized:
            return

        # 同步所有已知关节到 MuJoCo，保证重力补偿和 Jacobian 准确
        for name, (p, v) in self._full_state.items():
            if name in self._all_joint_qadr:
                self._data.qpos[self._all_joint_qadr[name]] = p
                self._data.qvel[self._all_joint_vadr[name]] = v
        mujoco.mj_forward(self._model, self._data)

        # 计算平动雅可比 (3 x 7)
        jacp = np.zeros((3, self._model.nv), dtype=np.float64)
        jacr = np.zeros((3, self._model.nv), dtype=np.float64)
        mujoco.mj_jacBody(
            self._model, self._data, jacp, jacr, self._ee_body_id)
        J = jacp[:, self._vadr]

        # 顺应动力学：M*a + D*v = F
        f = self._active_mask * self._f_ext
        a_ee = (f - self._D * self._ee_vel) / np.maximum(self._M, 1e-6)
        self._ee_vel += a_ee * self._dt

        # EE velocity saturation — prevents blow-up near singularities
        ee_vel_norm = np.linalg.norm(self._ee_vel)
        if ee_vel_norm > self._ee_vel_max:
            self._ee_vel = self._ee_vel * (self._ee_vel_max / ee_vel_norm)

        # DLS IK: q_dot = J^T (J J^T + lambda^2 I)^{-1} * v_ee
        JJT = J @ J.T
        reg = (self._lam ** 2) * np.eye(3, dtype=np.float64)
        self._q_dot_des = J.T @ np.linalg.solve(JJT + reg, self._ee_vel)

        # Joint velocity saturation
        self._q_dot_des = np.clip(
            self._q_dot_des, -self._qdot_des_max, self._qdot_des_max)

        # Joint limit protection: smoothly zero velocity as joints approach limits
        _MARGIN = 0.08  # rad — start scaling back within 0.08 rad of limit
        for i in range(len(self._joints)):
            q, qmin, qmax = self._q_cur[i], self._qmin[i], self._qmax[i]
            if qmax > qmin:
                if q < qmin + _MARGIN and self._q_dot_des[i] < 0:
                    self._q_dot_des[i] *= max(0.0, (q - qmin) / _MARGIN)
                elif q > qmax - _MARGIN and self._q_dot_des[i] > 0:
                    self._q_dot_des[i] *= max(0.0, (qmax - q) / _MARGIN)

        # 当 kp > 0 时保留位置轨迹积分（仅辅助）
        if self._kp > 0.0:
            self._q_des = np.clip(
                self._q_des + self._q_dot_des * self._dt,
                np.array(self._qmin), np.array(self._qmax))

        # 重力补偿（mj_forward 已更新 qfrc_bias）
        tau_bias = np.array(
            [self._data.qfrc_bias[v] for v in self._vadr], dtype=np.float64)

        # Computed-torque PD controller:
        #   τ = τ_bias + M_arm(q) · (K_bw·Δqdot + Kp·Δq)
        # Full inertia matrix gives uniform bandwidth K_bw for all joints.
        # Stability: K_bw·dt < 2 and Kp·dt² < K_bw·dt (K_bw=50, Kp=10, dt=0.01 ✓).
        M_full = np.zeros((self._model.nv, self._model.nv), dtype=np.float64)
        mujoco.mj_fullM(self._model, M_full, self._data.qM)
        M_arm = M_full[np.ix_(self._vadr, self._vadr)]
        delta_qdot = self._q_dot_des - self._qdot_cur
        delta_q = self._q_des - self._q_cur if self._kp > 0.0 else np.zeros(len(self._joints))
        tau = tau_bias + M_arm @ (self._kd * delta_qdot + self._kp * delta_q)
        tau_lim = np.minimum(self._tau_limits, self._max_torque)
        tau = np.clip(tau, -tau_lim, tau_lim)

        # 发布力矩指令
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self._joints)
        cmd.effort = [float(x) for x in tau]
        self._pub_cmd.publish(cmd)

        # 发布末端位姿（用于日志记录）
        ee_pos = self._data.xpos[self._ee_body_id]
        pose = PoseStamped()
        pose.header.stamp = cmd.header.stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(ee_pos[0])
        pose.pose.position.y = float(ee_pos[1])
        pose.pose.position.z = float(ee_pos[2])
        self._pub_pose.publish(pose)


def main() -> None:
    rclpy.init()
    node = ArmAdmittanceNode()
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
