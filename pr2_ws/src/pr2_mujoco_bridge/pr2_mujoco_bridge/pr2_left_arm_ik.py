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
        self.declare_parameter("joint_command_topic", "joint_commands")
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
        self._joint_command_topic = self.get_parameter("joint_command_topic").value
        hz = float(self.get_parameter("control_rate_hz").value)

        self._use_orientation = bool(self.get_parameter("use_orientation").value)
        self._w_pos = float(self.get_parameter("position_weight").value)
        self._w_ori = float(self.get_parameter("orientation_weight").value)
        self._lam = float(self.get_parameter("damping_lambda").value)
        self._alpha = float(self.get_parameter("step_size").value)
        self._max_step = float(self.get_parameter("max_step_rad").value)

        self._kp = float(self.get_parameter("torque_kp").value)
        self._kd = float(self.get_parameter("torque_kd").value)
        self._max_torque = float(self.get_parameter("max_torque").value)

        self._controlled_joints: List[str] = list(
            self.get_parameter("controlled_joints").value
        )

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name
        )
        if self._ee_body_id < 0:
            raise RuntimeError(f"end_effector_body 不存在: {ee_body_name}")

        # joint 映射（仅支持 1DoF 铰链/滑动关节）
        self._joint_ids: List[int] = []
        self._qadr: List[int] = []
        self._vadr: List[int] = []
        self._qmin: List[float] = []
        self._qmax: List[float] = []
        self._joint_to_tau_limit: Dict[str, float] = {}

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
        self._target_pos = None
        self._target_quat_wxyz = None

        self._pub_cmd = self.create_publisher(JointState, self._joint_command_topic, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self.create_subscription(PoseStamped, self._target_topic, self._on_target_pose, 10)
        self.create_timer(1.0 / max(hz, 1e-3), self._on_timer)

        self.get_logger().info(
            "IK 节点启动: "
            f"target={self._target_topic}, joints={self._controlled_joints}, ee_body={ee_body_name}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_state_pos[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self._joint_state_vel[name] = float(msg.velocity[i])

    def _on_target_pose(self, msg: PoseStamped) -> None:
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

    def _build_model_state_from_joint_states(self) -> bool:
        # 用接收到的 joint_states 回填 MuJoCo qpos/qvel
        missing = [jn for jn in self._controlled_joints if jn not in self._joint_state_pos]
        if missing:
            return False

        for jn in self._controlled_joints:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            qadr = int(self._model.jnt_qposadr[jid])
            vadr = int(self._model.jnt_dofadr[jid])
            self._data.qpos[qadr] = self._joint_state_pos[jn]
            self._data.qvel[vadr] = self._joint_state_vel.get(jn, 0.0)

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

    def _on_timer(self) -> None:
        if self._target_pos is None or self._target_quat_wxyz is None:
            return
        if not self._build_model_state_from_joint_states():
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

        # 关节空间 PD -> 力矩命令，交给 pr2_mujoco_sim 的 *_tau 执行器
        tau = self._kp * (q_next - q_cur) - self._kd * q_vel

        tau_lims = np.array(
            [self._joint_to_tau_limit.get(jn, self._max_torque) for jn in self._controlled_joints],
            dtype=np.float64,
        )
        tau_clip = np.minimum(tau_lims, self._max_torque)
        tau = np.clip(tau, -tau_clip, tau_clip)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._controlled_joints)
        msg.effort = [float(x) for x in tau]
        self._pub_cmd.publish(msg)



def main() -> None:
    rclpy.init()
    node = Pr2LeftArmIk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
