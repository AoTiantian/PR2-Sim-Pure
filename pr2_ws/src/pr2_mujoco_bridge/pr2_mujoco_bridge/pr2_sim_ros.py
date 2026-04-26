#!/usr/bin/env python3
"""
PR2 MuJoCo ↔ ROS 2 桥接节点（Jazzy）。

在仿真循环中调用 rclpy.spin_once，与 mujoco.viewer.launch_passive 兼容。

话题:
  发布: /joint_states (sensor_msgs/JointState)
        /odom (nav_msgs/Odometry)，frame: odom -> base_link
  订阅: /joint_commands (sensor_msgs/JointState)
          按关节名写入控制量：position→位置执行器，velocity→轮速执行器，effort→力矩电机
        /actuator_command (std_msgs/Float64MultiArray)
          长度须等于 model.nu，直接写入 MuJoCo ctrl（与旧脚本等价）
        /cmd_vel (geometry_msgs/Twist)，默认覆盖底盘转向(5–8)与轮速(9–16)，近似全向+自转

参数:
  model_path: MJCF 场景路径
  use_viewer: 是否打开 MuJoCo 被动窗口（无 DISPLAY 时默认 false；初始化失败会自动无头）
  torso_hold_effort: 躯干电机默认保持力（与原先 500 类似）
  cmd_vel_linear_gain / cmd_vel_angular_gain: 底盘速度映射增益
  use_cmd_vel: 若为 false，底盘仅由 /joint_commands 或 /actuator_command 控制
  demo_motion: 为 true 时复现旧版 pr2_sim.py 内置周期动作（夹爪/左臂/底盘）；完全由 ROS 控制时请设为 false
  lock_base_motion: 为 true 时锁定 base_link 的 free-joint（用于 arm-only 验证）
  joint_motion_log_rate_hz: >0 时按该频率在控制台打印关节 pos/vel（0 关闭）
  joint_motion_log_regex: 非空则只打印名称匹配该正则的关节；空则打印躯干/左臂/夹爪相关（名称含 torso_lift、以 l_ 开头、或含 gripper）

依赖: pip install mujoco（不在 rosdep 中）
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from typing import Dict, List, Tuple

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import glfw
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from tf2_ros import TransformBroadcaster


def _classify_actuator(name: str) -> str:
    if name.endswith("_pos"):
        return "position"
    if name.endswith("_vel"):
        return "velocity"
    if name.endswith("_tau"):
        return "effort"
    return "effort"


class Pr2MujocoSim(Node):
    def __init__(self) -> None:
        super().__init__("pr2_mujoco_sim")

        # #region agent log
        self._dbg_log_path = "/workspace/.cursor/debug-33df0d.log"
        self._dbg_last_mono = 0.0

        def _dbg_write(hypothesis_id: str, message: str, data: dict) -> None:
            try:
                import json as _json
                os.makedirs(os.path.dirname(self._dbg_log_path) or ".", exist_ok=True)

                payload = {
                    "sessionId": "33df0d",
                    "runId": os.environ.get("DEBUG_RUN_ID", "vel_mismatch"),
                    "hypothesisId": hypothesis_id,
                    "location": "pr2_sim_ros.py",
                    "message": message,
                    "data": data,
                    "timestamp": int(time.time() * 1000),
                }
                with open(self._dbg_log_path, "a", encoding="utf-8") as f:
                    f.write(_json.dumps(payload, ensure_ascii=False) + "\n")
            except Exception:
                pass

        self._dbg_write = _dbg_write
        self._dbg_write("H0_DebugInit", "sim init", {"pid": int(os.getpid())})
        # #endregion agent log

        self._dbg_prev_act_sign: Dict[str, int] = {}

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        )
        # 无图形环境（Docker/SSH 无 X11）时不要默认开窗口，避免 GLFW 报错退出
        _disp = os.environ.get("DISPLAY", "").strip()
        self.declare_parameter("use_viewer", bool(_disp))
        self.declare_parameter("torso_hold_effort", 500.0)
        self.declare_parameter("cmd_vel_linear_gain", 2.0)
        self.declare_parameter("cmd_vel_angular_gain", 1.5)
        self.declare_parameter("use_cmd_vel", True)
        # 默认开启：打开仿真窗口时能看到与旧 pr2_sim.py 相同的演示动作
        self.declare_parameter("demo_motion", True)
        self.declare_parameter("lock_base_motion", False)
        self.declare_parameter("lock_torso_motion", False)
        self.declare_parameter("lock_base_settle_sec", 0.6)
        # Minimal "hold initial pose" mode: do not step the simulator.
        # This effectively pauses MuJoCo time and keeps the model at the initial state.
        self.declare_parameter("pause_sim", False)
        # JSON dict of {joint_name: angle_rad} to set before simulation starts.
        # Eliminates large initial convergence motions (and the Coriolis disturbances
        # they cause) when initial_joint_pose targets differ from MuJoCo defaults.
        self.declare_parameter("initial_qpos_json", "")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("joint_motion_log_rate_hz", 0.0)
        self.declare_parameter("joint_motion_log_regex", "")

        model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self._use_viewer = (
            self.get_parameter("use_viewer").get_parameter_value().bool_value
        )
        self._torso_hold = (
            self.get_parameter("torso_hold_effort")
            .get_parameter_value()
            .double_value
        )
        self._lin_gain = (
            self.get_parameter("cmd_vel_linear_gain")
            .get_parameter_value()
            .double_value
        )
        self._ang_gain = (
            self.get_parameter("cmd_vel_angular_gain")
            .get_parameter_value()
            .double_value
        )
        self._use_cmd_vel = (
            self.get_parameter("use_cmd_vel").get_parameter_value().bool_value
        )
        self._demo_motion = (
            self.get_parameter("demo_motion").get_parameter_value().bool_value
        )
        self._lock_base_motion = (
            self.get_parameter("lock_base_motion").get_parameter_value().bool_value
        )
        self._lock_torso_motion = (
            self.get_parameter("lock_torso_motion").get_parameter_value().bool_value
        )
        self._lock_base_settle_sec = (
            self.get_parameter("lock_base_settle_sec").get_parameter_value().double_value
        )
        self._pause_sim = (
            self.get_parameter("pause_sim").get_parameter_value().bool_value
        )
        import json as _json
        _raw_qpos = str(self.get_parameter("initial_qpos_json").value).strip()
        self._initial_qpos: Dict[str, float] = {}
        if _raw_qpos:
            try:
                self._initial_qpos = {k: float(v) for k, v in _json.loads(_raw_qpos).items()}
                self.get_logger().info(f"initial_qpos_json 已加载: {self._initial_qpos}")
            except Exception as e:
                self.get_logger().warn(f"initial_qpos_json 解析失败: {e}")
        self._odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self._joint_motion_log_rate_hz = max(
            0.0,
            float(
                self.get_parameter("joint_motion_log_rate_hz")
                .get_parameter_value()
                .double_value
            ),
        )
        _jlog_re = (
            str(self.get_parameter("joint_motion_log_regex").value).strip()
        )
        self._joint_motion_log_re: re.Pattern[str] | None
        if _jlog_re:
            try:
                self._joint_motion_log_re = re.compile(_jlog_re)
            except re.error as exc:
                self.get_logger().warn(
                    f"joint_motion_log_regex 非法，已当作未设置: {exc}"
                )
                self._joint_motion_log_re = None
        else:
            self._joint_motion_log_re = None
        self._last_joint_motion_log_mono: float | None = None

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._nu = int(self._model.nu)

        # joint_name -> [(actuator_index, field), ...]
        self._joint_to_act: Dict[str, List[Tuple[int, str]]] = {}
        for a in range(self._nu):
            j_id = int(self._model.actuator_trnid[a, 0])
            jn = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, j_id
            )
            an = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, a
            )
            kind = _classify_actuator(an or "")
            if jn:
                self._joint_to_act.setdefault(jn, []).append((a, kind))

        # 躯干执行器：torso_lift_tau
        self._torso_act_id: int | None = None
        for a in range(self._nu):
            an = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, a
            )
            if an == "torso_lift_tau":
                self._torso_act_id = a
                break

        self._lock = threading.Lock()
        self._ctrl_target = np.zeros(self._nu, dtype=np.float64)
        if self._torso_act_id is not None:
            self._ctrl_target[self._torso_act_id] = self._torso_hold

        self._twist = Twist()
        self._full_actuator_override = False

        self._pub_joint_states = self.create_publisher(JointState, "joint_states", 10)
        # Extra dynamics signals for debugging gravity-induced droop in velocity-bottom control.
        # These publish per-joint generalized forces (N·m or N) in MuJoCo's generalized coordinates.
        self._pub_joint_bias = self.create_publisher(JointState, "mujoco/joint_bias", 10)
        self._pub_joint_actuator = self.create_publisher(JointState, "mujoco/joint_actuator", 10)
        self._pub_odom = self.create_publisher(Odometry, "odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            JointState, "joint_commands", self._cb_joint_commands, 10
        )
        self.create_subscription(
            Float64MultiArray,
            "actuator_command",
            self._cb_actuator_command,
            10,
        )
        self.create_subscription(Twist, "cmd_vel", self._cb_cmd_vel, 10)
        self.create_subscription(
            Bool,
            "disable_actuator_override",
            self._cb_disable_actuator_override,
            10,
        )

        self._body_base = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        if self._body_base < 0:
            self.get_logger().warn('未找到 body "base_link"，/odom 将不发布位姿')

        self._prev_base_pos = np.zeros(3)
        self._prev_odom_time = None
        self._base_free_qadr: int | None = None
        self._base_free_vadr: int | None = None
        self._base_lock_qpos: np.ndarray | None = None
        self._base_lock_pending = False
        self._torso_qadr: int | None = None
        self._torso_vadr: int | None = None
        self._torso_lock_qpos: float | None = None
        self._torso_lock_pending = False

        # Cache dof indices for key left arm joints (for debug only).
        self._dbg_arm_joints = [
            "l_shoulder_pan_joint",
            "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint",
            "l_elbow_flex_joint",
            "l_forearm_roll_joint",
            "l_wrist_flex_joint",
            "l_wrist_roll_joint",
        ]
        self._dbg_joint_vadr: Dict[str, int] = {}
        for jn in self._dbg_arm_joints:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self._dbg_joint_vadr[jn] = int(self._model.jnt_dofadr[jid])

        # 内置演示用执行器（与 scripts/pr2_sim.py 一致，按名称解析）
        self._demo_gripper_l = self._actuator_id_by_name("l_gripper_pos")
        self._demo_steer_ids = self._actuator_ids_by_names(
            (
                "fl_caster_steer",
                "fr_caster_steer",
                "bl_caster_steer",
                "br_caster_steer",
            )
        )
        self._demo_wheel_ids = self._actuator_ids_by_names(
            (
                "fl_caster_l_wheel_vel",
                "fl_caster_r_wheel_vel",
                "fr_caster_l_wheel_vel",
                "fr_caster_r_wheel_vel",
                "bl_caster_l_wheel_vel",
                "bl_caster_r_wheel_vel",
                "br_caster_l_wheel_vel",
                "br_caster_r_wheel_vel",
            )
        )
        self._demo_torso = self._actuator_id_by_name("torso_lift_tau")
        self._demo_arm_specs: List[Tuple[int, str]] = []
        for aname, label in (
            ("l_shoulder_pan_tau", "左肩-水平"),
            ("l_shoulder_lift_tau", "左肩-抬举"),
            ("l_elbow_flex_tau", "左肘"),
            ("l_wrist_flex_tau", "左腕"),
        ):
            aid = self._actuator_id_by_name(aname)
            if aid >= 0:
                self._demo_arm_specs.append((aid, label))
        gr = self._model.actuator_ctrlrange
        gi = self._demo_gripper_l
        if gi >= 0 and gr[gi][1] > 0:
            self._demo_gripper_max = float(gr[gi][1])
        else:
            self._demo_gripper_max = 0.5
        self._demo_l_shoulder_lift_act = self._actuator_id_by_name(
            "l_shoulder_lift_tau"
        )
        self._demo_l_elbow_act = self._actuator_id_by_name("l_elbow_flex_tau")

        self.get_logger().info(
            f"已加载 MuJoCo 模型: {model_path} (nu={self._nu})"
        )
        self.get_logger().info(
            f"demo_motion={self._demo_motion} | "
            f"lock_base_motion={self._lock_base_motion} | "
            f"lock_torso_motion={self._lock_torso_motion} | "
            f"lock_settle_sec={self._lock_base_settle_sec:.2f} | "
            f"joint_motion_log_rate_hz={self._joint_motion_log_rate_hz:.2f} | "
            "订阅: joint_commands, actuator_command, cmd_vel, disable_actuator_override | "
            "发布: joint_states, odom+tf"
        )

        if self._body_base >= 0:
            for j in range(self._model.njnt):
                if int(self._model.jnt_type[j]) != int(mujoco.mjtJoint.mjJNT_FREE):
                    continue
                if int(self._model.jnt_bodyid[j]) != int(self._body_base):
                    continue
                self._base_free_qadr = int(self._model.jnt_qposadr[j])
                self._base_free_vadr = int(self._model.jnt_dofadr[j])
                break

        # Find torso_lift_joint for optional locking
        torso_jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "torso_lift_joint")
        if torso_jid >= 0:
            self._torso_qadr = int(self._model.jnt_qposadr[torso_jid])
            self._torso_vadr = int(self._model.jnt_dofadr[torso_jid])

    def _actuator_id_by_name(self, name: str) -> int:
        i = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        return int(i)

    def _actuator_ids_by_names(self, names: Tuple[str, ...]) -> List[int]:
        out: List[int] = []
        for n in names:
            i = self._actuator_id_by_name(n)
            if i >= 0:
                out.append(i)
        return out

    def _cb_joint_commands(self, msg: JointState) -> None:
        with self._lock:
            if self._full_actuator_override:
                return
            for i, jn in enumerate(msg.name):
                for aid, kind in self._joint_to_act.get(jn, []):
                    if kind == "position" and i < len(msg.position):
                        v = msg.position[i]
                        if not math.isnan(v):
                            self._ctrl_target[aid] = float(v)
                    elif kind == "velocity" and i < len(msg.velocity):
                        v = msg.velocity[i]
                        if not math.isnan(v):
                            self._ctrl_target[aid] = float(v)
                    elif kind == "effort" and i < len(msg.effort):
                        v = msg.effort[i]
                        if not math.isnan(v):
                            self._ctrl_target[aid] = float(v)

    def _cb_actuator_command(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if len(msg.data) != self._nu:
                self.get_logger().warn(
                    f"actuator_command 长度 {len(msg.data)} != nu {self._nu}，已忽略"
                )
                return
            self._full_actuator_override = True
            self._ctrl_target[:] = np.array(msg.data, dtype=np.float64)

    def _cb_cmd_vel(self, msg: Twist) -> None:
        self._twist = msg

    def _cb_disable_actuator_override(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._full_actuator_override = False
            self.get_logger().info("已关闭 actuator 全向量覆盖，恢复 joint_commands / cmd_vel 逻辑")

    def _apply_cmd_vel_to_ctrl(self, target: np.ndarray) -> None:
        if not self._use_cmd_vel:
            return
        vx = float(self._twist.linear.x)
        vy = float(self._twist.linear.y)
        wz = float(self._twist.angular.z)
        # 全零时不改底盘执行器，避免关掉 demo 的轮速/转向
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            return

        vplanar = math.hypot(vx, vy)
        steer_ids = self._demo_steer_ids
        wheel_ids = self._demo_wheel_ids
        if len(steer_ids) < 4 or len(wheel_ids) < 8:
            return

        if vplanar > 1e-6:
            steer = math.atan2(vy, vx)
            v_wheel = vplanar * self._lin_gain
            for sid in steer_ids:
                target[sid] = steer
            for wid in wheel_ids:
                target[wid] = v_wheel
        elif abs(wz) > 1e-6:
            for sid in steer_ids:
                target[sid] = 0.0
            g = wz * self._ang_gain
            pattern = [g, g, -g, -g, g, g, -g, -g]
            for k, wid in enumerate(wheel_ids[:8]):
                target[wid] = pattern[k]

    def _apply_pr2_demo_motion(self, ctrl: np.ndarray, t: float) -> None:
        """与 scripts/pr2_sim.py 中 apply_advanced_separated_control 相同逻辑。"""
        ctrl[:] = 0.0
        if self._demo_torso >= 0:
            ctrl[self._demo_torso] = 500.0
        if self._demo_gripper_l >= 0:
            ctrl[self._demo_gripper_l] = (
                (math.sin(2.0 * t) + 1.0) / 2.0
            ) * self._demo_gripper_max
        if self._demo_arm_specs:
            arm_step = int(t / 3.0) % len(self._demo_arm_specs)
            target_act, _name = self._demo_arm_specs[arm_step]
            base_torque = (
                -45.0 if target_act == self._demo_l_shoulder_lift_act else 0.0
            )
            amplitude = (
                20.0 if target_act == self._demo_l_elbow_act else 40.0
            )
            ctrl[target_act] = base_torque + amplitude * math.sin(3.0 * t)

        base_phase = int(t / 6.0) % 2
        steer_ids = self._demo_steer_ids
        wheel_ids = self._demo_wheel_ids
        if len(steer_ids) >= 4 and len(wheel_ids) >= 8:
            if base_phase == 0:
                steer_ang = 0.0
            else:
                steer_ang = 1.5708
            for sid in steer_ids:
                ctrl[sid] = steer_ang
            wcmd = 2.0 * math.sin(1.0 * t)
            for wid in wheel_ids:
                ctrl[wid] = wcmd

    def _fill_joint_state(self, msg: JointState) -> None:
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        m, d = self._model, self._data
        for j in range(m.njnt):
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if not jn:
                continue
            jt = int(m.jnt_type[j])
            if jt == int(mujoco.mjtJoint.mjJNT_FREE):
                continue
            if jt == int(mujoco.mjtJoint.mjJNT_BALL):
                continue
            qadr = int(m.jnt_qposadr[j])
            vadr = int(m.jnt_dofadr[j])
            msg.name.append(jn)
            msg.position.append(float(d.qpos[qadr]))
            msg.velocity.append(float(d.qvel[vadr]))
            msg.effort.append(0.0)

    def _fill_joint_effort(self, msg: JointState, *, source: str) -> None:
        """
        Publish MuJoCo generalized forces aligned with the same joint ordering as /joint_states.

        source:
          - "bias": d.qfrc_bias (gravity + Coriolis/centrifugal + passive)
          - "actuator": d.qfrc_actuator (generalized force produced by actuators)
        """
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        m, d = self._model, self._data
        if source == "bias":
            qfrc = d.qfrc_bias
        elif source == "actuator":
            qfrc = d.qfrc_actuator
        else:
            raise ValueError(f"unknown effort source: {source}")

        for j in range(m.njnt):
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if not jn:
                continue
            jt = int(m.jnt_type[j])
            if jt == int(mujoco.mjtJoint.mjJNT_FREE):
                continue
            if jt == int(mujoco.mjtJoint.mjJNT_BALL):
                continue
            vadr = int(m.jnt_dofadr[j])
            msg.name.append(jn)
            # effort is the generalized force for this 1-DoF joint
            msg.effort.append(float(qfrc[vadr]))

    def _joint_motion_log_match(self, joint_name: str) -> bool:
        if self._joint_motion_log_re is not None:
            return self._joint_motion_log_re.search(joint_name) is not None
        return (
            "torso_lift" in joint_name
            or joint_name.startswith("l_")
            or "gripper" in joint_name
        )

    def _maybe_log_joint_motion(self, js: JointState) -> None:
        if self._joint_motion_log_rate_hz <= 0.0:
            return
        period = 1.0 / self._joint_motion_log_rate_hz
        now = time.monotonic()
        if self._last_joint_motion_log_mono is not None:
            if now - self._last_joint_motion_log_mono < period:
                return
        self._last_joint_motion_log_mono = now
        parts: List[str] = []
        for name, p, v in zip(js.name, js.position, js.velocity):
            if not self._joint_motion_log_match(name):
                continue
            parts.append(f"{name}:p={p:.3f},v={v:.3f}")
        if not parts:
            return
        self.get_logger().info(
            f"joint_motion sim_t={float(self._data.time):.3f} | " + " | ".join(parts)
        )

    def _publish_odom(self, stamp) -> None:
        if self._body_base < 0:
            return
        m, d = self._model, self._data
        i = self._body_base
        # MuJoCo 3.x: xpos/xquat 形状为 (nbody, 3) / (nbody, 4)，不能再用扁平切片 i*3:i*3+3
        pos = np.asarray(d.xpos[i, :], dtype=np.float64).copy()
        quat = np.asarray(d.xquat[i, :], dtype=np.float64)
        qw, qx, qy, qz = (float(quat[j]) for j in range(4))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        now = time.monotonic()
        if self._prev_odom_time is not None:
            dt = now - self._prev_odom_time
            if dt > 1e-6:
                lin = (pos - self._prev_base_pos) / dt
                odom.twist.twist.linear.x = float(lin[0])
                odom.twist.twist.linear.y = float(lin[1])
                odom.twist.twist.linear.z = float(lin[2])
        self._prev_base_pos = pos
        self._prev_odom_time = now

        self._pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def run(self) -> None:
        mujoco.mj_resetData(self._model, self._data)

        # Apply initial joint positions before the first step so the arm starts
        # at the desired configuration, avoiding large convergence motions.
        if self._initial_qpos:
            for jn, angle in self._initial_qpos.items():
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid >= 0:
                    self._data.qpos[int(self._model.jnt_qposadr[jid])] = float(angle)
            mujoco.mj_forward(self._model, self._data)
            self.get_logger().info("initial_qpos_json 已应用到 MuJoCo 初始状态")

        self._base_lock_qpos = None
        self._base_lock_pending = bool(
            self._lock_base_motion and self._base_free_qadr is not None
        )
        self._torso_lock_qpos = None
        self._torso_lock_pending = bool(
            self._lock_torso_motion and self._torso_qadr is not None
        )

        # If we pause the sim, lock immediately at the current state (no time progression).
        if self._pause_sim:
            if self._lock_base_motion and self._base_free_qadr is not None:
                q0 = int(self._base_free_qadr)
                self._base_lock_qpos = np.array(
                    self._data.qpos[q0 : q0 + 7], dtype=np.float64
                )
                self._base_lock_pending = False
            if self._lock_torso_motion and self._torso_qadr is not None:
                self._torso_lock_qpos = float(self._data.qpos[int(self._torso_qadr)])
                self._torso_lock_pending = False

        js = JointState()
        js.header.frame_id = self._base_frame
        js_bias = JointState()
        js_bias.header.frame_id = self._base_frame
        js_act = JointState()
        js_act.header.frame_id = self._base_frame

        def one_step() -> None:
            rclpy.spin_once(self, timeout_sec=0.0)

            with self._lock:
                override = self._full_actuator_override
                ctrl_saved = self._ctrl_target.copy()

            if override:
                ctrl = ctrl_saved
            elif self._demo_motion:
                ctrl = np.zeros(self._nu, dtype=np.float64)
                self._apply_pr2_demo_motion(ctrl, float(self._data.time))
            else:
                ctrl = ctrl_saved
                if self._torso_act_id is not None:
                    ctrl[self._torso_act_id] = self._torso_hold
            self._apply_cmd_vel_to_ctrl(ctrl)

            self._data.ctrl[:] = ctrl
            if not self._pause_sim:
                mujoco.mj_step(self._model, self._data)
                if self._data.time >= self._lock_base_settle_sec:
                    if self._base_lock_pending and self._base_free_qadr is not None:
                        q0 = self._base_free_qadr
                        self._base_lock_qpos = np.array(
                            self._data.qpos[q0 : q0 + 7], dtype=np.float64
                        )
                        self._base_lock_pending = False
                    if self._torso_lock_pending and self._torso_qadr is not None:
                        self._torso_lock_qpos = float(self._data.qpos[self._torso_qadr])
                        self._torso_lock_pending = False
            needs_forward = False
            if (
                self._lock_base_motion
                and self._base_lock_qpos is not None
                and self._base_free_qadr is not None
                and self._base_free_vadr is not None
            ):
                q0 = self._base_free_qadr
                v0 = self._base_free_vadr
                self._data.qpos[q0 : q0 + 7] = self._base_lock_qpos
                self._data.qvel[v0 : v0 + 6] = 0.0
                needs_forward = True
            if (
                self._lock_torso_motion
                and self._torso_lock_qpos is not None
                and self._torso_qadr is not None
                and self._torso_vadr is not None
            ):
                self._data.qpos[self._torso_qadr] = self._torso_lock_qpos
                self._data.qvel[self._torso_vadr] = 0.0
                needs_forward = True
            if needs_forward:
                mujoco.mj_forward(self._model, self._data)

            # #region agent log
            now_m = time.monotonic()
            if now_m - self._dbg_last_mono > 1.0:
                self._dbg_last_mono = now_m
                # For each arm joint: read commanded velocity target (from ctrl) for velocity actuators,
                # actual qvel, and generalized forces (bias/actuator).
                jdbg = {}
                for jn in self._dbg_arm_joints:
                    vadr = self._dbg_joint_vadr.get(jn, None)
                    if vadr is None:
                        continue
                    # commanded target: first velocity actuator mapped to this joint (if any)
                    vcmd = None
                    fr = None
                    for aid, kind in self._joint_to_act.get(jn, []):
                        if kind == "velocity":
                            vcmd = float(self._data.ctrl[aid])
                            try:
                                fr = [float(self._model.actuator_forcerange[aid][0]), float(self._model.actuator_forcerange[aid][1])]
                            except Exception:
                                fr = None
                            break
                    act = float(self._data.qfrc_actuator[vadr])
                    act_abs = float(abs(act))
                    sat = False
                    sat_margin = None
                    if fr is not None:
                        hi = float(max(abs(fr[0]), abs(fr[1])))
                        sat_margin = float(hi - act_abs)
                        sat = bool(sat_margin <= 1e-6)
                    prev_s = self._dbg_prev_act_sign.get(jn, 0)
                    cur_s = 1 if act > 1e-9 else (-1 if act < -1e-9 else 0)
                    flip = bool(prev_s != 0 and cur_s != 0 and cur_s != prev_s)
                    if cur_s != 0:
                        self._dbg_prev_act_sign[jn] = cur_s
                    jdbg[jn] = {
                        "vcmd": vcmd,
                        "vact": float(self._data.qvel[vadr]),
                        "bias": float(self._data.qfrc_bias[vadr]),
                        "act": act,
                        "forcerange": fr,
                        "v_err": (float(vcmd) - float(self._data.qvel[vadr])) if vcmd is not None else None,
                        "sat": sat,
                        "sat_margin": sat_margin,
                        "act_sign_flip": flip,
                    }
                self._dbg_write(
                    "H3_ActuatorOrMappingLimits",
                    "sim step arm velocity/forces",
                    {"sim_time": float(self._data.time), "joints": jdbg},
                )
            # #endregion agent log

            stamp = self.get_clock().now().to_msg()
            js.header.stamp = stamp
            self._fill_joint_state(js)
            self._pub_joint_states.publish(js)
            js_bias.header.stamp = stamp
            js_act.header.stamp = stamp
            self._fill_joint_effort(js_bias, source="bias")
            self._fill_joint_effort(js_act, source="actuator")
            self._pub_joint_bias.publish(js_bias)
            self._pub_joint_actuator.publish(js_act)
            self._maybe_log_joint_motion(js)
            self._publish_odom(stamp)

        def run_headless() -> None:
            self.get_logger().info(
                "无头模式运行 MuJoCo（无窗口）。需要图形界面时请设置 DISPLAY 并执行: "
                "xhost +local: 或正确配置 xauth，再用 -p use_viewer:=true"
            )
            while rclpy.ok():
                step_start = time.time()
                one_step()
                dt = self._model.opt.timestep - (time.time() - step_start)
                if dt > 0:
                    time.sleep(dt)

        def viewer_available() -> bool:
            # Probe GLFW first to avoid process exit when viewer init fails on X11 auth/display.
            try:
                ok = bool(glfw.init())
                if ok:
                    glfw.terminate()
                return ok
            except Exception:
                return False

        if self._use_viewer:
            if not viewer_available():
                self.get_logger().error(
                    "请求 use_viewer=true，但 GLFW 初始化失败（常见于 DISPLAY/xhost/xauth 未配置）。"
                    "自动切换为无头模式继续运行。"
                )
                run_headless()
                return
            try:
                with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
                    self.get_logger().info(
                        "MuJoCo 被动 viewer 已启动（关闭窗口即退出节点）"
                    )
                    while viewer.is_running():
                        step_start = time.time()
                        one_step()
                        viewer.sync()
                        dt = self._model.opt.timestep - (time.time() - step_start)
                        if dt > 0:
                            time.sleep(dt)
            except Exception as exc:
                self.get_logger().error(
                    f"无法打开 MuJoCo 窗口（{type(exc).__name__}: {exc}）。"
                    "常见原因：无桌面、DISPLAY/xhost 未授权、在容器内未映射 X11。"
                    "将自动改为无头模式；也可显式使用: --ros-args -p use_viewer:=false"
                )
                run_headless()
        else:
            run_headless()


def main() -> None:
    rclpy.init()
    node = Pr2MujocoSim()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
