#!/usr/bin/env python3
"""
PR2 MuJoCo ↔ ROS 2 桥接节点（Jazzy）。

在仿真循环中调用 rclpy.spin_once，与 mujoco.viewer.launch_passive 兼容。

话题:
  发布: /joint_states (sensor_msgs/JointState)
        /odom (nav_msgs/Odometry)，frame: odom -> base_link
  订阅: /joint_commands (sensor_msgs/JointState)
          按关节名写入控制量：position→位置执行器，velocity→轮速执行器，effort→力矩电机；
          左臂 *_tau 电机：JointState.velocity 为速度参考，由仿真内 CTC 换算为力矩（见参数 ctc_*）
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
  lock_base_motion / lock_arm_motion: 为 true 时锁定对应的基座/左臂姿态（用于固定目标 wrench 校准）
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
from typing import Any, Dict, List, Tuple

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import glfw
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
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
        # Use a user-writable path when the container launches ROS2 as the
        # non-root ``user`` account; the historical workspace trace is
        # root-owned and silently rejected writes.
        self._dbg_log_path = os.environ.get(
            "PR2_SIM_DEBUG_LOG", "/tmp/pr2_sim_debug_trace.log"
        )
        self._debug_trace = os.environ.get("PR2_DEBUG_TRACE", "0").lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        self._dbg_last_mono = 0.0
        self._dbg_last_cmdvel_mono = 0.0

        def _dbg_write(hypothesis_id: str, message: str, data: dict) -> None:
            if not self._debug_trace:
                return
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
        self._last_steer_cmd: float | None = None
        self._last_steer_cmd_mono: float | None = None

        # Cache base steering/wheel joint indices for tracking debug.
        # (Derived from actuator_trnid so it stays consistent with the model.)
        self._dbg_base_steer_qadr: List[int] = []
        self._dbg_base_steer_name: List[str] = []
        self._dbg_base_wheel_vadr: List[int] = []
        self._dbg_base_wheel_name: List[str] = []

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        )
        # 无图形环境（Docker/SSH 无 X11）时不要默认开窗口，避免 GLFW 报错退出
        _disp = os.environ.get("DISPLAY", "").strip()
        self.declare_parameter("use_viewer", bool(_disp))
        self.declare_parameter("viewer_lookat", [0.0, 0.0, 0.8])
        self.declare_parameter("viewer_distance", 4.0)
        self.declare_parameter("viewer_azimuth", 135.0)
        self.declare_parameter("viewer_elevation", -20.0)
        self.declare_parameter("torso_hold_effort", 500.0)
        self.declare_parameter("cmd_vel_linear_gain", 2.0)
        self.declare_parameter("cmd_vel_angular_gain", 1.5)
        self.declare_parameter("use_cmd_vel", True)
        # 默认开启：打开仿真窗口时能看到与旧 pr2_sim.py 相同的演示动作
        self.declare_parameter("demo_motion", True)
        self.declare_parameter("lock_base_motion", False)
        self.declare_parameter("base_passive_damping", 0.0)
        self.declare_parameter("lock_torso_motion", False)
        self.declare_parameter(
            "lock_arm_motion",
            False,
        )
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
        # Virtual human hand force: applied directly to the board body via xfrc_applied.
        self.declare_parameter("hand_force_topic", "virtual_human/hand_force")
        self.declare_parameter("hand_force_enable", False)
        # Offset from board COM to the hand contact point in board-local frame (metres).
        self.declare_parameter("hand_force_offset", [1.0, 0.0, 0.0])
        # Board-local point coupled to the robot by the orientation-free
        # connect constraint.  This is the robot endpoint, not a load-share
        # percentage or an injected torque.
        self.declare_parameter("robot_board_contact_offset", [-1.0, 0.0, 0.0])
        # If true, remove only the moment generated by the force lever arm. Any
        # explicit torque in the incoming WrenchStamped is still preserved.
        # Set false for a physically free board whose rotation is governed by
        # the endpoint force and torque balance.
        # Keep the physical lever-arm moment by default.  A cancellation
        # switch is retained only for legacy demos; the collaborative scene
        # must expose the natural r×F board moment.
        self.declare_parameter("hand_force_cancel_moment", False)
        self.declare_parameter("board_pose_topic", "mujoco/board_pose")
        self.declare_parameter("human_hand_pose_topic", "mujoco/human_hand_pose")
        self.declare_parameter("left_wrist_wrench_topic", "mujoco/left_wrist_wrench")
        # MuJoCo's site force sensor reports the internal cut force.  Negating it
        # gives the external board-on-robot wrench used by the admittance loop.
        self.declare_parameter("left_wrist_wrench_sign", -1.0)
        self.declare_parameter("left_wrist_tare_duration_sec", 1.0)
        self.declare_parameter("joint_motion_log_rate_hz", 0.0)
        self.declare_parameter("joint_motion_log_regex", "")
        # When cmd_vel steering changes fast, caster steering may lag.
        # Gate wheel speed until steering is (approximately) aligned.
        self.declare_parameter("cmd_vel_steer_gate_err_start_rad", 0.25)
        self.declare_parameter("cmd_vel_steer_gate_err_full_rad", 0.80)
        self.declare_parameter("cmd_vel_steer_gate_k_min", 0.20)
        self.declare_parameter("cmd_vel_steer_rate_limit_rad_s", 2.0)
        # If steering is badly misaligned, stop wheel drive until casters align.
        self.declare_parameter("cmd_vel_steer_gate_hard_stop", True)
        # Boost steer rate when target jumps far (prevents "stuck in old direction").
        self.declare_parameter("cmd_vel_steer_rate_boost_rad_s", 8.0)
        self.declare_parameter("cmd_vel_steer_rate_boost_err_rad", 1.0)
        # Timed gripper release: if > 0, the left gripper will open to its max position
        # at this sim-time (sec), releasing any grasped object.
        self.declare_parameter("gripper_open_time_sec", -1.0)
        # Optional persistent left-gripper position command.  A negative value
        # preserves the legacy default; collaborative grasp scenes set it.
        self.declare_parameter("left_gripper_hold_position", -1.0)
        # When cmd_vel magnitude is small, atan2(vy,vx) becomes very sensitive to noise.
        # Hold the steering direction under small planar speed to avoid caster hunting.
        self.declare_parameter("cmd_vel_dir_hold_vplanar_min", 0.05)
        self.declare_parameter("cmd_vel_dir_lpf_alpha", 0.25)
        self.declare_parameter("cmd_vel_steer_target_lpf_alpha", 0.35)
        # Left arm: computed torque (velocity ref in JointState.velocity -> motor torques).
        self.declare_parameter("ctc_enable", True)
        self.declare_parameter("ctc_kp", 100.0)
        self.declare_parameter("ctc_kd", 20.0)
        # Upward support used by the arm's computed-torque loop.  Auto-balance
        # derives the complete board load from the model and subtracts the
        # measured human endpoint force, leaving the robot force residual.
        # No endpoint torque is injected for vertical load sharing.
        self.declare_parameter("ctc_payload_force_z", 0.0)
        self.declare_parameter("ctc_payload_auto_balance", False)
        self.declare_parameter("ctc_payload_total_force_z", 0.0)
        # Avoid a one-sided payload impulse while the human ROS publisher is
        # still coming up and has only published its initial zero wrench.
        self.declare_parameter("ctc_balance_startup_duration_sec", 1.0)
        # Small robot-side height hold around the initially grasped board
        # height.  This is zero at the static equilibrium; it only supplies a
        # bounded restoring/damping force after a vertical disturbance.
        self.declare_parameter("ctc_vertical_hold_kp", 80.0)
        self.declare_parameter("ctc_vertical_hold_kd", 30.0)
        self.declare_parameter("ctc_vertical_hold_force_limit", 12.0)
        self.declare_parameter(
            "robot_support_force_topic", "mujoco/robot_support_force"
        )

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
        self._base_passive_damping = max(
            float(
                self.get_parameter("base_passive_damping")
                .get_parameter_value()
                .double_value
            ),
            0.0,
        )
        self._lock_torso_motion = (
            self.get_parameter("lock_torso_motion").get_parameter_value().bool_value
        )
        self._lock_arm_motion = bool(
            self.get_parameter("lock_arm_motion").get_parameter_value().bool_value
        )
        self._lock_base_settle_sec = (
            self.get_parameter("lock_base_settle_sec").get_parameter_value().double_value
        )
        self._pause_sim = (
            self.get_parameter("pause_sim").get_parameter_value().bool_value
        )
        import json as _json
        _raw_qpos = str(self.get_parameter("initial_qpos_json").value).strip()
        self._initial_qpos: Dict[str, Any] = {}
        if _raw_qpos:
            try:
                parsed = _json.loads(_raw_qpos)
                if not isinstance(parsed, dict):
                    raise ValueError("top-level value must be a JSON object")
                self._initial_qpos = dict(parsed)
                self.get_logger().info(f"initial_qpos_json 已加载: {self._initial_qpos}")
            except Exception as e:
                self.get_logger().warn(f"initial_qpos_json 解析失败: {e}")
                self._initial_qpos = {}
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
        self._steer_gate_err_start = float(
            self.get_parameter("cmd_vel_steer_gate_err_start_rad")
            .get_parameter_value()
            .double_value
        )
        self._steer_gate_err_full = float(
            self.get_parameter("cmd_vel_steer_gate_err_full_rad")
            .get_parameter_value()
            .double_value
        )
        self._steer_gate_k_min = float(
            self.get_parameter("cmd_vel_steer_gate_k_min")
            .get_parameter_value()
            .double_value
        )
        self._steer_rate_limit = float(
            self.get_parameter("cmd_vel_steer_rate_limit_rad_s")
            .get_parameter_value()
            .double_value
        )
        self._steer_gate_hard_stop = bool(
            self.get_parameter("cmd_vel_steer_gate_hard_stop")
            .get_parameter_value()
            .bool_value
        )
        self._steer_rate_boost = float(
            self.get_parameter("cmd_vel_steer_rate_boost_rad_s")
            .get_parameter_value()
            .double_value
        )
        self._steer_rate_boost = float(max(0.0, self._steer_rate_boost))
        self._steer_rate_boost_err = float(
            self.get_parameter("cmd_vel_steer_rate_boost_err_rad")
            .get_parameter_value()
            .double_value
        )
        self._steer_rate_boost_err = float(max(0.0, self._steer_rate_boost_err))
        self._cmd_vel_dir_hold_vmin = float(
            self.get_parameter("cmd_vel_dir_hold_vplanar_min")
            .get_parameter_value()
            .double_value
        )
        self._cmd_vel_dir_lpf_alpha = float(
            self.get_parameter("cmd_vel_dir_lpf_alpha").get_parameter_value().double_value
        )
        self._cmd_vel_dir_lpf_alpha = float(max(0.0, min(1.0, self._cmd_vel_dir_lpf_alpha)))
        self._cmd_vel_steer_target_lpf_alpha = float(
            self.get_parameter("cmd_vel_steer_target_lpf_alpha")
            .get_parameter_value()
            .double_value
        )
        self._cmd_vel_steer_target_lpf_alpha = float(
            max(0.0, min(1.0, self._cmd_vel_steer_target_lpf_alpha))
        )
        self._ctc_enable = bool(
            self.get_parameter("ctc_enable").get_parameter_value().bool_value
        )
        self._ctc_kp = float(
            self.get_parameter("ctc_kp").get_parameter_value().double_value
        )
        self._ctc_kd = float(
            self.get_parameter("ctc_kd").get_parameter_value().double_value
        )
        self._ctc_payload_force_z = float(
            self.get_parameter("ctc_payload_force_z")
            .get_parameter_value()
            .double_value
        )
        self._ctc_payload_auto_balance = bool(
            self.get_parameter("ctc_payload_auto_balance")
            .get_parameter_value()
            .bool_value
        )
        self._ctc_payload_total_force_z = float(
            self.get_parameter("ctc_payload_total_force_z")
            .get_parameter_value()
            .double_value
        )
        self._ctc_balance_startup_duration = max(
            float(
                self.get_parameter("ctc_balance_startup_duration_sec")
                .get_parameter_value()
                .double_value
            ),
            0.0,
        )
        self._ctc_vertical_hold_kp = max(
            float(self.get_parameter("ctc_vertical_hold_kp").value), 0.0
        )
        self._ctc_vertical_hold_kd = max(
            float(self.get_parameter("ctc_vertical_hold_kd").value), 0.0
        )
        self._ctc_vertical_hold_force_limit = max(
            float(self.get_parameter("ctc_vertical_hold_force_limit").value), 0.0
        )
        self._ctc_support_force_z = 0.0
        self._ctc_board_hold_target_z: float | None = None
        self._ctc_hold_prev_z: float | None = None
        self._gripper_open_time = float(
            self.get_parameter("gripper_open_time_sec")
            .get_parameter_value()
            .double_value
        )
        self._left_gripper_hold_position = float(
            self.get_parameter("left_gripper_hold_position").value
        )
        self._gripper_open_done = False
        self._cmd_vel_vx_f = 0.0
        self._cmd_vel_vy_f = 0.0
        self._last_steer_target = None
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

        # Left-arm CTC: velocity reference on /joint_commands -> motor torques (see robot_pr2.xml motors).
        self._CTC_ARM_JOINTS: Tuple[str, ...] = (
            "l_shoulder_pan_joint",
            "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint",
            "l_elbow_flex_joint",
            "l_forearm_roll_joint",
            "l_wrist_flex_joint",
            "l_wrist_roll_joint",
        )
        self._ctc_vel_ref: Dict[str, float] = {jn: 0.0 for jn in self._CTC_ARM_JOINTS}
        self._ctc_joint_set = frozenset(self._CTC_ARM_JOINTS)
        self._ctc_vadr: List[int] = []
        self._ctc_qadr: List[int] = []
        self._ctc_act_ids: List[int] = []
        self._ctc_M_buf = np.zeros((self._model.nv, self._model.nv), dtype=np.float64)
        self._ctc_q_ref = np.zeros(7, dtype=np.float64)
        self._ctc_qdot_r_prev = np.zeros(7, dtype=np.float64)
        self._ctc_vel_prev_inited = False
        _ctc_missing: List[str] = []
        for jn in self._CTC_ARM_JOINTS:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                _ctc_missing.append(jn)
                continue
            self._ctc_qadr.append(int(self._model.jnt_qposadr[jid]))
            self._ctc_vadr.append(int(self._model.jnt_dofadr[jid]))
            aname = jn.replace("_joint", "_tau")
            aid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                _ctc_missing.append(aname)
                continue
            self._ctc_act_ids.append(int(aid))
        if len(self._ctc_act_ids) != 7:
            self.get_logger().warn(
                "CTC 左臂未完整配置（需要 7 个 *_tau 执行器），已禁用 CTC。"
                f" missing={_ctc_missing}"
            )
            self._ctc_enable = False
            self._ctc_act_ids = []
            self._ctc_vadr = []
            self._ctc_qadr = []

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
        self._pub_board_grasped = self.create_publisher(Bool, "mujoco/board_grasped", 10)
        self._pub_sim_time = self.create_publisher(
            Float64, "mujoco/sim_time", 10
        )
        self._pub_board_pose = self.create_publisher(
            PoseStamped, str(self.get_parameter("board_pose_topic").value), 10
        )
        self._pub_human_hand_pose = self.create_publisher(
            PoseStamped, str(self.get_parameter("human_hand_pose_topic").value), 10
        )
        self._pub_left_wrist_wrench = self.create_publisher(
            WrenchStamped,
            str(self.get_parameter("left_wrist_wrench_topic").value),
            10,
        )
        self._pub_robot_support_force = self.create_publisher(
            Float64,
            str(self.get_parameter("robot_support_force_topic").value),
            10,
        )
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
        # Virtual human hand force — applied directly to board body via xfrc_applied.
        self._hand_force_enable = bool(
            self.get_parameter("hand_force_enable").get_parameter_value().bool_value
        )
        self._hand_force_latest: WrenchStamped | None = None
        self._robot_board_contact_offset = np.asarray(
            list(self.get_parameter("robot_board_contact_offset").value),
            dtype=np.float64,
        )
        if self._robot_board_contact_offset.shape != (3,):
            raise RuntimeError("robot_board_contact_offset must have len=3")
        if self._hand_force_enable:
            self.create_subscription(
                WrenchStamped,
                str(self.get_parameter("hand_force_topic").value),
                self._cb_hand_force,
                10,
            )
            self.get_logger().info(
                f"hand force enabled: subscribing to {self.get_parameter('hand_force_topic').value}"
            )

        self._body_base = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        if self._body_base < 0:
            self.get_logger().warn('未找到 body "base_link"，/odom 将不发布位姿')

        self._board_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "grasp_board"
        )
        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "l_gripper_tool_frame"
        )
        if self._board_body_id < 0:
            if self._hand_force_enable:
                self.get_logger().warn(
                    '未找到 body "grasp_board"，hand force 将无法施加'
                )
        self._ctc_model_payload_force_z = (
            float(self._model.body_mass[self._board_body_id])
            * abs(float(self._model.opt.gravity[2]))
            if self._board_body_id >= 0
            else 0.0
        )
        self._left_force_sensor_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SENSOR, "pr2/left_force"
        )
        self._left_torque_sensor_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SENSOR, "pr2/left_torque"
        )
        self._left_wrist_site_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SITE, "l_wrist_ft_site"
        )
        self._left_wrist_wrench_sign = float(
            self.get_parameter("left_wrist_wrench_sign").value
        )
        self._left_wrist_tare_duration = max(
            float(self.get_parameter("left_wrist_tare_duration_sec").value),
            0.0,
        )
        self._left_wrist_tare_sum = np.zeros(6, dtype=np.float64)
        self._left_wrist_tare_count = 0
        self._left_wrist_bias: np.ndarray | None = None
        if min(
            self._left_force_sensor_id,
            self._left_torque_sensor_id,
            self._left_wrist_site_id,
        ) < 0:
            self.get_logger().warn(
                "left wrist F/T sensor is incomplete; mujoco/left_wrist_wrench "
                "will not be published"
            )

        self._board_geom_ids = self._geom_ids_by_names(("board_geom",))
        self._board_left_finger_geom_ids = self._geom_ids_by_names(
            ("l_gripper_l_board_contact",)
        )
        self._board_right_finger_geom_ids = self._geom_ids_by_names(
            ("l_gripper_r_board_contact",)
        )
        self._board_connect_eq_id = int(
            mujoco.mj_name2id(
                self._model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                "robot_board_point_contact",
            )
        )
        self._board_grasp_ready = bool(
            self._board_geom_ids
            and self._board_left_finger_geom_ids
            and self._board_right_finger_geom_ids
        )
        if self._board_grasp_ready:
            self.get_logger().info(
                "board grasp contact 检测已启用: 发布 mujoco/board_grasped"
            )

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
        self._arm_lock_qpos: np.ndarray | None = None
        self._arm_lock_pending = False

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
        self._dbg_joint_qadr: Dict[str, int] = {}
        self._dbg_joint_range: Dict[str, Tuple[float, float]] = {}
        for jn in self._dbg_arm_joints:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self._dbg_joint_vadr[jn] = int(self._model.jnt_dofadr[jid])
                self._dbg_joint_qadr[jn] = int(self._model.jnt_qposadr[jid])
                try:
                    r0 = float(self._model.jnt_range[jid][0])
                    r1 = float(self._model.jnt_range[jid][1])
                    self._dbg_joint_range[jn] = (r0, r1)
                except Exception:
                    pass

        # 内置演示用执行器（与 scripts/pr2_sim.py 一致，按名称解析）
        self._demo_gripper_l = self._actuator_id_by_name("l_gripper_pos")
        self._gripper_open_act_id = self._demo_gripper_l  # same actuator
        if (
            self._demo_gripper_l >= 0
            and self._left_gripper_hold_position >= 0.0
        ):
            ctrl_range = self._model.actuator_ctrlrange[self._demo_gripper_l]
            self._ctrl_target[self._demo_gripper_l] = float(
                np.clip(
                    self._left_gripper_hold_position,
                    float(ctrl_range[0]),
                    float(ctrl_range[1]),
                )
            )
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
            "CTC payload support: "
            f"fallback={self._ctc_payload_force_z:.3f}N, "
            f"auto_balance={self._ctc_payload_auto_balance}, "
            f"total_override={self._ctc_payload_total_force_z:.3f}N, "
            f"model_weight={self._ctc_model_payload_force_z:.3f}N"
        )
        self.get_logger().info(
            f"demo_motion={self._demo_motion} | "
            f"lock_base_motion={self._lock_base_motion} | "
            f"lock_torso_motion={self._lock_torso_motion} | "
            f"lock_arm_motion={self._lock_arm_motion} | "
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

        # #region agent log
        # Build steering/wheel joint index caches for root-cause analysis of cmd_vel tracking.
        try:
            for aid in self._demo_steer_ids:
                if aid < 0:
                    continue
                jid = int(self._model.actuator_trnid[aid, 0])
                jn = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, jid) or f"jid_{jid}"
                qadr = int(self._model.jnt_qposadr[jid])
                self._dbg_base_steer_qadr.append(qadr)
                self._dbg_base_steer_name.append(str(jn))
            for aid in self._demo_wheel_ids[:8]:
                if aid < 0:
                    continue
                jid = int(self._model.actuator_trnid[aid, 0])
                jn = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, jid) or f"jid_{jid}"
                vadr = int(self._model.jnt_dofadr[jid])
                self._dbg_base_wheel_vadr.append(vadr)
                self._dbg_base_wheel_name.append(str(jn))
            self._dbg_write(
                "H_CmdVelSteerWheelInit",
                "cached steer/wheel joint indices",
                {
                    "n_steer": int(len(self._dbg_base_steer_qadr)),
                    "steer_joints": list(self._dbg_base_steer_name),
                    "n_wheels": int(len(self._dbg_base_wheel_vadr)),
                    "wheel_joints": list(self._dbg_base_wheel_name),
                },
            )
        except Exception as _exc:
            self._dbg_write("H_CmdVelSteerWheelInit", "cache failed", {"err": str(_exc)})
        # #endregion agent log

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

    def _geom_ids_by_names(self, names: Tuple[str, ...]) -> List[int]:
        out: List[int] = []
        for name in names:
            gid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if gid >= 0:
                out.append(int(gid))
        return out

    def _publish_board_grasped(self) -> None:
        if not self._board_grasp_ready:
            return
        board = set(self._board_geom_ids)
        left = set(self._board_left_finger_geom_ids)
        right = set(self._board_right_finger_geom_ids)
        left_touch = False
        right_touch = False
        for i in range(int(self._data.ncon)):
            c = self._data.contact[i]
            g1 = int(c.geom1)
            g2 = int(c.geom2)
            if (g1 in board and g2 in left) or (g2 in board and g1 in left):
                left_touch = True
            if (g1 in board and g2 in right) or (g2 in board and g1 in right):
                right_touch = True
            if left_touch and right_touch:
                break
        msg = Bool()
        # Point-contact scenes have no pad collision pair: the MuJoCo connect
        # equality itself is the active robot grasp reaction.
        point_contact = (
            self._board_connect_eq_id >= 0
            and bool(self._data.eq_active[self._board_connect_eq_id])
        )
        msg.data = bool(point_contact or (left_touch and right_touch))
        self._pub_board_grasped.publish(msg)

    def _publish_board_and_wrist_state(self, stamp) -> None:
        """Publish measured board/hand poses and the physical wrist wrench."""
        if self._board_body_id >= 0:
            bid = self._board_body_id
            p = self._data.xpos[bid]
            q = self._data.xquat[bid]
            board = PoseStamped()
            board.header.stamp = stamp
            board.header.frame_id = self._odom_frame
            board.pose.position.x = float(p[0])
            board.pose.position.y = float(p[1])
            board.pose.position.z = float(p[2])
            board.pose.orientation.w = float(q[0])
            board.pose.orientation.x = float(q[1])
            board.pose.orientation.y = float(q[2])
            board.pose.orientation.z = float(q[3])
            self._pub_board_pose.publish(board)

            r_local = np.asarray(
                list(self.get_parameter("hand_force_offset").value),
                dtype=np.float64,
            )
            hand_p = p + self._data.xmat[bid].reshape(3, 3) @ r_local
            hand = PoseStamped()
            hand.header = board.header
            hand.pose.position.x = float(hand_p[0])
            hand.pose.position.y = float(hand_p[1])
            hand.pose.position.z = float(hand_p[2])
            hand.pose.orientation.w = float(q[0])
            hand.pose.orientation.x = float(q[1])
            hand.pose.orientation.y = float(q[2])
            hand.pose.orientation.z = float(q[3])
            self._pub_human_hand_pose.publish(hand)

        if min(
            self._left_force_sensor_id,
            self._left_torque_sensor_id,
            self._left_wrist_site_id,
        ) < 0:
            return
        fadr = int(self._model.sensor_adr[self._left_force_sensor_id])
        tadr = int(self._model.sensor_adr[self._left_torque_sensor_id])
        force_local = self._data.sensordata[fadr : fadr + 3].copy()
        torque_local = self._data.sensordata[tadr : tadr + 3].copy()
        site_rotation = self._data.site_xmat[
            self._left_wrist_site_id
        ].reshape(3, 3)
        force_world = (
            self._left_wrist_wrench_sign * site_rotation @ force_local
        )
        torque_world = (
            self._left_wrist_wrench_sign * site_rotation @ torque_local
        )
        raw_wrench = np.concatenate([force_world, torque_world])
        if (
            self._left_wrist_bias is None
            and float(self._data.time) <= self._left_wrist_tare_duration
        ):
            self._left_wrist_tare_sum += raw_wrench
            self._left_wrist_tare_count += 1
            measured_wrench = np.zeros(6, dtype=np.float64)
        else:
            if self._left_wrist_bias is None:
                count = max(self._left_wrist_tare_count, 1)
                self._left_wrist_bias = self._left_wrist_tare_sum / count
                self.get_logger().info(
                    "left wrist F/T tare complete: "
                    f"bias={np.round(self._left_wrist_bias, 3).tolist()}"
                )
            measured_wrench = raw_wrench - self._left_wrist_bias
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = self._odom_frame
        wrench.wrench.force.x = float(measured_wrench[0])
        wrench.wrench.force.y = float(measured_wrench[1])
        wrench.wrench.force.z = float(measured_wrench[2])
        wrench.wrench.torque.x = float(measured_wrench[3])
        wrench.wrench.torque.y = float(measured_wrench[4])
        wrench.wrench.torque.z = float(measured_wrench[5])
        self._pub_left_wrist_wrench.publish(wrench)

    def _ctc_sync_reference_from_state(self) -> None:
        """Align CTC reference position with current q (after mj_forward / reset)."""
        if not self._ctc_enable or len(self._ctc_qadr) != 7:
            self._ctc_vel_prev_inited = False
            return
        for i, qadr in enumerate(self._ctc_qadr):
            self._ctc_q_ref[i] = float(self._data.qpos[qadr])
        self._ctc_qdot_r_prev[:] = 0.0
        self._ctc_vel_prev_inited = False

    def _sync_point_contact_anchor(self) -> None:
        """Synchronize the robot-board grasp after applying runtime arm qpos.

        MuJoCo compiles an equality from the MJCF defaults, while this demo
        supplies a different arm posture through ``initial_qpos_json``.  For
        the rigid weld grasp we update both the board-to-tool position and the
        relative orientation.  The connect branch is retained for old scene
        files so the bridge remains backwards compatible.
        """
        eq_id = int(getattr(self, "_board_connect_eq_id", -1))
        if eq_id < 0 or self._board_body_id < 0 or self._ee_body_id < 0:
            return
        board_id = int(self._board_body_id)
        ee_id = int(self._ee_body_id)
        board_pos = np.asarray(self._data.xpos[board_id], dtype=np.float64)
        board_rot = np.asarray(self._data.xmat[board_id], dtype=np.float64).reshape(3, 3)
        ee_pos = np.asarray(self._data.xpos[ee_id], dtype=np.float64)
        ee_rot = np.asarray(self._data.xmat[ee_id], dtype=np.float64).reshape(3, 3)
        eq_type = int(self._model.eq_type[eq_id])
        if eq_type == int(mujoco.mjtEq.mjEQ_WELD):
            # MuJoCo's weld layout is asymmetric (see mj_equalityAnchors):
            #   eq_data[0:3] = anchor on body2 (tool)
            #   eq_data[3:6] = anchor on body1 (board)
            #   eq_data[6:10] = relative orientation (wxyz)
            # The board endpoint must therefore be written to 3:6.  Writing
            # it to 0:3 attaches the tool origin to that point and makes the
            # board appear to pass through the gripper instead of being held
            # at its end.
            rel_pos = board_rot.T @ (ee_pos - board_pos)
            rel_rot = board_rot.T @ ee_rot
            rel_quat = np.zeros((4, 1), dtype=np.float64)
            mujoco.mju_mat2Quat(rel_quat, rel_rot.reshape(9, 1))
            # Use the measured runtime geometry rather than the nominal
            # [-1, 0, 0] value.  The configured arm posture places the tool
            # about 11 mm off that ideal endpoint; preserving the actual
            # relative pose avoids a startup impulse from the weld solver.
            self._model.eq_data[eq_id, 0:3] = 0.0
            self._model.eq_data[eq_id, 3:6] = rel_pos
            self._model.eq_data[eq_id, 6:10] = rel_quat[:, 0]
            # Runtime equality-data edits require rebuilding MuJoCo's
            # compiled constants before the next constraint solve.  Important:
            # mj_setConst() also resets the supplied MjData to the model's
            # default state.  This function is called *after* initial_qpos_json
            # has been applied, so preserve and restore the complete dynamic
            # state around the rebuild; otherwise the arm silently jumps back
            # to the all-zero XML posture and the board is no longer between
            # the gripper fingers in the viewer.
            qpos = self._data.qpos.copy()
            qvel = self._data.qvel.copy()
            act = self._data.act.copy()
            ctrl = self._data.ctrl.copy()
            qfrc_applied = self._data.qfrc_applied.copy()
            xfrc_applied = self._data.xfrc_applied.copy()
            mujoco.mj_setConst(self._model, self._data)
            self._data.qpos[:] = qpos
            self._data.qvel[:] = qvel
            self._data.act[:] = act
            self._data.ctrl[:] = ctrl
            self._data.qfrc_applied[:] = qfrc_applied
            self._data.xfrc_applied[:] = xfrc_applied
            mujoco.mj_forward(self._model, self._data)
            self.get_logger().info(
                "rigid board grasp synchronized: "
                f"board_to_tool={np.round(rel_pos, 4).tolist()}, "
                "initial_position_error_mm=0.000"
            )
            return

        anchor_world = board_pos + board_rot @ self._robot_board_contact_offset
        anchor_ee_local = ee_rot.T @ (anchor_world - ee_pos)
        self._model.eq_data[eq_id, 0:3] = self._robot_board_contact_offset
        self._model.eq_data[eq_id, 3:6] = anchor_ee_local
        qpos = self._data.qpos.copy()
        qvel = self._data.qvel.copy()
        act = self._data.act.copy()
        ctrl = self._data.ctrl.copy()
        qfrc_applied = self._data.qfrc_applied.copy()
        xfrc_applied = self._data.xfrc_applied.copy()
        mujoco.mj_setConst(self._model, self._data)
        self._data.qpos[:] = qpos
        self._data.qvel[:] = qvel
        self._data.act[:] = act
        self._data.ctrl[:] = ctrl
        self._data.qfrc_applied[:] = qfrc_applied
        self._data.xfrc_applied[:] = xfrc_applied
        mujoco.mj_forward(self._model, self._data)
        board_anchor_after = self._data.xpos[board_id] + self._data.xmat[board_id].reshape(3, 3) @ self._robot_board_contact_offset
        ee_anchor_after = self._data.xpos[ee_id] + self._data.xmat[ee_id].reshape(3, 3) @ anchor_ee_local
        self.get_logger().info(
            "point contact anchor synchronized: "
            f"board_local={np.round(self._robot_board_contact_offset, 4).tolist()}, "
            f"ee_local={np.round(anchor_ee_local, 4).tolist()}, "
            f"initial_error_mm={1000.0 * float(np.linalg.norm(board_anchor_after - ee_anchor_after)):.3f}"
        )

    def _apply_ctc_torques(self, ctrl: np.ndarray, *, active: bool) -> None:
        """Left arm motors: CTC τ = M(q̈_r + kdė + kpe) + h, or legacy τ = kv(q̇_r − q̇) if ctc_enable false."""
        self._ctc_support_force_z = 0.0
        if not active or len(self._ctc_act_ids) != 7:
            return
        dt = float(self._model.opt.timestep)
        if not (dt > 0.0 and math.isfinite(dt)):
            return
        with self._lock:
            qdot_r = np.array(
                [float(self._ctc_vel_ref[jn]) for jn in self._CTC_ARM_JOINTS],
                dtype=np.float64,
            )
        qdot = np.array(
            [float(self._data.qvel[vadr]) for vadr in self._ctc_vadr],
            dtype=np.float64,
        )

        if not self._ctc_enable:
            # Match former <velocity> actuator gains (robot_pr2.xml before motor switch).
            kv = np.array([35.0, 120.0, 8.0, 70.0, 30.0, 8.0, 20.0], dtype=np.float64)
            tau = kv * (qdot_r - qdot)
            for i, aid in enumerate(self._ctc_act_ids):
                lo = float(self._model.actuator_ctrlrange[aid, 0])
                hi = float(self._model.actuator_ctrlrange[aid, 1])
                ctrl[aid] = float(np.clip(tau[i], lo, hi))
            return

        # qddot_r (inertia feedforward via finite-diff) is intentionally disabled.
        # The QP outer loop runs at ~50 Hz while MuJoCo steps at 500 Hz, so any
        # step change in qdot_r produces qddot_r = Δqdot / 0.002 s → 500× amplified
        # torque spikes that violently destabilise the arm.  h_arm already feeds
        # forward gravity/Coriolis; the PD terms handle velocity tracking.
        qddot_r = np.zeros(7, dtype=np.float64)

        self._ctc_q_ref += qdot_r * dt

        q = np.array(
            [float(self._data.qpos[qadr]) for qadr in self._ctc_qadr],
            dtype=np.float64,
        )
        e = self._ctc_q_ref - q
        e_dot = qdot_r - qdot

        mujoco.mj_forward(self._model, self._data)
        mujoco.mj_fullM(self._model, self._data, self._ctc_M_buf)
        v_ix = np.asarray(self._ctc_vadr, dtype=np.int32)
        M_arm = self._ctc_M_buf[np.ix_(v_ix, v_ix)]
        h_arm = np.array(
            [float(self._data.qfrc_bias[int(v)]) + float(self._data.qfrc_passive[int(v)]) for v in self._ctc_vadr],
            dtype=np.float64,
        )
        kp = float(self._ctc_kp)
        kd = float(self._ctc_kd)
        tau = M_arm @ (kd * e_dot + kp * e) + h_arm

        # Apply only the robot endpoint force required by the board force
        # balance.  The endpoint moment is deliberately not added here: the
        # physical human/robot contact forces must generate and cancel their
        # own r×F moments.
        payload_force_z = float(self._ctc_payload_force_z)
        if self._ctc_payload_auto_balance:
            total_force_z = float(self._ctc_payload_total_force_z)
            if total_force_z <= 1.0e-9:
                # Derive the complete static board load from the loaded
                # MuJoCo model when no override is supplied.  This is a
                # model-based payload compensation, not a human/robot share.
                total_force_z = self._ctc_model_payload_force_z
                if total_force_z <= 1.0e-9:
                    total_force_z = max(payload_force_z, 0.0)
            # If the human publisher has not connected yet, the bridge must
            # still support the complete board.  Giving the robot only half
            # the load while no human force is actually applied is an
            # unbalanced startup and makes the board fall before LATCH.
            if self._hand_force_latest is None:
                payload_force_z = total_force_z
                human_force_z = 0.0
            elif float(self._data.time) < self._ctc_balance_startup_duration:
                # Once the human publisher is present, use a symmetric
                # half-load for the short hand-off window while its force
                # command settles.
                payload_force_z = 0.5 * total_force_z
                human_force_z = 0.5 * total_force_z
            else:
                human_force_z = float(self._hand_force_latest.wrench.force.z)
                # The point/ball contact can transmit the signed residual
                # reaction in either direction.  Keep the exact force balance
                # instead of clipping it to a prescribed load share.
                payload_force_z = total_force_z - human_force_z
            # The robot supplies the residual vertical force.  This is a
            # force-balance calculation, not a prescribed 50% load share:
            # F_h,z + F_r,z = m(g+a_z).  If the human/robot endpoint forces
            # happen to be equal, their opposite lever-arm moments cancel
            # naturally through the contact model.

            if (
                self._ctc_board_hold_target_z is None
                and self._board_body_id >= 0
                and float(self._data.time) >= self._ctc_balance_startup_duration
            ):
                # Use the robot grasp endpoint height rather than the board
                # COM: during the initial settling transient the board may
                # rotate about the weld, so its instantaneous COM z is not
                # yet the horizontal transport height.
                hold_body = self._ee_body_id if self._ee_body_id >= 0 else self._board_body_id
                self._ctc_board_hold_target_z = float(self._data.xpos[hold_body, 2])
                self.get_logger().info(
                    "CTC vertical hold height latched after startup: "
                    f"z={self._ctc_board_hold_target_z:.4f}m"
                )

            # Add only a bounded dynamic height correction.  It is not a
            # prescribed load share: at the nominal height and zero vertical
            # speed this term is exactly zero, leaving the automatic residual
            # force split above unchanged.  During a disturbance it provides
            # the robot's ordinary compliant support instead of relying on a
            # pure gravity feed-forward that cannot arrest a moving board.
            if (
                self._board_body_id >= 0
                and self._ctc_board_hold_target_z is not None
                and self._ctc_vertical_hold_force_limit > 0.0
            ):
                hold_body = self._ee_body_id if self._ee_body_id >= 0 else self._board_body_id
                hold_z = float(self._data.xpos[hold_body, 2])
                # The Jazzy MuJoCo Python binding used in the container does
                # not expose ``data.xvelp``.  Estimate world-frame vertical
                # speed from consecutive body positions at the fixed solver
                # timestep instead.
                if self._ctc_hold_prev_z is None:
                    hold_vz = 0.0
                else:
                    hold_vz = (hold_z - float(self._ctc_hold_prev_z)) / max(dt, 1.0e-9)
                self._ctc_hold_prev_z = hold_z
                hold_correction = (
                    self._ctc_vertical_hold_kp
                    * (float(self._ctc_board_hold_target_z) - hold_z)
                    - self._ctc_vertical_hold_kd * hold_vz
                )
                hold_correction = float(
                    np.clip(
                        hold_correction,
                        -self._ctc_vertical_hold_force_limit,
                        self._ctc_vertical_hold_force_limit,
                    )
                )
                payload_force_z += hold_correction

        self._ctc_support_force_z = float(payload_force_z)

        if abs(payload_force_z) > 1.0e-9 and self._ee_body_id >= 0:
            jacp = np.zeros((3, self._model.nv), dtype=np.float64)
            jacr = np.zeros((3, self._model.nv), dtype=np.float64)
            mujoco.mj_jacBody(
                self._model,
                self._data,
                jacp,
                jacr,
                int(self._ee_body_id),
            )
            support_force = np.array(
                [0.0, 0.0, payload_force_z], dtype=np.float64
            )
            tau += jacp[:, self._ctc_vadr].T @ support_force

        # #region agent log
        _now_ms = int(time.time() * 1000)
        if not hasattr(self, "_ctc_dbg_last_ms"):
            self._ctc_dbg_last_ms = 0
        if self._debug_trace and _now_ms - self._ctc_dbg_last_ms >= 200:  # log at 5Hz
            self._ctc_dbg_last_ms = _now_ms
            import json as _json
            # Check torque clipping for all joints
            tau_clipped = []
            for i, aid in enumerate(self._ctc_act_ids):
                lo = float(self._model.actuator_ctrlrange[aid, 0])
                hi = float(self._model.actuator_ctrlrange[aid, 1])
                tau_clipped.append(float(np.clip(tau[i], lo, hi)))
            saturated = [abs(float(tau[i]) - float(tau_clipped[i])) > 0.1 for i in range(len(tau))]
            # Tracking ratio: qdot / qdot_r (when qdot_r is non-trivial)
            track_ratio = [float(qdot[i]) / float(qdot_r[i]) if abs(float(qdot_r[i])) > 0.005 else float("nan")
                           for i in range(len(qdot_r))]
            _payload = {
                "sessionId": "a24b67", "runId": "wbc_tracking",
                "hypothesisId": "H_WBCTracking",
                "location": "pr2_sim_ros.py:_apply_ctc_torques",
                "message": "CTC tracking all joints",
                "data": {
                    "qdot_r": [float(x) for x in qdot_r],
                    "qdot": [float(x) for x in qdot],
                    "track_ratio": track_ratio,
                    "tau_computed": [float(x) for x in tau],
                    "tau_clipped": tau_clipped,
                    "any_saturated": any(saturated),
                    "saturated": saturated,
                    "e_dot": [float(x) for x in e_dot],
                    "h_arm": [float(x) for x in h_arm],
                    "kp": kp, "kd": kd,
                },
                "timestamp": _now_ms,
            }
            try:
                with open(self._dbg_log_path, "a") as _f:
                    _f.write(_json.dumps(_payload) + "\n")
            except Exception:
                pass
        # #endregion agent log

        for i, aid in enumerate(self._ctc_act_ids):
            lo = float(self._model.actuator_ctrlrange[aid, 0])
            hi = float(self._model.actuator_ctrlrange[aid, 1])
            ctrl[aid] = float(np.clip(tau[i], lo, hi))

        self._ctc_qdot_r_prev[:] = qdot_r
        self._ctc_vel_prev_inited = True

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
                    elif kind == "effort" and jn in self._ctc_joint_set:
                        if i < len(msg.velocity):
                            v = msg.velocity[i]
                            if not math.isnan(v):
                                self._ctc_vel_ref[jn] = float(v)
                        elif i < len(msg.effort) and not self._ctc_enable:
                            v = msg.effort[i]
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

    def _cb_hand_force(self, msg: WrenchStamped) -> None:
        """Receive virtual human force to apply directly to the board body."""
        self._hand_force_latest = msg

    def _apply_hand_force_to_board(self) -> None:
        """Apply latest hand force to the board body via xfrc_applied.

        The force is specified in the world (odom) frame at the far end of the
        board (the "hand" end, +1.0 m in board-local X from COM).  We convert
        it to a wrench at the board COM (force + resulting torque) and write
        xfrc_applied.
        """
        if (not self._hand_force_enable
                or self._hand_force_latest is None
                or self._board_body_id < 0):
            return

        msg = self._hand_force_latest
        F = np.array([
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
        ], dtype=np.float64)

        # Hand offset from board COM in board-local frame.
        r_local = np.array(
            list(self.get_parameter("hand_force_offset").value),
            dtype=np.float64,
        )

        # Get board COM position and rotation matrix (world frame)
        bid = self._board_body_id
        board_pos = self._data.xpos[bid].copy()
        board_mat = self._data.xmat[bid].reshape(3, 3).copy()

        # Offset in world frame
        r_world = board_mat @ r_local

        # Torque at COM: r × F plus any explicit hand torque.  The latter is
        # required for a rigid human grasp and for orientation PID control.
        tau_explicit = np.array([
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ], dtype=np.float64)
        tau = np.cross(r_world, F) + tau_explicit
        if bool(self.get_parameter("hand_force_cancel_moment").value):
            tau = tau_explicit

        # Write xfrc_applied (6D: force + torque at COM)
        self._data.xfrc_applied[bid, :] = np.concatenate([F, tau])

    def _apply_cmd_vel_to_ctrl(self, target: np.ndarray) -> None:
        if not self._use_cmd_vel:
            return

        def _wrap_to_pi(a: float) -> float:
            # Map angle to (-pi, pi]
            a = (a + math.pi) % (2.0 * math.pi) - math.pi
            return a

        def _angle_diff(a: float, b: float) -> float:
            # Shortest signed angle difference a - b in (-pi, pi]
            return _wrap_to_pi(a - b)

        vx = float(self._twist.linear.x)
        vy = float(self._twist.linear.y)
        wz = float(self._twist.angular.z)
        # If cmd_vel is (near) zero, actively stop wheels.
        # Previously we returned early and kept the last wheel command, which can look like
        # "base moving randomly" when higher-level control goes back to zero cmd_vel.
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            steer_ids = self._demo_steer_ids
            wheel_ids = self._demo_wheel_ids
            if len(wheel_ids) >= 8:
                for aid in wheel_ids[:8]:
                    if aid >= 0:
                        target[aid] = 0.0
                # #region agent log
                try:
                    now_m = time.monotonic()
                    if not hasattr(self, "_dbg_last_cmdvel_zero_mono"):
                        self._dbg_last_cmdvel_zero_mono = 0.0
                    if float(now_m - float(self._dbg_last_cmdvel_zero_mono)) > 0.5:
                        self._dbg_last_cmdvel_zero_mono = float(now_m)
                        self._dbg_write(
                            "H_CmdVelZeroStopsWheels",
                            "zero cmd_vel -> wheel vel set to 0",
                            {
                                "cmd_vel_base": [float(vx), float(vy), float(wz)],
                                "n_wheels": int(len(wheel_ids)),
                                "n_steer": int(len(steer_ids)),
                            },
                        )
                except Exception:
                    pass
                # #endregion agent log
            return

        # Low-pass cmd_vel planar components to reduce direction jitter.
        a = float(self._cmd_vel_dir_lpf_alpha)
        self._cmd_vel_vx_f = (1.0 - a) * float(self._cmd_vel_vx_f) + a * vx
        self._cmd_vel_vy_f = (1.0 - a) * float(self._cmd_vel_vy_f) + a * vy
        vx_f = float(self._cmd_vel_vx_f)
        vy_f = float(self._cmd_vel_vy_f)

        vplanar = math.hypot(vx_f, vy_f)
        steer_ids = self._demo_steer_ids
        wheel_ids = self._demo_wheel_ids
        if len(steer_ids) < 4 or len(wheel_ids) < 8:
            return

        if vplanar > 1e-6:
            # Use wrapped steering angle to avoid discontinuities near +/-pi.
            steer_target_raw = _wrap_to_pi(float(math.atan2(vy_f, vx_f)))
            # Angle-domain low-pass on steer_target to avoid sudden direction flips.
            # Uses shortest-angle interpolation on (-pi, pi].
            steer_target_smooth = steer_target_raw
            if self._last_steer_target is not None:
                a_ang = float(self._cmd_vel_steer_target_lpf_alpha)
                d = float(_angle_diff(steer_target_raw, float(self._last_steer_target)))
                steer_target_smooth = _wrap_to_pi(
                    float((1.0 - a_ang) * float(self._last_steer_target) + a_ang * (float(self._last_steer_target) + d))
                )
            # Under very small planar speed, hold the last steering direction to avoid caster hunting.
            steer_target = steer_target_smooth
            if (
                float(vplanar) < float(self._cmd_vel_dir_hold_vmin)
                and self._last_steer_target is not None
            ):
                steer_target = float(self._last_steer_target)
                # #region agent log
                try:
                    now_m = time.monotonic()
                    if not hasattr(self, "_dbg_last_dirhold_mono"):
                        self._dbg_last_dirhold_mono = 0.0
                    if float(now_m - float(self._dbg_last_dirhold_mono)) > 0.5:
                        self._dbg_last_dirhold_mono = float(now_m)
                        self._dbg_write(
                            "H_CmdVelDirHold",
                            "hold steer_target under small vplanar",
                            {
                                "cmd_vel_in": [float(vx), float(vy), float(wz)],
                                "cmd_vel_filt": [float(vx_f), float(vy_f)],
                                "vplanar": float(vplanar),
                                "vmin": float(self._cmd_vel_dir_hold_vmin),
                                "steer_target_raw": float(steer_target_raw),
                                "steer_target_smooth": float(steer_target_smooth),
                                "steer_target_held": float(steer_target),
                            },
                        )
                except Exception:
                    pass
                # #endregion agent log
            # Store the smooth target as the "memory" to maintain continuity.
            self._last_steer_target = float(steer_target_smooth)

            # Pre-check: current steering error w.r.t. *desired target* (before rate limiting).
            # If error is huge and we are going to hard-stop wheel drive anyway, do NOT rate-limit
            # the steer command. Otherwise, the casters may take too long to align and appear
            # "never points to correct direction".
            steer_errs_to_target = []
            try:
                for qadr in self._dbg_base_steer_qadr:
                    steer_act = float(self._data.qpos[qadr])
                    steer_errs_to_target.append(float(_angle_diff(steer_act, steer_target)))
            except Exception:
                steer_errs_to_target = []
            steer_err_to_target_max_abs = float(
                max((abs(e) for e in steer_errs_to_target), default=0.0)
            )

            # Rate-limit steer command to avoid impossible instant flips (caster steering lag root-cause).
            now_m = time.monotonic()
            steer = steer_target
            steer_rate_used = float(max(0.0, self._steer_rate_limit))
            steer_jump = float("nan")
            steer_rate_limit_bypassed = False
            start = float(max(1e-6, self._steer_gate_err_start))
            full = float(max(start + 1e-6, self._steer_gate_err_full))
            if bool(self._steer_gate_hard_stop) and (steer_err_to_target_max_abs >= full):
                steer_rate_limit_bypassed = True
                steer = steer_target
            # If casters are already aligned to the desired target, do not keep a lagging
            # steer command around (it would artificially create a large "error" and gate wheels).
            if steer_err_to_target_max_abs <= start:
                steer_rate_limit_bypassed = True
                steer = steer_target
            if self._last_steer_cmd is not None and self._last_steer_cmd_mono is not None:
                dt = float(max(1e-6, now_m - self._last_steer_cmd_mono))
                rate = float(max(0.0, self._steer_rate_limit))
                # If the desired steering direction jumps far, temporarily boost steer rate so
                # casters can catch up (otherwise wheels keep driving in the old direction).
                try:
                    steer_jump = float(abs(_angle_diff(steer_target, self._last_steer_cmd)))
                    if steer_jump > float(self._steer_rate_boost_err):
                        rate = float(max(rate, self._steer_rate_boost))
                except Exception:
                    pass
                steer_rate_used = float(rate)
                if not steer_rate_limit_bypassed:
                    max_step = rate * dt
                    diff = float(_angle_diff(steer_target, self._last_steer_cmd))
                    if abs(diff) > max_step:
                        steer = _wrap_to_pi(
                            float(self._last_steer_cmd + math.copysign(max_step, diff))
                        )
            self._last_steer_cmd = float(steer)
            self._last_steer_cmd_mono = float(now_m)

            v_wheel_raw = float(vplanar * self._lin_gain)

            # Steer gating should be based on actual caster alignment to the *desired target*,
            # not to a lagged steer command (which can create artificial error and stall motion).
            steer_errs = steer_errs_to_target
            steer_err_max_abs = float(steer_err_to_target_max_abs)
            k = 1.0
            if steer_err_max_abs >= full:
                k = 0.0
            elif steer_err_max_abs <= start:
                k = 1.0
            else:
                k = float(1.0 - (steer_err_max_abs - start) / (full - start))
            # Under big steer error, optionally hard-stop wheel drive until casters align.
            if (not bool(self._steer_gate_hard_stop)) or (steer_err_max_abs < full):
                kmin = float(min(1.0, max(0.0, self._steer_gate_k_min)))
                if k < kmin:
                    k = kmin
            v_wheel = float(v_wheel_raw * k)
            for sid in steer_ids:
                target[sid] = steer
            for wid in wheel_ids:
                target[wid] = v_wheel

            # #region agent log
            try:
                # base_link world yaw from MuJoCo body quat (wxyz)
                if self._body_base >= 0:
                    quat = np.asarray(self._data.xquat[self._body_base, :], dtype=np.float64)
                    qw, qx, qy, qz = (float(quat[j]) for j in range(4))
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    yaw = float(math.atan2(siny_cosp, cosy_cosp))
                else:
                    yaw = float("nan")
                period = 0.1 if float(vplanar) > 0.05 else 0.5
                if now_m - self._dbg_last_cmdvel_mono > period:
                    self._dbg_last_cmdvel_mono = now_m
                    wheel_errs = []
                    for vadr in self._dbg_base_wheel_vadr:
                        try:
                            wheel_errs.append(float(self._data.qvel[vadr]) - float(v_wheel))
                        except Exception:
                            pass
                    steer_acts = []
                    try:
                        for qadr in self._dbg_base_steer_qadr:
                            steer_acts.append(float(self._data.qpos[qadr]))
                    except Exception:
                        steer_acts = []
                    self._dbg_write(
                        "H_SimCmdVelFrame",
                        "apply_cmd_vel_to_ctrl planar",
                        {
                            "cmd_vel_base": [float(vx), float(vy), float(wz)],
                            "cmd_vel_filt_xy": [float(vx_f), float(vy_f)],
                            "vplanar": float(vplanar),
                            "steer_atan2_vy_vx": float(steer),
                            "base_yaw_world": float(yaw),
                            "lin_gain": float(self._lin_gain),
                            "ang_gain": float(self._ang_gain),
                            "v_wheel_cmd_raw": float(v_wheel_raw),
                            "v_wheel_cmd": float(v_wheel),
                            "steer_gate_err_start_rad": float(self._steer_gate_err_start),
                            "steer_gate_err_full_rad": float(self._steer_gate_err_full),
                            "steer_gate_k_min": float(self._steer_gate_k_min),
                            "steer_gate_k": float(k),
                            "steer_gate_hard_stop": bool(self._steer_gate_hard_stop),
                            "steer_rate_limit_rad_s": float(self._steer_rate_limit),
                            "steer_rate_used_rad_s": float(steer_rate_used),
                            "steer_rate_limit_bypassed": bool(steer_rate_limit_bypassed),
                            "steer_jump_abs_rad": float(steer_jump),
                            "steer_rate_boost_rad_s": float(self._steer_rate_boost),
                            "steer_rate_boost_err_rad": float(self._steer_rate_boost_err),
                            "steer_target": float(steer_target),
                            "steer_target_raw": float(steer_target_raw),
                            "steer_target_smooth": float(steer_target_smooth),
                            "steer_errs_to_target": steer_errs_to_target,
                            "steer_err_to_target_max_abs": float(steer_err_to_target_max_abs),
                            "steer_act_qpos": steer_acts,
                            "steer_errs": steer_errs,
                            "steer_err_max_abs": float(steer_err_max_abs),
                            "wheel_vel_err_rms": float(
                                math.sqrt(sum((e * e for e in wheel_errs)) / max(1, len(wheel_errs)))
                            ),
                        },
                    )
            except Exception:
                pass
            # #endregion agent log
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
                dpos = (pos - self._prev_base_pos).astype(np.float64, copy=False)
                lin = dpos / dt
                odom.twist.twist.linear.x = float(lin[0])
                odom.twist.twist.linear.y = float(lin[1])
                odom.twist.twist.linear.z = float(lin[2])
                # #region agent log
                try:
                    # Compare measured planar velocity vs latest cmd_vel (both in base_link frame).
                    # Note: /odom twist here is computed in odom/world axes from position difference.
                    # We rotate it into base_link using current base yaw for apples-to-apples comparison
                    # with cmd_vel which the sim consumes as base-frame planar velocity.
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    yaw = float(math.atan2(siny_cosp, cosy_cosp))
                    cy, sy = float(math.cos(yaw)), float(math.sin(yaw))
                    vx_w, vy_w = float(lin[0]), float(lin[1])
                    # world -> base
                    vx_b = cy * vx_w + sy * vy_w
                    vy_b = -sy * vx_w + cy * vy_w
                    # last cmd_vel (already base)
                    vx_cmd = float(self._twist.linear.x) if hasattr(self, "_twist") else 0.0
                    vy_cmd = float(self._twist.linear.y) if hasattr(self, "_twist") else 0.0
                    wz_cmd = float(self._twist.angular.z) if hasattr(self, "_twist") else 0.0
                    if abs(vx_cmd) + abs(vy_cmd) + abs(wz_cmd) > 1e-6:
                        cmd_planar = float(math.hypot(vx_cmd, vy_cmd))
                        period = 0.1 if cmd_planar > 0.05 else 0.5
                        if time.monotonic() - self._dbg_last_mono > period:
                            self._dbg_last_mono = time.monotonic()
                            self._dbg_write(
                                "H_SimMeasuredVsCmd",
                                "odom twist vs cmd_vel (base frame)",
                                {
                                    "base_yaw_world": float(yaw),
                                    "cmd_vel_base": [vx_cmd, vy_cmd, wz_cmd],
                                    "odom_vel_world": [float(lin[0]), float(lin[1])],
                                    "odom_vel_base": [float(vx_b), float(vy_b)],
                                    "track_err_base": [float(vx_b - vx_cmd), float(vy_b - vy_cmd)],
                                    "cmd_planar": cmd_planar,
                                    "odom_planar": float(math.hypot(vx_b, vy_b)),
                                    "dt_sec": float(dt),
                                    "pos_delta_world": [float(dpos[0]), float(dpos[1]), float(dpos[2])],
                                    "pos_world": [float(pos[0]), float(pos[1]), float(pos[2])],
                                },
                            )
                except Exception:
                    pass
                # #endregion agent log
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
            # #region agent log
            self._dbg_write(
                "H-initpose-1",
                "Applying initial_qpos_json to MuJoCo qpos",
                {"keys": sorted(list(self._initial_qpos.keys()))},
            )
            # #endregion agent log
            for jn, value in self._initial_qpos.items():
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid >= 0:
                    qadr = int(self._model.jnt_qposadr[jid])
                    jtype = int(self._model.jnt_type[jid])
                    if isinstance(value, (list, tuple)):
                        if jtype == int(mujoco.mjtJoint.mjJNT_FREE):
                            if len(value) != 7:
                                self.get_logger().warn(
                                    f"initial_qpos_json[{jn}] 是 freejoint，需要 7 个数，已忽略"
                                )
                                continue
                            self._data.qpos[qadr : qadr + 7] = np.array(
                                [float(x) for x in value], dtype=np.float64
                            )
                        else:
                            self.get_logger().warn(
                                f"initial_qpos_json[{jn}] 不是 freejoint，不能使用数组，已忽略"
                            )
                    else:
                        self._data.qpos[qadr] = float(value)
            mujoco.mj_forward(self._model, self._data)
            self.get_logger().info("initial_qpos_json 已应用到 MuJoCo 初始状态")
            self._sync_point_contact_anchor()
            self._ctc_sync_reference_from_state()
        else:
            # #region agent log
            self._dbg_write(
                "H-initpose-1",
                "initial_qpos_json is empty; using MuJoCo defaults",
                {},
            )
            # #endregion agent log
            mujoco.mj_forward(self._model, self._data)
            self._sync_point_contact_anchor()
            self._ctc_sync_reference_from_state()
        if self._board_body_id >= 0:
            # Do not latch the height before the weld/arm has completed its
            # first startup settle: mj_forward does not project the equality
            # position, so the pre-step board z can differ from the actual
            # grasped endpoint by several decimetres.  The first CTC step
            # after the startup guard latches the measured physical height.
            self._ctc_board_hold_target_z = None
            hold_body = self._ee_body_id if self._ee_body_id >= 0 else self._board_body_id
            self._ctc_hold_prev_z = float(self._data.xpos[hold_body, 2])
        self._base_lock_qpos = None
        self._base_lock_pending = bool(
            self._lock_base_motion and self._base_free_qadr is not None
        )
        self._torso_lock_qpos = None
        self._torso_lock_pending = bool(
            self._lock_torso_motion and self._torso_qadr is not None
        )
        self._arm_lock_qpos = None
        self._arm_lock_pending = bool(
            self._lock_arm_motion and len(self._ctc_qadr) == 7
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
            if self._lock_arm_motion and len(self._ctc_qadr) == 7:
                self._arm_lock_qpos = np.array(
                    [float(self._data.qpos[q]) for q in self._ctc_qadr],
                    dtype=np.float64,
                )
                self._arm_lock_pending = False

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
                # Timed gripper release: open gripper to max at the specified sim time.
                # Must write to persistent _ctrl_target (not the local ctrl copy),
                # otherwise the gripper only opens for a single step (~2ms).
                if (
                    not self._gripper_open_done
                    and self._gripper_open_act_id >= 0
                    and self._gripper_open_time > 0.0
                    and float(self._data.time) >= self._gripper_open_time
                ):
                    with self._lock:
                        self._ctrl_target[self._gripper_open_act_id] = 0.548
                    self._gripper_open_done = True
                    self.get_logger().info(
                        f"夹爪已张开 (t={self._data.time:.2f}s)"
                    )
            self._apply_cmd_vel_to_ctrl(ctrl)
            ctc_active = (not override) and (not self._demo_motion) and (len(self._ctc_act_ids) == 7)
            self._apply_ctc_torques(ctrl, active=ctc_active)
            support_msg = Float64()
            support_msg.data = float(self._ctc_support_force_z)
            self._pub_robot_support_force.publish(support_msg)

            self._data.ctrl[:] = ctrl
            if not self._pause_sim:
                self._apply_hand_force_to_board()
                if (
                    self._base_passive_damping > 0.0
                    and self._base_free_qadr is not None
                ):
                    # Keep the base dynamically free, but model a modest
                    # rolling-resistance force/torque from the wheels and
                    # floor.  This prevents the board reaction from
                    # accelerating the complete PR2 without kinematically
                    # freezing its pose.
                    v0 = int(self._base_free_qadr)
                    # ``qfrc_applied`` is persistent user input in MuJoCo;
                    # using ``+=`` here would accumulate the previous frame's
                    # damping force and eventually create an artificial
                    # feedback/oscillation.  Overwrite the six free-joint
                    # components with the instantaneous viscous force.
                    self._data.qfrc_applied[v0 : v0 + 6] = (
                        -self._base_passive_damping
                        * self._data.qvel[v0 : v0 + 6]
                    )
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
                    if self._arm_lock_pending and len(self._ctc_qadr) == 7:
                        self._arm_lock_qpos = np.array(
                            [float(self._data.qpos[q]) for q in self._ctc_qadr],
                            dtype=np.float64,
                        )
                        self._arm_lock_pending = False
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
            if (
                self._lock_arm_motion
                and self._arm_lock_qpos is not None
                and len(self._ctc_qadr) == 7
                and len(self._ctc_vadr) == 7
            ):
                for qadr, vadr, q_hold in zip(
                    self._ctc_qadr, self._ctc_vadr, self._arm_lock_qpos
                ):
                    self._data.qpos[qadr] = float(q_hold)
                    self._data.qvel[vadr] = 0.0
                needs_forward = True
            if needs_forward:
                mujoco.mj_forward(self._model, self._data)

            # #region agent log
            now_m = time.monotonic()
            # Increase debug sampling rate so we can see when/why joints don't track vcmd.
            if now_m - self._dbg_last_mono > 0.1:
                self._dbg_last_mono = now_m
                # For each arm joint: read commanded velocity target (from ctrl) for velocity actuators,
                # actual qvel, and generalized forces (bias/actuator).
                jdbg = {}
                for jn in self._dbg_arm_joints:
                    vadr = self._dbg_joint_vadr.get(jn, None)
                    if vadr is None:
                        continue
                    qadr = self._dbg_joint_qadr.get(jn, None)
                    qpos = float(self._data.qpos[qadr]) if qadr is not None else None
                    qrange = self._dbg_joint_range.get(jn, None)
                    qpos_to_min = (qpos - float(qrange[0])) if (qpos is not None and qrange is not None) else None
                    qpos_to_max = (float(qrange[1]) - qpos) if (qpos is not None and qrange is not None) else None
                    # commanded target: first velocity actuator mapped to this joint (if any)
                    vcmd = None
                    fr = None
                    act_ids = []
                    act_kinds = []
                    ctrl_vals = []
                    for aid, kind in self._joint_to_act.get(jn, []):
                        act_ids.append(int(aid))
                        act_kinds.append(str(kind))
                        try:
                            ctrl_vals.append(float(self._data.ctrl[aid]))
                        except Exception:
                            ctrl_vals.append(float("nan"))
                        if kind == "velocity":
                            vcmd = float(self._data.ctrl[aid])
                            try:
                                fr = [float(self._model.actuator_forcerange[aid][0]), float(self._model.actuator_forcerange[aid][1])]
                            except Exception:
                                fr = None
                            break
                    if vcmd is None and jn in self._ctc_joint_set:
                        with self._lock:
                            vcmd = float(self._ctc_vel_ref.get(jn, 0.0))
                        if fr is None:
                            for aid, kind in self._joint_to_act.get(jn, []):
                                if kind == "effort":
                                    try:
                                        fr = [
                                            float(self._model.actuator_forcerange[aid][0]),
                                            float(self._model.actuator_forcerange[aid][1]),
                                        ]
                                    except Exception:
                                        cr = self._model.actuator_ctrlrange[aid]
                                        fr = [float(cr[0]), float(cr[1])]
                                    break
                    act = float(self._data.qfrc_actuator[vadr])
                    qcon = float(self._data.qfrc_constraint[vadr])
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
                        "qpos": qpos,
                        "qrange": [float(qrange[0]), float(qrange[1])] if qrange is not None else None,
                        "qpos_to_min": qpos_to_min,
                        "qpos_to_max": qpos_to_max,
                        "bias": float(self._data.qfrc_bias[vadr]),
                        "act": act,
                        "constraint": qcon,
                        "forcerange": fr,
                        "v_err": (float(vcmd) - float(self._data.qvel[vadr])) if vcmd is not None else None,
                        "actuator_ids": act_ids,
                        "actuator_kinds": act_kinds,
                        "ctrl_vals": ctrl_vals,
                        "sat": sat,
                        "sat_margin": sat_margin,
                        "act_sign_flip": flip,
                    }
                self._dbg_write(
                    "H3_ActuatorOrMappingLimits",
                    "sim step arm velocity/forces",
                    {
                        "sim_time": float(self._data.time),
                        "nu": int(self._model.nu),
                        "ctrl_target_head": [float(x) for x in self._ctrl_target[: min(12, self._ctrl_target.shape[0])]],
                        "ctrl_data_head": [float(x) for x in self._data.ctrl[: min(12, self._data.ctrl.shape[0])]],
                        "joints": jdbg,
                    },
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
            sim_time_msg = Float64()
            sim_time_msg.data = float(self._data.time)
            self._pub_sim_time.publish(sim_time_msg)
            self._publish_board_grasped()
            self._publish_board_and_wrist_state(stamp)

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
                    viewer.cam.lookat[:] = np.asarray(
                        self.get_parameter("viewer_lookat").value,
                        dtype=np.float64,
                    )
                    viewer.cam.distance = float(
                        self.get_parameter("viewer_distance").value
                    )
                    viewer.cam.azimuth = float(
                        self.get_parameter("viewer_azimuth").value
                    )
                    viewer.cam.elevation = float(
                        self.get_parameter("viewer_elevation").value
                    )
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
