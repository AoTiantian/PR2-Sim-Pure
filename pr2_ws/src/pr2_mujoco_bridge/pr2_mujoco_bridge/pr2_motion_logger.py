"""
pr2_motion_logger — 将关键运动数据写成 CSV，供离线分析。

订阅话题（均可通过参数重映射）：
  ee_pose_topic      : geometry_msgs/PoseStamped  — 末端执行器实际位姿
  ik_target_topic    : geometry_msgs/PoseStamped  — IK 目标位姿（导纳控制输出）
  wrench_topic       : geometry_msgs/WrenchStamped — 施加的虚拟力/力矩
  joint_state_topic  : sensor_msgs/JointState      — 关节状态

CSV 列（按顺序）：
  wall_time_sec      系统时钟秒（float）
  ros_time_sec       ROS 时钟秒（float）
  ee_x ee_y ee_z     末端位置 (m)
  ee_qw ee_qx ee_qy ee_qz  末端姿态四元数
  tgt_x tgt_y tgt_z  IK 目标位置 (m)（未收到时为 nan）
  tgt_qw tgt_qx tgt_qy tgt_qz  IK 目标姿态
  fx fy fz tx ty tz  施加力 (N) 和力矩 (N·m)，与 WrenchStamped.header.frame_id 一致（常为 base_link）
  fx_odom fy_odom fz_odom  将线力旋转到 odom/world 水平面后的分量（与 ee_pose / QP 一致；无 odom 时与 fx..fz 相同）
  cmd_vx..cmd_wz    笛卡尔速度指令（TwistStamped）
  base_latched      导纳节点已锁定基准位姿后为 1，否则 0
  ee_speed_lin_est  由相邻日志采样估计的末端线速度范数 (m/s)（无 ee_pose 时为 nan）
  age_*_sec         自上次收到对应话题以来经过的 ROS 时间（秒）；从未收到为 nan
  stale_*           若 age > 对应阈值则为 1，否则 0（与 IK/WBC 内部超时含义对齐，阈值见参数）
  <joint_name>_pos <joint_name>_vel  各受监控关节的位置和速度

参数：
  output_path       : CSV 文件路径，默认 /workspace/logs/pr2_motion_<北京时间戳>.csv
  log_rate_hz       : 写入频率，默认 50.0
  watch_joints      : 要记录的关节名列表，默认为左臂 7 个关节 + torso
  ee_pose_topic / ik_target_topic / wrench_topic / joint_state_topic
  odom_topic / wbc_joint_ref_topic / wbc_cmd_vel_topic / state_joint_topic（空字符串表示不订阅）
  stale_thresh_*_sec  与各 age 比较的阈值（默认与 pr2_arm_admittance_validation.launch 中 IK/WBC 一致）
  qp_debug_topic      : sensor_msgs/JointState（可选）— QP 调试输出（例如 name=["qp_obj"], effort=[obj]）
"""

import csv
import math
import os
import time
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional

from zoneinfo import ZoneInfo

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, WrenchStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

def _beijing_strftime(fmt: str) -> str:
    """Wall-clock time in China Standard Time (UTC+8), for log filenames."""
    try:
        return datetime.now(ZoneInfo("Asia/Shanghai")).strftime(fmt)
    except Exception:
        return datetime.now(timezone(timedelta(hours=8))).strftime(fmt)


_DEFAULT_JOINTS = [
    "torso_lift_joint",
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_forearm_roll_joint",
    "l_wrist_flex_joint",
    "l_wrist_roll_joint",
]


class Pr2MotionLogger(Node):
    def __init__(self) -> None:
        super().__init__("pr2_motion_logger")

        # #region agent log
        self._dbg_log_path = "/workspace/.cursor/debug-a24b67.log"
        self._dbg_prev_wrench_norm: Optional[float] = None

        def _dbg_write(hypothesis_id: str, message: str, data: dict) -> None:
            try:
                import json as _json

                payload = {
                    "sessionId": "a24b67",
                    "runId": os.environ.get("DEBUG_RUN_ID", "motion_logger"),
                    "hypothesisId": hypothesis_id,
                    "location": "pr2_motion_logger.py",
                    "message": message,
                    "data": data,
                    "timestamp": int(time.time() * 1000),
                }
                with open(self._dbg_log_path, "a", encoding="utf-8") as f:
                    f.write(_json.dumps(payload, ensure_ascii=False) + "\n")
            except Exception:
                pass

        self._dbg_write = _dbg_write
        self._dbg_write("H0_DebugInit", "motion_logger init", {"pid": int(os.getpid())})
        # #endregion agent log

        self.declare_parameter("output_path", "")
        self.declare_parameter("output_prefix", "pr2_motion")
        self.declare_parameter("log_rate_hz", 50.0)
        self.declare_parameter("ee_pose_topic", "ee_pose")
        self.declare_parameter("ik_target_topic", "ik_target_pose")
        self.declare_parameter("wrench_topic", "wbc/arm_external_wrench")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("watch_joints", _DEFAULT_JOINTS)
        self.declare_parameter("cartesian_velocity_topic", "arm_cartesian_velocity")
        self.declare_parameter("base_pose_latched_topic", "base_pose_latched")
        # Optional MuJoCo dynamics topics (empty = do not subscribe)
        self.declare_parameter("mujoco_joint_bias_topic", "")
        self.declare_parameter("mujoco_joint_actuator_topic", "")
        # Optional debug topics from admittance controller (empty = do not subscribe)
        self.declare_parameter("admittance_wrench_topic", "")
        self.declare_parameter("admittance_dx_topic", "")
        # Optional debug topics from QP controller (empty = do not subscribe)
        self.declare_parameter("qp_debug_topic", "")
        # Optional extra topics for staleness (empty string = do not subscribe)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("wbc_joint_ref_topic", "wbc/reference/joint_command")
        self.declare_parameter("wbc_cmd_vel_topic", "wbc/reference/cmd_vel")
        self.declare_parameter("state_joint_topic", "state/joint_states")
        # Thresholds (sec) for stale_* flags — align with pr2_left_arm_ik / pr2_wbc_coordinator defaults
        self.declare_parameter("stale_joint_states_sec", 0.20)
        self.declare_parameter("stale_cart_vel_sec", 0.15)
        self.declare_parameter("stale_odom_sec", 0.20)
        self.declare_parameter("stale_ee_pose_sec", 0.12)
        self.declare_parameter("stale_ik_target_sec", 1.0)
        self.declare_parameter("stale_wrench_sec", 0.50)
        self.declare_parameter("stale_wbc_joint_ref_sec", 0.15)
        self.declare_parameter("stale_wbc_cmd_vel_sec", 0.15)
        self.declare_parameter("stale_state_joint_sec", 0.25)
        # Optional: align log start to a validator start signal.
        # When set (non-empty), logger will not write any rows until it receives this message.
        self.declare_parameter("validation_start_topic", "")
        # Post-run EE plot (mass-damper ODE vs logged ee_xyz); same dir as CSV, *_plot.png
        self.declare_parameter("plot_mass", [5.0, 5.0, 5.0])
        self.declare_parameter("plot_damping", [320.0, 320.0, 320.0])
        self.declare_parameter("plot_deadzone", [0.8, 0.8, 0.8])

        _raw_path = str(self.get_parameter("output_path").value).strip()
        if not _raw_path:
            ts = _beijing_strftime("%Y%m%d_%H%M%S")
            prefix = str(self.get_parameter("output_prefix").value).strip() or "pr2_motion"
            _raw_path = f"/workspace/logs/{prefix}_{ts}.csv"
        self._output_path = _raw_path
        rate_hz = float(self.get_parameter("log_rate_hz").value)
        self._watch_joints: List[str] = list(
            self.get_parameter("watch_joints").value
        )

        self._th_js = float(self.get_parameter("stale_joint_states_sec").value)
        self._th_cv = float(self.get_parameter("stale_cart_vel_sec").value)
        self._th_odom = float(self.get_parameter("stale_odom_sec").value)
        self._th_ee = float(self.get_parameter("stale_ee_pose_sec").value)
        self._th_tgt = float(self.get_parameter("stale_ik_target_sec").value)
        self._th_wr = float(self.get_parameter("stale_wrench_sec").value)
        self._th_wbc_j = float(self.get_parameter("stale_wbc_joint_ref_sec").value)
        self._th_wbc_v = float(self.get_parameter("stale_wbc_cmd_vel_sec").value)
        self._th_st_js = float(self.get_parameter("stale_state_joint_sec").value)

        self._plot_mass = [float(x) for x in self.get_parameter("plot_mass").value]
        self._plot_damping = [float(x) for x in self.get_parameter("plot_damping").value]
        self._plot_deadzone = [float(x) for x in self.get_parameter("plot_deadzone").value]

        # Latest values (updated by callbacks)
        self._ee: Optional[PoseStamped] = None
        self._tgt: Optional[PoseStamped] = None
        self._wrench = [0.0] * 6          # fx fy fz tx ty tz
        self._cart_vel = [0.0] * 6        # cmd_vx vy vz wx wy wz
        self._adm_wrench = [float("nan")] * 6   # transformed wrench in control frame
        self._adm_dx = [float("nan")] * 3       # dx in control frame
        self._qp_obj = float("nan")
        self._jpos: Dict[str, float] = {}
        self._jvel: Dict[str, float] = {}
        self._jbias: Dict[str, float] = {}
        self._jact: Dict[str, float] = {}
        self._base_latched = 0
        self._log_prev_ros_t: Optional[float] = None
        self._log_prev_ee_pos: Optional[tuple[float, float, float]] = None
        self._log_enabled = False
        self._t0_ros: Optional[float] = None

        self._t_last_joint: Optional[Time] = None
        self._t_last_cart: Optional[Time] = None
        self._t_last_ee: Optional[Time] = None
        self._t_last_tgt: Optional[Time] = None
        self._t_last_wrench: Optional[Time] = None
        self._t_last_adm_wrench: Optional[Time] = None
        self._t_last_adm_dx: Optional[Time] = None
        self._t_last_qp: Optional[Time] = None
        self._t_last_odom: Optional[Time] = None
        self._latest_odom: Optional[Odometry] = None
        self._wrench_frame_id: str = ""
        self._t_last_wbc_joint: Optional[Time] = None
        self._t_last_wbc_twist: Optional[Time] = None
        self._t_last_state_joint: Optional[Time] = None
        self._t_last_mj_bias: Optional[Time] = None
        self._t_last_mj_act: Optional[Time] = None
        self._sub_odom = False
        self._sub_wbc_j = False
        self._sub_wbc_v = False
        self._sub_st_js = False
        self._sub_ee = False
        self._sub_tgt = False
        self._sub_wrench = False
        self._sub_cart = False
        self._sub_latched = False
        self._sub_adm_wrench = False
        self._sub_adm_dx = False
        self._sub_qp = False
        self._sub_mj_bias = False
        self._sub_mj_act = False

        _ee_t = str(self.get_parameter("ee_pose_topic").value).strip()
        if _ee_t:
            self.create_subscription(PoseStamped, _ee_t, self._on_ee_pose, 10)
            self._sub_ee = True
        _tgt_t = str(self.get_parameter("ik_target_topic").value).strip()
        if _tgt_t:
            self.create_subscription(PoseStamped, _tgt_t, self._on_tgt_pose, 10)
            self._sub_tgt = True
        _wr_t = str(self.get_parameter("wrench_topic").value).strip()
        if _wr_t:
            self.create_subscription(WrenchStamped, _wr_t, self._on_wrench, 10)
            self._sub_wrench = True
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._on_joint_state,
            30,
        )
        _cv_t = str(self.get_parameter("cartesian_velocity_topic").value).strip()
        if _cv_t:
            self.create_subscription(TwistStamped, _cv_t, self._on_cart_vel, 10)
            self._sub_cart = True
        _odom_t = str(self.get_parameter("odom_topic").value).strip()
        if _odom_t:
            self.create_subscription(Odometry, _odom_t, self._on_odom, 20)
            self._sub_odom = True
        _wbc_j = str(self.get_parameter("wbc_joint_ref_topic").value).strip()
        if _wbc_j:
            self.create_subscription(JointState, _wbc_j, self._on_wbc_joint_ref, 10)
            self._sub_wbc_j = True
        _wbc_v = str(self.get_parameter("wbc_cmd_vel_topic").value).strip()
        if _wbc_v:
            self.create_subscription(Twist, _wbc_v, self._on_wbc_cmd_vel, 10)
            self._sub_wbc_v = True
        _st_js = str(self.get_parameter("state_joint_topic").value).strip()
        if _st_js:
            self.create_subscription(JointState, _st_js, self._on_state_joint, 20)
            self._sub_st_js = True

        _latched_topic = str(self.get_parameter("base_pose_latched_topic").value).strip()
        if _latched_topic:
            self.create_subscription(Bool, _latched_topic, self._on_base_latched, 10)
            self._sub_latched = True

        _aw = str(self.get_parameter("admittance_wrench_topic").value).strip()
        if _aw:
            self.create_subscription(WrenchStamped, _aw, self._on_adm_wrench, 10)
            self._sub_adm_wrench = True
        _adx = str(self.get_parameter("admittance_dx_topic").value).strip()
        if _adx:
            self.create_subscription(Vector3Stamped, _adx, self._on_adm_dx, 10)
            self._sub_adm_dx = True

        _qp = str(self.get_parameter("qp_debug_topic").value).strip()
        if _qp:
            self.create_subscription(JointState, _qp, self._on_qp_debug, 10)
            self._sub_qp = True

        _mj_bias = str(self.get_parameter("mujoco_joint_bias_topic").value).strip()
        if _mj_bias:
            self.create_subscription(JointState, _mj_bias, self._on_mujoco_bias, 10)
            self._sub_mj_bias = True
        _mj_act = str(self.get_parameter("mujoco_joint_actuator_topic").value).strip()
        if _mj_act:
            self.create_subscription(JointState, _mj_act, self._on_mujoco_actuator, 10)
            self._sub_mj_act = True

        _start_t = str(self.get_parameter("validation_start_topic").value).strip()
        if _start_t:
            self.create_subscription(Empty, _start_t, self._on_validation_start, 10)
            self.get_logger().info(f"motion_logger will start on: {_start_t}")
        else:
            self._log_enabled = True

        # Open CSV and write header
        os.makedirs(os.path.dirname(self._output_path) or ".", exist_ok=True)
        self._csv_file = open(self._output_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        header = [
            "wall_time_sec", "ros_time_sec", "t_rel_sec",
            "ee_x", "ee_y", "ee_z",
            "ee_qw", "ee_qx", "ee_qy", "ee_qz",
            "tgt_x", "tgt_y", "tgt_z",
            "tgt_qw", "tgt_qx", "tgt_qy", "tgt_qz",
            "fx", "fy", "fz", "tx", "ty", "tz",
            "fx_odom", "fy_odom", "fz_odom",
            "cmd_vx", "cmd_vy", "cmd_vz", "cmd_wx", "cmd_wy", "cmd_wz",
            "base_latched",
            "ee_speed_lin_est",
            "ee_vx_est", "ee_vy_est", "ee_vz_est",
            "age_joint_states_sec",
            "stale_joint_states",
            "age_cart_vel_sec",
            "stale_cart_vel",
            "age_odom_sec",
            "stale_odom",
            "age_ee_pose_sec",
            "stale_ee_pose",
            "age_ik_target_sec",
            "stale_ik_target",
            "age_wrench_sec",
            "stale_wrench",
            "age_wbc_joint_ref_sec",
            "stale_wbc_joint_ref",
            "age_wbc_cmd_vel_sec",
            "stale_wbc_cmd_vel",
            "age_state_joint_sec",
            "stale_state_joint",
            "adm_fx", "adm_fy", "adm_fz", "adm_tx", "adm_ty", "adm_tz",
            "adm_dx", "adm_dy", "adm_dz",
            "qp_obj",
        ]
        # MuJoCo dynamics (bias/actuator generalized forces) for watched joints
        for jn in self._watch_joints:
            header += [f"{jn}_bias", f"{jn}_act"]
        for jn in self._watch_joints:
            header += [f"{jn}_pos", f"{jn}_vel"]
        self._writer.writerow(header)
        self._csv_file.flush()

        self._row_count = 0
        self.create_timer(1.0 / max(rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"pr2_motion_logger 已启动: 输出={self._output_path}, "
            f"频率={rate_hz:.1f}Hz, 关节={self._watch_joints}, "
            f"staleness 列已启用 (odom={self._sub_odom}, wbc_j={self._sub_wbc_j}, "
            f"wbc_v={self._sub_wbc_v}, state_js={self._sub_st_js})"
        )

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        self._ee = msg
        self._t_last_ee = self.get_clock().now()

    def _on_tgt_pose(self, msg: PoseStamped) -> None:
        self._tgt = msg
        self._t_last_tgt = self.get_clock().now()

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._t_last_wrench = self.get_clock().now()
        self._wrench_frame_id = str(msg.header.frame_id)
        self._wrench = [
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ]

    def _on_cart_vel(self, msg: TwistStamped) -> None:
        self._t_last_cart = self.get_clock().now()
        self._cart_vel = [
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
            float(msg.twist.angular.x),
            float(msg.twist.angular.y),
            float(msg.twist.angular.z),
        ]

    def _on_adm_wrench(self, msg: WrenchStamped) -> None:
        self._t_last_adm_wrench = self.get_clock().now()
        self._adm_wrench = [
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ]

    def _on_adm_dx(self, msg: Vector3Stamped) -> None:
        self._t_last_adm_dx = self.get_clock().now()
        self._adm_dx = [float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]

    def _on_qp_debug(self, msg: JointState) -> None:
        self._t_last_qp = self.get_clock().now()
        for i, n in enumerate(msg.name):
            if str(n) == "qp_obj" and i < len(msg.effort):
                self._qp_obj = float(msg.effort[i])

    def _on_odom(self, msg: Odometry) -> None:
        self._t_last_odom = self.get_clock().now()
        self._latest_odom = msg

    @staticmethod
    def _wrench_linear_force_odom(
        fx_b: float,
        fy_b: float,
        fz: float,
        frame_id: str,
        odom: Optional[Odometry],
    ) -> tuple[float, float, float]:
        """Match pr2_qp_whole_body_admittance: base_link force -> odom xy via base yaw; fz unchanged."""
        fid = frame_id.strip()
        if fid and fid != "base_link":
            return float(fx_b), float(fy_b), float(fz)
        if odom is None:
            return float(fx_b), float(fy_b), float(fz)
        try:
            qb = odom.pose.pose.orientation
            qw, qx, qy, qz = float(qb.w), float(qb.x), float(qb.y), float(qb.z)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw_b = float(math.atan2(siny_cosp, cosy_cosp))
            cy, sy = float(math.cos(yaw_b)), float(math.sin(yaw_b))
            return cy * fx_b - sy * fy_b, sy * fx_b + cy * fy_b, float(fz)
        except Exception:
            return float(fx_b), float(fy_b), float(fz)

    def _on_wbc_joint_ref(self, _msg: JointState) -> None:
        self._t_last_wbc_joint = self.get_clock().now()

    def _on_wbc_cmd_vel(self, _msg: Twist) -> None:
        self._t_last_wbc_twist = self.get_clock().now()

    def _on_state_joint(self, _msg: JointState) -> None:
        self._t_last_state_joint = self.get_clock().now()

    def _on_base_latched(self, msg: Bool) -> None:
        if bool(msg.data):
            self._base_latched = 1

    def _on_joint_state(self, msg: JointState) -> None:
        self._t_last_joint = self.get_clock().now()
        for i, name in enumerate(msg.name):
            if name in self._watch_joints:
                if i < len(msg.position):
                    self._jpos[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self._jvel[name] = float(msg.velocity[i])

    def _on_validation_start(self, _msg: Empty) -> None:
        # Start logging immediately, and reset estimators so that the first row
        # represents t_rel_sec ~= 0.
        now = self.get_clock().now()
        self._t0_ros = now.nanoseconds * 1e-9
        self._log_prev_ros_t = None
        self._log_prev_ee_pos = None
        self._log_enabled = True

    def _on_mujoco_bias(self, msg: JointState) -> None:
        self._t_last_mj_bias = self.get_clock().now()
        for i, name in enumerate(msg.name):
            if name in self._watch_joints and i < len(msg.effort):
                self._jbias[name] = float(msg.effort[i])

    def _on_mujoco_actuator(self, msg: JointState) -> None:
        self._t_last_mj_act = self.get_clock().now()
        for i, name in enumerate(msg.name):
            if name in self._watch_joints and i < len(msg.effort):
                self._jact[name] = float(msg.effort[i])

    @staticmethod
    def _age_sec(now: Time, last: Optional[Time]) -> float:
        if last is None:
            return float("nan")
        return float((now - last).nanoseconds) * 1e-9

    def _stale_cols(self, now: Time) -> List[float]:
        """Pairs (age_sec, stale_flag) aligned with CSV header order."""

        def pair(last: Optional[Time], thresh: float, enabled: bool) -> List[float]:
            if not enabled:
                return [float("nan"), float("nan")]
            age = self._age_sec(now, last)
            if math.isnan(age):
                return [float("nan"), 1.0]
            return [age, 1.0 if age > thresh else 0.0]

        out: List[float] = []
        out += pair(self._t_last_joint, self._th_js, True)
        out += pair(self._t_last_cart, self._th_cv, self._sub_cart)
        out += pair(self._t_last_odom, self._th_odom, self._sub_odom)
        out += pair(self._t_last_ee, self._th_ee, self._sub_ee)
        out += pair(self._t_last_tgt, self._th_tgt, self._sub_tgt)
        out += pair(self._t_last_wrench, self._th_wr, self._sub_wrench)
        out += pair(self._t_last_wbc_joint, self._th_wbc_j, self._sub_wbc_j)
        out += pair(self._t_last_wbc_twist, self._th_wbc_v, self._sub_wbc_v)
        out += pair(self._t_last_state_joint, self._th_st_js, self._sub_st_js)
        return out

    def _tick(self) -> None:
        if not self._log_enabled:
            return
        nan = float("nan")
        wall = time.time()
        now = self.get_clock().now()
        ros = now.nanoseconds * 1e-9
        t_rel = (ros - self._t0_ros) if self._t0_ros is not None else 0.0

        ee_speed_lin = nan
        ee_vx = nan
        ee_vy = nan
        ee_vz = nan
        if self._ee is not None:
            p = self._ee.pose.position
            q = self._ee.pose.orientation
            ee_row = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
            cur = (float(p.x), float(p.y), float(p.z))
            if self._log_prev_ros_t is not None and self._log_prev_ee_pos is not None:
                dt = ros - self._log_prev_ros_t
                if dt > 1e-6:
                    ox, oy, oz = self._log_prev_ee_pos
                    ee_speed_lin = math.hypot(cur[0] - ox, cur[1] - oy, cur[2] - oz) / dt
                    ee_vx = (cur[0] - ox) / dt
                    ee_vy = (cur[1] - oy) / dt
                    ee_vz = (cur[2] - oz) / dt
            self._log_prev_ros_t = ros
            self._log_prev_ee_pos = cur
        else:
            ee_row = [nan] * 7
            self._log_prev_ros_t = None
            self._log_prev_ee_pos = None

        if self._tgt is not None:
            p = self._tgt.pose.position
            q = self._tgt.pose.orientation
            tgt_row = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        else:
            tgt_row = [nan] * 7

        joint_row: List[float] = []
        for jn in self._watch_joints:
            joint_row.append(self._jpos.get(jn, nan))
            joint_row.append(self._jvel.get(jn, nan))

        dyn_row: List[float] = []
        for jn in self._watch_joints:
            dyn_row.append(self._jbias.get(jn, nan))
            dyn_row.append(self._jact.get(jn, nan))

        stale_extra = self._stale_cols(now)
        extra = [float(self._base_latched), ee_speed_lin, ee_vx, ee_vy, ee_vz] + stale_extra
        dbg = self._adm_wrench + self._adm_dx + [float(self._qp_obj)]
        fod = self._wrench_linear_force_odom(
            float(self._wrench[0]),
            float(self._wrench[1]),
            float(self._wrench[2]),
            self._wrench_frame_id,
            self._latest_odom,
        )
        row = (
            [wall, ros, t_rel]
            + ee_row
            + tgt_row
            + self._wrench
            + [fod[0], fod[1], fod[2]]
            + self._cart_vel
            + extra
            + dbg
            + dyn_row
            + joint_row
        )

        # #region agent log
        # Detect "force removed" edge as seen by logger (raw wrench), and capture whether cmd_vel is still large.
        try:
            fx, fy, fz = float(self._wrench[0]), float(self._wrench[1]), float(self._wrench[2])
            wnorm = math.sqrt(fx * fx + fy * fy + fz * fz)
            prev = self._dbg_prev_wrench_norm
            self._dbg_prev_wrench_norm = float(wnorm)
            if prev is not None:
                # Edge: large drop in raw wrench norm within one tick.
                if (prev >= 50.0) and (wnorm <= 5.0):
                    cv = self._cart_vel
                    cv_norm = math.sqrt(float(cv[0]) ** 2 + float(cv[1]) ** 2 + float(cv[2]) ** 2)
                    self._dbg_write(
                        "H1_RawWrenchDrop",
                        "raw wrench dropped; snapshot logger state",
                        {
                            "t_rel_sec": float(t_rel),
                            "ros_time_sec": float(ros),
                            "wrench_norm_prev": float(prev),
                            "wrench_norm_now": float(wnorm),
                            "wrench_xyz": [float(fx), float(fy), float(fz)],
                            "cmd_v_xyz": [float(cv[0]), float(cv[1]), float(cv[2])],
                            "cmd_v_norm": float(cv_norm),
                            "ee_xyz": [float(ee_row[0]), float(ee_row[1]), float(ee_row[2])]
                            if self._ee is not None
                            else None,
                            "age_ee_pose_sec": float(stale_extra[6]) if len(stale_extra) >= 8 else float("nan"),
                            "stale_ee_pose": float(stale_extra[7]) if len(stale_extra) >= 8 else float("nan"),
                        },
                    )
        except Exception:
            pass
        # #endregion agent log

        self._writer.writerow([f"{v:.6f}" if not math.isnan(v) else "nan" for v in row])
        self._row_count += 1

        # Flush every 50 rows to keep data safe without excessive I/O
        if self._row_count % 50 == 0:
            self._csv_file.flush()

    def destroy_node(self) -> None:
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f"pr2_motion_logger 已关闭: 共写入 {self._row_count} 行 → {self._output_path}"
        )
        if self._row_count >= 2:
            try:
                from .ee_trajectory_plot import out_png_path_from_csv, plot_from_csv

                out_png = out_png_path_from_csv(self._output_path)
                plot_from_csv(
                    self._output_path,
                    out_png,
                    self._plot_mass,
                    self._plot_damping,
                    self._plot_deadzone,
                )
                self.get_logger().info(f"轨迹对比图已保存: {out_png}")
            except Exception as e:
                self.get_logger().warning(f"轨迹对比图生成失败: {e}")
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Pr2MotionLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
