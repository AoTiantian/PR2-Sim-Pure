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

依赖: pip install mujoco（不在 rosdep 中）
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
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


@dataclass(frozen=True)
class CoordinatedDemoProfile:
    """Smooth scripted whole-body motion for the visible demo.

    The sign convention follows the existing PR2 MuJoCo actuator setup in this
    branch: positive wheel velocity drives the robot toward the table in the
    scene, while negative shoulder/elbow torques lift/fold the left arm into a
    pre-grasp posture.
    """

    phase: str
    base_forward_speed: float
    steer_angle: float
    arm_lift_fraction: float
    shoulder_pan_torque: float
    shoulder_lift_torque: float
    elbow_flex_torque: float
    wrist_flex_torque: float
    gripper_command: float


def _smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def coordinated_demo_profile(t: float, gripper_max: float = 0.5) -> CoordinatedDemoProfile:
    """Return a coordinated approach/pre-grasp/retreat profile.

    Cycle timeline (16 s):
      0-8 s   approach the table while lifting/folding the left arm;
      8-10 s  pause in a grasp-ready pose and close the gripper;
      10-14 s retreat while keeping the arm high;
      14-16 s reopen/reset for the next cycle.
    """

    cycle_t = float(t) % 16.0
    if cycle_t < 8.0:
        phase = "approach_and_raise"
        lift = _smoothstep(cycle_t / 7.0)
        base_speed = 1.45 * math.sin(math.pi * min(cycle_t, 8.0) / 8.0)
        gripper = gripper_max * (0.88 - 0.18 * lift)
    elif cycle_t < 10.0:
        phase = "pregrasp_pause"
        lift = 1.0
        base_speed = 0.0
        close = _smoothstep((cycle_t - 8.0) / 2.0)
        gripper = gripper_max * (0.70 - 0.56 * close)
    elif cycle_t < 14.0:
        phase = "retreat_with_object"
        lift = 1.0 - 0.15 * _smoothstep((cycle_t - 10.0) / 4.0)
        base_speed = -1.05 * math.sin(math.pi * (cycle_t - 10.0) / 4.0)
        gripper = gripper_max * 0.12
    else:
        phase = "reset"
        reset = _smoothstep((cycle_t - 14.0) / 2.0)
        lift = 0.85 * (1.0 - reset)
        base_speed = 0.0
        gripper = gripper_max * (0.12 + 0.76 * reset)

    return CoordinatedDemoProfile(
        phase=phase,
        base_forward_speed=base_speed,
        steer_angle=0.0,
        arm_lift_fraction=lift,
        shoulder_pan_torque=10.0 * math.sin(0.7 * cycle_t),
        shoulder_lift_torque=-14.0 - 52.0 * lift,
        elbow_flex_torque=-6.0 - 24.0 * lift,
        wrist_flex_torque=5.0 - 17.0 * lift,
        gripper_command=min(gripper_max, max(0.0, gripper)),
    )


class Pr2MujocoSim(Node):
    def __init__(self) -> None:
        super().__init__("pr2_mujoco_sim")

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
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

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
        self._odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        # 将左臂设置到非奇异初始位形（肘关节弯曲，避免完全伸直的奇异点）
        self._arm_home = {
            'l_shoulder_pan_joint':    0.40,
            'l_shoulder_lift_joint':   0.30,
            'l_upper_arm_roll_joint':  0.50,  # 0.5 gives margin from lower limit (range [0,3.9])
            'l_elbow_flex_joint':     -1.20,
            'l_forearm_roll_joint':    0.00,
            'l_wrist_flex_joint':     -0.50,
            'l_wrist_roll_joint':      0.00,
        }
        for _jn, _val in self._arm_home.items():
            _jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, _jn)
            if _jid >= 0:
                self._data.qpos[self._model.jnt_qposadr[_jid]] = _val
        mujoco.mj_forward(self._model, self._data)
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
        # Coordinated demo actuators: approach the scene table while raising the
        # left arm into a pre-grasp posture instead of exciting one joint at a
        # time. This makes the built-in demo visually communicate base/arm
        # coordination.
        self._demo_l_shoulder_pan_act = self._actuator_id_by_name("l_shoulder_pan_tau")
        self._demo_l_shoulder_lift_act = self._actuator_id_by_name(
            "l_shoulder_lift_tau"
        )
        self._demo_l_elbow_act = self._actuator_id_by_name("l_elbow_flex_tau")
        self._demo_l_wrist_flex_act = self._actuator_id_by_name("l_wrist_flex_tau")

        self.get_logger().info(
            f"已加载 MuJoCo 模型: {model_path} (nu={self._nu})"
        )
        self.get_logger().info(
            f"demo_motion={self._demo_motion} | "
            "订阅: joint_commands, actuator_command, cmd_vel, disable_actuator_override | "
            "发布: joint_states, odom+tf"
        )

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
        """Apply a coordinated table-approach/pre-grasp whole-body demo."""
        ctrl[:] = 0.0
        if self._demo_torso >= 0:
            ctrl[self._demo_torso] = 500.0

        profile = coordinated_demo_profile(t, self._demo_gripper_max)

        if self._demo_gripper_l >= 0:
            ctrl[self._demo_gripper_l] = profile.gripper_command

        for aid, torque in (
            (self._demo_l_shoulder_pan_act, profile.shoulder_pan_torque),
            (self._demo_l_shoulder_lift_act, profile.shoulder_lift_torque),
            (self._demo_l_elbow_act, profile.elbow_flex_torque),
            (self._demo_l_wrist_flex_act, profile.wrist_flex_torque),
        ):
            if aid >= 0:
                ctrl[aid] = torque

        steer_ids = self._demo_steer_ids
        wheel_ids = self._demo_wheel_ids
        if len(steer_ids) >= 4 and len(wheel_ids) >= 8:
            for sid in steer_ids:
                ctrl[sid] = profile.steer_angle
            for wid in wheel_ids:
                ctrl[wid] = profile.base_forward_speed

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
        # Re-apply arm home positions after reset (mj_resetData clears qpos to 0,
        # which puts the elbow at full extension — a kinematic singularity).
        for _jn, _val in self._arm_home.items():
            _jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, _jn)
            if _jid >= 0:
                self._data.qpos[self._model.jnt_qposadr[_jid]] = _val
        mujoco.mj_forward(self._model, self._data)

        js = JointState()
        js.header.frame_id = self._base_frame

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
            mujoco.mj_step(self._model, self._data)

            stamp = self.get_clock().now().to_msg()
            js.header.stamp = stamp
            self._fill_joint_state(js)
            self._pub_joint_states.publish(js)
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

        if self._use_viewer:
            try:
                with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
                    # Keep the recorded demo focused on the whole PR2, the desk,
                    # and the left arm workspace instead of relying on MuJoCo's
                    # default passive-viewer camera.
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                    viewer.cam.lookat[:] = np.array([-1.2, 0.0, 1.0], dtype=np.float64)
                    viewer.cam.distance = 6.0
                    viewer.cam.azimuth = 140.0
                    viewer.cam.elevation = -17.0
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
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if rclpy.ok():
            raise exc
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    main()
