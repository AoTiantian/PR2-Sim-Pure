#!/usr/bin/env python3
"""Cartesian admittance controller for PR2 left arm — velocity-based (1st-order).

框图实现：外环一阶导纳滤波器
  B_d·Δẋ + K_d·Δx = F_ext
  → Δẋ = B_d⁻¹(F_ext - K_d·Δx)
  → ẋ_cmd = ẋ_d + Δẋ  (ẋ_d 默认为零)

输出 TwistStamped（笛卡尔速度指令），由下游 IK 节点通过雅可比逆映射到关节速度。
Δx 由订阅的 ee_pose 与锁定基准位姿之差计算（FK 反馈）。
"""

from __future__ import annotations

import math
from copy import deepcopy
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _validate_len3(node: Node, name: str, default: List[float]) -> List[float]:
    raw = list(node.get_parameter(name).value)
    if len(raw) != 3:
        node.get_logger().warn(f"parameter '{name}' must have len=3, fallback to {default}")
        return list(default)
    return [float(x) for x in raw]


def _apply_deadzone(v: float, dz: float) -> float:
    if abs(v) <= dz:
        return 0.0
    return (v - dz) if v > 0.0 else (v + dz)


def _quat_mul_xyzw(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Hamilton product, both quaternions as [x, y, z, w]."""
    x1, y1, z1, w1 = (float(q[i]) for i in range(4))
    x2, y2, z2, w2 = (float(r[i]) for i in range(4))
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def _quat_conj_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


def _relative_rot_angle_rad(q_prev_xyzw: np.ndarray, q_cur_xyzw: np.ndarray) -> float:
    """Rotation angle (rad) from q_prev to q_cur (shortest path)."""
    q0 = _quat_normalize(q_prev_xyzw)
    q1 = _quat_normalize(q_cur_xyzw)
    if float(np.dot(q0, q1)) < 0.0:
        q1 = -q1
    q_rel = _quat_normalize(_quat_mul_xyzw(_quat_conj_xyzw(q0), q1))
    v = q_rel[:3]
    w = float(q_rel[3])
    vmag = float(np.linalg.norm(v))
    return float(2.0 * math.atan2(vmag, max(abs(w), 1e-12)))


class Pr2ArmAdmittanceController(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_admittance_controller")

        self.declare_parameter("input_wrench_topic", "wbc/arm_external_wrench")
        self.declare_parameter("input_pose_topic", "ee_pose")
        self.declare_parameter("output_twist_topic", "arm_cartesian_velocity")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("wrench_lpf_alpha", 0.2)

        self.declare_parameter("damping_linear", [75.0, 75.0, 95.0])
        self.declare_parameter("stiffness_linear", [160.0, 160.0, 220.0])
        self.declare_parameter("force_deadzone", [0.8, 0.8, 0.8])
        self.declare_parameter("max_linear_velocity", [0.25, 0.25, 0.20])
        self.declare_parameter("max_linear_displacement", [0.08, 0.08, 0.08])

        self.declare_parameter("damping_angular", [12.0, 12.0, 10.0])
        self.declare_parameter("stiffness_angular", [20.0, 20.0, 12.0])
        self.declare_parameter("torque_deadzone", [0.08, 0.08, 0.08])
        self.declare_parameter("max_angular_velocity", [0.35, 0.35, 0.45])
        self.declare_parameter("max_angular_displacement", [0.25, 0.25, 0.35])

        self.declare_parameter("freeze_orientation", True)
        self.declare_parameter("target_frame_id", "world")
        self.declare_parameter("enable_wrench_frame_transform", True)
        self.declare_parameter("strict_frame_consistency", True)
        self.declare_parameter("suppress_output_on_tf_failure", True)
        self.declare_parameter("base_pose_latch_delay_sec", 1.5)
        # After min delay: latch only when EE linear/angular speed stays below thresholds
        # for latch_stable_duration_sec (or force latch after max_wait).
        self.declare_parameter("base_pose_latch_max_wait_sec", 30.0)
        self.declare_parameter("ee_linear_speed_thresh", 0.008)
        self.declare_parameter("ee_angular_speed_thresh", 0.12)
        self.declare_parameter("latch_stable_duration_sec", 0.35)
        self.declare_parameter("ee_speed_estimate_lpf_alpha", 0.35)
        self.declare_parameter("base_pose_latched_topic", "base_pose_latched")

        self._in_wrench_topic = str(self.get_parameter("input_wrench_topic").value)
        self._in_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self._out_twist_topic = str(self.get_parameter("output_twist_topic").value)
        self._target_frame_id = str(self.get_parameter("target_frame_id").value)
        self._freeze_orientation = bool(self.get_parameter("freeze_orientation").value)
        self._enable_tf = bool(self.get_parameter("enable_wrench_frame_transform").value)
        self._strict_frame_consistency = bool(self.get_parameter("strict_frame_consistency").value)
        self._suppress_output_on_tf_failure = bool(self.get_parameter("suppress_output_on_tf_failure").value)
        self._base_pose_latch_delay_sec = float(self.get_parameter("base_pose_latch_delay_sec").value)
        self._base_pose_latch_max_wait_sec = max(
            float(self.get_parameter("base_pose_latch_max_wait_sec").value),
            self._base_pose_latch_delay_sec,
        )
        self._ee_lin_speed_thresh = max(float(self.get_parameter("ee_linear_speed_thresh").value), 0.0)
        self._ee_ang_speed_thresh = max(float(self.get_parameter("ee_angular_speed_thresh").value), 0.0)
        self._latch_stable_duration_sec = max(
            float(self.get_parameter("latch_stable_duration_sec").value), 0.0
        )
        self._ee_speed_lpf_alpha = _clamp(
            float(self.get_parameter("ee_speed_estimate_lpf_alpha").value), 0.0, 1.0
        )
        self._base_pose_latched_topic = str(
            self.get_parameter("base_pose_latched_topic").value
        ).strip()

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._alpha = _clamp(float(self.get_parameter("wrench_lpf_alpha").value), 0.0, 1.0)

        self._b_lin = _validate_len3(self, "damping_linear", [75.0, 75.0, 95.0])
        self._k_lin = _validate_len3(self, "stiffness_linear", [160.0, 160.0, 220.0])
        self._dz_f = _validate_len3(self, "force_deadzone", [0.8, 0.8, 0.8])
        self._max_lin_vel = _validate_len3(self, "max_linear_velocity", [0.25, 0.25, 0.20])
        self._max_lin_pos = _validate_len3(self, "max_linear_displacement", [0.08, 0.08, 0.08])

        self._b_ang = _validate_len3(self, "damping_angular", [12.0, 12.0, 10.0])
        self._k_ang = _validate_len3(self, "stiffness_angular", [20.0, 20.0, 12.0])
        self._dz_t = _validate_len3(self, "torque_deadzone", [0.08, 0.08, 0.08])
        self._max_ang_vel = _validate_len3(self, "max_angular_velocity", [0.35, 0.35, 0.45])
        self._max_ang_pos = _validate_len3(self, "max_angular_displacement", [0.25, 0.25, 0.35])

        # 位移积分状态 Δx（笛卡尔空间，相对基准位姿）
        self._dx_lin = np.zeros(3, dtype=np.float64)
        self._dx_ang = np.zeros(3, dtype=np.float64)

        self._wrench_raw = np.zeros(6, dtype=np.float64)
        self._wrench_filtered = np.zeros(6, dtype=np.float64)
        self._wrench_frame = ""
        self._last_tf_warn_ns = 0
        self._last_frame_warn_ns = 0
        self._clock_start_s = self.get_clock().now().nanoseconds * 1e-9

        self._base_pose: PoseStamped | None = None
        self._latest_pose: PoseStamped | None = None

        # Base-pose latch speed gate (timer-rate samples of ee_pose)
        self._latch_prev_pos: np.ndarray | None = None
        self._latch_prev_quat_xyzw: np.ndarray | None = None
        self._latch_prev_t: float | None = None
        self._ee_lin_speed_lpf = 0.0
        self._ee_ang_speed_lpf = 0.0
        self._latch_stable_accum_sec = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_twist = self.create_publisher(TwistStamped, self._out_twist_topic, 10)
        self._pub_base_latched = (
            self.create_publisher(Bool, self._base_pose_latched_topic, 10)
            if self._base_pose_latched_topic
            else None
        )
        self.create_subscription(WrenchStamped, self._in_wrench_topic, self._on_wrench, 20)
        self.create_subscription(PoseStamped, self._in_pose_topic, self._on_pose, 20)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "arm admittance (velocity-based) ready: "
            f"{self._in_wrench_topic}+{self._in_pose_topic} -> {self._out_twist_topic}, "
            f"rate={rate_hz:.1f}Hz, target_frame={self._target_frame_id}, "
            f"base_latch(min_wait={self._base_pose_latch_delay_sec:.2f}s, "
            f"stable={self._latch_stable_duration_sec:.2f}s, "
            f"v_lin<{self._ee_lin_speed_thresh:.4f}m/s, max_wait={self._base_pose_latch_max_wait_sec:.1f}s)"
        )

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._wrench_raw[0] = float(msg.wrench.force.x)
        self._wrench_raw[1] = float(msg.wrench.force.y)
        self._wrench_raw[2] = float(msg.wrench.force.z)
        self._wrench_raw[3] = float(msg.wrench.torque.x)
        self._wrench_raw[4] = float(msg.wrench.torque.y)
        self._wrench_raw[5] = float(msg.wrench.torque.z)
        self._wrench_frame = str(msg.header.frame_id)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _update_latch_speed_estimates(self, t: float) -> tuple[float, float]:
        """Return (linear_speed_lpf, angular_speed_lpf) from ee_pose finite differences."""
        if self._latest_pose is None:
            return self._ee_lin_speed_lpf, self._ee_ang_speed_lpf

        p = self._latest_pose.pose.position
        o = self._latest_pose.pose.orientation
        pos = np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64)
        quat = np.array([float(o.x), float(o.y), float(o.z), float(o.w)], dtype=np.float64)

        if self._latch_prev_t is None:
            self._latch_prev_pos = pos.copy()
            self._latch_prev_quat_xyzw = quat.copy()
            self._latch_prev_t = t
            return 1e6, 1e6

        dt = float(t - self._latch_prev_t)
        if dt < 1e-6:
            return self._ee_lin_speed_lpf, self._ee_ang_speed_lpf

        v_inst = float(np.linalg.norm(pos - self._latch_prev_pos) / dt)
        ang_inst = (
            _relative_rot_angle_rad(self._latch_prev_quat_xyzw, quat) / dt
            if self._latch_prev_quat_xyzw is not None
            else 0.0
        )

        a = self._ee_speed_lpf_alpha
        self._ee_lin_speed_lpf = a * v_inst + (1.0 - a) * self._ee_lin_speed_lpf
        self._ee_ang_speed_lpf = a * ang_inst + (1.0 - a) * self._ee_ang_speed_lpf

        self._latch_prev_pos = pos.copy()
        self._latch_prev_quat_xyzw = quat.copy()
        self._latch_prev_t = t
        return self._ee_lin_speed_lpf, self._ee_ang_speed_lpf

    def _latch_base_pose(self, forced: bool) -> None:
        assert self._latest_pose is not None
        self._base_pose = deepcopy(self._latest_pose)
        self._dx_lin[:] = 0.0
        self._dx_ang[:] = 0.0
        if forced:
            self.get_logger().warn(
                "base_pose latched by timeout (max_wait): EE may not have been fully settled"
            )
        else:
            self.get_logger().info("base_pose latched after EE settled (speed gate passed)")
        if self._pub_base_latched is not None:
            self._pub_base_latched.publish(Bool(data=True))

    @staticmethod
    def _quat_to_rot_xyzw(x: float, y: float, z: float, w: float) -> np.ndarray:
        xx, yy, zz = x * x, y * y, z * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (xx + zz), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float64)

    def _transform_wrench_to_target(self, wr: np.ndarray) -> tuple[np.ndarray, bool]:
        if not self._enable_tf:
            return wr, True
        src = self._wrench_frame
        dst = self._target_frame_id
        if not src or not dst or src == dst:
            return wr, True
        try:
            tf_msg = self._tf_buffer.lookup_transform(dst, src, Time())
            q = tf_msg.transform.rotation
            rot = self._quat_to_rot_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
            out = wr.copy()
            out[:3] = rot @ out[:3]
            out[3:] = rot @ out[3:]
            return out, True
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"wrench tf transform failed ({src}->{dst}): {exc}"
                )
                self._last_tf_warn_ns = now_ns
            return wr, False

    def _tick(self) -> None:
        if self._latest_pose is None:
            return

        pose_frame = str(self._latest_pose.header.frame_id)
        if (
            self._strict_frame_consistency
            and pose_frame
            and self._target_frame_id
            and pose_frame != self._target_frame_id
        ):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_frame_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"ee_pose frame mismatch: pose={pose_frame}, target={self._target_frame_id}"
                )
                self._last_frame_warn_ns = now_ns
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        if self._base_pose is None:
            if (t - self._clock_start_s) < self._base_pose_latch_delay_sec:
                return
            lin_spd, ang_spd = self._update_latch_speed_estimates(t)
            ang_ok = self._freeze_orientation or (ang_spd < self._ee_ang_speed_thresh)
            if lin_spd < self._ee_lin_speed_thresh and ang_ok:
                self._latch_stable_accum_sec += self._dt
            else:
                self._latch_stable_accum_sec = 0.0

            elapsed = t - self._clock_start_s
            force_timeout = elapsed >= self._base_pose_latch_max_wait_sec
            if self._latch_stable_accum_sec >= self._latch_stable_duration_sec or force_timeout:
                forced = force_timeout and (
                    self._latch_stable_accum_sec < self._latch_stable_duration_sec
                )
                self._latch_base_pose(forced=forced)
            return

        # FK 反馈：从 ee_pose 计算当前 Δx（相对基准）
        cur = self._latest_pose.pose
        base = self._base_pose.pose
        self._dx_lin[0] = float(cur.position.x) - float(base.position.x)
        self._dx_lin[1] = float(cur.position.y) - float(base.position.y)
        self._dx_lin[2] = float(cur.position.z) - float(base.position.z)
        # 姿态误差用小角度近似（仅在非冻结模式下有意义）
        self._dx_ang[0] = float(cur.orientation.x) - float(base.orientation.x)
        self._dx_ang[1] = float(cur.orientation.y) - float(base.orientation.y)
        self._dx_ang[2] = float(cur.orientation.z) - float(base.orientation.z)

        # 力传感器 LPF
        self._wrench_filtered = (
            self._alpha * self._wrench_raw + (1.0 - self._alpha) * self._wrench_filtered
        )
        wr, tf_ok = self._transform_wrench_to_target(self._wrench_filtered.copy())
        if self._suppress_output_on_tf_failure and not tf_ok:
            return

        # 死区
        for i in range(3):
            wr[i] = _apply_deadzone(float(wr[i]), self._dz_f[i])
            wr[3 + i] = _apply_deadzone(float(wr[3 + i]), self._dz_t[i])

        # 一阶导纳：Δẋ = B_d⁻¹ · (F_ext - K_d · Δx)
        dxdot_lin = np.zeros(3, dtype=np.float64)
        dxdot_ang = np.zeros(3, dtype=np.float64)
        for i in range(3):
            b = max(self._b_lin[i], 1e-6)
            dxdot_lin[i] = (wr[i] - self._k_lin[i] * self._dx_lin[i]) / b
            dxdot_lin[i] = _clamp(dxdot_lin[i], -self._max_lin_vel[i], self._max_lin_vel[i])

            if not self._freeze_orientation:
                ba = max(self._b_ang[i], 1e-6)
                dxdot_ang[i] = (wr[3 + i] - self._k_ang[i] * self._dx_ang[i]) / ba
                dxdot_ang[i] = _clamp(dxdot_ang[i], -self._max_ang_vel[i], self._max_ang_vel[i])

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._target_frame_id or pose_frame
        out.twist.linear.x = float(dxdot_lin[0])
        out.twist.linear.y = float(dxdot_lin[1])
        out.twist.linear.z = float(dxdot_lin[2])
        out.twist.angular.x = float(dxdot_ang[0])
        out.twist.angular.y = float(dxdot_ang[1])
        out.twist.angular.z = float(dxdot_ang[2])
        self._pub_twist.publish(out)


def main() -> None:
    rclpy.init()
    node = Pr2ArmAdmittanceController()
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
