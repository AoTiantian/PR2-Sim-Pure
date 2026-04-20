#!/usr/bin/env python3
"""Cartesian admittance controller for PR2 left arm.

Subscribes external wrench and current end-effector pose, then publishes
admittance-generated target pose for IK.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
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
    if v > 0.0:
        return v - dz
    return v + dz


@dataclass
class AxisState:
    pos: float = 0.0
    vel: float = 0.0


class Pr2ArmAdmittanceController(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_admittance_controller")

        self.declare_parameter("input_wrench_topic", "wbc/arm_external_wrench")
        self.declare_parameter("input_pose_topic", "ee_pose")
        self.declare_parameter("output_target_pose_topic", "ik_target_pose")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("wrench_lpf_alpha", 0.2)

        self.declare_parameter("mass_linear", [6.0, 6.0, 8.0])
        self.declare_parameter("damping_linear", [75.0, 75.0, 95.0])
        self.declare_parameter("stiffness_linear", [160.0, 160.0, 220.0])
        self.declare_parameter("force_deadzone", [0.8, 0.8, 0.8])
        self.declare_parameter("max_linear_velocity", [0.25, 0.25, 0.20])
        self.declare_parameter("max_linear_displacement", [0.08, 0.08, 0.08])

        self.declare_parameter("mass_angular", [1.2, 1.2, 0.8])
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

        self._in_wrench_topic = str(self.get_parameter("input_wrench_topic").value)
        self._in_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self._out_pose_topic = str(self.get_parameter("output_target_pose_topic").value)
        self._target_frame_id = str(self.get_parameter("target_frame_id").value)
        self._freeze_orientation = bool(self.get_parameter("freeze_orientation").value)
        self._enable_tf = bool(self.get_parameter("enable_wrench_frame_transform").value)
        self._strict_frame_consistency = bool(
            self.get_parameter("strict_frame_consistency").value
        )
        self._suppress_output_on_tf_failure = bool(
            self.get_parameter("suppress_output_on_tf_failure").value
        )
        self._base_pose_latch_delay_sec = float(
            self.get_parameter("base_pose_latch_delay_sec").value
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._alpha = _clamp(float(self.get_parameter("wrench_lpf_alpha").value), 0.0, 1.0)

        self._m_lin = _validate_len3(self, "mass_linear", [6.0, 6.0, 8.0])
        self._d_lin = _validate_len3(self, "damping_linear", [75.0, 75.0, 95.0])
        self._k_lin = _validate_len3(self, "stiffness_linear", [160.0, 160.0, 220.0])
        self._dz_f = _validate_len3(self, "force_deadzone", [0.8, 0.8, 0.8])
        self._max_lin_vel = _validate_len3(self, "max_linear_velocity", [0.25, 0.25, 0.20])
        self._max_lin_pos = _validate_len3(self, "max_linear_displacement", [0.08, 0.08, 0.08])

        self._m_ang = _validate_len3(self, "mass_angular", [1.2, 1.2, 0.8])
        self._d_ang = _validate_len3(self, "damping_angular", [12.0, 12.0, 10.0])
        self._k_ang = _validate_len3(self, "stiffness_angular", [20.0, 20.0, 12.0])
        self._dz_t = _validate_len3(self, "torque_deadzone", [0.08, 0.08, 0.08])
        self._max_ang_vel = _validate_len3(self, "max_angular_velocity", [0.35, 0.35, 0.45])
        self._max_ang_pos = _validate_len3(self, "max_angular_displacement", [0.25, 0.25, 0.35])

        self._lin = [AxisState(), AxisState(), AxisState()]
        self._ang = [AxisState(), AxisState(), AxisState()]

        self._wrench_raw = np.zeros(6, dtype=np.float64)
        self._wrench_filtered = np.zeros(6, dtype=np.float64)
        self._wrench_frame = ""
        self._last_tf_warn_ns = 0
        self._last_frame_warn_ns = 0
        self._clock_start_s = self.get_clock().now().nanoseconds * 1e-9

        self._base_pose: PoseStamped | None = None
        self._base_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self._latest_pose: PoseStamped | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_pose = self.create_publisher(PoseStamped, self._out_pose_topic, 10)
        self.create_subscription(WrenchStamped, self._in_wrench_topic, self._on_wrench, 20)
        self.create_subscription(PoseStamped, self._in_pose_topic, self._on_pose, 20)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "arm admittance ready: "
            f"{self._in_wrench_topic}+{self._in_pose_topic} -> {self._out_pose_topic}, "
            f"rate={rate_hz:.1f}Hz, target_frame={self._target_frame_id}, tf={self._enable_tf}, "
            f"latch_delay={self._base_pose_latch_delay_sec:.2f}s, "
            f"strict_frame={self._strict_frame_consistency}"
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

    def _update_axis(self, st: AxisState, ext: float, m: float, d: float, k: float, max_v: float, max_p: float) -> None:
        acc = (ext - d * st.vel - k * st.pos) / max(m, 1e-6)
        st.vel += acc * self._dt
        st.vel = _clamp(st.vel, -max_v, max_v)
        st.pos += st.vel * self._dt
        st.pos = _clamp(st.pos, -max_p, max_p)

    @staticmethod
    def _quat_to_rot_xyzw(x: float, y: float, z: float, w: float) -> np.ndarray:
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

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
                    f"wrench tf transform failed ({src}->{dst}), fallback to raw wrench: {exc}"
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
                    "ee_pose frame mismatch: "
                    f"pose_frame={pose_frame}, target_frame={self._target_frame_id}; suppressing output"
                )
                self._last_frame_warn_ns = now_ns
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        t0 = self._clock_start_s
        if self._base_pose is None:
            if (t - t0) < self._base_pose_latch_delay_sec:
                return
            self._base_pose = self._latest_pose
            self._base_quat_xyzw = np.array(
                [
                    float(self._latest_pose.pose.orientation.x),
                    float(self._latest_pose.pose.orientation.y),
                    float(self._latest_pose.pose.orientation.z),
                    float(self._latest_pose.pose.orientation.w),
                ],
                dtype=np.float64,
            )
            return

        self._wrench_filtered = self._alpha * self._wrench_raw + (1.0 - self._alpha) * self._wrench_filtered
        wr, tf_ok = self._transform_wrench_to_target(self._wrench_filtered.copy())
        if self._suppress_output_on_tf_failure and not tf_ok:
            return
        for i in range(3):
            wr[i] = _apply_deadzone(float(wr[i]), self._dz_f[i])
            wr[3 + i] = _apply_deadzone(float(wr[3 + i]), self._dz_t[i])

        for i in range(3):
            self._update_axis(
                self._lin[i],
                float(wr[i]),
                self._m_lin[i],
                self._d_lin[i],
                self._k_lin[i],
                self._max_lin_vel[i],
                self._max_lin_pos[i],
            )
            self._update_axis(
                self._ang[i],
                float(wr[3 + i]),
                self._m_ang[i],
                self._d_ang[i],
                self._k_ang[i],
                self._max_ang_vel[i],
                self._max_ang_pos[i],
            )

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._target_frame_id or self._latest_pose.header.frame_id
        out.pose.position.x = float(self._base_pose.pose.position.x + self._lin[0].pos)
        out.pose.position.y = float(self._base_pose.pose.position.y + self._lin[1].pos)
        out.pose.position.z = float(self._base_pose.pose.position.z + self._lin[2].pos)

        if self._freeze_orientation:
            out.pose.orientation.x = float(self._base_quat_xyzw[0])
            out.pose.orientation.y = float(self._base_quat_xyzw[1])
            out.pose.orientation.z = float(self._base_quat_xyzw[2])
            out.pose.orientation.w = float(self._base_quat_xyzw[3])
        else:
            # Small-angle approximation around base orientation.
            out.pose.orientation.x = float(self._base_quat_xyzw[0] + 0.5 * self._ang[0].pos)
            out.pose.orientation.y = float(self._base_quat_xyzw[1] + 0.5 * self._ang[1].pos)
            out.pose.orientation.z = float(self._base_quat_xyzw[2] + 0.5 * self._ang[2].pos)
            out.pose.orientation.w = float(self._base_quat_xyzw[3])
            q = np.array(
                [
                    out.pose.orientation.x,
                    out.pose.orientation.y,
                    out.pose.orientation.z,
                    out.pose.orientation.w,
                ],
                dtype=np.float64,
            )
            n = np.linalg.norm(q)
            if n < 1e-9:
                q = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
            else:
                q = q / n
            out.pose.orientation.x = float(q[0])
            out.pose.orientation.y = float(q[1])
            out.pose.orientation.z = float(q[2])
            out.pose.orientation.w = float(q[3])

        self._pub_pose.publish(out)


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

