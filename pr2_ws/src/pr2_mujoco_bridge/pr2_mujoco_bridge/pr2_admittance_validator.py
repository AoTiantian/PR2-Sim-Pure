#!/usr/bin/env python3
"""Automated validator for admittance pipeline response.

It publishes a virtual wrench step and checks whether:
- admittance acceleration responds
- final cmd_vel responds
Then prints PASS/FAIL and exits.
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Accel, Twist, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class Metrics:
    max_abs_accel_x: float = 0.0
    max_abs_cmd_vx: float = 0.0
    joint_state_count: int = 0


class Pr2AdmittanceValidator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_admittance_validator")

        self.declare_parameter("wrench_topic", "wbc/external_wrench")
        self.declare_parameter("accel_topic", "wbc/base_acceleration")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("force_x", 30.0)
        self.declare_parameter("force_y", 0.0)
        self.declare_parameter("force_z", 0.0)
        self.declare_parameter("force_start_sec", 2.0)
        self.declare_parameter("duration_sec", 3.0)
        self.declare_parameter("settle_after_sec", 3.0)
        # Optional override parameters; negative means "auto-compute".
        self.declare_parameter("force_end_sec", -1.0)
        self.declare_parameter("total_duration_sec", -1.0)
        self.declare_parameter("min_peak_accel_x", 0.12)
        self.declare_parameter("min_peak_cmd_vx", 0.06)
        self.declare_parameter("min_joint_state_count", 200)
        self.declare_parameter("pub_rate_hz", 30.0)

        self._wrench_topic = str(self.get_parameter("wrench_topic").value)
        self._accel_topic = str(self.get_parameter("accel_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._js_topic = str(self.get_parameter("joint_states_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._fx = float(self.get_parameter("force_x").value)
        self._fy = float(self.get_parameter("force_y").value)
        self._fz = float(self.get_parameter("force_z").value)
        self._t_on = float(self.get_parameter("force_start_sec").value)
        duration = float(self.get_parameter("duration_sec").value)
        settle_after = float(self.get_parameter("settle_after_sec").value)
        force_end_override = float(self.get_parameter("force_end_sec").value)
        total_override = float(self.get_parameter("total_duration_sec").value)
        self._t_off = force_end_override if force_end_override >= 0.0 else (self._t_on + duration)
        self._t_total = total_override if total_override >= 0.0 else (self._t_off + settle_after)
        self._th_a = float(self.get_parameter("min_peak_accel_x").value)
        self._th_v = float(self.get_parameter("min_peak_cmd_vx").value)
        self._th_js = int(self.get_parameter("min_joint_state_count").value)
        rate_hz = float(self.get_parameter("pub_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._pub_wrench = self.create_publisher(WrenchStamped, self._wrench_topic, 10)
        self.create_subscription(Accel, self._accel_topic, self._on_accel, 20)
        self.create_subscription(Twist, self._cmd_topic, self._on_cmd, 20)
        self.create_subscription(JointState, self._js_topic, self._on_joint_states, 20)

        self._metrics = Metrics()
        self._start_t = self.get_clock().now()
        self._done = False
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            "admittance validator started: "
            f"wrench={self._wrench_topic}, accel={self._accel_topic}, cmd={self._cmd_topic}, js={self._js_topic}, "
            f"force_window=[{self._t_on:.2f}, {self._t_off:.2f}]s, total={self._t_total:.2f}s"
        )

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_t).nanoseconds * 1e-9

    def _on_accel(self, msg: Accel) -> None:
        self._metrics.max_abs_accel_x = max(
            self._metrics.max_abs_accel_x, abs(float(msg.linear.x))
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._metrics.max_abs_cmd_vx = max(
            self._metrics.max_abs_cmd_vx, abs(float(msg.linear.x))
        )

    def _on_joint_states(self, _msg: JointState) -> None:
        self._metrics.joint_state_count += 1

    def _publish_wrench(self, fx: float, fy: float, fz: float) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        self._pub_wrench.publish(msg)

    def _tick(self) -> None:
        if self._done:
            return

        t = self._elapsed()
        if self._t_on <= t <= self._t_off:
            self._publish_wrench(self._fx, self._fy, self._fz)
        else:
            self._publish_wrench(0.0, 0.0, 0.0)

        if t < self._t_total:
            return

        pass_accel = self._metrics.max_abs_accel_x >= self._th_a
        pass_cmd = self._metrics.max_abs_cmd_vx >= self._th_v
        pass_js = self._metrics.joint_state_count >= self._th_js
        passed = pass_accel and pass_cmd and pass_js

        self.get_logger().info(
            "admittance validation result: "
            f"{'PASS' if passed else 'FAIL'} | "
            f"peak|accel_x|={self._metrics.max_abs_accel_x:.4f} (th={self._th_a:.4f}), "
            f"peak|cmd_vx|={self._metrics.max_abs_cmd_vx:.4f} (th={self._th_v:.4f}), "
            f"joint_states_count={self._metrics.joint_state_count} (th={self._th_js})"
        )
        self._done = True
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = Pr2AdmittanceValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

