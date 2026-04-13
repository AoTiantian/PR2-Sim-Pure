#!/usr/bin/env python3
"""Admittance controller node.

输入外力/外力矩（WrenchStamped），在 6 轴上计算：
  M * a + D * v + K * x = F_ext
并发布 geometry_msgs/Accel 作为下游参考（例如给 base_accel_integrator）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Accel, WrenchStamped
from rclpy.node import Node


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _validate_len3(node: Node, name: str, default: List[float]) -> List[float]:
    raw = list(node.get_parameter(name).value)
    if len(raw) != 3:
        node.get_logger().warn(
            f"parameter '{name}' must have len=3, fallback to {default}"
        )
        return list(default)
    return [float(v) for v in raw]


def _apply_deadzone(value: float, threshold: float) -> float:
    if abs(value) <= threshold:
        return 0.0
    if value > 0.0:
        return value - threshold
    return value + threshold


@dataclass
class AxisState:
    pos: float = 0.0
    vel: float = 0.0


class Pr2AdmittanceStub(Node):
    """Backward-compatible executable name, now with real admittance dynamics."""

    def __init__(self) -> None:
        super().__init__("pr2_admittance_controller")

        self.declare_parameter("input_topic", "wbc/external_wrench")
        self.declare_parameter("output_topic", "wbc/base_acceleration")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("wrench_lpf_alpha", 0.2)

        self.declare_parameter("mass_linear", [8.0, 8.0, 12.0])
        self.declare_parameter("damping_linear", [80.0, 80.0, 120.0])
        self.declare_parameter("stiffness_linear", [0.0, 0.0, 0.0])

        self.declare_parameter("mass_angular", [1.5, 1.5, 1.0])
        self.declare_parameter("damping_angular", [18.0, 18.0, 12.0])
        self.declare_parameter("stiffness_angular", [0.0, 0.0, 0.0])

        self.declare_parameter("force_deadzone", [0.8, 0.8, 0.8])
        self.declare_parameter("torque_deadzone", [0.08, 0.08, 0.08])

        self.declare_parameter("max_linear_accel", 2.0)
        self.declare_parameter("max_angular_accel", 3.0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._alpha = float(self.get_parameter("wrench_lpf_alpha").value)
        self._alpha = _clamp(self._alpha, 0.0, 1.0)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._m_lin = _validate_len3(self, "mass_linear", [8.0, 8.0, 12.0])
        self._d_lin = _validate_len3(self, "damping_linear", [80.0, 80.0, 120.0])
        self._k_lin = _validate_len3(self, "stiffness_linear", [0.0, 0.0, 0.0])
        self._m_ang = _validate_len3(self, "mass_angular", [1.5, 1.5, 1.0])
        self._d_ang = _validate_len3(self, "damping_angular", [18.0, 18.0, 12.0])
        self._k_ang = _validate_len3(self, "stiffness_angular", [0.0, 0.0, 0.0])
        self._dz_f = _validate_len3(self, "force_deadzone", [0.8, 0.8, 0.8])
        self._dz_t = _validate_len3(self, "torque_deadzone", [0.08, 0.08, 0.08])

        self._max_lin_a = float(self.get_parameter("max_linear_accel").value)
        self._max_ang_a = float(self.get_parameter("max_angular_accel").value)

        self._lin = [AxisState(), AxisState(), AxisState()]
        self._ang = [AxisState(), AxisState(), AxisState()]

        self._wrench_raw = [0.0] * 6
        self._wrench_filtered = [0.0] * 6

        self._pub = self.create_publisher(Accel, self._output_topic, 10)
        self.create_subscription(WrenchStamped, self._input_topic, self._cb, 10)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "admittance controller ready: "
            f"{self._input_topic} -> {self._output_topic}, rate={rate_hz:.1f}Hz"
        )

    def _cb(self, msg: WrenchStamped) -> None:
        self._wrench_raw = [
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ]

    def _update_axis(
        self, state: AxisState, ext: float, mass: float, damping: float, stiffness: float
    ) -> float:
        m = max(1e-6, mass)
        acc = (ext - damping * state.vel - stiffness * state.pos) / m
        state.vel += acc * self._dt
        state.pos += state.vel * self._dt
        return acc

    def _tick(self) -> None:
        # Exponential LPF on wrench for smoother interaction.
        for i in range(6):
            self._wrench_filtered[i] = (
                self._alpha * self._wrench_raw[i]
                + (1.0 - self._alpha) * self._wrench_filtered[i]
            )

        w = list(self._wrench_filtered)
        for i in range(3):
            w[i] = _apply_deadzone(w[i], self._dz_f[i])
            w[3 + i] = _apply_deadzone(w[3 + i], self._dz_t[i])

        a_lin = [0.0, 0.0, 0.0]
        a_ang = [0.0, 0.0, 0.0]

        for i in range(3):
            a_lin[i] = self._update_axis(
                self._lin[i], w[i], self._m_lin[i], self._d_lin[i], self._k_lin[i]
            )
            a_ang[i] = self._update_axis(
                self._ang[i],
                w[3 + i],
                self._m_ang[i],
                self._d_ang[i],
                self._k_ang[i],
            )

            a_lin[i] = _clamp(a_lin[i], -self._max_lin_a, self._max_lin_a)
            a_ang[i] = _clamp(a_ang[i], -self._max_ang_a, self._max_ang_a)

        out = Accel()
        out.linear.x = a_lin[0]
        out.linear.y = a_lin[1]
        out.linear.z = a_lin[2]
        out.angular.x = a_ang[0]
        out.angular.y = a_ang[1]
        out.angular.z = a_ang[2]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = Pr2AdmittanceStub()
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
