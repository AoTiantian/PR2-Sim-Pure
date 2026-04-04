#!/usr/bin/env python3
"""
底座加速度 → 速度 积分器（对应框图「底座转换器(积分器)」的简化版）。

订阅 geometry_msgs/Accel（约定）：
  - linear.x, linear.y：平面加速度 (m/s^2)
  - angular.z：绕竖直轴角加速度 (rad/s^2)

积分得到 Twist，发布为 cmd_vel（可由 WBC 输出加速度后接本节点）。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Accel, Twist
from rclpy.node import Node


class Pr2BaseAccelIntegrator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_base_accel_integrator")

        self.declare_parameter("input_topic", "wbc/base_acceleration")
        self.declare_parameter("output_topic", "wbc/reference/cmd_vel")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_linear_vel", 1.5)
        self.declare_parameter("max_angular_vel", 2.0)

        self._max_lv = float(self.get_parameter("max_linear_vel").value)
        self._max_w = float(self.get_parameter("max_angular_vel").value)
        hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(hz, 1.0)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._accel = Accel()

        self.create_subscription(
            Accel,
            self.get_parameter("input_topic").value,
            self._cb_accel,
            10,
        )
        self._pub = self.create_publisher(
            Twist,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "base_accel_integrator: Accel -> Twist (integral), output to "
            f"{self.get_parameter('output_topic').value}"
        )

    def _cb_accel(self, msg: Accel) -> None:
        self._accel = msg

    def _tick(self) -> None:
        a = self._accel
        self._vx += float(a.linear.x) * self._dt
        self._vy += float(a.linear.y) * self._dt
        self._wz += float(a.angular.z) * self._dt

        v = math.hypot(self._vx, self._vy)
        if v > self._max_lv > 1e-9:
            s = self._max_lv / v
            self._vx *= s
            self._vy *= s
        self._wz = max(-self._max_w, min(self._max_w, self._wz))

        tw = Twist()
        tw.linear.x = self._vx
        tw.linear.y = self._vy
        tw.angular.z = self._wz
        self._pub.publish(tw)


def main() -> None:
    rclpy.init()
    node = Pr2BaseAccelIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
