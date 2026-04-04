#!/usr/bin/env python3
"""导纳控制占位节点：可接外力/名义轨迹后输出加速度（当前为直通/零输出示例）。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Accel, WrenchStamped
from rclpy.node import Node


class Pr2AdmittanceStub(Node):
    def __init__(self) -> None:
        super().__init__("pr2_admittance_stub")
        self.declare_parameter("output_topic", "wbc/admittance_accel")
        out = self.get_parameter("output_topic").value
        self._pub = self.create_publisher(Accel, out, 10)
        self.create_subscription(WrenchStamped, "wbc/external_wrench", self._cb, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"admittance_stub: publish zero Accel on {out} (placeholder)")

    def _cb(self, msg: WrenchStamped) -> None:
        self.get_logger().debug("received wrench (stub ignores)")

    def _tick(self) -> None:
        self._pub.publish(Accel())


def main() -> None:
    rclpy.init()
    node = Pr2AdmittanceStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
