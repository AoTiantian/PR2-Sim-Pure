#!/usr/bin/env python3
"""末端力到底盘力的转换节点（薄层，准静态近似）。

将 /wbc/arm/external_wrench（末端施加的力）转发到
/wbc/external_wrench（底盘顺应控制器的输入话题），
使底盘对末端力也产生顺应响应，从而实现全身力控。

准静态近似：EE 力 ≈ 底盘受到的等效外力（忽略手臂动力学），
适用于低速、小加速度的演示场景。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ForceProjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_force_projector")

        self.declare_parameter("input_topic",  "wbc/arm/external_wrench")
        self.declare_parameter("output_topic", "wbc/external_wrench")
        self.declare_parameter("force_scale",  0.4)   # 缩放系数，避免底盘过度响应

        in_topic  = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._scale = float(self.get_parameter("force_scale").value)

        self._pub = self.create_publisher(WrenchStamped, out_topic, 10)
        self.create_subscription(WrenchStamped, in_topic, self._cb, 10)

        self.get_logger().info(
            f"力投影器: {in_topic} -> {out_topic} (scale={self._scale})"
        )

    def _cb(self, msg: WrenchStamped) -> None:
        out = WrenchStamped()
        out.header = msg.header
        out.header.frame_id = "base_link"
        out.wrench.force.x  = msg.wrench.force.x  * self._scale
        out.wrench.force.y  = msg.wrench.force.y  * self._scale
        out.wrench.force.z  = msg.wrench.force.z  * self._scale
        # 不传递力矩（本方案仅处理纯力）
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ForceProjectorNode()
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
