#!/usr/bin/env python3
"""状态估计 / 状态汇总：同步 joint_states 与 odom，供上层 WBC / 规划使用。"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Pr2StateEstimator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_state_estimator")

        self.declare_parameter("joint_state_in", "joint_states")
        self.declare_parameter("odom_in", "odom")
        self.declare_parameter("joint_state_out", "state/joint_states")
        self.declare_parameter("odom_out", "state/odom")

        jin = self.get_parameter("joint_state_in").value
        oin = self.get_parameter("odom_in").value
        jout = self.get_parameter("joint_state_out").value
        oout = self.get_parameter("odom_out").value

        self._pub_js = self.create_publisher(JointState, jout, 10)
        self._pub_odom = self.create_publisher(Odometry, oout, 10)

        self.create_subscription(JointState, jin, self._on_js, 20)
        self.create_subscription(Odometry, oin, self._on_odom, 20)

        self.get_logger().info(
            f"state_estimator: {jin}->{jout}, {oin}->{oout}"
        )

    def _on_js(self, msg: JointState) -> None:
        self._pub_js.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._pub_odom.publish(msg)


def main() -> None:
    rclpy.init()
    node = Pr2StateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
