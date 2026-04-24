#!/usr/bin/env python3
"""将末端外力投影为底盘可执行的平面外力/偏航力矩。"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pr2_mujoco_bridge.admittance_core import (
    apply_deadband,
    project_end_effector_wrench_to_base,
)


class ForceProjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_force_projector")

        self.declare_parameter("input_topic", "wbc/arm/external_wrench")
        self.declare_parameter("ee_pose_topic", "wbc/arm/ee_pose_log")
        self.declare_parameter("output_topic", "wbc/external_wrench")
        self.declare_parameter("planar_force_scale", 0.18)
        self.declare_parameter("yaw_torque_scale", 0.12)
        self.declare_parameter("filter_alpha", 0.2)
        self.declare_parameter("force_deadband", 0.25)
        self.declare_parameter("yaw_deadband", 0.05)

        self._in_topic = str(self.get_parameter("input_topic").value)
        self._ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._planar_scale = float(self.get_parameter("planar_force_scale").value)
        self._yaw_scale = float(self.get_parameter("yaw_torque_scale").value)
        self._filter_alpha = float(self.get_parameter("filter_alpha").value)
        self._force_deadband = float(self.get_parameter("force_deadband").value)
        self._yaw_deadband = float(self.get_parameter("yaw_deadband").value)

        self._ee_pos_base: np.ndarray | None = None
        self._filtered = np.zeros(6, dtype=np.float64)
        self._warned_frame = False

        self._pub = self.create_publisher(WrenchStamped, self._out_topic, 10)
        self.create_subscription(PoseStamped, self._ee_pose_topic, self._pose_cb, 10)
        self.create_subscription(WrenchStamped, self._in_topic, self._wrench_cb, 10)

        self.get_logger().info(
            f"力投影器: {self._in_topic} + {self._ee_pose_topic} -> {self._out_topic} "
            f"(planar_scale={self._planar_scale}, yaw_scale={self._yaw_scale})"
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._ee_pos_base = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=np.float64,
        )
        if msg.header.frame_id not in ("", "base_link") and not self._warned_frame:
            self.get_logger().warn(
                f"期望 EE pose 使用 base_link，当前为 {msg.header.frame_id}，将按 base_link 解释"
            )
            self._warned_frame = True

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        if self._ee_pos_base is None:
            return

        if msg.header.frame_id not in ("", "base_link") and not self._warned_frame:
            self.get_logger().warn(
                f"期望外力使用 base_link，当前为 {msg.header.frame_id}，将按 base_link 解释"
            )
            self._warned_frame = True

        projected = project_end_effector_wrench_to_base(
            ee_pos_in_base=self._ee_pos_base,
            ee_force_in_base=np.array(
                [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
                dtype=np.float64,
            ),
            planar_scale=self._planar_scale,
            yaw_scale=self._yaw_scale,
        )
        projected[:2] = apply_deadband(
            projected[:2],
            np.array([self._force_deadband, self._force_deadband], dtype=np.float64),
        )
        if abs(projected[5]) < self._yaw_deadband:
            projected[5] = 0.0

        self._filtered = (
            (1.0 - self._filter_alpha) * self._filtered
            + self._filter_alpha * projected
        )

        out = WrenchStamped()
        out.header = msg.header
        out.header.frame_id = "base_link"
        out.wrench.force.x = float(self._filtered[0])
        out.wrench.force.y = float(self._filtered[1])
        out.wrench.force.z = float(self._filtered[2])
        out.wrench.torque.x = float(self._filtered[3])
        out.wrench.torque.y = float(self._filtered[4])
        out.wrench.torque.z = float(self._filtered[5])
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
