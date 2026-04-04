#!/usr/bin/env python3
"""零空间 / 次要任务占位：发布名义关节姿态话题（当前为空 JSON 占位）。"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Pr2NullSpaceStub(Node):
    def __init__(self) -> None:
        super().__init__("pr2_null_space_stub")
        self.declare_parameter("output_topic", "wbc/q_nominal")
        out = self.get_parameter("output_topic").value
        self._pub = self.create_publisher(String, out, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"null_space_stub: nominal posture JSON on {out}")

    def _tick(self) -> None:
        msg = String()
        msg.data = json.dumps({"note": "stub", "q_nominal": []})
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Pr2NullSpaceStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
