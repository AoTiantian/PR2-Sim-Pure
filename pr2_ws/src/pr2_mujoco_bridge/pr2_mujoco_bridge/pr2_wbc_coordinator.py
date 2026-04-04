#!/usr/bin/env python3
"""
全身控制（WBC）协调器 — 占位实现。

框图中 WBC 求解器应输出 \\ddot{q}^* 与 \\tau；此处用可替换的「参考汇总」：
  - 订阅 /wbc/reference/cmd_vel → 发布 cmd_vel（到底座）
  - 订阅 /wbc/reference/joint_command → 合并后发布 joint_commands（臂力矩、夹爪位置等）

后续可将本节点替换为真实 QP-WBC，保持话题名不变即可对接仿真。
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class Pr2WbcCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_wbc_coordinator")

        self.declare_parameter("output_cmd_vel", "cmd_vel")
        self.declare_parameter("output_joint_command", "joint_commands")
        self.declare_parameter("publish_rate_hz", 100.0)

        self._out_twist = self.get_parameter("output_cmd_vel").value
        self._out_joint = self.get_parameter("output_joint_command").value
        hz = float(self.get_parameter("publish_rate_hz").value)

        self._lock = threading.Lock()
        self._twist: Optional[Twist] = None
        self._joint_merge: Dict[str, Dict[str, float]] = {}
        # name -> {"position": v or nan, "velocity": v or nan, "effort": v or nan}
        self._gripper_pos: Optional[float] = None

        self._pub_cmd = self.create_publisher(Twist, self._out_twist, 10)
        self._pub_joint = self.create_publisher(JointState, self._out_joint, 10)

        self.create_subscription(
            Twist, "wbc/reference/cmd_vel", self._cb_twist, 10
        )
        self.create_subscription(
            JointState,
            "wbc/reference/joint_command",
            self._cb_joint,
            10,
        )
        self.create_subscription(
            Float64,
            "wbc/reference/gripper_position",
            self._cb_gripper,
            10,
        )

        self.create_timer(1.0 / max(hz, 1.0), self._tick)

        self.get_logger().info(
            "WBC coordinator (stub): refs on wbc/reference/* -> "
            f"{self._out_twist}, {self._out_joint}"
        )

    def _cb_twist(self, msg: Twist) -> None:
        with self._lock:
            self._twist = msg

    def _cb_joint(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if name not in self._joint_merge:
                    self._joint_merge[name] = {
                        "position": float("nan"),
                        "velocity": float("nan"),
                        "effort": float("nan"),
                    }
                d = self._joint_merge[name]
                if i < len(msg.position):
                    d["position"] = float(msg.position[i])
                if i < len(msg.velocity):
                    d["velocity"] = float(msg.velocity[i])
                if i < len(msg.effort):
                    d["effort"] = float(msg.effort[i])

    def _cb_gripper(self, msg: Float64) -> None:
        with self._lock:
            self._gripper_pos = float(msg.data)

    def _tick(self) -> None:
        with self._lock:
            t = self._twist
            merge = {k: dict(v) for k, v in self._joint_merge.items()}
            gp = self._gripper_pos

        if t is not None:
            self._pub_cmd.publish(t)

        names = sorted(merge.keys())
        if gp is not None and "l_gripper_l_finger_joint" not in names:
            names.append("l_gripper_l_finger_joint")

        if not names:
            return

        pos: List[float] = []
        vel: List[float] = []
        eff: List[float] = []
        for n in names:
            if n == "l_gripper_l_finger_joint" and gp is not None:
                if n in merge:
                    d = merge[n]
                    pos.append(d.get("position", gp))
                    vel.append(d.get("velocity", float("nan")))
                    eff.append(d.get("effort", float("nan")))
                else:
                    pos.append(gp)
                    vel.append(float("nan"))
                    eff.append(float("nan"))
                continue
            d = merge[n]
            pos.append(d.get("position", float("nan")))
            vel.append(d.get("velocity", float("nan")))
            eff.append(d.get("effort", float("nan")))

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = pos
        out.velocity = vel
        out.effort = eff
        self._pub_joint.publish(out)


def main() -> None:
    rclpy.init()
    node = Pr2WbcCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
