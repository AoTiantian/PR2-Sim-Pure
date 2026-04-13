#!/usr/bin/env python3
"""Null-space posture generator.

发布 q_nominal（JSON）供 WBC 协调器叠加次级姿态保持力矩。
"""

from __future__ import annotations

import json
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


DEFAULT_ARM_JOINTS = [
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_forearm_roll_joint",
    "l_wrist_flex_joint",
    "l_wrist_roll_joint",
]


class Pr2NullSpaceStub(Node):
    def __init__(self) -> None:
        super().__init__("pr2_null_space_controller")

        self.declare_parameter("output_topic", "wbc/q_nominal")
        self.declare_parameter("joint_state_topic", "state/joint_states")
        self.declare_parameter("controlled_joints", DEFAULT_ARM_JOINTS)
        self.declare_parameter("nominal_positions", [])
        self.declare_parameter("auto_capture_on_start", True)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("nullspace_kp", 15.0)
        self.declare_parameter("nullspace_kd", 2.0)
        self.declare_parameter("max_effort", 10.0)

        out = str(self.get_parameter("output_topic").value)
        js_topic = str(self.get_parameter("joint_state_topic").value)
        self._joints: List[str] = list(self.get_parameter("controlled_joints").value)
        self._q_nom: Dict[str, float] = {}
        self._captured = False

        q_nom_list = list(self.get_parameter("nominal_positions").value)
        if len(q_nom_list) == len(self._joints) and len(self._joints) > 0:
            self._q_nom = {
                n: float(q_nom_list[i]) for i, n in enumerate(self._joints)
            }
            self._captured = True

        self._auto_capture = bool(self.get_parameter("auto_capture_on_start").value)
        self._kp = float(self.get_parameter("nullspace_kp").value)
        self._kd = float(self.get_parameter("nullspace_kd").value)
        self._max_effort = float(self.get_parameter("max_effort").value)

        self._pub = self.create_publisher(String, out, 10)
        self.create_subscription(JointState, js_topic, self._on_js, 20)

        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, hz), self._tick)
        self.get_logger().info(
            f"null_space_controller: publish q_nominal JSON on {out}, joints={len(self._joints)}"
        )

    def _on_js(self, msg: JointState) -> None:
        if self._captured or not self._auto_capture:
            return
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in self._joints):
            return
        for n in self._joints:
            i = index[n]
            if i < len(msg.position):
                self._q_nom[n] = float(msg.position[i])
            else:
                self._q_nom[n] = 0.0
        self._captured = True
        self.get_logger().info("null_space_controller: captured current posture as q_nominal")

    def _tick(self) -> None:
        if not self._q_nom:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "mode": "posture_hold",
                "joints": self._joints,
                "q_nominal": [self._q_nom.get(n, 0.0) for n in self._joints],
                "kp": self._kp,
                "kd": self._kd,
                "max_effort": self._max_effort,
            }
        )
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
