#!/usr/bin/env python3
"""Record PR2 whole-body state from ROS topics for offline MuJoCo video rendering."""
from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WholeBodyStateRecorder(Node):
    def __init__(self, out: str, hz: float) -> None:
        super().__init__("pr2_whole_body_state_recorder")
        self.out = out
        self.joint_names: list[str] | None = None
        self.latest_joint_pos: np.ndarray | None = None
        self.latest_odom = np.zeros(7, dtype=float)  # x y z qx qy qz qw
        self.latest_force = np.zeros(3, dtype=float)
        self.t0: float | None = None
        self.samples_t: list[float] = []
        self.samples_odom: list[np.ndarray] = []
        self.samples_force: list[np.ndarray] = []
        self.samples_joint_pos: list[np.ndarray] = []
        self.create_subscription(JointState, "joint_states", self._joint_cb, 50)
        self.create_subscription(Odometry, "odom", self._odom_cb, 50)
        self.create_subscription(WrenchStamped, "wbc/arm/external_wrench", self._wrench_cb, 20)
        self.create_timer(1.0 / hz, self._sample)
        self.get_logger().info(f"recording whole-body state to {out} at {hz:.1f} Hz")

    def _now_s(self) -> float:
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _joint_cb(self, msg: JointState) -> None:
        if self.joint_names is None:
            self.joint_names = list(msg.name)
        self.latest_joint_pos = np.asarray(msg.position, dtype=float)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom[:] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self.latest_force[:] = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

    def _sample(self) -> None:
        if self.latest_joint_pos is None or self.joint_names is None:
            return
        self.samples_t.append(self._now_s())
        self.samples_odom.append(self.latest_odom.copy())
        self.samples_force.append(self.latest_force.copy())
        self.samples_joint_pos.append(self.latest_joint_pos.copy())

    def save(self) -> None:
        if not self.samples_t or self.joint_names is None:
            self.get_logger().warn("no samples captured")
            return
        Path(self.out).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            self.out,
            t=np.asarray(self.samples_t, dtype=float),
            odom=np.vstack(self.samples_odom),
            force=np.vstack(self.samples_force),
            joint_pos=np.vstack(self.samples_joint_pos),
            joint_names=np.asarray(self.joint_names, dtype=str),
        )
        meta = {
            "samples": len(self.samples_t),
            "duration_s": float(self.samples_t[-1]),
            "joint_count": len(self.joint_names),
            "out": self.out,
        }
        print(json.dumps(meta, indent=2))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()
    rclpy.init()
    node = WholeBodyStateRecorder(args.out, args.hz)
    stop = False

    def _stop(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
