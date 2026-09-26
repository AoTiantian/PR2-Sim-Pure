"""Verification logger: records raw ROS traffic of one human_robot run to an .npz.

usage: python3 verify_logger.py <out.npz>
Stops itself 2 s after /mujoco/sim_time goes silent (run finished).
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ARM = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
       "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]


class Logger(Node):
    def __init__(self, out):
        super().__init__("verify_logger")
        self.out = out
        q = QoSProfile(depth=20000, reliability=ReliabilityPolicy.RELIABLE)
        self.rows = {k: [] for k in
                     ["sim_time", "hand_force", "wrist", "js", "cmd_out", "qp_out", "support", "board", "hand_pose"]}
        self.last_sim_wall = None
        self.create_subscription(Float64, "/mujoco/sim_time", self.cb_sim, q)
        self.create_subscription(WrenchStamped, "/virtual_human/hand_force", self.cb_wrench("hand_force"), q)
        self.create_subscription(WrenchStamped, "/mujoco/left_wrist_wrench", self.cb_wrench("wrist"), q)
        self.create_subscription(JointState, "/joint_states", self.cb_js, q)
        self.create_subscription(JointState, "/joint_commands", self.cb_cmd("cmd_out"), q)
        self.create_subscription(JointState, "/wbc/reference/joint_command", self.cb_cmd("qp_out"), q)
        self.create_subscription(Float64, "/mujoco/robot_support_force", self.cb_support, q)
        self.create_subscription(PoseStamped, "/mujoco/board_pose", self.cb_pose("board"), q)
        self.create_subscription(PoseStamped, "/mujoco/human_hand_pose", self.cb_pose("hand_pose"), q)
        self.create_timer(0.5, self.check_done)

    def cb_sim(self, m):
        w = time.monotonic()
        self.last_sim_wall = w
        self.rows["sim_time"].append((w, m.data))

    def cb_wrench(self, key):
        def f(m):
            w = m.wrench
            self.rows[key].append((time.monotonic(), w.force.x, w.force.y, w.force.z,
                                   w.torque.x, w.torque.y, w.torque.z))
        return f

    def cb_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        try:
            pos = [m.position[idx[n]] for n in ARM]
            vel = [m.velocity[idx[n]] for n in ARM]
        except KeyError:
            return
        self.rows["js"].append((time.monotonic(), *pos, *vel))

    def cb_cmd(self, key):
        def f(m):
            idx = {n: i for i, n in enumerate(m.name)}
            try:
                vel = [m.velocity[idx[n]] for n in ARM]
            except (KeyError, IndexError):
                return
            self.rows[key].append((time.monotonic(), *vel))
        return f

    def cb_support(self, m):
        self.rows["support"].append((time.monotonic(), m.data))

    def cb_pose(self, key):
        def f(m):
            p = m.pose.position
            self.rows[key].append((time.monotonic(), p.x, p.y, p.z))
        return f

    def check_done(self):
        if self.last_sim_wall is not None and time.monotonic() - self.last_sim_wall > 2.0:
            np.savez(self.out, **{k: np.array(v, dtype=np.float64) for k, v in self.rows.items()})
            print("saved", self.out, {k: len(v) for k, v in self.rows.items()}, flush=True)
            raise SystemExit


def main():
    rclpy.init()
    node = Logger(sys.argv[1])
    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
