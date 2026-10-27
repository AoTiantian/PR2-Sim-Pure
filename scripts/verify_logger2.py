"""Verification logger v2: adds base steer/wheel diagnostics + desired/actual hand pose
to isolate what's driving residual X-lateral tracking error after the cmd_vel_linear_gain fix.
usage: python3 verify_logger2.py <out.npz>
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

STEER = ["fl_caster_rotation_joint", "fr_caster_rotation_joint",
          "bl_caster_rotation_joint", "br_caster_rotation_joint"]
WHEEL = ["fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
         "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
         "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
         "br_caster_l_wheel_joint", "br_caster_r_wheel_joint"]


class Logger(Node):
    def __init__(self, out):
        super().__init__("verify_logger2")
        self.out = out
        q = QoSProfile(depth=20000, reliability=ReliabilityPolicy.RELIABLE)
        self.rows = {k: [] for k in
                     ["sim_time", "desired_pose", "actual_pose", "steer_pos", "steer_trq",
                      "wheel_trq", "wheel_vel", "cmd_vel", "odom_vel"]}
        self.last_sim_wall = None
        self.create_subscription(Float64, "/mujoco/sim_time", self.cb_sim, q)
        self.create_subscription(PoseStamped, "/transport_comparison/desired_hand_pose", self.cb_pose("desired_pose"), q)
        self.create_subscription(PoseStamped, "/transport_comparison/actual_hand_pose", self.cb_pose("actual_pose"), q)
        self.create_subscription(JointState, "/joint_states", self.cb_js, q)
        self.create_subscription(JointState, "/mujoco/joint_actuator", self.cb_act, q)
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd_vel, q)
        self.create_subscription(__import__("nav_msgs.msg", fromlist=["Odometry"]).Odometry, "/odom", self.cb_odom, q)
        self.create_timer(0.5, self.check_done)

    def cb_sim(self, m):
        w = time.monotonic()
        self.last_sim_wall = w
        self.rows["sim_time"].append((w, m.data))

    def cb_pose(self, key):
        def f(m):
            p = m.pose.position
            self.rows[key].append((time.monotonic(), p.x, p.y, p.z))
        return f

    def cb_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        try:
            steer = [m.position[idx[n]] for n in STEER]
            wv = [m.velocity[idx[n]] for n in WHEEL]
        except KeyError:
            return
        t = time.monotonic()
        self.rows["steer_pos"].append((t, *steer))
        self.rows["wheel_vel"].append((t, *wv))

    def cb_act(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        try:
            st = [m.effort[idx[n]] for n in STEER]
            wh = [m.effort[idx[n]] for n in WHEEL]
        except KeyError:
            return
        t = time.monotonic()
        self.rows["steer_trq"].append((t, *st))
        self.rows["wheel_trq"].append((t, *wh))

    def cb_cmd_vel(self, m):
        self.rows["cmd_vel"].append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z))

    def cb_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        tw = m.twist.twist
        self.rows["odom_vel"].append((time.monotonic(), tw.linear.x, tw.linear.y, tw.angular.z,
                                       q.w, q.x, q.y, q.z, p.x, p.y))

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
