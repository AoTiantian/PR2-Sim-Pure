"""Bare-chassis diagnostic: log caster steer angle/torque + wheel vel + odom during a cmd_vel step."""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

STEER = ["fl_caster_rotation_joint", "fr_caster_rotation_joint",
          "bl_caster_rotation_joint", "br_caster_rotation_joint"]
WHEEL = ["fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
         "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
         "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
         "br_caster_l_wheel_joint", "br_caster_r_wheel_joint"]


class Logger(Node):
    def __init__(self, out, vx, vy, step_t, dur):
        super().__init__("base_step_logger")
        self.out, self.step_t, self.dur = out, step_t, dur
        self.vx, self.vy = vx, vy
        q = QoSProfile(depth=20000, reliability=ReliabilityPolicy.RELIABLE)
        self.rows = {"steer_pos": [], "steer_trq": [], "wheel_trq": [], "odom": [], "wheel_vel": []}
        self.t0 = time.monotonic()
        self.stepped = False
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self.cb_js, q)
        self.create_subscription(JointState, "/mujoco/joint_actuator", self.cb_act, q)
        self.create_subscription(Odometry, "/odom", self.cb_odom, q)
        self.create_timer(0.01, self.cb_tick)

    def cb_tick(self):
        t = time.monotonic() - self.t0
        msg = Twist()
        if t >= self.step_t:
            msg.linear.x = self.vx
            msg.linear.y = self.vy
        self.pub.publish(msg)
        if t >= self.dur:
            np.savez(self.out, **{k: np.array(v, dtype=np.float64) for k, v in self.rows.items()},
                     step_t=self.step_t, vx=self.vx, vy=self.vy)
            print("saved", self.out, {k: len(v) for k, v in self.rows.items()}, flush=True)
            raise SystemExit

    def cb_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        try:
            pos = [m.position[idx[n]] for n in STEER]
            wv = [m.velocity[idx[n]] for n in WHEEL]
        except KeyError:
            return
        t = time.monotonic() - self.t0
        self.rows["steer_pos"].append((t, *pos))
        self.rows["wheel_vel"].append((t, *wv))

    def cb_act(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        try:
            st = [m.effort[idx[n]] for n in STEER]
            wh = [m.effort[idx[n]] for n in WHEEL]
        except KeyError:
            return
        t = time.monotonic() - self.t0
        self.rows["steer_trq"].append((t, *st))
        self.rows["wheel_trq"].append((t, *wh))

    def cb_odom(self, m):
        t = time.monotonic() - self.t0
        tw = m.twist.twist
        self.rows["odom"].append((t, tw.linear.x, tw.linear.y, tw.angular.z))


def main():
    out, vx, vy, step_t, dur = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
    rclpy.init()
    node = Logger(out, vx, vy, step_t, dur)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
