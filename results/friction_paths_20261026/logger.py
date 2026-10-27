"""Read-only capture of the coupled transport execution chain."""
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

STEER = [f"{p}_caster_rotation_joint" for p in ["fl", "fr", "bl", "br"]]
WHEEL = [f"{p}_caster_{s}_wheel_joint" for p in ["fl", "fr", "bl", "br"] for s in ["l", "r"]]
ARM = [f"l_{n}_joint" for n in ["shoulder_pan", "shoulder_lift", "upper_arm_roll", "elbow_flex", "forearm_roll", "wrist_flex", "wrist_roll"]]


class Logger(Node):
    def __init__(self):
        super().__init__("cooperative_diagnosis_logger")
        self.rows = {}
        self.sim = 0.
        self.last = time.monotonic()
        self.started = False
        self.create_subscription(Float64, "/mujoco/sim_time", self.clock, 2000)
        for key, topic in [("board", "/mujoco/board_pose"), ("hand", "/mujoco/human_hand_pose"),
                           ("desired", "/transport_comparison/desired_hand_pose"), ("actual", "/transport_comparison/actual_hand_pose"),
                           ("ee", "/ee_pose")]:
            self.create_subscription(PoseStamped, topic, self.pose(key), 2000)
        for key, topic in [("human", "/virtual_human/hand_force"), ("wrist", "/mujoco/left_wrist_wrench")]:
            self.create_subscription(WrenchStamped, topic, self.wrench(key), 2000)
        for key, topic in [("cmd", "/cmd_vel"), ("reference", "/wbc/reference/cmd_vel")]:
            self.create_subscription(Twist, topic, self.twist(key), 2000)
        self.create_subscription(Odometry, "/odom", self.odom, 2000)
        self.create_subscription(JointState, "/joint_states", self.joints, 2000)
        self.create_subscription(JointState, "/mujoco/joint_actuator", self.actuators, 2000)
        self.create_subscription(String, "/transport_comparison/state", self.state, 100)

    def add(self, key, values):
        self.rows.setdefault(key, []).append([time.monotonic(), self.sim, *values])

    def clock(self, m):
        self.sim = m.data
        self.started = True
        self.last = time.monotonic()
        self.add("sim", [m.data])

    def pose(self, key):
        def callback(m):
            p, q = m.pose.position, m.pose.orientation
            self.add(key, [p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        return callback

    def wrench(self, key):
        def callback(m):
            f, t = m.wrench.force, m.wrench.torque
            self.add(key, [f.x, f.y, f.z, t.x, t.y, t.z])
        return callback

    def twist(self, key):
        def callback(m):
            self.add(key, [m.linear.x, m.linear.y, m.angular.z])
        return callback

    def odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        self.add("odom", [p.x, p.y, q.w, q.x, q.y, q.z, v.linear.x, v.linear.y, v.angular.z])

    def joints(self, m):
        index = {name: i for i, name in enumerate(m.name)}
        if all(name in index for name in STEER + WHEEL + ARM):
            self.add("joints", [m.position[index[n]] for n in STEER + ARM] + [m.velocity[index[n]] for n in WHEEL + ARM])

    def actuators(self, m):
        index = {name: i for i, name in enumerate(m.name)}
        if all(name in index for name in STEER + WHEEL):
            self.add("actuator", [m.effort[index[n]] for n in STEER + WHEEL])

    def state(self, m):
        self.add("state", [{"wait_for_state": 0, "latch_reference": 1, "settle": 2, "track": 3, "hold": 4, "done": 5}.get(m.data, -1)])


rclpy.init()
node = Logger()
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.1)
        if node.started and time.monotonic() - node.last > 3:
            break
except KeyboardInterrupt:
    pass
finally:
    np.savez_compressed(sys.argv[1], **{key: np.array(rows) for key, rows in node.rows.items()})
    print({key: len(rows) for key, rows in node.rows.items()}, flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
