#!/usr/bin/env python3
"""Record virtual human topics AND the wrench topic for plotting."""
import csv, time, os, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64, String

OUT = "/tmp/vh_data.csv"

class Recorder(Node):
    def __init__(self):
        super().__init__("vh_recorder")
        self._t0 = time.monotonic()
        self._rows = []
        self._n = 0
        self._last = {}

        self.create_subscription(PoseStamped, "virtual_human/actual_hand_pose", lambda m: self._cb("ah", m), 10)
        self.create_subscription(PoseStamped, "virtual_human/desired_hand_pose", lambda m: self._cb("dh", m), 10)
        self.create_subscription(Float64, "virtual_human/force_magnitude", lambda m: self._cb("fm", m), 10)
        self.create_subscription(String, "virtual_human/state", lambda m: self._cb("st", m), 10)
        self.create_subscription(WrenchStamped, "wbc/whole_body_wrench", lambda m: self._cb("wr", m), 10)

        self.create_timer(1.0, self._tick)

    def _cb(self, tag, msg):
        self._n += 1
        t = time.monotonic() - self._t0
        if tag in ("ah", "dh"):
            p = msg.pose.position
            self._last[tag] = (p.x, p.y, p.z)
        elif tag == "fm":
            self._last[tag] = msg.data
        elif tag == "st":
            self._last[tag] = msg.data
        elif tag == "wr":
            self._last["fx"] = msg.wrench.force.x
            self._last["fy"] = msg.wrench.force.y
            self._last["fz"] = msg.wrench.force.z

    def _tick(self):
        # flush every second
        if self._last:
            r = self._last.copy()
            self._rows.append(r)
            self._last = {}

    def save(self):
        with open(OUT, "w") as f:
            w = csv.writer(f)
            w.writerow(["t", "state", "ah_x","ah_y","ah_z","dh_x","dh_y","dh_z","f_mag","fx","fy","fz"])
            t0 = None
            for r in self._rows:
                # find t0 as first tracking entry
                st = r.get("st", "")
                if t0 is None and st in ("tracking", "hold"):
                    t0 = r.get("_t", 0)
                # Not stored properly, need to fix

def main():
    # This approach is getting too complex. Let me use a simpler method.
    pass
