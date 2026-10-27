"""Isolated runtime extension; original controller file is untouched.

Delegates filtering, original admittance integration and all publishing to the
existing QP node. A solver adapter replaces velocity allocation only while the
explicit experiment is in TRACK/HOLD. Post-QP base filtering stays unchanged.
"""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64, String

from angular_priority import allocate
from pr2_wbc_admittance_control.pr2_qp_whole_body_admittance import Pr2QpWholeBodyAdmittance
from pr2_wbc_admittance_control.pr2_dynamics_utils import get_ee_jacobian_6x10
from pr2_virtual_human.trajectory_6d import TrajectoryConfig, build_trajectory


class SolverAdapter:
    def __init__(self, original, node):
        self.original, self.node = original, node
        self.lower = self.upper = None

    def setup(self, **kwargs):
        self.lower = np.array(kwargs["l"]).copy()
        self.upper = np.array(kwargs["u"]).copy()
        return self.original.setup(**kwargs)

    def update(self, **kwargs):
        if "l" in kwargs:
            self.lower = np.array(kwargs["l"]).copy()
        if "u" in kwargs:
            self.upper = np.array(kwargs["u"]).copy()
        return self.original.update(**kwargs)

    def solve(self):
        result = self.original.solve()
        n = self.node
        if result.x is None or result.info.status_val not in (1, 2):
            return result
        if n.phase not in ("track", "hold") or n.path is None:
            return result
        p = n._latest_odom.pose.pose.position
        current = np.array([p.x, p.y, p.z])
        elapsed = max(0., n.sim_time-n.track_start)
        sample = n.path.sample(elapsed)
        b = np.r_[sample.linear_velocity[:2], sample.angular_velocity[2]]
        b[:2] += n.path_cfg["position_feedback_gain"] * (sample.position-current)[:2]
        b *= n.assist_gain
        speed = np.linalg.norm(b[:2])
        if speed > n.path_cfg["max_planar_speed"]:
            b[:2] *= n.path_cfg["max_planar_speed"] / speed
        b[2] = np.clip(b[2], -n.path_cfg["max_yaw_speed"], n.path_cfg["max_yaw_speed"])
        J = get_ee_jacobian_6x10(n._model, n._data, n._ee_body_id, n._dof)
        lower, upper = self.lower.copy(), self.upper.copy()
        # The explicitly enabled active regime can move with zero wrench.
        # Keep arm limits; do not change the passive node's no-wrench rule.
        lower[7:], upper[7:] = [-n._base_v_lim, -n._base_v_lim, -n._base_w_lim], [n._base_v_lim, n._base_v_lim, n._base_w_lim]
        u, detail = allocate(J, n._v_adm.copy(), b, lower, upper, n._vmax_lin)
        detail.update(sim_time=n.sim_time, elapsed=elapsed, phase=n.phase,
                      robot_path_position=sample.position.tolist(),
                      actual_base_position=current.tolist(),
                      path_error=(sample.position-current).tolist(),
                      original_admittance_twist=n._v_adm.tolist(),
                      actual_ee_quaternion=[n._latest_pose.pose.orientation.w,
                                            n._latest_pose.pose.orientation.x,
                                            n._latest_pose.pose.orientation.y,
                                            n._latest_pose.pose.orientation.z],
                      allocated_u=u.tolist(), before_filter_twist=(J @ u).tolist())
        if time.monotonic()-n.last_trace > .05:
            n.last_trace = time.monotonic()
            with n.trace_path.open("a") as f:
                f.write(json.dumps(detail)+"\n")
        result.x = u
        return result


class TrajectoryQp(Pr2QpWholeBodyAdmittance):
    def __init__(self):
        super().__init__()
        self.declare_parameter("control_regime", "base_path_with_angular_admittance")
        self.declare_parameter("robot_path_file", str(Path(__file__).with_name("robot_path.yaml")))
        self.declare_parameter("trajectory_assist_gain", 1.0)
        if self._pose_tracking_enable or self._orientation_tracking_enable or self._freeze_ori or self._fixed_target_mode:
            raise RuntimeError("this experiment requires the original unmodified admittance law")
        self.path_cfg = yaml.safe_load(Path(self.get_parameter("robot_path_file").value).read_text())
        self.assist_gain = float(self.get_parameter("trajectory_assist_gain").value)
        self.phase, self.sim_time, self.track_start = "wait_for_state", 0., 0.
        self.path = None
        self.last_trace = 0.
        self.trace_path = Path(os.environ["BASE_PATH_TRACE"])
        self._osqp = SolverAdapter(self._osqp, self)
        self.create_subscription(Float64, "transport_comparison/sim_time", self.on_sim, 20)
        self.create_subscription(String, "transport_comparison/state", self.on_phase, 20)
        self.get_logger().info("EXPERIMENT: known robot base path + angular admittance; gain="+str(self.assist_gain))

    def on_sim(self, msg):
        self.sim_time = float(msg.data)

    def on_phase(self, msg):
        self.phase = msg.data
        if self.phase == "track" and self.path is None and self._latest_odom is not None:
            p = self._latest_odom.pose.pose.position
            # Waypoints here are odom-relative XY; yaw reference is deliberately
            # constant for S22. No human/board desired-pose input is read.
            c = TrajectoryConfig(duration_sec=float(self.path_cfg["duration_sec"]), cycles=1.,
                                 position_amplitude=np.zeros(3), orientation_amplitude=np.zeros(3),
                                 mode="waypoints", waypoints=np.array(self.path_cfg["waypoints"]))
            self.path = build_trajectory(c, np.array([p.x,p.y,p.z]), np.array([1.,0,0,0]))
            self.track_start = self.sim_time


def main():
    rclpy.init()
    node = TrajectoryQp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
