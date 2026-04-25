#!/usr/bin/env python3
"""Inject end-effector wrench commands and record the response to CSV."""

from __future__ import annotations

import csv
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _yaw_from_quaternion_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class ArmForceInjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_force_injector")

        dynamic_str = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("force_axis", "x", dynamic_str)
        self.declare_parameter("force_magnitude", 10.0)
        self.declare_parameter("step_duration", 3.0)
        self.declare_parameter("injection_delay", 1.0)
        self.declare_parameter("ready_delay", 0.0)
        self.declare_parameter("settle_duration", 4.0)
        self.declare_parameter("log_file", "/tmp/arm_response.csv")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("waveform", "step", dynamic_str)
        self.declare_parameter("sine_freq", 0.5)
        self.declare_parameter("wrench_topic", "wbc/arm/external_wrench")
        self.declare_parameter("ee_pose_topic", "wbc/arm/ee_pose_log")
        self.declare_parameter("odom_topic", "odom")

        axis_raw = self.get_parameter("force_axis").value
        self._axis = "y" if axis_raw is True else str(axis_raw).lower()
        self._magnitude = float(self.get_parameter("force_magnitude").value)
        self._step_duration = float(self.get_parameter("step_duration").value)
        self._injection_delay = float(self.get_parameter("injection_delay").value)
        self._ready_delay = float(self.get_parameter("ready_delay").value)
        self._settle_duration = float(self.get_parameter("settle_duration").value)
        self._log_file = str(self.get_parameter("log_file").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._waveform = str(self.get_parameter("waveform").value)
        self._sine_freq = float(self.get_parameter("sine_freq").value)
        wrench_topic = str(self.get_parameter("wrench_topic").value)
        ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self._pub_wrench = self.create_publisher(WrenchStamped, wrench_topic, 10)
        self.create_subscription(PoseStamped, ee_pose_topic, self._pose_cb, 20)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)

        os.makedirs(os.path.dirname(self._log_file) or ".", exist_ok=True)
        self._csv_f = open(self._log_file, "w", newline="", encoding="utf-8")
        self._csv_w = csv.writer(self._csv_f)
        self._csv_w.writerow(
            [
                "timestamp",
                "pos_x",
                "pos_y",
                "pos_z",
                "force_x",
                "force_y",
                "force_z",
                "base_x",
                "base_y",
                "base_yaw",
            ]
        )

        self._current_force = np.zeros(3, dtype=np.float64)
        self._base_state = np.zeros(3, dtype=np.float64)
        self._start_time: float | None = None
        self._pose_ready_time: float | None = None
        self._pose_received = False
        self._done = False
        self._shutdown_requested = False

        self._timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._timer_cb)
        self.get_logger().info(
            "force injector ready: "
            f"axis={self._axis}, magnitude={self._magnitude:.3f} N, "
            f"waveform={self._waveform}, ready_delay={self._ready_delay:.3f} s, "
            f"delay={self._injection_delay:.3f} s, "
            f"duration={self._step_duration:.3f} s, settle={self._settle_duration:.3f} s, "
            f"log={self._log_file}"
        )

    def _compute_force(self, elapsed: float) -> float:
        if self._waveform == "step":
            return self._magnitude if elapsed >= self._injection_delay else 0.0
        if self._waveform == "ramp":
            if elapsed < self._injection_delay:
                return 0.0
            return min(self._magnitude, (elapsed - self._injection_delay) * self._magnitude)
        if self._waveform == "sine":
            if elapsed < self._injection_delay:
                return 0.0
            return self._magnitude * np.sin(
                2.0 * np.pi * self._sine_freq * (elapsed - self._injection_delay)
            )
        return 0.0

    def _timer_cb(self) -> None:
        if self._done:
            if self._shutdown_requested and rclpy.ok():
                self._shutdown_requested = False
                try:
                    self._timer.cancel()
                except RuntimeError:
                    pass
                rclpy.shutdown()
            return

        if not self._pose_received:
            self._publish_wrench(np.zeros(3, dtype=np.float64))
            return

        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self._pose_ready_time is None:
            self._pose_ready_time = now
        if self._start_time is None:
            if now - self._pose_ready_time < self._ready_delay:
                self._publish_wrench(np.zeros(3, dtype=np.float64))
                return
            self._start_time = now
            self.get_logger().info("admittance pose stream detected, starting force schedule")

        elapsed = now - self._start_time
        force_off_time = self._injection_delay + self._step_duration
        total_duration = force_off_time + self._settle_duration

        if elapsed > total_duration:
            self._publish_wrench(np.zeros(3, dtype=np.float64))
            self._csv_f.flush()
            self._csv_f.close()
            self._done = True
            self._shutdown_requested = True
            self.get_logger().info(f"force injection complete, log saved to {self._log_file}")
            return

        magnitude = self._compute_force(elapsed if elapsed <= force_off_time else total_duration)
        if elapsed > force_off_time:
            magnitude = 0.0

        force = np.zeros(3, dtype=np.float64)
        if self._axis == "x":
            force[0] = magnitude
        elif self._axis == "y":
            force[1] = magnitude
        elif self._axis == "z":
            force[2] = magnitude
        elif self._axis == "xyz":
            force[:] = magnitude

        self._current_force = force
        self._publish_wrench(force)

    def _publish_wrench(self, force: np.ndarray) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])
        self._pub_wrench.publish(msg)

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self._done:
            return
        if not self._pose_received:
            self._pose_received = True
        if self._start_time is None:
            return

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        self._csv_w.writerow(
            [
                f"{timestamp:.6f}",
                f"{msg.pose.position.x:.6f}",
                f"{msg.pose.position.y:.6f}",
                f"{msg.pose.position.z:.6f}",
                f"{self._current_force[0]:.4f}",
                f"{self._current_force[1]:.4f}",
                f"{self._current_force[2]:.4f}",
                f"{self._base_state[0]:.6f}",
                f"{self._base_state[1]:.6f}",
                f"{self._base_state[2]:.6f}",
            ]
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._base_state[0] = msg.pose.pose.position.x
        self._base_state[1] = msg.pose.pose.position.y
        self._base_state[2] = _yaw_from_quaternion_xyzw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )


def main() -> None:
    rclpy.init()
    node = ArmForceInjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok():
            raise exc
    finally:
        if not node._csv_f.closed:
            node._csv_f.close()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    main()
