#!/usr/bin/env python3
"""PR2 末端力注入与响应记录节点。

在 /wbc/arm/external_wrench 上发布可配置波形的力（step/ramp/sine），
同时将末端位姿响应记录到 CSV 文件，供 plot_arm_response.py 分析。
"""

from __future__ import annotations

import csv

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ArmForceInjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_force_injector")

        self.declare_parameter("force_axis", "x",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))  # x/y/z (avoid YAML bool parsing)
        self.declare_parameter("force_magnitude", 10.0)     # N
        self.declare_parameter("step_duration", 3.0)        # s
        self.declare_parameter("injection_delay", 1.0)      # s, 等待仿真稳定
        self.declare_parameter("log_file", "/tmp/arm_response.csv")
        self.declare_parameter("publish_rate", 100.0)       # Hz
        self.declare_parameter("waveform", "step",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("sine_freq", 0.5)            # Hz
        self.declare_parameter("wrench_topic", "wbc/arm/external_wrench")
        self.declare_parameter("ee_pose_topic", "wbc/arm/ee_pose_log")

        self._axis = str(self.get_parameter("force_axis").value).lower()
        self._magnitude = float(self.get_parameter("force_magnitude").value)
        self._step_dur = float(self.get_parameter("step_duration").value)
        self._delay = float(self.get_parameter("injection_delay").value)
        self._log_file = str(self.get_parameter("log_file").value)
        rate = float(self.get_parameter("publish_rate").value)
        self._waveform = str(self.get_parameter("waveform").value)
        self._sine_freq = float(self.get_parameter("sine_freq").value)
        wrench_topic = self.get_parameter("wrench_topic").value
        pose_topic = self.get_parameter("ee_pose_topic").value

        self._pub_wrench = self.create_publisher(WrenchStamped, wrench_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 20)

        os.makedirs(os.path.dirname(self._log_file) or ".", exist_ok=True)
        self._csv_f = open(self._log_file, "w", newline="")
        self._csv_w = csv.writer(self._csv_f)
        self._csv_w.writerow(
            ["timestamp", "pos_x", "pos_y", "pos_z",
             "force_x", "force_y", "force_z"])

        self._current_force = np.zeros(3)
        self._start_time: float | None = None
        self._done = False
        self._pose_received = False   # 等收到首个 pose 后才开始计时

        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)
        self.get_logger().info(
            f"力注入器启动: axis={self._axis}, magnitude={self._magnitude}N, "
            f"waveform={self._waveform}, delay={self._delay}s, "
            f"duration={self._step_dur}s, log={self._log_file}"
        )

    def _compute_force(self, elapsed: float) -> float:
        if self._waveform == "step":
            return self._magnitude if elapsed >= self._delay else 0.0
        elif self._waveform == "ramp":
            if elapsed < self._delay:
                return 0.0
            return min(self._magnitude,
                       (elapsed - self._delay) * self._magnitude)
        elif self._waveform == "sine":
            if elapsed < self._delay:
                return 0.0
            return self._magnitude * np.sin(
                2.0 * np.pi * self._sine_freq * (elapsed - self._delay))
        return 0.0

    def _timer_cb(self) -> None:
        if self._done:
            return

        # 等待 admittance 节点激活（收到第一个 pose 后）
        if not self._pose_received:
            self._publish_wrench(np.zeros(3))   # 保持零力
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._start_time is None:
            self._start_time = now
            self.get_logger().info(
                'admittance ready, 开始注入计时')

        elapsed = now - self._start_time
        total_dur = self._delay + self._step_dur

        if elapsed > total_dur:
            self._publish_wrench(np.zeros(3))
            self._csv_f.flush()
            self._csv_f.close()
            self._done = True
            self.get_logger().info(
                f"力注入完成，日志已保存至 {self._log_file}")
            return

        mag = self._compute_force(elapsed)
        f = np.zeros(3)
        if self._axis == "x":
            f[0] = mag
        elif self._axis == "y":
            f[1] = mag
        elif self._axis == "z":
            f[2] = mag

        self._current_force = f
        self._publish_wrench(f)

    def _publish_wrench(self, f: np.ndarray) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(f[0])
        msg.wrench.force.y = float(f[1])
        msg.wrench.force.z = float(f[2])
        self._pub_wrench.publish(msg)

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self._done:
            return
        if not self._pose_received:
            self._pose_received = True
        # 只在开始计时后记录
        if self._start_time is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._csv_w.writerow([
            f"{t:.6f}",
            f"{msg.pose.position.x:.6f}",
            f"{msg.pose.position.y:.6f}",
            f"{msg.pose.position.z:.6f}",
            f"{self._current_force[0]:.4f}",
            f"{self._current_force[1]:.4f}",
            f"{self._current_force[2]:.4f}",
        ])


def main() -> None:
    rclpy.init()
    node = ArmForceInjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node._csv_f.closed:
            node._csv_f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
