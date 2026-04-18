#!/usr/bin/env python3
"""Automated validator for admittance pipeline response.

It publishes a virtual wrench step and checks whether:
- admittance acceleration responds
- final cmd_vel responds
Then prints PASS/FAIL and exits.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Accel, Twist, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class Metrics:
    max_abs_accel_x: float = 0.0
    max_abs_accel_y: float = 0.0
    max_planar_accel: float = 0.0
    max_abs_cmd_vx: float = 0.0
    max_abs_cmd_vy: float = 0.0
    max_planar_cmd_v: float = 0.0
    joint_state_count: int = 0


class Pr2AdmittanceValidator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_admittance_validator")

        self.declare_parameter("wrench_topic", "wbc/external_wrench")
        self.declare_parameter("accel_topic", "wbc/base_acceleration")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("force_x", 30.0)
        self.declare_parameter("force_y", 0.0)
        self.declare_parameter("force_z", 0.0)
        self.declare_parameter("force_start_sec", 2.0)
        self.declare_parameter("duration_sec", 3.0)
        self.declare_parameter("settle_after_sec", 3.0)
        # Optional multi-stage force schedule JSON. Example:
        # [
        #   {"start": 2.0, "end": 7.0, "fx": 80.0},
        #   {"start": 7.0, "end": 12.0, "fy": 80.0},
        #   {"start": 12.0, "end": 17.0, "fx": -80.0},
        #   {"start": 17.0, "end": 22.0, "fy": -80.0}
        # ]
        self.declare_parameter("force_schedule_json", "")
        # Optional override parameters; negative means "auto-compute".
        self.declare_parameter("force_end_sec", -1.0)
        self.declare_parameter("total_duration_sec", -1.0)
        self.declare_parameter("min_peak_accel_x", 0.12)
        self.declare_parameter("min_peak_cmd_vx", 0.06)
        self.declare_parameter("min_peak_accel_planar", 0.12)
        self.declare_parameter("min_peak_cmd_v_planar", 0.06)
        self.declare_parameter("min_joint_state_count", 200)
        self.declare_parameter("pub_rate_hz", 30.0)

        self._wrench_topic = str(self.get_parameter("wrench_topic").value)
        self._accel_topic = str(self.get_parameter("accel_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._js_topic = str(self.get_parameter("joint_states_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._fx = float(self.get_parameter("force_x").value)
        self._fy = float(self.get_parameter("force_y").value)
        self._fz = float(self.get_parameter("force_z").value)
        self._schedule_raw = str(self.get_parameter("force_schedule_json").value).strip()
        self._t_on = float(self.get_parameter("force_start_sec").value)
        duration = float(self.get_parameter("duration_sec").value)
        settle_after = float(self.get_parameter("settle_after_sec").value)
        force_end_override = float(self.get_parameter("force_end_sec").value)
        total_override = float(self.get_parameter("total_duration_sec").value)
        self._segments = self._parse_schedule(self._schedule_raw)
        self._use_schedule = len(self._segments) > 0
        if self._use_schedule:
            self._t_on = min(seg[0] for seg in self._segments)
            self._t_off = max(seg[1] for seg in self._segments)
        else:
            self._t_off = (
                force_end_override if force_end_override >= 0.0 else (self._t_on + duration)
            )
        self._t_total = (
            total_override if total_override >= 0.0 else (self._t_off + settle_after)
        )
        self._th_a = float(self.get_parameter("min_peak_accel_x").value)
        self._th_v = float(self.get_parameter("min_peak_cmd_vx").value)
        self._th_a_planar = float(self.get_parameter("min_peak_accel_planar").value)
        self._th_v_planar = float(self.get_parameter("min_peak_cmd_v_planar").value)
        self._th_js = int(self.get_parameter("min_joint_state_count").value)
        rate_hz = float(self.get_parameter("pub_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._pub_wrench = self.create_publisher(WrenchStamped, self._wrench_topic, 10)
        self.create_subscription(Accel, self._accel_topic, self._on_accel, 20)
        self.create_subscription(Twist, self._cmd_topic, self._on_cmd, 20)
        self.create_subscription(JointState, self._js_topic, self._on_joint_states, 20)

        self._metrics = Metrics()
        self._start_t = self.get_clock().now()
        self._done = False
        self.create_timer(self._dt, self._tick)
        mode = "schedule" if self._use_schedule else "single-step"
        self.get_logger().info(
            "admittance validator started: "
            f"wrench={self._wrench_topic}, accel={self._accel_topic}, cmd={self._cmd_topic}, js={self._js_topic}, "
            f"mode={mode}, force_window=[{self._t_on:.2f}, {self._t_off:.2f}]s, total={self._t_total:.2f}s"
        )

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_t).nanoseconds * 1e-9

    def _parse_schedule(self, raw: str) -> List[Tuple[float, float, float, float, float]]:
        if not raw:
            return []
        try:
            items = json.loads(raw)
        except Exception as exc:
            self.get_logger().warn(f"invalid force_schedule_json, fallback to single-step: {exc}")
            return []

        if not isinstance(items, list):
            self.get_logger().warn("force_schedule_json must be a JSON list, fallback to single-step")
            return []

        out: List[Tuple[float, float, float, float, float]] = []
        for i, item in enumerate(items):
            if not isinstance(item, dict):
                self.get_logger().warn(f"schedule[{i}] is not an object, ignored")
                continue
            try:
                t0 = float(item.get("start", 0.0))
                t1 = float(item.get("end", 0.0))
                fx = float(item.get("fx", 0.0))
                fy = float(item.get("fy", 0.0))
                fz = float(item.get("fz", 0.0))
            except Exception as exc:
                self.get_logger().warn(f"schedule[{i}] parse failed, ignored: {exc}")
                continue
            if t1 <= t0:
                self.get_logger().warn(f"schedule[{i}] has end<=start, ignored")
                continue
            out.append((t0, t1, fx, fy, fz))

        out.sort(key=lambda x: x[0])
        if not out:
            self.get_logger().warn("force_schedule_json has no valid segment, fallback to single-step")
        return out

    def _active_force(self, t: float) -> Tuple[float, float, float]:
        if self._use_schedule:
            for t0, t1, fx, fy, fz in self._segments:
                if t0 <= t <= t1:
                    return fx, fy, fz
            return 0.0, 0.0, 0.0
        if self._t_on <= t <= self._t_off:
            return self._fx, self._fy, self._fz
        return 0.0, 0.0, 0.0

    def _on_accel(self, msg: Accel) -> None:
        ax = abs(float(msg.linear.x))
        ay = abs(float(msg.linear.y))
        axy = math.hypot(float(msg.linear.x), float(msg.linear.y))
        self._metrics.max_abs_accel_x = max(self._metrics.max_abs_accel_x, ax)
        self._metrics.max_abs_accel_y = max(self._metrics.max_abs_accel_y, ay)
        self._metrics.max_planar_accel = max(self._metrics.max_planar_accel, axy)

    def _on_cmd(self, msg: Twist) -> None:
        vx = abs(float(msg.linear.x))
        vy = abs(float(msg.linear.y))
        vxy = math.hypot(float(msg.linear.x), float(msg.linear.y))
        self._metrics.max_abs_cmd_vx = max(self._metrics.max_abs_cmd_vx, vx)
        self._metrics.max_abs_cmd_vy = max(self._metrics.max_abs_cmd_vy, vy)
        self._metrics.max_planar_cmd_v = max(self._metrics.max_planar_cmd_v, vxy)

    def _on_joint_states(self, _msg: JointState) -> None:
        self._metrics.joint_state_count += 1

    def _publish_wrench(self, fx: float, fy: float, fz: float) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        self._pub_wrench.publish(msg)

    def _tick(self) -> None:
        if self._done:
            return

        t = self._elapsed()
        fx, fy, fz = self._active_force(t)
        self._publish_wrench(fx, fy, fz)

        if t < self._t_total:
            return

        if self._use_schedule:
            pass_accel = self._metrics.max_planar_accel >= self._th_a_planar
            pass_cmd = self._metrics.max_planar_cmd_v >= self._th_v_planar
        else:
            pass_accel = self._metrics.max_abs_accel_x >= self._th_a
            pass_cmd = self._metrics.max_abs_cmd_vx >= self._th_v
        pass_js = self._metrics.joint_state_count >= self._th_js
        passed = pass_accel and pass_cmd and pass_js

        self.get_logger().info(
            "admittance validation result: "
            f"{'PASS' if passed else 'FAIL'} | "
            f"peak|accel_x|={self._metrics.max_abs_accel_x:.4f} (th={self._th_a:.4f}), "
            f"peak|accel_y|={self._metrics.max_abs_accel_y:.4f}, "
            f"peak|accel_xy|={self._metrics.max_planar_accel:.4f} (th={self._th_a_planar:.4f}), "
            f"peak|cmd_vx|={self._metrics.max_abs_cmd_vx:.4f} (th={self._th_v:.4f}), "
            f"peak|cmd_vy|={self._metrics.max_abs_cmd_vy:.4f}, "
            f"peak|cmd_vxy|={self._metrics.max_planar_cmd_v:.4f} (th={self._th_v_planar:.4f}), "
            f"joint_states_count={self._metrics.joint_state_count} (th={self._th_js})"
        )
        self._done = True
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = Pr2AdmittanceValidator()
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

