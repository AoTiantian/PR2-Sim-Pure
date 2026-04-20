#!/usr/bin/env python3
"""Automated validator for left-arm Cartesian admittance."""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import json
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class Metrics:
    peak_disp_x: float = 0.0
    peak_disp_y: float = 0.0
    peak_disp_z: float = 0.0
    peak_disp_xy: float = 0.0
    peak_disp_xyz: float = 0.0
    peak_left_effort: float = 0.0
    settle_time_sec: float = -1.0
    rms_disp_after_settle: float = 0.0
    effort_rms_after_settle: float = 0.0
    joint_state_count: int = 0


class Pr2ArmAdmittanceValidator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_admittance_validator")

        self.declare_parameter("wrench_topic", "wbc/arm_external_wrench")
        self.declare_parameter("ee_pose_topic", "ee_pose")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("expected_pose_frame_id", "")
        self.declare_parameter("control_target_frame_id", "")
        self.declare_parameter("left_joint_prefix", "l_")
        self.declare_parameter("arm_only_mode", False)
        self.declare_parameter("baseline_latch_mode", "at_force_start")

        self.declare_parameter("force_x", 30.0)
        self.declare_parameter("force_y", 0.0)
        self.declare_parameter("force_z", 0.0)
        self.declare_parameter("force_start_sec", 2.0)
        self.declare_parameter("duration_sec", 6.0)
        self.declare_parameter("settle_after_sec", 3.0)
        self.declare_parameter("force_schedule_json", "")

        self.declare_parameter("force_end_sec", -1.0)
        self.declare_parameter("total_duration_sec", -1.0)
        self.declare_parameter("min_peak_disp_xy", 0.0)
        self.declare_parameter("min_peak_disp_xyz", 0.012)
        self.declare_parameter("max_peak_disp_xy", 0.25)
        self.declare_parameter("max_peak_disp_xyz", 0.30)
        self.declare_parameter("max_peak_disp_z", 0.03)
        self.declare_parameter("max_allowed_left_effort", 120.0)
        self.declare_parameter("min_joint_state_count", 200)
        self.declare_parameter("settle_band_xy", 0.03)
        self.declare_parameter("settle_window_sec", 0.5)
        self.declare_parameter("max_settle_time_sec", 2.0)
        self.declare_parameter("max_rms_disp_after_settle", 0.02)
        self.declare_parameter("max_effort_rms_after_settle", 20.0)
        self.declare_parameter("pub_rate_hz", 30.0)

        self._wrench_topic = str(self.get_parameter("wrench_topic").value)
        self._ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        self._js_topic = str(self.get_parameter("joint_states_topic").value)
        self._jc_topic = str(self.get_parameter("joint_command_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._expected_pose_frame = str(self.get_parameter("expected_pose_frame_id").value).strip()
        self._control_target_frame = str(self.get_parameter("control_target_frame_id").value).strip()
        self._joint_prefix = str(self.get_parameter("left_joint_prefix").value)
        self._arm_only_mode = bool(self.get_parameter("arm_only_mode").value)
        self._baseline_mode = str(self.get_parameter("baseline_latch_mode").value).strip().lower()
        if self._baseline_mode not in ("at_startup", "at_force_start"):
            self.get_logger().warn(
                f"invalid baseline_latch_mode='{self._baseline_mode}', fallback to at_force_start"
            )
            self._baseline_mode = "at_force_start"
        self._schedule_raw = str(self.get_parameter("force_schedule_json").value).strip()
        self._fx = float(self.get_parameter("force_x").value)
        self._fy = float(self.get_parameter("force_y").value)
        self._fz = float(self.get_parameter("force_z").value)
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
            self._t_off = force_end_override if force_end_override >= 0.0 else (self._t_on + duration)
        self._t_total = total_override if total_override >= 0.0 else (self._t_off + settle_after)

        self._th_disp_xy = float(self.get_parameter("min_peak_disp_xy").value)
        self._th_disp_xyz = float(self.get_parameter("min_peak_disp_xyz").value)
        self._th_disp_xy_max = float(self.get_parameter("max_peak_disp_xy").value)
        self._th_disp_xyz_max = float(self.get_parameter("max_peak_disp_xyz").value)
        self._th_disp_z_max = float(self.get_parameter("max_peak_disp_z").value)
        self._max_effort = float(self.get_parameter("max_allowed_left_effort").value)
        self._th_js = int(self.get_parameter("min_joint_state_count").value)
        self._settle_band_xy = float(self.get_parameter("settle_band_xy").value)
        self._settle_window_sec = float(self.get_parameter("settle_window_sec").value)
        self._max_settle_time = float(self.get_parameter("max_settle_time_sec").value)
        self._max_rms_disp = float(self.get_parameter("max_rms_disp_after_settle").value)
        self._max_effort_rms = float(self.get_parameter("max_effort_rms_after_settle").value)
        hz = float(self.get_parameter("pub_rate_hz").value)
        self._dt = 1.0 / max(hz, 1.0)

        self._pub_wrench = self.create_publisher(WrenchStamped, self._wrench_topic, 10)
        self.create_subscription(PoseStamped, self._ee_pose_topic, self._on_ee_pose, 20)
        self.create_subscription(JointState, self._js_topic, self._on_joint_states, 20)
        self.create_subscription(JointState, self._jc_topic, self._on_joint_command, 20)

        self._metrics = Metrics()
        self._base_pose: Tuple[float, float, float] | None = None
        self._settled = False
        settle_n = max(1, int(round(self._settle_window_sec / max(self._dt, 1e-6))))
        self._settle_flags = deque(maxlen=settle_n)
        self._disp_sq_sum = 0.0
        self._disp_count = 0
        self._eff_sq_sum = 0.0
        self._eff_count = 0
        self._start_t = self.get_clock().now()
        self._done = False
        self._last_frame_warn_ns = 0
        self.create_timer(self._dt, self._tick)

        mode = "schedule" if self._use_schedule else "single-step"
        self.get_logger().info(
            "arm admittance validator started: "
            f"wrench={self._wrench_topic}, ee={self._ee_pose_topic}, jc={self._jc_topic}, "
            f"mode={mode}, force_window=[{self._t_on:.2f}, {self._t_off:.2f}]s, total={self._t_total:.2f}s, "
            f"baseline_mode={self._baseline_mode}, arm_only={self._arm_only_mode}, "
            f"frames(force={self._frame_id or 'unset'}, "
            f"disp={self._expected_pose_frame or 'msg.header.frame_id'}, "
            f"control={self._control_target_frame or 'unset'})"
        )

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
        return out

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_t).nanoseconds * 1e-9

    def _active_force(self, t: float) -> Tuple[float, float, float]:
        if self._use_schedule:
            for t0, t1, fx, fy, fz in self._segments:
                if t0 <= t <= t1:
                    return fx, fy, fz
            return 0.0, 0.0, 0.0
        if self._t_on <= t <= self._t_off:
            return self._fx, self._fy, self._fz
        return 0.0, 0.0, 0.0

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        if self._expected_pose_frame and str(msg.header.frame_id) != self._expected_pose_frame:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_frame_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    "ee pose frame mismatch in validator: "
                    f"expected={self._expected_pose_frame}, got={msg.header.frame_id}; drop sample"
                )
                self._last_frame_warn_ns = now_ns
            return
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if self._base_pose is None:
            if self._baseline_mode == "at_startup":
                self._base_pose = (x, y, z)
            elif self._elapsed() >= self._t_on:
                self._base_pose = (x, y, z)
            else:
                return

        if self._base_pose is None:
            return

        dx = x - self._base_pose[0]
        dy = y - self._base_pose[1]
        dz = z - self._base_pose[2]
        dxy = math.hypot(dx, dy)
        dxyz = math.sqrt(dx * dx + dy * dy + dz * dz)
        self._metrics.peak_disp_x = max(self._metrics.peak_disp_x, abs(dx))
        self._metrics.peak_disp_y = max(self._metrics.peak_disp_y, abs(dy))
        self._metrics.peak_disp_z = max(self._metrics.peak_disp_z, abs(dz))
        self._metrics.peak_disp_xy = max(self._metrics.peak_disp_xy, dxy)
        self._metrics.peak_disp_xyz = max(self._metrics.peak_disp_xyz, dxyz)

        t = self._elapsed()
        if t < self._t_off:
            return
        self._settle_flags.append(dxy <= self._settle_band_xy)
        if (not self._settled) and len(self._settle_flags) == self._settle_flags.maxlen and all(self._settle_flags):
            self._settled = True
            self._metrics.settle_time_sec = max(0.0, t - self._t_off)

        if self._settled:
            self._disp_sq_sum += dxy * dxy
            self._disp_count += 1

    def _on_joint_states(self, _msg: JointState) -> None:
        self._metrics.joint_state_count += 1

    def _on_joint_command(self, msg: JointState) -> None:
        peak = self._metrics.peak_left_effort
        sample_peak = 0.0
        for i, name in enumerate(msg.name):
            if not str(name).startswith(self._joint_prefix):
                continue
            if i < len(msg.effort):
                e = abs(float(msg.effort[i]))
                peak = max(peak, e)
                sample_peak = max(sample_peak, e)
        self._metrics.peak_left_effort = peak
        if self._elapsed() >= self._t_off:
            self._eff_sq_sum += sample_peak * sample_peak
            self._eff_count += 1

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

        pass_disp_xy = (
            True
            if self._th_disp_xy <= 0.0
            else (self._metrics.peak_disp_xy >= self._th_disp_xy)
        )
        pass_disp_xyz = (
            True
            if self._th_disp_xyz <= 0.0
            else (self._metrics.peak_disp_xyz >= self._th_disp_xyz)
        )
        pass_disp_xy_max = True if self._th_disp_xy_max <= 0.0 else (self._metrics.peak_disp_xy <= self._th_disp_xy_max)
        pass_disp_xyz_max = True if self._th_disp_xyz_max <= 0.0 else (self._metrics.peak_disp_xyz <= self._th_disp_xyz_max)
        pass_disp_z_max = True if self._th_disp_z_max <= 0.0 else (self._metrics.peak_disp_z <= self._th_disp_z_max)
        pass_effort = True if self._max_effort <= 0.0 else (self._metrics.peak_left_effort <= self._max_effort)
        pass_js = self._metrics.joint_state_count >= self._th_js
        self._metrics.rms_disp_after_settle = (
            math.sqrt(self._disp_sq_sum / self._disp_count) if self._disp_count > 0 else float("inf")
        )
        self._metrics.effort_rms_after_settle = (
            math.sqrt(self._eff_sq_sum / self._eff_count) if self._eff_count > 0 else float("inf")
        )
        pass_settle = (
            True
            if self._max_settle_time <= 0.0
            else (self._metrics.settle_time_sec >= 0.0 and self._metrics.settle_time_sec <= self._max_settle_time)
        )
        pass_rms_disp = (
            True
            if self._max_rms_disp <= 0.0
            else (self._metrics.rms_disp_after_settle <= self._max_rms_disp)
        )
        pass_effort_rms = (
            True
            if self._max_effort_rms <= 0.0
            else (self._metrics.effort_rms_after_settle <= self._max_effort_rms)
        )
        passed = (
            pass_disp_xy
            and pass_disp_xyz
            and pass_disp_xy_max
            and pass_disp_xyz_max
            and pass_disp_z_max
            and pass_effort
            and pass_settle
            and pass_rms_disp
            and pass_effort_rms
            and pass_js
        )

        self.get_logger().info(
            "arm admittance validation result: "
            f"{'PASS' if passed else 'FAIL'} | "
            f"peak|disp_x|={self._metrics.peak_disp_x:.4f}, "
            f"peak|disp_y|={self._metrics.peak_disp_y:.4f}, "
            f"peak|disp_z|={self._metrics.peak_disp_z:.4f} (max={self._th_disp_z_max:.4f}), "
            f"peak|disp_xy|={self._metrics.peak_disp_xy:.4f} (min={self._th_disp_xy:.4f}, max={self._th_disp_xy_max:.4f}), "
            f"peak|disp_xyz|={self._metrics.peak_disp_xyz:.4f} (min={self._th_disp_xyz:.4f}, max={self._th_disp_xyz_max:.4f}), "
            f"peak|left_effort|={self._metrics.peak_left_effort:.4f} (max={self._max_effort:.4f}), "
            f"settle_time={self._metrics.settle_time_sec:.4f}s (max={self._max_settle_time:.4f}s), "
            f"rms_disp_after_settle={self._metrics.rms_disp_after_settle:.4f} (max={self._max_rms_disp:.4f}), "
            f"effort_rms_after_settle={self._metrics.effort_rms_after_settle:.4f} (max={self._max_effort_rms:.4f}), "
            f"joint_states_count={self._metrics.joint_state_count} (th={self._th_js})"
        )
        self._done = True
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = Pr2ArmAdmittanceValidator()
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

