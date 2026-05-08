#!/usr/bin/env python3
"""PR2 底盘平面顺应控制器。"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pr2_mujoco_bridge.admittance_core import (
    AdmittanceState,
    AxisGains,
    ForceTrackingReferenceState,
    apply_deadband,
    clip_norm,
    settle_admittance_state,
)
from std_msgs.msg import Float64MultiArray


def _yaw_from_quaternion_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BaseAdmittanceNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_base_admittance")

        self.declare_parameter("input_topic", "wbc/external_wrench")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("output_topic", "wbc/reference/cmd_vel")
        self.declare_parameter("debug_topic", "wbc/base/admittance_debug")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("reference_mode", "fixed_equilibrium")
        self.declare_parameter("mass_linear", [18.0, 18.0])
        self.declare_parameter("damping_linear", [55.0, 55.0])
        self.declare_parameter("stiffness_linear", [32.0, 32.0])
        self.declare_parameter("mass_angular", 6.0)
        self.declare_parameter("damping_angular", 18.0)
        self.declare_parameter("stiffness_angular", 12.0)
        self.declare_parameter("wrench_filter_alpha", 0.2)
        self.declare_parameter("force_deadband", 0.2)
        self.declare_parameter("yaw_deadband", 0.04)
        self.declare_parameter("track_gain_linear", 3.2)
        self.declare_parameter("track_gain_angular", 2.6)
        self.declare_parameter("return_track_gain_linear", 5.0)
        self.declare_parameter("return_track_gain_angular", 3.4)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.28)
        self.declare_parameter("max_linear_displacement", 0.18)
        self.declare_parameter("max_yaw_displacement", 0.35)
        self.declare_parameter("force_reference_gain_linear", 0.020)
        self.declare_parameter("force_reference_gain_angular", 0.080)
        self.declare_parameter("force_reference_max_linear_speed", 0.18)
        self.declare_parameter("force_reference_max_angular_speed", 0.35)
        self.declare_parameter("force_reference_max_linear_displacement", 0.18)
        self.declare_parameter("force_reference_max_yaw_displacement", 0.35)
        self.declare_parameter("force_reference_idle_decay", 0.86)
        self.declare_parameter("force_reference_track_gain_linear", 3.8)
        self.declare_parameter("force_reference_track_gain_angular", 3.0)
        self.declare_parameter("settle_linear_displacement_epsilon", 0.004)
        self.declare_parameter("settle_yaw_displacement_epsilon", 0.008)
        self.declare_parameter("settle_linear_velocity_epsilon", 0.004)
        self.declare_parameter("settle_angular_velocity_epsilon", 0.01)
        self.declare_parameter("idle_velocity_decay", 1.0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._debug_topic = str(self.get_parameter("debug_topic").value)
        self._reference_mode = str(self.get_parameter("reference_mode").value)
        if self._reference_mode not in ("fixed_equilibrium", "force_tracking"):
            raise ValueError("reference_mode must be fixed_equilibrium or force_tracking")
        hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(hz, 1.0)

        self._gains = AxisGains(
            mass=np.array(
                list(self.get_parameter("mass_linear").value)
                + [self.get_parameter("mass_angular").value],
                dtype=np.float64,
            ),
            damping=np.array(
                list(self.get_parameter("damping_linear").value)
                + [self.get_parameter("damping_angular").value],
                dtype=np.float64,
            ),
            stiffness=np.array(
                list(self.get_parameter("stiffness_linear").value)
                + [self.get_parameter("stiffness_angular").value],
                dtype=np.float64,
            ),
        )
        self._wrench_filter_alpha = float(self.get_parameter("wrench_filter_alpha").value)
        self._force_deadband = float(self.get_parameter("force_deadband").value)
        self._yaw_deadband = float(self.get_parameter("yaw_deadband").value)
        self._track_gain_linear = float(self.get_parameter("track_gain_linear").value)
        self._track_gain_angular = float(self.get_parameter("track_gain_angular").value)
        self._return_track_gain_linear = float(
            self.get_parameter("return_track_gain_linear").value
        )
        self._return_track_gain_angular = float(
            self.get_parameter("return_track_gain_angular").value
        )
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._max_linear_displacement = float(
            self.get_parameter("max_linear_displacement").value
        )
        self._max_yaw_displacement = float(
            self.get_parameter("max_yaw_displacement").value
        )
        self._force_reference_gain = np.array(
            [
                self.get_parameter("force_reference_gain_linear").value,
                self.get_parameter("force_reference_gain_linear").value,
                self.get_parameter("force_reference_gain_angular").value,
            ],
            dtype=np.float64,
        )
        self._force_reference_max_velocity = np.array(
            [
                self.get_parameter("force_reference_max_linear_speed").value,
                self.get_parameter("force_reference_max_linear_speed").value,
                self.get_parameter("force_reference_max_angular_speed").value,
            ],
            dtype=np.float64,
        )
        self._force_reference_max_displacement = np.array(
            [
                self.get_parameter("force_reference_max_linear_displacement").value,
                self.get_parameter("force_reference_max_linear_displacement").value,
                self.get_parameter("force_reference_max_yaw_displacement").value,
            ],
            dtype=np.float64,
        )
        self._force_reference_idle_decay = float(
            self.get_parameter("force_reference_idle_decay").value
        )
        self._force_reference_track_gain_linear = float(
            self.get_parameter("force_reference_track_gain_linear").value
        )
        self._force_reference_track_gain_angular = float(
            self.get_parameter("force_reference_track_gain_angular").value
        )
        self._settle_disp_eps = np.array(
            [
                self.get_parameter("settle_linear_displacement_epsilon").value,
                self.get_parameter("settle_linear_displacement_epsilon").value,
                self.get_parameter("settle_yaw_displacement_epsilon").value,
            ],
            dtype=np.float64,
        )
        self._settle_vel_eps = np.array(
            [
                self.get_parameter("settle_linear_velocity_epsilon").value,
                self.get_parameter("settle_linear_velocity_epsilon").value,
                self.get_parameter("settle_angular_velocity_epsilon").value,
            ],
            dtype=np.float64,
        )
        self._idle_velocity_decay = float(self.get_parameter("idle_velocity_decay").value)

        self._raw_wrench_body = np.zeros(3, dtype=np.float64)
        self._filtered_wrench_body = np.zeros(3, dtype=np.float64)
        self._admittance_state = AdmittanceState.zeros(dim=3)
        self._force_ref_state = ForceTrackingReferenceState.zeros(dim=3)
        self._equilibrium_pose_world: np.ndarray | None = None
        self._current_pose_world: np.ndarray | None = None
        self._current_yaw = 0.0
        self._warned_frame = False

        self._pub_cmd = self.create_publisher(Twist, self._output_topic, 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, self._debug_topic, 10)
        self.create_subscription(WrenchStamped, self._input_topic, self._wrench_cb, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 20)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"底盘顺应控制器: mode={self._reference_mode}, "
            f"{self._input_topic} + {self._odom_topic} -> {self._output_topic}"
        )

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        if msg.header.frame_id not in ("", "base_link") and not self._warned_frame:
            self.get_logger().warn(
                f"期望底盘外力为 base_link，当前为 {msg.header.frame_id}，将按 base_link 解释"
            )
            self._warned_frame = True
        self._raw_wrench_body[:] = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.torque.z,
        ]

    def _odom_cb(self, msg: Odometry) -> None:
        yaw = _yaw_from_quaternion_xyzw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self._current_pose_world = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw],
            dtype=np.float64,
        )
        self._current_yaw = yaw
        if self._equilibrium_pose_world is None:
            self._equilibrium_pose_world = self._current_pose_world.copy()

    def _tick(self) -> None:
        if self._current_pose_world is None or self._equilibrium_pose_world is None:
            return

        cy = math.cos(self._current_yaw)
        sy = math.sin(self._current_yaw)
        rot_world_body = np.array([[cy, -sy], [sy, cy]], dtype=np.float64)

        if self._reference_mode == "force_tracking":
            raw_force_world = rot_world_body @ self._raw_wrench_body[:2]
            raw_wrench_world = np.array(
                [raw_force_world[0], raw_force_world[1], self._raw_wrench_body[2]],
                dtype=np.float64,
            )
            self._force_ref_state = self._force_ref_state.step(
                force=raw_wrench_world,
                dt=self._dt,
                force_deadband=np.array(
                    [self._force_deadband, self._force_deadband, self._yaw_deadband],
                    dtype=np.float64,
                ),
                filter_alpha=self._wrench_filter_alpha,
                force_to_velocity_gain=self._force_reference_gain,
                max_velocity=self._force_reference_max_velocity,
                max_displacement=self._force_reference_max_displacement,
                idle_velocity_decay=self._force_reference_idle_decay,
            )
            self._admittance_state = AdmittanceState(
                displacement=self._force_ref_state.reference.copy(),
                velocity=self._force_ref_state.velocity.copy(),
            )
            desired_pose_world = self._equilibrium_pose_world + self._admittance_state.displacement
            pose_error_world = desired_pose_world - self._current_pose_world
            pose_error_world[2] = _wrap_angle(pose_error_world[2])

            vel_world = (
                self._admittance_state.velocity[:2]
                + self._force_reference_track_gain_linear * pose_error_world[:2]
            )
            vel_world = clip_norm(vel_world, self._max_linear_speed)
            vel_body = rot_world_body.T @ vel_world
            wz = (
                self._admittance_state.velocity[2]
                + self._force_reference_track_gain_angular * pose_error_world[2]
            )
            wz = float(np.clip(wz, -self._max_angular_speed, self._max_angular_speed))

            cmd = Twist()
            cmd.linear.x = float(vel_body[0])
            cmd.linear.y = float(vel_body[1])
            cmd.angular.z = wz
            self._pub_cmd.publish(cmd)
            self._publish_debug(desired_pose_world, np.array([vel_body[0], vel_body[1], wz]))
            return

        self._filtered_wrench_body = (
            (1.0 - self._wrench_filter_alpha) * self._filtered_wrench_body
            + self._wrench_filter_alpha * self._raw_wrench_body
        )
        self._filtered_wrench_body[:2] = apply_deadband(
            self._filtered_wrench_body[:2],
            np.array([self._force_deadband, self._force_deadband], dtype=np.float64),
        )
        if abs(self._filtered_wrench_body[2]) < self._yaw_deadband:
            self._filtered_wrench_body[2] = 0.0
        return_mode = not np.any(np.abs(self._filtered_wrench_body) > 1.0e-9)

        force_world = rot_world_body @ self._filtered_wrench_body[:2]
        wrench_world = np.array(
            [force_world[0], force_world[1], self._filtered_wrench_body[2]],
            dtype=np.float64,
        )

        self._admittance_state = self._admittance_state.step(
            force=wrench_world,
            gains=self._gains,
            dt=self._dt,
        )

        disp = self._admittance_state.displacement.copy()
        disp[:2] = clip_norm(disp[:2], self._max_linear_displacement)
        disp[2] = float(
            np.clip(disp[2], -self._max_yaw_displacement, self._max_yaw_displacement)
        )
        vel = self._admittance_state.velocity.copy()
        vel[:2] = clip_norm(vel[:2], self._max_linear_speed)
        vel[2] = float(np.clip(vel[2], -self._max_angular_speed, self._max_angular_speed))
        self._admittance_state = settle_admittance_state(
            state=AdmittanceState(displacement=disp, velocity=vel),
            force=self._filtered_wrench_body,
            force_epsilon=np.full(3, 1.0e-9, dtype=np.float64),
            displacement_epsilon=self._settle_disp_eps,
            velocity_epsilon=self._settle_vel_eps,
            idle_velocity_decay=self._idle_velocity_decay,
        )

        desired_pose_world = self._equilibrium_pose_world + self._admittance_state.displacement
        pose_error_world = desired_pose_world - self._current_pose_world
        pose_error_world[2] = _wrap_angle(pose_error_world[2])

        track_gain_linear = (
            self._return_track_gain_linear if return_mode else self._track_gain_linear
        )
        track_gain_angular = (
            self._return_track_gain_angular if return_mode else self._track_gain_angular
        )
        vel_world = (
            self._admittance_state.velocity[:2]
            + track_gain_linear * pose_error_world[:2]
        )
        vel_world = clip_norm(vel_world, self._max_linear_speed)
        vel_body = rot_world_body.T @ vel_world

        wz = self._admittance_state.velocity[2] + track_gain_angular * pose_error_world[2]
        wz = float(np.clip(wz, -self._max_angular_speed, self._max_angular_speed))

        cmd = Twist()
        cmd.linear.x = float(vel_body[0])
        cmd.linear.y = float(vel_body[1])
        cmd.angular.z = wz
        self._pub_cmd.publish(cmd)
        self._publish_debug(desired_pose_world, np.array([vel_body[0], vel_body[1], wz]))

    def _publish_debug(self, base_ref_world: np.ndarray, base_vel_cmd: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [
            float(base_ref_world[0]),
            float(base_ref_world[1]),
            float(base_ref_world[2]),
            float(base_vel_cmd[0]),
            float(base_vel_cmd[1]),
            float(base_vel_cmd[2]),
        ]
        self._pub_debug.publish(msg)


def main() -> None:
    rclpy.init()
    node = BaseAdmittanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok():
            raise exc
    finally:
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
