#!/usr/bin/env python3
"""Endpoint-impedance virtual human for the PR2/board transport demo.

The controller closes the loop on the measured board-end pose published by
MuJoCo.  Its output is applied only to the board; the robot controller receives
the independently measured wrist F/T signal.
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
import time
from datetime import datetime
from enum import Enum, auto
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


def _vec3(node: Node, name: str) -> np.ndarray:
    value = np.asarray(list(node.get_parameter(name).value), dtype=np.float64)
    if value.shape != (3,):
        raise RuntimeError(f"{name} must contain exactly three values")
    return value


def _quat_to_rotmat(quaternion: np.ndarray) -> np.ndarray:
    """Convert a ROS/MuJoCo-order quaternion [w, x, y, z] to R_world_body."""
    q = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotmat_to_quat(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to [w, x, y, z]."""
    r = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = 2.0 * float(np.sqrt(trace + 1.0))
        q = np.array(
            [
                0.25 * s,
                (r[2, 1] - r[1, 2]) / s,
                (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s,
            ],
            dtype=np.float64,
        )
    else:
        diagonal = np.diag(r)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = 2.0 * float(np.sqrt(max(1.0 + r[0, 0] - r[1, 1] - r[2, 2], 1.0e-12)))
            q = np.array(
                [
                    (r[2, 1] - r[1, 2]) / s,
                    0.25 * s,
                    (r[0, 1] + r[1, 0]) / s,
                    (r[0, 2] + r[2, 0]) / s,
                ],
                dtype=np.float64,
            )
        elif index == 1:
            s = 2.0 * float(np.sqrt(max(1.0 + r[1, 1] - r[0, 0] - r[2, 2], 1.0e-12)))
            q = np.array(
                [
                    (r[0, 2] - r[2, 0]) / s,
                    (r[0, 1] + r[1, 0]) / s,
                    0.25 * s,
                    (r[1, 2] + r[2, 1]) / s,
                ],
                dtype=np.float64,
            )
        else:
            s = 2.0 * float(np.sqrt(max(1.0 + r[2, 2] - r[0, 0] - r[1, 1], 1.0e-12)))
            q = np.array(
                [
                    (r[1, 0] - r[0, 1]) / s,
                    (r[0, 2] + r[2, 0]) / s,
                    (r[1, 2] + r[2, 1]) / s,
                    0.25 * s,
                ],
                dtype=np.float64,
            )
    q /= max(float(np.linalg.norm(q)), 1.0e-12)
    return q


def _rotvec_to_rotmat(rotvec: np.ndarray) -> np.ndarray:
    """Rodrigues exponential map for a world-frame rotation vector."""
    v = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(v))
    if angle < 1.0e-10:
        skew = np.array(
            [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
            dtype=np.float64,
        )
        return np.eye(3) + skew
    axis = v / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _rotmat_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    """SO(3) logarithm, returning the shortest rotation vector."""
    r = np.asarray(rotation, dtype=np.float64)
    cosine = float(np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle < 1.0e-7:
        return 0.5 * vee
    sine = float(np.sin(angle))
    if abs(sine) > 1.0e-6:
        return angle / (2.0 * sine) * vee
    # The trajectory amplitudes are small, but retain a stable pi fallback.
    axis = np.sqrt(np.maximum((np.diag(r) + 1.0) * 0.5, 0.0))
    largest = int(np.argmax(axis))
    if axis[largest] < 1.0e-8:
        return np.zeros(3, dtype=np.float64)
    if largest == 0:
        axis[1] = np.copysign(axis[1], r[0, 1] + r[1, 0])
        axis[2] = np.copysign(axis[2], r[0, 2] + r[2, 0])
    elif largest == 1:
        axis[0] = np.copysign(axis[0], r[0, 1] + r[1, 0])
        axis[2] = np.copysign(axis[2], r[1, 2] + r[2, 1])
    else:
        axis[0] = np.copysign(axis[0], r[0, 2] + r[2, 0])
        axis[1] = np.copysign(axis[1], r[1, 2] + r[2, 1])
    axis /= max(float(np.linalg.norm(axis)), 1.0e-12)
    return angle * axis


def _pose_quaternion(msg: PoseStamped) -> np.ndarray:
    q = msg.pose.orientation
    return np.array([float(q.w), float(q.x), float(q.y), float(q.z)], dtype=np.float64)


def _orientation_error_world(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Rotation vector taking the current body frame to target in world frame."""
    return _rotmat_to_rotvec(_quat_to_rotmat(target) @ _quat_to_rotmat(current).T)


class State(Enum):
    WAITING = auto()
    LATCH = auto()
    TRACKING = auto()
    HOLD = auto()
    DONE = auto()


class Pr2VirtualHumanController(Node):
    def __init__(self) -> None:
        super().__init__("pr2_virtual_human_controller")

        self.declare_parameter("hand_pose_topic", "mujoco/human_hand_pose")
        self.declare_parameter("sim_time_topic", "mujoco/sim_time")
        self.declare_parameter("board_grasped_topic", "mujoco/board_grasped")
        self.declare_parameter("hand_force_topic", "virtual_human/hand_force")
        self.declare_parameter("hand_force_offset", [1.0, 0.0, 0.0])
        self.declare_parameter(
            "robot_support_force_topic", "mujoco/robot_support_force"
        )
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("require_board_grasped", True)

        self.declare_parameter("trajectory_start_delay", 1.0)
        self.declare_parameter(
            "fixed_target_mode",
            False,
        )
        self.declare_parameter("trajectory_position_enable", True)
        self.declare_parameter("trajectory_orientation_enable", True)
        self.declare_parameter("trajectory_x_amplitude", 0.20)
        self.declare_parameter("trajectory_y_amplitude", 0.12)
        self.declare_parameter("trajectory_z_amplitude", 0.0)
        self.declare_parameter("trajectory_roll_amplitude", 0.08)
        self.declare_parameter("trajectory_pitch_amplitude", 0.06)
        self.declare_parameter("trajectory_yaw_amplitude", 0.10)
        self.declare_parameter("trajectory_period", 12.0)
        self.declare_parameter("trajectory_ramp_duration", 1.0)
        self.declare_parameter("tracking_duration", 12.0)
        self.declare_parameter("hold_duration", 1.0)

        self.declare_parameter("board_mass", 1.8)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("automatic_vertical_balance_enable", False)
        self.declare_parameter("robot_contact_offset", [-1.0, 0.0, 0.0])
        # The human is modeled as a compliant endpoint, not as a controller
        # that observes or targets a robot-force percentage.  The desired hand
        # trajectory is the only reference used to generate the human wrench.
        self.declare_parameter("human_impedance_kp", [250.0, 250.0, 500.0])
        self.declare_parameter("human_impedance_ki", [0.0, 0.0, 5.0])
        self.declare_parameter("human_impedance_kd", [45.0, 45.0, 70.0])
        self.declare_parameter(
            "human_impedance_integral_limit", [0.10, 0.10, 0.20]
        )
        self.declare_parameter("human_force_limit", [35.0, 35.0, 45.0])
        self.declare_parameter(
            "human_force_slew_rate", [90.0, 90.0, 120.0]
        )
        self.declare_parameter("human_force_response_time", 0.08)
        self.declare_parameter("human_allow_vertical_pull", True)
        self.declare_parameter("robot_wrench_filter_alpha", 0.10)
        self.declare_parameter(
            "robot_wrench_is_board_on_robot",
            True,
        )
        # Board inertia is highly anisotropic: rotation about its long axis is
        # roughly 300x lighter than pitch/yaw. Keep gains axis-specific so the
        # thin-axis torque loop does not saturate and chatter.
        self.declare_parameter("orientation_kp", [0.20, 1.00, 1.00])
        self.declare_parameter("orientation_ki", [0.01, 0.05, 0.05])
        self.declare_parameter("orientation_kd", [0.03, 0.50, 0.50])
        self.declare_parameter("orientation_integral_limit", [0.05, 0.10, 0.10])
        self.declare_parameter("pid_max_torque", [0.30, 1.50, 1.50])
        self.declare_parameter("velocity_lpf_alpha", 0.15)
        self.declare_parameter("robot_wrench_topic", "mujoco/left_wrist_wrench")
        self.declare_parameter(
            "output_dir", "/workspace/results/pr2_virtual_human"
        )

        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self._dt = 1.0 / rate_hz
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._hand_force_offset = _vec3(self, "hand_force_offset")
        self._automatic_vertical_balance = bool(
            self.get_parameter("automatic_vertical_balance_enable").value
        )
        self._robot_contact_offset = _vec3(self, "robot_contact_offset")
        self._require_grasp = bool(
            self.get_parameter("require_board_grasped").value
        )
        self._start_delay = max(
            float(self.get_parameter("trajectory_start_delay").value), 0.0
        )
        self._trajectory_position_enable = bool(
            self.get_parameter("trajectory_position_enable").value
        )
        self._fixed_target_mode = bool(
            self.get_parameter("fixed_target_mode").value
        )
        self._trajectory_orientation_enable = bool(
            self.get_parameter("trajectory_orientation_enable").value
        )
        self._amp_x = float(
            self.get_parameter("trajectory_x_amplitude").value
        )
        self._amp_y = float(
            self.get_parameter("trajectory_y_amplitude").value
        )
        self._amp_z = float(
            self.get_parameter("trajectory_z_amplitude").value
        )
        self._amp_roll = float(
            self.get_parameter("trajectory_roll_amplitude").value
        )
        self._amp_pitch = float(
            self.get_parameter("trajectory_pitch_amplitude").value
        )
        self._amp_yaw = float(
            self.get_parameter("trajectory_yaw_amplitude").value
        )
        self._period = max(
            float(self.get_parameter("trajectory_period").value), 0.1
        )
        self._ramp_duration = max(
            float(self.get_parameter("trajectory_ramp_duration").value), 0.0
        )
        self._tracking_duration = max(
            float(self.get_parameter("tracking_duration").value), 0.1
        )
        self._hold_duration = max(
            float(self.get_parameter("hold_duration").value), 0.0
        )
        total_duration = (
            self._start_delay + self._tracking_duration + self._hold_duration
        )
        if total_duration > 15.0:
            raise RuntimeError(
                "collaborative demo must remain within 15 seconds; "
                f"configured duration is {total_duration:.2f}s"
            )

        self._human_kp = _vec3(self, "human_impedance_kp")
        self._human_ki = _vec3(self, "human_impedance_ki")
        self._human_kd = _vec3(self, "human_impedance_kd")
        self._human_integral_limit = _vec3(
            self, "human_impedance_integral_limit"
        )
        self._human_force_limit = _vec3(self, "human_force_limit")
        self._human_force_slew_rate = _vec3(self, "human_force_slew_rate")
        self._human_force_response_time = max(
            float(self.get_parameter("human_force_response_time").value),
            0.0,
        )
        self._human_allow_vertical_pull = bool(
            self.get_parameter("human_allow_vertical_pull").value
        )
        self._ori_kp = _vec3(self, "orientation_kp")
        self._ori_ki = _vec3(self, "orientation_ki")
        self._ori_kd = _vec3(self, "orientation_kd")
        self._ori_integral_limit = _vec3(
            self, "orientation_integral_limit"
        )
        self._max_torque = _vec3(self, "pid_max_torque")
        self._velocity_alpha = float(
            np.clip(
                float(self.get_parameter("velocity_lpf_alpha").value),
                0.0,
                1.0,
            )
        )
        self._board_mass = float(self.get_parameter("board_mass").value)
        self._gravity = abs(float(self.get_parameter("gravity").value))
        self._robot_wrench_filter_alpha = float(
            np.clip(
                float(self.get_parameter("robot_wrench_filter_alpha").value),
                0.0,
                1.0,
            )
        )
        self._robot_wrench_is_board_on_robot = bool(
            self.get_parameter("robot_wrench_is_board_on_robot").value
        )
        self._output_dir = str(self.get_parameter("output_dir").value)

        self._state = State.WAITING
        self._hand_pose: Optional[PoseStamped] = None
        self._sim_time: Optional[float] = None
        self._last_control_sim_time: Optional[float] = None
        self._grasped = False
        self._initial_position: Optional[np.ndarray] = None
        self._initial_quaternion: Optional[np.ndarray] = None
        self._desired_position = np.zeros(3)
        self._desired_velocity = np.zeros(3)
        self._desired_acceleration = np.zeros(3)
        self._desired_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self._angular_velocity = np.zeros(3)
        self._previous_position: Optional[np.ndarray] = None
        self._previous_position_time: Optional[float] = None
        self._previous_quaternion: Optional[np.ndarray] = None
        self._human_integral = np.zeros(3)
        self._orientation_integral = np.zeros(3)
        self._force = np.zeros(3)
        self._torque = np.zeros(3)
        self._robot_wrench = np.zeros(6)
        self._robot_wrench_filtered = np.zeros(6)
        self._robot_wrench_filter_initialized = False
        self._robot_support_force_z = 0.0
        self._human_force_target = np.zeros(3)
        self._human_force_slew = np.zeros(3)
        self._required_vertical_force = self._board_mass * self._gravity
        self._human_support_force_target_z = 0.0
        self._net_vertical_force_residual = 0.0
        self._robot_force_on_board = np.zeros(3)
        self._state_enter = time.monotonic()
        self._time_origin: Optional[float] = None
        self._history: list[dict[str, float]] = []
        self._results_saved = False
        self._shutdown_pending = False

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("hand_pose_topic").value),
            self._on_hand_pose,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("sim_time_topic").value),
            self._on_sim_time,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("board_grasped_topic").value),
            self._on_grasped,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("robot_wrench_topic").value),
            self._on_robot_wrench,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("robot_support_force_topic").value),
            self._on_robot_support_force,
            10,
        )
        self._force_pub = self.create_publisher(
            WrenchStamped,
            str(self.get_parameter("hand_force_topic").value),
            10,
        )
        self._desired_pub = self.create_publisher(
            PoseStamped, "virtual_human/desired_hand_pose", 10
        )
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            "virtual human ready: measured hand endpoint feedback, "
            f"xy_amplitude=({self._amp_x:.2f}, {self._amp_y:.2f})m, "
            f"z_height_hold={self._amp_z == 0.0}, "
            f"orientation_amplitude=({self._amp_roll:.3f}, "
            f"{self._amp_pitch:.3f}, {self._amp_yaw:.3f})rad, "
            f"duration={total_duration:.1f}s, "
            "human_model=endpoint_impedance, "
            f"human_force_response={self._human_force_response_time:.3f}s, "
            f"output_dir={self._output_dir}"
        )

    def _on_hand_pose(self, msg: PoseStamped) -> None:
        self._hand_pose = msg

    def _on_sim_time(self, msg: Float64) -> None:
        self._sim_time = float(msg.data)

    def _on_grasped(self, msg: Bool) -> None:
        self._grasped = bool(msg.data)

    def _on_robot_wrench(self, msg: WrenchStamped) -> None:
        raw = np.array([
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ], dtype=np.float64)
        self._robot_wrench[:] = raw
        if not self._robot_wrench_filter_initialized:
            self._robot_wrench_filtered[:] = raw
            self._robot_wrench_filter_initialized = True
        else:
            alpha = self._robot_wrench_filter_alpha
            self._robot_wrench_filtered[:] = (
                alpha * raw + (1.0 - alpha) * self._robot_wrench_filtered
            )
        if self._robot_wrench_is_board_on_robot:
            self._robot_force_on_board[:] = -self._robot_wrench_filtered[:3]
        else:
            self._robot_force_on_board[:] = self._robot_wrench_filtered[:3]

    def _on_robot_support_force(self, msg: Float64) -> None:
        """Store the robot endpoint vertical support command for diagnostics."""
        self._robot_support_force_z = float(msg.data)

    def _human_impedance_force(self, control_dt: float) -> np.ndarray:
        """Generate the virtual-human force from endpoint impedance only.

        The human has a preferred hand trajectory.  Deviations from that
        trajectory create a spring/damper response, just as they would for a
        compliant arm.  No robot force, load percentage, or model gravity
        compensation is fed into this command.  A finite response time and
        slew-rate limit model the fact that a person cannot change force
        instantaneously.
        """
        dt = max(float(control_dt), 1.0e-4)
        position_error = self._desired_position - self._position
        velocity_error = self._desired_velocity - self._velocity
        self._human_integral = np.clip(
            self._human_integral + position_error * dt,
            -self._human_integral_limit,
            self._human_integral_limit,
        )
        target = (
            self._human_kp * position_error
            + self._human_ki * self._human_integral
            + self._human_kd * velocity_error
        )
        if self._automatic_vertical_balance:
            self._human_support_force_target_z = (
                self._compute_human_vertical_support_target()
            )
            # The robot command is the exact residual of this support force.
            # Do not close a second z-position impedance loop on top of that
            # allocation: any z PID correction immediately changes the robot
            # residual in the opposite direction and creates a force-sharing
            # positive feedback (the observed 0↔45 N self-oscillation).
            self._human_integral[2] = 0.0
            target[2] = self._human_support_force_target_z
        if not self._automatic_vertical_balance:
            self._human_support_force_target_z = 0.0
        if self._fixed_target_mode:
            # In the fixed-pose load-measurement experiment the robot is the
            # position holder.  Feeding x/y position error back through the
            # human endpoint would make the human fight small robot/base
            # compliance and excite the coupled arm.  Keep only the automatic
            # vertical support (and the independent attitude torque).
            self._human_integral[:2] = 0.0
            target[:2] = 0.0
        if not self._human_allow_vertical_pull:
            target[2] = max(float(target[2]), 0.0)
        target = np.clip(target, -self._human_force_limit, self._human_force_limit)
        self._human_force_target[:] = target

        # Limit how quickly the hand force can change, then apply a small
        # first-order response lag.  Both operations are causal and use only
        # the human endpoint state.
        delta_limit = self._human_force_slew_rate * dt
        slew_target = self._force + np.clip(
            target - self._force, -delta_limit, delta_limit
        )
        if self._human_force_response_time <= 1.0e-9:
            response_alpha = 1.0
        else:
            response_alpha = dt / (self._human_force_response_time + dt)
        previous_force = self._force.copy()
        self._force[:] = previous_force + response_alpha * (
            slew_target - previous_force
        )
        self._human_force_slew[:] = (self._force - previous_force) / dt
        return self._force.copy()

    def _compute_human_vertical_support_target(self) -> float:
        """Solve the two-end static vertical force/moment balance.

        The split is obtained from the loaded board mass, gravity and the two
        contact offsets.  It is not a configurable 50% force reference:

          F_h + F_r = m(g + a_z),
          r_h,x F_h + r_r,x F_r = 0.

        For symmetric endpoints this naturally evaluates to half the required
        load at each endpoint.  If the geometry changes, the split changes
        automatically.
        """
        total_force = self._board_mass * (
            self._gravity + float(self._desired_acceleration[2])
        )
        # For the height-hold task, solve the static split from the initial
        # grasp geometry.  Re-solving from the instantaneous tilted board
        # creates a positive feedback loop: a small attitude error changes
        # the x lever arms, changes F_h,z, flips the robot residual, and then
        # drives a larger attitude/z error.  The target trajectory has no z
        # motion, so the nominal geometry is the physically relevant one.
        reference_quaternion = (
            self._initial_quaternion
            if self._initial_quaternion is not None
            else self._quaternion
        )
        board_rotation = _quat_to_rotmat(reference_quaternion)
        human_offset_world = board_rotation @ self._hand_force_offset
        robot_offset_world = board_rotation @ self._robot_contact_offset
        denominator = float(human_offset_world[0] - robot_offset_world[0])
        if abs(denominator) < 1.0e-6:
            return 0.5 * total_force
        human_force = (
            -float(robot_offset_world[0]) / denominator * total_force
        )
        return float(np.clip(human_force, 0.0, self._human_force_limit[2]))

    def _compose_hand_torque(self, orientation_torque: np.ndarray) -> np.ndarray:
        """Return only the explicit human orientation torque.

        The bridge separately adds the physical endpoint moment ``r×F``.
        No vertical-support moment is cancelled here.  If the two endpoint
        forces are balanced, their moments cancel naturally; if they are not,
        the board is allowed to rotate.
        """
        return np.asarray(orientation_torque, dtype=np.float64).copy()

    def _transition(self, state: State) -> None:
        if state == self._state:
            return
        old = self._state
        self._state = state
        self._state_enter = (
            self._sim_time if self._sim_time is not None else time.monotonic()
        )
        self.get_logger().info(f"state: {old.name} -> {state.name}")

    def _state_elapsed(self) -> float:
        now = self._sim_time if self._sim_time is not None else time.monotonic()
        return max(0.0, now - self._state_enter)

    def _update_trajectory(self, phase: float) -> None:
        assert self._initial_position is not None
        previous_velocity = self._desired_velocity.copy()
        ramp = min(self._ramp_duration, 0.5 * self._tracking_duration)
        total_angle = (
            2.0 * math.pi * self._tracking_duration / self._period
        )
        if ramp <= 0.0:
            angle_rate = total_angle / self._tracking_duration
            angle = angle_rate * phase
        else:
            cruise_rate = total_angle / (self._tracking_duration - ramp)
            if phase < ramp:
                angle_rate = cruise_rate * phase / ramp
                angle = 0.5 * cruise_rate * phase * phase / ramp
            elif phase > self._tracking_duration - ramp:
                remaining = max(self._tracking_duration - phase, 0.0)
                angle_rate = cruise_rate * remaining / ramp
                angle = (
                    total_angle
                    - 0.5 * cruise_rate * remaining * remaining / ramp
                )
            else:
                angle_rate = cruise_rate
                angle = cruise_rate * (phase - 0.5 * ramp)

        if self._trajectory_position_enable:
            self._desired_position = self._initial_position + np.array(
                [
                    self._amp_x * math.sin(angle),
                    self._amp_y * math.sin(2.0 * angle),
                    self._amp_z * math.sin(angle),
                ]
            )
            self._desired_velocity = np.array(
                [
                    self._amp_x * angle_rate * math.cos(angle),
                    2.0 * self._amp_y * angle_rate * math.cos(2.0 * angle),
                    self._amp_z * angle_rate * math.cos(angle),
                ]
            )
        else:
            self._desired_position = self._initial_position.copy()
            self._desired_velocity.fill(0.0)

        assert self._initial_quaternion is not None
        if self._trajectory_orientation_enable:
            relative_rotvec = np.array(
                [
                    self._amp_roll * math.sin(angle),
                    self._amp_pitch * math.sin(2.0 * angle),
                    self._amp_yaw * math.sin(angle),
                ],
                dtype=np.float64,
            )
            initial_rotation = _quat_to_rotmat(self._initial_quaternion)
            self._desired_quaternion = _rotmat_to_quat(
                initial_rotation @ _rotvec_to_rotmat(relative_rotvec)
            )
        else:
            self._desired_quaternion = self._initial_quaternion.copy()
        if self._dt > 1.0e-9:
            self._desired_acceleration = (
                self._desired_velocity - previous_velocity
            ) / self._dt

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = self._frame_id
        wrench.wrench.force.x = float(self._force[0])
        wrench.wrench.force.y = float(self._force[1])
        wrench.wrench.force.z = float(self._force[2])
        wrench.wrench.torque.x = float(self._torque[0])
        wrench.wrench.torque.y = float(self._torque[1])
        wrench.wrench.torque.z = float(self._torque[2])
        self._force_pub.publish(wrench)

        desired = PoseStamped()
        desired.header.stamp = stamp
        desired.header.frame_id = self._frame_id
        desired.pose.position.x = float(self._desired_position[0])
        desired.pose.position.y = float(self._desired_position[1])
        desired.pose.position.z = float(self._desired_position[2])
        desired.pose.orientation.w = float(self._desired_quaternion[0])
        desired.pose.orientation.x = float(self._desired_quaternion[1])
        desired.pose.orientation.y = float(self._desired_quaternion[2])
        desired.pose.orientation.z = float(self._desired_quaternion[3])
        self._desired_pub.publish(desired)

    def _record(self) -> None:
        if self._time_origin is None:
            self._time_origin = (
                self._sim_time if self._sim_time is not None else time.monotonic()
            )
        current_time = self._sim_time if self._sim_time is not None else time.monotonic()
        position_error = self._desired_position - self._position
        orientation_error = _orientation_error_world(
            self._quaternion, self._desired_quaternion
        )
        # Log the commanded and measured orientations in the same local
        # rotation-vector coordinates.  The previous CSV only contained the
        # orientation error, which made it impossible to tell what 3-D
        # orientation trajectory had actually been commanded.
        initial_rotation = _quat_to_rotmat(self._initial_quaternion)
        desired_rotation = _quat_to_rotmat(self._desired_quaternion)
        actual_rotation = _quat_to_rotmat(self._quaternion)
        desired_rotvec = _rotmat_to_rotvec(
            initial_rotation.T @ desired_rotation
        )
        actual_rotvec = _rotmat_to_rotvec(
            initial_rotation.T @ actual_rotation
        )
        self._required_vertical_force = self._board_mass * (
            self._gravity + float(self._desired_acceleration[2])
        )
        # The wrist sensor reports residual external wrench after tare; it does
        # not include the payload load already compensated inside the robot's
        # joint torque controller.  Keep this as an explicitly sensor-based
        # diagnostic only; it is never fed back into the human command.
        robot_support_magnitude = abs(float(self._robot_wrench_filtered[2]))
        self._net_vertical_force_residual = (
            float(self._force[2])
            + robot_support_magnitude
            - self._required_vertical_force
        )
        self._history.append(
            {
                "time": float(current_time - self._time_origin),
                "desired_x": float(self._desired_position[0]),
                "desired_y": float(self._desired_position[1]),
                "desired_z": float(self._desired_position[2]),
                "actual_x": float(self._position[0]),
                "actual_y": float(self._position[1]),
                "actual_z": float(self._position[2]),
                "force_x": float(self._force[0]),
                "force_y": float(self._force[1]),
                "force_z": float(self._force[2]),
                "force_norm": float(np.linalg.norm(self._force)),
                "human_force_target_x": float(self._human_force_target[0]),
                "human_force_target_y": float(self._human_force_target[1]),
                "human_force_target_z": float(self._human_force_target[2]),
                "human_support_force_target_z": float(
                    self._human_support_force_target_z
                ),
                "human_force_slew_x": float(self._human_force_slew[0]),
                "human_force_slew_y": float(self._human_force_slew[1]),
                "human_force_slew_z": float(self._human_force_slew[2]),
                "torque_x": float(self._torque[0]),
                "torque_y": float(self._torque[1]),
                "torque_z": float(self._torque[2]),
                "torque_norm": float(np.linalg.norm(self._torque)),
                "position_error_x": float(position_error[0]),
                "position_error_y": float(position_error[1]),
                "position_error_z": float(position_error[2]),
                "orientation_error_x": float(orientation_error[0]),
                "orientation_error_y": float(orientation_error[1]),
                "orientation_error_z": float(orientation_error[2]),
                "desired_orientation_x": float(desired_rotvec[0]),
                "desired_orientation_y": float(desired_rotvec[1]),
                "desired_orientation_z": float(desired_rotvec[2]),
                "actual_orientation_x": float(actual_rotvec[0]),
                "actual_orientation_y": float(actual_rotvec[1]),
                "actual_orientation_z": float(actual_rotvec[2]),
                "robot_force_x": float(self._robot_wrench[0]),
                "robot_force_y": float(self._robot_wrench[1]),
                "robot_force_z": float(self._robot_wrench[2]),
                "robot_torque_x": float(self._robot_wrench[3]),
                "robot_torque_y": float(self._robot_wrench[4]),
                "robot_torque_z": float(self._robot_wrench[5]),
                "robot_wrench_norm": float(np.linalg.norm(self._robot_wrench)),
                "robot_force_on_board_x": float(self._robot_force_on_board[0]),
                "robot_force_on_board_y": float(self._robot_force_on_board[1]),
                "robot_force_on_board_z": float(self._robot_force_on_board[2]),
                "robot_support_force_command_z": float(self._robot_support_force_z),
                "required_vertical_force": float(self._required_vertical_force),
                "wrench_based_vertical_residual": float(
                    self._net_vertical_force_residual
                ),
            }
        )

    def _tick(self) -> None:
        if self._hand_pose is None or self._sim_time is None:
            return
        if self._sim_time == self._last_control_sim_time:
            return
        now = self._sim_time
        self._last_control_sim_time = now
        p = self._hand_pose.pose.position
        position = np.array([p.x, p.y, p.z], dtype=np.float64)
        quaternion = _pose_quaternion(self._hand_pose)
        control_dt = self._dt
        if (
            self._previous_position is not None
            and self._previous_position_time is not None
        ):
            control_dt = float(
                np.clip(now - self._previous_position_time, 0.005, 0.1)
            )
            measured_velocity = (
                position - self._previous_position
            ) / control_dt
            self._velocity = (
                self._velocity_alpha * measured_velocity
                + (1.0 - self._velocity_alpha) * self._velocity
            )
        self._previous_position = position.copy()
        self._previous_position_time = now
        self._position = position
        if self._previous_quaternion is not None:
            relative_rotation = _quat_to_rotmat(quaternion) @ _quat_to_rotmat(
                self._previous_quaternion
            ).T
            measured_angular_velocity = _rotmat_to_rotvec(relative_rotation) / control_dt
            self._angular_velocity = (
                self._velocity_alpha * measured_angular_velocity
                + (1.0 - self._velocity_alpha) * self._angular_velocity
            )
        self._previous_quaternion = quaternion.copy()
        self._quaternion = quaternion

        ready = self._grasped or not self._require_grasp
        if self._state == State.WAITING:
            self._force.fill(0.0)
            self._human_force_target.fill(0.0)
            self._human_force_slew.fill(0.0)
            self._human_integral.fill(0.0)
            self._torque.fill(0.0)
            # The physical-contact scene starts with the board already between
            # the two robot pads. Publish the solved human support force even
            # before the grasp flag arrives, so startup never applies the
            # complete payload at only the robot endpoint.
            if self._automatic_vertical_balance:
                self._desired_acceleration.fill(0.0)
                self._human_support_force_target_z = (
                    self._compute_human_vertical_support_target()
                )
                self._force[2] = self._human_support_force_target_z
                self._human_force_target[2] = self._human_support_force_target_z
            if ready:
                self._initial_position = position.copy()
                self._initial_quaternion = quaternion.copy()
                self._desired_position = position.copy()
                self._desired_quaternion = quaternion.copy()
                self._time_origin = now
                self._transition(State.LATCH)
        elif self._state == State.LATCH:
            self._initial_position = position.copy()
            self._initial_quaternion = quaternion.copy()
            self._desired_position = position.copy()
            self._desired_quaternion = quaternion.copy()
            # In a physical-contact scene the board cannot be left with only
            # the robot's endpoint force during latch: that would create a
            # real one-sided r×F moment before tracking starts.  Start the
            # automatically solved two-end support forces immediately.  The
            # non-balance legacy mode retains the old zero-force latch.
            if self._automatic_vertical_balance:
                self._desired_acceleration.fill(0.0)
                self._human_support_force_target_z = (
                    self._compute_human_vertical_support_target()
                )
                # Do not pass the latch velocity transient through the human
                # impedance damper. It would create an artificial force spike
                # before tracking starts and destroy the intended force split.
                self._force.fill(0.0)
                self._force[2] = self._human_support_force_target_z
                self._human_force_target[:] = self._force
                self._human_force_slew.fill(0.0)
            else:
                self._force.fill(0.0)
                self._human_force_target.fill(0.0)
                self._human_force_slew.fill(0.0)
                self._human_integral.fill(0.0)
            self._torque.fill(0.0)
            if not ready:
                self._transition(State.WAITING)
            elif self._state_elapsed() >= self._start_delay:
                self._human_integral.fill(0.0)
                self._orientation_integral.fill(0.0)
                self._transition(State.TRACKING)
        elif self._state == State.TRACKING:
            if not ready:
                self.get_logger().warn("board grasp lost")
                self._transition(State.WAITING)
            else:
                phase = self._state_elapsed()
                self._update_trajectory(phase)
                self._human_impedance_force(control_dt)
                orientation_error = _orientation_error_world(
                    self._quaternion, self._desired_quaternion
                )
                self._orientation_integral = np.clip(
                    self._orientation_integral + orientation_error * control_dt,
                    -self._ori_integral_limit,
                    self._ori_integral_limit,
                )
                orientation_torque = np.clip(
                    self._ori_kp * orientation_error
                    + self._ori_ki * self._orientation_integral
                    - self._ori_kd * self._angular_velocity,
                    -self._max_torque,
                    self._max_torque,
                )
                self._torque = self._compose_hand_torque(orientation_torque)
                self._record()
                if phase >= self._tracking_duration:
                    self._desired_velocity.fill(0.0)
                    self._transition(State.HOLD)
        elif self._state == State.HOLD:
            self._human_impedance_force(control_dt)
            orientation_error = _orientation_error_world(
                self._quaternion, self._desired_quaternion
            )
            orientation_torque = np.clip(
                self._ori_kp * orientation_error
                - self._ori_kd * self._angular_velocity,
                -self._max_torque,
                self._max_torque,
            )
            self._torque = self._compose_hand_torque(orientation_torque)
            self._record()
            if self._state_elapsed() >= self._hold_duration:
                self._transition(State.DONE)
        elif self._state == State.DONE:
            self._force.fill(0.0)
            self._torque.fill(0.0)
            if not self._shutdown_pending:
                self._shutdown_pending = True
                self._save_results()
                self.create_timer(0.25, self._shutdown)

        self._publish()
        if int(time.monotonic() * 2) != int(
            (time.monotonic() - self._dt) * 2
        ):
            error = self._desired_position - self._position
            self.get_logger().info(
                f"[virtual_human] {self._state.name} "
                f"|e|={np.linalg.norm(error):.4f}m "
                f"F={np.round(self._force, 2).tolist()}N"
            )

    def _save_results(self) -> None:
        if self._results_saved:
            return
        if len(self._history) < 2:
            self.get_logger().warn("not enough tracking samples to plot")
            return
        run_dir = os.path.join(
            self._output_dir, datetime.now().strftime("run_%Y%m%d_%H%M%S")
        )
        os.makedirs(run_dir, exist_ok=True)
        csv_path = os.path.join(run_dir, "history.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=list(self._history[0].keys())
            )
            writer.writeheader()
            writer.writerows(self._history)

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        values = {
            key: np.asarray([row[key] for row in self._history])
            for key in self._history[0]
        }
        stable_start = max(
            float(values["time"][-1]) - max(self._hold_duration, 1.0),
            float(values["time"][0]),
        )
        stable_mask = values["time"] >= stable_start
        full_mask = np.ones_like(stable_mask, dtype=bool)
        position_rmse = lambda mask: {
            axis: float(
                np.sqrt(
                    np.mean(values[f"position_error_{axis}"][mask] ** 2)
                )
            )
            for axis in ("x", "y", "z")
        }
        orientation_rmse = lambda mask: {
            axis: float(
                np.sqrt(
                    np.mean(values[f"orientation_error_{axis}"][mask] ** 2)
                )
            )
            for axis in ("x", "y", "z")
        }
        metrics = {
            "stable_window_start_sec": stable_start,
            "stable_window_end_sec": float(values["time"][-1]),
            "position_rmse_m": position_rmse(stable_mask),
            "orientation_rmse_rad": orientation_rmse(stable_mask),
            "full_trajectory_position_rmse_m": position_rmse(full_mask),
            "full_trajectory_orientation_rmse_rad": orientation_rmse(full_mask),
            "human_force_mean": {
                component: float(
                    np.mean(values[f"force_{component}"][stable_mask])
                )
                for component in ("x", "y", "z")
            },
            "human_force_rms": {
                component: float(
                    np.sqrt(np.mean(values[f"force_{component}"][stable_mask] ** 2))
                )
                for component in ("x", "y", "z")
            },
            "human_force_peak_abs": {
                component: float(
                    np.max(np.abs(values[f"force_{component}"][stable_mask]))
                )
                for component in ("x", "y", "z")
            },
            "human_force_rate_rms": {
                component: float(
                    np.sqrt(
                        np.mean(values[f"human_force_slew_{component}"][stable_mask] ** 2)
                    )
                )
                for component in ("x", "y", "z")
            },
            "robot_support_force_command_z_mean": float(
                np.mean(values["robot_support_force_command_z"][stable_mask])
            ),
            "robot_support_force_command_z_rms": float(
                np.sqrt(
                    np.mean(values["robot_support_force_command_z"][stable_mask] ** 2)
                )
            ),
            "human_support_force_target_z_mean": float(
                np.mean(values["human_support_force_target_z"][stable_mask])
            ),
            "robot_wrench_mean": {
                component: float(
                    np.mean(values[f"robot_{component}"][stable_mask])
                )
                for component in (
                    "force_x", "force_y", "force_z",
                    "torque_x", "torque_y", "torque_z",
                )
            },
            "robot_wrench_rms": {
                component: float(
                    np.sqrt(np.mean(values[f"robot_{component}"][stable_mask] ** 2))
                )
                for component in (
                    "force_x", "force_y", "force_z",
                    "torque_x", "torque_y", "torque_z",
                )
            },
        }
        metrics_path = os.path.join(run_dir, "metrics.json")
        with open(metrics_path, "w", encoding="utf-8") as stream:
            json.dump(metrics, stream, ensure_ascii=False, indent=2)
        fig, (ax_xy, ax_z) = plt.subplots(
            1, 2, figsize=(10.0, 4.6), constrained_layout=True
        )
        ax_xy.plot(
            values["desired_x"],
            values["desired_y"],
            "--",
            label="Desired path",
        )
        ax_xy.plot(
            values["actual_x"],
            values["actual_y"],
            label="Actual path",
        )
        ax_xy.set_xlabel("World X position (m)")
        ax_xy.set_ylabel("World Y position (m)")
        ax_xy.set_title("Horizontal XY Path")
        ax_xy.set_aspect("equal", adjustable="box")
        ax_xy.grid(alpha=0.25)
        ax_xy.legend()

        ax_z.plot(
            values["time"],
            values["desired_z"],
            "--",
            label="Desired height",
        )
        ax_z.plot(
            values["time"],
            values["actual_z"],
            label="Actual height",
        )
        ax_z.set_xlabel("Tracking time (s)")
        ax_z.set_ylabel("World Z position (m)")
        ax_z.set_title("Z Height Hold")
        z_center = float(np.mean(values["desired_z"]))
        z_extent = max(
            float(np.max(np.abs(values["actual_z"] - z_center))),
            0.01,
        )
        ax_z.set_ylim(z_center - 1.25 * z_extent, z_center + 1.25 * z_extent)
        ax_z.grid(alpha=0.25)
        ax_z.legend()
        fig.suptitle("Human-Robot Horizontal Tracking with Z Height Hold")
        trajectory_png = os.path.join(run_dir, "trajectory_comparison.png")
        fig.savefig(trajectory_png, dpi=300, bbox_inches="tight")
        plt.close(fig)

        fig, axes = plt.subplots(3, 1, figsize=(6.4, 6.8), sharex=True)
        for axis, component, color in zip(
            axes,
            ("x", "y", "z"),
            ("#0072B2", "#D55E00", "#009E73"),
        ):
            axis.plot(
                values["time"],
                values[f"force_{component}"],
                color=color,
            )
            axis.set_ylabel(f"$F_{component}$ (N)")
            axis.grid(alpha=0.25)
        axes[-1].set_xlabel("Tracking time (s)")
        fig.suptitle("Virtual-Human Applied Force Components")
        force_png = os.path.join(run_dir, "applied_force_xyz.png")
        fig.savefig(force_png, dpi=300, bbox_inches="tight")
        plt.close(fig)

        # Six-dimensional pose tracking error: xyz in metres and the SO(3)
        # logarithm error in radians.  The rotation-vector representation avoids
        # Euler-angle wrapping at ±pi and is directly suitable for a torque PID.
        fig, axes = plt.subplots(
            3, 2, figsize=(10.0, 7.2), sharex=True, constrained_layout=True
        )
        for col, component, color in zip(
            axes[:, 0], ("x", "y", "z"), ("#0072B2", "#D55E00", "#009E73")
        ):
            col.plot(
                values["time"],
                values[f"position_error_{component}"],
                color=color,
            )
            col.set_ylabel(f"$e_{component}$ (m)")
            col.grid(alpha=0.25)
        for col, component, color in zip(
            axes[:, 1], ("x", "y", "z"), ("#CC79A7", "#E69F00", "#56B4E9")
        ):
            col.plot(
                values["time"],
                values[f"orientation_error_{component}"],
                color=color,
            )
            col.set_ylabel(f"$e_{{R{component}}}$ (rad)")
            col.grid(alpha=0.25)
        axes[-1, 0].set_xlabel("Tracking time (s)")
        axes[-1, 1].set_xlabel("Tracking time (s)")
        axes[0, 0].set_title("Position error")
        axes[0, 1].set_title("Orientation error (rotation vector)")
        pose_error_png = os.path.join(run_dir, "pose_tracking_error_6d.png")
        fig.suptitle("Six-Dimensional Pose Tracking Error")
        fig.savefig(pose_error_png, dpi=300, bbox_inches="tight")
        plt.close(fig)

        # Show the commanded and measured 6-D pose itself, not just the
        # error.  Position is plotted in metres and the orientation is the
        # rotation vector relative to the latched initial board pose.
        fig, axes = plt.subplots(
            3, 2, figsize=(10.0, 7.2), sharex=True, constrained_layout=True
        )
        for row, component, color in zip(
            axes[:, 0], ("x", "y", "z"), ("#0072B2", "#D55E00", "#009E73")
        ):
            row.plot(
                values["time"],
                values[f"desired_{component}"],
                "--",
                color=color,
                label="Desired",
            )
            row.plot(
                values["time"],
                values[f"actual_{component}"],
                color="#222222",
                label="Actual",
            )
            row.set_ylabel(f"{component} (m)")
            row.grid(alpha=0.25)
        for row, component, color in zip(
            axes[:, 1], ("x", "y", "z"), ("#CC79A7", "#E69F00", "#56B4E9")
        ):
            row.plot(
                values["time"],
                values[f"desired_orientation_{component}"],
                "--",
                color=color,
                label="Desired",
            )
            row.plot(
                values["time"],
                values[f"actual_orientation_{component}"],
                color="#222222",
                label="Actual",
            )
            row.set_ylabel(f"r{component} (rad)")
            row.grid(alpha=0.25)
        axes[0, 0].set_title("Position")
        axes[0, 1].set_title("Orientation rotation vector")
        axes[-1, 0].set_xlabel("Tracking time (s)")
        axes[-1, 1].set_xlabel("Tracking time (s)")
        axes[0, 0].legend()
        axes[0, 1].legend()
        pose_comparison_png = os.path.join(
            run_dir, "pose_tracking_6d_comparison.png"
        )
        fig.suptitle("Desired vs Actual Six-Dimensional Pose")
        fig.savefig(pose_comparison_png, dpi=300, bbox_inches="tight")
        plt.close(fig)

        # Compare the virtual-human wrench with the measured robot-side wrench.
        fig, axes = plt.subplots(
            3, 2, figsize=(10.0, 7.2), sharex=True, constrained_layout=True
        )
        for row, component in enumerate(("x", "y", "z")):
            axes[row, 0].plot(
                values["time"],
                values[f"force_{component}"],
                label="Human",
                color="#D55E00",
            )
            axes[row, 0].plot(
                values["time"],
                values[f"robot_force_{component}"],
                label="Robot",
                color="#0072B2",
            )
            axes[row, 0].set_ylabel(f"$F_{component}$ (N)")
            axes[row, 1].plot(
                values["time"],
                values[f"torque_{component}"],
                label="Human",
                color="#D55E00",
            )
            axes[row, 1].plot(
                values["time"],
                values[f"robot_torque_{component}"],
                label="Robot",
                color="#0072B2",
            )
            axes[row, 1].set_ylabel(rf"$\tau_{component}$ (N m)")
            axes[row, 0].grid(alpha=0.25)
            axes[row, 1].grid(alpha=0.25)
        axes[0, 0].set_title("Force components")
        axes[0, 1].set_title("Torque components")
        axes[-1, 0].set_xlabel("Tracking time (s)")
        axes[-1, 1].set_xlabel("Tracking time (s)")
        axes[0, 0].legend()
        axes[0, 1].legend()
        wrench_png = os.path.join(run_dir, "human_robot_wrench.png")
        fig.suptitle("Human and Robot Applied Wrench")
        fig.savefig(wrench_png, dpi=300, bbox_inches="tight")
        plt.close(fig)
        self.get_logger().info(
            f"saved results: {csv_path}, {trajectory_png}, {force_png}, "
            f"{pose_error_png}, {pose_comparison_png}, {wrench_png}, "
            f"{metrics_path}"
        )
        self._results_saved = True

    def _shutdown(self) -> None:
        self.get_logger().info("virtual human demo complete")
        self.destroy_node()
        rclpy.shutdown()
        raise SystemExit


def main() -> None:
    rclpy.init()
    node = Pr2VirtualHumanController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node._save_results()
        except Exception as exc:
            node.get_logger().error(
                "failed to save partial run results during shutdown: "
                f"{type(exc).__name__}: {exc}"
            )
        if node.context.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
