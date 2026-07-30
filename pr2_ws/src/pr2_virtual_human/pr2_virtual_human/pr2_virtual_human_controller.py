#!/usr/bin/env python3
"""
PID-based virtual human controller for human-robot co-transport.

A virtual "human hand" at the far end of a long board tracks a desired
trajectory.  The PID computes the force needed to minimise tracking error.

The force is applied directly to the board body in MuJoCo via xfrc_applied
(at the hand end), so the board physics naturally transmits the force to
the EE.  The QP admittance runs in hold mode (K=0) to yield naturally.

Conceptual model
----------------
  p_hand = p_ee + R_ee @ [board_half_length+offset_x, 0, 0]^T
  e      = p_desired - p_hand
  F      = Kp*e + Ki*integral(e)*dt + Kd*de/dt

The force is applied to the board at p_hand (xfrc_applied), not at the EE.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from enum import Enum, auto
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _quat_xyzw_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    r = np.empty((3, 3), dtype=np.float64)
    r[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    r[0, 1] = 2.0 * (qx * qy - qw * qz)
    r[0, 2] = 2.0 * (qx * qz + qw * qy)
    r[1, 0] = 2.0 * (qx * qy + qw * qz)
    r[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    r[1, 2] = 2.0 * (qy * qz - qw * qx)
    r[2, 0] = 2.0 * (qx * qz - qw * qy)
    r[2, 1] = 2.0 * (qy * qz + qw * qx)
    r[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    return r


def _clamp_vec(v: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.maximum(lo, np.minimum(hi, v))


# ---------------------------------------------------------------------------
# state machine
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE = auto()
    LATCH = auto()
    TRACKING = auto()
    HOLD = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
# controller node
# ---------------------------------------------------------------------------

class Pr2VirtualHumanController(Node):
    """PID virtual human — force applied at board hand end via xfrc_applied."""

    def __init__(self) -> None:
        super().__init__("pr2_virtual_human_controller")

        # ---- parameters ---------------------------------------------------
        self.declare_parameter("ee_pose_topic", "ee_pose")
        self.declare_parameter("hand_force_topic", "virtual_human/hand_force")
        self.declare_parameter("wrench_topic", "wbc/whole_body_wrench")
        self.declare_parameter("board_grasped_topic", "mujoco/board_grasped")
        self.declare_parameter("hand_force_frame_id", "odom")
        self.declare_parameter("wrench_frame_id", "base_link")
        self.declare_parameter("rate_hz", 50.0)

        # board geometry
        self.declare_parameter("board_half_length", 0.5)
        self.declare_parameter("board_offset_in_ee", [0.18, 0.0, 0.0])

        # trajectory
        self.declare_parameter("trajectory_mode", "linear_x")
        self.declare_parameter("trajectory_speed", 0.05)
        self.declare_parameter("trajectory_amplitude", 0.5)
        self.declare_parameter("trajectory_period", 10.0)
        self.declare_parameter("trajectory_direction", [1.0, 0.0, 0.0])
        self.declare_parameter("trajectory_start_delay", 2.0)
        self.declare_parameter("trajectory_hold_duration", 2.0)
        self.declare_parameter("max_tracking_duration", 12.0)

        # PID — much lower gains because force is applied directly to the
        # lightweight (1.8 kg) board body via xfrc_applied, not the QP
        # admittance model (5 kg + 320 Ns/m damping).
        self.declare_parameter("pid_enable", True)
        self.declare_parameter("pid_kp", [80.0, 200.0, 180.0])
        self.declare_parameter("pid_ki", [4.0, 8.0, 10.0])
        self.declare_parameter("pid_kd", [12.0, 25.0, 28.0])
        self.declare_parameter("pid_max_integral", [20.0, 25.0, 30.0])
        self.declare_parameter("pid_max_force", [50.0, 50.0, 70.0])
        self.declare_parameter("pid_deriv_lpf_alpha", 0.8)
        self.declare_parameter("force_deadzone", 0.3)
        self.declare_parameter("log_path", "")
        self.declare_parameter("plot_script", "/workspace/scripts/regenerate_plots.py")

        # ---- read parameters ----------------------------------------------
        self._ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        self._hand_force_topic = str(self.get_parameter("hand_force_topic").value)
        self._wrench_topic = str(self.get_parameter("wrench_topic").value)
        self._board_grasped_topic = str(self.get_parameter("board_grasped_topic").value)
        self._hand_force_frame_id = str(self.get_parameter("hand_force_frame_id").value)
        self._wrench_frame_id = str(self.get_parameter("wrench_frame_id").value)
        hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(hz, 1.0)

        self._board_half_length = float(self.get_parameter("board_half_length").value)
        self._board_offset_in_ee = np.array(
            list(self.get_parameter("board_offset_in_ee").value), dtype=np.float64
        )

        self._trajectory_mode = str(self.get_parameter("trajectory_mode").value)
        self._trajectory_speed = float(self.get_parameter("trajectory_speed").value)
        self._trajectory_amplitude = float(self.get_parameter("trajectory_amplitude").value)
        self._trajectory_period = float(self.get_parameter("trajectory_period").value)
        self._trajectory_direction = np.array(
            list(self.get_parameter("trajectory_direction").value), dtype=np.float64
        )
        self._trajectory_start_delay = float(self.get_parameter("trajectory_start_delay").value)
        self._trajectory_hold_duration = float(self.get_parameter("trajectory_hold_duration").value)
        self._max_tracking_duration = float(self.get_parameter("max_tracking_duration").value)

        self._pid_enable = bool(self.get_parameter("pid_enable").value)
        self._pid_kp = np.array(list(self.get_parameter("pid_kp").value), dtype=np.float64)
        self._pid_ki = np.array(list(self.get_parameter("pid_ki").value), dtype=np.float64)
        self._pid_kd = np.array(list(self.get_parameter("pid_kd").value), dtype=np.float64)
        self._pid_max_integral = np.array(list(self.get_parameter("pid_max_integral").value), dtype=np.float64)
        self._pid_max_force = np.array(list(self.get_parameter("pid_max_force").value), dtype=np.float64)
        self._pid_deriv_lpf_alpha = float(self.get_parameter("pid_deriv_lpf_alpha").value)
        self._force_deadzone = float(self.get_parameter("force_deadzone").value)
        self._log_path = str(self.get_parameter("log_path").value)
        self._plot_script = str(self.get_parameter("plot_script").value)

        # validate
        for name, arr in [
            ("pid_kp", self._pid_kp), ("pid_ki", self._pid_ki),
            ("pid_kd", self._pid_kd), ("pid_max_integral", self._pid_max_integral),
            ("pid_max_force", self._pid_max_force),
        ]:
            if arr.shape != (3,):
                raise RuntimeError(f"{name} must have 3 elements")

        # ---- state --------------------------------------------------------
        self._state = State.IDLE
        self._latest_pose: Optional[PoseStamped] = None
        self._board_grasped: bool = False
        self._latched_hand_pos: Optional[np.ndarray] = None
        self._t_state_enter: float = 0.0
        self._t_start: Optional[float] = None

        # PID state
        self._error_integral = np.zeros(3, dtype=np.float64)
        self._prev_error = np.zeros(3, dtype=np.float64)
        self._deriv_filt = np.zeros(3, dtype=np.float64)
        self._hand_pos = np.zeros(3, dtype=np.float64)
        self._des_pos = np.zeros(3, dtype=np.float64)
        self._shutdown_pending = False

        # ---- subscribers --------------------------------------------------
        self.create_subscription(PoseStamped, self._ee_pose_topic, self._cb_ee_pose, 10)
        self.create_subscription(Bool, self._board_grasped_topic, self._cb_board_grasped, 10)

        # ---- publishers ---------------------------------------------------
        self._pub_hand_force = self.create_publisher(WrenchStamped, self._hand_force_topic, 10)
        self._pub_wrench = self.create_publisher(WrenchStamped, self._wrench_topic, 10)

        # ---- timer --------------------------------------------------------
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"PID virtual human started: mode={self._trajectory_mode}, "
            f"kp={self._pid_kp}, kd={self._pid_kd}, ki={self._pid_ki}, "
            f"board_half_length={self._board_half_length:.2f}m, "
            f"hand_force→{self._hand_force_topic}, "
            f"ee_wrench→{self._wrench_topic}"
        )

    # -----------------------------------------------------------------------
    # callbacks
    # -----------------------------------------------------------------------

    def _cb_ee_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _cb_board_grasped(self, msg: Bool) -> None:
        prev = self._board_grasped
        self._board_grasped = bool(msg.data)
        if prev and not self._board_grasped:
            self.get_logger().warn("board grasp lost — returning to IDLE")
            self._transition_to(State.IDLE)

    # -----------------------------------------------------------------------
    # hand position
    # -----------------------------------------------------------------------

    def _compute_hand_position(self) -> np.ndarray:
        p = self._latest_pose.pose.position
        q = self._latest_pose.pose.orientation
        p_ee = np.array([p.x, p.y, p.z], dtype=np.float64)
        R_ee = _quat_xyzw_to_rotmat(float(q.x), float(q.y), float(q.z), float(q.w))
        offset = R_ee @ np.array([
            self._board_half_length + self._board_offset_in_ee[0],
            self._board_offset_in_ee[1],
            self._board_offset_in_ee[2],
        ], dtype=np.float64)
        return p_ee + offset

    # -----------------------------------------------------------------------
    # trajectory
    # -----------------------------------------------------------------------

    def _compute_desired_position(self, t_phase: float) -> np.ndarray:
        p0 = self._latched_hand_pos
        mode = self._trajectory_mode

        if mode == "hold":
            return p0.copy()
        if mode == "periodic":
            direction = self._trajectory_direction.copy()
            n = float(np.linalg.norm(direction))
            if n < 1e-9:
                direction = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                direction = direction / n
            offset = direction * self._trajectory_amplitude * math.sin(
                2.0 * math.pi * t_phase / max(self._trajectory_period, 1e-9))
            return p0 + offset
        if mode in ("linear_x", "linear_y", "linear_xy"):
            speed = self._trajectory_speed
            if mode == "linear_x":
                direction = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            elif mode == "linear_y":
                direction = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            else:
                direction = self._trajectory_direction.copy()
                n = float(np.linalg.norm(direction))
                if n < 1e-9:
                    direction = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                else:
                    direction = direction / n
            return p0 + direction * speed * t_phase
        if mode == "circle":
            R = self._trajectory_amplitude
            T = max(self._trajectory_period, 1e-9)
            omega = 2.0 * math.pi / T
            p = p0.copy()
            p[0] = float(p0[0]) + R * (1.0 - math.cos(omega * t_phase))
            p[1] = float(p0[1]) + R * math.sin(omega * t_phase)
            return p
        if mode == "sine":
            speed = self._trajectory_speed
            A = self._trajectory_amplitude
            T = max(self._trajectory_period, 1e-9)
            omega = 2.0 * math.pi / T
            p = p0.copy()
            p[0] = float(p0[0]) + speed * t_phase
            p[1] = float(p0[1]) + A * math.sin(omega * t_phase)
            return p
        return p0.copy()

    # -----------------------------------------------------------------------
    # PID
    # -----------------------------------------------------------------------

    def _compute_pid_force(self, error: np.ndarray) -> np.ndarray:
        if not self._pid_enable:
            return np.zeros(3, dtype=np.float64)

        dt = max(self._dt, 1e-6)
        self._error_integral = self._error_integral + error * dt
        self._error_integral = _clamp_vec(
            self._error_integral, -self._pid_max_integral, self._pid_max_integral)

        deriv_raw = (error - self._prev_error) / dt
        alpha_val = np.clip(self._pid_deriv_lpf_alpha, 0.0, 1.0)
        self._deriv_filt = alpha_val * deriv_raw + (1.0 - alpha_val) * self._deriv_filt
        self._prev_error = error.copy()

        force = (
            self._pid_kp * error
            + self._pid_ki * self._error_integral
            + self._pid_kd * self._deriv_filt
        )
        force = _clamp_vec(force, -self._pid_max_force, self._pid_max_force)
        return force

    # -----------------------------------------------------------------------
    # ee-equivalent wrench
    # -----------------------------------------------------------------------

    def _compute_ee_wrench(self, hand_force: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Convert hand force to equivalent wrench at the EE."""
        if self._latest_pose is None:
            return hand_force, np.zeros(3, dtype=np.float64)

        q = self._latest_pose.pose.orientation
        R_ee = _quat_xyzw_to_rotmat(float(q.x), float(q.y), float(q.z), float(q.w))
        r_ee_to_hand = R_ee @ np.array([
            self._board_half_length + self._board_offset_in_ee[0],
            self._board_offset_in_ee[1],
            self._board_offset_in_ee[2],
        ], dtype=np.float64)
        torque = np.cross(r_ee_to_hand, hand_force)
        return hand_force, torque

    # -----------------------------------------------------------------------
    # state machine
    # -----------------------------------------------------------------------

    def _transition_to(self, new_state: State) -> None:
        if self._state == new_state:
            return
        old = self._state
        self._state = new_state
        self._t_state_enter = self._elapsed()
        self.get_logger().info(f"state: {old.name} → {new_state.name}")
        if new_state == State.IDLE:
            self._error_integral.fill(0.0)
            self._prev_error.fill(0.0)
            self._deriv_filt.fill(0.0)
            self._latched_hand_pos = None

    def _elapsed(self) -> float:
        if self._t_start is None:
            return 0.0
        return time.monotonic() - self._t_start

    def _do_shutdown(self) -> None:
        self.get_logger().info("[virt_human] exiting")
        # Auto-generate plots if log path was provided
        if self._log_path and os.path.exists(self._log_path):
            script = self._plot_script
            if os.path.exists(script):
                try:
                    subprocess.run(
                        ["python3", script, self._log_path],
                        timeout=10,
                        capture_output=True,
                    )
                    self.get_logger().info(f"plots regenerated from {self._log_path}")
                except Exception as e:
                    self.get_logger().warn(f"plot script failed: {e}")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def _update_state(self) -> None:
        now = self._elapsed()
        if self._state == State.IDLE:
            if self._board_grasped:
                self._transition_to(State.LATCH)
        elif self._state == State.LATCH:
            if not self._board_grasped:
                self._transition_to(State.IDLE); return
            if (now - self._t_state_enter) >= self._trajectory_start_delay:
                self._transition_to(State.TRACKING)
        elif self._state == State.TRACKING:
            if not self._board_grasped:
                self._transition_to(State.IDLE); return
            if (now - self._t_state_enter) >= self._max_tracking_duration:
                self._transition_to(State.HOLD)
        elif self._state == State.HOLD:
            if (now - self._t_state_enter) >= self._trajectory_hold_duration:
                self._transition_to(State.DONE)
        elif self._state == State.DONE:
            pass

    # -----------------------------------------------------------------------
    # output
    # -----------------------------------------------------------------------

    def _publish_forces(self, force: np.ndarray) -> None:
        now = self.get_clock().now().to_msg()

        # 1) Hand force → MuJoCo xfrc_applied on board body (odom/world frame)
        hf = WrenchStamped()
        hf.header.stamp = now
        hf.header.frame_id = self._hand_force_frame_id
        hf.wrench.force.x = float(force[0])
        hf.wrench.force.y = float(force[1])
        hf.wrench.force.z = float(force[2])
        self._pub_hand_force.publish(hf)

        # 2) EE wrench → QP admittance controller (base_link frame, force only)
        #    The PID force at the hand transmits through the rigid board to
        #    the EE.  Publishing as pure force (no torque) in base_link lets
        #    the QP rotate it correctly and drive the robot actively.
        wr = WrenchStamped()
        wr.header.stamp = now
        wr.header.frame_id = self._wrench_frame_id
        wr.wrench.force.x = float(force[0])
        wr.wrench.force.y = float(force[1])
        wr.wrench.force.z = float(force[2])
        self._pub_wrench.publish(wr)

    # -----------------------------------------------------------------------
    # main tick
    # -----------------------------------------------------------------------

    def _tick(self) -> None:
        if self._t_start is None and self._latest_pose is not None:
            self._t_start = time.monotonic()

        if self._latest_pose is None:
            return

        self._hand_pos = self._compute_hand_position()

        if self._state in (State.IDLE, State.LATCH):
            self._des_pos = self._hand_pos.copy()
            if self._state == State.LATCH:
                self._latched_hand_pos = self._hand_pos.copy()
        elif self._state == State.TRACKING:
            t_phase = self._elapsed() - self._t_state_enter
            self._des_pos = self._compute_desired_position(t_phase)
        elif self._state == State.HOLD:
            pass
        elif self._state == State.DONE:
            self._publish_forces(np.zeros(3, dtype=np.float64))
            return

        error = self._des_pos - self._hand_pos

        if self._state in (State.TRACKING, State.HOLD):
            force = self._compute_pid_force(error)
            if float(np.linalg.norm(force)) < self._force_deadzone:
                force = np.zeros(3, dtype=np.float64)
        else:
            force = np.zeros(3, dtype=np.float64)

        self._publish_forces(force)

        # Log ~2 Hz
        now_m = time.monotonic()
        if int(now_m * 2) != int((now_m - self._dt) * 2):
            mag = float(np.linalg.norm(force))
            self.get_logger().info(
                f"[virt_human] state={self._state.name} "
                f"|F|={mag:.1f}N "
                f"F=[{force[0]:.1f}, {force[1]:.1f}, {force[2]:.1f}] "
                f"|e|={float(np.linalg.norm(error)):.4f}m "
                f"p_hand=[{self._hand_pos[0]:.3f}, {self._hand_pos[1]:.3f}, {self._hand_pos[2]:.3f}] "
                f"p_des=[{self._des_pos[0]:.3f}, {self._des_pos[1]:.3f}, {self._des_pos[2]:.3f}]"
            )

        self._update_state()

        if self._state == State.DONE and not self._shutdown_pending:
            self._shutdown_pending = True
            self.get_logger().info("[virt_human] trajectory complete, shutting down in 0.5s")
            self.create_timer(0.5, self._do_shutdown)


def main() -> None:
    rclpy.init()
    node = Pr2VirtualHumanController()
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
