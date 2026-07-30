#!/usr/bin/env python3
"""ROS 2 MuJoCo demo containing only a virtual human force and a free board."""

from __future__ import annotations

import csv
from datetime import datetime
import os
import time

import glfw
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class HumanBoardSim(Node):
    """Apply a virtual human wrench directly to a board in MuJoCo."""

    def __init__(self) -> None:
        super().__init__("human_board_sim")

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene_human_board.xml",
        )
        self.declare_parameter("use_viewer", bool(os.environ.get("DISPLAY", "").strip()))
        self.declare_parameter("force_x", 0.0)
        self.declare_parameter("force_y", 0.0)
        # A negative value selects exact gravity compensation.
        self.declare_parameter("force_z", -1.0)
        self.declare_parameter("hand_offset", 1.0)
        self.declare_parameter("allow_rotation", False)
        self.declare_parameter("pid_enable", True)
        self.declare_parameter("trajectory_start_delay", 1.0)
        self.declare_parameter("trajectory_x_amplitude", 0.45)
        self.declare_parameter("trajectory_y_amplitude", 0.30)
        self.declare_parameter("trajectory_period", 10.0)
        self.declare_parameter("pid_kp", [80.0, 80.0, 60.0])
        self.declare_parameter("pid_ki", [2.0, 2.0, 3.0])
        self.declare_parameter("pid_kd", [24.0, 24.0, 20.0])
        self.declare_parameter("pid_integral_limit", [2.0, 2.0, 1.0])
        self.declare_parameter("pid_max_force", [50.0, 50.0, 60.0])
        self.declare_parameter("velocity_lpf_alpha", 0.15)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("duration", 11.0)
        self.declare_parameter(
            "output_dir", "/workspace/results/human_board"
        )

        model_path = str(self.get_parameter("model_path").value)
        self._use_viewer = bool(self.get_parameter("use_viewer").value)
        self._allow_rotation = bool(self.get_parameter("allow_rotation").value)
        self._pid_enable = bool(self.get_parameter("pid_enable").value)
        requested_duration = float(self.get_parameter("duration").value)
        self._duration = min(max(requested_duration, 0.1), 15.0)
        if requested_duration != self._duration:
            self.get_logger().warn(
                f"duration={requested_duration:.3f}s is outside (0, 15]; "
                f"using {self._duration:.3f}s"
            )
        self._output_dir = str(self.get_parameter("output_dir").value)
        self._publish_period = 1.0 / max(
            float(self.get_parameter("publish_rate_hz").value), 1.0
        )
        self._trajectory_start_delay = float(
            self.get_parameter("trajectory_start_delay").value
        )
        self._trajectory_x_amplitude = float(
            self.get_parameter("trajectory_x_amplitude").value
        )
        self._trajectory_y_amplitude = float(
            self.get_parameter("trajectory_y_amplitude").value
        )
        self._trajectory_period = max(
            float(self.get_parameter("trajectory_period").value), 0.1
        )

        self._pid_kp = self._read_vec3_parameter("pid_kp")
        self._pid_ki = self._read_vec3_parameter("pid_ki")
        self._pid_kd = self._read_vec3_parameter("pid_kd")
        self._pid_integral_limit = self._read_vec3_parameter(
            "pid_integral_limit"
        )
        self._pid_max_force = self._read_vec3_parameter("pid_max_force")
        self._velocity_lpf_alpha = float(
            np.clip(
                float(self.get_parameter("velocity_lpf_alpha").value),
                0.0,
                1.0,
            )
        )

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._board_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "board"
        )
        if self._board_id < 0:
            raise RuntimeError(f'body "board" not found in {model_path}')

        requested_force_z = float(self.get_parameter("force_z").value)
        force_z = (
            self._model.body_mass[self._board_id]
            * abs(float(self._model.opt.gravity[2]))
            if requested_force_z < 0.0
            else requested_force_z
        )
        self._feedforward_force = np.array(
            [
                float(self.get_parameter("force_x").value),
                float(self.get_parameter("force_y").value),
                force_z,
            ],
            dtype=np.float64,
        )
        self._force = self._feedforward_force.copy()
        self._hand_offset = np.array(
            [float(self.get_parameter("hand_offset").value), 0.0, 0.0],
            dtype=np.float64,
        )

        self._pub_pose = self.create_publisher(PoseStamped, "board_pose", 10)
        self._pub_wrench = self.create_publisher(
            WrenchStamped, "virtual_human/hand_wrench", 10
        )
        self._pub_target_pose = self.create_publisher(
            PoseStamped, "board_target_pose", 10
        )
        self._next_publish_time = 0.0
        self._next_log_time = 0.0

        mujoco.mj_forward(self._model, self._data)
        self._initial_position = self._data.xpos[self._board_id].copy()
        self._initial_quaternion = self._data.xquat[self._board_id].copy()
        self._target_position = self._initial_position.copy()
        self._target_velocity = np.zeros(3, dtype=np.float64)
        self._position_error_integral = np.zeros(3, dtype=np.float64)
        self._board_velocity = np.zeros(3, dtype=np.float64)
        self._previous_position = self._initial_position.copy()
        self._history: list[dict[str, float]] = []
        self.get_logger().info(
            "human-board simulation ready: "
            f"mass={self._model.body_mass[self._board_id]:.3f} kg, "
            f"gravity_feedforward={self._feedforward_force.tolist()} N, "
            f"contact={'point' if self._allow_rotation else 'grip'}, "
            f"pid={'on' if self._pid_enable else 'off'}, "
            f"lissajous=(x_amplitude={self._trajectory_x_amplitude:.3f} m, "
            f"y_amplitude={self._trajectory_y_amplitude:.3f} m, "
            f"frequency_ratio=1:2, period={self._trajectory_period:.3f} s), "
            f"duration={self._duration:.1f} s, "
            f"model={model_path}"
        )

    def _read_vec3_parameter(self, name: str) -> np.ndarray:
        value = np.array(list(self.get_parameter(name).value), dtype=np.float64)
        if value.shape != (3,):
            raise RuntimeError(f"{name} must have 3 elements")
        return value

    def _update_target(self) -> None:
        tracking_time = max(
            0.0, float(self._data.time) - self._trajectory_start_delay
        )
        if self._data.time >= self._trajectory_start_delay:
            omega = 2.0 * np.pi / self._trajectory_period
            self._target_position = self._initial_position + np.array(
                [
                    self._trajectory_x_amplitude
                    * np.sin(omega * tracking_time),
                    self._trajectory_y_amplitude
                    * np.sin(2.0 * omega * tracking_time),
                    0.0,
                ],
                dtype=np.float64,
            )
            self._target_velocity = np.array(
                [
                    self._trajectory_x_amplitude
                    * omega
                    * np.cos(omega * tracking_time),
                    2.0
                    * self._trajectory_y_amplitude
                    * omega
                    * np.cos(2.0 * omega * tracking_time),
                    0.0,
                ],
                dtype=np.float64,
            )
        else:
            self._target_position = self._initial_position.copy()
            self._target_velocity.fill(0.0)

    def _compute_command_force(self) -> None:
        if not self._pid_enable:
            self._force = self._feedforward_force.copy()
            return

        self._update_target()
        position = self._data.xpos[self._board_id]
        position_error = self._target_position - position
        velocity_error = self._target_velocity - self._board_velocity

        dt = float(self._model.opt.timestep)
        self._position_error_integral += position_error * dt
        self._position_error_integral = np.clip(
            self._position_error_integral,
            -self._pid_integral_limit,
            self._pid_integral_limit,
        )

        feedback_force = (
            self._pid_kp * position_error
            + self._pid_ki * self._position_error_integral
            + self._pid_kd * velocity_error
        )
        self._force = np.clip(
            self._feedforward_force + feedback_force,
            -self._pid_max_force,
            self._pid_max_force,
        )

    def _apply_human_wrench(self) -> np.ndarray:
        self._compute_command_force()
        board_rotation = self._data.xmat[self._board_id].reshape(3, 3)
        offset_world = board_rotation @ self._hand_offset
        force_moment = np.cross(offset_world, self._force)

        # xfrc_applied stores the net force and torque at the body COM.
        self._data.xfrc_applied[self._board_id, :3] = self._force
        if self._allow_rotation:
            self._data.xfrc_applied[self._board_id, 3:] = force_moment
            return np.zeros(3, dtype=np.float64)

        # A grasp can transmit a moment. The hand moment cancels the moment
        # generated by applying the force away from the board COM.
        grip_moment = -force_moment
        self._data.xfrc_applied[self._board_id, 3:] = force_moment + grip_moment
        return grip_moment

    def _publish_state(self, grip_moment: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        position = self._data.xpos[self._board_id]
        quaternion_wxyz = self._data.xquat[self._board_id]
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = float(quaternion_wxyz[0])
        pose.pose.orientation.x = float(quaternion_wxyz[1])
        pose.pose.orientation.y = float(quaternion_wxyz[2])
        pose.pose.orientation.z = float(quaternion_wxyz[3])
        self._pub_pose.publish(pose)

        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = "world"
        target.pose.position.x = float(self._target_position[0])
        target.pose.position.y = float(self._target_position[1])
        target.pose.position.z = float(self._target_position[2])
        target.pose.orientation.w = float(self._initial_quaternion[0])
        target.pose.orientation.x = float(self._initial_quaternion[1])
        target.pose.orientation.y = float(self._initial_quaternion[2])
        target.pose.orientation.z = float(self._initial_quaternion[3])
        self._pub_target_pose.publish(target)

        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = "world"
        wrench.wrench.force.x = float(self._force[0])
        wrench.wrench.force.y = float(self._force[1])
        wrench.wrench.force.z = float(self._force[2])
        wrench.wrench.torque.x = float(grip_moment[0])
        wrench.wrench.torque.y = float(grip_moment[1])
        wrench.wrench.torque.z = float(grip_moment[2])
        self._pub_wrench.publish(wrench)

    def _record_sample(self) -> None:
        position = self._data.xpos[self._board_id]
        self._history.append(
            {
                "time": float(self._data.time),
                "desired_x": float(self._target_position[0]),
                "desired_y": float(self._target_position[1]),
                "desired_z": float(self._target_position[2]),
                "actual_x": float(position[0]),
                "actual_y": float(position[1]),
                "actual_z": float(position[2]),
                "force_x": float(self._force[0]),
                "force_y": float(self._force[1]),
                "force_z": float(self._force[2]),
                "force_norm": float(np.linalg.norm(self._force)),
            }
        )

    def _step_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        grip_moment = self._apply_human_wrench()
        mujoco.mj_step(self._model, self._data)
        measured_velocity = (
            self._data.xpos[self._board_id] - self._previous_position
        ) / float(self._model.opt.timestep)
        alpha = self._velocity_lpf_alpha
        self._board_velocity = (
            alpha * measured_velocity + (1.0 - alpha) * self._board_velocity
        )
        self._previous_position = self._data.xpos[self._board_id].copy()
        self._record_sample()
        if self._data.time >= self._next_publish_time:
            self._publish_state(grip_moment)
            self._next_publish_time += self._publish_period
        if self._data.time >= self._next_log_time:
            error = self._target_position - self._data.xpos[self._board_id]
            self.get_logger().info(
                f"t={self._data.time:.1f}s "
                f"target={np.round(self._target_position, 3).tolist()} "
                f"position={np.round(self._data.xpos[self._board_id], 3).tolist()} "
                f"error={np.round(error, 4).tolist()} "
                f"force={np.round(self._force, 3).tolist()}"
            )
            self._next_log_time += 1.0

    def save_results(self) -> None:
        """Save the run data and two publication-style tracking figures."""
        if len(self._history) < 2:
            self.get_logger().warn("not enough samples to generate result plots")
            return

        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        run_dir = os.path.join(self._output_dir, run_name)
        os.makedirs(run_dir, exist_ok=True)

        csv_path = os.path.join(run_dir, "history.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=list(self._history[0].keys())
            )
            writer.writeheader()
            writer.writerows(self._history)

        # Import lazily so the simulation can still run in minimal headless
        # environments; Agg never requires an X11 display.
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update(
            {
                "font.family": "serif",
                "font.serif": ["DejaVu Serif"],
                "font.size": 10,
                "axes.titlesize": 11,
                "axes.titleweight": "bold",
                "axes.labelsize": 10,
                "legend.fontsize": 9,
                "legend.frameon": False,
                "figure.dpi": 150,
                "savefig.dpi": 300,
                "savefig.bbox": "tight",
                "axes.spines.top": False,
                "axes.spines.right": False,
                "axes.grid": True,
                "grid.alpha": 0.2,
                "grid.linestyle": "-",
                "lines.linewidth": 1.8,
            }
        )

        time_data = np.array([row["time"] for row in self._history])
        desired_x = np.array([row["desired_x"] for row in self._history])
        desired_y = np.array([row["desired_y"] for row in self._history])
        actual_x = np.array([row["actual_x"] for row in self._history])
        actual_y = np.array([row["actual_y"] for row in self._history])
        force_x = np.array([row["force_x"] for row in self._history])
        force_y = np.array([row["force_y"] for row in self._history])
        force_z = np.array([row["force_z"] for row in self._history])

        # Okabe-Ito colors: distinguishable under common color-vision deficits.
        desired_color = "#0072B2"
        actual_color = "#D55E00"

        trajectory_stem = os.path.join(run_dir, "trajectory_comparison")
        fig, ax = plt.subplots(figsize=(6.4, 4.4))
        ax.plot(
            desired_x,
            desired_y,
            linestyle="--",
            color=desired_color,
            label="Desired trajectory",
            zorder=2,
        )
        ax.plot(
            actual_x,
            actual_y,
            color=actual_color,
            label="Actual trajectory",
            zorder=3,
        )
        ax.scatter(
            desired_x[0], desired_y[0], color="#000000", marker="o",
            s=28, label="Start", zorder=4,
        )
        ax.scatter(
            desired_x[-1], desired_y[-1], color="#000000", marker="x",
            s=38, label="End", zorder=4,
        )
        ax.set_xlabel("World X position (m)")
        ax.set_ylabel("World Y position (m)")
        ax.set_title("Horizontal Figure-Eight Trajectory Tracking")
        ax.axis("equal")
        ax.legend(loc="best")
        fig.savefig(trajectory_stem + ".png")
        plt.close(fig)

        force_stem = os.path.join(run_dir, "applied_force_xyz")
        fig, axes = plt.subplots(
            3, 1, figsize=(6.4, 6.8), sharex=True, constrained_layout=True
        )
        force_series = (
            ("X", force_x, "#0072B2"),
            ("Y", force_y, "#D55E00"),
            ("Z", force_z, "#009E73"),
        )
        for axis, (label, values, color) in zip(axes, force_series):
            axis.plot(
                time_data,
                values,
                color=color,
                label=rf"$F_{label.lower()}$",
            )
            axis.set_ylabel(f"$F_{label.lower()}$ (N)")
            axis.legend(loc="best")
            if label in ("X", "Y"):
                axis.axhline(
                    0.0,
                    color="#7F7F7F",
                    linestyle="--",
                    linewidth=1.0,
                )

        board_weight = (
            self._model.body_mass[self._board_id]
            * abs(float(self._model.opt.gravity[2]))
        )
        axes[2].axhline(
            board_weight,
            color="#7F7F7F",
            linestyle="--",
            linewidth=1.2,
            label=f"Board weight ({board_weight:.2f} N)",
        )
        axes[2].legend(loc="best")
        axes[2].set_xlabel("Simulation time (s)")
        fig.suptitle("Virtual-Human Applied Force Components", fontweight="bold")
        fig.savefig(force_stem + ".png")
        plt.close(fig)

        self.get_logger().info(
            "saved run data and plots: "
            f"{csv_path}, {trajectory_stem}.png, {force_stem}.png"
        )

    def _run_loop(self, viewer=None) -> None:
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            if self._duration > 0.0 and self._data.time >= self._duration:
                break
            step_start = time.perf_counter()
            self._step_once()
            if viewer is not None:
                viewer.sync()
            remaining = self._model.opt.timestep - (
                time.perf_counter() - step_start
            )
            if remaining > 0.0:
                time.sleep(remaining)

    def run(self) -> None:
        if not self._use_viewer:
            self.get_logger().info("running without MuJoCo viewer")
            self._run_loop()
            return

        try:
            viewer_available = bool(glfw.init())
            if viewer_available:
                glfw.terminate()
        except Exception:
            viewer_available = False

        if not viewer_available:
            self.get_logger().error(
                "GLFW viewer unavailable; continuing in headless mode"
            )
            self._run_loop()
            return

        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = self._data.xpos[self._board_id]
            viewer.cam.lookat[2] *= 0.5
            viewer.cam.distance = 5.0
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -18.0
            viewer.sync()
            self.get_logger().info(
                "MuJoCo viewer started; mouse rotate/pan/zoom is enabled"
            )
            self._run_loop(viewer)


def main() -> None:
    rclpy.init()
    node = HumanBoardSim()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.save_results()
        except Exception as exc:
            node.get_logger().error(
                f"failed to save result plots: {type(exc).__name__}: {exc}"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
