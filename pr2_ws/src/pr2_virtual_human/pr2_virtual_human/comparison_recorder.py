"""Condition-independent recorder; it never publishes control feedback."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .comparison_config import load_comparison_config
from .comparison_metrics import compute_run_metrics
from .comparison_ros import (
    ACTUAL_HAND_POSE_TOPIC, BOARD_POSE_TOPIC, DESIRED_HAND_POSE_TOPIC,
    HUMAN_WRENCH_TOPIC, NATURAL_MOMENT_TOPIC, ROBOT_EE_POSE_TOPIC,
    ROBOT_WRIST_WRENCH_TOPIC, SIM_TIME_TOPIC, STATE_TOPIC, pose_arrays,
)
from .experiment_state import ExperimentState
from .spatial import matrix_to_rotvec, quaternion_to_matrix
from .trajectory_6d import Trajectory6D
from .run_plotting import save_run_plots


class ComparisonRecorder(Node):
    def __init__(self) -> None:
        super().__init__("transport_comparison_recorder")
        self.declare_parameter("config_file", "")
        self.declare_parameter("condition", "human_only")
        self.declare_parameter("experiment_id", "default")
        self.declare_parameter("output_root", "")
        self.declare_parameter("robot_wrist_source_topic", "mujoco/left_wrist_wrench")
        self.declare_parameter("robot_ee_source_topic", "ee_pose")
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            raise RuntimeError("config_file is required")
        self.config = load_comparison_config(config_path)
        self.condition = str(self.get_parameter("condition").value)
        if self.condition not in ("human_only", "human_robot"):
            raise RuntimeError("condition must be human_only or human_robot")
        self.experiment_id = str(self.get_parameter("experiment_id").value)
        self.output_root = str(self.get_parameter("output_root").value) or self.config.output_root
        self.latest: dict[str, object] = {}
        self.state = ExperimentState.WAIT_FOR_STATE.value
        self.rows: list[dict[str, float | str]] = []
        self.saved = False
        self.last_record_time = -np.inf
        self.record_period = 1.0 / self.config.publish_rate_hz
        self.record_trajectory: Trajectory6D | None = None

        self.create_subscription(PoseStamped, ACTUAL_HAND_POSE_TOPIC, lambda msg: self._store("actual", msg), 20)
        self.create_subscription(PoseStamped, DESIRED_HAND_POSE_TOPIC, lambda msg: self._store("desired", msg), 20)
        self.create_subscription(PoseStamped, BOARD_POSE_TOPIC, lambda msg: self._store("board", msg), 20)
        self.create_subscription(WrenchStamped, HUMAN_WRENCH_TOPIC, lambda msg: self._store("human_wrench", msg), 20)
        self.create_subscription(Vector3Stamped, NATURAL_MOMENT_TOPIC, lambda msg: self._store("natural_moment", msg), 20)
        self.create_subscription(String, STATE_TOPIC, self._on_state, 20)
        self.create_subscription(Float64, SIM_TIME_TOPIC, self._on_sim_time, 20)
        if self.condition == "human_robot":
            wrist_topic = str(self.get_parameter("robot_wrist_source_topic").value)
            ee_topic = str(self.get_parameter("robot_ee_source_topic").value)
            self.create_subscription(WrenchStamped, wrist_topic, self._on_robot_wrist, 20)
            self.create_subscription(PoseStamped, ee_topic, self._on_robot_ee, 20)
            self.robot_wrist_pub = self.create_publisher(WrenchStamped, ROBOT_WRIST_WRENCH_TOPIC, 10)
            self.robot_ee_pub = self.create_publisher(PoseStamped, ROBOT_EE_POSE_TOPIC, 10)

    def _store(self, key: str, msg) -> None:
        self.latest[key] = msg

    def _on_state(self, msg: String) -> None:
        self.state = str(msg.data)

    def _on_robot_wrist(self, msg: WrenchStamped) -> None:
        self.latest["robot_wrench"] = msg
        self.robot_wrist_pub.publish(msg)

    def _on_robot_ee(self, msg: PoseStamped) -> None:
        self.latest["robot_ee"] = msg
        self.robot_ee_pub.publish(msg)

    @staticmethod
    def _wrench(msg: WrenchStamped | None) -> tuple[np.ndarray, np.ndarray]:
        if msg is None:
            return np.full(3, np.nan), np.full(3, np.nan)
        force = msg.wrench.force
        torque = msg.wrench.torque
        return np.array([force.x, force.y, force.z]), np.array([torque.x, torque.y, torque.z])

    def _on_sim_time(self, msg: Float64) -> None:
        if self.state == ExperimentState.DONE.value:
            self.save()
            rclpy.shutdown()
            return
        if self.state not in (ExperimentState.TRACK.value, ExperimentState.HOLD.value):
            return
        now = float(msg.data)
        if now + 1.0e-12 < self.last_record_time + self.record_period:
            return
        required = ("actual", "desired", "board", "human_wrench", "natural_moment")
        if any(key not in self.latest for key in required):
            return
        actual_p, actual_q = pose_arrays(self.latest["actual"])
        desired_p, desired_q = pose_arrays(self.latest["desired"])
        if self.record_trajectory is None:
            self.record_trajectory = Trajectory6D(
                self.config.trajectory, desired_p, desired_q
            )
        trajectory_time = (
            len(self.rows) * self.record_period
            if self.state == ExperimentState.TRACK.value
            else self.config.timing.tracking_sec
        )
        recorded_target = self.record_trajectory.sample(trajectory_time)
        desired_p = recorded_target.position
        desired_q = recorded_target.quaternion
        board_p, board_q = pose_arrays(self.latest["board"])
        human_force, task_torque = self._wrench(self.latest["human_wrench"])
        robot_force, robot_torque = self._wrench(self.latest.get("robot_wrench"))
        natural_msg = self.latest["natural_moment"].vector
        natural = np.array([natural_msg.x, natural_msg.y, natural_msg.z])
        position_error = desired_p - actual_p
        desired_rotvec = matrix_to_rotvec(quaternion_to_matrix(desired_q))
        actual_rotvec = matrix_to_rotvec(quaternion_to_matrix(actual_q))
        orientation_error = matrix_to_rotvec(quaternion_to_matrix(desired_q) @ quaternion_to_matrix(actual_q).T)
        row: dict[str, float | str] = {
            "time": now, "state": self.state, "condition": self.condition,
        }
        for axis, index in zip("xyz", range(3)):
            row[f"desired_position_{axis}"] = float(desired_p[index])
            row[f"actual_position_{axis}"] = float(actual_p[index])
            row[f"position_error_{axis}"] = float(position_error[index])
            row[f"desired_rotvec_{axis}"] = float(desired_rotvec[index])
            row[f"actual_rotvec_{axis}"] = float(actual_rotvec[index])
            row[f"orientation_error_{axis}"] = float(orientation_error[index])
            row[f"human_force_{axis}"] = float(human_force[index])
            row[f"human_task_torque_{axis}"] = float(task_torque[index])
            row[f"natural_endpoint_moment_{axis}"] = float(natural[index])
            row[f"board_position_{axis}"] = float(board_p[index])
            row[f"robot_force_{axis}"] = float(robot_force[index])
            row[f"robot_torque_{axis}"] = float(robot_torque[index])
        self.rows.append(row)
        self.last_record_time = now

    def _git_commit(self) -> str:
        try:
            return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        except Exception:
            return "unknown"

    def save(self) -> Path | None:
        if self.saved or not self.rows:
            return None
        run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        run_dir = Path(self.output_root) / self.experiment_id / self.condition / run_id
        run_dir.mkdir(parents=True, exist_ok=False)
        csv_path = run_dir / "history.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        metrics = compute_run_metrics(
            self.rows,
            {"force": self.config.human_impedance.force_limit,
             "task_torque": self.config.human_impedance.task_torque_limit},
        )
        with (run_dir / "metrics.json").open("w", encoding="utf-8") as stream:
            json.dump(metrics, stream, indent=2, ensure_ascii=False)
        shutil.copy2(self.config.source_path, run_dir / "experiment_config.yaml")
        manifest = {
            "schema_version": 1,
            "condition": self.condition,
            "experiment_id": self.experiment_id,
            "config_sha256": self.config.config_hash,
            "git_commit": self._git_commit(),
            "model_path": self.config.human_only_model if self.condition == "human_only" else self.config.human_robot_model,
            "sample_count": len(self.rows),
            "sample_rate_hz": self.config.publish_rate_hz,
            "pid": os.getpid(),
            "launch_entry": "pr2_virtual_human transport_comparison.launch.py",
            "simulation_timestep_sec": 0.002,
            "python_version": sys.version.split()[0],
            "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
            "robot_wrist_wrench_semantics": (
                "tare-referenced measured diagnostic; never used by the human controller"
                if self.condition == "human_robot" else None
            ),
            "wrist_tare_duration_sec": (
                self.config.timing.settle_sec if self.condition == "human_robot" else None
            ),
        }
        with (run_dir / "run_manifest.json").open("w", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, ensure_ascii=False)
        plot_paths = save_run_plots(self.rows, self.condition, run_dir)
        self.saved = True
        self.get_logger().info(
            f"comparison run saved with {len(plot_paths)} plots: {run_dir}"
        )
        return run_dir


def main() -> None:
    rclpy.init()
    node = ComparisonRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        finally:
            if node.context.ok():
                node.destroy_node()
            rclpy.try_shutdown()
