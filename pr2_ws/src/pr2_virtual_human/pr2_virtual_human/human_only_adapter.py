"""MuJoCo plant adapter for the human-only side of the A/B comparison."""

from __future__ import annotations

import os
import time

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .comparison_config import load_comparison_config
from .comparison_ros import (
    ACTUAL_HAND_POSE_TOPIC, BOARD_POSE_TOPIC, DESIRED_HAND_POSE_TOPIC,
    HUMAN_WRENCH_TOPIC, NATURAL_MOMENT_TOPIC, SIM_TIME_TOPIC, STATE_TOPIC,
    pose_message, vector_message, wrench_message,
)
from .experiment_state import ExperimentState, ExperimentStateMachine
from .human_impedance_6d import HumanImpedance6D, wrench_at_body_com
from .spatial import quaternion_to_matrix
from .trajectory_6d import Trajectory6D


class HumanOnlyAdapter(Node):
    def __init__(self) -> None:
        super().__init__("transport_human_only_adapter")
        self.declare_parameter("config_file", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("use_viewer", bool(os.environ.get("DISPLAY", "").strip()))
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            raise RuntimeError("config_file is required")
        self.config = load_comparison_config(config_path)
        model_path = str(self.get_parameter("model_path").value) or self.config.human_only_model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.board_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "board"))
        if self.board_id < 0:
            raise RuntimeError(f'body "board" not found in {model_path}')
        self.use_viewer = bool(self.get_parameter("use_viewer").value)
        self.machine = ExperimentStateMachine(self.config.timing)
        self.impedance = HumanImpedance6D(self.config.human_impedance)
        self.trajectory: Trajectory6D | None = None
        self.hand_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.body_velocity = np.zeros(6)
        self.last_publish_time = -np.inf
        self.publish_period = 1.0 / self.config.publish_rate_hz

        self.sim_time_pub = self.create_publisher(Float64, SIM_TIME_TOPIC, 10)
        self.actual_pub = self.create_publisher(PoseStamped, ACTUAL_HAND_POSE_TOPIC, 10)
        self.desired_pub = self.create_publisher(PoseStamped, DESIRED_HAND_POSE_TOPIC, 10)
        self.board_pub = self.create_publisher(PoseStamped, BOARD_POSE_TOPIC, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, HUMAN_WRENCH_TOPIC, 10)
        self.natural_moment_pub = self.create_publisher(Vector3Stamped, NATURAL_MOMENT_TOPIC, 10)
        self.state_pub = self.create_publisher(String, STATE_TOPIC, 10)

    def _measure(self) -> tuple[np.ndarray, np.ndarray]:
        board_position = self.data.xpos[self.board_id].copy()
        quaternion = self.data.xquat[self.board_id].copy()
        rotation = self.data.xmat[self.board_id].reshape(3, 3)
        return board_position + rotation @ self.config.hand_offset, quaternion

    def _update_velocity(self, quaternion: np.ndarray) -> None:
        # MuJoCo already solved the body's world-frame spatial velocity.
        # Avoid differentiating the very small-inertia roll motion again.
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.board_id,
            self.body_velocity,
            0,
        )
        self.angular_velocity = self.body_velocity[:3].copy()
        offset_world = quaternion_to_matrix(quaternion) @ self.config.hand_offset
        self.hand_velocity = self.body_velocity[3:].copy() + np.cross(
            self.angular_velocity, offset_world
        )

    def _publish(self, hand_position, quaternion, target, force, task_torque, natural_moment) -> None:
        now = float(self.data.time)
        if now + 1.0e-12 < self.last_publish_time + self.publish_period:
            return
        self.last_publish_time = now
        stamp = self.get_clock().now().to_msg()
        sim_msg = Float64()
        sim_msg.data = now
        self.actual_pub.publish(pose_message(hand_position, quaternion, self.config.frame_id, stamp))
        self.desired_pub.publish(pose_message(target.position, target.quaternion, self.config.frame_id, stamp))
        self.board_pub.publish(pose_message(self.data.xpos[self.board_id], quaternion, self.config.frame_id, stamp))
        self.wrench_pub.publish(wrench_message(force, task_torque, self.config.frame_id, stamp))
        self.natural_moment_pub.publish(vector_message(natural_moment, self.config.frame_id, stamp))
        state = String()
        state.data = self.machine.state.value
        self.state_pub.publish(state)
        self.sim_time_pub.publish(sim_msg)

    def step(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        hand_position, quaternion = self._measure()
        self._update_velocity(quaternion)
        previous_state = self.machine.state
        state = self.machine.update(float(self.data.time), True)
        if state == ExperimentState.TRACK and previous_state != ExperimentState.TRACK:
            # Anchor the recorded 50 Hz grid at trajectory time zero, matching
            # the robot condition's first TRACK sample.
            self.last_publish_time = -np.inf
        if self.machine.reference_latch_requested:
            self.trajectory = Trajectory6D(self.config.trajectory, hand_position, quaternion)
        assert self.trajectory is not None
        target = self.trajectory.sample(self.machine.trajectory_time(float(self.data.time)))
        wrench = self.impedance.evaluate(
            target, hand_position, self.hand_velocity, quaternion, self.angular_velocity
        )
        if state in (ExperimentState.WAIT_FOR_STATE, ExperimentState.LATCH_REFERENCE, ExperimentState.DONE):
            force = np.zeros(3)
            task_torque = np.zeros(3)
        else:
            force = wrench.force
            task_torque = wrench.task_torque
        offset_world = quaternion_to_matrix(quaternion) @ self.config.hand_offset
        applied_force, applied_torque, natural_moment = wrench_at_body_com(force, task_torque, offset_world)
        self.data.xfrc_applied[self.board_id, :3] = applied_force
        self.data.xfrc_applied[self.board_id, 3:] = applied_torque
        mujoco.mj_step(self.model, self.data)
        self._publish(hand_position, quaternion, target, force, task_torque, natural_moment)

    def run(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if self.use_viewer:
            from mujoco import viewer as mujoco_viewer
            with mujoco_viewer.launch_passive(self.model, self.data) as viewer:
                while rclpy.ok() and viewer.is_running():
                    start = time.monotonic()
                    self.step()
                    viewer.sync()
                    delay = self.model.opt.timestep - (time.monotonic() - start)
                    if delay > 0.0:
                        time.sleep(delay)
        else:
            while rclpy.ok():
                start = time.monotonic()
                self.step()
                delay = self.model.opt.timestep - (time.monotonic() - start)
                if delay > 0.0:
                    time.sleep(delay)


def main() -> None:
    rclpy.init()
    node = HumanOnlyAdapter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
        rclpy.try_shutdown()
