"""Robot-condition virtual human: same impedance, no robot-aware input."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from .comparison_config import load_comparison_config
from .comparison_ros import (
    ACTUAL_HAND_POSE_TOPIC, BOARD_POSE_TOPIC, CONTRACT_VALID_TOPIC,
    DESIRED_HAND_POSE_TOPIC, HUMAN_WRENCH_TOPIC, NATURAL_MOMENT_TOPIC,
    SIM_TIME_TOPIC, STATE_TOPIC, pose_arrays, pose_message, vector_message,
    wrench_message,
)
from .experiment_state import ExperimentState, ExperimentStateMachine
from .human_impedance_6d import HumanImpedance6D
from .spatial import orientation_error_world, quaternion_to_matrix
from .trajectory_6d import Trajectory6D


class HumanRobotController(Node):
    def __init__(self) -> None:
        super().__init__("transport_human_robot_controller")
        self.declare_parameter("config_file", "")
        self.declare_parameter("source_hand_pose_topic", "mujoco/human_hand_pose")
        self.declare_parameter("source_board_pose_topic", "mujoco/board_pose")
        self.declare_parameter("source_sim_time_topic", "mujoco/sim_time")
        self.declare_parameter("bridge_hand_wrench_topic", "virtual_human/hand_force")
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            raise RuntimeError("config_file is required")
        self.config = load_comparison_config(config_path)
        self.machine = ExperimentStateMachine(self.config.timing)
        self.impedance = HumanImpedance6D(self.config.human_impedance)
        self.trajectory: Trajectory6D | None = None
        self.hand_pose: PoseStamped | None = None
        self.hand_pose_is_new = False
        self.board_pose: PoseStamped | None = None
        self.contract_valid = False
        self.last_sim_time: float | None = None
        self.last_measurement_time: float | None = None
        self.previous_position: np.ndarray | None = None
        self.previous_quaternion: np.ndarray | None = None
        self.hand_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        self.create_subscription(PoseStamped, str(self.get_parameter("source_hand_pose_topic").value), self._on_hand_pose, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("source_board_pose_topic").value), self._on_board_pose, 10)
        self.create_subscription(Float64, str(self.get_parameter("source_sim_time_topic").value), self._on_sim_time, 10)
        self.create_subscription(Bool, CONTRACT_VALID_TOPIC, self._on_contract, 10)
        self.bridge_wrench_pub = self.create_publisher(WrenchStamped, str(self.get_parameter("bridge_hand_wrench_topic").value), 10)
        self.sim_time_pub = self.create_publisher(Float64, SIM_TIME_TOPIC, 10)
        self.actual_pub = self.create_publisher(PoseStamped, ACTUAL_HAND_POSE_TOPIC, 10)
        self.desired_pub = self.create_publisher(PoseStamped, DESIRED_HAND_POSE_TOPIC, 10)
        self.board_pub = self.create_publisher(PoseStamped, BOARD_POSE_TOPIC, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, HUMAN_WRENCH_TOPIC, 10)
        self.natural_moment_pub = self.create_publisher(Vector3Stamped, NATURAL_MOMENT_TOPIC, 10)
        self.state_pub = self.create_publisher(String, STATE_TOPIC, 10)

    def _on_hand_pose(self, msg: PoseStamped) -> None:
        self.hand_pose = msg
        self.hand_pose_is_new = True

    def _on_board_pose(self, msg: PoseStamped) -> None:
        self.board_pose = msg

    def _on_contract(self, msg: Bool) -> None:
        self.contract_valid = bool(msg.data)

    def _on_sim_time(self, msg: Float64) -> None:
        now = float(msg.data)
        if self.last_sim_time is not None and now <= self.last_sim_time:
            return
        if self.hand_pose is None:
            return
        position, quaternion = pose_arrays(self.hand_pose)
        if (
            self.hand_pose_is_new
            and self.last_measurement_time is not None
            and self.previous_position is not None
            and now > self.last_measurement_time
        ):
            dt = now - self.last_measurement_time
            self.hand_velocity = (position - self.previous_position) / dt
            self.angular_velocity = orientation_error_world(self.previous_quaternion, quaternion) / dt
        if self.hand_pose_is_new:
            self.last_measurement_time = now
            self.previous_position = position.copy()
            self.previous_quaternion = quaternion.copy()
            self.hand_pose_is_new = False
        self.last_sim_time = now
        state = self.machine.update(now, self.contract_valid)
        if self.machine.reference_latch_requested:
            self.trajectory = Trajectory6D(self.config.trajectory, position, quaternion)
        if self.trajectory is None:
            self._publish_zero_state(now, position, quaternion)
            return
        target = self.trajectory.sample(self.machine.trajectory_time(now))
        wrench = self.impedance.evaluate(target, position, self.hand_velocity, quaternion, self.angular_velocity)
        if state in (ExperimentState.WAIT_FOR_STATE, ExperimentState.LATCH_REFERENCE, ExperimentState.DONE):
            force = np.zeros(3)
            task_torque = np.zeros(3)
        else:
            force = wrench.force
            task_torque = wrench.task_torque
        stamp = self.get_clock().now().to_msg()
        command = wrench_message(force, task_torque, self.config.frame_id, stamp)
        self.bridge_wrench_pub.publish(command)
        self.wrench_pub.publish(command)
        self.actual_pub.publish(pose_message(position, quaternion, self.config.frame_id, stamp))
        self.desired_pub.publish(pose_message(target.position, target.quaternion, self.config.frame_id, stamp))
        if self.board_pose is not None:
            self.board_pub.publish(self.board_pose)
        offset_world = quaternion_to_matrix(quaternion) @ self.config.hand_offset
        self.natural_moment_pub.publish(vector_message(np.cross(offset_world, force), self.config.frame_id, stamp))
        state_msg = String()
        state_msg.data = state.value
        self.state_pub.publish(state_msg)
        self.sim_time_pub.publish(msg)

    def _publish_zero_state(self, now: float, position: np.ndarray, quaternion: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        zero = wrench_message(np.zeros(3), np.zeros(3), self.config.frame_id, stamp)
        self.bridge_wrench_pub.publish(zero)
        self.wrench_pub.publish(zero)
        sim = Float64()
        sim.data = now
        self.actual_pub.publish(pose_message(position, quaternion, self.config.frame_id, stamp))
        state_msg = String()
        state_msg.data = self.machine.state.value
        self.state_pub.publish(state_msg)
        self.sim_time_pub.publish(sim)


def main() -> None:
    rclpy.init()
    node = HumanRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
        rclpy.try_shutdown()
