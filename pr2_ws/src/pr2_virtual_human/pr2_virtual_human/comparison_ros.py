"""ROS message helpers and the stable topic contract for the comparison demo."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped


TOPIC_PREFIX = "transport_comparison"
SIM_TIME_TOPIC = f"{TOPIC_PREFIX}/sim_time"
ACTUAL_HAND_POSE_TOPIC = f"{TOPIC_PREFIX}/actual_hand_pose"
DESIRED_HAND_POSE_TOPIC = f"{TOPIC_PREFIX}/desired_hand_pose"
HUMAN_WRENCH_TOPIC = f"{TOPIC_PREFIX}/human_wrench"
BOARD_POSE_TOPIC = f"{TOPIC_PREFIX}/board_pose"
STATE_TOPIC = f"{TOPIC_PREFIX}/state"
NATURAL_MOMENT_TOPIC = f"{TOPIC_PREFIX}/natural_endpoint_moment"
ROBOT_WRIST_WRENCH_TOPIC = f"{TOPIC_PREFIX}/robot_wrist_wrench"
ROBOT_EE_POSE_TOPIC = f"{TOPIC_PREFIX}/robot_ee_pose"
CONTRACT_VALID_TOPIC = f"{TOPIC_PREFIX}/contract_valid"


def pose_message(position: np.ndarray, quaternion: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
    msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = map(float, quaternion)
    return msg


def wrench_message(force: np.ndarray, task_torque: np.ndarray, frame_id: str, stamp) -> WrenchStamped:
    msg = WrenchStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = map(float, force)
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = map(float, task_torque)
    return msg


def vector_message(vector: np.ndarray, frame_id: str, stamp) -> Vector3Stamped:
    msg = Vector3Stamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.vector.x, msg.vector.y, msg.vector.z = map(float, vector)
    return msg


def pose_arrays(msg: PoseStamped) -> tuple[np.ndarray, np.ndarray]:
    p = msg.pose.position
    q = msg.pose.orientation
    return (
        np.array([p.x, p.y, p.z], dtype=np.float64),
        np.array([q.w, q.x, q.y, q.z], dtype=np.float64),
    )

