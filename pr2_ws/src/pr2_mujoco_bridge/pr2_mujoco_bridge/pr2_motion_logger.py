"""
pr2_motion_logger — 将关键运动数据写成 CSV，供离线分析。

订阅话题（均可通过参数重映射）：
  ee_pose_topic      : geometry_msgs/PoseStamped  — 末端执行器实际位姿
  ik_target_topic    : geometry_msgs/PoseStamped  — IK 目标位姿（导纳控制输出）
  wrench_topic       : geometry_msgs/WrenchStamped — 施加的虚拟力/力矩
  joint_state_topic  : sensor_msgs/JointState      — 关节状态

CSV 列（按顺序）：
  wall_time_sec      系统时钟秒（float）
  ros_time_sec       ROS 时钟秒（float）
  ee_x ee_y ee_z     末端位置 (m)
  ee_qw ee_qx ee_qy ee_qz  末端姿态四元数
  tgt_x tgt_y tgt_z  IK 目标位置 (m)（未收到时为 nan）
  tgt_qw tgt_qx tgt_qy tgt_qz  IK 目标姿态
  fx fy fz tx ty tz  施加力 (N) 和力矩 (N·m)
  <joint_name>_pos <joint_name>_vel  各受监控关节的位置和速度

参数：
  output_path       : CSV 文件路径，默认 /tmp/pr2_motion_<timestamp>.csv
  log_rate_hz       : 写入频率，默认 50.0
  watch_joints      : 要记录的关节名列表，默认为左臂 7 个关节 + torso
  ee_pose_topic / ik_target_topic / wrench_topic / joint_state_topic
"""

import csv
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState

_DEFAULT_JOINTS = [
    "torso_lift_joint",
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_forearm_roll_joint",
    "l_wrist_flex_joint",
    "l_wrist_roll_joint",
]


class Pr2MotionLogger(Node):
    def __init__(self) -> None:
        super().__init__("pr2_motion_logger")

        self.declare_parameter("output_path", "")
        self.declare_parameter("log_rate_hz", 50.0)
        self.declare_parameter("ee_pose_topic", "ee_pose")
        self.declare_parameter("ik_target_topic", "ik_target_pose")
        self.declare_parameter("wrench_topic", "wbc/arm_external_wrench")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("watch_joints", _DEFAULT_JOINTS)

        _raw_path = str(self.get_parameter("output_path").value).strip()
        if not _raw_path:
            ts = time.strftime("%Y%m%d_%H%M%S")
            _raw_path = f"/workspace/logs/pr2_motion_{ts}.csv"
        self._output_path = _raw_path
        rate_hz = float(self.get_parameter("log_rate_hz").value)
        self._watch_joints: List[str] = list(
            self.get_parameter("watch_joints").value
        )

        # Latest values (updated by callbacks)
        self._ee: Optional[PoseStamped] = None
        self._tgt: Optional[PoseStamped] = None
        self._wrench = [0.0] * 6          # fx fy fz tx ty tz
        self._jpos: Dict[str, float] = {}
        self._jvel: Dict[str, float] = {}

        self.create_subscription(
            PoseStamped,
            self.get_parameter("ee_pose_topic").value,
            lambda m: setattr(self, "_ee", m),
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("ik_target_topic").value,
            lambda m: setattr(self, "_tgt", m),
            10,
        )
        self.create_subscription(
            WrenchStamped,
            self.get_parameter("wrench_topic").value,
            self._on_wrench,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._on_joint_state,
            30,
        )

        # Open CSV and write header
        os.makedirs(os.path.dirname(self._output_path) or ".", exist_ok=True)
        self._csv_file = open(self._output_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        header = [
            "wall_time_sec", "ros_time_sec",
            "ee_x", "ee_y", "ee_z",
            "ee_qw", "ee_qx", "ee_qy", "ee_qz",
            "tgt_x", "tgt_y", "tgt_z",
            "tgt_qw", "tgt_qx", "tgt_qy", "tgt_qz",
            "fx", "fy", "fz", "tx", "ty", "tz",
        ]
        for jn in self._watch_joints:
            header += [f"{jn}_pos", f"{jn}_vel"]
        self._writer.writerow(header)
        self._csv_file.flush()

        self._row_count = 0
        self.create_timer(1.0 / max(rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"pr2_motion_logger 已启动: 输出={self._output_path}, "
            f"频率={rate_hz:.1f}Hz, 关节={self._watch_joints}"
        )

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._wrench = [
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        ]

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if name in self._watch_joints:
                if i < len(msg.position):
                    self._jpos[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self._jvel[name] = float(msg.velocity[i])

    def _tick(self) -> None:
        nan = float("nan")
        wall = time.time()
        ros = self.get_clock().now().nanoseconds * 1e-9

        if self._ee is not None:
            p = self._ee.pose.position
            q = self._ee.pose.orientation
            ee_row = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        else:
            ee_row = [nan] * 7

        if self._tgt is not None:
            p = self._tgt.pose.position
            q = self._tgt.pose.orientation
            tgt_row = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        else:
            tgt_row = [nan] * 7

        joint_row: List[float] = []
        for jn in self._watch_joints:
            joint_row.append(self._jpos.get(jn, nan))
            joint_row.append(self._jvel.get(jn, nan))

        row = [wall, ros] + ee_row + tgt_row + self._wrench + joint_row
        self._writer.writerow([f"{v:.6f}" if not math.isnan(v) else "nan" for v in row])
        self._row_count += 1

        # Flush every 50 rows to keep data safe without excessive I/O
        if self._row_count % 50 == 0:
            self._csv_file.flush()

    def destroy_node(self) -> None:
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f"pr2_motion_logger 已关闭: 共写入 {self._row_count} 行 → {self._output_path}"
        )
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Pr2MotionLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
