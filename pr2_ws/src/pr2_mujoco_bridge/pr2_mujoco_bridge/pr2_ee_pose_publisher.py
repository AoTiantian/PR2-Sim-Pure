#!/usr/bin/env python3
"""发布 PR2 指定末端执行器位姿（由 joint_states + MuJoCo FK 计算）。"""

from __future__ import annotations

from typing import Dict

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Pr2EePosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("pr2_ee_pose_publisher")

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        )
        self.declare_parameter("end_effector_body", "l_gripper_tool_frame")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("pose_topic", "ee_pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_rate_hz", 50.0)

        model_path = self.get_parameter("model_path").value
        ee_body_name = self.get_parameter("end_effector_body").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        hz = float(self.get_parameter("publish_rate_hz").value)

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name
        )
        if self._ee_body_id < 0:
            raise RuntimeError(f"末端 body 不存在: {ee_body_name}")

        self._joint_pos: Dict[str, float] = {}
        self._joint_vel: Dict[str, float] = {}

        self._pub_pose = self.create_publisher(PoseStamped, pose_topic, 10)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 30)
        self.create_timer(1.0 / max(hz, 1e-3), self._on_timer)

        self.get_logger().info(
            f"EE 位姿发布节点启动: body={ee_body_name}, pub={pose_topic}, sub={joint_state_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_pos[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self._joint_vel[name] = float(msg.velocity[i])

    def _sync_model_state(self) -> bool:
        # 用 joint_states 回填到 MuJoCo 的 1DoF 关节
        got_any = False
        for j in range(self._model.njnt):
            jtype = int(self._model.jnt_type[j])
            if jtype not in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ):
                continue
            jn = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if not jn or jn not in self._joint_pos:
                continue
            qadr = int(self._model.jnt_qposadr[j])
            vadr = int(self._model.jnt_dofadr[j])
            self._data.qpos[qadr] = self._joint_pos[jn]
            self._data.qvel[vadr] = self._joint_vel.get(jn, 0.0)
            got_any = True

        if not got_any:
            return False

        mujoco.mj_forward(self._model, self._data)
        return True

    def _on_timer(self) -> None:
        if not self._sync_model_state():
            return

        pos = np.asarray(self._data.xpos[self._ee_body_id, :], dtype=np.float64)
        quat_wxyz = np.asarray(self._data.xquat[self._ee_body_id, :], dtype=np.float64)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        # ROS quaternion: x y z w; MuJoCo: w x y z
        msg.pose.orientation.x = float(quat_wxyz[1])
        msg.pose.orientation.y = float(quat_wxyz[2])
        msg.pose.orientation.z = float(quat_wxyz[3])
        msg.pose.orientation.w = float(quat_wxyz[0])
        self._pub_pose.publish(msg)



def main() -> None:
    rclpy.init()
    node = Pr2EePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
