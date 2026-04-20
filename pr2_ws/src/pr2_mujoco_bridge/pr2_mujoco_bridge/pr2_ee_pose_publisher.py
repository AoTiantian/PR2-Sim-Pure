#!/usr/bin/env python3
"""发布 PR2 指定末端执行器位姿（由 joint_states + MuJoCo FK 计算）。"""

from __future__ import annotations

from typing import Dict

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
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
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("pose_topic", "ee_pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("base_body", "base_link")
        self.declare_parameter("publish_rate_hz", 50.0)

        model_path = self.get_parameter("model_path").value
        ee_body_name = self.get_parameter("end_effector_body").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        base_body_name = self.get_parameter("base_body").value
        hz = float(self.get_parameter("publish_rate_hz").value)

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name
        )
        if self._ee_body_id < 0:
            raise RuntimeError(f"末端 body 不存在: {ee_body_name}")
        self._base_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, base_body_name
        )
        if self._base_body_id < 0:
            raise RuntimeError(f"基座 body 不存在: {base_body_name}")

        self._joint_pos: Dict[str, float] = {}
        self._joint_vel: Dict[str, float] = {}
        self._odom_pose = None
        self._odom_twist = None

        self._pub_pose = self.create_publisher(PoseStamped, pose_topic, 10)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 30)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_timer(1.0 / max(hz, 1e-3), self._on_timer)

        self.get_logger().info(
            "EE 位姿发布节点启动: "
            f"body={ee_body_name}, base={base_body_name}, frame_id={self._frame_id}, "
            f"pub={pose_topic}, sub={joint_state_topic}+{odom_topic}"
        )

    @staticmethod
    def _quat_to_rot_wxyz(q: np.ndarray) -> np.ndarray:
        qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        aw, ax, ay, az = float(a[0]), float(a[1]), float(a[2]), float(a[3])
        bw, bx, by, bz = float(b[0]), float(b[1]), float(b[2]), float(b[3])
        return np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=np.float64,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_pos[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self._joint_vel[name] = float(msg.velocity[i])

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_pose = msg.pose.pose
        self._odom_twist = msg.twist.twist

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

        # Sync base free-joint from odom for world-frame consistency.
        if self._odom_pose is not None:
            base_free_qadr = None
            base_free_vadr = None
            for j in range(self._model.njnt):
                if int(self._model.jnt_type[j]) != int(mujoco.mjtJoint.mjJNT_FREE):
                    continue
                if int(self._model.jnt_bodyid[j]) != self._base_body_id:
                    continue
                base_free_qadr = int(self._model.jnt_qposadr[j])
                base_free_vadr = int(self._model.jnt_dofadr[j])
                break

            if base_free_qadr is not None and base_free_vadr is not None:
                p = self._odom_pose.position
                q = self._odom_pose.orientation
                n = float(np.linalg.norm([q.w, q.x, q.y, q.z]))
                if n < 1e-9:
                    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
                else:
                    qw, qx, qy, qz = q.w / n, q.x / n, q.y / n, q.z / n

                self._data.qpos[base_free_qadr + 0] = float(p.x)
                self._data.qpos[base_free_qadr + 1] = float(p.y)
                self._data.qpos[base_free_qadr + 2] = float(p.z)
                self._data.qpos[base_free_qadr + 3] = float(qw)
                self._data.qpos[base_free_qadr + 4] = float(qx)
                self._data.qpos[base_free_qadr + 5] = float(qy)
                self._data.qpos[base_free_qadr + 6] = float(qz)
                if self._odom_twist is not None:
                    tw = self._odom_twist
                    self._data.qvel[base_free_vadr + 0] = float(tw.linear.x)
                    self._data.qvel[base_free_vadr + 1] = float(tw.linear.y)
                    self._data.qvel[base_free_vadr + 2] = float(tw.linear.z)
                    self._data.qvel[base_free_vadr + 3] = float(tw.angular.x)
                    self._data.qvel[base_free_vadr + 4] = float(tw.angular.y)
                    self._data.qvel[base_free_vadr + 5] = float(tw.angular.z)
                got_any = True

        if not got_any:
            return False

        mujoco.mj_forward(self._model, self._data)
        return True

    def _on_timer(self) -> None:
        if not self._sync_model_state():
            return

        ee_pos = np.asarray(self._data.xpos[self._ee_body_id, :], dtype=np.float64)
        ee_q = np.asarray(self._data.xquat[self._ee_body_id, :], dtype=np.float64)

        if self._frame_id == "base_link":
            base_pos = np.asarray(self._data.xpos[self._base_body_id, :], dtype=np.float64)
            base_q = np.asarray(self._data.xquat[self._base_body_id, :], dtype=np.float64)
            r_base = self._quat_to_rot_wxyz(base_q)
            pos = r_base.T @ (ee_pos - base_pos)
            q_base_inv = np.array([base_q[0], -base_q[1], -base_q[2], -base_q[3]], dtype=np.float64)
            quat_wxyz = self._quat_mul_wxyz(q_base_inv, ee_q)
            n = np.linalg.norm(quat_wxyz)
            if n > 1e-9:
                quat_wxyz = quat_wxyz / n
            else:
                quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            pos = ee_pos
            quat_wxyz = ee_q

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Launch global-shutdown can race with spin and trigger take_message conversion errors.
        if rclpy.ok():
            raise exc
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
