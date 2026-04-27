#!/usr/bin/env python3
"""PR2 left-arm Cartesian admittance controller."""

from __future__ import annotations

from typing import Dict, List

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from pr2_mujoco_bridge.admittance_core import (
    AdmittanceState,
    AxisGains,
    apply_deadband,
    clip_norm,
    limit_joint_velocity_near_limits,
    rotate_body_vector_to_world,
    rotate_world_vector_to_body,
    settle_admittance_state,
    solve_dls_velocity_with_nullspace,
)


class ArmAdmittanceNode(Node):
    def __init__(self) -> None:
        super().__init__("pr2_arm_admittance")

        self.declare_parameter(
            "model_path",
            "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        )
        self.declare_parameter("ee_body_name", "l_gripper_tool_frame")
        self.declare_parameter("base_body_name", "base_link")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter(
            "controlled_joints",
            [
                "l_shoulder_pan_joint",
                "l_shoulder_lift_joint",
                "l_upper_arm_roll_joint",
                "l_elbow_flex_joint",
                "l_forearm_roll_joint",
                "l_wrist_flex_joint",
                "l_wrist_roll_joint",
            ],
        )
        self.declare_parameter("active_axes", [1, 0, 0])
        self.declare_parameter("mass_x", 2.0)
        self.declare_parameter("mass_y", 2.0)
        self.declare_parameter("mass_z", 2.0)
        self.declare_parameter("damping_x", 38.0)
        self.declare_parameter("damping_y", 38.0)
        self.declare_parameter("damping_z", 42.0)
        self.declare_parameter("stiffness_x", 45.0)
        self.declare_parameter("stiffness_y", 45.0)
        self.declare_parameter("stiffness_z", 50.0)
        self.declare_parameter("control_frequency", 100.0)
        self.declare_parameter("dls_lambda", 0.25)
        self.declare_parameter("wrench_filter_alpha", 0.25)
        self.declare_parameter("force_deadband_x", 0.25)
        self.declare_parameter("force_deadband_y", 0.25)
        self.declare_parameter("force_deadband_z", 0.35)
        self.declare_parameter("home_capture_duration", 0.5)
        self.declare_parameter("ee_vel_max", 0.035)
        self.declare_parameter("ee_disp_max", 0.12)
        self.declare_parameter("ee_track_gain", 8.0)
        self.declare_parameter("return_track_gain", 12.0)
        self.declare_parameter("return_ee_vel_max", 0.05)
        self.declare_parameter("nullspace_gain", 0.0)
        self.declare_parameter("qdot_des_max", 0.22)
        self.declare_parameter("return_qdot_des_max", 0.3)
        self.declare_parameter("qdot_smoothing_alpha", 0.3)
        self.declare_parameter("joint_limit_margin", 0.12)
        self.declare_parameter("torque_kp", 18.0)
        self.declare_parameter("torque_kd", 18.0)
        self.declare_parameter("max_torque", 60.0)
        self.declare_parameter("settle_displacement_epsilon", 0.003)
        self.declare_parameter("settle_velocity_epsilon", 0.003)
        self.declare_parameter("wrench_topic", "wbc/arm/external_wrench")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("ee_pose_topic", "wbc/arm/ee_pose_log")
        self.declare_parameter("debug_topic", "wbc/arm/admittance_debug")

        self._command_frame = str(self.get_parameter("command_frame").value)
        if self._command_frame not in ("base_link", "world"):
            raise ValueError("command_frame must be base_link or world")

        model_path = str(self.get_parameter("model_path").value)
        ee_body_name = str(self.get_parameter("ee_body_name").value)
        base_body_name = str(self.get_parameter("base_body_name").value)
        self._joints: List[str] = list(self.get_parameter("controlled_joints").value)
        self._active_mask = np.array(
            [float(v) for v in list(self.get_parameter("active_axes").value)[:3]],
            dtype=np.float64,
        )
        self._gains = AxisGains(
            mass=np.array(
                [
                    self.get_parameter("mass_x").value,
                    self.get_parameter("mass_y").value,
                    self.get_parameter("mass_z").value,
                ],
                dtype=np.float64,
            ),
            damping=np.array(
                [
                    self.get_parameter("damping_x").value,
                    self.get_parameter("damping_y").value,
                    self.get_parameter("damping_z").value,
                ],
                dtype=np.float64,
            ),
            stiffness=np.array(
                [
                    self.get_parameter("stiffness_x").value,
                    self.get_parameter("stiffness_y").value,
                    self.get_parameter("stiffness_z").value,
                ],
                dtype=np.float64,
            ),
        )
        self._force_deadband = np.array(
            [
                self.get_parameter("force_deadband_x").value,
                self.get_parameter("force_deadband_y").value,
                self.get_parameter("force_deadband_z").value,
            ],
            dtype=np.float64,
        )

        hz = float(self.get_parameter("control_frequency").value)
        self._dt = 1.0 / max(hz, 1.0)
        self._lam = float(self.get_parameter("dls_lambda").value)
        self._wrench_filter_alpha = float(self.get_parameter("wrench_filter_alpha").value)
        self._home_capture_duration = float(self.get_parameter("home_capture_duration").value)
        self._ee_vel_max = float(self.get_parameter("ee_vel_max").value)
        self._ee_disp_max = float(self.get_parameter("ee_disp_max").value)
        self._ee_track_gain = float(self.get_parameter("ee_track_gain").value)
        self._return_track_gain = float(self.get_parameter("return_track_gain").value)
        self._return_ee_vel_max = float(self.get_parameter("return_ee_vel_max").value)
        self._nullspace_gain = float(self.get_parameter("nullspace_gain").value)
        self._qdot_des_max = float(self.get_parameter("qdot_des_max").value)
        self._return_qdot_des_max = float(self.get_parameter("return_qdot_des_max").value)
        self._qdot_smoothing_alpha = float(self.get_parameter("qdot_smoothing_alpha").value)
        self._joint_limit_margin = float(self.get_parameter("joint_limit_margin").value)
        self._kp = float(self.get_parameter("torque_kp").value)
        self._kd = float(self.get_parameter("torque_kd").value)
        self._max_torque = float(self.get_parameter("max_torque").value)
        settle_disp_eps = float(self.get_parameter("settle_displacement_epsilon").value)
        settle_vel_eps = float(self.get_parameter("settle_velocity_epsilon").value)
        self._settle_disp_eps = np.full(3, settle_disp_eps, dtype=np.float64)
        self._settle_vel_eps = np.full(3, settle_vel_eps, dtype=np.float64)

        wrench_topic = str(self.get_parameter("wrench_topic").value)
        cmd_topic = str(self.get_parameter("joint_command_topic").value)
        pose_topic = str(self.get_parameter("ee_pose_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)

        self._ee_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name
        )
        self._base_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, base_body_name
        )
        if self._ee_body_id < 0:
            raise RuntimeError(f"end-effector body missing: {ee_body_name}")
        if self._base_body_id < 0:
            raise RuntimeError(f"base body missing: {base_body_name}")

        self._qadr: List[int] = []
        self._vadr: List[int] = []
        self._qmin: List[float] = []
        self._qmax: List[float] = []
        for joint_name in self._joints:
            joint_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id < 0:
                raise RuntimeError(f"controlled joint missing: {joint_name}")
            self._qadr.append(int(self._model.jnt_qposadr[joint_id]))
            self._vadr.append(int(self._model.jnt_dofadr[joint_id]))
            self._qmin.append(float(self._model.jnt_range[joint_id, 0]))
            self._qmax.append(float(self._model.jnt_range[joint_id, 1]))

        self._tau_limits = np.full(len(self._joints), self._max_torque)
        for actuator_id in range(self._model.nu):
            actuator_name = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
            ) or ""
            if not actuator_name.endswith("_tau"):
                continue
            joint_id = int(self._model.actuator_trnid[actuator_id, 0])
            joint_name = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            ) or ""
            if joint_name in self._joints:
                idx = self._joints.index(joint_name)
                ctrl_range = self._model.actuator_ctrlrange[actuator_id]
                self._tau_limits[idx] = float(max(abs(ctrl_range[0]), abs(ctrl_range[1])))

        self._all_joint_qadr: Dict[str, int] = {}
        self._all_joint_vadr: Dict[str, int] = {}
        for joint_id in range(self._model.njnt):
            joint_name = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            )
            if not joint_name:
                continue
            joint_type = int(self._model.jnt_type[joint_id])
            if joint_type in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ):
                self._all_joint_qadr[joint_name] = int(self._model.jnt_qposadr[joint_id])
                self._all_joint_vadr[joint_name] = int(self._model.jnt_dofadr[joint_id])

        self._adm_state = AdmittanceState.zeros(dim=3)
        self._force_raw = np.zeros(3, dtype=np.float64)
        self._force_filtered = np.zeros(3, dtype=np.float64)
        self._force_frame_id = "base_link"
        self._q_cur = np.zeros(len(self._joints), dtype=np.float64)
        self._qdot_cur = np.zeros(len(self._joints), dtype=np.float64)
        self._q_dot_cmd = np.zeros(len(self._joints), dtype=np.float64)
        self._q_home = np.zeros(len(self._joints), dtype=np.float64)
        self._full_state: Dict[str, tuple[float, float]] = {}
        self._ee_home_cmd: np.ndarray | None = None
        self._home_capture_deadline_ns: int | None = None
        self._initialized = False
        self._warned_frame = False

        self._pub_cmd = self.create_publisher(JointState, cmd_topic, 10)
        self._pub_pose = self.create_publisher(PoseStamped, pose_topic, 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, debug_topic, 10)
        self.create_subscription(WrenchStamped, wrench_topic, self._wrench_cb, 10)
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 20)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"arm admittance: frame={self._command_frame}, active_axes={self._active_mask.tolist()}, "
            f"M={self._gains.mass.tolist()}, D={self._gains.damping.tolist()}, "
            f"K={self._gains.stiffness.tolist()}"
        )

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        self._force_raw[:] = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ]
        self._force_frame_id = msg.header.frame_id or "base_link"

    def _joint_state_cb(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            position = float(msg.position[i]) if i < len(msg.position) else 0.0
            velocity = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
            self._full_state[name] = (position, velocity)

        for idx, joint_name in enumerate(self._joints):
            if joint_name in self._full_state:
                self._q_cur[idx], self._qdot_cur[idx] = self._full_state[joint_name]

        if not self._initialized and all(name in self._full_state for name in self._joints):
            self._initialized = True
            self._q_home = self._q_cur.copy()
            self._home_capture_deadline_ns = self.get_clock().now().nanoseconds + int(
                self._home_capture_duration * 1e9
            )
            self.get_logger().info("joint state initialized")

    def _sync_model_state(self) -> None:
        for name, (position, velocity) in self._full_state.items():
            if name in self._all_joint_qadr:
                self._data.qpos[self._all_joint_qadr[name]] = position
                self._data.qvel[self._all_joint_vadr[name]] = velocity
        mujoco.mj_forward(self._model, self._data)

    def _rotation_world_body(self, body_id: int) -> np.ndarray:
        return np.asarray(self._data.xmat[body_id], dtype=np.float64).reshape(3, 3)

    def _ee_pose_frames(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        base_rot = self._rotation_world_body(self._base_body_id)
        base_pos = np.asarray(self._data.xpos[self._base_body_id], dtype=np.float64)
        ee_pos_world = np.asarray(self._data.xpos[self._ee_body_id], dtype=np.float64)
        ee_pos_base = rotate_world_vector_to_body(ee_pos_world - base_pos, base_rot)
        return ee_pos_world, ee_pos_base, base_rot

    def _force_in_command_frame(self, base_rot: np.ndarray) -> np.ndarray:
        force = self._force_raw.copy()
        frame = self._force_frame_id
        if self._command_frame == "base_link":
            if frame in ("", "base_link"):
                return force
            if frame in ("world", "odom"):
                return rotate_world_vector_to_body(force, base_rot)
        else:
            if frame in ("", "world", "odom"):
                return force
            if frame == "base_link":
                return rotate_body_vector_to_world(force, base_rot)
        if not self._warned_frame:
            self.get_logger().warn(
                f"unknown wrench frame {frame}, using it as {self._command_frame}"
            )
            self._warned_frame = True
        return force

    def _publish_ee_pose(self, stamp, ee_pos_cmd: np.ndarray) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._command_frame
        pose.pose.position.x = float(ee_pos_cmd[0])
        pose.pose.position.y = float(ee_pos_cmd[1])
        pose.pose.position.z = float(ee_pos_cmd[2])
        self._pub_pose.publish(pose)

    def _publish_debug(
        self,
        admittance_displacement: np.ndarray,
        admittance_velocity: np.ndarray,
        ee_des_cmd: np.ndarray,
        ee_vel_cmd: np.ndarray,
        qdot_cmd: np.ndarray,
        tau: np.ndarray,
    ) -> None:
        msg = Float64MultiArray()
        msg.data = [
            *[float(value) for value in admittance_displacement[:3]],
            *[float(value) for value in admittance_velocity[:3]],
            *[float(value) for value in ee_des_cmd[:3]],
            *[float(value) for value in ee_vel_cmd[:3]],
            float(np.linalg.norm(qdot_cmd)),
            float(np.linalg.norm(tau)),
            float(np.max(np.abs(tau))) if len(tau) else 0.0,
        ]
        self._pub_debug.publish(msg)

    def _publish_torque_command(self, tau: np.ndarray, ee_pos_cmd: np.ndarray) -> None:
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self._joints)
        cmd.effort = [float(value) for value in tau]
        self._pub_cmd.publish(cmd)
        self._publish_ee_pose(cmd.header.stamp, ee_pos_cmd)

    def _control_loop(self) -> None:
        if not self._initialized:
            return

        self._sync_model_state()
        ee_pos_world, ee_pos_base, base_rot = self._ee_pose_frames()
        ee_pos_cmd = ee_pos_base if self._command_frame == "base_link" else ee_pos_world

        jacp = np.zeros((3, self._model.nv), dtype=np.float64)
        jacr = np.zeros((3, self._model.nv), dtype=np.float64)
        mujoco.mj_jacBody(self._model, self._data, jacp, jacr, self._ee_body_id)
        jac_cmd = jacp[:, self._vadr]
        if self._command_frame == "base_link":
            jac_cmd = base_rot.T @ jac_cmd

        tau_bias = np.array([self._data.qfrc_bias[v] for v in self._vadr], dtype=np.float64)
        m_full = np.zeros((self._model.nv, self._model.nv), dtype=np.float64)
        mujoco.mj_fullM(self._model, m_full, self._data.qM)
        m_arm = m_full[np.ix_(self._vadr, self._vadr)]
        tau_lim = np.minimum(self._tau_limits, self._max_torque)

        now_ns = self.get_clock().now().nanoseconds
        if self._home_capture_deadline_ns is not None and now_ns <= self._home_capture_deadline_ns:
            self._ee_home_cmd = ee_pos_cmd.copy()
            self._q_home = self._q_cur.copy()
            self._adm_state = AdmittanceState.zeros(dim=3)
            self._force_filtered[:] = 0.0
            self._q_dot_cmd[:] = 0.0
            tau = tau_bias + m_arm @ (self._kd * (-self._qdot_cur))
            tau = np.clip(tau, -tau_lim, tau_lim)
            self._publish_torque_command(tau, ee_pos_cmd)
            return

        if self._ee_home_cmd is None:
            self._ee_home_cmd = ee_pos_cmd.copy()
            self._q_home = self._q_cur.copy()

        force_cmd = self._force_in_command_frame(base_rot)
        self._force_filtered = (
            (1.0 - self._wrench_filter_alpha) * self._force_filtered
            + self._wrench_filter_alpha * force_cmd
        )
        force_cmd = apply_deadband(
            self._active_mask * self._force_filtered,
            self._force_deadband,
        )
        return_mode = not np.any(np.abs(force_cmd) > 1.0e-9)

        self._adm_state = self._adm_state.step(force=force_cmd, gains=self._gains, dt=self._dt)
        disp = np.clip(
            self._adm_state.displacement,
            -self._ee_disp_max,
            self._ee_disp_max,
        )
        vel = clip_norm(self._adm_state.velocity, self._ee_vel_max)
        for axis in range(3):
            if abs(disp[axis]) >= self._ee_disp_max and vel[axis] * disp[axis] > 0.0:
                vel[axis] = 0.0
        self._adm_state = settle_admittance_state(
            state=AdmittanceState(displacement=disp, velocity=vel),
            force=force_cmd,
            force_epsilon=np.full(3, 1.0e-9, dtype=np.float64),
            displacement_epsilon=self._settle_disp_eps,
            velocity_epsilon=self._settle_vel_eps,
        )

        ee_des_cmd = self._ee_home_cmd + self._adm_state.displacement
        track_gain = self._return_track_gain if return_mode else self._ee_track_gain
        ee_vel_max = self._return_ee_vel_max if return_mode else self._ee_vel_max
        qdot_des_max = self._return_qdot_des_max if return_mode else self._qdot_des_max
        ee_vel_cmd = self._adm_state.velocity + track_gain * (ee_des_cmd - ee_pos_cmd)
        ee_vel_cmd = clip_norm(ee_vel_cmd, ee_vel_max)

        qdot_target = solve_dls_velocity_with_nullspace(
            jacobian=jac_cmd,
            ee_velocity=ee_vel_cmd,
            damping_lambda=self._lam,
            q_error=self._q_home - self._q_cur,
            nullspace_gain=self._nullspace_gain,
        )
        qdot_target = np.clip(qdot_target, -qdot_des_max, qdot_des_max)
        qdot_target = limit_joint_velocity_near_limits(
            q=self._q_cur,
            qdot_des=qdot_target,
            qmin=np.array(self._qmin, dtype=np.float64),
            qmax=np.array(self._qmax, dtype=np.float64),
            margin=self._joint_limit_margin,
        )
        alpha = self._qdot_smoothing_alpha
        self._q_dot_cmd = (1.0 - alpha) * self._q_dot_cmd + alpha * qdot_target

        q_des_step = np.clip(
            self._q_cur + self._q_dot_cmd * self._dt,
            np.array(self._qmin, dtype=np.float64),
            np.array(self._qmax, dtype=np.float64),
        )
        delta_qdot = self._q_dot_cmd - self._qdot_cur
        delta_q = q_des_step - self._q_cur
        tau = tau_bias + m_arm @ (self._kd * delta_qdot + self._kp * delta_q)
        tau = np.clip(tau, -tau_lim, tau_lim)

        self._publish_debug(
            admittance_displacement=self._adm_state.displacement,
            admittance_velocity=self._adm_state.velocity,
            ee_des_cmd=ee_des_cmd,
            ee_vel_cmd=ee_vel_cmd,
            qdot_cmd=self._q_dot_cmd,
            tau=tau,
        )
        self._publish_torque_command(tau, ee_pos_cmd)


def main() -> None:
    rclpy.init()
    node = ArmAdmittanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok():
            raise exc
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    main()
