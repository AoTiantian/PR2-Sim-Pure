"""Whole-body force-response demo for the PR2 mobile manipulator."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _default_model_path() -> str:
    suffix = Path("unitree_mujoco") / "unitree_robots" / "pr2" / "scene.xml"
    candidates = []
    env_path = os.environ.get("PR2_MUJOCO_MODEL_PATH")
    if env_path:
        candidates.append(Path(env_path))
    cwd = Path.cwd().resolve()
    candidates.extend([
        cwd / suffix,
        cwd.parent / suffix,
        Path(__file__).resolve().parents[4] / suffix,
        Path("/workspace") / suffix,
    ])
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return str(candidates[-1])


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value=_default_model_path(),
        description="MuJoCo scene XML path",
    )
    log_arg = DeclareLaunchArgument(
        "log_file",
        default_value="/tmp/arm_response_wb.csv",
        description="CSV log path",
    )
    force_axis_arg = DeclareLaunchArgument(
        "force_axis",
        default_value="x",
        description="Injected force axis: x / y / z / xyz",
    )
    force_mag_arg = DeclareLaunchArgument(
        "force_magnitude",
        default_value="10.0",
        description="Injected force magnitude [N]",
    )
    waveform_arg = DeclareLaunchArgument(
        "waveform",
        default_value="step",
        description="Force waveform: step / ramp / sine",
    )
    settle_arg = DeclareLaunchArgument(
        "settle_duration",
        default_value="4.0",
        description="Observation window after force release [s]",
    )
    viewer_arg = DeclareLaunchArgument(
        "use_viewer",
        default_value="false",
        description="Enable the MuJoCo viewer",
    )

    model_path = LaunchConfiguration("model_path")
    log_file = LaunchConfiguration("log_file")
    force_axis = LaunchConfiguration("force_axis")
    force_mag = LaunchConfiguration("force_magnitude")
    waveform = LaunchConfiguration("waveform")
    settle_duration = LaunchConfiguration("settle_duration")
    use_viewer = LaunchConfiguration("use_viewer")

    sim_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "use_viewer": ParameterValue(use_viewer, value_type=bool),
            "demo_motion": False,
        }],
    )

    arm_adm_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_admittance",
        name="pr2_arm_admittance",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "active_axes": [1, 1, 1],
            "mass_x": 2.0,
            "mass_y": 2.0,
            "mass_z": 2.0,
            "damping_x": 46.0,
            "damping_y": 50.0,
            "damping_z": 54.0,
            "stiffness_x": 58.0,
            "stiffness_y": 60.0,
            "stiffness_z": 66.0,
            "control_frequency": 100.0,
            "dls_lambda": 0.28,
            "wrench_filter_alpha": 0.28,
            "ee_vel_max": 0.020,
            "ee_disp_max": 0.07,
            "ee_track_gain": 9.0,
            "return_track_gain": 10.5,
            "return_ee_vel_max": 0.030,
            "nullspace_gain": 0.10,
            "qdot_des_max": 0.10,
            "return_qdot_des_max": 0.12,
            "qdot_smoothing_alpha": 0.34,
            "torque_kp": 18.0,
            "torque_kd": 18.0,
            "max_torque": 60.0,
            "settle_displacement_epsilon": 0.003,
            "settle_velocity_epsilon": 0.003,
            "ee_body_name": "l_gripper_tool_frame",
            "joint_command_topic": "wbc/arm/joint_command",
        }],
    )

    projector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_force_projector",
        name="pr2_force_projector",
        output="screen",
        parameters=[{
            "input_topic": "wbc/arm/external_wrench",
            "ee_pose_topic": "wbc/arm/ee_pose_log",
            "output_topic": "wbc/external_wrench",
            "planar_force_scale": 0.05,
            "yaw_torque_scale": 0.015,
            "filter_alpha": 0.18,
        }],
    )

    base_adm_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_base_admittance",
        name="pr2_base_admittance",
        output="screen",
        parameters=[{
            "input_topic": "wbc/external_wrench",
            "odom_topic": "odom",
            "output_topic": "wbc/reference/cmd_vel",
            "mass_linear": [18.0, 18.0],
            "damping_linear": [76.0, 76.0],
            "stiffness_linear": [44.0, 44.0],
            "mass_angular": 6.0,
            "damping_angular": 24.0,
            "stiffness_angular": 17.0,
            "track_gain_linear": 3.2,
            "track_gain_angular": 2.6,
            "return_track_gain_linear": 5.8,
            "return_track_gain_angular": 3.8,
            "max_linear_speed": 0.09,
            "max_angular_speed": 0.20,
            "settle_linear_displacement_epsilon": 0.020,
            "settle_yaw_displacement_epsilon": 0.015,
            "settle_linear_velocity_epsilon": 0.004,
            "settle_angular_velocity_epsilon": 0.01,
            "idle_velocity_decay": 0.85,
        }],
    )

    wbc_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="screen",
        parameters=[{
            "arm_joint_cmd_topic": "wbc/arm/joint_command",
            "nullspace_enable": False,
        }],
    )

    injector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_force_injector",
        name="pr2_arm_force_injector",
        output="screen",
        parameters=[{
            "force_axis": ParameterValue(force_axis, value_type=str),
            "force_magnitude": force_mag,
            "step_duration": 3.0,
            "injection_delay": 1.0,
            "ready_delay": 1.5,
            "settle_duration": settle_duration,
            "log_file": log_file,
            "publish_rate": 100.0,
            "waveform": ParameterValue(waveform, value_type=str),
        }],
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=injector_node,
            on_exit=[EmitEvent(event=Shutdown(reason="force injection completed"))],
        )
    )

    return LaunchDescription([
        model_arg,
        log_arg,
        force_axis_arg,
        force_mag_arg,
        waveform_arg,
        settle_arg,
        viewer_arg,
        sim_node,
        arm_adm_node,
        projector_node,
        base_adm_node,
        wbc_node,
        injector_node,
        shutdown_when_done,
    ])
