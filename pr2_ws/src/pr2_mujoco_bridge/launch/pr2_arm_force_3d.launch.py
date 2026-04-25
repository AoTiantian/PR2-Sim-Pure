"""Single-arm 3D Cartesian force-response demo."""

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
        default_value="/tmp/arm_response_3d.csv",
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
        default_value="5.0",
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
    force_magnitude = LaunchConfiguration("force_magnitude")
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

    admittance_node = Node(
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
            "damping_x": 40.0,
            "damping_y": 42.0,
            "damping_z": 46.0,
            "stiffness_x": 46.0,
            "stiffness_y": 48.0,
            "stiffness_z": 54.0,
            "control_frequency": 100.0,
            "dls_lambda": 0.28,
            "wrench_filter_alpha": 0.28,
            "ee_vel_max": 0.028,
            "ee_disp_max": 0.08,
            "ee_track_gain": 9.0,
            "return_track_gain": 10.5,
            "return_ee_vel_max": 0.036,
            "nullspace_gain": 0.12,
            "qdot_des_max": 0.16,
            "return_qdot_des_max": 0.18,
            "qdot_smoothing_alpha": 0.34,
            "torque_kp": 18.0,
            "torque_kd": 18.0,
            "max_torque": 60.0,
            "settle_displacement_epsilon": 0.003,
            "settle_velocity_epsilon": 0.003,
            "ee_body_name": "l_gripper_tool_frame",
        }],
    )

    injector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_force_injector",
        name="pr2_arm_force_injector",
        output="screen",
        parameters=[{
            "force_axis": ParameterValue(force_axis, value_type=str),
            "force_magnitude": force_magnitude,
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
        admittance_node,
        injector_node,
        shutdown_when_done,
    ])
