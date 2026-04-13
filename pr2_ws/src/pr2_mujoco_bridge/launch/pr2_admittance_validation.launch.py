"""One-command admittance validation launch (no null-space)."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
    )
    # Validation reliability is better in headless mode; set true if you need window.
    viewer_arg = DeclareLaunchArgument("use_viewer", default_value="false")
    force_arg = DeclareLaunchArgument("force_x", default_value="30.0")
    # duration_sec is the virtual-force active duration.
    duration_arg = DeclareLaunchArgument("duration_sec", default_value="3.0")
    force_start_arg = DeclareLaunchArgument("force_start_sec", default_value="2.0")
    settle_arg = DeclareLaunchArgument("settle_after_sec", default_value="3.0")

    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"demo_motion": False},
            {
                "use_viewer": ParameterValue(
                    LaunchConfiguration("use_viewer"), value_type=bool
                )
            },
        ],
    )

    state_est = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_state_estimator",
        name="pr2_state_estimator",
        output="screen",
    )

    wbc = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="screen",
        parameters=[{"nullspace_enable": False}],
    )

    integrator = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_base_accel_integrator",
        name="pr2_base_accel_integrator",
        output="screen",
    )

    admittance = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_admittance_controller",
        name="pr2_admittance_controller",
        output="screen",
    )

    validator = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_admittance_validator",
        name="pr2_admittance_validator",
        output="screen",
        parameters=[
            {
                "force_x": ParameterValue(
                    LaunchConfiguration("force_x"), value_type=float
                )
            },
            {
                "force_start_sec": ParameterValue(
                    LaunchConfiguration("force_start_sec"), value_type=float
                )
            },
            {
                "duration_sec": ParameterValue(
                    LaunchConfiguration("duration_sec"), value_type=float
                )
            },
            {
                "settle_after_sec": ParameterValue(
                    LaunchConfiguration("settle_after_sec"), value_type=float
                )
            },
        ],
    )

    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=validator, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            model_arg,
            viewer_arg,
            force_arg,
            duration_arg,
            force_start_arg,
            settle_arg,
            sim,
            state_est,
            wbc,
            integrator,
            admittance,
            validator,
            shutdown_on_done,
        ]
    )

