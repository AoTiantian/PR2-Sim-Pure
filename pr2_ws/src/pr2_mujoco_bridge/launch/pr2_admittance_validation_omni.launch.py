"""One-command omni-direction admittance validation preset."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
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
    viewer_arg = DeclareLaunchArgument("use_viewer", default_value="false")

    # Default schedule: +X -> +Y -> -X -> -Y, each 5s.
    force_schedule = (
        '[{"start":2.0,"end":7.0,"fx":80.0},'
        '{"start":7.0,"end":12.0,"fy":80.0},'
        '{"start":12.0,"end":17.0,"fx":-80.0},'
        '{"start":17.0,"end":22.0,"fy":-80.0}]'
    )

    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"demo_motion": False},
            {"use_viewer": ParameterValue(LaunchConfiguration("use_viewer"), value_type=bool)},
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
            {"force_schedule_json": force_schedule},
            {"settle_after_sec": 3.0},
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
            sim,
            state_est,
            wbc,
            integrator,
            admittance,
            validator,
            shutdown_on_done,
        ]
    )

