"""移动机械臂控制栈：仿真 + 状态汇总 + WBC 协调器（占位）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
    )
    viewer_arg = DeclareLaunchArgument(
        "use_viewer",
        default_value="false",
    )
    demo_arg = DeclareLaunchArgument(
        "demo_motion",
        default_value="false",
    )

    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {
                "use_viewer": ParameterValue(
                    LaunchConfiguration("use_viewer"), value_type=bool
                )
            },
            {
                "demo_motion": ParameterValue(
                    LaunchConfiguration("demo_motion"), value_type=bool
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
    )

    return LaunchDescription(
        [
            model_arg,
            viewer_arg,
            demo_arg,
            sim,
            state_est,
            wbc,
        ]
    )
