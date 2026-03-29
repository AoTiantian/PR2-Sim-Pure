from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        description="MuJoCo MJCF 场景路径",
    )

    ik_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_left_arm_ik",
        name="pr2_left_arm_ik",
        output="screen",
        parameters=[{"model_path": LaunchConfiguration("model_path")}],
    )

    return LaunchDescription([model_arg, ik_node])
