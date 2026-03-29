from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        description="MJCF 场景路径",
    )
    viewer_arg = DeclareLaunchArgument(
        "use_viewer",
        default_value="true",
        description="是否打开 MuJoCo 窗口",
    )

    sim_node = Node(
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
        ],
    )

    return LaunchDescription([model_arg, viewer_arg, sim_node])
