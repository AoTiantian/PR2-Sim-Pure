"""Launch the standalone virtual-human + board MuJoCo demo (no robot)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_path = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene_human_board.xml",
    )
    use_viewer = DeclareLaunchArgument("use_viewer", default_value="true")
    force_z = DeclareLaunchArgument(
        "force_z",
        default_value="-1.0",
        description="Upward hand force in N; negative means exact gravity compensation",
    )
    allow_rotation = DeclareLaunchArgument(
        "allow_rotation",
        default_value="false",
        description="Use a point contact with no stabilizing grip moment",
    )
    duration = DeclareLaunchArgument(
        "duration",
        default_value="0.0",
        description="Simulation duration in seconds; zero runs until shutdown",
    )

    sim = Node(
        package="pr2_virtual_human",
        executable="human_board_sim",
        name="human_board_sim",
        output="both",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {
                "use_viewer": ParameterValue(
                    LaunchConfiguration("use_viewer"), value_type=bool
                )
            },
            {
                "force_z": ParameterValue(
                    LaunchConfiguration("force_z"), value_type=float
                )
            },
            {
                "allow_rotation": ParameterValue(
                    LaunchConfiguration("allow_rotation"), value_type=bool
                )
            },
            {
                "duration": ParameterValue(
                    LaunchConfiguration("duration"), value_type=float
                )
            },
        ],
    )

    return LaunchDescription(
        [model_path, use_viewer, force_z, allow_rotation, duration, sim]
    )
