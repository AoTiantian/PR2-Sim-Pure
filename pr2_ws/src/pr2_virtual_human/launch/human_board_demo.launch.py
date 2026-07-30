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
    pid_enable = DeclareLaunchArgument(
        "pid_enable",
        default_value="true",
        description="Track the horizontal trajectory using PID-controlled force",
    )
    trajectory_start_delay = DeclareLaunchArgument(
        "trajectory_start_delay",
        default_value="1.0",
        description="Seconds to hover before horizontal tracking starts",
    )
    trajectory_x_amplitude = DeclareLaunchArgument(
        "trajectory_x_amplitude",
        default_value="0.45",
        description="World-X amplitude of the horizontal figure-eight trajectory",
    )
    trajectory_y_amplitude = DeclareLaunchArgument(
        "trajectory_y_amplitude",
        default_value="0.30",
        description="World-Y amplitude of the horizontal figure-eight trajectory",
    )
    trajectory_period = DeclareLaunchArgument(
        "trajectory_period",
        default_value="10.0",
        description="Time to complete one closed figure-eight trajectory",
    )
    duration = DeclareLaunchArgument(
        "duration",
        default_value="11.0",
        description="Simulation duration in seconds; clamped to at most 15",
    )
    output_dir = DeclareLaunchArgument(
        "output_dir",
        default_value="/workspace/results/human_board",
        description="Root directory for timestamped CSV and plot outputs",
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
                "pid_enable": ParameterValue(
                    LaunchConfiguration("pid_enable"), value_type=bool
                )
            },
            {
                "trajectory_start_delay": ParameterValue(
                    LaunchConfiguration("trajectory_start_delay"), value_type=float
                )
            },
            {
                "trajectory_x_amplitude": ParameterValue(
                    LaunchConfiguration("trajectory_x_amplitude"), value_type=float
                )
            },
            {
                "trajectory_y_amplitude": ParameterValue(
                    LaunchConfiguration("trajectory_y_amplitude"), value_type=float
                )
            },
            {
                "trajectory_period": ParameterValue(
                    LaunchConfiguration("trajectory_period"), value_type=float
                )
            },
            {
                "duration": ParameterValue(
                    LaunchConfiguration("duration"), value_type=float
                )
            },
            {"output_dir": LaunchConfiguration("output_dir")},
        ],
    )

    return LaunchDescription(
        [
            model_path,
            use_viewer,
            force_z,
            allow_rotation,
            pid_enable,
            trajectory_start_delay,
            trajectory_x_amplitude,
            trajectory_y_amplitude,
            trajectory_period,
            duration,
            output_dir,
            sim,
        ]
    )
