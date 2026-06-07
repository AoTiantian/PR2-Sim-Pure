"""Minimal PR2 board grasp demo.

This launch runs only the MuJoCo bridge with a free board in front of the left
gripper. The gripper starts open and the bridge's default position target closes
it, so the board grasp is produced by MuJoCo contact/friction.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene_grasp_board.xml",
    )
    viewer_arg = DeclareLaunchArgument("use_viewer", default_value="true")
    initial_qpos_arg = DeclareLaunchArgument(
        "initial_qpos_json",
        default_value=(
            '{"l_shoulder_pan_joint": 0.35, '
            '"l_shoulder_lift_joint": 0.95, '
            '"l_upper_arm_roll_joint": 0.0, '
            '"l_elbow_flex_joint": -1.35, '
            '"l_forearm_roll_joint": 0.0, '
            '"l_wrist_flex_joint": -0.55, '
            '"l_wrist_roll_joint": 0.0, '
            '"l_gripper_l_finger_joint": 0.548, '
            '"board_free": [-2.5334, 1.3713, 0.8858, '
            '0.779700, -0.301167, -0.344025, -0.427801]}'
        ),
    )

    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_board_grasp_demo",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {
                "use_viewer": ParameterValue(
                    LaunchConfiguration("use_viewer"), value_type=bool
                )
            },
            {"demo_motion": False},
            {"use_cmd_vel": False},
            {"lock_base_motion": True},
            {"lock_torso_motion": True},
            {"gripper_open_time_sec": 8.0},
            {"joint_motion_log_rate_hz": 1.0},
            {"joint_motion_log_regex": "l_gripper|board"},
            {
                "initial_qpos_json": ParameterValue(
                    LaunchConfiguration("initial_qpos_json"), value_type=str
                )
            },
        ],
    )

    return LaunchDescription([model_arg, viewer_arg, initial_qpos_arg, sim])
