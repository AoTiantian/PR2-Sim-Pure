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
    ee_arg = DeclareLaunchArgument(
        "end_effector_body",
        default_value="l_gripper_tool_frame",
        description="要监测的末端 body 名称",
    )

    node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_ee_pose_publisher",
        name="pr2_ee_pose_publisher",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"end_effector_body": LaunchConfiguration("end_effector_body")},
        ],
    )

    return LaunchDescription([model_arg, ee_arg, node])
