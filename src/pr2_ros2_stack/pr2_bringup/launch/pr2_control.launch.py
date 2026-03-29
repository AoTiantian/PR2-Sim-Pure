from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("pr2_bringup"), "config", "pr2_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("pr2_description"), "urdf", "pr2.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="/pr2",
        parameters=[robot_description, controllers_yaml],
        output={"stdout": "screen", "stderr": "screen"},
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/pr2",
        output="screen",
        parameters=[robot_description, {"frame_prefix": "/pr2/"}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/pr2",
        arguments=["joint_state_broadcaster", "--controller-manager", "/pr2/controller_manager"],
    )

    pr2_omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/pr2",
        arguments=["pr2_omni_controller", "--controller-manager", "/pr2/controller_manager"],
    )

    return LaunchDescription(
        [
            ros2_control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            pr2_omni_controller_spawner,
        ]
    )
