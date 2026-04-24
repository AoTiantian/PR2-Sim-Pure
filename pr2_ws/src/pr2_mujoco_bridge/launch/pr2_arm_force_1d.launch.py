"""Phase 1 演示启动文件：单臂 X 轴方向笛卡尔力顺应控制。

启动节点：
  - pr2_mujoco_sim    MuJoCo 仿真器桥接
  - pr2_arm_admittance 臂顺应控制器（仅 X 轴激活）
  - pr2_arm_force_injector 力注入 + CSV 日志

用法：
  ros2 launch pr2_mujoco_bridge pr2_arm_force_1d.launch.py \\
      model_path:=/path/to/scene.xml \\
      log_file:=/tmp/arm_response_1d.csv
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_DEFAULT_MODEL = "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml"


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path", default_value=_DEFAULT_MODEL,
        description="MuJoCo 场景 XML 路径")
    log_arg = DeclareLaunchArgument(
        "log_file", default_value="/tmp/arm_response_1d.csv",
        description="CSV 日志输出路径")
    force_arg = DeclareLaunchArgument(
        "force_magnitude", default_value="10.0",
        description="施加力大小 [N]")
    waveform_arg = DeclareLaunchArgument(
        "waveform", default_value="step",
        description="波形类型: step / ramp / sine")
    settle_arg = DeclareLaunchArgument(
        "settle_duration", default_value="4.0",
        description="撤力后的留观时长 [s]")
    viewer_arg = DeclareLaunchArgument(
        "use_viewer", default_value="false",
        description="是否打开 MuJoCo viewer")

    model_path = LaunchConfiguration("model_path")
    log_file = LaunchConfiguration("log_file")
    force_magnitude = LaunchConfiguration("force_magnitude")
    waveform = LaunchConfiguration("waveform")
    settle_duration = LaunchConfiguration("settle_duration")
    use_viewer = LaunchConfiguration("use_viewer")

    sim_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "use_viewer": ParameterValue(use_viewer, value_type=bool),
            "demo_motion": False,
        }],
    )

    admittance_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_admittance",
        name="pr2_arm_admittance",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "active_axes": [1, 0, 0],   # Phase 1: 仅 X 轴
            "mass_x": 2.0,
            "mass_y": 2.0,
            "mass_z": 2.0,
            "damping_x": 38.0,
            "damping_y": 38.0,
            "damping_z": 42.0,
            "stiffness_x": 42.0,
            "stiffness_y": 42.0,
            "stiffness_z": 46.0,
            "control_frequency": 100.0,
            "dls_lambda": 0.25,
            "wrench_filter_alpha": 0.25,
            "ee_vel_max": 0.03,
            "ee_disp_max": 0.10,
            "ee_track_gain": 8.0,
            "qdot_des_max": 0.18,
            "qdot_smoothing_alpha": 0.3,
            "torque_kp": 16.0,
            "torque_kd": 18.0,
            "max_torque": 60.0,
            "ee_body_name": "l_gripper_tool_frame",
        }],
    )

    injector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_force_injector",
        name="pr2_arm_force_injector",
        output="screen",
        parameters=[{
            "force_axis": "x",
            "force_magnitude": force_magnitude,
            "step_duration": 3.0,
            "injection_delay": 1.0,
            "settle_duration": settle_duration,
            "log_file": log_file,
            "publish_rate": 100.0,
            "waveform": ParameterValue(waveform, value_type=str),
        }],
    )

    return LaunchDescription([
        model_arg, log_arg, force_arg, waveform_arg, settle_arg, viewer_arg,
        sim_node,
        admittance_node,
        injector_node,
    ])
