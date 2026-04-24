"""Phase 2 演示启动文件：单臂全三维笛卡尔力顺应控制（不含力矩）。

基于 Phase 1 扩展，将 active_axes 开放为全三轴 [1,1,1]。
force_axis 参数可指定注入哪个轴的力（默认 x，可改为 y 或 z 分别验证）。

用法：
  # X 轴
  ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py model_path:=... force_axis:=x
  # Y 轴
  ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py model_path:=... force_axis:=y
  # Z 轴
  ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py model_path:=... force_axis:=z
"""

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
        "log_file", default_value="/tmp/arm_response_3d.csv",
        description="CSV 日志输出路径")
    force_axis_arg = DeclareLaunchArgument(
        "force_axis", default_value="x",
        description="注入力的轴向: x / y / z / xyz")
    force_mag_arg = DeclareLaunchArgument(
        "force_magnitude", default_value="10.0",
        description="施加力大小 [N]")
    waveform_arg = DeclareLaunchArgument(
        "waveform", default_value="step",
        description="波形: step / ramp / sine")
    settle_arg = DeclareLaunchArgument(
        "settle_duration", default_value="4.0",
        description="撤力后的留观时长 [s]")
    viewer_arg = DeclareLaunchArgument(
        "use_viewer", default_value="false",
        description="是否打开 MuJoCo viewer")

    model_path = LaunchConfiguration("model_path")
    log_file = LaunchConfiguration("log_file")
    force_axis = LaunchConfiguration("force_axis")
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
            "active_axes": [1, 1, 1],   # Phase 2: 全三轴
            "mass_x": 2.0,
            "mass_y": 2.0,
            "mass_z": 2.0,
            "damping_x": 40.0,
            "damping_y": 42.0,
            "damping_z": 46.0,
            "stiffness_x": 46.0,
            "stiffness_y": 48.0,
            "stiffness_z": 54.0,
            "control_frequency": 100.0,
            "dls_lambda": 0.28,
            "wrench_filter_alpha": 0.28,
            "ee_vel_max": 0.028,
            "ee_disp_max": 0.08,
            "ee_track_gain": 9.0,
            "qdot_des_max": 0.16,
            "qdot_smoothing_alpha": 0.34,
            "torque_kp": 18.0,
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
            "force_axis": ParameterValue(force_axis, value_type=str),
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
        model_arg, log_arg, force_axis_arg, force_mag_arg, waveform_arg, settle_arg, viewer_arg,
        sim_node,
        admittance_node,
        injector_node,
    ])
