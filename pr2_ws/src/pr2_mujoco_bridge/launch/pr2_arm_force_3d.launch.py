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
        description="注入力的轴向: x / y / z")
    force_mag_arg = DeclareLaunchArgument(
        "force_magnitude", default_value="10.0",
        description="施加力大小 [N]")
    waveform_arg = DeclareLaunchArgument(
        "waveform", default_value="step",
        description="波形: step / ramp / sine")

    model_path = LaunchConfiguration("model_path")
    log_file = LaunchConfiguration("log_file")
    force_axis = LaunchConfiguration("force_axis")
    force_magnitude = LaunchConfiguration("force_magnitude")
    waveform = LaunchConfiguration("waveform")

    sim_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "use_viewer": True,
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
            "damping_x": 150.0,
            "damping_y": 150.0,
            "damping_z": 150.0,
            "control_frequency": 100.0,
            "dls_lambda": 0.08,
            "torque_kp": 300.0,
            "torque_kd": 25.0,
            "max_torque": 80.0,
            "ee_body_name": "l_gripper_tool_frame",
        }],
    )

    injector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_force_injector",
        name="pr2_arm_force_injector",
        output="screen",
        parameters=[{
            "force_axis": force_axis,
            "force_magnitude": force_magnitude,
            "step_duration": 3.0,
            "injection_delay": 1.0,
            "log_file": log_file,
            "publish_rate": 100.0,
            "waveform": waveform,
        }],
    )

    return LaunchDescription([
        model_arg, log_arg, force_axis_arg, force_mag_arg, waveform_arg,
        sim_node,
        admittance_node,
        injector_node,
    ])
