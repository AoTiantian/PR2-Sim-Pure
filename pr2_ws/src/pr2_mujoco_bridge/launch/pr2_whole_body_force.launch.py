"""Phase 3 演示启动文件：臂＋底盘联动顺应控制。

节点拓扑：
  外部力 --> pr2_arm_force_injector --> /wbc/arm/external_wrench
                |
                +--> pr2_arm_admittance --> /wbc/arm/joint_command --> pr2_wbc_coordinator
                |                                                            |
                +--> pr2_force_projector --> /wbc/external_wrench            +--> /joint_commands --> pr2_mujoco_sim
                         |                                                   |
                         v                                                   +--> /cmd_vel  --> pr2_mujoco_sim
                 pr2_base_admittance --> /wbc/reference/cmd_vel

用法：
  ros2 launch pr2_mujoco_bridge pr2_whole_body_force.launch.py \\
      model_path:=/path/to/scene.xml force_axis:=x
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
        "log_file", default_value="/tmp/arm_response_wb.csv",
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

    model_path    = LaunchConfiguration("model_path")
    log_file      = LaunchConfiguration("log_file")
    force_axis    = LaunchConfiguration("force_axis")
    force_mag     = LaunchConfiguration("force_magnitude")
    waveform      = LaunchConfiguration("waveform")
    settle_duration = LaunchConfiguration("settle_duration")
    use_viewer    = LaunchConfiguration("use_viewer")

    # ── 仿真器 ────────────────────────────────────────────────────────────
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

    # ── 臂顺应控制器（输出到 wbc/arm/joint_command，经协调器合并） ────────
    arm_adm_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_admittance",
        name="pr2_arm_admittance",
        output="screen",
        parameters=[{
            "model_path":           model_path,
            "active_axes":          [1, 1, 1],
            "mass_x": 2.0, "mass_y": 2.0, "mass_z": 2.0,
            "damping_x": 46.0, "damping_y": 48.0, "damping_z": 52.0,
            "stiffness_x": 58.0, "stiffness_y": 60.0, "stiffness_z": 66.0,
            "control_frequency":    100.0,
            "dls_lambda":           0.28,
            "wrench_filter_alpha":  0.28,
            "ee_vel_max":           0.022,
            "ee_disp_max":          0.07,
            "ee_track_gain":        9.0,
            "qdot_des_max":         0.12,
            "qdot_smoothing_alpha": 0.34,
            "torque_kp":            18.0,
            "torque_kd":            18.0,
            "max_torque":           60.0,
            "ee_body_name":         "l_gripper_tool_frame",
            "joint_command_topic":  "wbc/arm/joint_command",  # 路由到协调器
        }],
    )

    # ── 力投影器（EE力 -> 底盘顺应） ─────────────────────────────────────
    projector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_force_projector",
        name="pr2_force_projector",
        output="screen",
        parameters=[{
            "input_topic":  "wbc/arm/external_wrench",
            "ee_pose_topic": "wbc/arm/ee_pose_log",
            "output_topic": "wbc/external_wrench",
            "planar_force_scale": 0.08,
            "yaw_torque_scale": 0.06,
            "filter_alpha": 0.2,
        }],
    )

    # ── 底盘顺应控制器 ────────────────────────────────────────────────────
    base_adm_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_base_admittance",
        name="pr2_base_admittance",
        output="screen",
        parameters=[{
            "input_topic":       "wbc/external_wrench",
            "odom_topic":        "odom",
            "output_topic":      "wbc/reference/cmd_vel",
            "mass_linear":       [18.0, 18.0],
            "damping_linear":    [70.0, 70.0],
            "stiffness_linear":  [42.0, 42.0],
            "mass_angular":      6.0,
            "damping_angular":   22.0,
            "stiffness_angular": 16.0,
            "max_linear_speed":  0.10,
            "max_angular_speed": 0.20,
        }],
    )

    # ── WBC 协调器（合并臂关节指令 + 底盘速度） ──────────────────────────
    wbc_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="screen",
        parameters=[{
            "arm_joint_cmd_topic": "wbc/arm/joint_command",
            "nullspace_enable": False,
        }],
    )

    # ── 力注入器 ──────────────────────────────────────────────────────────
    injector_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_force_injector",
        name="pr2_arm_force_injector",
        output="screen",
        parameters=[{
            "force_axis":       ParameterValue(force_axis, value_type=str),
            "force_magnitude":  force_mag,
            "step_duration":    3.0,
            "injection_delay":  1.0,
            "settle_duration":  settle_duration,
            "log_file":         log_file,
            "publish_rate":     100.0,
            "waveform":         ParameterValue(waveform, value_type=str),
        }],
    )

    return LaunchDescription([
        model_arg, log_arg, force_axis_arg, force_mag_arg, waveform_arg, settle_arg, viewer_arg,
        sim_node,
        arm_adm_node,
        projector_node,
        base_adm_node,
        wbc_node,
        injector_node,
    ])
