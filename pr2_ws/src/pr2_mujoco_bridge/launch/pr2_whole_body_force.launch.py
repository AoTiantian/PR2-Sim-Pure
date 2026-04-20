"""Phase 3 演示启动文件：臂＋底盘全身笛卡尔力顺应控制。

节点拓扑：
  外部力 --> pr2_arm_force_injector --> /wbc/arm/external_wrench
                |
                +--> pr2_arm_admittance --> /wbc/arm/joint_command --> pr2_wbc_coordinator
                |                                                            |
                +--> pr2_force_projector --> /wbc/external_wrench            +--> /joint_commands --> pr2_mujoco_sim
                         |                                                   |
                         v                                                   +--> /cmd_vel  --> pr2_mujoco_sim
               pr2_admittance_stub --> /wbc/base_acceleration
                         |
               pr2_base_accel_integrator --> /wbc/reference/cmd_vel --> pr2_wbc_coordinator

用法：
  ros2 launch pr2_mujoco_bridge pr2_whole_body_force.launch.py \\
      model_path:=/path/to/scene.xml force_axis:=x
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
        "log_file", default_value="/tmp/arm_response_wb.csv",
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

    model_path    = LaunchConfiguration("model_path")
    log_file      = LaunchConfiguration("log_file")
    force_axis    = LaunchConfiguration("force_axis")
    force_mag     = LaunchConfiguration("force_magnitude")
    waveform      = LaunchConfiguration("waveform")

    # ── 仿真器 ────────────────────────────────────────────────────────────
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
            "damping_x": 150.0, "damping_y": 150.0, "damping_z": 150.0,
            "control_frequency":    100.0,
            "dls_lambda":           0.08,
            "torque_kp":            300.0,
            "torque_kd":            25.0,
            "max_torque":           80.0,
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
            "output_topic": "wbc/external_wrench",
            "force_scale":  0.4,
        }],
    )

    # ── 底盘顺应控制器 ────────────────────────────────────────────────────
    base_adm_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_admittance_stub",
        name="pr2_base_admittance",
        output="screen",
        parameters=[{
            "input_topic":       "wbc/external_wrench",
            "output_topic":      "wbc/base_acceleration",
            "mass_linear":       [8.0, 8.0, 12.0],
            "damping_linear":    [80.0, 80.0, 120.0],
            "stiffness_linear":  [0.0, 0.0, 0.0],
            "mass_angular":      [1.5, 1.5, 1.0],
            "damping_angular":   [18.0, 18.0, 12.0],
        }],
    )

    # ── 底盘加速度积分器 ──────────────────────────────────────────────────
    base_int_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_base_accel_integrator",
        name="pr2_base_accel_integrator",
        output="screen",
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
            "force_axis":       force_axis,
            "force_magnitude":  force_mag,
            "step_duration":    3.0,
            "injection_delay":  1.0,
            "log_file":         log_file,
            "publish_rate":     100.0,
            "waveform":         waveform,
        }],
    )

    return LaunchDescription([
        model_arg, log_arg, force_axis_arg, force_mag_arg, waveform_arg,
        sim_node,
        arm_adm_node,
        projector_node,
        base_adm_node,
        base_int_node,
        wbc_node,
        injector_node,
    ])
