from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
        description="MJCF 场景路径",
    )
    viewer_arg = DeclareLaunchArgument(
        "use_viewer",
        default_value="true",
        description="是否打开 MuJoCo 窗口",
    )
    joint_log_hz_arg = DeclareLaunchArgument(
        "joint_motion_log_rate_hz",
        default_value="1.0",
        description="仿真节点控制台打印关节 pos/vel 的频率 (Hz)，0 关闭",
    )
    joint_log_re_arg = DeclareLaunchArgument(
        "joint_motion_log_regex",
        default_value="",
        description="仅打印名称匹配该正则的关节；留空则用默认（躯干/左臂/夹爪）",
    )
    log_path_arg = DeclareLaunchArgument(
        "log_path",
        default_value="",
        description="CSV 日志输出路径；留空则输出到 /workspace/logs/ 下自动命名",
    )
    log_prefix_arg = DeclareLaunchArgument(
        "log_prefix",
        default_value="pr2_sim",
        description="当 log_path 为空时，自动文件名前缀（例如 pr2_sim_YYYYMMDD_HHMMSS.csv）",
    )

    sim_node = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {
                "use_viewer": ParameterValue(
                    LaunchConfiguration("use_viewer"), value_type=bool
                )
            },
            {
                "joint_motion_log_rate_hz": ParameterValue(
                    LaunchConfiguration("joint_motion_log_rate_hz"),
                    value_type=float,
                )
            },
            {
                "joint_motion_log_regex": ParameterValue(
                    LaunchConfiguration("joint_motion_log_regex"), value_type=str
                )
            },
        ],
    )

    # Sim-only minimal logging: focus on joint_states/odom, skip ee/wrench/cart_vel related topics.
    logger = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_motion_logger",
        name="pr2_sim_logger",
        output="screen",
        parameters=[
            {"output_path": ParameterValue(LaunchConfiguration("log_path"), value_type=str)},
            {"output_prefix": ParameterValue(LaunchConfiguration("log_prefix"), value_type=str)},
            {"ee_pose_topic": ""},
            {"ik_target_topic": ""},
            {"wrench_topic": ""},
            {"cartesian_velocity_topic": ""},
            {"base_pose_latched_topic": ""},
            {"wbc_joint_ref_topic": ""},
            {"wbc_cmd_vel_topic": ""},
            {"state_joint_topic": ""},
            {"joint_state_topic": "joint_states"},
            {"odom_topic": "odom"},
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            viewer_arg,
            joint_log_hz_arg,
            joint_log_re_arg,
            log_path_arg,
            log_prefix_arg,
            sim_node,
            logger,
        ]
    )
