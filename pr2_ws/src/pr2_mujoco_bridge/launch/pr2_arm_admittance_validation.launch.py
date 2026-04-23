"""One-command left-arm Cartesian admittance validation launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml",
    )
    viewer_arg = DeclareLaunchArgument("use_viewer", default_value="true")
    force_arg = DeclareLaunchArgument("force_x", default_value="10.0")
    duration_arg = DeclareLaunchArgument("duration_sec", default_value="10.0")
    force_start_arg = DeclareLaunchArgument("force_start_sec", default_value="5.0")
    # Keep optional schedule for advanced tests; empty means single-step mode.
    force_schedule_arg = DeclareLaunchArgument(
        "force_schedule_json",
        default_value="",
    )
    settle_arg = DeclareLaunchArgument("settle_after_sec", default_value="3.0")

    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"demo_motion": False},
            {"lock_base_motion": True},
            {"lock_torso_motion": True},
            {"lock_base_settle_sec": 0.8},
            {"use_cmd_vel": False},
            {"use_viewer": ParameterValue(LaunchConfiguration("use_viewer"), value_type=bool)},
            # Start arm at the same configuration as initial_joint_pose_json to
            # eliminate large initial convergence motions and Coriolis disturbances.
            {"initial_qpos_json": (
                '{"l_shoulder_pan_joint":0.4,'
                '"l_shoulder_lift_joint":-0.2,'
                '"l_upper_arm_roll_joint":0.0,'
                '"l_elbow_flex_joint":-0.8,'
                '"l_forearm_roll_joint":0.0,'
                '"l_wrist_flex_joint":-0.3,'
                '"l_wrist_roll_joint":0.0}'
            )},
        ],
    )

    state_est = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_state_estimator",
        name="pr2_state_estimator",
        output="screen",
    )

    wbc = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="screen",
        parameters=[
            {"nullspace_enable": False},
            {"cmd_timeout_sec": 0.15},
        ],
    )

    ee_pose_control = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_ee_pose_publisher",
        name="pr2_ee_pose_publisher_control",
        output="screen",
        parameters=[
            {"pose_topic": "ee_pose"},
            {"frame_id": "odom"},
        ],
    )

    # Admittance + IK: base latch uses speed gate (see pr2_arm_admittance_controller). After runs are stable,
    # tune in order: latch thresholds/durations -> max_joint_velocity_rad_s -> damping_linear/stiffness_linear
    # -> max_linear_velocity -> velocity_integration_gain with torque_kp/torque_kd and damping_lambda.
    arm_adm = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_admittance_controller",
        name="pr2_arm_admittance_controller",
        output="screen",
        parameters=[
            {"input_wrench_topic": "wbc/arm_external_wrench"},
            {"input_pose_topic": "ee_pose"},
            {"output_twist_topic": "arm_cartesian_velocity"},
            {"target_frame_id": "odom"},
            {"enable_wrench_frame_transform": True},
            {"strict_frame_consistency": True},
            {"suppress_output_on_tf_failure": True},
            {"freeze_orientation": True},
            {"base_pose_latch_delay_sec": 0.5},
            {"base_pose_latch_max_wait_sec": 8.0},
            {"ee_linear_speed_thresh": 0.02},
            {"ee_angular_speed_thresh": 0.15},
            {"latch_stable_duration_sec": 0.3},
            {"ee_speed_estimate_lpf_alpha": 0.25},
            {"base_pose_latched_topic": "base_pose_latched"},
            {"wrench_lpf_alpha": 0.15},
            {"damping_linear": [320.0, 320.0, 400.0]},
            {"stiffness_linear": [260.0, 260.0, 320.0]},
            {"max_linear_velocity": [0.12, 0.12, 0.10]},
            {"max_linear_displacement": [0.06, 0.06, 0.05]},
            {"damping_angular": [24.0, 24.0, 18.0]},
            {"stiffness_angular": [35.0, 35.0, 20.0]},
        ],
    )

    ik = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_left_arm_ik",
        name="pr2_left_arm_ik",
        output="screen",
        parameters=[
            {"joint_command_topic": "wbc/reference/joint_command"},
            {"control_mode": "velocity_tracking"},
            {"cartesian_velocity_topic": "arm_cartesian_velocity"},
            {"velocity_cmd_timeout_sec": 0.15},
            {"joint_state_topic": "joint_states"},
            {"odom_topic": "odom"},
            {"use_orientation": False},
            {"require_odom_sync": True},
            {"state_timeout_sec": 0.20},
            {"initial_joint_pose_json": (
                '{"l_shoulder_pan_joint":0.4,'
                '"l_shoulder_lift_joint":-0.2,'
                '"l_upper_arm_roll_joint":0.0,'
                '"l_elbow_flex_joint":-0.8,'
                '"l_forearm_roll_joint":0.0,'
                '"l_wrist_flex_joint":-0.3,'
                '"l_wrist_roll_joint":0.0}'
            )},
            {"damping_lambda": 0.15},
            {"max_joint_velocity_rad_s": 10.0},
            {"torque_kp": 120.0},
            {"torque_kd": 18.0},
            {"velocity_integration_gain": 1.0},
            {"velocity_sync_alpha": 0.3},
            {"max_torque": 300.0},
            {"tau_rate_limit": 600.0},
            {"tau_lpf_alpha": 0.35},
            {"joint_soft_limit_margin": 0.35},
        ],
    )

    validator = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_admittance_validator",
        name="pr2_arm_admittance_validator",
        output="screen",
        parameters=[
            {"wrench_topic": "wbc/arm_external_wrench"},
            {"frame_id": "odom"},
            {"expected_pose_frame_id": "odom"},
            {"control_target_frame_id": "odom"},
            {"ee_pose_topic": "ee_pose"},
            {"joint_command_topic": "joint_commands"},
            {"arm_only_mode": True},
            {"baseline_latch_mode": "at_force_start"},
            {"force_x": ParameterValue(LaunchConfiguration("force_x"), value_type=float)},
            {"duration_sec": ParameterValue(LaunchConfiguration("duration_sec"), value_type=float)},
            {"force_start_sec": ParameterValue(LaunchConfiguration("force_start_sec"), value_type=float)},
            {"force_schedule_json": ParameterValue(LaunchConfiguration("force_schedule_json"), value_type=str)},
            {"settle_after_sec": ParameterValue(LaunchConfiguration("settle_after_sec"), value_type=float)},
            {"max_peak_disp_z": 0.015},
        ],
    )

    log_arg = DeclareLaunchArgument("log_path", default_value="")

    logger = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_motion_logger",
        name="pr2_motion_logger",
        output="screen",
        parameters=[
            {"ee_pose_topic": "ee_pose"},
            {"ik_target_topic": "ik_target_pose"},
            {"cartesian_velocity_topic": "arm_cartesian_velocity"},
            {"base_pose_latched_topic": "base_pose_latched"},
            {"wrench_topic": "wbc/arm_external_wrench"},
            {"joint_state_topic": "joint_states"},
            {"odom_topic": "odom"},
            {"wbc_joint_ref_topic": "wbc/reference/joint_command"},
            {"wbc_cmd_vel_topic": "wbc/reference/cmd_vel"},
            {"state_joint_topic": "state/joint_states"},
            {"stale_joint_states_sec": 0.20},
            {"stale_cart_vel_sec": 0.15},
            {"stale_odom_sec": 0.20},
            {"stale_wbc_joint_ref_sec": 0.15},
            {"stale_wbc_cmd_vel_sec": 0.15},
            {"stale_state_joint_sec": 0.25},
            {"log_rate_hz": 50.0},
            {"output_path": ParameterValue(LaunchConfiguration("log_path"), value_type=str)},
        ],
    )

    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=validator, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            model_arg,
            viewer_arg,
            force_arg,
            duration_arg,
            force_start_arg,
            force_schedule_arg,
            settle_arg,
            log_arg,
            sim,
            state_est,
            wbc,
            ee_pose_control,
            arm_adm,
            ik,
            validator,
            logger,
            shutdown_on_done,
        ]
    )