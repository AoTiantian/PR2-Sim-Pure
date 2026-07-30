"""PID virtual human co-transport demo — force at board hand end via xfrc_applied.

The PID computes force to track a desired hand trajectory.  The force is:
1) Applied to the board body in MuJoCo at the hand end (xfrc_applied)
2) Published as EE-equivalent wrench for QP admittance compliance

Usage:
    ros2 launch pr2_virtual_human pr2_virtual_human_demo.launch.py \
        trajectory_mode:=linear_x trajectory_speed:=0.05 use_viewer:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    # ---- trajectory / PID args -------------------------------------------
    traj_mode_arg = DeclareLaunchArgument("trajectory_mode", default_value="linear_x")
    traj_speed_arg = DeclareLaunchArgument("trajectory_speed", default_value="0.05")
    traj_start_delay_arg = DeclareLaunchArgument("trajectory_start_delay", default_value="2.0")
    max_tracking_arg = DeclareLaunchArgument("max_tracking_duration", default_value="12.0")
    traj_hold_arg = DeclareLaunchArgument("trajectory_hold_duration", default_value="2.0")
    board_half_arg = DeclareLaunchArgument("board_half_length", default_value="0.5")

    # ---- logging ----------------------------------------------------------
    log_path_arg = DeclareLaunchArgument("log_path", default_value="")

    # -----------------------------------------------------------------------
    # sim
    # -----------------------------------------------------------------------
    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="both",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"use_viewer": ParameterValue(LaunchConfiguration("use_viewer"), value_type=bool)},
            {"demo_motion": False},
            {"use_cmd_vel": True},
            {"lock_base_motion": False},
            {"lock_torso_motion": True},
            {"cmd_vel_linear_gain": 17.0},
            {"initial_qpos_json": ParameterValue(LaunchConfiguration("initial_qpos_json"), value_type=str)},
            {"ctc_enable": True},
            {"ctc_kp": 30.0},
            {"ctc_kd": 160.0},
            {"cmd_vel_steer_gate_k_min": 0.12},
            {"gripper_open_time_sec": 0.0},
            {"joint_motion_log_rate_hz": 1.0},
            {"joint_motion_log_regex": "l_gripper|board"},
            # Apply PID force directly to board body via xfrc_applied
            {"hand_force_enable": False},
        ],
    )

    # -----------------------------------------------------------------------
    # state estimator
    # -----------------------------------------------------------------------
    state_est = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_state_estimator",
        name="pr2_state_estimator",
        output="both",
    )

    # -----------------------------------------------------------------------
    # EE pose publisher
    # -----------------------------------------------------------------------
    ee_pose = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_ee_pose_publisher",
        name="pr2_ee_pose_publisher",
        output="both",
        parameters=[
            {"ee_pose_topic": "ee_pose"},
            {"odom_topic": "odom"},
            {"joint_state_topic": "joint_states"},
            {"frame_id": "odom"},
        ],
    )

    # -----------------------------------------------------------------------
    # QP whole-body admittance controller
    # -----------------------------------------------------------------------
    qp = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_qp_whole_body_admittance",
        name="pr2_qp_whole_body_admittance",
        output="both",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"input_wrench_topic": "wbc/whole_body_wrench"},
            {"ee_pose_topic": "ee_pose"},
            {"odom_topic": "odom"},
            {"state_joint_topic": "state/joint_states"},
            {"output_cmd_vel_topic": "wbc/reference/cmd_vel"},
            {"output_joint_command_topic": "wbc/reference/joint_command"},
            {"force_despring_thresh": [20.0, 20.0, 130.0]},
            {"stiffness_linear": [0.0, 0.0, 0.0]},
            {"hold_stiffness_linear": [0.0, 0.0, 0.0]},
            {"hold_until_wrench_active": False},
            {"stiffness_angular": [0.0, 0.0, 0.0]},
            {"freeze_orientation": False},
            {"W_ee": [1.0, 1.0, 4.0, 1.0, 1.0, 1.0]},
            {"W_reg": [2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3]},
            {"posture_hold_enable": True},
            {"posture_hold_kp": 1.5},
            {"W_posture": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]},
            {"cmd_vel_world_scale": [1.03, 1.12, 1.0]},
            {"mass_linear": [5.0, 5.0, 5.0]},
            {"mass_angular": [0.5, 0.5, 0.5]},
        ],
    )

    # -----------------------------------------------------------------------
    # WBC coordinator
    # -----------------------------------------------------------------------
    wbc = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="both",
        parameters=[{"nullspace_enable": False}],
    )

    # -----------------------------------------------------------------------
    # PID virtual human controller — force at hand via xfrc_applied
    # -----------------------------------------------------------------------
    virt_human = Node(
        package="pr2_virtual_human",
        executable="pr2_virtual_human_controller",
        name="pr2_virtual_human_controller",
        output="both",
        parameters=[
            {"ee_pose_topic": "ee_pose"},
            {"hand_force_topic": "virtual_human/hand_force"},
            {"wrench_topic": "wbc/whole_body_wrench"},
            {"board_grasped_topic": "mujoco/board_grasped"},
            {"hand_force_frame_id": "odom"},
            {"wrench_frame_id": "base_link"},
            {"rate_hz": 50.0},
            {"board_half_length": ParameterValue(LaunchConfiguration("board_half_length"), value_type=float)},
            {"board_offset_in_ee": [0.18, 0.0, 0.0]},
            {"trajectory_mode": ParameterValue(LaunchConfiguration("trajectory_mode"), value_type=str)},
            {"trajectory_speed": ParameterValue(LaunchConfiguration("trajectory_speed"), value_type=float)},
            {"trajectory_start_delay": ParameterValue(LaunchConfiguration("trajectory_start_delay"), value_type=float)},
            {"max_tracking_duration": ParameterValue(LaunchConfiguration("max_tracking_duration"), value_type=float)},
            {"trajectory_hold_duration": ParameterValue(LaunchConfiguration("trajectory_hold_duration"), value_type=float)},
            {"pid_enable": True},
            {"pid_kp": [150.0, 150.0, 200.0]},
            {"pid_ki": [8.0, 8.0, 12.0]},
            {"pid_kd": [20.0, 20.0, 30.0]},
            {"pid_max_integral": [25.0, 25.0, 40.0]},
            {"pid_max_force": [50.0, 50.0, 70.0]},
            {"pid_deriv_lpf_alpha": 0.8},
            {"force_deadzone": 0.3},
            {"log_path": ParameterValue(LaunchConfiguration("log_path"), value_type=str)},
        ],
    )

    # -----------------------------------------------------------------------
    # motion logger
    # -----------------------------------------------------------------------
    logger = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_motion_logger",
        name="pr2_motion_logger",
        output="both",
        parameters=[
            {"ee_pose_topic": "ee_pose"},
            {"ik_target_topic": "ik_target_pose"},
            {"cartesian_velocity_topic": "arm_cartesian_velocity"},
            {"base_pose_latched_topic": "base_pose_latched"},
            {"wrench_topic": "wbc/whole_body_wrench"},
            {"admittance_wrench_topic": "arm_adm/debug_wrench"},
            {"admittance_dx_topic": "arm_adm/debug_dx"},
            {"mujoco_joint_bias_topic": "mujoco/joint_bias"},
            {"mujoco_joint_actuator_topic": "mujoco/joint_actuator"},
            {"joint_state_topic": "joint_states"},
            {"odom_topic": "odom"},
            {"wbc_joint_ref_topic": "wbc/reference/joint_command"},
            {"wbc_cmd_vel_topic": "wbc/reference/cmd_vel"},
            {"state_joint_topic": "state/joint_states"},
            {"qp_debug_topic": "qp/debug_residual"},
            {"log_rate_hz": 50.0},
            {"output_path": ParameterValue(LaunchConfiguration("log_path"), value_type=str)},
            {"validation_start_topic": ""},
            {"plot_mass": [5.0, 5.0, 5.0]},
            {"plot_damping": [320.0, 320.0, 400.0]},
            {"plot_deadzone": [0.8, 0.8, 0.8]},
        ],
    )

    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=virt_human, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([
        model_arg, viewer_arg, initial_qpos_arg,
        traj_mode_arg, traj_speed_arg, traj_start_delay_arg,
        max_tracking_arg, traj_hold_arg, board_half_arg,
        log_path_arg,
        sim, state_est, ee_pose, qp, wbc, virt_human, logger,
        shutdown_on_done,
    ])
