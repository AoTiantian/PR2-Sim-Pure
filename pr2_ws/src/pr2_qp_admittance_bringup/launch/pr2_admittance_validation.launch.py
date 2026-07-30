"""Pure admittance validation launch — fixed-force step response.

This launch runs the PR2 whole-body admittance stack with a constant-force
validator instead of the PID virtual human controller.  The purpose is to
measure the *open-loop admittance accuracy*: when we command a known force,
how far and how smoothly does the EE move?

Compare the resulting motion log against the virtual human PID controller
to separate "admittance tracking error" from "PID tracking error".

Usage:
    ros2 launch pr2_qp_admittance_bringup pr2_admittance_validation.launch.py \
        force_x:=30.0 force_y:=30.0 force_start_sec:=3.0 duration_sec:=8.0 \
        log_path:=/workspace/logs/pr2_admittance_step.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # ---- shared args --------------------------------------------------------
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml",
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

    # ---- force step parameters ----------------------------------------------
    force_x_arg = DeclareLaunchArgument("force_x", default_value="30.0")
    force_y_arg = DeclareLaunchArgument("force_y", default_value="0.0")
    force_z_arg = DeclareLaunchArgument("force_z", default_value="0.0")
    force_start_sec_arg = DeclareLaunchArgument("force_start_sec", default_value="3.0")
    duration_sec_arg = DeclareLaunchArgument("duration_sec", default_value="6.0")
    total_duration_sec_arg = DeclareLaunchArgument("total_duration_sec", default_value="12.0")
    settle_after_sec_arg = DeclareLaunchArgument("settle_after_sec", default_value="3.0")

    # ---- logging ------------------------------------------------------------
    log_path_arg = DeclareLaunchArgument("log_path", default_value="")

    # -------------------------------------------------------------------------
    # sim
    # -------------------------------------------------------------------------
    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="both",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {
                "use_viewer": ParameterValue(
                    LaunchConfiguration("use_viewer"), value_type=bool
                )
            },
            {"demo_motion": False},
            {"use_cmd_vel": True},
            {"lock_base_motion": False},
            {"lock_torso_motion": True},
            {"cmd_vel_linear_gain": 17.0},
            {
                "initial_qpos_json": ParameterValue(
                    LaunchConfiguration("initial_qpos_json"), value_type=str
                )
            },
            {"ctc_enable": True},
            {"ctc_kp": 30.0},
            {"ctc_kd": 160.0},
            {"cmd_vel_steer_gate_k_min": 0.12},
            {"gripper_open_time_sec": 0.0},
            {"joint_motion_log_rate_hz": 1.0},
            {"joint_motion_log_regex": "l_gripper|board"},
        ],
    )

    # -------------------------------------------------------------------------
    # state estimator
    # -------------------------------------------------------------------------
    state_est = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_state_estimator",
        name="pr2_state_estimator",
        output="both",
    )

    # -------------------------------------------------------------------------
    # EE pose publisher
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # QP whole-body admittance controller
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # WBC coordinator
    # -------------------------------------------------------------------------
    wbc = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="both",
        parameters=[{"nullspace_enable": False}],
    )

    # -------------------------------------------------------------------------
    # fixed-force validator (publishes constant wrench step)
    # -------------------------------------------------------------------------
    validator = Node(
        package="pr2_wrench_input",
        executable="pr2_arm_admittance_validator",
        name="pr2_arm_admittance_validator",
        output="both",
        parameters=[
            {"wrench_topic": "wbc/whole_body_wrench"},
            # Provide force in base_link for intuitive "push +x/+y of robot".
            # QP node will rotate it into odom/world using odom yaw.
            {"frame_id": "base_link"},
            {"expected_pose_frame_id": "odom"},
            {"control_target_frame_id": "odom"},
            {"ee_pose_topic": "ee_pose"},
            {"joint_states_topic": "joint_states"},
            {"joint_command_topic": "joint_commands"},
            {"arm_only_mode": False},
            {"baseline_latch_mode": "at_force_start"},
            {"force_x": ParameterValue(LaunchConfiguration("force_x"), value_type=float)},
            {"force_y": ParameterValue(LaunchConfiguration("force_y"), value_type=float)},
            {"force_z": ParameterValue(LaunchConfiguration("force_z"), value_type=float)},
            {"force_start_sec": ParameterValue(LaunchConfiguration("force_start_sec"), value_type=float)},
            {"duration_sec": ParameterValue(LaunchConfiguration("duration_sec"), value_type=float)},
            {"total_duration_sec": ParameterValue(LaunchConfiguration("total_duration_sec"), value_type=float)},
            {"settle_after_sec": ParameterValue(LaunchConfiguration("settle_after_sec"), value_type=float)},
            {"pub_rate_hz": 50.0},
            {"start_time_on_first_joint_state": True},
            # Disable pass/fail thresholds — we just want data
            {"min_peak_disp_xy": 0.0},
            {"min_peak_disp_xyz": 0.0},
            {"max_peak_disp_xy": 99.0},
            {"max_peak_disp_xyz": 99.0},
            {"max_peak_disp_z": 99.0},
            {"max_allowed_left_effort": 999.0},
            {"max_settle_time_sec": 99.0},
            {"max_rms_disp_after_settle": 99.0},
            {"max_effort_rms_after_settle": 999.0},
        ],
    )

    # -------------------------------------------------------------------------
    # motion logger
    # -------------------------------------------------------------------------
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

    # Auto-shutdown when validator exits (force sequence complete)
    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=validator, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(
        [
            model_arg,
            viewer_arg,
            initial_qpos_arg,
            force_x_arg,
            force_y_arg,
            force_z_arg,
            force_start_sec_arg,
            duration_sec_arg,
            total_duration_sec_arg,
            settle_after_sec_arg,
            log_path_arg,
            sim,
            state_est,
            ee_pose,
            qp,
            wbc,
            validator,
            logger,
            shutdown_on_done,
        ]
    )
