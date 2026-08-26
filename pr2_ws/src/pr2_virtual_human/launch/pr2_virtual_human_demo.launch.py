"""PID virtual-human/PR2 collaborative board transport demo.

The human force is applied only at the far board endpoint.  The PR2 admittance
controller reacts only to the physical MuJoCo wrist F/T measurement.

Usage:
    ros2 launch pr2_virtual_human pr2_virtual_human_demo.launch.py \
        use_viewer:=true

Fixed-pose wrench calibration:
    ros2 launch pr2_virtual_human pr2_virtual_human_demo.launch.py \
        fixed_target_mode:=true trajectory_position_enable:=false \
        trajectory_orientation_enable:=false

The board remains a dynamic free rigid body and the robot wrench is read from
the left wrist force/torque sensor; the mobile base is not artificially locked.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # Keep ROS node logs beside the experiment CSV/PNG/metrics outputs.
    ros_log_dir = SetEnvironmentVariable(
        "ROS_LOG_DIR", "/workspace/results/pr2_virtual_human/ros_logs"
    )
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspace/unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml",
    )
    viewer_arg = DeclareLaunchArgument("use_viewer", default_value="true")
    initial_qpos_arg = DeclareLaunchArgument(
        "initial_qpos_json",
        default_value=(
            '{"l_shoulder_pan_joint": 0.11735056, '
            '"l_shoulder_lift_joint": 0.69968519, '
            '"l_upper_arm_roll_joint": 0.00321579, '
            '"l_elbow_flex_joint": -1.38673805, '
            '"l_forearm_roll_joint": -1.78800816, '
            '"l_wrist_flex_joint": -1.31706743, '
            '"l_wrist_roll_joint": 2.28528285, '
            '"l_gripper_l_finger_joint": 0.52, '
            '"l_gripper_r_finger_joint": 0.52, '
            '"l_gripper_l_finger_tip_joint": 0.52, '
            '"l_gripper_r_finger_tip_joint": 0.52}'
        ),
    )

    # ---- trajectory / PID args -------------------------------------------
    traj_start_delay_arg = DeclareLaunchArgument("trajectory_start_delay", default_value="2.0")
    traj_position_enable_arg = DeclareLaunchArgument("trajectory_position_enable", default_value="true")
    traj_orientation_enable_arg = DeclareLaunchArgument("trajectory_orientation_enable", default_value="true")
    fixed_target_mode_arg = DeclareLaunchArgument(
        "fixed_target_mode",
        default_value="false",
        description="Hold the latched 6D pose while measuring the robot wrench",
    )
    traj_x_amp_arg = DeclareLaunchArgument("trajectory_x_amplitude", default_value="0.20")
    traj_y_amp_arg = DeclareLaunchArgument("trajectory_y_amplitude", default_value="0.12")
    traj_z_amp_arg = DeclareLaunchArgument("trajectory_z_amplitude", default_value="0.0")
    # Use a slow, large roll about the board's long axis so the rotation is
    # visible without demanding a metre-scale x/y displacement at the robot
    # endpoint.  Pitch remains modest because it couples directly into z.
    traj_roll_amp_arg = DeclareLaunchArgument("trajectory_roll_amplitude", default_value="0.80")
    traj_pitch_amp_arg = DeclareLaunchArgument("trajectory_pitch_amplitude", default_value="0.06")
    # Moderate yaw rotation keeps the board transport controller within its
    # force/velocity limits; larger angles require a slower trajectory.
    traj_yaw_amp_arg = DeclareLaunchArgument("trajectory_yaw_amplitude", default_value="0.20")
    traj_period_arg = DeclareLaunchArgument("trajectory_period", default_value="30.0")
    traj_ramp_arg = DeclareLaunchArgument("trajectory_ramp_duration", default_value="1.0")
    tracking_duration_arg = DeclareLaunchArgument("tracking_duration", default_value="12.0")
    traj_hold_arg = DeclareLaunchArgument("hold_duration", default_value="1.0")
    output_dir_arg = DeclareLaunchArgument(
        "output_dir", default_value="/workspace/results/pr2_virtual_human"
    )
    pose_tracking_enable_arg = DeclareLaunchArgument(
        "pose_tracking_enable",
        default_value="true",
        description=(
            "Enable robot end-effector pose assistance.  The human force is "
            "still applied and logged; use pose_tracking_enable:=false for "
            "the pure wrench-driven comparison."
        ),
    )
    # The rigid board grasp and free base need a stiffer, well-damped inner
    # joint loop.  The previous 200/80 setting left enough endpoint motion to
    # re-excite the vertical force split; 1000/200 is the stable tested default
    # (both values remain overrideable from the command line).
    ctc_kp_arg = DeclareLaunchArgument("ctc_kp", default_value="1000.0")
    ctc_kd_arg = DeclareLaunchArgument("ctc_kd", default_value="200.0")
    ctc_payload_arg = DeclareLaunchArgument(
        "ctc_payload_force_z",
        default_value="0.0",
        description="Optional robot vertical-force override; zero enables force-balance residual (N)",
    )
    ctc_payload_auto_arg = DeclareLaunchArgument(
        "ctc_payload_auto_balance",
        default_value="true",
        description="Compute robot vertical force as board-load residual after human force",
    )
    ctc_payload_total_arg = DeclareLaunchArgument(
        "ctc_payload_total_force_z",
        default_value="0.0",
        description="Optional total support override; zero derives board mass×|gravity| (N)",
    )

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
            {"viewer_lookat": [-1.55, 1.20, 0.80]},
            {"viewer_distance": 4.5},
            {"viewer_azimuth": 135.0},
            {"viewer_elevation": -20.0},
            {"demo_motion": False},
            {"use_cmd_vel": True},
            # The mobile base is part of the collaborative dynamics.  It must
            # remain free in both trajectory and fixed-target experiments.
            {"lock_base_motion": False},
            # Free base with finite wheel/ground rolling resistance; this is
            # passive damping, not a kinematic base lock.
            {"base_passive_damping": 12.0},
            # The physical contact must be allowed to react to the endpoint
            # forces in both fixed-target and trajectory modes.
            {"lock_arm_motion": False},
            {"lock_torso_motion": True},
            {"cmd_vel_linear_gain": 17.0},
            {"initial_qpos_json": ParameterValue(LaunchConfiguration("initial_qpos_json"), value_type=str)},
            {"ctc_enable": True},
            {"ctc_kp": ParameterValue(LaunchConfiguration("ctc_kp"), value_type=float)},
            {"ctc_kd": ParameterValue(LaunchConfiguration("ctc_kd"), value_type=float)},
            {"ctc_payload_force_z": ParameterValue(LaunchConfiguration("ctc_payload_force_z"), value_type=float)},
            {"ctc_payload_auto_balance": ParameterValue(LaunchConfiguration("ctc_payload_auto_balance"), value_type=bool)},
            {"ctc_payload_total_force_z": ParameterValue(LaunchConfiguration("ctc_payload_total_force_z"), value_type=float)},
            # Let the arm/weld settle before latching the dynamic z-hold
            # reference; the trajectory controller also starts at 2 s.
            {"ctc_balance_startup_duration_sec": 2.0},
            # Endpoint-height damping is zero at the static target, and only
            # restores the robot grasp point after a z disturbance. It does
            # not prescribe a 50% share or cancel the vertical support moment.
            {"ctc_vertical_hold_kp": 80.0},
            {"ctc_vertical_hold_kd": 30.0},
            {"ctc_vertical_hold_force_limit": 12.0},
            {"cmd_vel_steer_gate_k_min": 0.12},
            {"gripper_open_time_sec": 0.0},
            {"left_gripper_hold_position": 0.52},
            {"joint_motion_log_rate_hz": 1.0},
            {"joint_motion_log_regex": "l_gripper|board"},
            # Apply the human impedance wrench directly to the board body via
            # xfrc_applied.  The robot side supplies only the force residual
            # required by the board force balance; no endpoint torque is
            # injected to hide a load-sharing mismatch.
            {"hand_force_enable": True},
            {"hand_force_offset": [1.0, 0.0, 0.0]},
            {"hand_force_cancel_moment": False},
            {"robot_support_force_topic": "mujoco/robot_support_force"},
            {"left_wrist_wrench_topic": "mujoco/left_wrist_wrench"},
            # The physical two-end support is already active during latch, so
            # do not tare it away as if it were an unloaded wrist.
            {"left_wrist_tare_duration_sec": 0.0},
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
            {"input_wrench_topic": "mujoco/left_wrist_wrench"},
            {"ee_pose_topic": "ee_pose"},
            {"odom_topic": "odom"},
            {"state_joint_topic": "state/joint_states"},
            {"output_cmd_vel_topic": "wbc/reference/cmd_vel"},
            {"output_joint_command_topic": "wbc/reference/joint_command"},
            {"force_despring_thresh": [60.0, 60.0, 130.0]},
            # The 10 s horizontal trajectory reaches about 0.13 m/s in X and
            # 0.15 m/s in Y.  Z is a constant-height closed-loop task.
            {"damping_linear": [30.0, 30.0, 80.0]},
            {"hold_damping_linear": [50.0, 50.0, 100.0]},
            # The human PID and robot admittance form one coupled loop.  Keep
            # the wrench and base command bandwidth below the observed 4 Hz
            # oscillation of the previous trajectory run.
            {"wrench_lpf_alpha": 0.06},
            {"cmd_vel_lpf_alpha": 0.10},
            {"max_linear_velocity": [0.25, 0.25, 0.25]},
            {"stiffness_linear": [20.0, 20.0, 40.0]},
            {"hold_stiffness_linear": [80.0, 80.0, 160.0]},
            {"hold_until_wrench_active": False},
            {"fixed_target_mode": ParameterValue(LaunchConfiguration("fixed_target_mode"), value_type=bool)},
            {"stiffness_angular": [8.0, 8.0, 6.0]},
            # The board is a free rigid body now; orientation is controlled by
            # measured wrench rather than removed from the model.
            {"freeze_orientation": False},
            {"desired_orientation_topic": "virtual_human/desired_hand_pose"},
            {"orientation_tracking_enable": True},
            {"orientation_tracking_kp": [2.0, 2.0, 3.0]},
            # Pose assistance is the default stabilized collaboration mode;
            # disable it explicitly to reproduce the pure wrench experiment.
            {"pose_tracking_enable": ParameterValue(LaunchConfiguration("pose_tracking_enable"), value_type=bool)},
            # The mobile base is part of the collaborative dynamics.  Do not
            # clamp its QP velocity to zero in the trajectory demo.
            {"pose_tracking_freeze_base": False},
            # Keep the robot z loop deliberately soft: the human PID remains
            # the primary height controller, while a small robot z gain keeps
            # the 6-D orientation target geometrically consistent.
            {"pose_tracking_kp": [4.0, 4.0, 3.0]},
            {"human_hand_offset": [1.0, 0.0, 0.0]},
            {"board_to_ee_position": [-1.0006433, 0.0114053, -0.00000213]},
            {"board_to_ee_quaternion": [0.99410946, 0.00002539, -0.00013115, -0.10838066]},
            {"damping_angular": [8.0, 8.0, 6.0]},
            {"stiffness_angular": [8.0, 8.0, 6.0]},
            {"max_angular_velocity": [0.25, 0.25, 0.25]},
            {"W_ee": [1.0, 1.0, 4.0, 3.0, 3.0, 3.0]},
            {"W_reg": [2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3]},
            # In fixed-target mode the Cartesian task is already a complete
            # hold.  A separate null-space posture objective can move the arm
            # while the weld is transmitting the board load, so leave it off
            # for this coupled dynamics experiment.
            {"posture_hold_enable": False},
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
    # Endpoint-impedance virtual human — force at hand via xfrc_applied
    # -----------------------------------------------------------------------
    virt_human = Node(
        package="pr2_virtual_human",
        executable="pr2_virtual_human_controller",
        name="pr2_virtual_human_controller",
        output="both",
        parameters=[
            {"hand_pose_topic": "mujoco/human_hand_pose"},
            {"hand_force_topic": "virtual_human/hand_force"},
            {"hand_force_offset": [1.0, 0.0, 0.0]},
            # The endpoint r×F moment is preserved.  Equal vertical forces at
            # the two ends cancel their moments through the physical contacts.
            {"robot_support_force_topic": "mujoco/robot_support_force"},
            {"robot_wrench_topic": "mujoco/left_wrist_wrench"},
            {"board_grasped_topic": "mujoco/board_grasped"},
            {"frame_id": "odom"},
            {"rate_hz": 50.0},
            # The scene represents an already completed rigid robot grasp.
            {"require_board_grasped": False},
            {"fixed_target_mode": ParameterValue(LaunchConfiguration("fixed_target_mode"), value_type=bool)},
            {"trajectory_start_delay": ParameterValue(LaunchConfiguration("trajectory_start_delay"), value_type=float)},
            {"trajectory_position_enable": ParameterValue(LaunchConfiguration("trajectory_position_enable"), value_type=bool)},
            {"trajectory_orientation_enable": ParameterValue(LaunchConfiguration("trajectory_orientation_enable"), value_type=bool)},
            {"trajectory_x_amplitude": ParameterValue(LaunchConfiguration("trajectory_x_amplitude"), value_type=float)},
            {"trajectory_y_amplitude": ParameterValue(LaunchConfiguration("trajectory_y_amplitude"), value_type=float)},
            {"trajectory_z_amplitude": ParameterValue(LaunchConfiguration("trajectory_z_amplitude"), value_type=float)},
            {"trajectory_roll_amplitude": ParameterValue(LaunchConfiguration("trajectory_roll_amplitude"), value_type=float)},
            {"trajectory_pitch_amplitude": ParameterValue(LaunchConfiguration("trajectory_pitch_amplitude"), value_type=float)},
            {"trajectory_yaw_amplitude": ParameterValue(LaunchConfiguration("trajectory_yaw_amplitude"), value_type=float)},
            {"trajectory_period": ParameterValue(LaunchConfiguration("trajectory_period"), value_type=float)},
            {"trajectory_ramp_duration": ParameterValue(LaunchConfiguration("trajectory_ramp_duration"), value_type=float)},
            {"tracking_duration": ParameterValue(LaunchConfiguration("tracking_duration"), value_type=float)},
            {"hold_duration": ParameterValue(LaunchConfiguration("hold_duration"), value_type=float)},
            {"output_dir": LaunchConfiguration("output_dir")},
            # Human endpoint impedance.  These forces depend on endpoint
            # tracking error and velocity, not on the measured robot wrench.
            {"human_impedance_kp": [20.0, 20.0, 40.0]},
            {"human_impedance_ki": [0.0, 0.0, 2.0]},
            {"human_impedance_kd": [8.0, 8.0, 8.0]},
            # Compute the human/robot vertical support split from the two
            # endpoint offsets and board dynamics.  This yields equal forces
            # only for the symmetric default geometry; no 50% constant is
            # prescribed.
            {"automatic_vertical_balance_enable": True},
            # Runtime geometry of the left tool origin relative to the board
            # COM (the weld synchronizer measures the same offset at startup).
            {"robot_contact_offset": [-1.0006433, 0.0114053, -0.0000021]},
            {"human_impedance_integral_limit": [0.10, 0.10, 0.20]},
            {"human_force_limit": [8.0, 8.0, 45.0]},
            {"human_force_slew_rate": [80.0, 80.0, 300.0]},
            {"human_force_response_time": 0.05},
            {"human_allow_vertical_pull": False},
            {"robot_wrench_filter_alpha": 0.10},
            {"robot_wrench_is_board_on_robot": True},
            {"velocity_lpf_alpha": 0.15},
            # The robot-side point contact does not transmit orientation
            # torque.  A slightly stronger, better-damped human attitude
            # loop keeps the board near level before the two vertical forces
            # can create a large tilted-body moment.
            {"orientation_kp": [0.50, 3.00, 3.00]},
            {"orientation_ki": [0.01, 0.05, 0.05]},
            {"orientation_kd": [0.10, 1.20, 1.20]},
            # The endpoint-force moment is cancelled explicitly above; keep
            # enough wrench range for that couple while the residual attitude
            # controller remains low gain.
            {"pid_max_torque": [0.80, 5.00, 5.00]},
        ],
    )

    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=virt_human, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([
        ros_log_dir,
        model_arg, viewer_arg, initial_qpos_arg,
        traj_start_delay_arg, traj_position_enable_arg, traj_orientation_enable_arg,
        fixed_target_mode_arg,
        traj_x_amp_arg, traj_y_amp_arg, traj_z_amp_arg,
        traj_roll_amp_arg, traj_pitch_amp_arg, traj_yaw_amp_arg,
        traj_period_arg, traj_ramp_arg,
        tracking_duration_arg, traj_hold_arg, output_dir_arg,

        pose_tracking_enable_arg,
        ctc_kp_arg, ctc_kd_arg, ctc_payload_arg,
        ctc_payload_auto_arg, ctc_payload_total_arg,
        sim, state_est, ee_pose, qp, wbc, virt_human,
        shutdown_on_done,
    ])
