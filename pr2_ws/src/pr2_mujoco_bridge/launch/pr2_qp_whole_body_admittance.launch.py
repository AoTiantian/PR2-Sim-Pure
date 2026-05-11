"""Unified-QP whole-body admittance validation launch.

This launch runs:
- MuJoCo sim (base unlocked, cmd_vel enabled)
- state estimator (filtered state)
- ee_pose publisher
- QP whole-body admittance controller (publishes wbc/reference/cmd_vel + joint_command)
- WBC coordinator (for consistent sim output topics)
- validator + motion logger
"""

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
    viewer_arg = DeclareLaunchArgument("use_viewer", default_value="false")

    force_x_arg = DeclareLaunchArgument("force_x", default_value="0.0")
    force_y_arg = DeclareLaunchArgument("force_y", default_value="0.0")
    force_z_arg = DeclareLaunchArgument("force_z", default_value="0.0")
    torque_x_arg = DeclareLaunchArgument("torque_x", default_value="0.0")
    torque_y_arg = DeclareLaunchArgument("torque_y", default_value="0.0")
    torque_z_arg = DeclareLaunchArgument("torque_z", default_value="0.0")
    duration_arg = DeclareLaunchArgument("duration_sec", default_value="6.0")
    force_start_arg = DeclareLaunchArgument("force_start_sec", default_value="3.0")
    settle_arg = DeclareLaunchArgument("settle_after_sec", default_value="3.0")

    log_arg = DeclareLaunchArgument("log_path", default_value="")
    initial_qpos_arg = DeclareLaunchArgument(
        "initial_qpos_json",
        default_value='{"l_shoulder_pan_joint": 0.35, "l_shoulder_lift_joint": 0.95, "l_upper_arm_roll_joint": 0.0, "l_elbow_flex_joint": -1.35, "l_forearm_roll_joint": 0.0, "l_wrist_flex_joint": -0.55, "l_wrist_roll_joint": 0.0}',
    )

    sim = Node(
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
            {"demo_motion": False},
            {"lock_base_motion": False},
            {"use_cmd_vel": True},
            # cmd_vel -> wheel velocity mapping gain. If too small, actual base speed
            # (measured from odom) will lag far behind cmd_vel. Tuned from debug logs.
            {"cmd_vel_linear_gain": 14.0},
            {
                # Left arm initial posture (joint angle map). This is applied to MuJoCo qpos
                # before the first simulation step.
                #
                # Default: "斜向下举" (downward-tilted) instead of forward-level.
                #
                # You can override at launch-time by passing initial_qpos_json:='{"...": ...}'.
                "initial_qpos_json": ParameterValue(
                    LaunchConfiguration("initial_qpos_json"), value_type=str
                )
            },
            {"ctc_enable": True},
            {"ctc_kp": 0.0},
            {"ctc_kd": 80.0},
        ],
    )

    state_est = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_state_estimator",
        name="pr2_state_estimator",
        output="screen",
    )

    ee_pose = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_ee_pose_publisher",
        name="pr2_ee_pose_publisher",
        output="screen",
        parameters=[
            {"ee_pose_topic": "ee_pose"},
            {"odom_topic": "odom"},
            {"joint_state_topic": "joint_states"},
            {"frame_id": "odom"},
        ],
    )

    qp = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_qp_whole_body_admittance",
        name="pr2_qp_whole_body_admittance",
        output="screen",
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
            {"mass_linear": [5.0, 5.0, 5.0]},
            {"mass_angular": [0.5, 0.5, 0.5]},
        ],
    )

    wbc = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="screen",
        parameters=[{"nullspace_enable": False}],
    )

    validator = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_arm_admittance_validator",
        name="pr2_qp_whole_body_validator",
        output="screen",
        parameters=[
            {"wrench_topic": "wbc/whole_body_wrench"},
            # Provide force in base_link for intuitive "push +x/+y of robot".
            # QP node will rotate it into odom/world using odom yaw.
            {"frame_id": "base_link"},
            {"expected_pose_frame_id": "odom"},
            {"control_target_frame_id": "odom"},
            {"ee_pose_topic": "ee_pose"},
            {"joint_command_topic": "joint_commands"},
            {"arm_only_mode": False},
            {"baseline_latch_mode": "at_force_start"},
            {"force_x": ParameterValue(LaunchConfiguration("force_x"), value_type=float)},
            {"force_y": ParameterValue(LaunchConfiguration("force_y"), value_type=float)},
            {"force_z": ParameterValue(LaunchConfiguration("force_z"), value_type=float)},
            {"torque_x": ParameterValue(LaunchConfiguration("torque_x"), value_type=float)},
            {"torque_y": ParameterValue(LaunchConfiguration("torque_y"), value_type=float)},
            {"torque_z": ParameterValue(LaunchConfiguration("torque_z"), value_type=float)},
            {"duration_sec": ParameterValue(LaunchConfiguration("duration_sec"), value_type=float)},
            {"force_start_sec": ParameterValue(
                LaunchConfiguration("force_start_sec"), value_type=float
            )},
            {"settle_after_sec": ParameterValue(
                LaunchConfiguration("settle_after_sec"), value_type=float
            )},
            {"validation_start_topic": "validation/start"},
        ],
    )

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
            {"validation_start_topic": "validation/start"},
            {"plot_mass": [5.0, 5.0, 5.0]},
            {"plot_damping": [320.0, 320.0, 400.0]},
            {"plot_deadzone": [0.8, 0.8, 0.8]},
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
            force_x_arg,
            force_y_arg,
            force_z_arg,
            torque_x_arg,
            torque_y_arg,
            torque_z_arg,
            duration_arg,
            force_start_arg,
            settle_arg,
            log_arg,
            initial_qpos_arg,
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

