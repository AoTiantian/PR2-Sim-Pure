"""Unified A/B launch for human-only and pure-wrench human-robot transport."""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build(context):
    condition = LaunchConfiguration("condition").perform(context)
    robot_mode = LaunchConfiguration("robot_mode").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    use_viewer = LaunchConfiguration("use_viewer")
    experiment_id = LaunchConfiguration("experiment_id")
    output_root = LaunchConfiguration("output_root")
    human_only_model_override = LaunchConfiguration("human_only_model_path").perform(context)
    human_robot_model_override = LaunchConfiguration("human_robot_model_path").perform(context)
    if condition not in ("human_only", "human_robot"):
        raise RuntimeError("condition must be human_only or human_robot")
    if robot_mode != "admittance":
        raise RuntimeError("only robot_mode:=admittance is valid for the formal comparison")
    with Path(config_file).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    experiment = config["experiment"]
    robot = config["robot"]
    admittance = robot["admittance"]

    recorder = Node(
        package="pr2_virtual_human",
        executable="comparison_recorder",
        name="transport_comparison_recorder",
        output="both",
        parameters=[{
            "config_file": config_file,
            "condition": condition,
            "experiment_id": experiment_id,
            "output_root": output_root,
        }],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=recorder, on_exit=[EmitEvent(event=Shutdown())])
    )
    if condition == "human_only":
        adapter = Node(
            package="pr2_virtual_human",
            executable="human_only_adapter",
            name="transport_human_only_adapter",
            output="both",
            parameters=[
                {"config_file": config_file},
                {"model_path": human_only_model_override or str(config["models"]["human_only"])},
                {"use_viewer": ParameterValue(use_viewer, value_type=bool)},
            ],
        )
        return [adapter, recorder, shutdown]

    model_path = human_robot_model_override or str(config["models"]["human_robot"])
    sim = Node(
        package="pr2_mujoco_bridge",
        executable="pr2_mujoco_sim",
        name="pr2_mujoco_sim",
        output="both",
        parameters=[
            {"model_path": model_path},
            {"use_viewer": ParameterValue(use_viewer, value_type=bool)},
            {"demo_motion": False},
            {"use_cmd_vel": True},
            {"lock_base_motion": False},
            {"base_passive_damping": 0.0},
            {"lock_arm_motion": False},
            {"lock_torso_motion": True},
            {"initial_qpos_json": str(robot["initial_qpos_json"])},
            {"ctc_enable": True},
            {"ctc_kp": float(robot["ctc_kp"])},
            {"ctc_kd": float(robot["ctc_kd"])},
            {"ctc_payload_auto_balance": False},
            {"ctc_payload_force_z": 0.0},
            {"ctc_vertical_hold_force_limit": 0.0},
            {"hand_force_enable": True},
            {"hand_force_offset": list(experiment["human_hand_offset"])},
            {"hand_force_cancel_moment": False},
            {"left_wrist_wrench_sign": -1.0},
            # Sensor-zeroing only: it changes the admittance measurement
            # reference, never the MuJoCo force, r x F, or weld reaction.
            {"left_wrist_tare_duration_sec": float(experiment["settle_sec"])},
            {"gripper_open_time_sec": 0.0},
            {"left_gripper_hold_position": 0.52},
        ],
    )
    state_estimator = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_state_estimator",
        name="pr2_state_estimator",
        output="both",
    )
    ee_pose = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_ee_pose_publisher",
        name="pr2_ee_pose_publisher",
        output="both",
        parameters=[{"ee_pose_topic": "ee_pose", "frame_id": "odom"}],
    )
    qp = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_qp_whole_body_admittance",
        name="pr2_qp_whole_body_admittance",
        output="both",
        parameters=[
            {"model_path": model_path},
            {"input_wrench_topic": "mujoco/left_wrist_wrench"},
            {"ee_pose_topic": "ee_pose"},
            {"odom_topic": "odom"},
            {"state_joint_topic": "state/joint_states"},
            {"output_cmd_vel_topic": "wbc/reference/cmd_vel"},
            {"output_joint_command_topic": "wbc/reference/joint_command"},
            {"pose_tracking_enable": False},
            {"orientation_tracking_enable": False},
            {"freeze_orientation": False},
            {"fixed_target_mode": False},
            {"posture_hold_enable": False},
            {"hold_until_wrench_active": False},
            {"mass_linear": list(admittance["mass_linear"])},
            {"damping_linear": list(admittance["damping_linear"])},
            {"stiffness_linear": list(admittance["stiffness_linear"])},
            {"hold_damping_linear": list(admittance["damping_linear"])},
            {"hold_stiffness_linear": list(admittance["stiffness_linear"])},
            {"mass_angular": list(admittance["mass_angular"])},
            {"damping_angular": list(admittance["damping_angular"])},
            {"stiffness_angular": list(admittance["stiffness_angular"])},
            {"max_linear_velocity": list(admittance["max_linear_velocity"])},
            {"max_angular_velocity": list(admittance["max_angular_velocity"])},
            {"wrench_lpf_alpha": float(admittance["wrench_lpf_alpha"])},
            {"cmd_vel_lpf_alpha": float(admittance["cmd_vel_lpf_alpha"])},
            {"force_deadzone": [0.2, 0.2, 0.2]},
            {"torque_deadzone": [0.02, 0.02, 0.02]},
            {"force_despring_thresh": [1.0e6, 1.0e6, 1.0e6]},
        ],
    )
    coordinator = Node(
        package="pr2_wbc_admittance_control",
        executable="pr2_wbc_coordinator",
        name="pr2_wbc_coordinator",
        output="both",
        parameters=[{"nullspace_enable": False}],
    )
    controller = Node(
        package="pr2_virtual_human",
        executable="human_robot_controller",
        name="transport_human_robot_controller",
        output="both",
        parameters=[{"config_file": config_file}],
    )
    guard = Node(
        package="pr2_virtual_human",
        executable="comparison_contract_guard",
        name="transport_comparison_contract_guard",
        output="both",
        parameters=[{"expected_tare_duration_sec": float(experiment["settle_sec"])}],
    )
    guard_shutdown = RegisterEventHandler(
        OnProcessExit(target_action=guard, on_exit=[EmitEvent(event=Shutdown())])
    )
    return [sim, state_estimator, ee_pose, qp, coordinator, controller, guard, recorder, shutdown, guard_shutdown]


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory("pr2_virtual_human"))
        / "config" / "transport_comparison.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("condition", default_value="human_only"),
        DeclareLaunchArgument("robot_mode", default_value="admittance"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("experiment_id", default_value="default"),
        DeclareLaunchArgument("output_root", default_value=""),
        DeclareLaunchArgument("human_only_model_path", default_value=""),
        DeclareLaunchArgument("human_robot_model_path", default_value=""),
        OpaqueFunction(function=_build),
    ])
