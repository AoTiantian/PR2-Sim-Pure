from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_launch_disables_mixed_control_and_active_compensation() -> None:
    source = (PACKAGE / "launch" / "transport_comparison.launch.py").read_text()
    required = (
        '{"hand_force_cancel_moment": False}',
        '{"ctc_payload_auto_balance": False}',
        '{"ctc_payload_force_z": 0.0}',
        '{"ctc_vertical_hold_force_limit": 0.0}',
        '{"pose_tracking_enable": False}',
        '{"orientation_tracking_enable": False}',
        '{"freeze_orientation": False}',
        '{"fixed_target_mode": False}',
        '{"input_wrench_topic": "mujoco/left_wrist_wrench"}',
    )
    for contract in required:
        assert contract in source


def test_robot_is_pure_mass_damper_admittance() -> None:
    config = yaml.safe_load(
        (PACKAGE / "config" / "transport_comparison.yaml").read_text()
    )
    admittance = config["robot"]["admittance"]
    assert admittance["stiffness_linear"] == [0.0, 0.0, 0.0]
    assert admittance["stiffness_angular"] == [0.0, 0.0, 0.0]
