from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[3]


def _read_package_file(relative: str) -> str:
    return (PACKAGE_ROOT / relative).read_text(encoding="utf-8")


def test_whole_body_launch_routes_arm_and_base_through_wbc():
    launch_text = _read_package_file("launch/pr2_whole_body_force.launch.py")

    assert '"demo_motion": False' in launch_text
    assert '"joint_command_topic": "wbc/arm/joint_command"' in launch_text
    assert '"output_topic": "wbc/reference/cmd_vel"' in launch_text
    assert '"arm_joint_cmd_topic": "wbc/arm/joint_command"' in launch_text
    assert '"nullspace_enable": False' in launch_text


def test_amplified_whole_body_launch_routes_arm_and_base_through_wbc():
    launch_text = _read_package_file("launch/pr2_whole_body_force_amplified.launch.py")

    assert '"demo_motion": False' in launch_text
    assert '"joint_command_topic": "wbc/arm/joint_command"' in launch_text
    assert '"output_topic": "wbc/reference/cmd_vel"' in launch_text
    assert '"arm_joint_cmd_topic": "wbc/arm/joint_command"' in launch_text
    assert '"nullspace_enable": False' in launch_text


def test_wbc_coordinator_is_single_merge_point_for_whole_body_references():
    source = _read_package_file("pr2_mujoco_bridge/pr2_wbc_coordinator.py")

    for expected in (
        'declare_parameter("arm_joint_cmd_topic"',
        '"wbc/reference/cmd_vel"',
        '"wbc/reference/joint_command"',
        'create_publisher(Twist, self._out_twist',
        'create_publisher(JointState, self._out_joint',
        'get_parameter("output_cmd_vel")',
        'get_parameter("output_joint_command")',
        'def _is_fresh',
        'publish(Twist())',
        'merge[jn] = dict(d)',
    ):
        assert expected in source


def test_whole_body_architecture_has_documented_bypass_surfaces_for_future_cleanup():
    """Lock in known bypass surfaces so future work can remove or document them deliberately."""
    arm_source = _read_package_file("pr2_mujoco_bridge/pr2_arm_admittance.py")
    sim_source = _read_package_file("pr2_mujoco_bridge/pr2_sim_ros.py")
    plan_text = _read_package_file("docs/2026-04-27_whole_body_admittance_meeting_plan.md")

    # Standalone/default arm admittance remains a known direct path unless launch overrides it.
    assert 'declare_parameter("joint_command_topic", "joint_commands")' in arm_source
    # The bridge intentionally exposes direct sim-level inputs; whole-body launches should avoid bypassing WBC.
    assert '"actuator_command"' in sim_source
    assert '"cmd_vel"' in sim_source
    # The meeting plan requires this risk to stay visible during the follow-up work.
    assert "单一控制器" in plan_text or "single-controller" in plan_text
