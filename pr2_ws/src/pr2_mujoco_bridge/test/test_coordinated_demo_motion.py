import math

from pr2_mujoco_bridge.pr2_sim_ros import coordinated_demo_profile


def test_coordinated_profile_approach_raises_arm_before_grasp():
    early = coordinated_demo_profile(1.0)
    prep = coordinated_demo_profile(5.0)
    grasp_ready = coordinated_demo_profile(7.5)

    assert early.base_forward_speed > 0.0
    assert prep.base_forward_speed > 0.0
    assert grasp_ready.base_forward_speed > 0.0

    assert early.arm_lift_fraction < prep.arm_lift_fraction < grasp_ready.arm_lift_fraction
    assert grasp_ready.arm_lift_fraction > 0.9

    assert prep.shoulder_lift_torque < early.shoulder_lift_torque
    assert prep.elbow_flex_torque < 0.0
    assert grasp_ready.gripper_command < prep.gripper_command


def test_coordinated_profile_retreats_after_grasp_with_arm_held_high():
    retreat = coordinated_demo_profile(11.0)

    assert retreat.base_forward_speed < 0.0
    assert retreat.arm_lift_fraction > 0.8
    assert retreat.gripper_command < 0.12
    assert math.isclose(retreat.steer_angle, 0.0, abs_tol=1e-6)
