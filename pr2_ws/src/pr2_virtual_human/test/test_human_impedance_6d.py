import numpy as np
import pytest
from dataclasses import replace

from pr2_virtual_human.human_impedance_6d import (
    HumanImpedance6D,
    HumanImpedanceConfig,
    wrench_at_body_com,
)
from pr2_virtual_human.trajectory_6d import TrajectorySample
from pr2_virtual_human.spatial import matrix_to_quaternion, rotvec_to_matrix


def config() -> HumanImpedanceConfig:
    return HumanImpedanceConfig(
        linear_stiffness=np.array([20.0, 20.0, 40.0]),
        linear_damping=np.array([8.0, 8.0, 8.0]),
        angular_stiffness=np.array([0.5, 3.0, 3.0]),
        angular_damping=np.array([0.1, 1.2, 1.2]),
        force_limit=np.array([8.0, 8.0, 45.0]),
        task_torque_limit=np.array([0.8, 5.0, 5.0]),
    )


def target() -> TrajectorySample:
    return TrajectorySample(
        position=np.array([0.1, -0.2, 1.0]),
        linear_velocity=np.zeros(3),
        linear_acceleration=np.zeros(3),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity=np.zeros(3),
        relative_rotvec=np.zeros(3),
    )


def test_zero_error_produces_zero_wrench() -> None:
    result = HumanImpedance6D(config()).evaluate(
        target(), target().position, np.zeros(3), target().quaternion, np.zeros(3)
    )
    np.testing.assert_allclose(result.force, 0.0)
    np.testing.assert_allclose(result.task_torque, 0.0)


def test_wrench_is_limited() -> None:
    result = HumanImpedance6D(config()).evaluate(
        target(), np.array([-10.0, 10.0, -10.0]), np.zeros(3),
        np.array([0.0, 1.0, 0.0, 0.0]), np.zeros(3),
    )
    assert np.all(np.abs(result.force) <= config().force_limit)
    assert np.all(np.abs(result.task_torque) <= config().task_torque_limit)


def test_natural_endpoint_moment_is_preserved_not_cancelled() -> None:
    force = np.array([0.0, 0.0, 12.0])
    offset = np.array([1.0, 0.0, 0.0])
    task_torque = np.array([0.2, 0.3, -0.1])
    mapped_force, total_torque, natural_moment = wrench_at_body_com(force, task_torque, offset)
    np.testing.assert_allclose(mapped_force, force)
    np.testing.assert_allclose(natural_moment, [0.0, -12.0, 0.0])
    np.testing.assert_allclose(total_torque, natural_moment + task_torque)
    assert not np.allclose(total_torque, task_torque)


def test_task_torque_does_not_depend_on_endpoint_force() -> None:
    # The impedance API has no endpoint force, robot wrench, or lever-arm input.
    impedance = HumanImpedance6D(config())
    actual_q = np.array([0.9996875, 0.0249974, 0.0, 0.0])
    first = impedance.evaluate(target(), target().position, np.zeros(3), actual_q, np.zeros(3))
    second = impedance.evaluate(target(), target().position, np.zeros(3), actual_q, np.zeros(3))
    np.testing.assert_allclose(first.task_torque, second.task_torque)


@pytest.mark.parametrize("yaw", [0.0, np.pi / 2, np.pi])
def test_body_angular_gains_follow_the_long_axis_through_a_turn(yaw: float) -> None:
    heading = rotvec_to_matrix(np.array([0.0, 0.0, yaw]))
    actual_rotation = heading @ rotvec_to_matrix(np.array([0.1, 0.0, 0.0]))
    sample = replace(target(), quaternion=matrix_to_quaternion(heading))
    result = HumanImpedance6D(replace(config(), angular_frame="body")).evaluate(
        sample, sample.position, np.zeros(3), matrix_to_quaternion(actual_rotation),
        actual_rotation @ np.array([0.2, 0.0, 0.0]),
    )
    # The same physical roll error/speed must use Kx=.5 and Dx=.1 at
    # every heading. Torque is returned in world coordinates.
    np.testing.assert_allclose(result.task_torque, heading @ np.array([-0.07, 0.0, 0.0]), atol=1e-12)


def test_legacy_world_frame_keeps_its_original_heading_dependent_response() -> None:
    heading = rotvec_to_matrix(np.array([0.0, 0.0, np.pi / 2]))
    actual_rotation = heading @ rotvec_to_matrix(np.array([0.1, 0.0, 0.0]))
    sample = replace(target(), quaternion=matrix_to_quaternion(heading))
    result = HumanImpedance6D(config()).evaluate(
        sample, sample.position, np.zeros(3), matrix_to_quaternion(actual_rotation),
        actual_rotation @ np.array([0.2, 0.0, 0.0]),
    )
    np.testing.assert_allclose(result.task_torque, [0.0, -0.54, 0.0], atol=1e-12)


def test_body_frame_preserves_world_component_torque_limits() -> None:
    result = HumanImpedance6D(replace(config(), angular_frame="body")).evaluate(
        target(), target().position, np.zeros(3),
        matrix_to_quaternion(rotvec_to_matrix(np.array([0.0, 0.0, 1.0]))),
        np.array([100.0, -100.0, 100.0]),
    )
    assert np.all(np.abs(result.task_torque) <= config().task_torque_limit)


def test_invalid_angular_frame_is_rejected() -> None:
    with pytest.raises(ValueError, match="angular_frame"):
        replace(config(), angular_frame="invalid")

