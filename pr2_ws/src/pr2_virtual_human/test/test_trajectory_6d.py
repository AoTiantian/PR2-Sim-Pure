import numpy as np

from pr2_virtual_human.spatial import orientation_error_world
from pr2_virtual_human.trajectory_6d import Trajectory6D, TrajectoryConfig


def make_trajectory() -> Trajectory6D:
    return Trajectory6D(
        TrajectoryConfig(
            duration_sec=12.0,
            cycles=1.0,
            position_amplitude=np.array([0.20, 0.12, 0.03]),
            orientation_amplitude=np.array([0.08, 0.06, 0.10]),
        ),
        initial_position=np.array([1.0, -2.0, 0.8]),
        initial_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
    )


def test_closed_pose_velocity_and_acceleration() -> None:
    trajectory = make_trajectory()
    start = trajectory.sample(0.0)
    end = trajectory.sample(12.0)
    np.testing.assert_allclose(end.position, start.position, atol=1.0e-12)
    np.testing.assert_allclose(end.quaternion, start.quaternion, atol=1.0e-12)
    np.testing.assert_allclose(start.linear_velocity, 0.0, atol=1.0e-12)
    np.testing.assert_allclose(end.linear_velocity, 0.0, atol=1.0e-12)
    np.testing.assert_allclose(start.linear_acceleration, 0.0, atol=1.0e-12)
    np.testing.assert_allclose(end.linear_acceleration, 0.0, atol=1.0e-12)
    np.testing.assert_allclose(start.angular_velocity, 0.0, atol=1.0e-12)
    np.testing.assert_allclose(end.angular_velocity, 0.0, atol=1.0e-12)


def test_clamps_before_and_after_trajectory() -> None:
    trajectory = make_trajectory()
    before = trajectory.sample(-5.0)
    after = trajectory.sample(99.0)
    np.testing.assert_allclose(before.position, after.position, atol=1.0e-12)
    np.testing.assert_allclose(orientation_error_world(before.quaternion, after.quaternion), 0.0, atol=1.0e-12)


def test_all_six_axes_are_excited() -> None:
    trajectory = make_trajectory()
    samples = [trajectory.sample(t) for t in np.linspace(0.0, 12.0, 101)]
    positions = np.array([sample.position for sample in samples])
    orientations = np.array([sample.relative_rotvec for sample in samples])
    assert np.all(np.ptp(positions, axis=0) > 0.01)
    assert np.all(np.ptp(orientations, axis=0) > 0.01)

