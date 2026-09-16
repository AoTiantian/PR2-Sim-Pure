import numpy as np

from pr2_virtual_human.spatial import orientation_error_world
from pr2_virtual_human.trajectory_6d import (
    Trajectory6D,
    TrajectoryConfig,
    WaypointTrajectory6D,
    build_trajectory,
)


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



def make_waypoint_trajectory() -> Trajectory6D | WaypointTrajectory6D:
    config = TrajectoryConfig(
        duration_sec=12.0,
        cycles=1.0,
        position_amplitude=np.zeros(3),
        orientation_amplitude=np.zeros(3),
        mode="waypoints",
        waypoints=np.array([
            [0.0, 0.0, 0.0],
            [0.0, -0.8, 0.0],
            [0.3, -0.8, np.pi],
        ]),
        segment_durations_sec=np.array([6.0, 6.0]),
    )
    return build_trajectory(
        config,
        np.array([1.0, -2.0, 0.8]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )


def test_waypoint_passes_through_waypoints() -> None:
    trajectory = make_waypoint_trajectory()
    anchors = np.array([[1.0, -2.0, 0.8], [1.0, -2.8, 0.8], [1.3, -2.8, 0.8]])
    yaws = [0.0, 0.0, np.pi]
    for t, anchor, yaw in zip((0.0, 6.0, 12.0), anchors, yaws):
        sample = trajectory.sample(t)
        np.testing.assert_allclose(sample.position, anchor, atol=1.0e-9)
        expected = np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])
        np.testing.assert_allclose(
            orientation_error_world(sample.quaternion, expected), 0.0, atol=1.0e-9
        )


def test_waypoint_zero_velocity_at_boundaries() -> None:
    trajectory = make_waypoint_trajectory()
    for t in (0.0, 6.0, 12.0):
        sample = trajectory.sample(t)
        np.testing.assert_allclose(sample.linear_velocity, 0.0, atol=1.0e-9)
        np.testing.assert_allclose(sample.angular_velocity, 0.0, atol=1.0e-9)


def test_waypoint_interior_motion_and_signs() -> None:
    trajectory = make_waypoint_trajectory()
    mid_first = trajectory.sample(3.0)
    assert -2.6 < mid_first.position[1] < -2.0
    assert mid_first.linear_velocity[1] < 0.0
    turn = trajectory.sample(9.0)
    assert 0.5 < turn.relative_rotvec[2] < np.pi


def test_waypoint_clamps_after_duration() -> None:
    trajectory = make_waypoint_trajectory()
    end = trajectory.sample(12.0)
    after = trajectory.sample(99.0)
    np.testing.assert_allclose(after.position, end.position, atol=1.0e-12)
    np.testing.assert_allclose(after.quaternion, end.quaternion, atol=1.0e-12)


def test_build_trajectory_dispatch() -> None:
    sinusoid = make_trajectory()
    assert isinstance(sinusoid, Trajectory6D)
    assert isinstance(make_waypoint_trajectory(), WaypointTrajectory6D)
    try:
        TrajectoryConfig(
            duration_sec=12.0,
            cycles=1.0,
            position_amplitude=np.zeros(3),
            orientation_amplitude=np.zeros(3),
            mode="waypoints",
            waypoints=np.array([[0.1, 0.0, 0.0], [0.0, -0.8, 0.0]]),
        )
        raise AssertionError("non-zero first waypoint must be rejected")
    except ValueError:
        pass
