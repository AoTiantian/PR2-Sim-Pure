import math

import numpy as np

from pr2_mujoco_bridge.admittance_core import (
    AdmittanceState,
    AxisGains,
    advance_bounded_reference,
    clip_norm,
    limit_joint_velocity_near_limits,
    project_end_effector_wrench_to_base,
    rotate_world_vector_to_body,
    settle_admittance_state,
    solve_dls_velocity,
    solve_dls_velocity_with_nullspace,
)


def test_second_order_admittance_returns_to_equilibrium_after_force_release() -> None:
    gains = AxisGains(
        mass=np.array([2.0, 2.0, 2.0], dtype=np.float64),
        damping=np.array([35.0, 35.0, 35.0], dtype=np.float64),
        stiffness=np.array([45.0, 45.0, 45.0], dtype=np.float64),
    )
    state = AdmittanceState.zeros(dim=3)

    dt = 0.01
    for step in range(700):
        wrench = np.array([8.0, 0.0, 0.0], dtype=np.float64) if step < 250 else np.zeros(3)
        state = state.step(force=wrench, gains=gains, dt=dt)

    assert np.linalg.norm(state.velocity) < 1e-2
    assert np.linalg.norm(state.displacement) < 2e-2


def test_rotate_world_vector_to_body_handles_base_yaw() -> None:
    yaw = math.pi / 2.0
    rot_world_body = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    world_force = np.array([0.0, 10.0, 0.0], dtype=np.float64)

    body_force = rotate_world_vector_to_body(world_force, rot_world_body)

    np.testing.assert_allclose(body_force, np.array([10.0, 0.0, 0.0]), atol=1e-6)


def test_project_end_effector_wrench_to_base_keeps_planar_force_and_yaw() -> None:
    ee_pos_in_base = np.array([0.8, 0.2, 1.1], dtype=np.float64)
    ee_force_in_base = np.array([6.0, 4.0, 9.0], dtype=np.float64)

    projected = project_end_effector_wrench_to_base(
        ee_pos_in_base=ee_pos_in_base,
        ee_force_in_base=ee_force_in_base,
        planar_scale=0.5,
        yaw_scale=0.25,
    )

    np.testing.assert_allclose(projected[:3], np.array([3.0, 2.0, 0.0]), atol=1e-6)
    assert projected[3] == 0.0
    assert projected[4] == 0.0
    assert projected[5] == ((0.8 * 4.0) - (0.2 * 6.0)) * 0.25


def test_clip_norm_scales_without_changing_direction() -> None:
    vec = np.array([3.0, 4.0, 0.0], dtype=np.float64)

    clipped = clip_norm(vec, max_norm=2.5)

    np.testing.assert_allclose(clipped, np.array([1.5, 2.0, 0.0]), atol=1e-6)


def test_solve_dls_velocity_respects_damping() -> None:
    jacobian = np.array(
        [
            [1.0, 0.0],
            [0.0, 0.2],
            [0.0, 0.0],
        ],
        dtype=np.float64,
    )
    ee_velocity = np.array([0.2, 0.2, 0.0], dtype=np.float64)

    qdot = solve_dls_velocity(jacobian, ee_velocity, damping_lambda=0.4)

    assert qdot.shape == (2,)
    assert qdot[1] < 0.5


def test_limit_joint_velocity_near_limits_scales_continuously() -> None:
    q = np.array([0.02, 0.5], dtype=np.float64)
    qdot = np.array([-0.3, 0.3], dtype=np.float64)
    qmin = np.array([0.0, 0.0], dtype=np.float64)
    qmax = np.array([1.0, 1.0], dtype=np.float64)

    limited = limit_joint_velocity_near_limits(
        q=q,
        qdot_des=qdot,
        qmin=qmin,
        qmax=qmax,
        margin=0.1,
    )

    assert -0.1 < limited[0] < 0.0
    assert limited[1] == qdot[1]


def test_settle_admittance_state_zeros_small_idle_axes() -> None:
    state = AdmittanceState(
        displacement=np.array([2.0e-4, -3.0e-4, 1.0e-4], dtype=np.float64),
        velocity=np.array([4.0e-4, -6.0e-4, 2.0e-4], dtype=np.float64),
    )

    settled = settle_admittance_state(
        state=state,
        force=np.zeros(3, dtype=np.float64),
        force_epsilon=np.array([0.05, 0.05, 0.05], dtype=np.float64),
        displacement_epsilon=np.array([5.0e-4, 5.0e-4, 5.0e-4], dtype=np.float64),
        velocity_epsilon=np.array([1.0e-3, 1.0e-3, 1.0e-3], dtype=np.float64),
        idle_velocity_decay=0.6,
    )

    assert np.allclose(settled.displacement, 0.0)
    assert np.allclose(settled.velocity, 0.0)


def test_settle_admittance_state_preserves_active_axes() -> None:
    state = AdmittanceState(
        displacement=np.array([1.5e-2, 2.0e-4, 0.0], dtype=np.float64),
        velocity=np.array([8.0e-3, 5.0e-4, 0.0], dtype=np.float64),
    )

    settled = settle_admittance_state(
        state=state,
        force=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        force_epsilon=np.array([0.05, 0.05, 0.05], dtype=np.float64),
        displacement_epsilon=np.array([5.0e-4, 5.0e-4, 5.0e-4], dtype=np.float64),
        velocity_epsilon=np.array([1.0e-3, 1.0e-3, 1.0e-3], dtype=np.float64),
        idle_velocity_decay=0.5,
    )

    assert settled.displacement[0] == state.displacement[0]
    assert settled.velocity[0] == state.velocity[0]
    assert settled.displacement[1] == 0.0
    assert settled.velocity[1] == 0.0


def test_advance_bounded_reference_integrates_velocity_inside_limits() -> None:
    q_ref = np.array([0.0, 0.1], dtype=np.float64)
    q_cur = np.array([0.0, 0.1], dtype=np.float64)
    qdot = np.array([0.5, -0.25], dtype=np.float64)
    qmin = np.array([-1.0, -1.0], dtype=np.float64)
    qmax = np.array([1.0, 1.0], dtype=np.float64)

    updated = advance_bounded_reference(
        q_ref=q_ref,
        q_cur=q_cur,
        qdot_cmd=qdot,
        dt=0.1,
        qmin=qmin,
        qmax=qmax,
        max_error=0.2,
    )

    np.testing.assert_allclose(updated, np.array([0.05, 0.075]), atol=1e-9)


def test_advance_bounded_reference_clips_joint_error_to_current_state() -> None:
    q_ref = np.array([0.25, -0.4], dtype=np.float64)
    q_cur = np.array([0.0, 0.0], dtype=np.float64)
    qdot = np.zeros(2, dtype=np.float64)
    qmin = np.array([-1.0, -1.0], dtype=np.float64)
    qmax = np.array([1.0, 1.0], dtype=np.float64)

    updated = advance_bounded_reference(
        q_ref=q_ref,
        q_cur=q_cur,
        qdot_cmd=qdot,
        dt=0.1,
        qmin=qmin,
        qmax=qmax,
        max_error=0.12,
    )

    np.testing.assert_allclose(updated, np.array([0.12, -0.12]), atol=1e-9)


def test_solve_dls_velocity_with_nullspace_keeps_task_velocity() -> None:
    jacobian = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
    ee_velocity = np.array([0.2], dtype=np.float64)
    q_error = np.array([0.0, 1.0, -1.0], dtype=np.float64)

    qdot = solve_dls_velocity_with_nullspace(
        jacobian=jacobian,
        ee_velocity=ee_velocity,
        damping_lambda=0.1,
        q_error=q_error,
        nullspace_gain=0.5,
    )

    assert qdot.shape == (3,)
    np.testing.assert_allclose(jacobian @ qdot, np.array([0.1980198]), atol=1e-6)
    assert qdot[1] > 0.0
    assert qdot[2] < 0.0


def test_solve_dls_velocity_with_nullspace_is_noop_without_redundancy() -> None:
    jacobian = np.eye(2, dtype=np.float64)
    ee_velocity = np.array([0.1, -0.2], dtype=np.float64)
    q_error = np.array([1.0, 1.0], dtype=np.float64)

    qdot = solve_dls_velocity_with_nullspace(
        jacobian=jacobian,
        ee_velocity=ee_velocity,
        damping_lambda=0.1,
        q_error=q_error,
        nullspace_gain=0.5,
    )
    plain = solve_dls_velocity(jacobian, ee_velocity, damping_lambda=0.1)

    np.testing.assert_allclose(qdot, plain, atol=1e-9)
