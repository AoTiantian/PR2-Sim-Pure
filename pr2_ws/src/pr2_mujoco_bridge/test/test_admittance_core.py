import math

import numpy as np

from pr2_mujoco_bridge.admittance_core import (
    AdmittanceState,
    AxisGains,
    clip_norm,
    limit_joint_velocity_near_limits,
    project_end_effector_wrench_to_base,
    rotate_world_vector_to_body,
    solve_dls_velocity,
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
