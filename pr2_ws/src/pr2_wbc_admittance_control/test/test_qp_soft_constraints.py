from __future__ import annotations

import numpy as np

from pr2_wbc_admittance_control.qp_soft_constraints import (
    nullspace_posture_cost,
    task_nullspace_projector,
)


def test_task_nullspace_projector_removes_task_motion() -> None:
    rng = np.random.default_rng(7)
    j = rng.normal(size=(6, 10))
    task_weight = np.diag([1.0, 1.0, 4.0, 1.0, 1.0, 0.5])
    posture_target = rng.normal(size=10)

    n = task_nullspace_projector(j, task_weight)
    projected = n @ posture_target

    assert np.linalg.norm(j @ projected) < 1e-10
    assert np.linalg.norm(n @ n - n) < 1e-10


def test_nullspace_posture_cost_does_not_change_full_rank_task_solution() -> None:
    rng = np.random.default_rng(11)
    j = rng.normal(size=(6, 6))
    while np.linalg.matrix_rank(j) < 6:
        j = rng.normal(size=(6, 6))

    task_weight = np.diag([1.0, 1.0, 4.0, 1.0, 1.0, 0.5])
    posture_weight = np.eye(6) * 10.0
    posture_target = rng.normal(size=6)
    v_des = rng.normal(size=6)
    reg = np.eye(6) * 1e-9

    p_primary = j.T @ task_weight @ j + reg
    q_primary = -(j.T @ task_weight @ v_des)
    primary_solution = np.linalg.solve(p_primary, -q_primary)

    h_posture, g_posture = nullspace_posture_cost(
        j,
        task_weight,
        posture_weight,
        posture_target,
    )
    soft_solution = np.linalg.solve(p_primary + h_posture, -(q_primary - g_posture))

    assert np.linalg.norm(soft_solution - primary_solution) < 1e-8


def test_direct_posture_cost_would_compete_with_task_solution() -> None:
    rng = np.random.default_rng(17)
    j = rng.normal(size=(6, 6))
    while np.linalg.matrix_rank(j) < 6:
        j = rng.normal(size=(6, 6))

    task_weight = np.eye(6)
    posture_weight = np.eye(6) * 10.0
    posture_target = rng.normal(size=6) * 2.0
    v_des = rng.normal(size=6)
    reg = np.eye(6) * 1e-9

    p_primary = j.T @ task_weight @ j + reg
    q_primary = -(j.T @ task_weight @ v_des)
    direct_solution = np.linalg.solve(
        p_primary + posture_weight,
        -(q_primary - posture_weight @ posture_target),
    )

    assert np.linalg.norm(j @ direct_solution - v_des) > 1e-3


def test_nullspace_posture_cost_biases_only_redundant_dofs() -> None:
    rng = np.random.default_rng(23)
    j = rng.normal(size=(6, 10))
    while np.linalg.matrix_rank(j) < 6:
        j = rng.normal(size=(6, 10))

    task_weight = np.diag([1.0, 1.0, 4.0, 1.0, 1.0, 0.5])
    posture_weight = np.diag([0.1] * 7 + [0.0, 0.0, 0.0])
    posture_target = rng.normal(size=10)
    v_des = rng.normal(size=6)
    reg = np.eye(10) * 1e-9

    p_primary = j.T @ task_weight @ j + reg
    q_primary = -(j.T @ task_weight @ v_des)
    primary_solution = np.linalg.solve(p_primary, -q_primary)

    h_posture, g_posture = nullspace_posture_cost(
        j,
        task_weight,
        posture_weight,
        posture_target,
    )
    soft_solution = np.linalg.solve(p_primary + h_posture, -(q_primary - g_posture))

    n = task_nullspace_projector(j, task_weight)
    primary_error = np.linalg.norm(n @ (primary_solution - posture_target))
    soft_error = np.linalg.norm(n @ (soft_solution - posture_target))

    assert np.linalg.norm(j @ soft_solution - j @ primary_solution) < 1e-7
    assert soft_error < primary_error
