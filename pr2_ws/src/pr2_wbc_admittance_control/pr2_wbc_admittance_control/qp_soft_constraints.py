from __future__ import annotations

import numpy as np


def task_nullspace_projector(
    task_jacobian: np.ndarray,
    task_weight: np.ndarray | None = None,
    *,
    rcond: float = 1e-9,
) -> np.ndarray:
    """Return the Euclidean projector onto the weighted task nullspace."""
    j = np.asarray(task_jacobian, dtype=np.float64)
    if j.ndim != 2:
        raise ValueError("task_jacobian must be a 2-D array")

    rows, cols = j.shape
    if task_weight is None:
        j_weighted = j
    else:
        sqrt_w = _sqrt_task_weight(np.asarray(task_weight, dtype=np.float64), rows)
        j_weighted = sqrt_w @ j

    projector = np.eye(cols, dtype=np.float64) - np.linalg.pinv(j_weighted, rcond=rcond) @ j_weighted
    return 0.5 * (projector + projector.T)


def nullspace_posture_cost(
    task_jacobian: np.ndarray,
    task_weight: np.ndarray | None,
    posture_weight: np.ndarray,
    posture_target: np.ndarray,
    *,
    rcond: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray]:
    """Build H/g for 0.5 * ||N (u - posture_target)||^2_W.

    The returned matrices are intended for the OSQP objective
    0.5 u.T @ H @ u - g.T @ u. Because the cost is projected through N,
    it can only bias velocities in the task nullspace.
    """
    j = np.asarray(task_jacobian, dtype=np.float64)
    if j.ndim != 2:
        raise ValueError("task_jacobian must be a 2-D array")
    cols = int(j.shape[1])

    w = _as_square_weight(posture_weight, cols, "posture_weight")
    target = np.asarray(posture_target, dtype=np.float64)
    if target.shape != (cols,):
        raise ValueError(f"posture_target must have shape ({cols},)")

    n = task_nullspace_projector(j, task_weight, rcond=rcond)
    h = n.T @ w @ n
    h = 0.5 * (h + h.T)
    g = h @ target
    return h, g


def _sqrt_task_weight(weight: np.ndarray, rows: int) -> np.ndarray:
    if weight.ndim == 1:
        if weight.shape != (rows,):
            raise ValueError(f"task_weight must have len={rows}")
        if np.any(weight < -1e-12):
            raise ValueError("task_weight must be positive semidefinite")
        return np.diag(np.sqrt(np.maximum(weight, 0.0)))

    if weight.shape != (rows, rows):
        raise ValueError(f"task_weight must have shape ({rows}, {rows})")

    weight_sym = 0.5 * (weight + weight.T)
    eigvals, eigvecs = np.linalg.eigh(weight_sym)
    if np.any(eigvals < -1e-12):
        raise ValueError("task_weight must be positive semidefinite")
    eigvals = np.maximum(eigvals, 0.0)
    return eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T


def _as_square_weight(weight: np.ndarray, cols: int, name: str) -> np.ndarray:
    w = np.asarray(weight, dtype=np.float64)
    if w.ndim == 1:
        if w.shape != (cols,):
            raise ValueError(f"{name} must have len={cols}")
        if np.any(w < -1e-12):
            raise ValueError(f"{name} must be positive semidefinite")
        return np.diag(np.maximum(w, 0.0))

    if w.shape != (cols, cols):
        raise ValueError(f"{name} must have shape ({cols}, {cols})")
    w = 0.5 * (w + w.T)
    eigvals = np.linalg.eigvalsh(w)
    if np.any(eigvals < -1e-12):
        raise ValueError(f"{name} must be positive semidefinite")
    return w
