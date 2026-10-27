"""Experimental allocation: angular admittance first, planned translation second."""
import numpy as np
import osqp
import scipy.sparse as sp


def _solve(P, q, A, lower, upper):
    qp = osqp.OSQP()
    qp.setup(P=sp.csc_matrix(P), q=q, A=sp.csc_matrix(A), l=lower, u=upper,
             verbose=False, eps_abs=1e-7, eps_rel=1e-7, max_iter=10000,
             polishing=True)
    result = qp.solve()
    return result


def allocate(J, admittance_twist, base_reference, lower, upper, linear_velocity_limit=None):
    """Return u and diagnostics; references/linear Jacobians are in odom.

    Input angular velocity is the existing, limited angular-admittance output.
    No orientation reference or orientation error enters this allocation.
    If it is infeasible, preserve the closest achievable angular task instead.
    """
    J = np.asarray(J, dtype=float)
    v = np.asarray(admittance_twist, dtype=float)
    b = np.asarray(base_reference, dtype=float)
    lower, upper = np.asarray(lower), np.asarray(upper)
    if J.shape != (6, 10) or v.shape != (6,) or b.shape != (3,):
        raise ValueError("expected J[6,10], admittance_twist[6], base_reference[3]")
    if not all(np.isfinite(x).all() for x in (J, v, b, lower, upper)):
        raise ValueError("allocation input is not finite")
    if lower.shape != (10,) or upper.shape != (10,) or np.any(lower > upper):
        raise ValueError("invalid velocity bounds")
    Jp, Jr = J[:3], J[3:]
    E = np.eye(10)[7:]
    linear = v[:3] + Jp[:, 7:] @ b
    if linear_velocity_limit is not None:
        cap = np.asarray(linear_velocity_limit, dtype=float)
        if cap.shape != (3,) or np.any(cap <= 0) or not np.isfinite(cap).all():
            raise ValueError("invalid linear velocity limit")
        linear = np.clip(linear, -cap, cap)
    # Same .01 velocity regularization as the current baseline; base tracking
    # has unit weight. This is a feasibility prototype, not tuned parameters.
    P = Jp.T @ Jp + E.T @ E + .01 * np.eye(10)
    q = -Jp.T @ linear - E.T @ b
    A = np.vstack([np.eye(10), Jr])
    angular = v[3:].copy()
    result = _solve(P, q, A, np.r_[lower, angular], np.r_[upper, angular])
    fallback = False
    if result.info.status_val not in (1, 2):
        # A tiny tie breaker makes the first-stage QP well conditioned.
        first = _solve(Jr.T @ Jr + 1e-12 * np.eye(10), -Jr.T @ angular,
                       np.eye(10), lower, upper)
        if first.info.status_val not in (1, 2):
            raise RuntimeError(f"angular task infeasible: {first.info.status}")
        angular = Jr @ first.x
        result = _solve(P, q, A, np.r_[lower, angular], np.r_[upper, angular])
        fallback = True
    if result.info.status_val not in (1, 2):
        raise RuntimeError(f"allocation failed: {result.info.status}")
    u = np.asarray(result.x)
    angular_error = Jr @ u - angular
    violation = float(np.maximum.reduce([lower-u, u-upper, np.zeros(10)]).max())
    if max(abs(angular_error)) > 1e-5 or violation > 1e-5:
        raise RuntimeError("allocation solution violates task priority or bounds")
    return u, dict(angular_target=angular.tolist(),
                   angular_requested=v[3:].tolist(),
                   angular_error=angular_error.tolist(),
                   primary_infeasible=fallback,
                   linear_reference=linear.tolist(),
                   linear_achieved=(Jp @ u).tolist(),
                   base_reference=b.tolist(), bound_violation=violation)
