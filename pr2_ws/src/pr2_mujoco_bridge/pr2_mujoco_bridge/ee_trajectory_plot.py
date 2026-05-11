"""Post-process motion CSV: mass-damper ODE vs logged EE position (no ROS)."""

from __future__ import annotations

import csv
import math
import os
from typing import List, Sequence, Tuple

import numpy as np


def _apply_deadzone(v: float, dz: float) -> float:
    """Match pr2_qp_whole_body_admittance._apply_deadzone."""
    if abs(v) <= dz:
        return 0.0
    return v - dz if v > 0.0 else v + dz


def _parse_float(s: str) -> float | None:
    s = s.strip()
    if s.lower() == "nan" or s == "":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _read_motion_series(
    csv_path: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool, np.ndarray]:
    """Return (t, fx, fy, fz, ee_xyz Nx3, used_odom_force, cmd_vel Nx3).

    Prefer fx_odom/fy_odom/fz_odom when present (same frame as ee_pose / QP).
    cmd_vel columns: cmd_vx, cmd_vy, cmd_vz (EE cartesian velocity command, odom frame).
    """
    t_list: List[float] = []
    fx_list: List[float] = []
    fy_list: List[float] = []
    fz_list: List[float] = []
    ee: List[Tuple[float, float, float]] = []
    cmd: List[Tuple[float, float, float]] = []

    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        use_odom_cols = (
            "fx_odom" in fieldnames and "fy_odom" in fieldnames and "fz_odom" in fieldnames
        )
        has_cmd = "cmd_vx" in fieldnames and "cmd_vy" in fieldnames and "cmd_vz" in fieldnames
        for row in reader:
            t_v = _parse_float(row.get("t_rel_sec", ""))
            if t_v is None:
                continue
            ex = _parse_float(row.get("ee_x", ""))
            ey = _parse_float(row.get("ee_y", ""))
            ez = _parse_float(row.get("ee_z", ""))
            if ex is None or ey is None or ez is None:
                continue
            if use_odom_cols:
                fx = _parse_float(row.get("fx_odom", "")) or 0.0
                fy = _parse_float(row.get("fy_odom", "")) or 0.0
                fz = _parse_float(row.get("fz_odom", "")) or 0.0
            else:
                fx = _parse_float(row.get("fx", "")) or 0.0
                fy = _parse_float(row.get("fy", "")) or 0.0
                fz = _parse_float(row.get("fz", "")) or 0.0
            t_list.append(float(t_v))
            fx_list.append(float(fx))
            fy_list.append(float(fy))
            fz_list.append(float(fz))
            ee.append((float(ex), float(ey), float(ez)))
            if has_cmd:
                cvx = _parse_float(row.get("cmd_vx", "")) or 0.0
                cvy = _parse_float(row.get("cmd_vy", "")) or 0.0
                cvz = _parse_float(row.get("cmd_vz", "")) or 0.0
                cmd.append((float(cvx), float(cvy), float(cvz)))
            else:
                cmd.append((0.0, 0.0, 0.0))

    if not t_list:
        empty = np.zeros(0)
        return (empty, empty, empty, empty, np.zeros((0, 3)), False, np.zeros((0, 3)))

    ee_arr = np.array(ee, dtype=np.float64)
    cmd_arr = np.array(cmd, dtype=np.float64)
    return (
        np.array(t_list, dtype=np.float64),
        np.array(fx_list, dtype=np.float64),
        np.array(fy_list, dtype=np.float64),
        np.array(fz_list, dtype=np.float64),
        ee_arr,
        bool(use_odom_cols),
        cmd_arr,
    )


def _ode_integrate(
    t: np.ndarray,
    fx: np.ndarray,
    fy: np.ndarray,
    fz: np.ndarray,
    ee: np.ndarray,
    mass: np.ndarray,
    damping: np.ndarray,
    deadzone: np.ndarray,
    lpf_alpha: float = 0.15,
) -> np.ndarray:
    """Explicit Euler: v += dt*(f_dz - B*v)/M; x += v*dt.

    When force is below deadzone on all axes, the ODE tracks the actual EE
    position (zero-chase) and resets velocity to zero.  This matches the QP's
    latch-and-hold behaviour and eliminates pre-force-onset position drift.

    lpf_alpha: 1st-order IIR coefficient applied to raw force before deadzone,
    matching the QP's internal wrench LPF (default 0.15 at 100 Hz).
    """
    n = int(t.shape[0])
    if n == 0:
        return np.zeros((0, 3))
    x_sim = np.zeros((n, 3), dtype=np.float64)
    v = np.zeros(3, dtype=np.float64)
    x_sim[0, :] = ee[0, :]
    M = np.maximum(mass.astype(np.float64), 1e-9)
    B = np.maximum(damping.astype(np.float64), 1e-9)
    dz = deadzone.astype(np.float64)
    f_filt = np.zeros(3, dtype=np.float64)

    for k in range(1, n):
        dt = float(t[k] - t[k - 1])
        if not math.isfinite(dt) or dt <= 0.0:
            dt = 1e-6
        f_raw = np.array([float(fx[k]), float(fy[k]), float(fz[k])], dtype=np.float64)
        f_filt = lpf_alpha * f_raw + (1.0 - lpf_alpha) * f_filt
        f0 = _apply_deadzone(float(f_filt[0]), float(dz[0]))
        f1 = _apply_deadzone(float(f_filt[1]), float(dz[1]))
        f2 = _apply_deadzone(float(f_filt[2]), float(dz[2]))
        f_vec = np.array([f0, f1, f2], dtype=np.float64)
        if float(np.max(np.abs(f_vec))) == 0.0:
            # No active force: track actual EE (model the latch-and-hold phase)
            x_sim[k, :] = ee[k, :]
            v[:] = 0.0
            f_filt[:] = 0.0
        else:
            v += dt * (f_vec - B * v) / M
            x_sim[k, :] = x_sim[k - 1, :] + v * dt
    return x_sim


def _integrate_cmd_vel(
    t: np.ndarray,
    cmd: np.ndarray,
    ee: np.ndarray,
    fx: np.ndarray,
    fy: np.ndarray,
    fz: np.ndarray,
    deadzone: np.ndarray,
) -> np.ndarray:
    """Integrate cmd_vel (EE velocity command) to obtain commanded EE position.

    Like the ODE zero-chase, track actual EE when force is below deadzone so
    that the curve starts at the right position when force activates.
    """
    n = int(t.shape[0])
    if n == 0:
        return np.zeros((0, 3))
    x_cmd = np.zeros((n, 3), dtype=np.float64)
    x_cmd[0, :] = ee[0, :]
    dz = deadzone.astype(np.float64)

    for k in range(1, n):
        dt = float(t[k] - t[k - 1])
        if not math.isfinite(dt) or dt <= 0.0:
            dt = 1e-6
        f_raw = np.array([abs(float(fx[k])), abs(float(fy[k])), abs(float(fz[k]))])
        any_force = bool(np.any(f_raw > dz))
        if not any_force:
            x_cmd[k, :] = ee[k, :]
        else:
            x_cmd[k, :] = x_cmd[k - 1, :] + cmd[k, :] * dt
    return x_cmd


def plot_from_csv(
    csv_path: str,
    out_png_path: str,
    mass: Sequence[float],
    damping: Sequence[float],
    deadzone: Sequence[float],
) -> None:
    """Read motion CSV, integrate M dv/dt + B v = F (with deadzone), plot vs ee_*."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, fx, fy, fz, ee, used_odom, cmd = _read_motion_series(csv_path)
    if t.size < 2:
        raise RuntimeError("not enough valid rows with t_rel_sec and ee_x/y/z to plot")

    mass_arr = np.array(list(mass)[:3], dtype=np.float64)
    damp_arr = np.array(list(damping)[:3], dtype=np.float64)
    dz_arr = np.array(list(deadzone)[:3], dtype=np.float64)
    if mass_arr.size < 3:
        mass_arr = np.pad(mass_arr, (0, 3 - mass_arr.size), constant_values=5.0)
    if damp_arr.size < 3:
        damp_arr = np.pad(damp_arr, (0, 3 - damp_arr.size), constant_values=320.0)
    if dz_arr.size < 3:
        dz_arr = np.pad(dz_arr, (0, 3 - dz_arr.size), constant_values=0.8)

    x_sim = _ode_integrate(t, fx, fy, fz, ee, mass_arr, damp_arr, dz_arr)
    x_cmd = _integrate_cmd_vel(t, cmd, ee, fx, fy, fz, dz_arr)

    has_cmd = bool(np.any(cmd != 0.0))
    labels = ("x", "y", "z")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    sub = " (force in odom, from fx_odom/fy_odom/fz_odom)" if used_odom else " (WARNING: legacy CSV: force uses fx/fy/fz wrench frame; may not match ee in odom)"
    fig.suptitle("EE position: kinematic (CSV) vs admittance model" + sub)
    for i, ax in enumerate(axes):
        ax.plot(t, ee[:, i], label="EE (kinematic)", color="C0", linewidth=1.2)
        ax.plot(t, x_sim[:, i], label="ODE (admittance model)", color="C1", linewidth=1.0, linestyle="--")
        if has_cmd:
            ax.plot(t, x_cmd[:, i], label="∫cmd_vel (QP output)", color="C2", linewidth=1.0, linestyle=":")
        ax.set_ylabel(f"pos {labels[i]} (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t_rel_sec (s)")
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png_path) or ".", exist_ok=True)
    fig.savefig(out_png_path, dpi=120)
    plt.close(fig)


def out_png_path_from_csv(csv_path: str) -> str:
    if csv_path.endswith(".csv"):
        return csv_path[:-4] + "_plot.png"
    return csv_path + "_plot.png"
