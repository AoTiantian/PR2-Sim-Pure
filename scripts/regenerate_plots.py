#!/usr/bin/env python3
"""Auto-plot: regenerate virtual_human_force.png + virtual_human_plots.png.

Usage: python3 regenerate_plots.py <log.csv>
"""
import csv, math, sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams.update({"font.size": 11, "figure.dpi": 120})


def load(path):
    rows = []
    with open(path) as fh:
        r = csv.reader(fh); _ = next(r)
        for row in r:
            try:
                t = float(row[0])
                ee = [float(row[i]) for i in (3, 4, 5)]
                q = [float(row[i]) for i in (6, 7, 8, 9)]
                f = [float(row[i]) for i in (17, 18, 19)]
                if not all(np.isfinite(v) for v in ee + q + f):
                    continue
                rows.append([t] + ee + q + f)
            except (ValueError, IndexError):
                continue
    return np.array(rows)


def hand_pos(ee_i, qw, qx, qy, qz):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        R = np.eye(3)
    else:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
        ])
    return ee_i + R @ np.array([1.18, 0.0, 0.0])


def main():
    if len(sys.argv) < 2:
        import glob
        logs = sorted(glob.glob("/workspace/logs/vh_*.csv"))
        if not logs:
            print("No logs found")
            return
        csv_path = logs[-1]
    else:
        csv_path = sys.argv[1]

    print(f"Plotting from: {csv_path}")
    d = load(csv_path)
    t = d[:, 0] - d[0, 0]
    ee = d[:, 1:4]

    # Hand position
    h = np.array([hand_pos(ee[i], d[i, 4], d[i, 5], d[i, 6], d[i, 7]) for i in range(len(d))])

    # Active segment
    i0 = np.argmax(t > 2.0)
    i1 = max(i0 + 10, len(t) - 30)
    T = t[i0:i1] - t[i0]
    H = h[i0:i1]
    EE_t = ee[i0:i1]

    # Desired: linear_x at 0.05 m/s from latched
    speed = 0.05
    latched = H[0].copy()
    des_x = latched[0] + speed * T
    des_y = np.full_like(T, latched[1])
    des_z = np.full_like(T, latched[2])

    # Errors
    ex = H[:, 0] - des_x
    ey = H[:, 1] - des_y
    ez = H[:, 2] - des_z
    emag = np.sqrt(ex**2 + ey**2 + ez**2)

    # ===================================================================
    # FIGURE 1: virtual_human_force.png — tracking error + position
    # ===================================================================
    fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Top: tracking error
    ax1a.plot(T, ex, "b-", linewidth=1.5, label="err_x")
    ax1a.plot(T, ey, "r-", linewidth=1.5, label="err_y")
    ax1a.plot(T, ez, "g-", linewidth=1.5, label="err_z")
    ax1a.axhline(0, color="gray", ls="--", lw=0.5)
    ax1a.set_ylabel("Error (m)")
    ax1a.set_title(f"Hand Tracking Error  (final: x={abs(ex[-1]):.3f}m, y={abs(ey[-1]):.3f}m, z={abs(ez[-1]):.3f}m)")
    ax1a.legend(loc="best", fontsize=8)
    ax1a.grid(True, alpha=0.3)

    # Bottom: hand position tracking
    ax1b.plot(T, des_x, "b--", linewidth=1.5, label="desired x")
    ax1b.plot(T, des_y, "r--", linewidth=1.5, label="desired y")
    ax1b.plot(T, H[:, 0], "b-", linewidth=2, label="actual x")
    ax1b.plot(T, H[:, 1], "r-", linewidth=2, label="actual y")
    ax1b.set_xlabel("Time (s)")
    ax1b.set_ylabel("Hand Position (m)")
    ax1b.set_title("Hand Position Tracking (desired vs actual)")
    ax1b.legend(loc="best", fontsize=8)
    ax1b.grid(True, alpha=0.3)

    plt.tight_layout()
    fig1.savefig("/workspace/virtual_human_force.png", dpi=120)
    print("Saved: /workspace/virtual_human_force.png")
    plt.close(fig1)

    # ===================================================================
    # FIGURE 2: virtual_human_plots.png — XY trajectory + EE position
    # ===================================================================
    fig2, (ax2a, ax2b) = plt.subplots(1, 2, figsize=(16, 7))

    # Left: XY hand trajectory
    ax2a.plot(des_x, des_y, "b--", linewidth=1.5, label="desired")
    ax2a.plot(H[:, 0], H[:, 1], "b-", linewidth=2, label="actual")
    ax2a.plot(H[0, 0], H[0, 1], "go", markersize=8, label="start")
    ax2a.plot(H[-1, 0], H[-1, 1], "ro", markersize=8, label="end")
    # Mark tracking error with a connecting line at the end
    ax2a.plot([des_x[-1], H[-1, 0]], [des_y[-1], H[-1, 1]], "r:", lw=1, alpha=0.5)
    ax2a.annotate(f"{emag[-1]:.3f}m", ((des_x[-1] + H[-1, 0]) / 2, (des_y[-1] + H[-1, 1]) / 2),
                  fontsize=7, color="red", ha="center",
                  bbox=dict(boxstyle="round,pad=0.1", facecolor="white", alpha=0.7))
    ax2a.set_xlabel("x (m)")
    ax2a.set_ylabel("y (m)")
    ax2a.set_title("XY Hand Trajectory (top-down) — linear_x")
    ax2a.legend(loc="best", fontsize=8)
    ax2a.grid(True, alpha=0.3)

    # Right: EE position response (what the robot is doing)
    ax2b.plot(T, EE_t[:, 0], "b-", linewidth=1.5, label="EE x")
    ax2b.plot(T, EE_t[:, 1], "r-", linewidth=1.5, label="EE y")
    ax2b.plot(T, EE_t[:, 2], "g-", linewidth=1.5, label="EE z")
    ax2b.set_xlabel("Time (s)")
    ax2b.set_ylabel("EE Position (m)")
    ax2b.set_title("EE Response")
    ax2b.legend(loc="best", fontsize=8)
    ax2b.grid(True, alpha=0.3)

    # Stats box
    dx_h = H[-1, 0] - H[0, 0]
    dy_h = H[-1, 1] - H[0, 1]
    dx_d = speed * T[-1]
    ratio = dx_h / dx_d * 100 if dx_d > 0 else 0
    stats = (
        f"Hand tracking:\n"
        f"  Δx={dx_h:.3f}m (desired {dx_d:.3f}m, {ratio:.0f}%)\n"
        f"  Δy={dy_h:.3f}m\n"
        f"  err x={abs(ex[-1]):.3f}m  y={abs(ey[-1]):.3f}m\n"
        f"  err z={abs(ez[-1]):.3f}m\n"
        f"Force: xfrc_applied (no QP wrench)"
    )
    ax2b.text(0.98, 0.97, stats, transform=ax2b.transAxes, fontsize=8,
              va="top", ha="right", fontfamily="monospace",
              bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.9))

    plt.tight_layout()
    fig2.savefig("/workspace/virtual_human_plots.png", dpi=120)
    print("Saved: /workspace/virtual_human_plots.png")
    plt.close(fig2)


if __name__ == "__main__":
    main()
