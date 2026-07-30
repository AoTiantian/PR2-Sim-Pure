#!/usr/bin/env python3
"""Generate comparison plots: v1 (original) vs v3 (tuned) PID virtual human."""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from numpy.fft import rfft, rfftfreq

matplotlib.rcParams.update({"font.size": 10, "figure.dpi": 120})

OLD_LOG = "/workspace/logs/pr2_motion_20260727_163241.csv"
NEW_LOG = "/workspace/logs/vh_pid_v3.csv"
OUT = "/workspace/virtual_human_compare.png"


def load(path):
    rows = []
    with open(path) as fh:
        r = csv.reader(fh)
        _ = next(r)
        for row in r:
            try:
                t = float(row[0])
                fx = float(row[17])
                if row[3] in ("nan", ""):
                    continue
                eex, eey, eez = float(row[3]), float(row[4]), float(row[5])
                fmag = np.sqrt(fx**2 + float(row[18])**2 + float(row[19])**2)
                rows.append((t, eex, eey, eez, fx, float(row[18]), float(row[19]), fmag))
            except (ValueError, IndexError):
                continue
    d = np.array(rows)
    d[:, 0] -= d[0, 0]
    return d


d_old = load(OLD_LOG)
d_new = load(NEW_LOG)


def active_segment(d):
    fmag = d[:, 7]
    idx = np.where(fmag > 0.5)[0]
    if len(idx) < 10:
        return d  # fallback
    return d[idx[0]:idx[-1] + 1]


d1 = active_segment(d_old)
d2 = active_segment(d_new)

fig, axes = plt.subplots(3, 2, figsize=(16, 12),
                          gridspec_kw={"width_ratios": [1, 1]})

# --- Row 1: EE trajectory ---
ax = axes[0, 0]
ax.plot(d1[:, 0], d1[:, 1], "b-", lw=1.2, alpha=0.6, label="ee_x")
ax.plot(d1[:, 0], d1[:, 2], "r-", lw=1.2, alpha=0.6, label="ee_y")
ax.plot(d1[:, 0], d1[:, 3], "g-", lw=1.2, alpha=0.6, label="ee_z")
ax.set_title("v1 (original) — EE Position")
ax.set_ylabel("m"); ax.legend(loc="best", fontsize=7); ax.grid(alpha=0.3)

ax = axes[0, 1]
ax.plot(d2[:, 0], d2[:, 1], "b-", lw=1.2, alpha=0.6, label="ee_x")
ax.plot(d2[:, 0], d2[:, 2], "r-", lw=1.2, alpha=0.6, label="ee_y")
ax.plot(d2[:, 0], d2[:, 3], "g-", lw=1.2, alpha=0.6, label="ee_z")
ax.set_title("v3 (tuned) — EE Position")
ax.set_ylabel("m"); ax.legend(loc="best", fontsize=7); ax.grid(alpha=0.3)

# --- Row 2: Force ---
ax = axes[1, 0]
ax.plot(d1[:, 0], d1[:, 4], "C0-", lw=1, alpha=0.7, label="fx")
ax.plot(d1[:, 0], d1[:, 5], "C1-", lw=1, alpha=0.7, label="fy")
ax.plot(d1[:, 0], d1[:, 6], "C2-", lw=1, alpha=0.7, label="fz")
ax.axhline(100, color="C1", ls="--", lw=0.7, alpha=0.5)
ax.axhline(-100, color="C1", ls="--", lw=0.7, alpha=0.5)
ax.text(d1[0, 0] + 0.5, 105, "fy limit ±100N", fontsize=7, color="C1", alpha=0.6)
fx_sat = np.sum(np.abs(d1[:, 4]) >= 99.9) / len(d1) * 100
fy_sat = np.sum(np.abs(d1[:, 5]) >= 99.9) / len(d1) * 100
ax.set_title(f"v1 (original) — Force  [fx sat={fx_sat:.0f}%, fy sat={fy_sat:.0f}%]")
ax.set_ylabel("N"); ax.legend(loc="best", fontsize=7); ax.grid(alpha=0.3)

ax = axes[1, 1]
ax.plot(d2[:, 0], d2[:, 4], "C0-", lw=1, alpha=0.7, label="fx")
ax.plot(d2[:, 0], d2[:, 5], "C1-", lw=1, alpha=0.7, label="fy")
ax.plot(d2[:, 0], d2[:, 6], "C2-", lw=1, alpha=0.7, label="fz")
ax.axhline(120, color="C0", ls="--", lw=0.7, alpha=0.5)
ax.axhline(150, color="C1", ls="--", lw=0.7, alpha=0.5)
ax.text(d2[0, 0] + 0.5, 155, "fx limit 120N / fy limit 150N", fontsize=7, alpha=0.5)
fx_sat2 = np.sum(np.abs(d2[:, 4]) >= 119.5) / len(d2) * 100
fy_sat2 = np.sum(np.abs(d2[:, 5]) >= 149.5) / len(d2) * 100
ax.set_title(f"v3 (tuned) — Force  [fx sat={fx_sat2:.0f}%, fy sat={fy_sat2:.0f}%]")
ax.set_ylabel("N"); ax.legend(loc="best", fontsize=7); ax.grid(alpha=0.3)

# --- Row 3: Force magnitude |F| + FFT ---
ax = axes[2, 0]
ax.plot(d1[:, 0], d1[:, 7], "k-", lw=1.5, alpha=0.8)
ax.fill_between(d1[:, 0], 0, d1[:, 7], alpha=0.15, color="black")
ax.set_title(f"v1 — |F|  mean={np.mean(d1[:,7]):.0f}N  max={np.max(d1[:,7]):.0f}N  std={np.std(d1[:,7]):.0f}N")
ax.set_xlabel("time (s)"); ax.set_ylabel("|F| (N)"); ax.grid(alpha=0.3)

ax = axes[2, 1]
ax.plot(d2[:, 0], d2[:, 7], "k-", lw=1.5, alpha=0.8)
ax.fill_between(d2[:, 0], 0, d2[:, 7], alpha=0.15, color="black")
ax.set_title(f"v3 — |F|  mean={np.mean(d2[:,7]):.0f}N  max={np.max(d2[:,7]):.0f}N  std={np.std(d2[:,7]):.0f}N")
ax.set_xlabel("time (s)"); ax.set_ylabel("|F| (N)"); ax.grid(alpha=0.3)

# Summary text
improvements = (
    f"Improvements (v1 → v3):\n"
    f"  Mean |F|:  {np.mean(d1[:,7]):.0f} → {np.mean(d2[:,7]):.0f} N  ({-(1-np.mean(d2[:,7])/np.mean(d1[:,7]))*100:.0f}%)\n"
    f"  Max |F|:  {np.max(d1[:,7]):.0f} → {np.max(d2[:,7]):.0f} N\n"
    f"  |F| std:  {np.std(d1[:,7]):.0f} → {np.std(d2[:,7]):.0f} N  (-{-(1-np.std(d2[:,7])/np.std(d1[:,7]))*100:.0f}%)\n"
    f"  fy sat:  {fy_sat:.0f}% → {fy_sat2:.0f}%\n"
    f"  fx sat:  {fx_sat:.0f}% → {fx_sat2:.0f}%"
)
fig.text(0.02, 0.01, improvements, fontfamily="monospace", fontsize=9,
         va="bottom", ha="left",
         bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.8))

plt.suptitle("Virtual Human PID: v1 (original) vs v3 (tuned)", fontsize=13, y=1.01)
plt.tight_layout(rect=[0, 0.08, 1, 0.97])
plt.savefig(OUT, dpi=120, bbox_inches="tight")
print(f"Saved: {OUT}")
