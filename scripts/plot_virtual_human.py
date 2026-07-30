#!/usr/bin/env python3
"""Plot virtual human tracking performance from recorded CSV data."""
import sys
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) < 3:
    print("usage: plot_virtual_human.py <actual_csv> <force_csv> <output_png>")
    sys.exit(1)

actual_csv, force_csv, out_png = sys.argv[1], sys.argv[2], sys.argv[3]

# --- parse actual hand pose ---
act_t, act_x, act_y, act_z = [], [], [], []
with open(actual_csv) as f:
    # ros2 topic echo --csv outputs with header row + time,p.x,p.y,p.z
    reader = csv.reader(f)
    header = next(reader, None)  # skip header
    for row in reader:
        try:
            t = float(row[0])
            x = float(row[5])   # pose.position.x
            y = float(row[6])   # pose.position.y
            z = float(row[7])   # pose.position.z
            act_t.append(t * 1e-9)  # ros2 csv uses nanoseconds
            act_x.append(x)
            act_y.append(y)
            act_z.append(z)
        except (IndexError, ValueError):
            pass
act_t = np.array(act_t) - act_t[0] if act_t else np.array([])

# --- parse force magnitude ---
frc_t, f_mag = [], []
with open(force_csv) as f:
    reader = csv.reader(f)
    _ = next(reader, None)
    for row in reader:
        try:
            t = float(row[0])
            v = float(row[1])
            frc_t.append(t * 1e-9)
            f_mag.append(v)
        except (IndexError, ValueError):
            pass
frc_t = np.array(frc_t) - frc_t[0] if frc_t else np.array([])

# We don't have a separate desired_hand_pose CSV since both topics
# are the same struct.  Let's compute desired x from the actual trajectory:
# desired starts at act_x[LATCH_end] and moves at 0.05 m/s.
# Since we don't know exact latch time, let's instead look for when
# force becomes non-zero (TRACKING start) to define t0.
#
# For this demo, we capture desired_hand_pose too.
# Let's try reading it if provided as arg 4.
des_t = act_t.copy()  # fallback

# Build figure with two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# --- subplot 1: trajectory ---
if "desired_csv" in sys.argv:
    des_csv = sys.argv[sys.argv.index("desired_csv") + 1]
    des_t2, des_x, des_y, des_z = [], [], [], []
    with open(des_csv) as f:
        reader = csv.reader(f)
        _ = next(reader, None)
        for row in reader:
            try:
                des_t2.append(float(row[0]) * 1e-9)
                des_x.append(float(row[5]))
                des_y.append(float(row[6]))
                des_z.append(float(row[7]))
            except (IndexError, ValueError):
                pass
    des_t2 = np.array(des_t2) - des_t2[0] if des_t2 else np.array([])
    ax1.plot(des_t2, des_x, "b--", linewidth=1.5, label="desired x")
    ax1.plot(des_t2, des_y, "r--", linewidth=1.5, label="desired y")
    ax1.plot(des_t2, des_z, "g--", linewidth=1.5, label="desired z")

ax1.plot(act_t, act_x, "b-", linewidth=2, label="actual x")
ax1.plot(act_t, act_y, "r-", linewidth=2, label="actual y")
ax1.plot(act_t, act_z, "g-", linewidth=2, label="actual z")
ax1.set_ylabel("position (m)")
ax1.set_title("Virtual Human Hand Position Tracking")
ax1.legend(loc="best")
ax1.grid(True, alpha=0.3)

# --- subplot 2: virtual force ---
ax2.plot(frc_t, f_mag, "k-", linewidth=2)
ax2.set_xlabel("time (s)")
ax2.set_ylabel("force magnitude (N)")
ax2.set_title("Virtual Force Magnitude |F|")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(out_png, dpi=120)
print(f"saved to {out_png}")
