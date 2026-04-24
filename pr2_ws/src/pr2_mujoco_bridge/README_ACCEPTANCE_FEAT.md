# PR2 MuJoCo Feat Branch Acceptance

This document is the acceptance entrypoint for the `feat` branch.

## Environment

Run inside the repo devcontainer or any Ubuntu 24.04 + ROS 2 Jazzy environment that matches this workspace layout:

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_mujoco_bridge
source install/setup.bash
```

Validation helper:

```bash
VALIDATOR=/workspace/pr2_ws/src/pr2_mujoco_bridge/scripts/validate_force_response.py
```

All launch files default to `use_viewer:=false` for reproducible headless runs. If your environment has X11 configured, you can append `use_viewer:=true`.

## Zero-Force Stability

Command:

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_arm_force_1d.launch.py \
  use_viewer:=false \
  force_magnitude:=0.0 \
  log_file:=/tmp/arm_zero.csv

python3 $VALIDATOR \
  --csv /tmp/arm_zero.csv \
  --baseline-skip-samples 60 \
  --max-ee-tail-std-mm 0.1
```

Expected:
- The arm may settle during the startup warm-up window.
- After warm-up, the end-effector trace should become effectively flat with no sustained twitching.

Pass rule:
- Validator prints `RESULT: PASS`.

## Single-Arm 1D

Command:

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_arm_force_1d.launch.py \
  use_viewer:=false \
  log_file:=/tmp/arm_1d.csv

python3 $VALIDATOR \
  --csv /tmp/arm_1d.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-tail-std-mm 5
```

Expected:
- The arm yields in the force direction.
- After force removal it settles without high-frequency chatter.

Pass rule:
- Validator prints `RESULT: PASS`.

Optional plot:

```bash
python3 /workspace/pr2_ws/src/pr2_mujoco_bridge/scripts/plot_arm_response.py \
  --csv /tmp/arm_1d.csv \
  --title "PR2 Arm 1D Response" \
  --save /tmp/arm_1d.png
```

## Single-Arm 3D

Use all three checks below. `xyz` uses a frozen simultaneous-force amplitude of `6.0 N` per axis because `10 N` on all three axes at once pushes the current MuJoCo stack beyond the stable acceptance envelope.

### Y Axis

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_axis:=y \
  log_file:=/tmp/arm_y.csv

python3 $VALIDATOR \
  --csv /tmp/arm_y.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-tail-std-mm 5
```

### Z Axis

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_axis:=z \
  log_file:=/tmp/arm_z.csv

python3 $VALIDATOR \
  --csv /tmp/arm_z.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-tail-std-mm 5
```

### Simultaneous XYZ

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_axis:=xyz \
  force_magnitude:=6.0 \
  log_file:=/tmp/arm_xyz.csv

python3 $VALIDATOR \
  --csv /tmp/arm_xyz.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-tail-std-mm 5
```

Expected:
- The arm remains bounded under `y`, `z`, and simultaneous `xyz` forcing.
- No sustained high-frequency oscillation appears in the tail segment.

Pass rule:
- All three validator runs print `RESULT: PASS`.

## Whole-Body Coupling

`feat` is still not a full optimization-based WBC stack. The current branch now uses a stable coordinator architecture:
- Arm-relative Cartesian admittance in `base_link`
- Filtered planar wrench projection from end-effector to base
- Base planar admittance on top of `odom`
- Joint-command plus `cmd_vel` merge in `pr2_wbc_coordinator`

For acceptance, use the frozen whole-body force levels below.

### X Axis

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_whole_body_force.launch.py \
  use_viewer:=false \
  force_axis:=x \
  log_file:=/tmp/wb_x.csv

python3 $VALIDATOR \
  --csv /tmp/wb_x.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-tail-std-mm 4 \
  --min-base-linear-peak-mm 2 \
  --max-base-linear-tail-std-mm 0.05 \
  --max-base-yaw-tail-std-deg 0.05
```

### Simultaneous XYZ

```bash
timeout --signal=INT 14s ros2 launch pr2_mujoco_bridge pr2_whole_body_force.launch.py \
  use_viewer:=false \
  force_axis:=xyz \
  force_magnitude:=4.0 \
  log_file:=/tmp/wb_xyz.csv

python3 $VALIDATOR \
  --csv /tmp/wb_xyz.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-tail-std-mm 3 \
  --min-base-linear-peak-mm 1 \
  --max-base-linear-tail-std-mm 0.05 \
  --max-base-yaw-tail-std-deg 0.05
```

Expected:
- The base moves, but only modestly.
- The arm and base both settle after force removal.
- No persistent arm-base ping-pong remains in the tail segment.

Pass rule:
- Both validator runs print `RESULT: PASS`.

## Logs And Artifacts

- CSV logs are written to `/tmp/*.csv` exactly as shown in the commands above.
- The validator prints the peak displacement, final residual, tail standard deviation, and sign-change count.
- `plot_arm_response.py` can be used for arm plots. Whole-body CSVs also include `base_x`, `base_y`, and `base_yaw` columns for custom inspection.

## Recommended Order

Run the acceptance steps in this order:

1. Environment build
2. Zero-force stability
3. Single-arm 1D
4. Single-arm 3D: `y`, `z`, then `xyz`
5. Whole-body: `x`, then `xyz`
