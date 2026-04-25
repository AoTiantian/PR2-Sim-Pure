# PR2 MuJoCo Feat Branch Acceptance

This document is the acceptance entrypoint for the `feat` branch.

The launch files below exit automatically after force injection and post-release settle logging complete. Do not wrap them with `timeout`.

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
CLEANUP=/workspace/pr2_ws/src/pr2_mujoco_bridge/scripts/cleanup_pr2_ros_processes.sh
```

Engineering-chain checks:

```bash
python3 -m pytest /workspace/pr2_ws/src/pr2_mujoco_bridge/test/test_admittance_core.py \
  /workspace/pr2_ws/src/pr2_mujoco_bridge/test/test_validate_force_response.py -q

colcon test --packages-select pr2_mujoco_bridge
colcon test-result --verbose --all
```

Run the cleanup helper before each acceptance case. This avoids stale ROS processes from one case leaking into the next case:

```bash
bash $CLEANUP
```

## Zero-Force Stability

Command:

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_magnitude:=0.0 \
  log_file:=/tmp/arm_zero.csv

python3 $VALIDATOR \
  --csv /tmp/arm_zero.csv \
  --baseline-skip-samples 60 \
  --max-ee-final-mm 1 \
  --max-ee-tail-std-mm 0.1
```

Expected:
- The arm remains effectively still after the startup warm-up.
- No sustained twitching or self-excited oscillation appears in the tail segment.

Pass rule:
- Validator prints `RESULT: PASS`.

## Single-Arm 1D

Command:

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_arm_force_1d.launch.py \
  use_viewer:=false \
  log_file:=/tmp/arm_1d.csv

python3 $VALIDATOR \
  --csv /tmp/arm_1d.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-final-mm 12 \
  --max-ee-tail-std-mm 5
```

Expected:
- The arm yields along `x`.
- After force removal it settles back without high-frequency chatter.

Pass rule:
- Validator prints `RESULT: PASS`.

## Single-Arm 3D

All three checks below use the stabilized `feat` workpoints. The simultaneous `xyz` case remains frozen at `6.0 N` per axis.

### Y Axis

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_axis:=y \
  log_file:=/tmp/arm_y.csv

python3 $VALIDATOR \
  --csv /tmp/arm_y.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-final-mm 8 \
  --max-ee-tail-std-mm 5
```

### Z Axis

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_axis:=z \
  log_file:=/tmp/arm_z.csv

python3 $VALIDATOR \
  --csv /tmp/arm_z.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-final-mm 8 \
  --max-ee-tail-std-mm 5
```

### Simultaneous XYZ

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py \
  use_viewer:=false \
  force_axis:=xyz \
  force_magnitude:=6.0 \
  log_file:=/tmp/arm_xyz.csv

python3 $VALIDATOR \
  --csv /tmp/arm_xyz.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-final-mm 15 \
  --max-ee-tail-std-mm 5
```

Expected:
- `y`, `z`, and simultaneous `xyz` all remain bounded.
- The tail segment shows no persistent high-frequency oscillation.

Pass rule:
- All three validator runs print `RESULT: PASS`.

## Whole-Body Coupling

`feat` is still a coordinator-style whole-body stack, not an optimization-based WBC solver. Acceptance is therefore based on reproducible closed-loop behavior, not on a QP formulation claim.

### X Axis

This case intentionally accepts a modest base response. The pass thresholds focus on "no arm-base ping-pong and no visible tail motion", not on forcing a large base travel.

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_whole_body_force.launch.py \
  use_viewer:=false \
  force_axis:=x \
  log_file:=/tmp/wb_x.csv

python3 $VALIDATOR \
  --csv /tmp/wb_x.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-final-mm 8 \
  --max-ee-tail-std-mm 4.1 \
  --min-base-linear-peak-mm 1.0 \
  --max-base-linear-tail-std-mm 0.08 \
  --max-base-linear-tail-drift-mm 0.25 \
  --max-base-yaw-tail-std-deg 0.05 \
  --max-base-yaw-tail-drift-deg 0.05
```

### Simultaneous XYZ

```bash
bash $CLEANUP

ros2 launch pr2_mujoco_bridge pr2_whole_body_force.launch.py \
  use_viewer:=false \
  force_axis:=xyz \
  force_magnitude:=4.0 \
  log_file:=/tmp/wb_xyz.csv

python3 $VALIDATOR \
  --csv /tmp/wb_xyz.csv \
  --baseline-skip-samples 60 \
  --min-ee-peak-mm 20 \
  --max-ee-final-mm 15 \
  --max-ee-tail-std-mm 3 \
  --min-base-linear-peak-mm 1 \
  --max-base-linear-tail-std-mm 0.05 \
  --max-base-linear-tail-drift-mm 0.2 \
  --max-base-yaw-tail-std-deg 0.05 \
  --max-base-yaw-tail-drift-deg 0.05
```

Expected:
- The base responds slightly instead of staying completely rigid.
- The arm and base both settle after force removal.
- No persistent arm-base ping-pong remains in the tail segment.

Pass rule:
- Both validator runs print `RESULT: PASS`.

## Logs And Artifacts

- CSV logs are written to the exact `/tmp/*.csv` paths shown above unless you override `log_file:=...`.
- `validate_force_response.py` now checks peak displacement, final residual, tail standard deviation, and tail drift.
- `plot_arm_response.py` can still be used for arm-only plots.

## Recommended Order

Run the acceptance steps in this order:

1. Build
2. `pytest`
3. `colcon test`
4. Zero-force stability
5. Single-arm 1D
6. Single-arm 3D: `y`, `z`, then `xyz`
7. Whole-body: `x`, then `xyz`
