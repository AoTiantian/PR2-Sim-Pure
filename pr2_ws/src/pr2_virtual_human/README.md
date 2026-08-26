# PR2 Virtual Human Transport Comparison

The formal comparison uses one trajectory generator and one virtual-human
six-dimensional endpoint impedance for both experiment conditions. The robot
condition is pure wrench-driven admittance: the robot does not receive the
desired trajectory.

## Run

```bash
# Human only
ros2 launch pr2_virtual_human transport_comparison.launch.py \
  condition:=human_only use_viewer:=false experiment_id:=trial_01

# Human + robot
ros2 launch pr2_virtual_human transport_comparison.launch.py \
  condition:=human_robot robot_mode:=admittance \
  use_viewer:=false experiment_id:=trial_01
```

Both commands read `config/transport_comparison.yaml`. Use the same config file
for both runs. The recorder embeds its SHA-256 hash in each run manifest and the
comparison tool refuses mismatched runs.

```bash
ros2 run pr2_virtual_human compare_transport_runs \
  <human_only_run_dir> <human_robot_run_dir> \
  --output <comparison_output_dir>
```

## Physical contract

- The robot-board grasp is a rigid MuJoCo weld.
- Human and robot endpoint forces retain their natural `r x F` moments.
- Unequal vertical endpoint forces are allowed to rotate the board naturally.
- No `-r x F`, load-share, automatic level-hold, residual payload force, or
  cancellation torque is added.
- Human task torque is permitted only as the output of the shared orientation
  impedance required to execute the roll/pitch/yaw trajectory.
- The public MuJoCo bridge and WBC/QP packages are configured through their
  existing parameters and are not modified for this demo.

The runtime contract guard prevents a robot run from starting if pose tracking,
desired-orientation tracking, automatic residual payload support, endpoint
height hold, or lever-arm moment cancellation is enabled.

The robot uses a pure mass-damper admittance (`K=0` in all six axes). This is
intentional: it avoids embedding a fixed pose target, and it avoids the public
QP's angular displacement/sign mismatch from acting as a negative virtual
spring. Wrist tare changes only the measured admittance reference during
`SETTLE`; it does not change MuJoCo forces, weld reactions, or natural moments.

## Outputs

Runs are stored under:

```text
results/transport_comparison/<experiment_id>/<condition>/<run_id>/
```

Each run contains `history.csv`, `metrics.json`, `run_manifest.json`, and the
exact experiment config. Natural endpoint moment and human task torque are
recorded separately. Robot wrench columns are tare-referenced sensor
diagnostics and never feed the virtual-human controller.

At the end of every individual run the recorder automatically writes
`trajectory_6d.png` and `human_applied_wrench_6d.png` into that same `run_*`
directory. A human-robot run additionally writes
`robot_measured_wrench_6d.png`. These plots describe that run only and do not
require a matching run from the other condition.

The comparison command generates:

- `trajectory_6d.png`: desired versus actual XYZ and rotation-vector XYZ for
  both conditions;
- `human_wrench_6d.png`: applied human Fx/Fy/Fz and task torque X/Y/Z;
- `robot_assistance.png`: tare-referenced measured robot force and torque;
- `board_attitude.png`, `human_effort.png`, and `comparison_metrics.json`.
