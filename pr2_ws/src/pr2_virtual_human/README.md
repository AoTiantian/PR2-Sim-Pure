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

## Angular impedance and physics stepping

The current config explicitly selects `human_impedance.angular_frame: body`.
Angular stiffness and damping act on the measured board axes, so the low-inertia
long axis keeps its X gains throughout a yaw turn. The orientation and angular
velocity errors are transformed into that frame, the impedance torque is
transformed back to the world frame, and the existing world-component torque
limits are applied. Linear impedance and the natural endpoint moment are
unchanged. This shared human law is used by both experiment conditions.

`angular_frame: world` reproduces the earlier angular law; configuration files
that omit the field retain that legacy behavior. Use a matching config and
record its frame choice when comparing runs. The changed frame defines a new
human-model condition; earlier world-frame results are separate baselines.

The human-only adapter uses `mj_step1`, evaluates feedback on the current pose
and velocity, then calls `mj_step2`. This avoids reading derived MuJoCo state
from the preceding physical step. The timestep and the board model are unchanged.
S12 validation and the original file snapshots are retained under
`results/validated_humanonly_20261026/`.

## Physical contract

- The robot-board grasp uses finite fingertip contact friction. The board has
  a free joint and no weld/connect to the robot. Both fingertip collision pads
  are enabled; the gripper closes with its position actuator.
- Human and robot endpoint forces retain their natural `r x F` moments.
- Unequal vertical endpoint forces are allowed to rotate the board naturally.
- No `-r x F`, load-share, automatic level-hold, residual payload force, or
  cancellation torque is added.
- Human task torque is permitted only as the output of the shared orientation
  impedance required to execute the roll/pitch/yaw trajectory.
- The WBC/QP uses its existing parameters. The bridge rejects an active
  board weld/connect when `require_friction_grasp` is enabled by this launch.

The runtime contract guard prevents a robot run from starting if pose tracking,
desired-orientation tracking, automatic residual payload support, endpoint
height hold, or lever-arm moment cancellation is enabled.

The robot uses a pure mass-damper admittance (`K=0` in all six axes). This is
intentional: it avoids embedding a fixed pose target, and it avoids the public
QP's angular displacement/sign mismatch from acting as a negative virtual
spring. Wrist tare changes only the measured admittance reference during
`SETTLE`; it does not change MuJoCo forces, contact reactions, or natural moments.

## Contact friction grasp

The production scene uses symmetric fingertip pads with the existing sliding
friction coefficient 2.5 and `condim=4`. The initial arm pose aligns the fingers
with the board. The board starts 60 mm inside the fingertips, leaving contact
patch clearance during relative yaw; the earlier 15 mm insertion allowed
loaded contacts to reach the board end. Finger coordinates start at 0.45 rad and the finite closing
command is 0.30 rad. The existing gripper actuator gains are unchanged. Only
initial joint coordinates are set; the board pose is never constrained to follow
the gripper during simulation. A dropped board can contact the ground.

The contact scene uses the original 0.002 s physics step, `implicitfast`, an elliptic
friction cone, `impratio=10`, and tolerance `1e-10`. These are explicit numerical
settings for the friction model; the human-only scene retains its own timestep.
The recorder reads the actual model timestep instead of writing a fixed 0.002 s.
Do not interpret condition comparisons as using an identical physics timestep
without checking the manifests.

The measured `mujoco/board_grasped` flag requires positive normal contact force
at both pads. Debug traces (`PR2_DEBUG_TRACE=1`) include per-pad contact forces
and the board position in the tool frame. Tests and friction-model runs are
retained in `results/friction_grasp_20261026/`. The shallow-grasp S12 failures
were subsequently addressed by increasing insertion from 15 to 60 mm; see
`results/friction_diagnosis_20261026/REPORT.md` for repeated full-system
validation and contact-position evidence. The former weld tracking results
remain a separate model condition. Smaller timestep and feedback timestamp
trials did not reliably remove the shallow-grasp failure and were reverted.

The S12 damping values in the config are inherited from the earlier weld
experiment and are a starting point for the friction model, not validated
optimal friction-grasp parameters. The legacy weld sensor compatibility helper
is inactive here because the board has no equality attachment.

## Archived weld-model S12 validation

The following results describe the previous weld grasp, not the current
contact-friction model. They are retained for provenance only.


MuJoCo 3.10.0 reconstructs weld rotational constraint multipliers incorrectly
in its force/torque sensor path. The bridge backports the physical torque
conversion from [upstream ac329bd](https://github.com/google-deepmind/mujoco/commit/ac329bd)
for the board weld before wrist tare. This changes the published sensor torque,
preserves actual lever-arm moments, and does not change the physics solver.
The compatibility correction activates only on the locally verified 3.10.0
release; other versions use their native sensor path.

For the 22 s S12 turn, the preset uses linear damping `[20, 20, 120]` and angular
damping `[80, 80, 20]`. This is scenario-specific controller tuning, with positive
mass and damping and zero stiffness. The robot still receives only measured
wrist wrench. Other presets retain the nominal damping. Neither the human
trajectory nor a desired board orientation is supplied to the robot.

Three independent runs reduced peak roll from 40–43 degrees to 1.7–4.4 degrees
and final yaw error from 13–15 degrees to 2.1–3.6 degrees. Transient attitude
error remains 12–15 degrees; this is improved tracking, not exact tracking or
evidence of reduced human effort. The final viewer run and the tested/reverted
alternatives are documented in `results/validated_coop_20261026/REPORT.md`.

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
