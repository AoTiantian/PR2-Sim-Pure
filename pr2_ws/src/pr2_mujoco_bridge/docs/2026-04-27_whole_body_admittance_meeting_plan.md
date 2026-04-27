# Whole-Body Admittance Follow-up Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Turn the 2026-04-27 meeting feedback into a concrete optimization path for PR2 whole-body force/admittance behavior: larger and more interpretable motion, verified single-controller arm/base coordination, improved return-to-neutral performance, and force-tracking references instead of fixed-point goals.

**Architecture:** Keep the current separation between acceptance behavior and presentation/demo behavior, but audit the lower-level MuJoCo XML actuator/contact configuration before changing gains. Treat `pr2_wbc_coordinator` as the single execution-layer coordinator: arm and base references should be produced by force/admittance modules, then merged and bounded in one controller path. Replace any fixed-target reference behavior with a force-tracking reference generator that follows the measured wrench while retaining safe decay and return behavior.

**Tech Stack:** ROS 2 Jazzy, MuJoCo MJCF/XML, Python `rclpy` nodes in `pr2_mujoco_bridge`, pytest, colcon, CSV-based validators, Matplotlib visual diagnostics, Git branches `feat` and `feat_python`.

---

## Meeting Notes Captured

- 当前机器人动作幅度仍偏小，需要审查底层 XML 中可能限制受力响应的配置，并进行优化。
- 需要验证机械臂与底座是否已经统一由单一控制器驱动；如果尚未完全统一，需要收敛到单控制器路径。
- 需要调整阻尼、刚度、回正/回零相关参数，以改善撤力后的回正性能。
- 参考值设定需要从固定定点策略改为“力追踪”策略，即参考目标随外力输入动态生成，而不是追向一个固定目标点。
- 曲线图需要进一步优化，使其更直观暴露问题：动作幅度小、回正慢、耦合明显、控制命令饱和或振荡等。
- 后续需要合并 `feat` 与 `feat_python` 分支；合并前应完成差异审查、冲突预判和验证计划。

## Current Repository Observations

- 当前本地分支：`feat`，远程存在 `origin/feat` 与 `origin/feat_python`。
- PR2 MJCF 文件位于：
  - `unitree_mujoco/unitree_robots/pr2/robot_pr2.xml`
  - `unitree_mujoco/unitree_robots/pr2/scene.xml`
- 底层 XML 中值得优先审查的字段包括：
  - caster/wheel joint 的 `actuatorfrcrange`、`damping`；
  - arm/torso/head/gripper joint 的 `actuatorfrcrange`、`damping`、`range`；
  - actuator section 中的 `gear`、`ctrlrange`、`forcerange`、`gainprm`、`biastype`；
  - contact/friction 参数是否导致底座响应被过度抑制。
- 当前 Python 控制节点中与会议问题直接相关的文件包括：
  - `pr2_mujoco_bridge/admittance_core.py`
  - `pr2_mujoco_bridge/pr2_arm_admittance.py`
  - `pr2_mujoco_bridge/pr2_base_admittance.py`
  - `pr2_mujoco_bridge/pr2_force_projector.py`
  - `pr2_mujoco_bridge/pr2_wbc_coordinator.py`
  - `pr2_mujoco_bridge/pr2_sim_ros.py`
  - `launch/pr2_whole_body_force.launch.py`
  - `launch/pr2_whole_body_force_amplified.launch.py`
  - `scripts/validate_force_response.py`
  - `scripts/plot_arm_response.py`
  - `scripts/plot_whole_body_response.py`
  - `scripts/generate_pr2_admittance_visuals.py`

---

## Task 1: Audit MuJoCo XML force/actuator limits

**Objective:** Determine whether low motion amplitude is caused or worsened by MJCF actuator force ranges, damping, contact/friction, or gear scaling before tuning ROS-level gains.

**Files:**
- Inspect: `unitree_mujoco/unitree_robots/pr2/robot_pr2.xml`
- Inspect: `unitree_mujoco/unitree_robots/pr2/scene.xml`
- Create: `pr2_ws/src/pr2_mujoco_bridge/docs/xml_force_limit_audit.md`
- Optional test/helper: `pr2_ws/src/pr2_mujoco_bridge/scripts/audit_pr2_mjcf_limits.py`

**Steps:**
1. Extract all joint and actuator entries with `actuatorfrcrange`, `damping`, `frictionloss`, `gear`, `ctrlrange`, `forcerange`, and contact/friction attributes.
2. Group the results by subsystem: base casters/wheels, torso, left arm, right arm, grippers.
3. Identify likely bottlenecks:
   - wheel/caster force limits too small for whole-body demo;
   - joint damping values too high for admittance motion;
   - actuator gear/ctrlrange values clipping commands;
   - contact/friction settings suppressing base translation.
4. Do **not** change XML values yet; first produce the audit document with proposed candidates and risk notes.

**Verification:**

```bash
python3 pr2_ws/src/pr2_mujoco_bridge/scripts/audit_pr2_mjcf_limits.py \
  --model unitree_mujoco/unitree_robots/pr2/robot_pr2.xml \
  --out /tmp/pr2_mjcf_limit_audit.json
```

Expected: JSON/report lists all limit-like fields and identifies base/arm bottleneck candidates.

---

## Task 2: Verify single-controller arm/base execution path

**Objective:** Prove whether the arm and base are already unified under one controller path, and close any gaps if commands can still bypass the coordinator.

**Files:**
- Inspect/modify: `pr2_mujoco_bridge/pr2_wbc_coordinator.py`
- Inspect/modify: `pr2_mujoco_bridge/pr2_arm_admittance.py`
- Inspect/modify: `pr2_mujoco_bridge/pr2_base_admittance.py`
- Inspect/modify: `pr2_mujoco_bridge/pr2_sim_ros.py`
- Test: `test/test_wbc_single_controller.py` or extend existing tests.

**Acceptance Criteria:**
- Mechanical arm torque/joint commands and base velocity/reference commands are merged by `pr2_wbc_coordinator` or one clearly documented equivalent controller path.
- No launch used for whole-body admittance allows arm/base controllers to fight `demo_motion` or independent direct command paths.
- Stale command guards remain active; missing arm/base reference decays safely.

**Verification:**

```bash
PYTHONPATH=pr2_ws/src/pr2_mujoco_bridge:${PYTHONPATH:-} \
python3 -m pytest pr2_ws/src/pr2_mujoco_bridge/test/test_wbc_single_controller.py -q
```

Expected: tests fail before any missing unification fix, then pass after implementation.

---

## Task 3: Improve return-to-neutral tuning for admittance states

**Objective:** Tune damping, stiffness, return gains, idle velocity decay, and settle thresholds so force-off recovery is faster and less oscillatory without hiding real residual error.

**Files:**
- Modify: `pr2_mujoco_bridge/admittance_core.py` only if the settle model itself needs changes.
- Modify: `pr2_mujoco_bridge/pr2_arm_admittance.py` parameter defaults if needed.
- Modify: `pr2_mujoco_bridge/pr2_base_admittance.py` parameter defaults if needed.
- Modify: `launch/pr2_whole_body_force.launch.py` for acceptance defaults.
- Modify: `launch/pr2_whole_body_force_amplified.launch.py` for presentation defaults.
- Test: `test/test_admittance_core.py`
- Test: `test/test_validate_force_response.py`

**Tuning Approach:**
1. Establish baseline metrics from existing CSV runs:
   - peak displacement;
   - final residual;
   - tail standard deviation;
   - tail range;
   - return time after force-off;
   - command norm and torque saturation.
2. Tune return behavior separately from force-on compliance:
   - force-on: sufficient amplitude;
   - force-off: faster decay, lower tail oscillation;
   - no artificial snapping that invalidates physical response.
3. Keep acceptance and demo values separate.

**Verification:**

```bash
set +u; source /opt/ros/jazzy/setup.bash; source pr2_ws/install/setup.bash; set -u
python3 -m pytest pr2_ws/src/pr2_mujoco_bridge/test/test_admittance_core.py \
  pr2_ws/src/pr2_mujoco_bridge/test/test_validate_force_response.py -q
```

Expected: unit tests pass, and acceptance CSV metrics show lower final residual/tail variance than the current baseline.

---

## Task 4: Replace fixed-point reference strategy with force-tracking reference generation

**Objective:** Implement the force-tracking strategy suggested in the meeting: reference motion should be generated continuously from the measured external force rather than tracking a fixed target point.

**Files:**
- Modify: `pr2_mujoco_bridge/admittance_core.py`
- Modify: `pr2_mujoco_bridge/pr2_arm_admittance.py`
- Modify: `pr2_mujoco_bridge/pr2_base_admittance.py`
- Modify: launch files to expose mode selection if both strategies must coexist temporarily.
- Test: add pure-function tests in `test/test_admittance_core.py`.

**Design Notes:**
- Add an explicit mode name such as `reference_mode:=force_tracking` vs `reference_mode:=fixed_equilibrium`.
- In force-tracking mode:
  - external wrench drives a bounded dynamic reference through admittance state;
  - reference velocity is clipped and smoothed;
  - when force falls below deadband, reference decays according to tuned return dynamics;
  - no hard-coded fixed target should dominate during force-on.
- Preserve safety limits: displacement caps, velocity caps, joint limit guards, command freshness guards.

**Verification:**
- Unit tests should show that sustained force causes reference drift/compliance in force direction.
- Force-off tests should show bounded return and no runaway reference.
- CSV should include both `force_*` and `reference/admittance` columns so plots prove force tracking.

---

## Task 5: Improve diagnostic plots so they expose problems directly

**Objective:** Make generated curves visually answer the meeting questions: “why is motion small?”, “where is the response clipped?”, “how fast does it return?”, and “are arm and base coordinated?”.

**Files:**
- Modify: `scripts/generate_pr2_admittance_visuals.py`
- Modify: `scripts/plot_arm_response.py`
- Modify: `scripts/plot_whole_body_response.py`
- Optional: add `scripts/compare_force_response_runs.py`

**Plot Enhancements:**
- Add baseline vs tuned overlay plots.
- Add force-on / force-off shaded windows.
- Add peak/final/tail metrics in plot annotations.
- Add return-time metric after force release.
- Add command saturation indicators: `qdot_cmd_norm`, `tau_norm`, `tau_max_abs`, and base velocity clamp usage.
- Add desired-vs-actual displacement error and cross-axis coupling score.
- Add whole-body coordination panel: arm EE displacement vs base XY/yaw response under the same wrench.

**Verification:**

```bash
python3 pr2_ws/src/pr2_mujoco_bridge/scripts/generate_pr2_admittance_visuals.py \
  --csv /tmp/current_run.csv \
  --outdir /tmp/pr2_response_visuals \
  --baseline-skip-samples 60
```

Expected: plots make small amplitude, poor return, saturation, or coupling obvious without manual CSV inspection.

---

## Task 6: Plan and execute `feat` / `feat_python` branch merge

**Objective:** Merge `feat` and `feat_python` only after understanding code/data divergence and preserving validated demo/acceptance behavior.

**Files:**
- Inspect: all diffs between `origin/feat` and `origin/feat_python`
- Create: `pr2_ws/src/pr2_mujoco_bridge/docs/feat_feat_python_merge_notes.md`

**Steps:**
1. Fetch both branches:

```bash
git fetch origin feat feat_python
```

2. Inspect divergence:

```bash
git log --oneline --decorate --graph origin/feat..origin/feat_python
git log --oneline --decorate --graph origin/feat_python..origin/feat
git diff --stat origin/feat...origin/feat_python
git diff --name-status origin/feat...origin/feat_python
```

3. Create a temporary integration branch instead of merging directly into `feat`:

```bash
git checkout feat
git checkout -b integration/feat-python-admittance
```

4. Merge and resolve conflicts:

```bash
git merge origin/feat_python
```

5. Run full validation before pushing integration branch:

```bash
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd pr2_ws
colcon build --packages-select pr2_mujoco_bridge --symlink-install
cd ..
PYTHONPATH=pr2_ws/src/pr2_mujoco_bridge:${PYTHONPATH:-} \
python3 -m pytest pr2_ws/src/pr2_mujoco_bridge/test -q
```

6. Only after validation, fast-forward/merge back to `feat` or open a PR, depending on repository workflow.

**Do not:** force-push, reset, or overwrite either branch without explicit approval.

---

## Proposed Execution Order

1. XML force/actuator limit audit.
2. Single-controller path verification.
3. Baseline acceptance/demo run with current `feat`.
4. Return-to-neutral parameter tuning.
5. Force-tracking reference implementation.
6. Plot/diagnostic improvements.
7. Re-run acceptance and obvious-motion demo.
8. Prepare `feat` / `feat_python` integration branch and merge validation.

## Definition of Done

- XML audit document identifies whether model-level force/damping limits contribute to low motion amplitude.
- Tests confirm single-controller arm/base execution path.
- Force-tracking reference mode is implemented and documented.
- Return metrics improve versus baseline without unsafe clipping or hidden snap-to-zero behavior.
- Plots clearly show force input, force-tracking reference, actual motion, error/coupling, command saturation, and tail settling.
- `feat` and `feat_python` are merged through a validated integration branch or PR.
- Final demo includes CSV, plots, and video evidence from recorded `/joint_states`, `/odom`, and wrench data.
