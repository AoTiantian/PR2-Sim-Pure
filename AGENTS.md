# PR2-Sim-Pure AI Agent Instructions

This repository is Haohua's active PR2 robotics simulation project. These rules apply to Codex CLI, Claude Code, and any other AI coding agent working in this repo.

## Project identity

- Project: `PR2-Sim-Pure` / `PR2-Sim-Pure-Plus`.
- Target platform: Ubuntu 24.04 + ROS 2 Jazzy.
- Domain: PR2 robot simulation, MuJoCo 3.x, ROS 2 bridge, arm/base admittance experiments, practical whole-body coordination, and PR2 ROS 2 stack migration.
- Primary workspace for the current feature branch: `pr2_ws/` with package `pr2_ws/src/pr2_mujoco_bridge`.
- Secondary ROS 2 stack: `src/pr2_ros2_stack/` containing C++/ament_cmake packages (`pr2_description`, `pr2_bringup`, `pr2_mujoco_hardware`, `pr2_mobile_controller`, `pr2_mujoco_publisher`).
- MuJoCo PR2 model assets: `unitree_mujoco/unitree_robots/pr2/`.

## Repository layout rules

Do not flatten or relocate the existing workspaces without an explicit design decision.

```text
README.md                                      # top-level project overview
.devcontainer/                                # VS Code dev container for ROS 2 + MuJoCo
pr2_ws/src/pr2_mujoco_bridge/                 # active ament_python bridge package
pr2_ws/src/pr2_mujoco_bridge/pr2_mujoco_bridge/ # Python nodes and control logic
pr2_ws/src/pr2_mujoco_bridge/launch/          # launch files for simulation/admittance/acceptance
pr2_ws/src/pr2_mujoco_bridge/scripts/         # standalone helpers and validators
pr2_ws/src/pr2_mujoco_bridge/test/            # pytest/colcon tests
src/pr2_ros2_stack/                           # C++ ROS 2 stack packages
unitree_mujoco/unitree_robots/pr2/            # MJCF scene/model and meshes
```

ROS packages must remain below a workspace `src/` directory. Do not add root-level `package.xml`, `setup.py`, `setup.cfg`, or `CMakeLists.txt` unless the repository is intentionally restructured.

Generated outputs such as `build/`, `install/`, `log/`, `.pytest_cache/`, `__pycache__/`, and local virtualenvs must not be committed.

## Development environment

- Use ROS 2 Jazzy. Source ROS setup before ROS commands:
  - `source /opt/ros/jazzy/setup.bash`
- When using strict shell scripts, do not source ROS under `set -u`; temporarily `set +u` first.
- Python runtime dependency `mujoco` is not normally provided by rosdep; install it in a venv/devcontainer, not into system Python unless explicitly requested.
- Prefer the repo devcontainer or an isolated Python environment for MuJoCo/Python dependencies.
- Do not commit secrets, API keys, Wi-Fi credentials, robot credentials, host-specific tokens, or connection strings. Replace any discovered secret-like value with `[REDACTED]` in docs/examples.

## Build and test commands

Use the least risky validation needed for the change.

### Active bridge package

```bash
cd pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_mujoco_bridge --symlink-install
source install/setup.bash
python3 -m pytest src/pr2_mujoco_bridge/test -q
colcon test --packages-select pr2_mujoco_bridge
colcon test-result --verbose --all
```

### Acceptance checks for the feat branch

Use `pr2_ws/src/pr2_mujoco_bridge/README_ACCEPTANCE_FEAT.md` as the acceptance entrypoint. It contains the current force/admittance scenarios and pass thresholds.

Important: the acceptance launch files are designed to exit automatically after logging and settling. Do not wrap them with `timeout` unless there is a new bug that causes them to hang.

Before each acceptance case, run:

```bash
bash pr2_ws/src/pr2_mujoco_bridge/scripts/cleanup_pr2_ros_processes.sh
```

Prefer headless simulation for automated validation:

```bash
ros2 launch pr2_mujoco_bridge pr2_arm_force_3d.launch.py use_viewer:=false force_magnitude:=0.0 log_file:=/tmp/arm_zero.csv
python3 pr2_ws/src/pr2_mujoco_bridge/scripts/validate_force_response.py --csv /tmp/arm_zero.csv --baseline-skip-samples 60 --max-ee-final-mm 1 --max-ee-tail-std-mm 0.1
```

### Secondary C++ stack

For changes under `src/pr2_ros2_stack/`, build from a workspace root that has those packages under `src/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

If the current checkout layout prevents building both workspaces together cleanly, explain the limitation and build only the affected package/workspace.

## PR2/MuJoCo control safety rules

This repository is simulation-first, but treat robot-control code as safety-sensitive.

- Default to simulation, headless MuJoCo, dry-run validation, RViz, rosbag replay, and unit tests before proposing real hardware actions.
- Do not add commands that drive real motors, GPIO, CAN, EtherCAT, serial devices, batteries, brakes, grippers, or physical PR2 hardware unless the user explicitly asks and a safe bench-test plan is documented.
- Keep `demo_motion:=false` when testing external control nodes such as IK, admittance, or WBC so controllers do not fight the built-in demo motion.
- Respect actuator dimensions: `actuator_command` must match `model.nu` exactly; partial commands should use `joint_commands` or higher-level topics.
- Preserve timeout/freshness guards in coordinator/control nodes. Stale commands must decay to safe zero/idle outputs.
- Be careful with gains, force magnitudes, stiffness/damping, nullspace gains, and ctrl ranges. Explain stability implications in code review notes.
- Prefer bounded, clipped, deadbanded commands and deterministic validators for admittance/force-response changes.

## Coding conventions

### Python / ROS 2 nodes

- Keep ROS node entry points in `pr2_ws/src/pr2_mujoco_bridge/setup.py` synchronized with modules under `pr2_mujoco_bridge/`.
- Use explicit ROS parameters with safe defaults and document any new topics/parameters in package README files.
- Keep pure math/control helpers testable without ROS when practical; add pytest coverage for validators, admittance math, safety clipping, and parsing logic.
- Avoid broad exception swallowing in control loops unless a warning is emitted and safe output is maintained.

### C++ / ament_cmake packages

- Follow ROS 2 Jazzy conventions: `ament_cmake`, `rclcpp`, `pluginlib`, `controller_interface`, `hardware_interface` as appropriate.
- Keep plugin XML, package exports, and `CMakeLists.txt` synchronized.
- For controller/hardware changes, validate lifecycle transitions and parameter handling.

### Launch/config/docs

- Keep launch arguments typed correctly. In ROS 2 launch Python, use `ParameterValue(..., value_type=str)` where YAML coercion could turn strings like `y`/`n` into booleans.
- New launch files should support `use_viewer:=false` for CI/headless testing when they start MuJoCo.
- Update `README.md`, package READMEs, or `README_ACCEPTANCE_FEAT.md` when commands, topics, acceptance thresholds, or launch arguments change.

## Git and collaboration

- Before editing, check `git status --short --branch` and avoid overwriting user changes.
- Keep changes scoped and explain which workspace/package is affected.
- Do not run destructive cleanup commands such as `git clean -fdx`, reset, rebase, or force-push unless explicitly requested.
- Do not reboot/logout/shutdown the workstation or restart `hermes-gateway.service`.
- Before restarting Docker or long-running robotics services, verify whether doing so could kill the current agent/session and ask for confirmation.

## Completion checklist for AI agents

Before reporting completion, run the applicable subset:

```bash
git status --short --branch
bash scripts/verify_ai_project.sh
bash -n scripts/verify_ai_project.sh scripts/ai-codex.sh scripts/ai-claude.sh
python3 -m pytest pr2_ws/src/pr2_mujoco_bridge/test -q
```

If ROS 2 and MuJoCo are available and the change touches runtime behavior, also run the relevant `colcon build`, `colcon test`, and acceptance scenario from `README_ACCEPTANCE_FEAT.md`. If a check cannot be run because dependencies are missing or it would take too long, state that explicitly.
