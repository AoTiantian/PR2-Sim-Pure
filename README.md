# PR2-Sim-Pure-Plus

PR2 robot simulation environment based on **MuJoCo 3.x** and **ROS 2 Jazzy**, with whole-body admittance control via quadratic programming (QP).

Optional **Docker / Dev Container** support with NVIDIA GPU acceleration.

## Key Features

- **MuJoCo ↔ ROS 2 bridge**: `pr2_mujoco_bridge` publishes `joint_states`, `odom`, TF and subscribes to `cmd_vel`, `joint_commands`.
- **QP whole-body admittance control**: 10-DOF QP (7 arm joints + base vx, vy, wz) maps external wrench to compliant Cartesian motion via mass-damper admittance law.
- **GPU passthrough**: NVIDIA hardware acceleration support (RTX series, etc.).
- **One-click environment**: VS Code Dev Container simplifies driver and dependency setup.
- **Omnidirectional base**: Approximate lateral, rotational, and arm torque/position control.
- **Computed torque control (CTC)**: Left arm velocity commands executed via MuJoCo inverse dynamics with configurable PD gains.

## Requirements (Host Machine)

On Ubuntu 24.04 or similar:

1. **GPU driver**: Install NVIDIA drivers. Handle Secure Boot in BIOS if needed for GPU passthrough.
2. **Docker**: Install and configure rootless access.
3. **NVIDIA Container Toolkit**: Required for GPU mapping into containers.
4. **X11 access**: For MuJoCo viewer, allow X11 on host (`xhost +local:root` or equivalent). Headless mode available without X11.

## Quick Start

### 1. Clone

```bash
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
```

### 2. Dev Container (Recommended)

Open the repo root in VS Code. When prompted **"Reopen in Container"**, accept. First build may take a few minutes.

### 3. Run Simulation (ROS 2)

Inside the container or on a machine with ROS 2 Jazzy installed:

```bash
cd pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_mujoco_bridge --symlink-install
source install/setup.bash

# With MuJoCo viewer (requires DISPLAY / X11)
ros2 launch pr2_mujoco_bridge pr2_mujoco_sim.launch.py

# Headless mode (no window, suitable for SSH / CI / Docker without X11)
ros2 launch pr2_mujoco_bridge pr2_mujoco_sim.launch.py use_viewer:=false

# Disable built-in demo motion for pure ROS control
ros2 launch pr2_mujoco_bridge pr2_mujoco_sim.launch.py demo_motion:=false
```

### 4. QP Whole-Body Admittance Control

Launch the full stack (simulation + state estimator + QP controller + WBC coordinator):

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py
```

Apply virtual wrench for validation:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py \
    force_x:=50 force_y:=50 force_z:=50
```

Key launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_viewer` | `false` | Open MuJoCo viewer (requires GPU + X11) |
| `force_x/y/z` | `0.0` | Virtual wrench force in base_link frame (N) |
| `torque_x/y/z` | `0.0` | Virtual wrench torque in base_link frame (Nm) |
| `duration_sec` | `6.0` | Force application duration (s) |
| `force_start_sec` | `3.0` | Delay before force onset (s) |
| `initial_qpos_json` | `'{"l_shoulder_pan_joint": 0.35, ...}'` | Initial arm joint angles (rad) |

### 5. Pure Python Demo (No ROS)

Direct MuJoCo connection without ROS:

```bash
python3 pr2_ws/src/pr2_mujoco_bridge/scripts/pr2_sim.py
```

## Project Structure

```
.
├── .devcontainer/                  Dev Container / Docker configuration
├── pr2_ws/                         Main ROS 2 workspace
│   └── src/pr2_mujoco_bridge/      pr2_mujoco_bridge package
│       ├── launch/                 Launch files
│       ├── pr2_mujoco_bridge/      Python package (import name: pr2_mujoco_bridge)
│       └── scripts/                Standalone scripts (no-ROS demos, run helpers)
├── unitree_mujoco/                 MuJoCo MJCF model files
│   └── unitree_robots/pr2/         PR2 scene and robot definitions
├── third_party/                    External dependency clones (git-ignored)
│   └── README.md                   Clone & setup instructions
└── logs/                           Simulation run logs (git-ignored)
```

### Core Files (Python — Main Development Path)

| File | Role |
|------|------|
| `pr2_mujoco_bridge/pr2_sim_ros.py` | Main simulation loop: MuJoCo stepping, ROS pub/sub, cmd_vel→wheel mapping, CTC |
| `pr2_mujoco_bridge/pr2_qp_whole_body_admittance.py` | QP admittance controller: mass-damper ODE → QP solve → cmd_vel + joint_command |
| `pr2_mujoco_bridge/pr2_wbc_coordinator.py` | WBC coordinator: aggregates reference commands, null-space posture hold |
| `pr2_mujoco_bridge/pr2_state_estimator.py` | State estimator: filters joint states and odometry |
| `pr2_mujoco_bridge/pr2_ee_pose_publisher.py` | Forward kinematics → `ee_pose` topic |
| `pr2_mujoco_bridge/pr2_dynamics_utils.py` | Jacobian computation (6×10 reduced), DOF indexing helpers |
| `pr2_mujoco_bridge/pr2_motion_logger.py` | CSV logger + ODE-vs-actual trajectory plot |
| `pr2_mujoco_bridge/pr2_arm_admittance_validator.py` | Validation: applies virtual wrench, checks displacement metrics |

### Control Architecture

```
External Wrench (base_link)
    │
    ▼
┌─────────────────────────────┐
│ Admittance Law              │
│ M dv/dt + B v = F - K dx   │  ← mass-damper-impedance
│ v_des ∈ R⁶ (twist)          │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│ QP Solver (OSQP)            │
│ min ‖J·u - v_des‖² + ‖u‖²  │  ← 10-DOF: arm(7) + base(vx,vy,wz)
│ s.t. u_min ≤ u ≤ u_max     │
└──────────────┬──────────────┘
               │
       ┌───────┴───────┐
       ▼               ▼
  cmd_vel          joint_command
  (base twist)     (arm velocities)
       │               │
       ▼               ▼
┌──────────┐   ┌──────────────┐
│ Wheel    │   │ CTC (M·q̈ + h)│
│ Mapping  │   │ + PD control │
└────┬─────┘   └──────┬───────┘
     │                │
     ▼                ▼
  MuJoCo Simulation (mj_step)
```

## Tuning the QP Admittance Controller

Key parameters in `pr2_qp_whole_body_admittance.launch.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `damping_linear` | `[320, 320, 400]` | Damping B per axis (Ns/m) |
| `stiffness_linear` | `[0, 0, 0]` | Stiffness K per axis (N/m) — zero = pure force→velocity |
| `mass_linear` | `[5, 5, 5]` | Virtual mass M per axis (kg) |
| `W_ee` | `[1,1,4,1,1,1]` | EE velocity error weights (Z boosted for gravity) |
| `W_reg` | `[2e-3]×10` | Regularization on joint/base velocities |
| `cmd_vel_linear_gain` | `16.0` | Base velocity → wheel speed gain |
| `cmd_vel_world_scale` | `[1.03, 1.12, 1.0]` | Per-axis compensation for anisotropic base tracking |
| `ctc_kp` | `30.0` | Arm CTC position gain |
| `ctc_kd` | `160.0` | Arm CTC velocity gain |

## Documentation

- **Package overview**: `pr2_ws/src/pr2_mujoco_bridge/README.md`
- **WBC stack topics & launch**: `pr2_ws/src/pr2_mujoco_bridge/README_WBC_STACK.md`
- **IK usage**: `pr2_ws/src/pr2_mujoco_bridge/README_IK.md`
- **Third-party repos**: `third_party/README.md`

## Troubleshooting

| Symptom | Solution |
|---------|----------|
| No simulation window | Host: `xhost +local:root`; verify `DISPLAY` and X11 forwarding |
| `Failed to open display` / GLFW error | Use `-p use_viewer:=false` for headless; or fix container graphics |
| MESA / llvmpipe rendering | Container lacks NVIDIA GL libraries. Install `nvidia-driver-libs` or use headless |
| Viewer mode tracks worse than headless | Software rendering (CPU) steals compute from physics. Fix GPU OpenGL passthrough or use headless |
| No GPU in container | Check `nvidia-smi`; verify NVIDIA Container Toolkit and runtime config |
| Slow builds / network | Configure mirror sources in Dockerfile |

## License

MIT — see [LICENSE](./LICENSE).
