# PR2 QP Whole-Body Admittance

Minimal ROS 2 Jazzy + MuJoCo project for PR2 whole-body admittance control.
The maintained workflow is the QP whole-body launch:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py
```

This repository intentionally excludes old arm-only admittance experiments,
standalone Python control scripts, local logs, build products, and third-party
source checkouts.

## What Is Included

```text
.
├── .devcontainer/                  Optional Docker/dev-container setup
├── pr2_ws/src/pr2_mujoco_bridge/   Main ROS 2 package
│   ├── launch/
│   │   └── pr2_qp_whole_body_admittance.launch.py
│   └── pr2_mujoco_bridge/
│       ├── pr2_sim_ros.py
│       ├── pr2_state_estimator.py
│       ├── pr2_ee_pose_publisher.py
│       ├── pr2_qp_whole_body_admittance.py
│       ├── pr2_wbc_coordinator.py
│       ├── pr2_arm_admittance_validator.py
│       ├── pr2_motion_logger.py
│       ├── pr2_dynamics_utils.py
│       └── ee_trajectory_plot.py
├── unitree_mujoco/unitree_robots/pr2/
│   ├── scene.xml
│   ├── robot_pr2.xml
│   └── meshes/
```

## Dependencies

- ROS 2 Jazzy
- Python packages: `mujoco`, `numpy`, `osqp`, `scipy`, `matplotlib`, `glfw`
- ROS packages: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`,
  `nav_msgs`, `tf2_ros`

The dev container is the recommended environment when available.

## Build

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_mujoco_bridge --symlink-install
source install/setup.bash
```

## Run

Default headless validation run:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py
```

Apply a virtual wrench in the robot `base_link` frame:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py \
  force_x:=50 force_y:=0 force_z:=0 duration_sec:=6.0
```

Enable the MuJoCo viewer when graphics are configured:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py \
  use_viewer:=true
```

When running outside the dev container, pass an absolute model path:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py \
  model_path:=/absolute/path/to/unitree_mujoco/unitree_robots/pr2/scene.xml
```

## Control Stack

`pr2_qp_whole_body_admittance.launch.py` starts:

| Node | Purpose |
| --- | --- |
| `pr2_mujoco_sim` | MuJoCo simulation, odom, TF, joint states, base and arm actuation |
| `pr2_state_estimator` | Filtered state topics |
| `pr2_ee_pose_publisher` | End-effector pose |
| `pr2_qp_whole_body_admittance` | QP whole-body admittance controller |
| `pr2_wbc_coordinator` | Routes base and arm references to simulator commands |
| `pr2_arm_admittance_validator` | Injects validation wrench and ends the run |
| `pr2_motion_logger` | Records CSV logs and trajectory plots |

The QP controller solves a 10-DOF velocity allocation problem: 7 left-arm
joints plus base `vx`, `vy`, and `wz`.

## Ignored Local Content

`.gitignore` excludes:

- ROS build outputs: `build/`, `install/`, `log/`
- Runtime logs and CSV files
- Local IDE state: `.vscode/`, `.cursor/`, `.claude/`
- Local third-party content under `third_party/`
- Auxiliary workspaces such as `cc_ws/` and `src/pr2_ros2_stack/`

## License

See [LICENSE](./LICENSE).
