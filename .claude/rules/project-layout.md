# Project layout rule

This repository intentionally has two ROS 2 workspace-style areas:

1. `pr2_ws/src/pr2_mujoco_bridge/` — the active ament_python MuJoCo ↔ ROS 2 bridge package.
2. `src/pr2_ros2_stack/` — a ROS 2 C++ stack containing description, bringup, ros2_control hardware/controller packages, and a TF publisher.

Do not merge, flatten, or relocate these directories automatically. Add new bridge nodes/tests under `pr2_ws/src/pr2_mujoco_bridge/`; add PR2 stack/controller/hardware work under `src/pr2_ros2_stack/` only when that is the actual target.

Root-level generated build artifacts (`build`, `install`, `log`) and virtual environments must not be committed. ROS package markers should not be added at repo root.
