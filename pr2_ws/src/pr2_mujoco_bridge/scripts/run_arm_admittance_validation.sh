#!/usr/bin/env bash
set -euo pipefail

cd /workspace/pr2_ws
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --packages-select pr2_mujoco_bridge --symlink-install
set +u
source install/setup.bash
set -u

ros2 launch pr2_mujoco_bridge pr2_arm_admittance_validation.launch.py "${@}"

