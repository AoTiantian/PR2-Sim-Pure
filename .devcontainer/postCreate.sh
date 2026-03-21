#!/usr/bin/env bash
set -euo pipefail

cd /workspaces/PR2-Sim-Pure
source /opt/ros/jazzy/setup.bash

if [ -d "pr2_ws/src" ]; then
  colcon build \
    --base-paths pr2_ws/src \
    --packages-select pr2_description pr2_mobile_controller pr2_bringup \
    --symlink-install
fi

echo "Devcontainer setup done."
echo "Next step:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch pr2_bringup pr2_control_mock.launch.py"
