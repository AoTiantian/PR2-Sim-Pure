#!/bin/bash
# usage: verify_run.sh <tag> [extra launch args...]   -> results/verify_zsupport/<tag>/ + /tmp/v/<tag>.npz
TAG=$1; shift
source /opt/ros/jazzy/setup.bash
source /workspace/pr2_ws/install/setup.bash
mkdir -p /tmp/v /workspace/results/verify_zsupport
cd /workspace/pr2_ws
python3 /workspace/scripts/verify_logger.py /tmp/v/$TAG.npz > /tmp/v/$TAG.logger.log 2>&1 &
LG=$!
ros2 launch pr2_virtual_human transport_comparison.launch.py condition:=human_robot robot_mode:=admittance use_viewer:=false experiment_id:=$TAG output_root:=/workspace/results/verify_zsupport "$@" > /tmp/v/$TAG.launch.log 2>&1
wait $LG
cat /tmp/v/$TAG.logger.log | tail -1
