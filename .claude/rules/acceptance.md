# Acceptance and validation rule

Use `pr2_ws/src/pr2_mujoco_bridge/README_ACCEPTANCE_FEAT.md` as the current feat-branch acceptance entrypoint.

Minimum engineering checks for bridge changes:

```bash
python3 -m pytest pr2_ws/src/pr2_mujoco_bridge/test -q
cd pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_mujoco_bridge --symlink-install
colcon test --packages-select pr2_mujoco_bridge
colcon test-result --verbose --all
```

Acceptance launch files exit automatically after force injection and settle logging complete; do not wrap them in `timeout` by default. Run `bash pr2_ws/src/pr2_mujoco_bridge/scripts/cleanup_pr2_ros_processes.sh` before each acceptance scenario to avoid stale ROS process leakage.
