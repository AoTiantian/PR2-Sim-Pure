# ROS 2 Jazzy rule

Target ROS distribution is Jazzy on Ubuntu 24.04.

Use Jazzy-compatible packages, APIs, launch conventions, and dependency names. Source `/opt/ros/jazzy/setup.bash` before ROS commands. For shell scripts with `set -u`, use `set +u` while sourcing ROS setup files.

For Python package `pr2_mujoco_bridge`, keep `setup.py`, `package.xml`, launch files, console scripts, and README commands synchronized. For C++ packages, keep `CMakeLists.txt`, plugin XML, package exports, and headers synchronized.

When string launch parameters could be parsed as YAML booleans (for example `y`, `n`, `x`, `xyz` in force-axis workflows), use explicit launch parameter typing such as `ParameterValue(..., value_type=str)`.
