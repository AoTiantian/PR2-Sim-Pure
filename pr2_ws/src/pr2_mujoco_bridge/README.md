# pr2_mujoco_bridge

ROS 2 Jazzy package for running PR2 MuJoCo simulation with QP whole-body
admittance control.

The maintained entry point is:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py
```

## Runtime Nodes

The launch file starts this minimal stack:

| Executable | Role |
| --- | --- |
| `pr2_mujoco_sim` | MuJoCo stepping, ROS topics, odom, TF, joint states, base and arm actuation |
| `pr2_state_estimator` | Filtered joint and base state |
| `pr2_ee_pose_publisher` | End-effector pose from MuJoCo kinematics |
| `pr2_qp_whole_body_admittance` | 10-DOF QP admittance controller for base and left arm |
| `pr2_wbc_coordinator` | Routes QP references to simulator command topics |
| `pr2_arm_admittance_validator` | Applies validation wrench and terminates the run |
| `pr2_motion_logger` | Logs motion and QP debug data |

## Dependencies

- ROS 2 Jazzy
- Python packages: `mujoco`, `numpy`, `osqp`, `scipy`, `matplotlib`, `glfw`
- ROS packages: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`,
  `nav_msgs`, `tf2_ros`

## Build

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_mujoco_bridge --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py
```

Validation wrench arguments are in the `base_link` frame:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py \
  force_x:=50 force_y:=0 force_z:=0 duration_sec:=6.0
```

The launch default model path is:

```text
/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml
```

Override it when running outside the dev container:

```bash
ros2 launch pr2_mujoco_bridge pr2_qp_whole_body_admittance.launch.py \
  model_path:=/absolute/path/to/unitree_mujoco/unitree_robots/pr2/scene.xml
```
