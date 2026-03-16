# PR2 ROS 2 Workspace

该工作区仅保留 PR2 仿真必要包：

- `pr2_description`
- `pr2_mujoco_hardware`
- `pr2_mobile_controller`
- `pr2_bringup`
- `pr2_mujoco_publisher`

## 编译

```bash
cd pr2_ws
colcon build
source install/setup.bash
```

## 启动

```bash
ros2 launch pr2_bringup pr2_control.launch.py
```
