# PR2 左臂拟运动学（IK）说明

本文对应节点：`pr2_left_arm_ik`。

## 1. 设计目标

在当前工程中，`pr2_mujoco_sim` 已支持关节/执行器控制，但没有末端笛卡尔目标求解。该节点新增一层“拟运动学 + 力矩控制”能力：

- 输入末端目标位姿（Pose）
- 数值求解关节增量（DLS IK）
- 用 PD 将关节目标转成力矩（effort）
- 通过 `joint_commands` 下发给仿真桥

## 2. 话题接口

- 订阅 `ik_target_pose`（`geometry_msgs/PoseStamped`）
- 订阅 `joint_states`（`sensor_msgs/JointState`）
- 发布 `joint_commands`（`sensor_msgs/JointState`，仅 `effort` 字段）

## 3. 算法简述

每个控制周期：

1. 从 `joint_states` 回填 MuJoCo 状态，调用 `mj_forward`
2. 读取末端当前位姿（默认 body：`l_gripper_tool_frame`）
3. 计算误差 `e = [e_pos, e_ori]`
4. 计算雅可比 `J`（`mj_jacBody`）
5. 阻尼最小二乘：

\[
\Delta q = J^T (J J^T + \lambda^2 I)^{-1} e
\]

6. 关节限幅后得到 `q_next`
7. 关节 PD -> 力矩：`tau = Kp * (q_next - q_cur) - Kd * qdot`

## 4. 默认控制关节

- `l_shoulder_pan_joint`
- `l_shoulder_lift_joint`
- `l_upper_arm_roll_joint`
- `l_elbow_flex_joint`
- `l_forearm_roll_joint`
- `l_wrist_flex_joint`
- `l_wrist_roll_joint`

## 5. 运行方法

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_controller
source install/setup.bash
```

终端 1：启动仿真（建议关 demo）

```bash
ros2 run pr2_controller pr2_mujoco_sim --ros-args -p demo_motion:=false
```

终端 2：启动 IK 节点

```bash
ros2 run pr2_controller pr2_left_arm_ik
# 或
ros2 launch pr2_controller pr2_left_arm_ik.launch.py
```

终端 3：发布目标位姿

```bash
ros2 topic pub /ik_target_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'world'
pose:
  position: {x: -2.6, y: 2.2, z: 1.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

## 6. 参数建议

- `damping_lambda`: 0.05 ~ 0.15
- `step_size`: 0.2 ~ 0.5
- `max_step_rad`: 0.03 ~ 0.08
- `torque_kp`: 80 ~ 220
- `torque_kd`: 8 ~ 35
- `use_orientation`: false（先调位置），稳定后再 true

## 7. 常见问题

1. **机械臂抖动**：增大 `damping_lambda`，减小 `step_size` 或 `torque_kp`。
2. **跟踪慢**：适当增大 `step_size` 或 `torque_kp`，但注意振荡。
3. **不动**：确认 `pr2_mujoco_sim` 的 `demo_motion=false`，并检查 `ik_target_pose` 是否持续发布。
4. **姿态误差大**：先设置 `use_orientation=false`，仅位置收敛后再开启姿态。


## 8. 监测末端执行器位置（新增）

已新增节点：`pr2_ee_pose_publisher`，用于实时发布末端位姿。

- 订阅：`joint_states`
- 发布：`ee_pose`（`geometry_msgs/PoseStamped`）
- 默认末端：`l_gripper_tool_frame`

启动：

```bash
ros2 run pr2_controller pr2_ee_pose_publisher
# 或
ros2 launch pr2_controller pr2_ee_pose.launch.py
```

查看末端位姿：

```bash
ros2 topic echo /ee_pose
```

如果你使用右臂或其他末端，可改参数：

```bash
ros2 run pr2_controller pr2_ee_pose_publisher --ros-args -p end_effector_body:=l_gripper_tool_frame
```
