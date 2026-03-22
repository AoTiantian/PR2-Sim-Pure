"""
直连 MuJoCo 的演示（无 ROS）：在 Python 里直接写 data.ctrl。

若要通过 ROS 2 Jazzy 控制同一模型，请使用已编译包中的节点：
  ros2 run pr2_controller pr2_mujoco_sim
  ros2 launch pr2_controller pr2_mujoco_sim.launch.py
详见包内 README.md（pr2_controller）。
"""
import mujoco
import mujoco.viewer
import time
import numpy as np
import os

# 1. 模型加载
model_path = "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# --- 2. 执行器 ID 定义 ---
GRIPPER_L_ID = 1
STEER_IDS = [5, 6, 7, 8]
WHEEL_IDS = [9, 10, 11, 12, 13, 14, 15, 16]
TORSO_ID = 17

# 手臂测试配置
ARM_JOINTS_TO_TEST = [
    (25, "左肩 - 水平摆动"),
    (26, "左肩 - 垂直抬举"),
    (28, "左肘 - 弯曲"),
    (30, "左腕 - 上下摆动")
]

# 自动检测夹爪限位
test_max = model.actuator_ctrlrange[GRIPPER_L_ID][1] if model.actuator_ctrlrange[GRIPPER_L_ID][1] > 0 else 0.5

# --- 3. 核心控制逻辑 ---
def apply_advanced_separated_control(model, data, t):
    # a. 初始化控制数组
    data.ctrl[:] = 0.0
    
    # b. 维持躯干高度
    data.ctrl[TORSO_ID] = 500.0

    # c. 夹爪控制
    data.ctrl[GRIPPER_L_ID] = ((np.sin(2.0 * t) + 1) / 2.0) * test_max

    # d. 手臂：每 3 秒切换一个关节
    arm_step = int(t / 3) % len(ARM_JOINTS_TO_TEST)
    target_joint_id, joint_name = ARM_JOINTS_TO_TEST[arm_step]
    
    base_torque = -45.0 if target_joint_id == 26 else 0.0
    amplitude = 40.0 if target_joint_id != 28 else 20.0
    data.ctrl[target_joint_id] = base_torque + amplitude * np.sin(3.0 * t)

    # e. 底盘：每 6 秒切换一次模式 (前后 <-> 侧移)
    base_phase = int(t / 6) % 2
    
    if base_phase == 0:
        base_mode_name = "前后移动 (Longitudinal)"
        # 脚轮指向前方 (0度)
        for s_id in STEER_IDS: 
            data.ctrl[s_id] = 0.0
        # 轮子转动
        for w_id in WHEEL_IDS: 
            data.ctrl[w_id] = 2.0 * np.sin(1.0 * t)
    else:
        base_mode_name = "侧向平移 (Strafing)"
        # 脚轮指向侧方 (90度)
        for s_id in STEER_IDS: 
            data.ctrl[s_id] = 1.5708
        # 轮子转动
        for w_id in WHEEL_IDS: 
            data.ctrl[w_id] = 2.0 * np.sin(1.0 * t)

    # 终端实时状态打印
    if int(t * 10) % 10 == 0:
        print(f"\r[手臂]: {joint_name} | [底盘]: {base_mode_name}          ", end="")

# --- 4. 运行可视化 ---
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("\n" + "="*50)
    print("PR2 综合分时调试模式启动")
    print("1. 手臂关节每 3 秒轮换动一次")
    print("2. 底盘每 6 秒在 '前后' 和 '侧移' 之间切换")
    print("="*50)
    
    mujoco.mj_resetData(model, data)

    while viewer.is_running():
        step_start = time.time()
        
        apply_advanced_separated_control(model, data, data.time)
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 维持仿真频率
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)