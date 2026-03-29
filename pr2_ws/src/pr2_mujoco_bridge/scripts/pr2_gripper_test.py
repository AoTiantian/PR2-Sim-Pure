import mujoco
import mujoco.viewer
import time
import numpy as np

# 1. 加载模型
model_path = "/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# --- 2. 自动检测夹爪的控制限位 ---
GRIPPER_L_ID = 1  # l_gripper_pos
GRIPPER_R_ID = 0  # r_gripper_pos
TORSO_ID = 17

# 获取模型中定义的夹爪控制范围
# model.actuator_ctrlrange 形状是 (nu, 2)，存储了每个执行器的 [最小值, 最大值]
ctrl_min = model.actuator_ctrlrange[GRIPPER_L_ID][0]
ctrl_max = model.actuator_ctrlrange[GRIPPER_L_ID][1]

print(f"检测到夹爪控制限位: 最小值={ctrl_min}, 最大值={ctrl_max}")

# 如果模型里没定义限位（显示为0），我们手动给一个较大的测试值
if ctrl_max == 0:
    print("警告：模型未定义限位，切换为手动测试值 1.0")
    test_max = 1.0
else:
    test_max = ctrl_max

# 3. 控制逻辑
def apply_full_range_control(model, data, t):
    data.ctrl[:] = 0.0
    data.ctrl[TORSO_ID] = 500.0 # 稳住躯干

    # 使用三角波，让它在完全闭合和完全打开之间匀速往复
    # 使用 np.abs 配合线性增长的时间实现 0 -> test_max 的往返
    cycle = 2.0  # 2秒一个周期
    val = (t % cycle) / cycle  # 0 到 1 线性增长
    if val > 0.5:
        target = test_max * (1.0 - (val - 0.5) * 2) # 从最大回到0
    else:
        target = test_max * (val * 2) # 从0增长到最大

    data.ctrl[GRIPPER_L_ID] = target
    data.ctrl[GRIPPER_R_ID] = target
    
    # 在控制台打印当前发送的数值，方便比对画面
    if int(t*10) % 20 == 0: # 约每2秒打印一次
        print(f"当前仿真时间: {t:.1f}s | 发送给夹爪的控制值: {target:.4f}")

# 4. 运行可视化
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("="*40)
    print("PR2 满幅度夹爪开合测试")
    print(f"目标：从 {ctrl_min} 运动到 {test_max}")
    print("="*40)
    
    while viewer.is_running():
        step_start = time.time()
        apply_full_range_control(model, data, data.time)
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)