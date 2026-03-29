import mujoco
model = mujoco.MjModel.from_xml_path("/workspace/unitree_mujoco/unitree_robots/pr2/scene.xml")
for i in range(model.nu):
    print(f"执行器 ID {i}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)}")