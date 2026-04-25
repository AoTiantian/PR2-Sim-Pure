from __future__ import annotations

import time
from pathlib import Path

import mujoco
import mujoco.viewer


def _default_model_path() -> str:
    repo_root = Path(__file__).resolve().parents[4]
    return str(repo_root / "unitree_mujoco" / "unitree_robots" / "pr2" / "scene.xml")


GRIPPER_L_ID = 1  # l_gripper_pos
GRIPPER_R_ID = 0  # r_gripper_pos
TORSO_ID = 17


def apply_full_range_control(data: mujoco.MjData, t: float, test_max: float) -> None:
    data.ctrl[:] = 0.0
    data.ctrl[TORSO_ID] = 500.0

    cycle = 2.0
    val = (t % cycle) / cycle
    if val > 0.5:
        target = test_max * (1.0 - (val - 0.5) * 2.0)
    else:
        target = test_max * (val * 2.0)

    data.ctrl[GRIPPER_L_ID] = target
    data.ctrl[GRIPPER_R_ID] = target

    if int(t * 10) % 20 == 0:
        print(f"当前仿真时间: {t:.1f}s | 发送给夹爪的控制值: {target:.4f}")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(_default_model_path())
    data = mujoco.MjData(model)

    ctrl_min = model.actuator_ctrlrange[GRIPPER_L_ID][0]
    ctrl_max = model.actuator_ctrlrange[GRIPPER_L_ID][1]
    print(f"检测到夹爪控制限位: 最小值={ctrl_min}, 最大值={ctrl_max}")

    test_max = 1.0 if ctrl_max == 0 else ctrl_max
    if ctrl_max == 0:
        print("警告：模型未定义限位，切换为手动测试值 1.0")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("=" * 40)
        print("PR2 满幅度夹爪开合测试")
        print(f"目标：从 {ctrl_min} 运动到 {test_max}")
        print("=" * 40)

        while viewer.is_running():
            step_start = time.time()
            apply_full_range_control(data, data.time, test_max)
            mujoco.mj_step(model, data)
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
