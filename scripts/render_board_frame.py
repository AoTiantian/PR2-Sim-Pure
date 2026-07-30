#!/usr/bin/env python3
"""Render a single frame of the human-board demo."""
import os
os.environ["MUJOCO_GL"] = "egl"

import mujoco
import numpy as np

SCENE = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "unitree_mujoco/unitree_robots/pr2/scene_human_board.xml",
)

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)

board_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "board")
force = np.array(
    [0.0, 0.0, model.body_mass[board_id] * abs(float(model.opt.gravity[2]))],
    dtype=np.float64,
)

# Run a few steps to verify the static hold.
for _ in range(500):  # 1 second at 0.002 timestep
    data.xfrc_applied[board_id, :3] = force
    data.xfrc_applied[board_id, 3:] = 0.0
    mujoco.mj_step(model, data)

# Render
renderer = mujoco.Renderer(model, height=800, width=1200)
renderer.update_scene(data, camera="fixed")
pixels = renderer.render()
renderer.close()

# Save
from PIL import Image
img = Image.fromarray(pixels)
img.save("/tmp/board_frame.png")
print(f"Saved /tmp/board_frame.png  t={data.time:.1f}s  pos={data.xpos[board_id]}")
