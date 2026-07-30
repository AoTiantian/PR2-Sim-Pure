#!/usr/bin/env python3
"""Render the human-board demo frames to PNG files using EGL offscreen."""
import os
import time
os.environ["MUJOCO_GL"] = "egl"

import mujoco
import numpy as np
from PIL import Image

SCENE = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "unitree_mujoco/unitree_robots/pr2/scene_human_board.xml",
)
OUTDIR = "/tmp/board_frames"

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)

bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "board")
F = np.array(
    [0.0, 0.0, model.body_mass[bid] * abs(float(model.opt.gravity[2]))],
    dtype=np.float64,
)
os.makedirs(OUTDIR, exist_ok=True)
# Clean old frames
for f in os.listdir(OUTDIR):
    os.remove(os.path.join(OUTDIR, f))

renderer = mujoco.Renderer(model, height=800, width=1200)

print(f"Rendering frames to {OUTDIR}/ ...")
fps = 30
frame_dt = 1.0 / fps
sim_steps_per_frame = int(frame_dt / model.opt.timestep)
render_every = max(1, sim_steps_per_frame)

frame_count = 0
t_start = time.perf_counter()

try:
    while data.time < 8.0:
        # Physics
        for _ in range(render_every):
            data.xfrc_applied[bid, :3] = F
            data.xfrc_applied[bid, 3:] = 0.0
            mujoco.mj_step(model, data)

        # Render
        renderer.update_scene(data, camera="fixed")
        pixels = renderer.render()
        Image.fromarray(pixels).save(f"{OUTDIR}/frame_{frame_count:04d}.png")
        frame_count += 1

        elapsed = time.perf_counter() - t_start
        if frame_count % 30 == 0:
            z = data.xpos[bid, 2]
            print(f"  frame {frame_count}  t={data.time:.1f}s  z={z:.2f}m")

except KeyboardInterrupt:
    pass

renderer.close()
print(f"\nDone. {frame_count} frames in {OUTDIR}/")
print(f"To view as slideshow:  eog {OUTDIR}")
print(f"To make video:  ffmpeg -framerate 30 -i {OUTDIR}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p /tmp/board.mp4")
