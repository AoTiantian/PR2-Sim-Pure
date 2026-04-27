#!/usr/bin/env python3
"""Render an offline MuJoCo video from recorded PR2 joint_states/odom."""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")


def default_model_path() -> str:
    suffix = Path("unitree_mujoco") / "unitree_robots" / "pr2" / "scene.xml"
    env_path = os.environ.get("PR2_MUJOCO_MODEL_PATH")
    if env_path:
        env_candidate = Path(env_path)
        if env_candidate.is_file():
            return str(env_candidate)

    roots = [Path.cwd().resolve(), *Path(__file__).resolve().parents, Path("/workspace")]
    for root in roots:
        candidate = root / suffix
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError(
        "Could not find unitree_mujoco/unitree_robots/pr2/scene.xml; "
        "set PR2_MUJOCO_MODEL_PATH or run from the PR2-Sim-Pure checkout."
    )

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def load_csv_metrics(path: str, baseline_skip: int = 60) -> dict:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    def arr(name): return np.asarray([float(r[name]) for r in rows], dtype=float)
    t = arr("timestamp"); t = t - t[0]
    force = np.vstack([arr("force_x"), arr("force_y"), arr("force_z")]).T
    active = np.max(np.abs(force), axis=1) > 0.5
    onset = float(t[np.flatnonzero(active)[0]]) if active.any() else None
    release = float(t[min(np.flatnonzero(active)[-1] + 1, len(t)-1)]) if active.any() else None
    idx = min(max(baseline_skip, 0), len(rows)-1)
    ee = {a: (arr(f"pos_{a}") - arr(f"pos_{a}")[idx]) * 1000.0 for a in "xyz"}
    base = {a: (arr(f"base_{a}") - arr(f"base_{a}")[idx]) * 1000.0 for a in "xy"}
    base_yaw = (arr("base_yaw") - arr("base_yaw")[idx]) * 180.0 / np.pi
    return {
        "onset": onset, "release": release,
        "ee_peak_mm": max(float(np.max(np.abs(v))) for v in ee.values()),
        "base_peak_mm": max(float(np.max(np.abs(v))) for v in base.values()),
        "base_yaw_peak_deg": float(np.max(np.abs(base_yaw))),
        "duration_s": float(t[-1]),
    }


def draw_overlay(img: np.ndarray, elapsed: float, metrics: dict, base_xy_mm: np.ndarray, force_vec: np.ndarray) -> np.ndarray:
    im = Image.fromarray(img)
    draw = ImageDraw.Draw(im, "RGBA")
    try:
        font_big = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
        font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13)
    except Exception:
        font_big = font = font_small = None
    w, h = im.size
    active = metrics["onset"] is not None and metrics["onset"] <= elapsed <= metrics["release"]
    draw.rounded_rectangle([18, 16, 500, 154], radius=12, fill=(8, 12, 22, 185), outline=(255,255,255,80), width=1)
    draw.text((34, 28), "PR2 whole-body admittance demo", fill=(255,255,255,255), font=font_big)
    status = "XYZ FORCE ON" if active else ("settling / return" if metrics["release"] and elapsed > metrics["release"] else "ready")
    color = (255, 98, 98, 255) if active else (116, 192, 252, 255)
    draw.text((34, 64), f"t={elapsed:4.1f}s   {status}", fill=color, font=font)
    draw.text((34, 91), f"EE peak ≈ {metrics['ee_peak_mm']:.0f} mm   base peak ≈ {metrics['base_peak_mm']:.0f} mm   yaw ≈ {metrics['base_yaw_peak_deg']:.1f}°", fill=(230,230,230,255), font=font)
    draw.text((34, 118), "Amplified presentation gains: arm + mobile base both visibly compliant", fill=(200,220,255,255), font=font_small)

    # Force arrow badge.
    if active:
        cx, cy = w - 130, 95
        draw.ellipse([cx-64, cy-64, cx+64, cy+64], fill=(120,20,20,135), outline=(255,120,120,220), width=3)
        draw.line([cx-42, cy+30, cx+38, cy-28], fill=(255,220,80,255), width=8)
        draw.polygon([(cx+38,cy-28),(cx+12,cy-20),(cx+28,cy+2)], fill=(255,220,80,255))
        draw.text((cx-48, cy+42), "Fxyz", fill=(255,255,255,255), font=font_big)

    # Top-down base trail inset.
    inset = [w-310, h-185, w-26, h-24]
    draw.rounded_rectangle(inset, radius=10, fill=(8, 12, 22, 180), outline=(255,255,255,80), width=1)
    draw.text((inset[0]+12, inset[1]+8), "base XY trail", fill=(255,255,255,240), font=font_small)
    pts = base_xy_mm
    if len(pts) > 1:
        max_abs = max(120.0, float(np.max(np.abs(pts))) * 1.15)
        cx = (inset[0]+inset[2]) / 2; cy = (inset[1]+inset[3]) / 2 + 12
        sx = (inset[2]-inset[0]-36) / (2*max_abs); sy = (inset[3]-inset[1]-48) / (2*max_abs)
        xy = [(cx + x*sx, cy - y*sy) for x,y in pts]
        draw.line(xy, fill=(99,230,190,230), width=3)
        r=5; draw.ellipse([xy[-1][0]-r, xy[-1][1]-r, xy[-1][0]+r, xy[-1][1]+r], fill=(255,212,59,255))
        draw.text((inset[0]+12, inset[3]-24), f"±{max_abs:.0f} mm", fill=(210,210,210,230), font=font_small)
    return np.asarray(im)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--model", default=None, help="path to PR2 MuJoCo scene.xml; auto-detected if omitted")
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()

    rec = np.load(args.state, allow_pickle=False)
    t = np.asarray(rec["t"], dtype=float)
    odom = np.asarray(rec["odom"], dtype=float)
    force = np.asarray(rec["force"], dtype=float)
    joint_pos = np.asarray(rec["joint_pos"], dtype=float)
    joint_names = [str(x) for x in rec["joint_names"]]

    active = np.max(np.abs(force), axis=1) > 0.5
    if active.any():
        i0 = max(0, int(np.flatnonzero(active)[0]) - args.fps)
        i1 = min(len(t)-1, int(np.flatnonzero(active)[-1]) + 5 * args.fps)
    else:
        i0, i1 = 0, len(t)-1
    t0 = t[i0]

    model_path = args.model or default_model_path()
    model = mujoco.MjModel.from_xml_path(model_path)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, width=args.width, height=args.height)

    # Map joint_state names to scalar qpos addresses.
    qadr_by_name = {}
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if not name:
            continue
        jt = int(model.jnt_type[j])
        if jt in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            qadr_by_name[name] = int(model.jnt_qposadr[j])

    base_free_qadr = 0
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    for j in range(model.njnt):
        if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE) and int(model.jnt_bodyid[j]) == base_body:
            base_free_qadr = int(model.jnt_qposadr[j])
            break

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    # Fixed, relatively close camera: do not follow the robot, otherwise base
    # translation would be visually cancelled.  The wider yaw makes the base
    # XY drift and left-arm compliance visible at the same time.
    cam.lookat[:] = np.array([-0.55, 0.02, 0.90], dtype=np.float64)
    cam.distance = 3.55
    cam.azimuth = 132
    cam.elevation = -16

    metrics = load_csv_metrics(args.csv)
    baseline_idx = min(60, len(odom)-1)
    base_xy_mm_all = (odom[:, :2] - odom[baseline_idx, :2]) * 1000.0

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{args.width}x{args.height}", "-r", str(args.fps),
        "-i", "-", "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p", args.out,
    ], stdin=subprocess.PIPE)

    written = 0
    for i in range(i0, i1 + 1):
        mujoco.mj_resetData(model, data)
        # base free qpos: x,y,z,qw,qx,qy,qz. ROS odom quat is qx,qy,qz,qw.
        x,y,z,qx,qy,qz,qw = odom[i]
        data.qpos[base_free_qadr:base_free_qadr+7] = [x, y, z, qw, qx, qy, qz]
        for k, name in enumerate(joint_names):
            qadr = qadr_by_name.get(name)
            if qadr is not None and k < joint_pos.shape[1]:
                data.qpos[qadr] = joint_pos[i, k]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        frame = renderer.render()
        elapsed = float(t[i] - t0)
        overlay = draw_overlay(frame, elapsed, metrics, base_xy_mm_all[i0:i+1], force[i])
        proc.stdin.write(overlay.tobytes())
        written += 1
    proc.stdin.close(); rc = proc.wait()
    if rc != 0:
        raise SystemExit(f"ffmpeg failed with code {rc}")
    print(json.dumps({"out": args.out, "frames": written, "duration_s": written / args.fps, "trim_indices": [int(i0), int(i1)], "metrics": metrics}, indent=2))


if __name__ == "__main__":
    main()
