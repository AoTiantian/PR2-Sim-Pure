#!/usr/bin/env python3
"""
Minimal demo: human holding a board in the air.

A virtual human applies force at one end of a free board.  No robot.
The board is a 2m × 10cm × 4cm box (1.8 kg) floating horizontally in space.

Usage:
    python3 human_board_demo.py                          # viewer on
    python3 human_board_demo.py --no-viewer --duration 5 # headless, record data

Physics:
  - Gravity pulls the board down (–Z)
  - Human applies upward force at the far end of the board
  - The default force is exactly mg, so the board floats at its initial height
  - A gripping hand also applies the moment needed to keep the board level
  - Use --allow-rotation to model a point contact that cannot apply a moment
"""

import argparse
import os
import sys
import time
import csv

import mujoco
import mujoco.viewer
import numpy as np

SCENE = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                     "unitree_mujoco/unitree_robots/pr2/scene_human_board.xml")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--duration", type=float, default=0,
                        help="seconds to run headless (0 = forever with viewer)")
    parser.add_argument("--force-x", type=float, default=0.0)
    parser.add_argument("--force-y", type=float, default=0.0)
    parser.add_argument("--force-z", type=float, default=None,
                        help="upward force in world Z (N); default is exactly mg")
    parser.add_argument("--hand-offset", type=float, default=1.0,
                        help="force application point from COM in board X (m)")
    parser.add_argument(
        "--allow-rotation",
        action="store_true",
        help="model a point contact: do not apply the gripping moment",
    )
    parser.add_argument("--csv", type=str, default="",
                        help="output CSV path for headless recording")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)

    board_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "board")
    if board_id < 0:
        print("ERROR: board body not found in scene")
        sys.exit(1)

    force_z = (
        model.body_mass[board_id] * abs(float(model.opt.gravity[2]))
        if args.force_z is None
        else args.force_z
    )
    force = np.array([args.force_x, args.force_y, force_z], dtype=np.float64)
    r_local = np.array([args.hand_offset, 0.0, 0.0], dtype=np.float64)

    print(f"Board mass: {model.body_mass[board_id]:.2f} kg")
    print(f"Board weight: {model.body_mass[board_id] * 9.81:.1f} N")
    print(f"Hand force: {force} N")
    print(f"Hand offset from COM: {r_local[0]} m")
    print(
        "Contact model: "
        + ("point force (board may rotate)" if args.allow_rotation else "gripping hand")
    )
    if abs(np.linalg.norm(force) - model.body_mass[board_id] * 9.81) < 0.5:
        if args.allow_rotation:
            print("→ force ≈ mg: COM should stay up, but the board will rotate")
        else:
            print("→ force ≈ mg with grip moment: board should remain level")

    rows = []
    t0 = time.perf_counter()

    if args.no_viewer:
        # Headless mode
        print(f"\nRunning headless for {args.duration}s...")
        while data.time < args.duration:
            _apply_force(
                data, board_id, force, r_local,
                cancel_force_moment=not args.allow_rotation,
            )
            mujoco.mj_step(model, data)

            if args.csv:
                rows.append(_snapshot(data, board_id))

        if args.csv and rows:
            _write_csv(args.csv, rows)
            print(f"Wrote {len(rows)} rows to {args.csv}")

        _print_summary(data, board_id)
    else:
        # Viewer mode
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Start from a useful overview, but keep the interactive viewer in
            # free-camera mode so mouse rotate/pan/zoom continues to work.
            mujoco.mj_forward(model, data)
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = data.xpos[board_id]
            # Look halfway between the floor and board so the 1.2 m air gap is
            # visually obvious instead of projecting the board onto the floor.
            viewer.cam.lookat[2] *= 0.5
            viewer.cam.distance = 5.0
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -18.0
            viewer.sync()
            print("Viewer started — close window to exit")
            while viewer.is_running():
                step_start = time.perf_counter()
                _apply_force(
                    data, board_id, force, r_local,
                    cancel_force_moment=not args.allow_rotation,
                )
                mujoco.mj_step(model, data)
                viewer.sync()
                remaining = model.opt.timestep - (time.perf_counter() - step_start)
                if remaining > 0.0:
                    time.sleep(remaining)


def _apply_force(
    data, board_id, force_world, r_local, *, cancel_force_moment=False
):
    """Apply a force at the hand, optionally including the hand's grip moment."""
    board_mat = data.xmat[board_id].reshape(3, 3)
    r_world = board_mat @ r_local
    tau = np.cross(r_world, force_world)
    data.xfrc_applied[board_id, :3] = force_world
    # A hand that grasps the board can transmit a moment. Cancelling r x F
    # gives zero net torque at the COM and therefore a level static hold.
    data.xfrc_applied[board_id, 3:] = (
        np.zeros(3, dtype=np.float64) if cancel_force_moment else tau
    )


def _snapshot(data, board_id):
    return {
        "t": float(data.time),
        "x": float(data.xpos[board_id, 0]),
        "y": float(data.xpos[board_id, 1]),
        "z": float(data.xpos[board_id, 2]),
        "qw": float(data.xquat[board_id, 0]),
        "qx": float(data.xquat[board_id, 1]),
        "qy": float(data.xquat[board_id, 2]),
        "qz": float(data.xquat[board_id, 3]),
    }


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)


def _print_summary(data, board_id):
    p = data.xpos[board_id]
    print(f"Final board position: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]")


if __name__ == "__main__":
    main()
