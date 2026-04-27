# PR2 whole-body admittance demo artifacts

This directory contains the presentation-oriented whole-body XYZ force demo used
to verify that the robot visibly responds to an external 3D wrench.  It is a
visual/demo run, not the bounded acceptance run.  For acceptance thresholds, use
`launch/pr2_whole_body_force.launch.py`.

## Main artifacts

- `pr2_whole_body_xyz_response_motion_obvious.mp4` — 9 s, 1280×720, 30 fps
  offline MuJoCo replay video.  The video is rendered from recorded
  `/joint_states`, `/odom`, and `wbc/arm/external_wrench`, so the robot motion
  corresponds to the measured state history rather than a static overlay.
- `pr2_wb_obvious_motion_contact.png` — before/contact/after triptych for visual
  comparison of the robot pose and base position.
- `wb_xyz_obvious3.csv` — source force-response CSV, including force input,
  end-effector pose, admittance debug channels, command norms, and base odometry.
- `pr2_whole_body_base_arm_summary.png` — compact whole-body summary plot.
- `pr2_admittance_dashboard_white.png` / `pr2_admittance_dashboard_dark.png` —
  report and slide dashboards.
- `pr2_admittance_paper_summary.png` — compact paper-style summary.
- `pr2_admittance_diagnostics_scorecard.png` — diagnostic scorecard.
- `pr2_admittance_axis_{x,y,z}_focus.png` — per-axis focus plots.
- `pr2_admittance_metrics.json` — numeric metrics from the plotting script.

## Reproduction commands

From the repository root:

```bash
set +u
source /opt/ros/jazzy/setup.bash
source pr2_ws/install/setup.bash
set -u

# Terminal A: record the state used for offline rendering.
PYTHONPATH=pr2_ws/src/pr2_mujoco_bridge:/tmp/pr2-feat-python/pr2_ws/vtest/lib/python3.12/site-packages:${PYTHONPATH:-} \
/usr/bin/python3 pr2_ws/src/pr2_mujoco_bridge/scripts/record_pr2_whole_body_state.py \
  --out /tmp/pr2_wb_obvious_state.npz \
  --hz 30

# Terminal B: run the amplified presentation demo.
PYTHONPATH=pr2_ws/src/pr2_mujoco_bridge:/tmp/pr2-feat-python/pr2_ws/vtest/lib/python3.12/site-packages:${PYTHONPATH:-} \
ros2 launch pr2_mujoco_bridge pr2_whole_body_force_amplified.launch.py \
  use_viewer:=false \
  force_axis:=xyz \
  force_magnitude:=30.0 \
  log_file:=/tmp/wb_xyz_obvious.csv
```

After the ROS launch exits, stop the recorder so it saves the NPZ, then render:

```bash
MUJOCO_GL=egl \
PYTHONPATH=/tmp/pr2-feat-python/pr2_ws/vtest/lib/python3.12/site-packages:${PYTHONPATH:-} \
/usr/bin/python3 pr2_ws/src/pr2_mujoco_bridge/scripts/render_pr2_whole_body_video.py \
  --state /tmp/pr2_wb_obvious_state.npz \
  --csv /tmp/wb_xyz_obvious.csv \
  --out /tmp/pr2_whole_body_xyz_response_motion_obvious.mp4 \
  --width 1280 \
  --height 720 \
  --fps 30

/usr/bin/python3 pr2_ws/src/pr2_mujoco_bridge/scripts/generate_pr2_admittance_visuals.py \
  --csv /tmp/wb_xyz_obvious.csv \
  --outdir /tmp/pr2_wb_obvious_visuals \
  --baseline-skip-samples 60

/usr/bin/python3 pr2_ws/src/pr2_mujoco_bridge/scripts/plot_whole_body_response.py \
  --csv /tmp/wb_xyz_obvious.csv \
  --save /tmp/pr2_wb_obvious_visuals/pr2_whole_body_base_arm_summary.png \
  --baseline-skip-samples 60
```

## Reference metrics for the committed run

The committed `wb_xyz_obvious3.csv` and video correspond to this reference run:

```text
rows: 901
duration: 8.9998 s
force-on window: 1.0099 s → 4.0099 s
end-effector peak displacement: 87.13 mm
base XY peak displacement: 188.95 mm
base yaw peak: 0.35 deg
video: 1280×720, 30 fps, 270 frames, H.264
```

The stricter ad-hoc visual threshold of `min-base-linear-peak-mm=200` was not
claimed as passed; the recorded base peak is `188.95 mm`, which was chosen
because it is visually clear while remaining a bounded presentation demo.
