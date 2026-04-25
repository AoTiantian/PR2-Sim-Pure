#!/usr/bin/env bash
set -euo pipefail

pkill -f 'ros2 launch pr2_mujoco_bridge|pr2_mujoco_sim|pr2_arm_admittance|pr2_arm_force_injector|pr2_force_projector|pr2_base_admittance|pr2_wbc_coordinator' || true
sleep 1
