#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
cd /workspaces/PR2-Sim-Pure

colcon --log-base /tmp/pr2_log build \
  --base-paths pr2_ws/src \
  --packages-select pr2_description pr2_mobile_controller pr2_bringup \
  --symlink-install \
  --build-base /tmp/pr2_build \
  --install-base /tmp/pr2_install

source /tmp/pr2_install/setup.bash

ros2 launch pr2_bringup pr2_control_mock.launch.py >/tmp/pr2_launch.log 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q '/pr2/controller_manager/list_controllers'; then
    break
  fi
  sleep 1
done

if ! ros2 service list 2>/dev/null | grep -q '/pr2/controller_manager/list_controllers'; then
  echo "ERROR: controller_manager service not available in time"
  tail -n 120 /tmp/pr2_launch.log || true
  exit 1
fi

echo "=== Controllers ==="
ros2 control list_controllers -c /pr2/controller_manager | tee /tmp/pr2_controllers.txt

grep -q 'pr2_omni_controller .* active' /tmp/pr2_controllers.txt || {
  echo "ERROR: pr2_omni_controller not active"
  tail -n 120 /tmp/pr2_launch.log || true
  exit 1
}

grep -q 'joint_state_broadcaster .* active' /tmp/pr2_controllers.txt || {
  echo "ERROR: joint_state_broadcaster not active"
  tail -n 120 /tmp/pr2_launch.log || true
  exit 1
}

(timeout 12 ros2 topic echo /pr2/joint_states >/tmp/pr2_joint_states.log) &
ECHO_PID=$!
sleep 1

python3 pr2_ws/scripts/pr2_base_fixed_cmd.py \
  --vx 0.20 --vy 0.00 --wz 0.00 --duration 3.0 --stop-duration 1.0 \
  | tee /tmp/pr2_cmd.log

wait "${ECHO_PID}" || true

python3 - <<'PY'
import re
from pathlib import Path

text = Path("/tmp/pr2_joint_states.log").read_text(errors="ignore")
msgs = [m.strip() for m in text.split("---") if m.strip()]
sampled = 0
nonzero = 0

for m in msgs:
    if "velocity:" not in m:
        continue
    sampled += 1
    vel = m.split("velocity:", 1)[1]
    if "effort:" in vel:
        vel = vel.split("effort:", 1)[0]
    nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", vel)]
    if any(abs(n) > 1e-4 for n in nums):
        nonzero += 1

print(f"SAMPLED_MESSAGES={sampled}")
print(f"NONZERO_VELOCITY_MESSAGES={nonzero}")
if sampled == 0:
    raise SystemExit("ERROR: No /pr2/joint_states samples captured")
if nonzero == 0:
    raise SystemExit("ERROR: Captured samples but all velocities are zero")
PY

echo "SMOKE_TEST=PASS"
