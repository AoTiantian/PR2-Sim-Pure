#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

fail=0
check_path() {
  local p="$1"
  if [[ -e "$p" ]]; then
    printf 'OK   %s\n' "$p"
  else
    printf 'MISS %s\n' "$p" >&2
    fail=1
  fi
}

check_absent() {
  local p="$1"
  if [[ -e "$p" ]]; then
    printf 'BAD  generated/root artifact should not be committed: %s\n' "$p" >&2
    fail=1
  else
    printf 'OK   absent %s\n' "$p"
  fi
}

printf '== PR2-Sim-Pure AI/project layout verification ==\n'
check_path AGENTS.md
check_path CLAUDE.md
check_path .claude/rules/project-layout.md
check_path .claude/rules/ros2-jazzy.md
check_path .claude/rules/pr2-safety.md
check_path .claude/rules/acceptance.md
check_path README.md
check_path .devcontainer/devcontainer.json
check_path .devcontainer/Dockerfile
check_path pr2_ws/src/pr2_mujoco_bridge/package.xml
check_path pr2_ws/src/pr2_mujoco_bridge/setup.py
check_path pr2_ws/src/pr2_mujoco_bridge/README_ACCEPTANCE_FEAT.md
check_path pr2_ws/src/pr2_mujoco_bridge/scripts/validate_force_response.py
check_path pr2_ws/src/pr2_mujoco_bridge/scripts/cleanup_pr2_ros_processes.sh
check_path pr2_ws/src/pr2_mujoco_bridge/test/test_admittance_core.py
check_path pr2_ws/src/pr2_mujoco_bridge/test/test_validate_force_response.py
check_path src/pr2_ros2_stack/pr2_description/package.xml
check_path src/pr2_ros2_stack/pr2_bringup/package.xml
check_path unitree_mujoco/unitree_robots/pr2/scene.xml

for p in build install log .venv venv; do
  check_absent "$p"
done

for marker in package.xml setup.py setup.cfg CMakeLists.txt; do
  if [[ -e "$marker" ]]; then
    printf 'BAD  root-level ROS package marker exists: %s\n' "$marker" >&2
    fail=1
  fi
done

printf '\n== ROS package markers ==\n'
find pr2_ws/src src/pr2_ros2_stack -name package.xml -print | sort

if [[ "$fail" -ne 0 ]]; then
  printf '\nFAIL: project layout verification failed.\n' >&2
  exit 1
fi
printf '\nPASS: project layout verification passed.\n'
