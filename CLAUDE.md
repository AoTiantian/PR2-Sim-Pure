# Claude Code entrypoint for PR2-Sim-Pure

This is Haohua's active PR2 robotics simulation project. Claude Code must read and follow the repository-level source of truth plus focused rules below:

@AGENTS.md
@.claude/rules/project-layout.md
@.claude/rules/ros2-jazzy.md
@.claude/rules/pr2-safety.md
@.claude/rules/acceptance.md

Operational reminders:

- Work from the repository root unless a command explicitly says `cd pr2_ws`.
- Check `git status --short --branch` before edits.
- Preserve the two existing ROS workspace areas: `pr2_ws/` and `src/pr2_ros2_stack/`.
- Prefer headless MuJoCo (`use_viewer:=false`) for automated validation.
- For controller/admittance/WBC changes, run or update tests and consult `pr2_ws/src/pr2_mujoco_bridge/README_ACCEPTANCE_FEAT.md`.
- Never execute physical robot/hardware-affecting steps without explicit user approval.
