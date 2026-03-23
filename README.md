# PR2-Sim-Pure

Minimal ROS 2 control-link validation project for PR2 (no MuJoCo / no external PR2 SDK IDL required).

This repository packages:
- A reproducible Docker environment (ROS 2 Jazzy)
- VSCode devcontainer config
- Minimal PR2 control packages for weekly acceptance:
  - `pr2_mobile_controller`
  - `pr2_description`
  - `pr2_bringup` (mock launch)
  - `pr2_base_fixed_cmd.py` test script

## Quick Start (VSCode Devcontainer)

1. Open this repo in VSCode.
2. Run `Dev Containers: Reopen in Container`.
3. Wait for post-create build to finish.

## Manual Docker Build

```bash
cd /path/to/PR2-Sim-Pure
docker build -t pr2-sim-pure:jazzy .
```

If your user is not yet in `docker` group, use `sudo docker ...` first.

## One-Command Smoke Test In Docker

Run from host:

```bash
cd /path/to/PR2-Sim-Pure
docker run --rm -it \
  -v "$(pwd)":/workspaces/PR2-Sim-Pure \
  pr2-sim-pure:jazzy \
  bash scripts/container_smoke.sh
```

What it checks:
- build `pr2_description` + `pr2_mobile_controller` + `pr2_bringup`
- launch `pr2_control_mock.launch.py`
- verify `pr2_omni_controller` and `joint_state_broadcaster` are `active`
- run `pr2_base_fixed_cmd.py`
- confirm `/pr2/joint_states` contains non-zero wheel velocities during motion

## Control-Link Acceptance (No SDK / No MuJoCo)

Terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/PR2-Sim-Pure
source install/setup.bash
ros2 launch pr2_bringup pr2_control_mock.launch.py
```

Terminal 2:
```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/PR2-Sim-Pure
source install/setup.bash
ros2 control list_controllers -c /pr2/controller_manager
python3 pr2_ws/scripts/pr2_base_fixed_cmd.py --vx 0.20 --duration 3.0 --stop-duration 1.0
```

Terminal 3:
```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/PR2-Sim-Pure
source install/setup.bash
ros2 topic echo /pr2/joint_states
```

Acceptance target:
- `pr2_omni_controller` and `joint_state_broadcaster` are `active`
- command script publishes successfully
- wheel joint velocity in `/pr2/joint_states` changes non-zero during motion and returns to zero after stop
