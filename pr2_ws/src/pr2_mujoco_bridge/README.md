# pr2_mujoco_bridge

ROS 2 Jazzy package for the PR2 MuJoCo simulation bridge.

This package contains only the simulator bridge executable:

```bash
ros2 run pr2_mujoco_bridge pr2_mujoco_sim
```

For the complete QP whole-body admittance workflow, use the bringup package:

```bash
ros2 launch pr2_qp_admittance_bringup pr2_qp_whole_body_admittance.launch.py
```

## Role

`pr2_mujoco_sim` loads the PR2 MuJoCo model, advances simulation, publishes
`joint_states`, `odom`, and TF, and subscribes to `cmd_vel` and
`joint_commands`.
