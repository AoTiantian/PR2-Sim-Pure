# PR2 simulation and robotics safety rule

Treat PR2 control code as safety-sensitive even when current validation is MuJoCo-only.

Prefer simulation, headless MuJoCo, unit tests, validators, and dry-run checks. Do not add or run real-hardware motor/CAN/EtherCAT/serial/GPIO/battery/brake/gripper actions unless Haohua explicitly requests it and the plan documents safe limits and emergency stop assumptions.

Do not let multiple controllers fight. For external IK, admittance, force injection, or WBC nodes, normally launch `pr2_mujoco_sim` with `demo_motion:=false`.

Preserve stale-command timeouts, force/velocity clipping, deadbands, saturation limits, and safe idle behavior. Any gain/threshold changes must explain stability and acceptance-test impact.
