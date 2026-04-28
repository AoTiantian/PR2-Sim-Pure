# PR2 MJCF Force/Actuator Limit Audit

This report is generated from the read-only MJCF audit helper. It identifies values that may limit or mask whole-body admittance motion; it does not recommend changing XML before simulation evidence is collected.

## Input files
- Model: `unitree_mujoco/unitree_robots/pr2/robot_pr2.xml`
- Scene: `unitree_mujoco/unitree_robots/pr2/scene.xml`

## Extracted entry counts

- `joints`: 40
- `actuators`: 32
- `geoms`: 41
- `other_limit_fields`: 0

## Bottleneck candidates

- **review** `fl_caster_rotation_joint` (joint, force/control range): base wheel/caster range '-6.5 6.5' may clip visible whole-body motion
- **review** `fl_caster_l_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `fl_caster_l_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `fl_caster_r_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `fl_caster_r_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `fr_caster_rotation_joint` (joint, force/control range): base wheel/caster range '-6.5 6.5' may clip visible whole-body motion
- **review** `fr_caster_l_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `fr_caster_l_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `fr_caster_r_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `fr_caster_r_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `bl_caster_rotation_joint` (joint, force/control range): base wheel/caster range '-6.5 6.5' may clip visible whole-body motion
- **review** `bl_caster_l_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `bl_caster_l_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `bl_caster_r_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `bl_caster_r_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `br_caster_rotation_joint` (joint, force/control range): base wheel/caster range '-6.5 6.5' may clip visible whole-body motion
- **review** `br_caster_l_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `br_caster_l_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `br_caster_r_wheel_joint` (joint, force/control range): base wheel/caster range '-7 7' may clip visible whole-body motion
- **note** `br_caster_r_wheel_joint` (joint, damping): base damping='1' should be considered when diagnosing small base response
- **review** `torso_lift_joint` (joint, damping): very high damping='20000'; likely hold-like behavior and not a compliance tuning target
- **review** `r_wrist_roll_joint` (joint, force/control range): wrist range '-10 10' is much smaller than shoulder/elbow ranges
- **review** `l_wrist_roll_joint` (joint, force/control range): wrist range '-10 10' is much smaller than shoulder/elbow ranges
- **review** `fl_caster_l_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `fl_caster_r_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `fr_caster_l_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `fr_caster_r_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `bl_caster_l_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `bl_caster_r_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `br_caster_l_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `br_caster_r_wheel_vel` (actuator, force/control range): base wheel/caster range '-20 20' may clip visible whole-body motion
- **review** `r_wrist_roll_tau` (actuator, force/control range): wrist range '-10 10' is much smaller than shoulder/elbow ranges
- **review** `l_wrist_roll_tau` (actuator, force/control range): wrist range '-10 10' is much smaller than shoulder/elbow ranges
- **note** `floor` (geom, contact): floor/contact parameters can dominate mobile-base response; compare with MuJoCo defaults if friction is omitted

## Subsystem summary

### base_casters_wheels

- `fl_caster_rotation_joint` (joint/joint): name='fl_caster_rotation_joint', actuatorfrcrange='-6.5 6.5', damping='0.1'
- `fl_caster_l_wheel_joint` (joint/joint): name='fl_caster_l_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `fl_caster_r_wheel_joint` (joint/joint): name='fl_caster_r_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `fr_caster_rotation_joint` (joint/joint): name='fr_caster_rotation_joint', actuatorfrcrange='-6.5 6.5', damping='0.1'
- `fr_caster_l_wheel_joint` (joint/joint): name='fr_caster_l_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `fr_caster_r_wheel_joint` (joint/joint): name='fr_caster_r_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `bl_caster_rotation_joint` (joint/joint): name='bl_caster_rotation_joint', actuatorfrcrange='-6.5 6.5', damping='0.1'
- `bl_caster_l_wheel_joint` (joint/joint): name='bl_caster_l_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `bl_caster_r_wheel_joint` (joint/joint): name='bl_caster_r_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `br_caster_rotation_joint` (joint/joint): name='br_caster_rotation_joint', actuatorfrcrange='-6.5 6.5', damping='0.1'
- `br_caster_l_wheel_joint` (joint/joint): name='br_caster_l_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `br_caster_r_wheel_joint` (joint/joint): name='br_caster_r_wheel_joint', actuatorfrcrange='-7 7', damping='1'
- `fl_caster_steer` (actuator/position): name='fl_caster_steer', joint='fl_caster_rotation_joint', ctrlrange='-3000 3000'
- `fr_caster_steer` (actuator/position): name='fr_caster_steer', joint='fr_caster_rotation_joint', ctrlrange='-3000 3000'
- `bl_caster_steer` (actuator/position): name='bl_caster_steer', joint='bl_caster_rotation_joint', ctrlrange='-3000 3000'
- `br_caster_steer` (actuator/position): name='br_caster_steer', joint='br_caster_rotation_joint', ctrlrange='-3000 3000'
- `fl_caster_l_wheel_vel` (actuator/velocity): name='fl_caster_l_wheel_vel', joint='fl_caster_l_wheel_joint', ctrlrange='-20 20'
- `fl_caster_r_wheel_vel` (actuator/velocity): name='fl_caster_r_wheel_vel', joint='fl_caster_r_wheel_joint', ctrlrange='-20 20'
- `fr_caster_l_wheel_vel` (actuator/velocity): name='fr_caster_l_wheel_vel', joint='fr_caster_l_wheel_joint', ctrlrange='-20 20'
- `fr_caster_r_wheel_vel` (actuator/velocity): name='fr_caster_r_wheel_vel', joint='fr_caster_r_wheel_joint', ctrlrange='-20 20'
- `bl_caster_l_wheel_vel` (actuator/velocity): name='bl_caster_l_wheel_vel', joint='bl_caster_l_wheel_joint', ctrlrange='-20 20'
- `bl_caster_r_wheel_vel` (actuator/velocity): name='bl_caster_r_wheel_vel', joint='bl_caster_r_wheel_joint', ctrlrange='-20 20'
- `br_caster_l_wheel_vel` (actuator/velocity): name='br_caster_l_wheel_vel', joint='br_caster_l_wheel_joint', ctrlrange='-20 20'
- `br_caster_r_wheel_vel` (actuator/velocity): name='br_caster_r_wheel_vel', joint='br_caster_r_wheel_joint', ctrlrange='-20 20'

### grippers

- `r_gripper_l_finger_joint` (joint/joint): name='r_gripper_l_finger_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.02'
- `r_gripper_l_finger_tip_joint` (joint/joint): name='r_gripper_l_finger_tip_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.1'
- `r_gripper_r_finger_joint` (joint/joint): name='r_gripper_r_finger_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.02'
- `r_gripper_r_finger_tip_joint` (joint/joint): name='r_gripper_r_finger_tip_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.1'
- `r_gripper_joint` (joint/joint): name='r_gripper_joint', type='slide', range='0 0.09', actuatorfrcrange='-1000 1000', damping='10'
- `l_gripper_l_finger_joint` (joint/joint): name='l_gripper_l_finger_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.02'
- `l_gripper_l_finger_tip_joint` (joint/joint): name='l_gripper_l_finger_tip_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.001'
- `l_gripper_r_finger_joint` (joint/joint): name='l_gripper_r_finger_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.02'
- `l_gripper_r_finger_tip_joint` (joint/joint): name='l_gripper_r_finger_tip_joint', range='0 0.548', actuatorfrcrange='-1000 1000', damping='0.001'
- `l_gripper_joint` (joint/joint): name='l_gripper_joint', type='slide', range='0 0.09', actuatorfrcrange='-1000 1000', damping='10'
- `r_gripper_pos` (actuator/position): name='r_gripper_pos', joint='r_gripper_l_finger_joint', ctrlrange='0.01 0.548'
- `l_gripper_pos` (actuator/position): name='l_gripper_pos', joint='l_gripper_l_finger_joint', ctrlrange='0.01 0.548'

### head_sensors

- `head_pan_joint` (joint/joint): name='head_pan_joint', range='-3.007 3.007', actuatorfrcrange='-2.645 2.645', damping='0.5'
- `head_tilt_joint` (joint/joint): name='head_tilt_joint', range='-0.471238 1.39626', actuatorfrcrange='-18 18', damping='10'
- `laser_tilt_mount_joint` (joint/joint): name='laser_tilt_mount_joint', range='-0.7854 1.48353', actuatorfrcrange='-0.65 0.65', damping='0.008'
- `head_pan_pos` (actuator/position): name='head_pan_pos', joint='head_pan_joint', ctrlrange='-2.645 2.645'
- `head_tilt_pos` (actuator/position): name='head_tilt_pos', joint='head_tilt_joint', ctrlrange='-0.471238 1.39626'
- `laser_tilt_pos` (actuator/position): name='laser_tilt_pos', joint='laser_tilt_mount_joint', ctrlrange='-0.65 0.65'

### left_arm

- `l_shoulder_pan_joint` (joint/joint): name='l_shoulder_pan_joint', range='-0.000714602 2.2854', actuatorfrcrange='-30 30', damping='10'
- `l_shoulder_lift_joint` (joint/joint): name='l_shoulder_lift_joint', range='-0.5236 1.3963', actuatorfrcrange='-200 200', damping='10'
- `l_upper_arm_roll_joint` (joint/joint): name='l_upper_arm_roll_joint', range='-0.0008 3.9', actuatorfrcrange='-100 100', damping='0.1'
- `l_elbow_flex_joint` (joint/joint): name='l_elbow_flex_joint', range='-2.3213 0', actuatorfrcrange='-100 100', damping='1'
- `l_forearm_roll_joint` (joint/joint): name='l_forearm_roll_joint', range='-300 300', actuatorfrcrange='-30 30', damping='0.1'
- `l_wrist_flex_joint` (joint/joint): name='l_wrist_flex_joint', range='-2.18 0', actuatorfrcrange='-30 30', damping='0.1'
- `l_wrist_roll_joint` (joint/joint): name='l_wrist_roll_joint', range='-300 300', actuatorfrcrange='-10 10', damping='0.1'
- `l_shoulder_pan_tau` (actuator/motor): name='l_shoulder_pan_tau', joint='l_shoulder_pan_joint', gear='1', ctrlrange='-200 200'
- `l_shoulder_lift_tau` (actuator/motor): name='l_shoulder_lift_tau', joint='l_shoulder_lift_joint', gear='1', ctrlrange='-200 200'
- `l_upper_arm_roll_tau` (actuator/motor): name='l_upper_arm_roll_tau', joint='l_upper_arm_roll_joint', gear='1', ctrlrange='-100 100'
- `l_elbow_flex_tau` (actuator/motor): name='l_elbow_flex_tau', joint='l_elbow_flex_joint', gear='1', ctrlrange='-100 100'
- `l_forearm_roll_tau` (actuator/motor): name='l_forearm_roll_tau', joint='l_forearm_roll_joint', gear='1', ctrlrange='-30 30'
- `l_wrist_flex_tau` (actuator/motor): name='l_wrist_flex_tau', joint='l_wrist_flex_joint', gear='1', ctrlrange='-30 30'
- `l_wrist_roll_tau` (actuator/motor): name='l_wrist_roll_tau', joint='l_wrist_roll_joint', gear='1', ctrlrange='-10 10'

### other

- `<unnamed>` (geom/geom): type='mesh', contype='2', conaffinity='2'
- `<unnamed>` (geom/geom): type='box', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='box', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='2', conaffinity='2'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='2', conaffinity='2'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `<unnamed>` (geom/geom): type='mesh', contype='0', conaffinity='0'
- `table_top` (geom/geom): name='table_top', type='box', contype='1', conaffinity='1'

### right_arm

- `r_shoulder_pan_joint` (joint/joint): name='r_shoulder_pan_joint', range='-2.2854 0.000', actuatorfrcrange='-30 30', damping='10'
- `r_shoulder_lift_joint` (joint/joint): name='r_shoulder_lift_joint', range='-0.5236 1.3963', actuatorfrcrange='-200 200', damping='10'
- `r_upper_arm_roll_joint` (joint/joint): name='r_upper_arm_roll_joint', range='-3.9 0.000', actuatorfrcrange='-100 100', damping='0.1'
- `r_elbow_flex_joint` (joint/joint): name='r_elbow_flex_joint', range='-2.3213 0', actuatorfrcrange='-100 100', damping='1'
- `r_forearm_roll_joint` (joint/joint): name='r_forearm_roll_joint', range='-300 300', actuatorfrcrange='-30 30', damping='0.1'
- `r_wrist_flex_joint` (joint/joint): name='r_wrist_flex_joint', range='-2.18 0', actuatorfrcrange='-30 30', damping='0.1'
- `r_wrist_roll_joint` (joint/joint): name='r_wrist_roll_joint', range='-300 300', actuatorfrcrange='-10 10', damping='0.1'
- `r_shoulder_pan_tau` (actuator/motor): name='r_shoulder_pan_tau', joint='r_shoulder_pan_joint', gear='1', ctrlrange='-30 30'
- `r_shoulder_lift_tau` (actuator/motor): name='r_shoulder_lift_tau', joint='r_shoulder_lift_joint', gear='1', ctrlrange='-200 200'
- `r_upper_arm_roll_tau` (actuator/motor): name='r_upper_arm_roll_tau', joint='r_upper_arm_roll_joint', gear='1', ctrlrange='-100 100'
- `r_elbow_flex_tau` (actuator/motor): name='r_elbow_flex_tau', joint='r_elbow_flex_joint', gear='1', ctrlrange='-100 100'
- `r_forearm_roll_tau` (actuator/motor): name='r_forearm_roll_tau', joint='r_forearm_roll_joint', gear='1', ctrlrange='-30 30'
- `r_wrist_flex_tau` (actuator/motor): name='r_wrist_flex_tau', joint='r_wrist_flex_joint', gear='1', ctrlrange='-30 30'
- `r_wrist_roll_tau` (actuator/motor): name='r_wrist_roll_tau', joint='r_wrist_roll_joint', gear='1', ctrlrange='-10 10'

### scene_contact

- `floor` (geom/geom): name='floor', type='plane', condim='3', solref='0.001 0.01', solimp='0.9 0.95 0.001'

### torso

- `torso_lift_joint` (joint/joint): name='torso_lift_joint', type='slide', range='0 0.33', actuatorfrcrange='-10000 10000', damping='20000'
- `torso_lift_tau` (actuator/motor): name='torso_lift_tau', joint='torso_lift_joint', gear='1', ctrlrange='-10000 10000'

## Risk notes and next experiments

- Keep acceptance and presentation/demo tuning separate; do not raise XML limits just to make a demo look larger without validating acceptance behavior.
- Base caster/wheel force and velocity ranges should be tested with CSV metrics before changing admittance gains.
- Very high torso damping is likely intentional hold behavior; changing it can destabilize posture and should be treated as a separate experiment.
- Compare actuator `ctrlrange` with joint `actuatorfrcrange` before assuming which MuJoCo limit is binding.
- Contact/friction parameters can dominate mobile-base translation; if floor friction is omitted, document the MuJoCo defaults used by the installed version.
