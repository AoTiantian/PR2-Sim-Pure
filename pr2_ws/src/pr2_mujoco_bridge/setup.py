from setuptools import find_packages, setup

package_name = "pr2_mujoco_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README_WBC_STACK.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/pr2_mujoco_sim.launch.py",
                "launch/pr2_left_arm_ik.launch.py",
                "launch/pr2_ee_pose.launch.py",
                "launch/pr2_mobile_manipulator_stack.launch.py",
                "launch/pr2_admittance_validation.launch.py",
                "launch/pr2_admittance_validation_omni.launch.py",
                # Phase 1: 单臂 X 轴力控
                "launch/pr2_arm_force_1d.launch.py",
                # Phase 2: 单臂 3D 力控
                "launch/pr2_arm_force_3d.launch.py",
                # Phase 3: 全身力控
                "launch/pr2_whole_body_force.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="maintainer@example.com",
    description="PR2 MuJoCo ROS 2 bridge",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pr2_mujoco_sim = pr2_mujoco_bridge.pr2_sim_ros:main",
            "pr2_left_arm_ik = pr2_mujoco_bridge.pr2_left_arm_ik:main",
            "pr2_ee_pose_publisher = pr2_mujoco_bridge.pr2_ee_pose_publisher:main",
            "pr2_state_estimator = pr2_mujoco_bridge.pr2_state_estimator:main",
            "pr2_wbc_coordinator = pr2_mujoco_bridge.pr2_wbc_coordinator:main",
            "pr2_base_accel_integrator = pr2_mujoco_bridge.pr2_base_accel_integrator:main",
            "pr2_admittance_controller = pr2_mujoco_bridge.pr2_admittance_stub:main",
            "pr2_admittance_stub = pr2_mujoco_bridge.pr2_admittance_stub:main",
            "pr2_admittance_validator = pr2_mujoco_bridge.pr2_admittance_validator:main",
            "pr2_null_space_stub = pr2_mujoco_bridge.pr2_null_space_stub:main",
            # Phase 1: 臂顺应控制
            "pr2_arm_admittance = pr2_mujoco_bridge.pr2_arm_admittance:main",
            "pr2_arm_force_injector = pr2_mujoco_bridge.pr2_arm_force_injector:main",
            # Phase 3: 全身力控
            "pr2_force_projector = pr2_mujoco_bridge.pr2_force_projector:main",
        ],
    },
)
