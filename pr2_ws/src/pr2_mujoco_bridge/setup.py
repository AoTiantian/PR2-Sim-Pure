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
            "pr2_admittance_stub = pr2_mujoco_bridge.pr2_admittance_stub:main",
            "pr2_null_space_stub = pr2_mujoco_bridge.pr2_null_space_stub:main",
        ],
    },
)
