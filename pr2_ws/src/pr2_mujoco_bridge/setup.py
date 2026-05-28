from setuptools import find_packages, setup

package_name = "pr2_mujoco_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            ["launch/pr2_qp_whole_body_admittance.launch.py"],
        ),
    ],
    install_requires=["setuptools", "mujoco", "numpy", "osqp", "scipy", "matplotlib", "glfw"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="maintainer@example.com",
    description="PR2 MuJoCo ROS 2 bridge with QP whole-body admittance control",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pr2_mujoco_sim = pr2_mujoco_bridge.pr2_sim_ros:main",
            "pr2_ee_pose_publisher = pr2_mujoco_bridge.pr2_ee_pose_publisher:main",
            "pr2_state_estimator = pr2_mujoco_bridge.pr2_state_estimator:main",
            "pr2_wbc_coordinator = pr2_mujoco_bridge.pr2_wbc_coordinator:main",
            "pr2_arm_admittance_validator = pr2_mujoco_bridge.pr2_arm_admittance_validator:main",
            "pr2_motion_logger = pr2_mujoco_bridge.pr2_motion_logger:main",
            "pr2_qp_whole_body_admittance = pr2_mujoco_bridge.pr2_qp_whole_body_admittance:main",
        ],
    },
)
