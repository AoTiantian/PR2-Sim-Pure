from glob import glob

from setuptools import find_packages, setup

package_name = "pr2_mujoco_bridge"

launch_files = sorted(glob("launch/*.launch.py"))
doc_files = [
    path
    for path in ["README.md", "README_IK.md", "README_ACCEPTANCE_FEAT.md"]
    if glob(path)
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"] + doc_files),
        ("share/" + package_name + "/launch", launch_files),
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
            "pr2_wbc_coordinator = pr2_mujoco_bridge.pr2_wbc_coordinator:main",
            "pr2_arm_admittance = pr2_mujoco_bridge.pr2_arm_admittance:main",
            "pr2_arm_force_injector = pr2_mujoco_bridge.pr2_arm_force_injector:main",
            "pr2_force_projector = pr2_mujoco_bridge.pr2_force_projector:main",
            "pr2_base_admittance = pr2_mujoco_bridge.pr2_base_admittance:main",
        ],
    },
)
