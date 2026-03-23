from setuptools import find_packages, setup

package_name = "pr2_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pr2_mujoco_sim.launch.py", "launch/pr2_left_arm_ik.launch.py", "launch/pr2_ee_pose.launch.py"]),
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
            "pr2_mujoco_sim = pr2_controller.pr2_sim_ros:main",
            "pr2_left_arm_ik = pr2_controller.pr2_left_arm_ik:main",
            "pr2_ee_pose_publisher = pr2_controller.pr2_ee_pose_publisher:main",
        ],
    },
)
