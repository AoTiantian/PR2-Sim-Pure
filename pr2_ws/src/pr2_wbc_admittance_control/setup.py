from setuptools import find_packages, setup

package_name = "pr2_wbc_admittance_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools", "mujoco", "numpy", "osqp", "scipy", "matplotlib"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="maintainer@example.com",
    description="PR2 whole-body QP admittance control nodes",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pr2_ee_pose_publisher = pr2_wbc_admittance_control.pr2_ee_pose_publisher:main",
            "pr2_state_estimator = pr2_wbc_admittance_control.pr2_state_estimator:main",
            "pr2_wbc_coordinator = pr2_wbc_admittance_control.pr2_wbc_coordinator:main",
            "pr2_motion_logger = pr2_wbc_admittance_control.pr2_motion_logger:main",
            "pr2_qp_whole_body_admittance = pr2_wbc_admittance_control.pr2_qp_whole_body_admittance:main",
        ],
    },
)
