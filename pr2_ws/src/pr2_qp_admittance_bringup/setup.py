from setuptools import setup

package_name = "pr2_qp_admittance_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/pr2_board_grasp_demo.launch.py",
                "launch/pr2_qp_whole_body_admittance.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="maintainer@example.com",
    description="Bringup launch files for PR2 QP whole-body admittance",
    license="Apache-2.0",
)
