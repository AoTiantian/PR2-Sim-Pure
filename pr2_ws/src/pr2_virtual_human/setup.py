from setuptools import find_packages, setup

package_name = "pr2_virtual_human"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/human_board_demo.launch.py",
            "launch/pr2_virtual_human_demo.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="maintainer@example.com",
    description="Virtual-human force demos for standalone board and co-transport simulations",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "human_board_sim = pr2_virtual_human.human_board_sim:main",
            "pr2_virtual_human_controller = pr2_virtual_human.pr2_virtual_human_controller:main",
        ],
    },
)
