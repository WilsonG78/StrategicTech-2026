from setuptools import find_packages, setup

package_name = "uav_raspberry"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/uav.launch.py",
            "launch/test_vision_launch.py",
            "launch/full_mission_launch.py",
        ]),
        ("share/" + package_name + "/config", ["config/mission_params.yaml"]),
    ],
    install_requires=[
        "setuptools",
        "mavsdk",
        "numpy",
        "rich",
        "matplotlib",
    ],
    zip_safe=True,
    maintainer="StrategicTech",
    maintainer_email="strategic.tech.pl@gmail.com",
    description=("UAV Raspberry Pi mission stack: MAVSDK telemetry bridge, "
                 "two-phase reconnaissance (lawnmower scan + target identify) "
                 "with live CV detection, YOLO classification, QR decode, "
                 "and a rich terminal GUI."),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "telemetry_node = uav_raspberry.telemetry_node:main",
            "run_mission    = uav_raspberry.run_mission:main",
            "mission_gui    = uav_raspberry.gui.terminal_gui:main",
            "test_vision    = uav_raspberry.tools.test_vision:main",
        ],
    },
)
