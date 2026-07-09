from glob import glob
from setuptools import find_packages, setup

package_name = "nav_robot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Navigation Maintainer",
    maintainer_email="maintainer@example.com",
    description="Switchable robot control adapters for Navigation",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "go2_webrtc_bridge = nav_robot_control.go2_webrtc_bridge:main",
            "b2_cmd_vel_bridge = nav_robot_control.b2_cmd_vel_bridge:main",
        ],
    },
)
