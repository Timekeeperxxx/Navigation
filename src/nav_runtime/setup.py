from glob import glob
from setuptools import find_packages, setup

package_name = "nav_runtime"

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
    description="导航运行时话题封装与兼容脚本调度节点",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "runtime_node = nav_runtime.runtime_node:main",
        ],
    },
)
