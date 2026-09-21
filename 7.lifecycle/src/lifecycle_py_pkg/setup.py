import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lifecycle_py_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kin",
    maintainer_email="yuunan.kin@icloud.com",
    description="ROS 2 Lifecycle Nodes and Lifecycle Publishers Python Practice Package",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "sensor_station = lifecycle_py_pkg.sensor_station:main",
            "sensor_monitor = lifecycle_py_pkg.sensor_monitor:main",
            "lifecycle_manager = lifecycle_py_pkg.lifecycle_manager:main",
        ],
    },
)
