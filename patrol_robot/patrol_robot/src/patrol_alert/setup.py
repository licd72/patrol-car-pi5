from setuptools import find_packages, setup
from glob import glob
import os

package_name = "patrol_alert"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lichengdong",
    maintainer_email="licd72@example.com",
    description="报警联动节点",
    license="MIT",
    entry_points={
        "console_scripts": [
            "alert_dispatcher = patrol_alert.alert_dispatcher:main",
        ],
    },
)
