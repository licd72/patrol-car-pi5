from setuptools import find_packages, setup
from glob import glob
import os

package_name = "patrol_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "templates"), glob("templates/*.html")),
    ],
    install_requires=["setuptools", "flask"],
    zip_safe=True,
    maintainer="lichengdong",
    description="巡逻小车 Web 监控面板",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_server = patrol_web.web_server:main",
        ],
    },
)
