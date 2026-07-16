from setuptools import find_packages, setup
from glob import glob
import os

package_name = "patrol_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="lichengdong",
    description="巡逻车语音交互",
    license="MIT",
    entry_points={
        "console_scripts": [
            "voice_node = patrol_voice.voice_node:main",
        ],
    },
)
