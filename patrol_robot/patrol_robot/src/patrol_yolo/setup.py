from setuptools import find_packages, setup
import os
from glob import glob

package_name = "patrol_yolo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 包含 launch 文件
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        # 包含配置文件
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lichengdong",
    maintainer_email="licd72@example.com",
    description="巡逻小车 YOLO 异常目标检测节点 (RPi5 优化)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_detector = patrol_yolo.yolo_detector:main",
        ],
    },
)
