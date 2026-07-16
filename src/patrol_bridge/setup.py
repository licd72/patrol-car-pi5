from setuptools import setup

package_name = "patrol_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lichengdong",
    description="底盘桥 /cmd_vel → Rosmaster",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_bridge = patrol_bridge.cmd_vel_bridge:main",
        ],
    },
)
