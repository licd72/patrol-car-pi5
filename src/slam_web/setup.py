from setuptools import setup
import os
from glob import glob

package_name = 'slam_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/templates', glob('templates/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='licd72',
    maintainer_email='licd72@github.com',
    description='SLAM mapping web control panel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = slam_web.web_server:main',
        ],
    },
)
