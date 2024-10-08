from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2bag_write_benchmark'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/benchmarker_launch.py']),
        ('share/' + package_name + '/params', ['params/benchmarker_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuxw',
    maintainer_email='yuxw@udel.edu',
    description='ROS 2 package for benchmarking ros2bag recording operations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchmarker = ros2bag_write_benchmark.benchmarker:main',
        ],
    },
)
