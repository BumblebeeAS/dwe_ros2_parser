import os
from glob import glob
from setuptools import setup

package_name = 'dwe_ros2_parser'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package with launch files',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your entry points here if needed
        ],
    },
)
