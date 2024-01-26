from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'robot_localization_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='boulos.mansour.1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],  
    entry_points={
        'console_scripts': [
                'ekf_node = robot_localization_project.ekf_node:main',
                'recorder = robot_localization_project.recorder:main',
        ],
    },
)
