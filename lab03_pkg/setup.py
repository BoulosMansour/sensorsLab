from setuptools import find_packages, setup

package_name = 'lab03_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'compute_trajectory = lab03_pkg.compute_trajectory:main',
                'move_forward = lab03_pkg.move_forward:main',
                'rotate_absolute = lab03_pkg.rotate_absolute:main',
                'goal_generator = lab03_pkg.goal_generator:main',
                'start_trajectory = lab03_pkg.start_trajectory:main'
        ],
    },
)
