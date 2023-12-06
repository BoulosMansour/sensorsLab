from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        ld = LaunchDescription()
        config=os.path.join(
                get_package_share_directory('lab03_interfaces'),
                'config',
                'parameters.yaml'
        )
        compute_trajectory = Node(
                package='lab03_pkg',
                executable='compute_trajectory',
                output = 'screen'
        )
        goal_generator=Node(
                package='lab03_pkg',
                output='screen',
                executable='goal_generator'
        )
        move_forward=Node(
                package='lab03_pkg',
                executable='move_forward',
                output='screen'
        )
        rotate_absolute=Node(
                package='lab03_pkg',
                executable='rotate_absolute',
                output='screen'
        )
        start_trajectory=Node(
                package='lab03_pkg',
                executable='start_trajectory',
                output='screen'
        )
        ld.add_action(rotate_absolute)
        ld.add_action(move_forward)
        ld.add_action(goal_generator)
        ld.add_action(compute_trajectory)
        ld.add_action(start_trajectory)

        return ld       
