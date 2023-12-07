from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        ld = LaunchDescription()
        config=os.path.join(
                 get_package_share_directory('lab03_interfaces'),
                 'config',
                 'parameters.yaml'
        )
        lab03_launch_file= IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                                FindPackageShare('turtlebot3_bringup'),
                                'launch/lab03.launch.py'
                        ])
                )
                
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
                output='screen',
                parameters=[config]
        )
        rotate_absolute=Node(
                package='lab03_pkg',
                executable='rotate_absolute',
                output='screen',
                parameters=[config]
        )
        start_trajectory=Node(
                package='lab03_pkg',
                executable='start_trajectory',
                output='screen'
        )
        ld.add_action(lab03_launch_file)
        ld.add_action(start_trajectory)
        ld.add_action(rotate_absolute)
        ld.add_action(move_forward)
        ld.add_action(goal_generator)
        ld.add_action(compute_trajectory)
        
        return ld       