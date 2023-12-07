from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
        ld = LaunchDescription()
        config=os.path.join(
                 get_package_share_directory('lab02_interfaces'),
                 'config',
                 'parameters.yaml'
        )
        turtlesim_node = Node(
                package='turtlesim',
                executable='turtlesim_node',
                parameters=[config]
        )
        compute_trajectory=Node(
                package='lab02_pkg',
                executable='compute_trajectory',
        )

        goal_generator=Node(
                package = 'lab02_pkg',
                executable = 'goal_generator',
        )
        move_distance = Node(
                package= 'lab02_pkg',
                executable = 'move_distance',
                parameters=[config]
        )
        teleop_key = Node(
                package='turtlesim',
                executable='turtle_teleop_key',
                prefix=['xterm -e'],
                parameters=[config],
        )

        ld.add_action(turtlesim_node)
        ld.add_action(compute_trajectory)
        ld.add_action(goal_generator)
        ld.add_action(move_distance)
        ld.add_action(teleop_key)

        return ld