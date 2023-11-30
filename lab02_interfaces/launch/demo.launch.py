from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
        ld = LaunchDescription()
        turtlesim_node = Node(
                package='turtlesim',
                executable='turtlesim_node',
                output = 'screen'
        )
        compute_trajectory=Node(
                package='lab02_pkg',
                executable='compute_trajectory',
                output='screen'
        )
        



        goal_generator=Node(
                package = 'lab02_pkg',
                executable = 'goal_generator',
                output='screen'
        )
        move_distance = Node(
                package= 'lab02_pkg',
                executable = 'move_distance',
                output = 'screen',
                #namespace='turtle1'
        )

        ld.add_action(turtlesim_node)
        ld.add_action(compute_trajectory)
        ld.add_action(goal_generator)
        ld.add_action(move_distance)

        return ld       