from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        return LaunchDescription([
                Node(
                    package='turtlesim',
                    namespace='turtle1',
                    executable='turtlesim_node',
                    name='sim'
                ),
                Node(
                    package='turtlesim',
                    executable='turtle_teleop_key',
                    name='turtlesim_teleop_key',
                    output='screen',
                    emulate_tty=True,
                    prefix='xterm -e',
                ),
                Node(
                    package='lab02_pkg',
                    executable='goal_generator',
                    name='goal_generator'
                ),
                Node(
                    package='lab02_pkg',
                    executable='move_distance',
                    namespace='turtle1',
                    name='move_distance'
                )
        ])