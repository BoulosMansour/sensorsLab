from itertools import tee
from sys import prefix
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    EmitEvent,
    TimerAction,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.conditions import IfCondition

from launch.substitutions import (
    ThisLaunchFileDir,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config = PathJoinSubstitution([FindPackageShare("robot_localization_project"), "config", "ekf.yaml"])
    ld=LaunchDescription()
    
    project_gazebo_launch= IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                                FindPackageShare('turtlebot3_bringup'),
                                'launch/gazebo.launch.py'
                        ])
                )
    )

    teleop_keyboard = Node(
        package="turtlebot3_teleop",
        executable="teleop_keyboard",
        output="screen",
        parameters=[config],
        prefix=['xterm -e'],
    )

    # recorder = Node(
    #     package="robot_localization_project",
    #     executable="recorder",
    #     #prefix=['xterm -e'],
    #     output='screen',
    # )

    ekf = Node(
        package="robot_localization_project",
        executable="ekf_node",
        name="filter",
        output="screen",
        parameters=[config],
        #prefix=['xterm -e'],
        remappings=[("cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    ld.add_action(project_gazebo_launch)
    ld.add_action(ekf)
    ld.add_action(teleop_keyboard)


    return ld
