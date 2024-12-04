from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from datetime import datetime
import os

def generate_launch_description():
    # Declare package paths
    pkg_share = FindPackageShare('cleansweep')
    turtlebot3_gazebo_pkg = FindPackageShare('turtlebot3_gazebo')

    # Get the world file path - updated to match the install directory structure
    world_file_path = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'empty_warehouse.world'
    ])

    # Declare the robot model argument
    model = LaunchConfiguration('model', default='waffle_pi')

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true'
        }.items()
    )

    # Spawn Turtlebot
    spawn_turtlebot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'models',
                'turtlebot3_waffle_pi',
                'model.sdf'
            ])
        ],
        output='screen'
    )

    # Launch the robot control node - updated to match executable name in CMakeLists.txt
    robot_control_node = Node(
        package='cleansweep',
        executable='robot_control',
        name='cleansweep',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_turtlebot_node,
        robot_control_node
    ])