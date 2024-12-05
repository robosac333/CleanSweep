from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from datetime import datetime
import os
import random

def generate_launch_description():
    # Declare package paths
    clean_sweep_pkg = FindPackageShare('cleansweep')
    turtlebot3_gazebo_pkg = FindPackageShare('turtlebot3_gazebo')

    # Get the world file path
    world_file_path = PathJoinSubstitution([
        clean_sweep_pkg,
        'worlds',
        'empty_warehouse.world'
    ])
    
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
    spawn_turtlebot_node = TimerAction(
        period=5.0,
        actions=[Node(
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
        )]
    )

    # Create spawn actions for multiple coke cans
    spawn_cans_actions = []
    for i in range(20):  # Spawn 5 cans
        x = random.uniform(-4.0, 4.0)
        y = random.uniform(-4.0, 4.0)
        
        spawn_can = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'coke_can_{i}',  # Unique name for each can
                '-file', PathJoinSubstitution([
                    clean_sweep_pkg,
                    'models',
                    'coke_can',
                    'model.sdf'
                ]),
                '-x', str(x),
                '-y', str(y),
                '-z', '0.0'
            ],
            output='screen'
        )
        spawn_cans_actions.append(spawn_can)

    # Group all can spawning actions together with a delay
    spawn_cans_group = TimerAction(
        period=8.0,
        actions=[GroupAction(spawn_cans_actions)]
    )

    # Launch the walker node
    walker_node = TimerAction(
        period=10.0,
        actions=[Node(
            package='walker',
            executable='robot_control',
            name='walker',
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        spawn_turtlebot_node,
        spawn_cans_group,
        walker_node
    ])