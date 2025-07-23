#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Simple two-step navigation: rotate then move forward"""
    
    # Robot launch with fixed wheel collision
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('my_robot_bringup'),
            '/launch/my_robot_launch.py'
        ])
    )
    
    # Robot 1: GPS simulator and move server
    
    robot_1_move_server = Node(
        package='navigation', 
        executable='simple_move_robot_server',
        name='robot_1_simple_move_server',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.0,     # Reduced from 1.0 to be less aggressiveive
        }]
    )
    
    # Robot 2: GPS simulator and move server
    
    robot_2_move_server = Node(
        package='navigation',
        executable='simple_move_robot_server',
        name='robot_2_simple_move_server',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.0,     # Reduced from 1.0 to be less aggressive
        }]
    )
    
    return LaunchDescription([
        robot_launch,
        robot_1_move_server,
        robot_2_move_server,
    ])
