#!/usr/bin/env python3
import sys
import yaml
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch with central server coordination"""
    
    # Robot launch with fixed wheel collision
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('my_robot_bringup'),
            '/launch/my_robot_launch.py'
        ])
    )
    
    # Robot 1: Move server
    robot_1_move_server = Node(
        package='navigation',
        executable='simple_move_robot_server',
        name='robot_1_simple_move_server',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 0.5,
        }]
    )
    
    # Robot 2: Move server
    robot_2_move_server = Node(
        package='navigation',
        executable='simple_move_robot_server',
        name='robot_2_simple_move_server',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 0.5,
        }]
    )
    
    # Central server node that coordinates everything
    central_server = Node(
        package='navigation',
        executable='central_server_node',
        name='central_server_node',
        output='screen'
    )
    
    return LaunchDescription([
        robot_launch,
        robot_1_move_server,
        robot_2_move_server,
        central_server
    ])