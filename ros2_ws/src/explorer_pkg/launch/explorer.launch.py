#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'explorer_pkg'  
    package_share_directory = get_package_share_directory(package_name)
    print(package_share_directory)
    config_file_path = os.path.join(
        package_share_directory,
        'config',
        'explorator_params.yaml'
    )
    print(config_file_path)
    
    frontier_scanner_node = Node(
        package=package_name,
        executable='frontier_scanner',
        name='frontier_scanner',  # deve combaciare con la sezione yaml
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[config_file_path],
    )
    
    goal_manager_node = Node(
        package=package_name,
        executable='goal_manager',
        name='goal_manager',  # deve combaciare con la sezione yaml
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[config_file_path],
    )
    
    return LaunchDescription([
        frontier_scanner_node,
        goal_manager_node
    ])
