#!/usr/bin/env python3
"""
Point Cloud Preprocessing Launch File
Phase 1: Basic infrastructure with visualization

File: src/robot_pointcloud_preprocessing/launch/pointcloud_preprocessing.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='preprocessing_params.yaml',
        description='Preprocessing configuration file'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    config_file = LaunchConfiguration('config_file')
    
    # Configuration file path
    config_path = PathJoinSubstitution([
        FindPackageShare('robot_pointcloud_preprocessing'),
        'config',
        config_file
    ])
    
    # Point Cloud Preprocessor Node
    preprocessor_node = Node(
        package='robot_pointcloud_preprocessing',
        executable='pointcloud_preprocessor',
        name='pointcloud_preprocessor',
        output='screen',
        parameters=[config_path],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # RViz for visualization (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_pointcloud_preprocessing'),
        'config', 'preprocessing_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_preprocessing',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': False
        }],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        config_file_arg,
        
        # Nodes
        preprocessor_node,
        rviz_node,
    ])