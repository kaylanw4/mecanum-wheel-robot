#!/usr/bin/env python3
"""
Simple test launch for point cloud preprocessing with RViz
Use this when ZED2i is already running from another terminal

File: src/robot_pointcloud_preprocessing/launch/test_preprocessing.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Get package share directory
    pkg_share = get_package_share_directory('robot_pointcloud_preprocessing')
    
    # Configuration files
    config_file = os.path.join(pkg_share, 'config', 'preprocessing_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'preprocessing_view.rviz')
    
    # Point Cloud Preprocessor Node
    preprocessor_node = Node(
        package='robot_pointcloud_preprocessing',
        executable='pointcloud_preprocessor',
        name='pointcloud_preprocessor',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_preprocessing',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        
        # Nodes
        preprocessor_node,
        rviz_node,
    ])