#!/usr/bin/env python3
"""
Standalone Point Cloud Preprocessing Launch File
For testing the preprocessing node independently
File: src/robot_pointcloud_preprocessing/launch/mapping_preprocessing.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/zed2i/zed_node/point_cloud/cloud_registered',
        description='Input point cloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/cleaned_mapping_cloud',
        description='Output cleaned point cloud topic'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable performance diagnostics'
    )
    
    # Get launch configurations
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    enable_diagnostics = LaunchConfiguration('enable_diagnostics')
    
    # Configuration file
    preprocessing_config = PathJoinSubstitution([
        FindPackageShare('robot_pointcloud_preprocessing'),
        'config', 'preprocessing_params.yaml'
    ])
    
    # Point cloud preprocessing node
    preprocessing_node = Node(
        package='robot_pointcloud_preprocessing',
        executable='mapping_preprocessor_node',
        name='mapping_preprocessor',
        output='screen',
        parameters=[
            preprocessing_config,
            {
                'input_topic': input_topic,
                'output_topic': output_topic,
                'enable_diagnostics': enable_diagnostics,
            }
        ]
    )
    
    return LaunchDescription([
        # Arguments
        input_topic_arg,
        output_topic_arg,
        enable_diagnostics_arg,
        
        # Nodes
        preprocessing_node,
    ])