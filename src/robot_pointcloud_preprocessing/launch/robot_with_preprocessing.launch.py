#!/usr/bin/env python3
"""
Robot with ZED2i camera and point cloud preprocessing launch file
Integrates preprocessing pipeline with existing robot_with_zed2i launch

File: src/robot_pointcloud_preprocessing/launch/robot_with_preprocessing.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for preprocessing visualization'
    )
    
    use_robot_arg = DeclareLaunchArgument(
        'use_robot',
        default_value='true',
        description='Whether to launch full robot system with ZED2i'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Yahboom hardware'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot = LaunchConfiguration('use_robot')
    serial_port = LaunchConfiguration('serial_port')
    
    # Include the existing robot_with_zed2i launch file (without its RViz)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch', 'robot_with_zed2i.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false',  # We'll use our own RViz config
            'serial_port': serial_port
        }.items(),
        condition=IfCondition(use_robot)
    )
    
    # Point Cloud Preprocessing configuration
    preprocessing_config = PathJoinSubstitution([
        FindPackageShare('robot_pointcloud_preprocessing'),
        'config', 'preprocessing_params.yaml'
    ])
    
    # Point Cloud Preprocessor Node
    preprocessor_node = Node(
        package='robot_pointcloud_preprocessing',
        executable='pointcloud_preprocessor',
        name='pointcloud_preprocessor',
        output='screen',
        parameters=[preprocessing_config],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # RViz for preprocessing visualization
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
        # condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_robot_arg,
        serial_port_arg,
        
        # Robot system (optional)
        robot_launch,
        
        # Preprocessing pipeline
        preprocessor_node,
        rviz_node,
    ])