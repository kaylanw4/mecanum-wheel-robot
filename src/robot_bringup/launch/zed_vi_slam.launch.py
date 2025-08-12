#!/usr/bin/env python3
"""
Phase 2: ZED VI-SLAM Positional Tracking Launch
Starts ZED2i with positional tracking for VI-SLAM integration
File: src/robot_bringup/launch/zed_vi_slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed2i',
        description='Name of the ZED2i camera'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz for visualization'
    )
    
    # Get launch configurations
    camera_name = LaunchConfiguration('camera_name')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Configuration files
    zed_vi_slam_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'zed_vi_slam_config.yaml'
    ])
    
    # Debug: Log the configuration being used
    debug_config_info = LogInfo(
        msg=['Starting ZED VI-SLAM with config: ', zed_vi_slam_config]
    )
    
    # ZED2i Camera with VI-SLAM positional tracking
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch', 'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': camera_name,
            'publish_urdf': 'false',  # Don't publish URDF (robot handles this)
            'ros_params_override_path': zed_vi_slam_config,
        }.items()
    )
    
    # RViz for VI-SLAM visualization (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'robot_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': False
        }],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        camera_name_arg,
        use_rviz_arg,
        
        # Debug information
        debug_config_info,
        
        # ZED VI-SLAM
        zed_camera_launch,
        
        # Optional visualization
        rviz_node,
    ])