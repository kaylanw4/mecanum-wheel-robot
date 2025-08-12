#!/usr/bin/env python3
"""
Phase 1 Test Launch: VI-SLAM URDF and TF Tree Validation
Tests the new VI-SLAM URDF structure and coordinate frame setup
File: src/robot_bringup/launch/robot_vi_slam_test.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for TF visualization'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed2i',
        description='Name of the ZED2i camera'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    camera_name = LaunchConfiguration('camera_name')
    
    # Configuration files
    zed_vi_slam_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'zed_vi_slam_config.yaml'
    ])
    
    # Robot description with VI-SLAM URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', 
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'urdf', 'yahboomcar_vi_slam.urdf.xacro'
            ]),
            ' camera_name:=', camera_name
        ]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher for wheel joints visualization
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # Debug: Log the URDF and config being used
    debug_urdf_info = LogInfo(
        msg=['Testing VI-SLAM URDF: yahboomcar_vi_slam.urdf.xacro']
    )
    
    debug_config_info = LogInfo(
        msg=['Using VI-SLAM config: ', zed_vi_slam_config]
    )
    
    # ZED2i Camera with VI-SLAM configuration
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
            'publish_urdf': 'false',  # Use our custom VI-SLAM URDF
            'ros_params_override_path': zed_vi_slam_config,
        }.items()
    )
    
    # RViz configuration for TF tree visualization
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
        use_rviz_arg,
        camera_name_arg,
        
        # Debug information
        debug_urdf_info,
        debug_config_info,
        
        # Core components
        robot_state_publisher,
        joint_state_publisher,
        
        # ZED VI-SLAM
        zed_camera_launch,
        
        # Visualization
        rviz_node,
    ])