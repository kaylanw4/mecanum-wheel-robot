#!/usr/bin/env python3
"""
Test launch file for VI-SLAM URDF validation
"""

import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'yahboomcar_vi_slam.urdf.xacro')
    
    # Process the URDF file
    robot_description = Command([
        'xacro ', urdf_file
    ])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher (for wheel joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])