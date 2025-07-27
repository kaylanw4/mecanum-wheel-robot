#!/usr/bin/env python3
"""
Test launch file for hardware driver only
File: src/robot_bringup/launch/hardware_test.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Yahboom hardware'
    )
    
    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    
    # Yahboom hardware driver
    yahboom_driver = Node(
        package='robot_hardware',
        executable='yahboom_driver',
        name='yahboom_hardware_driver',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'imu_frame_id': 'imu_link',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_odom_tf': False,
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        yahboom_driver,
    ])
