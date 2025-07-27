#!/usr/bin/env python3
"""
Test launch file for joystick control only
File: src/robot_bringup/launch/joystick_test.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='Joystick device ID'
    )
    
    # Get launch configurations
    device_id = LaunchConfiguration('device_id')
    
    # Joy node (PS4 controller input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': device_id,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
            'use_sim_time': False
        }]
    )
    
    # Enhanced Yahboom joystick controller
    yahboom_joystick = Node(
        package='robot_teleop',
        executable='yahboom_joystick',
        name='yahboom_joystick_controller',
        output='screen',
        parameters=[{
            'max_linear_vel': 1.0,
            'max_angular_vel': 2.0,
            'deadzone': 0.1,
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        device_id_arg,
        joy_node,
        yahboom_joystick,
    ])