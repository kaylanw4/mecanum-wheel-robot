#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    joy_params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'joystick_velocity.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/cmd_vel')] 
        )
    ])