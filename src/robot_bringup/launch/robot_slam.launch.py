#!/usr/bin/env python3
"""
Robot SLAM launch file
Combines robot with ZED2i camera and slam_toolbox for mapping
File: src/robot_bringup/launch/robot_slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Yahboom hardware'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Configuration files
    slam_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'slam_params.yaml'
    ])
    
    # Robot with ZED2i launch (without RViz to avoid conflicts)
    robot_with_zed2i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch', 'robot_with_zed2i.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false',  # Always disable RViz in the included launch file
            'serial_port': serial_port,
        }.items()
    )
    
    # SLAM Toolbox (Online SLAM)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )
    
    # Map Saver Service
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
        }]
    )
    
    # RViz with SLAM visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'slam_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        # condition=IfCondition(use_rviz)
    )
    
    # Log info about SLAM setup
    slam_info = LogInfo(
        msg=['Starting SLAM with ZED2i. Drive the robot manually to build a map!']
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        serial_port_arg,
        use_sim_time_arg,
        
        # Info
        slam_info,
        
        # Robot system with ZED2i
        robot_with_zed2i,
        
        # SLAM
        slam_toolbox_node,
        map_saver_server,
        
        # Visualization
        rviz_node,
    ])