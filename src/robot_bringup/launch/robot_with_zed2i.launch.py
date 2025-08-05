#!/usr/bin/env python3
"""
Robot with ZED2i camera launch file
Combines robot hardware with ZED2i camera and depth-to-laserscan conversion
File: src/robot_bringup/launch/robot_with_zed2i.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed2i',
        description='Name of the ZED2i camera'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    camera_name = LaunchConfiguration('camera_name')
    
    # Configuration files
    hardware_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'hardware.yaml'
    ])
    
    zed_common_config = PathJoinSubstitution([
        FindPackageShare('zed_wrapper'),
        'config', 'common_stereo.yaml'
    ])
    
    zed_camera_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'zed2i_robot_config.yaml'
    ])
    
    
    depth_to_scan_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'zed_depth_to_scan.yaml'
    ])
    
    # Robot description with ZED2i camera
    robot_description_content = ParameterValue(
        Command(['xacro ', 
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'urdf', 'yahboomcar_with_zed2i.urdf.xacro'
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
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # PS4 Controller for joy input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Enhanced Yahboom joystick controller with gear modes and RGB control
    yahboom_joystick = Node(
        package='robot_teleop',
        executable='yahboom_joystick',
        name='joystick_controller',
        output='screen'
    )
    
    # Yahboom hardware driver
    yahboom_driver = Node(
        package='robot_hardware',
        executable='yahboom_driver', 
        name='yahboom_hardware_driver',
        output='screen',
        parameters=[
            hardware_config,
            {
                'serial_port': serial_port,
                'publish_odom_tf': False,  # ZED handles visual odometry
                'use_sim_time': False
            }
        ]
    )
    
    # Debug: Log the config file being used
    debug_config_info = LogInfo(
        msg=['Using ZED2i custom config: ', zed_camera_config]
    )
    
    # ZED2i Camera Launch (using existing launch file)
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
            'publish_urdf': 'false',          # Robot URDF already includes camera
            'publish_tf': 'false',            # Disable TF publishing to avoid conflicts
            'publish_map_tf': 'false',        # Disable map TF publishing  
            'ros_params_override_path': zed_camera_config,
        }.items()
    )
    
    # Depth to LaserScan Node (separate from ZED container)
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_laserscan',
        parameters=[
            depth_to_scan_config,
            {
                'output_frame': 'zed2i_left_camera_frame'
            }
        ],
        remappings=[
            ('depth', [camera_name, '/zed_node/depth/depth_registered']),
            ('depth_camera_info', [camera_name, '/zed_node/depth/camera_info']),
            ('scan', '/scan')
        ],
        output='screen'
    )
    
    # ZED2i will publish:
    # 1. odom->base_link transform (visual-inertial odometry)  
    # 2. Internal camera transforms (camera_link->camera_center->camera_frames)
    # Robot URDF provides the complete integrated description
    
    # RViz with robot visualization (use existing config to avoid crashes)
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
        serial_port_arg,
        camera_name_arg,
        
        # Debug
        debug_config_info,
        
        # Robot system
        robot_state_publisher,
        joint_state_publisher, 
        yahboom_driver,
        joy_node,
        yahboom_joystick,
        
        # Camera system
        zed_camera_launch,
        depth_to_scan_node,
        
        # Visualization
        rviz_node,
    ])