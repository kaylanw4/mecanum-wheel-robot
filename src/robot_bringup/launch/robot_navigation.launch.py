#!/usr/bin/env python3
"""
ZED Navigation Mode Launch File - Industry Standard
NAVIGATION PHASE: Full nav2 stack with ZED localization for autonomous navigation
File: src/robot_bringup/launch/robot_navigation.launch.py
"""

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
        description='Whether to launch RViz for navigation visualization'
    )
    
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Whether to enable joystick control during navigation'
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
    use_joystick = LaunchConfiguration('use_joystick')
    serial_port = LaunchConfiguration('serial_port')
    camera_name = LaunchConfiguration('camera_name')
    
    # Configuration files
    hardware_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'hardware.yaml'
    ])
    
    zed_navigation_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'zed_navigation_config.yaml'
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
    
    # PS4 Controller for joy input (conditional - for manual override during navigation)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(use_joystick)
    )
    
    # Enhanced Yahboom joystick controller (conditional - for manual override)
    yahboom_joystick = Node(
        package='robot_teleop',
        executable='yahboom_joystick',
        name='joystick_controller',
        output='screen',
        condition=IfCondition(use_joystick)
    )
    
    # Yahboom hardware driver - FULL CONTROL MODE for navigation
    yahboom_driver = Node(
        package='robot_hardware',
        executable='yahboom_driver', 
        name='yahboom_hardware_driver',
        output='screen',
        parameters=[
            hardware_config,
            {
                'serial_port': serial_port,
                'publish_odom_tf': False,   # ZED handles primary odometry
                'publish_odom': True,       # Publish wheel odometry for sensor fusion
                'use_sim_time': False
            }
        ]
    )
    
    # Debug: Log the configuration being used
    debug_config_info = LogInfo(
        msg=['Using ZED Navigation config: ', zed_navigation_config]
    )
    
    # ZED2i Camera with NAVIGATION mode (localization + obstacle detection)
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
            'ros_params_override_path': zed_navigation_config,
        }.items()
    )
    
    # Depth image to laser scan conversion for nav2 obstacle avoidance
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'config', 'zed_depth_to_scan.yaml'
            ])
        ],
        remappings=[
            ('depth', '/zed2i/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed2i/zed_node/depth/camera_info'),
            ('scan', '/scan')
        ]
    )
    
    # RViz with navigation visualization (conditional)  
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'robot_with_zed2i.rviz'  # Use existing config for now
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
    
    # TODO: Add nav2 launch file inclusion here
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('nav2_bringup'),
    #             'launch', 'navigation_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': 'false',
    #         'params_file': nav2_params_file,
    #         'map': map_file,
    #     }.items()
    # )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_joystick_arg,
        serial_port_arg,
        camera_name_arg,
        
        # Debug information
        debug_config_info,
        
        # Robot system
        robot_state_publisher,
        joint_state_publisher, 
        yahboom_driver,
        
        # Joystick control (conditional - for manual override)
        joy_node,
        yahboom_joystick,
        
        # ZED NAVIGATION (localization + obstacle detection)
        zed_camera_launch,
        
        # Depth to laser scan for nav2 obstacle avoidance
        depthimage_to_laserscan,
        
        # Visualization
        rviz_node,
        
        # TODO: Add nav2 launch when ready
        # nav2_launch,
    ])