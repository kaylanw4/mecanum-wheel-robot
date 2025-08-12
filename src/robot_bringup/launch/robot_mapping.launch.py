#!/usr/bin/env python3
"""
Phase 4: ZED Spatial Mapping Mode Launch File
Enables ZED2i spatial mapping for real-time map building and loop closure
File: src/robot_bringup/launch/robot_mapping.launch.py
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
        description='Whether to launch RViz for mapping visualization'
    )
    
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Whether to enable joystick control for mapping'
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
    
    zed_mapping_config = PathJoinSubstitution([
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
    
    # PS4 Controller for joy input (conditional)
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
    
    # Enhanced Yahboom joystick controller (conditional)
    yahboom_joystick = Node(
        package='robot_teleop',
        executable='yahboom_joystick',
        name='joystick_controller',
        output='screen',
        condition=IfCondition(use_joystick)
    )
    
    # Yahboom hardware driver with DISABLED wheel odometry
    yahboom_driver = Node(
        package='robot_hardware',
        executable='yahboom_driver', 
        name='yahboom_hardware_driver',
        output='screen',
        parameters=[
            hardware_config,
            {
                'serial_port': serial_port,
                'publish_odom_tf': False,   # ZED handles TF
                'publish_odom': False,      # ZED provides odometry
                'use_sim_time': False
            }
        ]
    )
    
    # Debug: Log the configuration being used
    debug_config_info = LogInfo(
        msg=['Using ZED Mapping config: ', zed_mapping_config]
    )
    
    # ZED2i Camera with SPATIAL MAPPING enabled
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
            'ros_params_override_path': zed_mapping_config,
        }.items()
    )
    
    # RViz with mapping visualization (conditional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'mapping_view.rviz'
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
        use_joystick_arg,
        serial_port_arg,
        camera_name_arg,
        
        # Debug information
        debug_config_info,
        
        # Robot system
        robot_state_publisher,
        joint_state_publisher, 
        yahboom_driver,
        
        # Joystick control (conditional)
        joy_node,
        yahboom_joystick,
        
        # ZED MAPPING (primary odometry + spatial mapping)
        zed_camera_launch,
        
        # Visualization
        rviz_node,
    ])