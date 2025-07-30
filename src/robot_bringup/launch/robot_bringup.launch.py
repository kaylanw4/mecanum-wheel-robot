#!/usr/bin/env python3
"""
Main launch file for Yahboom mecanum wheel robot
Launches hardware driver, robot description, and optionally joystick control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Launch arguments
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Whether to launch joystick control'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Yahboom hardware'
    )
    
    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Whether hardware driver should publish odom->base_link TF'
    )
    
    # Get launch configurations
    use_joystick = LaunchConfiguration('use_joystick')
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    
    # FIXED: Load hardware configuration file
    hardware_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'hardware.yaml'
    ])
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', 
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'urdf', 'yahboomcar_chassis.urdf.xacro'
            ])
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
    
    # Joint state publisher (for visualization when no hardware encoders)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # FIXED: Yahboom hardware driver with config file loading
    yahboom_driver = Node(
        package='robot_hardware',
        executable='yahboom_driver',
        name='yahboom_hardware_driver',
        output='screen',
        parameters=[
            hardware_config,  # Load hardware.yaml config file FIRST
            {
                # Launch arguments override config file values
                'serial_port': serial_port,
                'publish_odom_tf': publish_odom_tf,
                'use_sim_time': False
            }
        ]
    )
    
    # Joy node (PS4 controller input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
            'use_sim_time': False
        }],
        condition=IfCondition(use_joystick)
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
        }],
        condition=IfCondition(use_joystick)
    )
    
    # RViz (optional)
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
        use_joystick_arg,
        use_rviz_arg,
        serial_port_arg,
        publish_odom_tf_arg,
        
        # Core robot nodes
        robot_state_publisher,
        joint_state_publisher,
        yahboom_driver,
        
        # Joystick control (conditional)
        joy_node,
        yahboom_joystick,
        
        # Visualization (conditional)
        rviz_node,
    ])