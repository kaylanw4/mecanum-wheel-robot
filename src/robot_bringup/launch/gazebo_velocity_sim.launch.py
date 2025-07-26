#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='yahboom_robot',
        description='Name of the robot'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='empty.sdf',
        description='Gazebo world file to load'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')
    
    # Robot description from XACRO - using velocity movement URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', 
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'urdf', 'yahboomcar_chassis_velocity.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    
    # Launch Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch', 'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments=[
            ('ign_args', ['-r -v4 ', world_file])
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher for basic wheel visualization
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ]
    )
    
    # CRITICAL BRIDGE: Maps /cmd_vel to the VelocityControl plugin topic
    cmd_vel_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/cmd_vel', '/model/yahboom_robot/link/base_link/cmd_vel')
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        world_file_arg,
        
        # Launch Gazebo simulation
        gazebo,
        
        # Launch robot description and visualization
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        cmd_vel_bridge,
        clock_bridge,
    ])