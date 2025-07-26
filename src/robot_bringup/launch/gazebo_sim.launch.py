#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
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
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    
    # Robot description from XACRO - properly wrapped for ROS2 launch
    robot_description_content = ParameterValue(
        Command(['xacro ', 
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'urdf', 'yahboomcar_chassis.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    
    # FIXED: Use Ignition Gazebo (not newer Gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),  # Changed from ros_gz_sim
                'launch', 'ign_gazebo.launch.py'      # Changed from gz_sim.launch.py
            ])
        ]),
        launch_arguments=[
            ('ign_args', '-r -v4 empty.sdf')  # Changed from gz_args
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
    
    # FIXED: Spawn robot in Ignition Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',  # Changed from ros_gz_sim
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ]
    )
    
    # FIXED: Bridge between Ignition Gazebo and ROS2
    bridge = Node(
        package='ros_ign_bridge',  # Changed from ros_gz_bridge
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'  # Changed from gz.msgs.Clock
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'config', 'ros2_controllers.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Delay controller spawning to allow Gazebo to fully load
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'controller_manager', 'spawner', 
                     'joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    mecanum_drive_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'controller_manager', 'spawner', 
                     'mecanum_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        
        # Launch Ignition Gazebo
        gazebo,
        
        # Launch robot description and control
        robot_state_publisher,
        spawn_entity,
        bridge,
        controller_manager,
        
        # Spawn controllers (with delay)
        joint_state_broadcaster_spawner,
        mecanum_drive_controller_spawner,
    ])