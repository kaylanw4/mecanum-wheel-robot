# ===== robot_navigation.launch.py =====
#!/usr/bin/env python3
"""
Complete robot navigation launch file
File: src/robot_bringup/launch/robot_navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_bringup'),
            'maps', 'simple_room.yaml'
        ]),
        description='Map file to use for navigation'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Navigation configuration file
    nav2_config = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'nav2_params.yaml'
    ])
    
    # Robot bringup (hardware + description)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch', 'robot_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false',  # We'll launch RViz separately with nav config
        }.items()
    )
    
    # Nav2 lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }]
    )
    
    # AMCL (Adaptive Monte Carlo Localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config]
    )
    
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config]
    )
    
    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config]
    )
    
    # Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config]
    )
    
    # RViz with navigation configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'nav2_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        map_file_arg,
        use_rviz_arg,
        
        # Robot system
        robot_bringup,
        
        # Navigation nodes
        lifecycle_manager,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        
        # Visualization
        rviz_node,
    ])


# ===== odometry_test.launch.py =====
#!/usr/bin/env python3
"""
Odometry accuracy testing launch file
File: src/robot_bringup/launch/odometry_test.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for odometry visualization'
    )
    
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Robot bringup with enhanced odometry
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch', 'robot_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )
    
    # RViz for odometry visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 
        'config', 'odometry_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )
    
    # Odometry test node
    odom_test_node = Node(
        package='robot_bringup',
        executable='odometry_test',
        name='odometry_test',
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_bringup,
        rviz_node,
        # odom_test_node,  # Uncomment when you create the test node
    ])