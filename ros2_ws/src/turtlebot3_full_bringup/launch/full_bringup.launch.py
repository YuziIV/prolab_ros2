from sympy import true
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import launch
from launch.utilities import perform_substitutions

def generate_launch_description():
    pkg_bringup = FindPackageShare('turtlebot3_full_bringup').find('turtlebot3_full_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Define paths
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')
    #map_file = os.path.join(pkg_bringup, 'maps', 'playground_map.yaml')
    #map_file = LaunchConfiguration('map', default=os.path.join(pkg_bringup, 'maps', 'playground_map.yaml'))
    #map_file = LaunchConfiguration('map', default = os.path.expanduser('/home/ubuntu/ros2_ws/src/turtlebot3_full_bringup/maps/playground_map.yaml'))
    #map_file = "/home/ubuntu/ros2_ws/src/turtlebot3_full_bringup/maps/playground_map.yaml"
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/ros2_ws/src/turtlebot3_full_bringup/maps/playground_map.yaml'))
    print (f"Map file path: {map_file}")
    dummy_context = launch.LaunchContext()
    resolved_map_path = perform_substitutions(dummy_context, [map_file])
    print("✅ Resolved map path:", resolved_map_path)
    # Gazebo Launch
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch')

    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')
    # Robot State Publisher
    robostate_package = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    robostate_launch = os.path.join(robostate_package,'launch','robot_state_publisher.launch.py')
    robotspawn_launch = os.path.join(robostate_package, 'launch', 'spawn_turtlebot3.launch.py')
    
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    
    #NODES
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time}.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzclient.launch.py')
        ),
    )
    
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robotspawn_launch)
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robostate_launch),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items()
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
        rviz,
        nav2,
        ])
