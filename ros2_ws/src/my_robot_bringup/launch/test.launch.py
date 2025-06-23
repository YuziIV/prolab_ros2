from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_bringup = FindPackageShare('turtlebot3_full_bringup').find('turtlebot3_full_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')
    map_file = LaunchConfiguration('map', default=os.path.join(pkg_bringup, 'maps', 'playground_map.yaml'))
    
    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')
    
    # Gazebo Launch
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch')
    
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time}.items()
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
        nav2,
        gzserver,
        ])
