from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    pkg_bringup = FindPackageShare('turtlebot3_full_bringup').find(
        'turtlebot3_full_bringup'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define paths
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')
    map_file = LaunchConfiguration(
        'map', default=os.path.join(pkg_bringup, 'maps', 'playground_map.yaml')
    )
    #urdf_file = os.path.join(pkg_bringup, 'assets', 'model.sdf')

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch')

    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_param = os.path.join(nav2_pkg, 'param')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_pkg, 'launch')
    #nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    # RViz Config
    rviz_config_dir = os.path.join(pkg_bringup, 'rviz', 'rviz.rviz')
    #rviz_config_dir = os.path.join(
    #    nav2_pkg,
    #    'rviz',
    #    'tb3_navigation2.rviz')
    
    # Robot State Publisher
    robostate_package = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    robostate_launch = os.path.join(
        robostate_package, 'launch', 'robot_state_publisher.launch.py'
    )
    robotspawn_launch = os.path.join(
        robostate_package, 'launch', 'spawn_turtlebot3.launch.py'
    )

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    
    # Parameters
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                nav2_param,
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                nav2_param,
                param_file_name))
    # NODES
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzclient.launch.py')
        ),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robotspawn_launch)),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )


    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robostate_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    #nav2 = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(nav2_launch),
    #    launch_arguments={
    #        'map': map_file,
    #        'use_sim_time': use_sim_time,
    #        'autostart': 'true',
    #    }.items(),
    #)
    
    # Map server
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
        }.items(),
    )

    # start_gazebo_ros_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', TURTLEBOT3_MODEL,
    #         '-file', urdf_file,
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', '0.01',
    #     ],
    #     output='screen',
    # )
    
    filter_node = Node(
        package='turtlebot3_full_bringup',
        executable='kalman_filter',
        name='kalman_filter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ground_truth_publisher = Node(
        package='turtlebot3_full_bringup',
        executable='ground_truth',
        name='ground_truth',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    return LaunchDescription(
        [
        #nav2,
        map_server,
        rviz,
        gzserver,
        gzclient,
        spawn_turtlebot_cmd,
        #start_gazebo_ros_spawner_cmd,
        robot_state_publisher_cmd,
        filter_node,
        #ground_truth_publisher,
        ]
    )
