from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    
    pkg_gazebo_ros = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    # Define paths
    urdf_file = os.path.join(os.getenv('HOME'), 'ros2_ws', 'src', 'my_robot_bringup', 'urdf', 'turtlebot3', 'turtlebot3_burger.urdf.xacro')
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')
    
    # Nodes

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    #joint_state_publisher = Node(
    #    package="joint_state_publisher",
    #    executable="joint_state_publisher",
    #    name="joint_state_publisher",
    #    output="screen",
    #    parameters=[{
    #        "use_sim_time": use_sim_time
    #    }]
    #
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    rviz= Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        
        gzserver_cmd,
        gzclient_cmd,        
        robot_state_publisher_cmd,
        #joint_state_publisher,
        spawn_turtlebot_cmd,
        #rviz
    ])
