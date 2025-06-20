from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    ld = LaunchDescription()

    ld.add_action(start_lifecycle_manager_cmd)

    return ld