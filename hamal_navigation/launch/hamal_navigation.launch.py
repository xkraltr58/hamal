import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_packages_share_directory

def generate_launch_description():
    # Get the package share directory
    hamal_navigation_dir = get_package_share_directory('hamal_navigation')

    # Define parameters namespace
    params_namespace = '/hamal_nav2'

    return launch.LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='nav2_bringup',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_local_costmap',
            executable='local_costmap_node',
            name='local_costmap_node',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_global_costmap',
            executable='global_costmap_node',
            name='global_costmap_node',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_dwa_local_planner',
            executable='dwa_planner',
            name='dwa_local_planner',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_traj_planner',
            executable='trajectory_planner',
            name='trajectory_planner',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='teb_local_planner',
            executable='teb_local_planner_node',
            name='teb_local_planner_node',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])