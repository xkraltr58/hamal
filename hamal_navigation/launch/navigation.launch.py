import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hamal_navigation_dir = get_package_share_directory('hamal_navigation')

    params_namespace = '/hamal_nav2'

    return launch.LaunchDescription([
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
            package='nav2_behavior_tree',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_costmap_2d',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=params_namespace,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
