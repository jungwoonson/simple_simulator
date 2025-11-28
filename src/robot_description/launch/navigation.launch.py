import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')

    map_file = os.path.join(pkg_path, 'maps', 'factory_map.yaml')
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
        ),

        # AMCL (위치 추정)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])