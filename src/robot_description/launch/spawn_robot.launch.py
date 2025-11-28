import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_path, 'resource', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', os.path.join(pkg_path, 'worlds', 'factory_warehouse.sdf')],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'simple_robot',
                '-topic', '/robot_description',
                '-x', '3.0',
                '-y', '3.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/world/factory_warehouse/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/factory_warehouse/model/simple_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            parameters=[{
                'qos_overrides./clock.publisher.reliability': 'best_effort'
            }],
            remappings=[
                ('/scan', '/scan_raw'),
                ('/world/factory_warehouse/clock', '/clock'),
                ('/world/factory_warehouse/model/simple_robot/joint_state', '/joint_states'),
            ],
            output='screen'
        ),

        Node(
            package='robot_description',
            executable='scan_frame_changer',
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(pkg_path, 'config', 'slam_params.yaml'), {'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            parameters=[{
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['slam_toolbox']
            }],
            output='screen'
        ),
    ])