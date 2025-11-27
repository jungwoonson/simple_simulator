import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지의 설치 경로 가져오기
    pkg_path = get_package_share_directory('robot_description')

    # URDF 파일의 전체 경로 생성
    urdf_file = os.path.join(pkg_path, 'resource', 'robot.urdf')

    # URDF 파일 내용을 문자열로 읽기
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1. Gazebo 시뮬레이터 실행
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],  # gz sim: Gazebo 실행, -r: 자동 시작, empty.sdf: 빈 월드
            output='screen'  # 터미널에 로그 출력
        ),

        # 2. Gazebo에 로봇 spawn
        # ros_gz_sim의 create 실행파일로 /robot_description 토픽에서 URDF를 읽어 Gazebo에 로봇 생성
        Node(
            package='ros_gz_sim',  # ROS-Gazebo 통합 패키지
            executable='create',  # 로봇 생성 실행 파일
            arguments=[
                '-name', 'simple_robot',  # Gazebo에서 사용할 로봇 이름
                '-topic', '/robot_description'  # URDF를 받을 토픽
            ],
            output='screen'
        ),

        # 3. robot_state_publisher 실행
        # URDF 정보를 ROS2 토픽으로 발행하고 TF(Transform) 정보 제공
        # TF는 로봇 각 링크 간 상대적 위치/회전 정보 (SLAM, Navigation 등에 필수)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]  # URDF 내용을 파라미터로 전달
        ),

        # 4. ROS2-Gazebo 브릿지 실행
        # ROS2와 Gazebo는 서로 다른 메시지 시스템을 사용하므로 브릿지를 통해 메시지 변환
        # 이 브릿지가 없으면 ROS2 명령이 Gazebo에 전달되지 않음
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            # 토픽@ROS2메시지타입@Gazebo메시지타입 형식으로 브릿지 설정
            # /cmd_vel 토픽을 ROS2 ↔ Gazebo 간 연결
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),
    ])