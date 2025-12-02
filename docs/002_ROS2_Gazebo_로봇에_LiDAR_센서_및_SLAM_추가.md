# ROS2 + Gazebo 로봇에 LiDAR 센서 및 SLAM 추가

## 이 문서에서 다루는 내용
기본 2륜 구동 로봇에 LiDAR 센서를 추가하고 SLAM(Simultaneous Localization and Mapping) 기능을 구현하는 과정을 다룹니다.

**추가된 기능:**
- LiDAR 센서 (360도 레이저 스캔)
- SLAM 매핑 (slam_toolbox 사용)
- Odometry (주행 거리 측정)
- TF frame 관리

## 1. 디렉터리 구조 추가

### 1.1 config 디렉터리 생성
```bash
mkdir -p ~/simple_simulator/src/robot_description/config
mkdir -p ~/simple_simulator/src/robot_description/robot_description
```

최종 구조:
```
simple_simulator/
└── src/
    └── robot_description/
        ├── config/              # SLAM 설정 파일 (새로 추가)
        │   └── slam_params.yaml
        ├── launch/
        │   └── spawn_robot.launch.py
        ├── resource/
        │   └── robot.urdf
        ├── robot_description/   # Python 노드 (새로 추가)
        │   ├── __init__.py
        │   └── scan_frame_changer.py
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## 2. URDF에 LiDAR 센서 추가

### 2.1 robot.urdf 파일 수정
`src/robot_description/resource/robot.urdf` 파일 수정

**수정 내용 1: DiffDrive 플러그인 수정**

기존 DiffDrive 플러그인 부분을 찾아서 다음과 같이 수정:

```xml
    <gazebo>
        <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
            <left_joint>left_wheel_joint</left_joint>
            <right_joint>right_wheel_joint</right_joint>
            <wheel_separation>0.48</wheel_separation>
            <wheel_radius>0.1</wheel_radius>
            <odom_publish_frequency>50</odom_publish_frequency>  <!-- Odometry 발행 빈도 (새로 추가) -->
            <topic>cmd_vel</topic>
            <tf_topic>/tf</tf_topic>  <!-- TF 발행 (새로 추가) -->
            <frame_id>odom</frame_id>  <!-- Odometry 프레임 (새로 추가) -->
            <child_frame_id>base_link</child_frame_id>  <!-- 로봇 프레임 (새로 추가) -->
        </plugin>
    </gazebo>
```

**추가 내용 1: Sensors System 플러그인**

DiffDrive 플러그인 아래에 추가:

```xml
    <!-- Gazebo Sensors System Plugin: 센서 시스템 활성화 -->
    <gazebo>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>  <!-- 렌더링 엔진 지정 -->
        </plugin>
    </gazebo>
```

**추가 내용 2: LiDAR 센서 링크 및 조인트**

```xml
    <!-- LiDAR Sensor Link: 센서의 물리적 형태 정의 -->
    <link name="lidar_link">
        <!-- Visual: 센서의 시각적 모양 (빨간 원통) -->
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.07"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
        <!-- Collision: 충돌 감지용 형상 -->
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.07"/>
            </geometry>
        </collision>
        <!-- Inertial: 질량 및 관성 정보 -->
        <inertial>
            <mass value="0.3"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <!-- LiDAR Joint: 센서를 본체에 고정 연결 -->
    <joint name="lidar_joint" type="fixed">
        <parent link="base_link"/>  <!-- 부모: 로봇 본체 -->
        <child link="lidar_link"/>   <!-- 자식: LiDAR 센서 -->
        <origin xyz="0.2 0 0.2" rpy="0 0 0"/>  <!-- 위치: 앞쪽 0.2m, 위쪽 0.2m -->
    </joint>
```

**추가 내용 3: LiDAR Gazebo 플러그인**

```xml
    <!-- Gazebo LiDAR Plugin: 실제 센서 기능 구현 -->
    <gazebo reference="lidar_link">  <!-- lidar_link에 센서 플러그인 적용 -->
        <sensor name="gpu_lidar" type="gpu_lidar">  <!-- 센서 이름 및 타입 정의 -->
            <topic>scan</topic>  <!-- 센서 데이터를 발행할 Gazebo 토픽 이름 -->
            <update_rate>10</update_rate>  <!-- 센서 업데이트 빈도 (Hz) -->
            <lidar>  <!-- LiDAR 센서 설정 -->
                <scan>  <!-- 스캔 패턴 설정 -->
                    <horizontal>  <!-- 수평 스캔 설정 -->
                        <samples>360</samples>  <!-- 한 회전당 측정점 개수 -->
                        <resolution>1</resolution>  <!-- 각 측정점 간 해상도 -->
                        <min_angle>-3.14159</min_angle>  <!-- 스캔 시작 각도 (라디안, -180도) -->
                        <max_angle>3.14159</max_angle>   <!-- 스캔 종료 각도 (라디안, +180도) -->
                    </horizontal>
                </scan>
                <range>  <!-- 거리 측정 범위 -->
                    <min>0.12</min>  <!-- 최소 감지 거리 (미터) -->
                    <max>10.0</max>  <!-- 최대 감지 거리 (미터) -->
                </range>
            </lidar>
            <visualize>true</visualize>  <!-- Gazebo에서 시각적 표시 여부 -->
        </sensor>
    </gazebo>
```

**추가 내용 4: Joint State Publisher 플러그인**

```xml
    <gazebo>
        <plugin filename="gz-sim-joint-state-publisher-system"
                name="gz::sim::systems::JointStatePublisher">
            <topic>/world/empty/joint_state</topic>
        </plugin>
    </gazebo>
```

## 3. SLAM 설정 파일 작성

### 3.1 SLAM 파라미터 파일 생성
`src/robot_description/config/slam_params.yaml` 파일 생성

**slam_toolbox란?**
- ROS2에서 가장 많이 사용되는 SLAM 패키지
- LiDAR 스캔 데이터를 이용해 실시간으로 지도 생성
- 로봇의 위치도 동시에 추정

```yaml
slam_toolbox:
  ros__parameters:
    # 최적화 알고리즘 설정 (Ceres Solver 사용)
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG

    # 프레임 및 토픽 설정
    map_frame: map  # 지도 프레임 이름
    odom_frame: odom  # Odometry 프레임 이름
    base_frame: base_link  # 로봇 본체 프레임 이름
    scan_topic: /scan  # LiDAR 스캔 데이터를 받을 토픽
    mode: mapping  # 매핑 모드 (지도 생성)

    # SLAM 업데이트 설정
    map_update_interval: 1.0  # 지도 업데이트 주기 (초)
    max_laser_range: 10.0  # 최대 레이저 범위 (m)
    minimum_travel_distance: 0.3  # 최소 이동 거리 (m) - 이만큼 이동해야 업데이트
    minimum_travel_heading: 0.3  # 최소 회전 각도 (rad) - 이만큼 회전해야 업데이트
    resolution: 0.05  # 지도 해상도 (m) - 한 픽셀이 5cm

    # TF(Transform) 관련 설정
    transform_publish_period: 0.05  # TF 발행 주기 (초)
    tf_buffer_duration: 30.0  # TF 버퍼 유지 시간 (초)

    # Lifecycle 관리자 사용 (자동 시작)
    use_lifecycle_manager: true
```

## 4. Frame 변환 노드 작성

### 4.1 왜 필요한가?
Gazebo Harmonic과 ROS2 Jazzy 간 TF frame 이름 충돌 문제를 해결하기 위해 중간에서 frame_id를 변경해주는 노드가 필요합니다.

**문제 상황:**
- Gazebo에서 발행하는 /scan 토픽의 frame_id: `simple_robot/base_link/lidar_link`
- SLAM toolbox가 기대하는 frame_id: `lidar_link`
- 이름이 달라서 SLAM이 작동하지 않음

**해결 방법:**
- `/scan_raw` (Gazebo 원본) → Frame 변환 노드 → `/scan` (변경된 frame_id) → SLAM

### 4.2 Python 노드 파일 생성
`src/robot_description/robot_description/scan_frame_changer.py` 파일 생성

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameChanger(Node):
    """
    LiDAR 스캔 메시지의 frame_id를 변경하는 노드
    
    역할:
    1. /scan_raw 토픽 구독 (Gazebo에서 오는 원본)
    2. frame_id를 'lidar_link'로 변경
    3. /scan 토픽으로 재발행 (SLAM으로 전달)
    """
    def __init__(self):
        super().__init__('scan_frame_changer')
        
        # /scan_raw 토픽 구독 (Gazebo 브릿지에서 받음)
        self.subscription = self.create_subscription(
            LaserScan,  # 메시지 타입
            '/scan_raw',  # 구독할 토픽
            self.scan_callback,  # 콜백 함수
            10)  # QoS (메시지 큐 크기)
        
        # /scan 토픽 발행 (SLAM으로 보냄)
        self.publisher = self.create_publisher(
            LaserScan,  # 메시지 타입
            '/scan',  # 발행할 토픽
            10)  # QoS

    def scan_callback(self, msg):
        """
        메시지를 받을 때마다 실행되는 콜백 함수
        
        Parameters:
            msg: LaserScan 메시지 (Gazebo에서 온 원본)
        """
        # frame_id를 변경 (예: 'simple_robot/base_link/lidar_link' → 'lidar_link')
        msg.header.frame_id = 'lidar_link'
        
        # 변경된 메시지를 /scan 토픽으로 발행
        self.publisher.publish(msg)

def main(args=None):
    """노드 실행 메인 함수"""
    rclpy.init(args=args)  # ROS2 초기화
    node = ScanFrameChanger()  # 노드 생성
    rclpy.spin(node)  # 노드 실행 (종료될 때까지 계속 실행)
    node.destroy_node()  # 노드 정리
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
```

### 4.3 __init__.py 파일 생성
`src/robot_description/robot_description/__init__.py` 파일 생성 (빈 파일)

```bash
touch ~/simple_simulator/src/robot_description/robot_description/__init__.py
```

이 파일은 Python 패키지로 인식되게 하기 위해 필요합니다.

## 5. Launch 파일 수정

### 5.1 spawn_robot.launch.py 파일 수정
`src/robot_description/launch/spawn_robot.launch.py` 파일 수정

**수정 내용 1: robot_state_publisher에 use_sim_time 추가**

기존:
```python
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
```

수정 후:
```python
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True  # 시뮬레이션 시간 사용 (새로 추가)
            }]
        ),
```

**수정 내용 2: ROS2-Gazebo 브릿지 확장**

기존:
```python
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),
```

수정 후:
```python
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # 기본 제어
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # LiDAR 스캔 (새로 추가)
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # Odometry (새로 추가)
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # TF (새로 추가)
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                # Clock (새로 추가) - 시뮬레이션 시간 동기화
                '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint State (새로 추가) - 관절 상태
                '/world/empty/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            parameters=[{
                # Clock 토픽을 best_effort로 설정 (시간 동기화 안정성)
                'qos_overrides./clock.publisher.reliability': 'best_effort'
            }],
            remappings=[
                # 토픽 이름 재매핑
                ('/scan', '/scan_raw'),  # /scan → /scan_raw (frame 변환 전)
                ('/world/empty/clock', '/clock'),  # Clock 토픽 이름 단순화
                ('/world/empty/joint_state', '/joint_states'),  # Joint state 이름 단순화
            ],
            output='screen'
        ),
```

**추가 내용 1: Scan Frame 변환 노드**

return LaunchDescription의 마지막 `])` 앞에 추가:

```python
        # Scan Frame 변환 노드 실행 (새로 추가)
        # /scan_raw의 frame_id를 변경하여 /scan으로 재발행
        Node(
            package='robot_description',
            executable='scan_frame_changer',
            output='screen'
        ),
```

**추가 내용 2: SLAM toolbox 노드**

```python
        # SLAM toolbox 실행 (새로 추가)
        # 실시간 지도 생성
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # 동기식 SLAM 노드
            name='slam_toolbox',
            parameters=[
                os.path.join(pkg_path, 'config', 'slam_params.yaml'),  # SLAM 설정 파일
                {'use_sim_time': True}  # 시뮬레이션 시간 사용
            ],
            output='screen',
        ),
```

**추가 내용 3: Lifecycle Manager**

```python
        # Lifecycle Manager 실행 (새로 추가)
        # SLAM 노드를 자동으로 시작/관리
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            parameters=[{
                'autostart': True,  # 자동 시작
                'bond_timeout': 0.0,  # Bond 타임아웃 (0=무제한)
                'node_names': ['slam_toolbox']  # 관리할 노드 목록
            }],
            output='screen'
        ),
```

## 6. setup.py 수정

### 6.1 setup.py 파일 수정
`src/robot_description/setup.py` 파일 수정

**수정 내용 1: data_files에 config 디렉터리 추가**

기존 data_files 리스트에서:
```python
    data_files=[
        ...
        ('share/' + package_name + '/resource', glob('resource/*.urdf')),
    ],
```

아래에 추가:
```python
        # config 디렉터리의 모든 .yaml 파일을 설치 디렉터리로 복사 (새로 추가)
        ('share/' + package_name + '/config', glob('config/*.yaml')),
```

**수정 내용 2: entry_points에 scan_frame_changer 실행 파일 등록**

기존:
```python
    entry_points={
        'console_scripts': [
            # 실행 가능한 스크립트가 있다면 여기에 추가
        ],
    },
```

수정 후:
```python
    entry_points={
        'console_scripts': [
            # scan_frame_changer 실행 파일 등록 (새로 추가)
            # 명령어 이름 = 패키지.모듈:함수
            'scan_frame_changer = robot_description.scan_frame_changer:main',
        ],
    },
```

## 7. 필요한 패키지 설치

### 7.1 SLAM 관련 패키지 설치
```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-lifecycle-manager -y
```

**설치되는 패키지:**
- `slam-toolbox`: SLAM 알고리즘 패키지
- `nav2-lifecycle-manager`: SLAM 노드 자동 시작/관리 도구

## 8. 빌드 및 실행

### 8.1 환경 설정
```bash
source /opt/ros/jazzy/setup.bash
```

### 8.2 빌드
```bash
cd ~/simple_simulator
colcon build
```

### 8.3 워크스페이스 환경 설정
```bash
source install/setup.bash
```

### 8.4 실행
```bash
ros2 launch robot_description spawn_robot.launch.py
```

**실행되는 것들:**
1. Gazebo 시뮬레이터
2. 로봇 spawn (본체 + 바퀴 + LiDAR)
3. LiDAR 센서 시작 (빨간 레이저 빔 표시)
4. SLAM 시작 (지도 생성 시작)

## 9. SLAM 지도 확인 (RViz2)

### 9.1 새 터미널에서 RViz2 실행
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator
source install/setup.bash
rviz2
```

### 9.2 RViz2 설정

**1단계: Fixed Frame 설정**
- 왼쪽 Displays 패널 → Global Options → Fixed Frame
- `map`으로 변경

**2단계: Map 추가**
- 하단 Add 버튼 클릭
- By topic 탭 → `/map` → `Map` 선택

**3단계: LaserScan 추가 (선택사항)**
- Add 버튼 → By topic → `/scan` → `LaserScan` 선택
- LiDAR 스캔 빔을 실시간으로 볼 수 있음

### 9.3 로봇 움직여서 지도 생성
새 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator
source install/setup.bash

# 전진
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 회전하며 스캔
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

로봇이 움직이면서 RViz2의 Map에 흰색/검은색 픽셀로 지도가 그려지는 것을 확인할 수 있습니다.

## 10. 요약

**추가된 기능:**
1. ✅ LiDAR 센서 (360도 스캔)
2. ✅ SLAM 매핑 (실시간 지도 생성)
3. ✅ Odometry (위치 추정)
4. ✅ Frame 변환 노드 (Gazebo-ROS2 호환)

**주요 파일:**
- `robot.urdf`: LiDAR 센서 추가
- `slam_params.yaml`: SLAM 설정
- `scan_frame_changer.py`: Frame ID 변환
- `spawn_robot.launch.py`: 모든 노드 실행
- `setup.py`: 패키지 설정
