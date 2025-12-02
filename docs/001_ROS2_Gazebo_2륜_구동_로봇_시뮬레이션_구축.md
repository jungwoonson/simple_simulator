# ROS2 + Gazebo 2륜 구동 로봇 시뮬레이션 구축

## 1. 프로젝트 구조 생성

### 1.1 워크스페이스 생성
```bash
mkdir -p ~/simple_simulator/src
cd ~/simple_simulator
```

### 1.2 ROS2 패키지 생성
```bash
cd ~/simple_simulator/src
ros2 pkg create --build-type ament_python robot_description
```

생성된 구조:
```
simple_simulator/
├── build/           # 빌드 파일 (자동 생성)
├── install/         # 설치 파일 (자동 생성)
├── log/            # 로그 파일 (자동 생성)
└── src/
    └── robot_description/
        ├── launch/          # launch 파일 디렉터리 (수동 생성 필요)
        ├── resource/        # URDF 파일 디렉터리
        ├── robot_description/
        ├── test/
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## 2. URDF 로봇 모델 작성

**URDF(Unified Robot Description Format)란?**
- 로봇의 구조를 XML 형식으로 정의하는 파일
- 로봇의 부품(Link)과 관절(Joint)을 기술하여 로봇 형태 표현

### 2.1 URDF 파일 생성
`src/robot_description/resource/robot.urdf` 파일 생성

### 2.2 URDF 내용
```xml
<?xml version="1.0"?>
<robot name="simple_robot">

    <!-- Base Link: 로봇의 본체 (중심 링크)
         모든 다른 링크들이 연결되는 기준점 -->
    <link name="base_link">
        <!-- Visual: Gazebo에서 보이는 시각적 표현 -->
        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>  <!-- xyz: 위치(m), rpy: 회전(rad), z=0.05로 바퀴 위에 배치 -->
            <geometry>
                <box size="0.6 0.4 0.2"/>  <!-- 직육면체: 길이 0.6m, 너비 0.4m, 높이 0.2m (센서 추가 공간 고려) -->
            </geometry>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>  <!-- RGBA: Red, Green, Blue, Alpha (0~1) -->
            </material>
        </visual>
        <!-- Collision: 물리 엔진이 충돌을 계산할 때 사용하는 형상 (보통 visual과 동일) -->
        <collision>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
        </collision>
        <!-- Inertial: 물리 시뮬레이션에 필요한 질량과 관성 정보 -->
        <inertial>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <mass value="10"/>  <!-- 질량 10kg -->
            <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/>  <!-- 관성 모멘트 텐서 -->
        </inertial>
    </link>

    <!-- Left Wheel: 왼쪽 구동 바퀴 (모터로 제어됨) -->
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.08"/>  <!-- 원통: 반지름 0.1m(지름 20cm), 두께 0.08m -->
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>  <!-- 검은색 -->
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.08"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>  <!-- 바퀴 질량 1kg -->
            <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>  <!-- 원통의 관성 모멘트 -->
        </inertial>
    </link>

    <!-- Joint: 본체와 바퀴를 연결하는 관절 -->
    <joint name="left_wheel_joint" type="continuous">  <!-- continuous: 360도 무한 회전 가능 -->
        <parent link="base_link"/>  <!-- 부모 링크: 본체 -->
        <child link="left_wheel"/>  <!-- 자식 링크: 바퀴 -->
        <origin xyz="0.15 0.24 0" rpy="1.5708 0 0"/>  <!-- 위치: 앞쪽 0.15m, 왼쪽 0.24m, 회전: 90도(바퀴를 세움) -->
        <axis xyz="0 0 -1"/>  <!-- 회전축: -z축 방향 (바퀴 회전 방향 반전) -->
    </joint>

    <!-- Right Wheel: 오른쪽 구동 바퀴 (모터로 제어됨) -->
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.08"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.08"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0.15 -0.24 0" rpy="1.5708 0 0"/>  <!-- 위치: 앞쪽 0.15m, 오른쪽 -0.24m (y축 음수 방향) -->
        <axis xyz="0 0 -1"/>  <!-- 회전축: -z축 방향 (바퀴 회전 방향 반전) -->
    </joint>

    <!-- Rear Caster: 뒤쪽 캐스터 볼 (자유롭게 구르는 보조 바퀴) -->
    <link name="rear_caster">
        <visual>
            <geometry>
                <sphere radius="0.05"/>  <!-- 구: 반지름 0.05m -->
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>  <!-- 회색 -->
            </material>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>  <!-- 캐스터 질량 0.5kg -->
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>  <!-- 구의 관성 모멘트 -->
        </inertial>
    </link>

    <joint name="rear_caster_joint" type="fixed">  <!-- fixed: 고정 조인트 (회전하지 않음) -->
        <parent link="base_link"/>
        <child link="rear_caster"/>
        <origin xyz="-0.2 0 -0.05" rpy="0 0 0"/>  <!-- 위치: 뒤쪽 -0.2m, 중앙, 아래 -0.05m -->
    </joint>

    <!-- Gazebo에서 캐스터 볼이 자유롭게 미끄러지도록 마찰 설정 -->
    <gazebo reference="rear_caster">
        <mu1>0.0</mu1>  <!-- 마찰 계수 0 (완전히 미끄러움) -->
        <mu2>0.0</mu2>
    </gazebo>

    <!-- Gazebo Differential Drive Plugin
         두 개의 바퀴를 독립적으로 제어하여 전진/후진/회전 구현
         양쪽 바퀴 속도가 같으면 직진, 다르면 회전 -->
    <gazebo>
        <plugin name="gz::sim::systems::DiffDrive" filename="libgz-sim-diff-drive-system.so">
            <left_joint>left_wheel_joint</left_joint>  <!-- 왼쪽 구동 바퀴 지정 -->
            <right_joint>right_wheel_joint</right_joint>  <!-- 오른쪽 구동 바퀴 지정 -->
            <wheel_separation>0.48</wheel_separation>  <!-- 좌우 바퀴 중심 간 거리 0.48m (0.24*2) -->
            <wheel_radius>0.1</wheel_radius>  <!-- 바퀴 반지름 0.1m (속도 계산에 사용) -->
            <topic>/cmd_vel</topic>  <!-- ROS2 토픽: geometry_msgs/Twist 타입, linear.x(전후진 m/s), angular.z(회전 rad/s) -->
        </plugin>
    </gazebo>

</robot>
```

## 3. Launch 파일 작성

**Launch 파일이란?**
- 여러 ROS2 노드와 프로그램을 한 번에 실행하는 스크립트
- 로봇 시스템 구동에 필요한 모든 컴포넌트를 자동으로 실행

### 3.1 Launch 디렉터리 생성
```bash
mkdir -p ~/simple_simulator/src/robot_description/launch
```

### 3.2 Launch 파일 생성
`src/robot_description/launch/spawn_robot.launch.py` 파일 생성

### 3.3 Launch 파일 내용
```python
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
```

## 4. setup.py 수정

**setup.py란?**
- ROS2 Python 패키지의 설정 파일
- 패키지 빌드 시 어떤 파일들을 설치할지 정의

### 4.1 setup.py 파일 수정
`src/robot_description/setup.py` 파일을 열어 다음과 같이 수정:

```python
from setuptools import find_packages, setup
import os
from glob import glob  # 파일 패턴 매칭을 위한 모듈

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 인덱스에 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch 디렉터리의 모든 .py 파일을 설치 디렉터리로 복사
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # resource 디렉터리의 모든 .urdf 파일을 설치 디렉터리로 복사
        ('share/' + package_name + '/resource', glob('resource/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot description package',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 실행 가능한 스크립트가 있다면 여기에 추가
        ],
    },
)
```

## 5. 빌드 및 실행

### 5.1 ROS2 환경 설정
**중요:** 새 터미널을 열 때마다 반드시 먼저 실행해야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
```

이 명령어는:
- ROS2 Jazzy의 기본 환경 변수를 설정
- `ros2` 명령어를 사용 가능하게 함
- 모든 ROS2 패키지와 도구를 로드

### 5.2 패키지 빌드
```bash
cd ~/simple_simulator
colcon build
```

### 5.3 워크스페이스 환경 설정 적용
```bash
source install/setup.bash
```

이 명령어는:
- 방금 빌드한 패키지들을 ROS2 환경에 등록
- 빌드 후 항상 실행해야 함

### 5.4 로봇 시뮬레이션 실행
```bash
ros2 launch robot_description spawn_robot.launch.py
```

실행되면:
- Gazebo 창이 열림
- 파란색 본체, 앞쪽에 검은색 구동 바퀴 2개, 뒤쪽에 회색 캐스터볼 1개를 가진 로봇 생성

## 6. 로봇 제어

### 6.1 새 터미널에서 환경 설정
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator
source install/setup.bash
```

### 6.2 로봇 제어 명령

**전진**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

**전진하면서 좌회전**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
```

**정지**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**참고:**
- `linear.x`: 전후진 속도 (양수=전진, 음수=후진)
- `angular.z`: 회전 속도 (양수=좌회전, 음수=우회전)
- 위 두 값을 조합하여 다양한 움직임 구현 가능
