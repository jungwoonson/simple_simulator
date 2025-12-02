# Simple Simulator

ROS2 Jazzy + Gazebo Harmonic 기반 2륜 구동 로봇 시뮬레이터

## 주요 기능

- 2륜 구동 로봇 시뮬레이션 (전륜구동 + 후방 캐스터볼)
- LiDAR 센서 통합
- SLAM 기반 실시간 맵핑 (slam_toolbox)
- Nav2 자율주행 및 장애물 회피
- ROS2-Gazebo 메시지 브리지
- TF 프레임 자동 리매핑

## 자율주행 테스트 방법

### 1. 환경 설정
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator
colcon build
source install/setup.bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=test
```

### 2. 로봇 스폰 (터미널 1)
```bash
ros2 launch robot_description spawn_robot.launch.py
```

### 3. Nav2 실행 (터미널 2)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator
source install/setup.bash
ros2 launch robot_description navigation_launch.py
```

### 4. RViz2 실행 및 설정 (터미널 3)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator
source install/setup.bash
rviz2
```

**RViz2 설정:**

1. **Fixed Frame 변경**
    - 왼쪽 위 Displays 패널에서 Global Options → Fixed Frame을 `map`으로 변경

2. **맵 추가**
    - 왼쪽 아래 **Add** 버튼 클릭
    - **By topic** 탭 선택
    - `/map` → **Map** 선택 → **OK**

3. **로봇 모델 추가**
    - **Add** 버튼 클릭
    - **By display type** 탭 선택
    - **RobotModel** 선택 → **OK**
    - Description Topic을 `/robot_description`으로 설정

4. **LiDAR 스캔 추가**
    - **Add** 버튼 클릭
    - **By topic** 탭 선택
    - `/scan` → **LaserScan** 선택 → **OK**

### 5. 로봇 초기 위치 설정
1. RViz2 상단의 **"2D Pose Estimate"** 버튼 클릭
2. 맵에서 로봇의 현재 위치를 클릭하고 드래그로 방향 설정

### 6. 목표 지점 설정
1. RViz2 상단의 **"2D Goal Pose"** 버튼 클릭
2. 맵에서 목표 지점을 클릭하고 드래그로 방향 설정
3. 로봇이 자동으로 경로 계획 및 이동 시작

### 주요 기능
- 동적 장애물 회피
- 실시간 경로 재계획
- costmap 기반 충돌 방지

## 기술 스택

- ROS2 Jazzy
- Gazebo Harmonic
- slam_toolbox
- Nav2 navigation stack
- Python 3