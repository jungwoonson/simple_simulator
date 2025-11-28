# Simple Simulator

ROS2 Jazzy + Gazebo Harmonic 기반 2륜 구동 로봇 시뮬레이터

## 주요 기능

- 2륜 구동 로봇 시뮬레이션 (전륜구동 + 후방 캐스터볼)
- LiDAR 센서 통합
- SLAM 기반 실시간 맵핑 (slam_toolbox)
- ROS2-Gazebo 메시지 브리지
- TF 프레임 자동 리매핑

## 실행 방법
```bash
# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash

# 빌드
cd ~/simple_simulator
colcon build
source install/setup.bash

# 환경 변수 설정
export GZ_IP=127.0.0.1
export GZ_PARTITION=test

# 시뮬레이터 실행
ros2 launch robot_description gazebo.launch.py
```

## 로봇 제어
```bash
# 전진
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# 정지
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 기술 스택

- ROS2 Jazzy
- Gazebo Harmonic
- slam_toolbox
- Python 3