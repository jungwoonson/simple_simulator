# ROS2 + Gazebo + Nav2 자율주행 구현

## 1. 프로젝트 구조

이전 단계에서 구축한 SLAM 시스템 위에 Nav2를 추가합니다.

```
simple_simulator/
└── src/
    └── robot_description/
        ├── config/
        │   ├── slam_params.yaml
        │   └── nav2_params.yaml      # (신규)
        ├── launch/
        │   ├── spawn_robot_launch.py
        │   └── navigation_launch.py  # (신규)
        ├── maps/                      # (신규)
        │   ├── factory_map.yaml
        │   └── factory_map.pgm
        ├── resource/
        │   └── robot.urdf
        ├── robot_description/
        │   ├── __init__.py
        │   └── scan_frame_changer.py
        ├── worlds/
        │   └── factory_warehouse.sdf  # (신규)
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## 2. Gazebo World 파일 생성

### 2.1 worlds 디렉터리 생성

```bash
mkdir -p ~/simple_simulator/src/robot_description/worlds
```

### 2.2 factory_warehouse.sdf 파일 생성

`src/robot_description/worlds/factory_warehouse.sdf` 전체 내용:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="factory_warehouse">

    <!-- 조명 -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 바닥 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 벽 1 (북쪽) -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 8 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>16 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>16 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 벽 2 (남쪽) -->
    <model name="wall_south">
      <static>true</static>
      <pose>0 -8 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>16 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>16 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 벽 3 (동쪽) -->
    <model name="wall_east">
      <static>true</static>
      <pose>8 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 16 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 16 2</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 벽 4 (서쪽) -->
    <model name="wall_west">
      <static>true</static>
      <pose>-8 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 16 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 16 2</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 박스 스택 1 (3단) -->
    <model name="box_stack_1_base">
      <static>true</static>
      <pose>-5 5 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_stack_1_mid">
      <static>true</static>
      <pose>-5 5 0.9 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_stack_1_top">
      <static>true</static>
      <pose>-5 5 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.3 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 박스 스택 2 (2단) -->
    <model name="box_stack_2_base">
      <static>true</static>
      <pose>5 5 0.4 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.8 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.8 0.8 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.4 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_stack_2_top">
      <static>true</static>
      <pose>5 5 1.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.8 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.8 0.8 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.4 0.3 1</ambient>
            <diffuse>0.5 0.4 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 단일 박스들 -->
    <model name="box_1">
      <static>true</static>
      <pose>-6 -3 0.25 0 0 0.5</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.5 0.4 1</ambient>
            <diffuse>0.6 0.5 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_2">
      <static>true</static>
      <pose>3 -5 0.3 0 0 0.3</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.4 0.3 1</ambient>
            <diffuse>0.5 0.4 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_3">
      <static>true</static>
      <pose>-3 2 0.35 0 0 -0.4</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.7 0.7 0.7</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.7 0.7 0.7</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_4">
      <static>true</static>
      <pose>6 -2 0.4 0 0 0.8</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.8 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.8 0.8 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.4 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_5">
      <static>true</static>
      <pose>-2 -6 0.3 0 0 -0.6</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 기둥 1 -->
    <model name="pillar_1">
      <static>true</static>
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.3</radius><length>2</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.3</radius><length>2</length></cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 큰 박스들 (팔레트) -->
    <model name="large_box_1">
      <static>true</static>
      <pose>-5 -5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1.2 1.0 1.0</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.2 1.0 1.0</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="large_box_2">
      <static>true</static>
      <pose>5 -6 0.6 0 0 0.5</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1.0 1.2 1.2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.0 1.2 1.2</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 추가 박스들 -->
    <model name="box_6">
      <static>true</static>
      <pose>2 3 0.25 0 0 1.2</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.3 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_7">
      <static>true</static>
      <pose>-4 0 0.3 0 0 -0.5</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.4 0.2 1</ambient>
            <diffuse>0.5 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_8">
      <static>true</static>
      <pose>4 1 0.25 0 0 0.9</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.5 0.4 1</ambient>
            <diffuse>0.6 0.5 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 박스 스택 3 (4단) -->
    <model name="box_stack_3_1">
      <static>true</static>
      <pose>-6 2 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.4 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_stack_3_2">
      <static>true</static>
      <pose>-6 2 0.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.4 0.3 1</ambient>
            <diffuse>0.5 0.4 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_stack_3_3">
      <static>true</static>
      <pose>-6 2 1.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.5 0.4 1</ambient>
            <diffuse>0.6 0.5 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_stack_3_4">
      <static>true</static>
      <pose>-6 2 1.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.6 0.5 1</ambient>
            <diffuse>0.7 0.6 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 추가 박스들 -->
    <model name="box_9">
      <static>true</static>
      <pose>1 -3 0.3 0 0 0.4</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.6 0.6 0.6</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.2 1</ambient>
            <diffuse>0.5 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_10">
      <static>true</static>
      <pose>-1 6 0.35 0 0 -0.7</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.7 0.7 0.7</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.7 0.7 0.7</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.3 1</ambient>
            <diffuse>0.6 0.4 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Gazebo 플러그인 -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>

  </world>
</sdf>
```

## 3. spawn_robot_launch.py 수정

`src/robot_description/launch/spawn_robot_launch.py` 파일에서 다음 부분 수정:

**변경 1: 월드 파일 경로**
```python
# 기존
cmd=['gz', 'sim', '-r', 'empty.sdf'],

# 변경
cmd=['gz', 'sim', '-r', os.path.join(pkg_path, 'worlds', 'factory_warehouse.sdf')],
```

**변경 2: 로봇 초기 위치**
```python
# 기존
arguments=['-name', 'simple_robot', '-topic', '/robot_description'],

# 변경
arguments=[
    '-name', 'simple_robot',
    '-topic', '/robot_description',
    '-x', '3.0',
    '-y', '3.0',
    '-z', '0.1'
],
```

**변경 3: clock 토픽**
```python
# 기존
'/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

# 변경
'/world/factory_warehouse/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
```

**변경 4: joint_state 토픽**
```python
# 기존
'/world/empty/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

# 변경
'/world/factory_warehouse/model/simple_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
```

**변경 5: remappings**
```python
# 기존
remappings=[
    ('/scan', '/scan_raw'),
    ('/world/empty/clock', '/clock'),
    ('/world/empty/joint_state', '/joint_states'),
],

# 변경
remappings=[
    ('/scan', '/scan_raw'),
    ('/world/factory_warehouse/clock', '/clock'),
    ('/world/factory_warehouse/model/simple_robot/joint_state', '/joint_states'),
],
```

## 4. robot.urdf 수정

`src/robot_description/resource/robot.urdf` 맨 끝, `</robot>` 태그 **직전**에 추가:

```xml
    <!-- Joint State Publisher Plugin -->
    <gazebo>
        <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
        </plugin>
    </gazebo>

</robot>
```

## 5. setup.py 수정

`src/robot_description/setup.py`에서 `data_files` 섹션에 worlds 추가:

```python
data_files=[
    # ... (기존 항목들)
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),  # 추가
],

```
## 6. 맵 저장하기

### 6.1 맵 디렉터리 생성

```bash
mkdir -p ~/simple_simulator/src/robot_description/maps
```

### 6.2 SLAM 실행 및 맵 생성

**터미널 1: SLAM 실행**
```bash
cd ~/simple_simulator
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description spawn_robot_launch.py
```

**터미널 2: 로봇 제어**
```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

키보드 조작으로 **전체 공간을 구석구석 탐색**:
```
u i o : 전진+회전
j k l : 좌회전/정지/우회전
m , . : 후진+회전
```

**중요:** 
- 창고의 모든 벽, 장애물, 통로를 빠짐없이 돌아다녀야 완전한 맵이 생성됩니다
- LiDAR가 닿지 않은 영역은 맵에 표시되지 않습니다
- 로봇을 천천히 움직이면서 모든 구역을 스캔하세요

**터미널 3: 맵 저장**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/simple_simulator/src/robot_description/maps
ros2 run nav2_map_server map_saver_cli -f factory_map
```

### 6.3 생성된 파일 확인

`map_saver_cli` 명령어가 자동으로 생성하는 파일들:

**factory_map.yaml** (맵 메타데이터 - 자동 생성):
```yaml
image: factory_map.pgm
mode: trinary
resolution: 0.050
origin: [-11.070, -10.988, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**factory_map.pgm** (맵 이미지 - 자동 생성)

**참고:** 두 파일 모두 자동 생성되므로 직접 작성하지 않습니다.

### 6.4 setup.py에 maps 추가

맵 파일이 생성되었으므로 이제 `src/robot_description/setup.py`에 maps 디렉토리 추가:

```python
data_files=[
    # ... (기존 항목들)
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ('share/' + package_name + '/maps', glob('maps/*')),  # 추가
],
```

## 7. Nav2 설정 파일 작성

`src/robot_description/config/nav2_params.yaml` 파일 생성:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5  # 0.5m 이동해야 진행으로 판단
      movement_time_allowance: 10.0  # 10초 내 이동 없으면 실패

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25  # 목표 위치 허용 오차 25cm
      yaw_goal_tolerance: 0.25  # 목표 방향 허용 오차 0.25rad

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5  # 최대 전진 속도 0.5m/s
      max_vel_y: 0.0
      max_vel_theta: 1.0  # 최대 회전 속도 1.0rad/s
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5  # 전진 가속도 제한
      acc_lim_y: 0.0
      acc_lim_theta: 3.2  # 회전 가속도 제한
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20  # 전진 속도 샘플 개수
      vy_samples: 0
      vtheta_samples: 40  # 회전 속도 샘플 개수
      sim_time: 1.7  # 궤적 시뮬레이션 시간
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02  # 장애물 회피 가중치
      PathAlign.scale: 32.0  # 경로 정렬 가중치
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0  # 목표 정렬 가중치
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0  # 목표 거리 가중치
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom  # Odometry 기준 좌표계
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true  # 로봇 중심으로 이동하는 윈도우
      width: 3  # 로봇 주변 3x3m 영역
      height: 3
      resolution: 0.05  # 5cm 해상도
      robot_radius: 0.3  # 로봇 반경 30cm
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # 장애물 주변 55cm 팽창
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0  # 광선 추적 최대 3m
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5  # 장애물 인식 최대 2.5m
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map  # 전역 맵 좌표계
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true  # 미탐색 영역 추적
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "factory_map.yaml"

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # 목표 허용 오차 50cm
      use_astar: false  # Dijkstra 알고리즘 사용 (false=Dijkstra, true=A*)
      allow_unknown: true  # 미탐색 영역 통과 허용

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0  # 복구 동작 시 최대 회전 속도
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200  # 웨이포인트 도착 시 200ms 대기

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0]  # 최대 [x, y, theta] 속도
    min_velocity: [-0.5, 0.0, -1.0]  # 최소 [x, y, theta] 속도
    max_accel: [2.5, 0.0, 3.2]  # 최대 가속도
    max_decel: [-2.5, 0.0, -3.2]  # 최대 감속도
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2  # 회전에 의한 회전 노이즈
    alpha2: 0.2  # 전진에 의한 회전 노이즈
    alpha3: 0.2  # 전진에 의한 전진 노이즈
    alpha4: 0.2  # 회전에 의한 전진 노이즈
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60  # 사용할 레이저 빔 개수
    max_particles: 2000  # 최대 입자 개수 (위치 추정 정확도↑)
    min_particles: 500  # 최소 입자 개수
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true  # map→odom TF 발행
    transform_tolerance: 1.0
    update_min_a: 0.2  # 0.2rad(약 11도) 회전 시 업데이트
    update_min_d: 0.25  # 0.25m 이동 시 업데이트
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
```

## 8. Navigation Launch 파일 작성

`src/robot_description/launch/navigation_launch.py` 파일 생성:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    map_file = os.path.join(pkg_path, 'maps', 'factory_map.yaml')
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Map Server: 저장된 맵 로드
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
        ),

        # AMCL: 몬테카를로 위치 추정
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Controller Server: DWB 경로 추종
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Planner Server: Dijkstra 전역 경로 계획
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Behavior Server: 복구 행동 (spin, backup, wait)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # BT Navigator: 행동 트리 기반 네비게이션 로직
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),

        # Lifecycle Manager: Nav2 노드들의 생명주기 관리
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
```


## 9. 빌드 및 실행

### 9.1 패키지 빌드

```bash
cd ~/simple_simulator
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 9.2 실행

**터미널 1: 로봇 + SLAM**
```bash
cd ~/simple_simulator
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description spawn_robot_launch.py
```

**터미널 2: Nav2**
```bash
cd ~/simple_simulator
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description navigation_launch.py
```

**터미널 3: RViz2**
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

### 9.3 RViz2 설정

1. Fixed Frame: `map`
2. 추가할 Display:
   - Map → `/map` (저장된 맵)
   - RobotModel (로봇 모델)
   - LaserScan → `/scan` (LiDAR 데이터)
3. "2D Pose Estimate" 클릭 → 로봇 현재 위치 지정

## 10. 자율주행 테스트

### 10.1 목표 지점 설정 (RViz2)

1. "2D Goal Pose" 클릭
2. 맵에서 목표 위치 클릭 후 드래그 (방향 지정)

### 10.2 목표 지점 설정 (명령어)

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## 11. 시스템 구조

```
┌─────────┐
│  RViz2  │ ← 사용자 인터페이스
└────┬────┘
     │
┌────▼────────────────────────────────────┐
│            Nav2 Stack                   │
├─────────────────────────────────────────┤
│ Map Server    → 저장된 맵 제공          │
│ AMCL          → 위치 추정 (map↔odom)    │
│ Planner       → 전역 경로 계획          │
│ Controller    → 지역 경로 + 속도 제어   │
│ Behavior      → 복구 행동               │
│ BT Navigator  → 행동 트리 관리          │
└────┬────────────────────────────────────┘
     │ /cmd_vel
┌────▼────┐
│ Gazebo  │ ← 로봇 시뮬레이션
└─────────┘
```

**데이터 흐름:**
1. LiDAR 데이터 → AMCL → 로봇 위치 추정
2. 위치 + 저장된 맵 → Planner → 전역 경로 생성
3. 경로 + 실시간 LiDAR → Controller → 속도 명령 생성
4. 속도 명령 → Gazebo → 로봇 이동
