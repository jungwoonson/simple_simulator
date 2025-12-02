# URDF에서 Xacro로 전환

## 1. Xacro를 사용하는 이유

### URDF의 한계
- 반복되는 코드 (좌우 바퀴, 센서 등)
- 하드코딩된 수치 값
- 수동 계산된 관성 모멘트 (물리적 부정확)
- 유지보수 어려움

### Xacro의 장점
1. **변수 관리**: 모든 값을 한 곳에서 정의
2. **매크로**: 반복 구조 재사용
3. **수학 연산**: 관성 모멘트 자동 계산 (물리적 정확도↑)
4. **확장성**: 센서/링크 추가가 쉬움

## 2. 관성 모멘트 자동 계산

### 직육면체(Box)
- Ixx = (1/12) × m × (y² + z²)
- Iyy = (1/12) × m × (x² + z²)
- Izz = (1/12) × m × (x² + y²)

### 원통(Cylinder)
- Ixx, Iyy = (1/12) × m × (3r² + h²)
- Izz = (1/2) × m × r²

### 구(Sphere)
- Ixx = Iyy = Izz = (2/5) × m × r²

## 3. robot.xacro 파일 작성

`src/robot_description/resource/robot.xacro` 파일 생성:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">

    <!-- ========================================== -->
    <!-- 수학 상수 -->
    <!-- ========================================== -->
    <!-- pi는 xacro 내장 상수이므로 정의 불필요 -->
    <xacro:property name="pi_2" value="${pi/2}"/>

    <!-- ========================================== -->
    <!-- 로봇 기하학 파라미터 -->
    <!-- ========================================== -->
    
    <!-- Base(본체) -->
    <xacro:property name="base_length" value="0.6"/>
    <xacro:property name="base_width" value="0.4"/>
    <xacro:property name="base_height" value="0.2"/>
    <xacro:property name="base_mass" value="10"/>
    <xacro:property name="base_z_offset" value="0.05"/>

    <!-- Wheel(바퀴) -->
    <xacro:property name="wheel_radius" value="0.1"/>
    <xacro:property name="wheel_thickness" value="0.08"/>
    <xacro:property name="wheel_mass" value="1"/>
    <xacro:property name="wheel_separation" value="0.48"/>
    <xacro:property name="wheel_x_offset" value="0.15"/>
    <xacro:property name="wheel_z_offset" value="0"/>
    <xacro:property name="wheel_axis_direction" value="0 0 -1"/>

    <!-- Caster(캐스터) -->
    <xacro:property name="caster_radius" value="0.05"/>
    <xacro:property name="caster_mass" value="0.5"/>
    <xacro:property name="caster_x_offset" value="-0.2"/>
    <xacro:property name="caster_y_offset" value="0"/>
    <xacro:property name="caster_z_offset" value="-0.05"/>
    <xacro:property name="caster_friction_mu1" value="0.0"/>
    <xacro:property name="caster_friction_mu2" value="0.0"/>

    <!-- LiDAR -->
    <xacro:property name="lidar_radius" value="0.05"/>
    <xacro:property name="lidar_length" value="0.07"/>
    <xacro:property name="lidar_mass" value="0.3"/>
    <xacro:property name="lidar_x_offset" value="0.2"/>
    <xacro:property name="lidar_y_offset" value="0"/>
    <xacro:property name="lidar_z_offset" value="0.2"/>

    <!-- ========================================== -->
    <!-- 색상 정의 -->
    <!-- ========================================== -->
    <xacro:property name="color_blue" value="0 0 0.8 1"/>
    <xacro:property name="color_black" value="0 0 0 1"/>
    <xacro:property name="color_gray" value="0.5 0.5 0.5 1"/>
    <xacro:property name="color_red" value="1 0 0 1"/>

    <!-- ========================================== -->
    <!-- ROS/Gazebo 토픽 및 프레임 -->
    <!-- ========================================== -->
    <xacro:property name="odom_frame" value="odom"/>
    <xacro:property name="base_frame" value="base_link"/>
    <xacro:property name="cmd_vel_topic" value="cmd_vel"/>
    <xacro:property name="tf_topic" value="/tf"/>
    <xacro:property name="scan_topic" value="scan"/>

    <!-- ========================================== -->
    <!-- 센서 파라미터 -->
    <!-- ========================================== -->
    
    <!-- LiDAR -->
    <xacro:property name="lidar_update_rate" value="10"/>
    <xacro:property name="lidar_samples" value="360"/>
    <xacro:property name="lidar_resolution" value="1"/>
    <xacro:property name="lidar_min_angle" value="${-pi}"/>
    <xacro:property name="lidar_max_angle" value="${pi}"/>
    <xacro:property name="lidar_min_range" value="0.12"/>
    <xacro:property name="lidar_max_range" value="10.0"/>
    <xacro:property name="lidar_visualize" value="true"/>

    <!-- Odometry -->
    <xacro:property name="odom_publish_frequency" value="50"/>

    <!-- ========================================== -->
    <!-- Gazebo 플러그인 파라미터 -->
    <!-- ========================================== -->
    <xacro:property name="render_engine" value="ogre2"/>

    <!-- ========================================== -->
    <!-- 관성 모멘트 계산 매크로 -->
    <!-- ========================================== -->
    
    <!-- 직육면체(Box) -->
    <xacro:macro name="box_inertia" params="mass x y z">
        <inertial>
            <mass value="${mass}"/>
            <inertia 
                ixx="${(1/12) * mass * (y*y + z*z)}" 
                ixy="0.0" 
                ixz="0.0"
                iyy="${(1/12) * mass * (x*x + z*z)}" 
                iyz="0.0" 
                izz="${(1/12) * mass * (x*x + y*y)}"/>
        </inertial>
    </xacro:macro>

    <!-- 원통(Cylinder) -->
    <xacro:macro name="cylinder_inertia" params="mass radius length">
        <inertial>
            <mass value="${mass}"/>
            <inertia 
                ixx="${(1/12) * mass * (3 * radius * radius + length * length)}" 
                ixy="0.0" 
                ixz="0.0"
                iyy="${(1/12) * mass * (3 * radius * radius + length * length)}" 
                iyz="0.0" 
                izz="${(1/2) * mass * radius * radius}"/>
        </inertial>
    </xacro:macro>

    <!-- 구(Sphere) -->
    <xacro:macro name="sphere_inertia" params="mass radius">
        <inertial>
            <mass value="${mass}"/>
            <inertia 
                ixx="${(2/5) * mass * radius * radius}" 
                ixy="0.0" 
                ixz="0.0"
                iyy="${(2/5) * mass * radius * radius}" 
                iyz="0.0" 
                izz="${(2/5) * mass * radius * radius}"/>
        </inertial>
    </xacro:macro>

    <!-- ========================================== -->
    <!-- 색상 재료 매크로 -->
    <!-- ========================================== -->
    <xacro:macro name="material_color" params="name rgba">
        <material name="${name}">
            <color rgba="${rgba}"/>
        </material>
    </xacro:macro>

    <!-- ========================================== -->
    <!-- Base Link -->
    <!-- ========================================== -->
    <link name="${base_frame}">
        <visual>
            <origin xyz="0 0 ${base_z_offset}" rpy="0 0 0"/>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <xacro:material_color name="blue" rgba="${color_blue}"/>
        </visual>
        <collision>
            <origin xyz="0 0 ${base_z_offset}" rpy="0 0 0"/>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
        </collision>
        <xacro:box_inertia mass="${base_mass}" 
                          x="${base_length}" 
                          y="${base_width}" 
                          z="${base_height}"/>
    </link>

    <!-- ========================================== -->
    <!-- Wheel 매크로 -->
    <!-- ========================================== -->
    <xacro:macro name="wheel" params="prefix reflect">
        <link name="${prefix}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_thickness}"/>
                </geometry>
                <xacro:material_color name="black" rgba="${color_black}"/>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_thickness}"/>
                </geometry>
            </collision>
            <xacro:cylinder_inertia mass="${wheel_mass}" 
                                   radius="${wheel_radius}" 
                                   length="${wheel_thickness}"/>
        </link>

        <joint name="${prefix}_wheel_joint" type="continuous">
            <parent link="${base_frame}"/>
            <child link="${prefix}_wheel"/>
            <origin xyz="${wheel_x_offset} ${reflect * wheel_separation/2} ${wheel_z_offset}" 
                    rpy="${pi_2} 0 0"/>
            <axis xyz="${wheel_axis_direction}"/>
        </joint>
    </xacro:macro>

    <!-- Wheel 인스턴스 생성 -->
    <xacro:wheel prefix="left" reflect="1"/>
    <xacro:wheel prefix="right" reflect="-1"/>

    <!-- ========================================== -->
    <!-- Caster 매크로 -->
    <!-- ========================================== -->
    <xacro:macro name="caster" params="name x_offset y_offset z_offset">
        <link name="${name}">
            <visual>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
                <xacro:material_color name="gray" rgba="${color_gray}"/>
            </visual>
            <collision>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
            </collision>
            <xacro:sphere_inertia mass="${caster_mass}" 
                                 radius="${caster_radius}"/>
        </link>

        <joint name="${name}_joint" type="fixed">
            <parent link="${base_frame}"/>
            <child link="${name}"/>
            <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 0 0"/>
        </joint>

        <gazebo reference="${name}">
            <mu1>${caster_friction_mu1}</mu1>
            <mu2>${caster_friction_mu2}</mu2>
        </gazebo>
    </xacro:macro>

    <!-- Caster 인스턴스 생성 -->
    <xacro:caster name="rear_caster" 
                  x_offset="${caster_x_offset}" 
                  y_offset="${caster_y_offset}" 
                  z_offset="${caster_z_offset}"/>

    <!-- ========================================== -->
    <!-- LiDAR 매크로 -->
    <!-- ========================================== -->
    <xacro:macro name="lidar_sensor" params="name parent x_offset y_offset z_offset">
        <link name="${name}">
            <visual>
                <geometry>
                    <cylinder radius="${lidar_radius}" length="${lidar_length}"/>
                </geometry>
                <xacro:material_color name="red" rgba="${color_red}"/>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${lidar_radius}" length="${lidar_length}"/>
                </geometry>
            </collision>
            <xacro:cylinder_inertia mass="${lidar_mass}" 
                                   radius="${lidar_radius}" 
                                   length="${lidar_length}"/>
        </link>

        <joint name="${name}_joint" type="fixed">
            <parent link="${parent}"/>
            <child link="${name}"/>
            <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 0 0"/>
        </joint>

        <gazebo reference="${name}">
            <sensor name="gpu_lidar" type="gpu_lidar">
                <topic>${scan_topic}</topic>
                <update_rate>${lidar_update_rate}</update_rate>
                <lidar>
                    <scan>
                        <horizontal>
                            <samples>${lidar_samples}</samples>
                            <resolution>${lidar_resolution}</resolution>
                            <min_angle>${lidar_min_angle}</min_angle>
                            <max_angle>${lidar_max_angle}</max_angle>
                        </horizontal>
                    </scan>
                    <range>
                        <min>${lidar_min_range}</min>
                        <max>${lidar_max_range}</max>
                    </range>
                </lidar>
                <visualize>${lidar_visualize}</visualize>
            </sensor>
        </gazebo>
    </xacro:macro>

    <!-- LiDAR 인스턴스 생성 -->
    <xacro:lidar_sensor name="lidar_link" 
                        parent="${base_frame}"
                        x_offset="${lidar_x_offset}" 
                        y_offset="${lidar_y_offset}" 
                        z_offset="${lidar_z_offset}"/>

    <!-- ========================================== -->
    <!-- Gazebo Plugins -->
    <!-- ========================================== -->
    
    <!-- Differential Drive Plugin -->
    <gazebo>
        <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
            <left_joint>left_wheel_joint</left_joint>
            <right_joint>right_wheel_joint</right_joint>
            <wheel_separation>${wheel_separation}</wheel_separation>
            <wheel_radius>${wheel_radius}</wheel_radius>
            <odom_publish_frequency>${odom_publish_frequency}</odom_publish_frequency>
            <topic>${cmd_vel_topic}</topic>
            <tf_topic>${tf_topic}</tf_topic>
            <frame_id>${odom_frame}</frame_id>
            <child_frame_id>${base_frame}</child_frame_id>
        </plugin>
    </gazebo>

    <!-- Sensors System Plugin -->
    <gazebo>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>${render_engine}</render_engine>
        </plugin>
    </gazebo>

    <!-- Joint State Publisher Plugin -->
    <gazebo>
        <plugin filename="gz-sim-joint-state-publisher-system"
                name="gz::sim::systems::JointStatePublisher">
        </plugin>
    </gazebo>

</robot>
```

## 4. setup.py 수정

`src/robot_description/setup.py`에서 xacro 파일 추가:

```python
data_files=[
    # ... 기존 내용 ...
    ('share/' + package_name + '/resource', glob('resource/*.xacro')),  # 추가
    # ... 기존 내용 ...
],
```

## 5. Launch 파일 수정

`src/robot_description/launch/spawn_robot.launch.py` 수정:

```python
# import 추가
from launch.substitutions import Command

# 파일 경로 변경
urdf_file = os.path.join(pkg_path, 'resource', 'robot.xacro')  # .urdf → .xacro

# 파일 읽기 방식 변경
robot_desc = Command(['xacro ', urdf_file])  # open().read() 대신 Command 사용
```

## 6. 빌드 및 실행

```bash
cd ~/workspace/simple_simulator
rm -rf build install log  # 기존 빌드 삭제 (문제 발생 시)
colcon build
source install/setup.bash
ros2 launch robot_description spawn_robot.launch.py
```

## 7. 주요 개선사항

| 항목 | URDF | Xacro |
|------|------|-------|
| 바퀴 반지름 변경 | 4곳 수정 | 1곳 수정 |
| 관성 재계산 | 수동 계산 | 자동 계산 |
| 센서 추가 | 전체 코드 | 매크로 호출 |
| 코드량 | ~400줄 | ~300줄 |
