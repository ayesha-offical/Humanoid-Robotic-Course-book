---
title: Week 6 - Gazebo Simulation and URDF/SDF Formats
description: Master the Gazebo Physics Engine, URDF robot descriptions, and world modeling for classical robotics simulation
sidebar_position: 1
hardware_tier: intermediate
learning_objectives:
  - Set up Gazebo Classic and Gazebo Sim (Ignition) environments
  - Create robot descriptions using URDF and SDF formats
  - Configure physics parameters and collision models
  - Spawn and control robots in simulated worlds
estimated_time_minutes: 150
---

# Week 6: Gazebo Simulation and URDF/SDF Formats

## Introduction to Gazebo

**Gazebo** is an open-source 3D robotics simulator that provides:
- **Physics simulation**: ODE, Bullet, Simbody, DART engines
- **Sensor simulation**: LIDAR, cameras, IMUs, GPS, contact sensors
- **ROS 2 integration**: Seamless topic/service communication
- **Plugin system**: Extensible with custom sensors and controllers

### Gazebo Classic vs Gazebo Sim (Ignition)

| Feature | Gazebo Classic | Gazebo Sim (Ignition) |
|---------|----------------|------------------------|
| Physics engines | ODE, Bullet, Simbody, DART | DART (default) |
| Rendering | OGRE 1.x | OGRE 2.x (PBR) |
| Format | SDF 1.6 | SDF 1.8+ |
| ROS 2 support | gazebo_ros_pkgs | ros_gz_bridge |
| Performance | Good | Excellent (multi-threaded) |

For this course, we'll focus on **Gazebo Sim** (modern version) with ROS 2 Humble.

## Installation and Setup

```bash
# Install Gazebo Sim (Fortress) for ROS 2 Humble
sudo apt update
sudo apt install ros-humble-ros-gz

# Verify installation
gz sim --version

# Install additional ROS 2 Gazebo packages
sudo apt install \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-image

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

## URDF: Unified Robot Description Format

**URDF** (XML-based) describes robot kinematics and geometry:

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.183" ixy="0.0" ixz="0.0"
               iyy="0.383" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Wheel Link -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00152" ixy="0" ixz="0"
               iyy="0.00152" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.2 0.25 -0.1" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Complete Differential Drive Robot URDF

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Xacro properties for parameterization -->
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Base Link -->
  <link name="base_footprint"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="robot_blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.183" ixy="0.0" ixz="0.0"
               iyy="0.483" iyz="0.0" izz="0.6"/>
    </inertial>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- Wheel Macro -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="wheel_black">
          <color rgba="0.1 0.1 0.1 1"/>
        </material>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.00203" ixy="0" ixz="0"
                 iyy="0.00203" iyz="0" izz="0.00375"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${-base_length/4} ${reflect*base_width/2 + reflect*wheel_width/2} ${-base_height/2}"
              rpy="${-pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

  <!-- Caster Wheel -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="caster_gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>
            <mu2>0.1</mu2>
          </ode>
        </friction>
      </surface>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0"
               iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="${base_length/3} 0 ${-base_height/2 - 0.05}" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Differential Drive Plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${base_width + wheel_width}</wheel_separation>
      <wheel_diameter>${2*wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
  </gazebo>
</robot>
```

**Save and visualize:**
```bash
# Check URDF for errors
check_urdf robot.urdf.xacro

# Convert xacro to URDF
xacro robot.urdf.xacro > robot.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf
```

## SDF: Simulation Description Format

**SDF** is Gazebo's native format, more feature-rich than URDF:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_world">
    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
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
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include robot model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Plugin for ROS 2 bridge -->
    <plugin name="gz_ros2_control" filename="libgz_ros2_control.so">
      <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
    </plugin>
  </world>
</sdf>
```

## Launching Gazebo with ROS 2

### Launch File for Gazebo + Robot Spawn

```python
# launch/gazebo_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_description')

    # Paths
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
```

**Run the simulation:**
```bash
ros2 launch my_robot_description gazebo_robot.launch.py

# Control the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

## Physics Configuration

### Tuning Physics Parameters

```xml
<!-- In URDF/SDF collision elements -->
<collision>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>      <!-- Coefficient of friction -->
        <mu2>0.8</mu2>    <!-- Secondary friction -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e6</kp>      <!-- Stiffness -->
        <kd>1.0</kd>      <!-- Damping -->
        <max_vel>0.01</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Key Takeaways

1. **Gazebo Sim** is the modern, high-performance successor to Gazebo Classic
2. **URDF** describes robot kinematics; **SDF** extends it with simulation-specific features
3. **Physics parameters** (friction, damping, stiffness) must be tuned for realistic behavior
4. **ROS 2 integration** via `ros_gz_bridge` enables seamless topic communication
5. **Launch files** orchestrate Gazebo, robot spawning, and ROS 2 nodes

## Next Steps

In **Week 7**, we'll add **sensor simulation** (LIDAR, cameras, IMUs) to create perception-enabled robots in Gazebo.

---

**References:**
- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS 2 Gazebo Integration](https://github.com/gazebosim/ros_gz)
