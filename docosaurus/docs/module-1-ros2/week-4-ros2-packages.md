---
title: Week 4 - Building ROS 2 Packages with Python
description: Create production-ready ROS 2 packages with launch files, parameters, and package management
sidebar_position: 2
hardware_tier: beginner
learning_objectives:
  - Structure and build ROS 2 packages using ament_python
  - Use launch files to orchestrate multi-node systems
  - Manage runtime parameters with YAML configuration files
  - Implement best practices for package organization and dependencies
estimated_time_minutes: 120
---

# Week 4: Building ROS 2 Packages with Python

## ROS 2 Package Structure

A **ROS 2 package** is the organizational unit for distributing code. Packages contain:
- **Source code**: Python scripts, C++ files
- **Configuration**: Launch files, parameter files, URDF models
- **Metadata**: `package.xml` (dependencies), `setup.py` (build instructions)

### Creating a New ROS 2 Package

```bash
# Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python my_robot_controller \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# Expected directory structure:
# my_robot_controller/
# ├── my_robot_controller/
# │   ├── __init__.py
# ├── resource/
# │   └── my_robot_controller
# ├── test/
# │   ├── test_copyright.py
# │   ├── test_flake8.py
# │   └── test_pep257.py
# ├── package.xml
# ├── setup.py
# ├── setup.cfg
# └── README.md
```

### Package Metadata: `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>0.1.0</version>
  <description>Robot controller package with teleoperation and sensor processing</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <!-- Testing dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Build Configuration: `setup.py`

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Include parameter files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robot controller package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_controller.robot_controller:main',
            'sensor_processor = my_robot_controller.sensor_processor:main',
        ],
    },
)
```

## Creating a Robot Controller Node

Let's build a complete robot controller with velocity commands and sensor processing.

```python
# my_robot_controller/robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters with defaults
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('obstacle_distance', 0.5)

        # Get parameter values
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.obstacle_dist = self.get_parameter('obstacle_distance').value

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.obstacle_detected = False
        self.get_logger().info(
            f'Robot Controller started - '
            f'max_linear: {self.max_linear}, '
            f'max_angular: {self.max_angular}'
        )

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Check front sector (330-30 degrees)
        front_ranges = (
            msg.ranges[-30:] + msg.ranges[:30]  # Wrap around 0 degrees
        )

        # Filter out invalid readings
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.obstacle_detected = min_distance < self.obstacle_dist

            if self.obstacle_detected:
                self.get_logger().warn(
                    f'Obstacle detected at {min_distance:.2f}m',
                    throttle_duration_sec=1.0
                )

    def control_loop(self):
        """Main control loop - simple obstacle avoidance"""
        twist = Twist()

        if self.obstacle_detected:
            # Rotate to avoid obstacle
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular
        else:
            # Move forward
            twist.linear.x = self.max_linear
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files: Orchestrating Multi-Node Systems

**Launch files** in ROS 2 (Python-based) enable:
- Starting multiple nodes simultaneously
- Setting parameters and remapping topics
- Conditional node execution
- Namespace management for multi-robot systems

### Creating a Launch File

```python
# launch/robot_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('my_robot_controller')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Robot namespace'
    )

    # Path to parameter file
    config_file = os.path.join(
        pkg_dir,
        'config',
        'robot_params.yaml'
    )

    # Robot controller node
    robot_controller_node = Node(
        package='my_robot_controller',
        executable='robot_controller',
        name='robot_controller',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True,
    )

    # Sensor processor node
    sensor_processor_node = Node(
        package='my_robot_controller',
        executable='sensor_processor',
        name='sensor_processor',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[config_file],
        output='screen',
    )

    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'robot_view.rviz')],
        condition=LaunchCondition('false'),  # Disabled by default
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        robot_controller_node,
        sensor_processor_node,
    ])
```

**Run the launch file:**
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash

# Launch the system
ros2 launch my_robot_controller robot_system.launch.py

# Launch with custom parameters
ros2 launch my_robot_controller robot_system.launch.py \
  use_sim_time:=true \
  robot_name:=my_robot
```

## Parameter Management with YAML Files

**Parameter files** centralize configuration:

```yaml
# config/robot_params.yaml
/**:
  ros__parameters:
    # Robot controller parameters
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    obstacle_distance: 0.5

    # Sensor processor parameters
    sensor_update_rate: 10.0
    filter_size: 5

    # TF parameters
    publish_tf: true
    base_frame: "base_link"
    odom_frame: "odom"
```

### Loading Parameters Dynamically

```python
# In your node's __init__ method:
self.declare_parameters(
    namespace='',
    parameters=[
        ('max_linear_speed', 0.5),
        ('max_angular_speed', 1.0),
        ('obstacle_distance', 0.5),
    ]
)

# Get all parameters
params = self.get_parameters([
    'max_linear_speed',
    'max_angular_speed',
    'obstacle_distance'
])

# Parameter change callback
self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'max_linear_speed':
            self.max_linear = param.value
            self.get_logger().info(f'Updated max_linear_speed to {param.value}')
    return SetParametersResult(successful=True)
```

**Update parameters at runtime:**
```bash
# Set a parameter
ros2 param set /robot_controller max_linear_speed 0.8

# Get a parameter
ros2 param get /robot_controller max_linear_speed

# Load parameters from file
ros2 param load /robot_controller config/robot_params.yaml
```

## Building and Testing the Package

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_controller

# Source the workspace
source install/setup.bash

# Run tests
colcon test --packages-select my_robot_controller
colcon test-result --verbose

# Run a specific node
ros2 run my_robot_controller robot_controller

# Check package info
ros2 pkg list | grep my_robot_controller
ros2 pkg xml my_robot_controller
```

## Best Practices for ROS 2 Packages

1. **Use descriptive names**: `my_robot_controller`, not `pkg1`
2. **Declare all dependencies**: In both `package.xml` and `setup.py`
3. **Use parameters**: Avoid hardcoding values in code
4. **Logging**: Use `self.get_logger()` with appropriate levels (DEBUG, INFO, WARN, ERROR)
5. **Error handling**: Validate inputs, handle exceptions gracefully
6. **Documentation**: Add docstrings and README files
7. **Testing**: Write unit tests for critical functionality

## Key Takeaways

1. **ROS 2 packages** organize code, configuration, and metadata into reusable units
2. **Launch files** orchestrate multi-node systems with parameters and arguments
3. **Parameter files** (YAML) centralize configuration for easy tuning
4. **ament_python** build system handles package installation and dependencies
5. **Best practices** ensure maintainable, reusable robotics software

## Next Steps

In **Week 5**, we'll explore advanced ROS 2 topics including custom message types, plugins, debugging tools, and performance optimization.

---

**References:**
- [ROS 2 Python Package Creation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
