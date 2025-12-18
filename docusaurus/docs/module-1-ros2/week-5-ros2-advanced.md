---
title: Week 5 - Advanced ROS 2 Topics
description: Master custom messages, plugins, debugging tools, and performance optimization in ROS 2
sidebar_position: 3
hardware_tier: intermediate
learning_objectives:
  - Create custom message and service definitions for specialized robotics applications
  - Implement ROS 2 plugins for extensible architectures
  - Use debugging tools (rqt, ros2 bag, ros2 doctor) for troubleshooting
  - Optimize ROS 2 systems for real-time performance
estimated_time_minutes: 150
---

# Week 5: Advanced ROS 2 Topics

## Custom Message Types

While ROS 2 provides standard message types (`std_msgs`, `sensor_msgs`, `geometry_msgs`), custom messages enable domain-specific communication.

### Creating a Custom Message Package

```bash
# Create a package for interfaces (messages, services, actions)
ros2 pkg create --build-type ament_cmake robot_interfaces

# Directory structure:
# robot_interfaces/
# ├── msg/
# │   ├── RobotState.msg
# │   └── BatteryStatus.msg
# ├── srv/
# │   └── SetMode.srv
# ├── action/
# │   └── NavigateToGoal.action
# ├── CMakeLists.txt
# └── package.xml
```

### Defining Custom Messages

```msg
# msg/RobotState.msg
std_msgs/Header header

# Robot identification
string robot_id
string robot_type

# Position and velocity
geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# System status
uint8 MODE_IDLE = 0
uint8 MODE_TELEOP = 1
uint8 MODE_AUTONOMOUS = 2
uint8 MODE_EMERGENCY_STOP = 3
uint8 mode

# Battery information
float32 battery_voltage
float32 battery_percentage
bool is_charging

# Diagnostic flags
bool motors_enabled
bool sensors_ok
string[] active_warnings
```

```msg
# msg/BatteryStatus.msg
std_msgs/Header header

float32 voltage          # Battery voltage in volts
float32 current          # Current draw in amperes
float32 percentage       # Remaining capacity (0-100%)
float32 temperature      # Battery temperature in Celsius
uint32 charge_cycles     # Total charge cycles
bool is_charging
bool is_low_battery      # Below 20%
string health_status     # "good", "fair", "poor", "critical"
```

### Defining Custom Services

```srv
# srv/SetMode.srv
# Request
uint8 MODE_IDLE = 0
uint8 MODE_TELEOP = 1
uint8 MODE_AUTONOMOUS = 2
uint8 MODE_EMERGENCY_STOP = 3
uint8 requested_mode
string reason  # Optional reason for mode change
---
# Response
bool success
uint8 current_mode
string message
```

### Defining Custom Actions

```action
# action/NavigateToGoal.action
# Goal
geometry_msgs/PoseStamped target_pose
float32 tolerance  # Distance tolerance in meters
---
# Result
bool success
geometry_msgs/PoseStamped final_pose
float32 final_distance
float64 total_time
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 estimated_time_remaining
```

### Building Custom Messages

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/BatteryStatus.msg"
  "srv/SetMode.srv"
  "action/NavigateToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
```

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>robot_interfaces</name>
  <version>0.1.0</version>
  <description>Custom interfaces for robot system</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Build and use custom messages:**
```bash
# Build the interfaces package
cd ~/ros2_ws
colcon build --packages-select robot_interfaces
source install/setup.bash

# Verify message creation
ros2 interface show robot_interfaces/msg/RobotState
ros2 interface show robot_interfaces/srv/SetMode
```

### Using Custom Messages in Python Nodes

```python
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotState, BatteryStatus
from robot_interfaces.srv import SetMode
from geometry_msgs.msg import Pose, Twist

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Publisher for robot state
        self.state_pub = self.create_publisher(
            RobotState,
            'robot_state',
            10
        )

        # Service for mode changes
        self.mode_service = self.create_service(
            SetMode,
            'set_mode',
            self.set_mode_callback
        )

        # Timer to publish state
        self.timer = self.create_timer(0.5, self.publish_state)
        self.current_mode = RobotState.MODE_IDLE

    def publish_state(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.robot_id = "robot_001"
        msg.robot_type = "mobile_manipulator"
        msg.mode = self.current_mode

        # Fill in pose and velocity
        msg.pose = Pose()
        msg.velocity = Twist()

        # Battery information
        msg.battery_voltage = 24.5
        msg.battery_percentage = 85.0
        msg.is_charging = False

        # Status flags
        msg.motors_enabled = True
        msg.sensors_ok = True
        msg.active_warnings = []

        self.state_pub.publish(msg)

    def set_mode_callback(self, request, response):
        """Service callback to change robot mode"""
        valid_modes = [
            RobotState.MODE_IDLE,
            RobotState.MODE_TELEOP,
            RobotState.MODE_AUTONOMOUS,
            RobotState.MODE_EMERGENCY_STOP
        ]

        if request.requested_mode in valid_modes:
            self.current_mode = request.requested_mode
            response.success = True
            response.current_mode = self.current_mode
            response.message = f"Mode changed to {self.current_mode}"
            self.get_logger().info(f'Mode changed: {request.reason}')
        else:
            response.success = False
            response.current_mode = self.current_mode
            response.message = "Invalid mode requested"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Plugins: Extensible Architecture

**Plugins** enable runtime-loaded components for navigation algorithms, sensor drivers, and controllers.

### Creating a Plugin Interface

```cpp
// include/robot_plugins/controller_interface.hpp
#ifndef ROBOT_PLUGINS__CONTROLLER_INTERFACE_HPP_
#define ROBOT_PLUGINS__CONTROLLER_INTERFACE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace robot_plugins
{

class ControllerInterface
{
public:
  virtual ~ControllerInterface() = default;

  // Pure virtual methods that plugins must implement
  virtual void initialize() = 0;
  virtual geometry_msgs::msg::Twist computeVelocity(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

}  // namespace robot_plugins

#endif  // ROBOT_PLUGINS__CONTROLLER_INTERFACE_HPP_
```

### Implementing a Plugin

```cpp
// src/simple_controller_plugin.cpp
#include "robot_plugins/controller_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace robot_plugins
{

class SimpleController : public ControllerInterface
{
public:
  void initialize() override
  {
    // Initialize controller parameters
  }

  geometry_msgs::msg::Twist computeVelocity(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) override
  {
    geometry_msgs::msg::Twist cmd_vel;

    // Simple obstacle avoidance logic
    float min_distance = *std::min_element(
      scan->ranges.begin(), scan->ranges.end());

    if (min_distance < 0.5) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.5;
    } else {
      cmd_vel.linear.x = 0.3;
      cmd_vel.angular.z = 0.0;
    }

    return cmd_vel;
  }
};

}  // namespace robot_plugins

// Register the plugin
PLUGINLIB_EXPORT_CLASS(robot_plugins::SimpleController, robot_plugins::ControllerInterface)
```

## Debugging Tools

### 1. RQT: Graphical Debugging Suite

```bash
# Launch RQT with common plugins
rqt

# Specific RQT tools:
rqt_graph         # Visualize node/topic graph
rqt_console       # View log messages
rqt_plot          # Real-time data plotting
rqt_bag           # Playback recorded data
rqt_reconfigure   # Dynamic parameter adjustment
```

### 2. ROS 2 Bag: Recording and Playback

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /robot_state /cmd_vel /scan

# Record with compression and max file size
ros2 bag record -a \
  --compression-mode file \
  --compression-format zstd \
  --max-bag-size 1000000000  # 1GB

# Play back recorded data
ros2 bag play my_bag_file

# Play at different speeds
ros2 bag play my_bag_file --rate 0.5  # Half speed
ros2 bag play my_bag_file --rate 2.0  # Double speed

# Inspect bag contents
ros2 bag info my_bag_file
```

### 3. ROS 2 Doctor: System Diagnostics

```bash
# Run comprehensive system check
ros2 doctor

# Generate detailed report
ros2 doctor --report

# Example output checks:
# - ROS 2 installation
# - Network configuration
# - Environment variables
# - Package dependencies
```

### 4. Topic/Node Introspection

```bash
# Monitor topic bandwidth
ros2 topic bw /scan

# Monitor topic publish rate
ros2 topic hz /scan

# Echo topic with filter
ros2 topic echo /robot_state --field battery_percentage

# Node performance
ros2 node info /robot_controller --verbose
```

## Performance Optimization

### 1. Real-Time Executor Configuration

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class OptimizedNode(Node):
    def __init__(self):
        super().__init__('optimized_node')

        # Use reentrant callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()

        # High-priority timer
        self.control_timer = self.create_timer(
            0.01,  # 100 Hz control loop
            self.control_callback,
            callback_group=self.callback_group
        )

def main():
    rclpy.init()
    node = OptimizedNode()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
```

### 2. Zero-Copy Communication (Intra-Process)

```python
# Enable intra-process communication for reduced latency
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy

class IntraProcNode(Node):
    def __init__(self):
        super().__init__(
            'intra_proc_node',
            enable_rosout=False,  # Reduce overhead
        )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Minimize latency
        )

        self.pub = self.create_publisher(
            LaserScan,
            'scan',
            qos_profile=qos
        )
```

### 3. Performance Profiling

```bash
# Install performance tools
sudo apt install ros-humble-ros2-tracing

# Trace node execution
ros2 trace

# Analyze with Chrome tracing
chrome://tracing
```

## Key Takeaways

1. **Custom messages** enable domain-specific data structures for specialized robotics applications
2. **Plugins** provide runtime-extensible architectures for controllers, planners, and sensors
3. **Debugging tools** (rqt, ros2 bag, ros2 doctor) are essential for troubleshooting complex systems
4. **Performance optimization** techniques (multi-threading, intra-process, zero-copy) enable real-time robotics
5. **Best practices** include profiling, systematic testing, and incremental optimization

## Next Steps

In **Module 2**, we'll transition to **Gazebo Simulation** to create virtual environments for testing ROS 2 systems before deploying to physical robots.

---

**References:**
- [ROS 2 Custom Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 Pluginlib](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
- [ROS 2 Performance Tuning](https://docs.ros.org/en/humble/Tutorials/Demos/Real-Time-Programming.html)
