---
title: Week 7 - Sensor Simulation in Gazebo
description: Implement LIDAR, cameras, IMUs, and GPS sensors for perception-enabled robotics in Gazebo
sidebar_position: 2
hardware_tier: intermediate
learning_objectives:
  - Add LIDAR sensors for obstacle detection and mapping
  - Integrate RGB and depth cameras for vision-based perception
  - Configure IMU sensors for orientation and acceleration measurement
  - Process sensor data in ROS 2 nodes for autonomous navigation
estimated_time_minutes: 150
---

# Week 7: Sensor Simulation in Gazebo

## Overview of Gazebo Sensors

Gazebo provides physics-based sensor simulation for:
- **LIDAR** (2D/3D laser scanners)
- **Cameras** (RGB, depth, RGB-D, stereo)
- **IMU** (Inertial Measurement Units)
- **GPS** (Global Positioning System)
- **Contact sensors** (collision detection)
- **Force-torque sensors**

All sensors publish standard ROS 2 messages for seamless integration with navigation and perception stacks.

## LIDAR Sensor Simulation

### Adding a 2D LIDAR to URDF

```xml
<!-- Add to your robot URDF -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="lidar_black">
      <color rgba="0.1 0.1 0.1 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" ixy="0" ixz="0"
             iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>

<!-- Gazebo LIDAR Plugin -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate>
    <topic>scan</topic>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </lidar>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Processing LIDAR Data in ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('LIDAR Processor started')

    def scan_callback(self, msg: LaserScan):
        """Process LIDAR scan data"""
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)

        # Filter out invalid readings
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        valid_ranges = ranges[valid_mask]

        if len(valid_ranges) == 0:
            return

        # Find closest obstacle
        min_distance = np.min(valid_ranges)
        min_index = np.argmin(ranges)
        min_angle = msg.angle_min + min_index * msg.angle_increment

        # Detect obstacles in sectors
        front_sector = self.get_sector_min(ranges, msg, -30, 30)
        left_sector = self.get_sector_min(ranges, msg, 30, 90)
        right_sector = self.get_sector_min(ranges, msg, -90, -30)

        self.get_logger().info(
            f'Closest: {min_distance:.2f}m at {np.degrees(min_angle):.0f}° | '
            f'Front: {front_sector:.2f}m | Left: {left_sector:.2f}m | Right: {right_sector:.2f}m',
            throttle_duration_sec=1.0
        )

    def get_sector_min(self, ranges, msg, start_deg, end_deg):
        """Get minimum distance in a sector"""
        start_angle = np.radians(start_deg)
        end_angle = np.radians(end_deg)

        start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_idx = int((end_angle - msg.angle_min) / msg.angle_increment)

        sector_ranges = ranges[start_idx:end_idx]
        valid = sector_ranges[(sector_ranges >= msg.range_min) & (sector_ranges <= msg.range_max)]

        return np.min(valid) if len(valid) > 0 else msg.range_max

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Camera Sensor Simulation

### RGB Camera

```xml
<!-- Add camera link to URDF -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.03 0.08 0.03"/>
    </geometry>
    <material name="camera_dark">
      <color rgba="0.2 0.2 0.2 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.03 0.08 0.03"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.00006" ixy="0" ixz="0"
             iyy="0.00002" iyz="0" izz="0.00006"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo Camera Plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <topic>camera/image_raw</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Depth Camera (RGB-D)

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <update_rate>20</update_rate>
    <topic>camera/depth</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R_FLOAT32</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Processing Camera Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')

        self.bridge = CvBridge()

        # RGB camera subscriber
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Depth camera subscriber
        self.depth_sub = self.create_subscription(
            Image,
            'camera/depth',
            self.depth_callback,
            10
        )

        self.get_logger().info('Camera Processor started')

    def image_callback(self, msg: Image):
        """Process RGB image"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Example: Detect red objects
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area > 500:  # Minimum area threshold
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        self.get_logger().info(
                            f'Red object detected at ({cx}, {cy}), area: {area:.0f}',
                            throttle_duration_sec=1.0
                        )

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def depth_callback(self, msg: Image):
        """Process depth image"""
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Get depth at center of image
            h, w = depth_image.shape
            center_depth = depth_image[h//2, w//2]

            if not np.isnan(center_depth) and not np.isinf(center_depth):
                self.get_logger().info(
                    f'Center depth: {center_depth:.2f}m',
                    throttle_duration_sec=1.0
                )

        except Exception as e:
            self.get_logger().error(f'Depth processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Sensor Simulation

### Adding IMU to URDF

```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0" ixz="0"
             iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo IMU Plugin -->
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <update_rate>100</update_rate>
    <topic>imu/data</topic>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <always_on>true</always_on>
  </sensor>
</gazebo>
```

### Processing IMU Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.get_logger().info('IMU Processor started')

    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        # Extract orientation (quaternion)
        orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        rot = R.from_quat(orientation)
        euler = rot.as_euler('xyz', degrees=True)
        roll, pitch, yaw = euler

        # Extract angular velocity
        angular_vel = msg.angular_velocity
        omega_x = angular_vel.x
        omega_y = angular_vel.y
        omega_z = angular_vel.z

        # Extract linear acceleration
        linear_acc = msg.linear_acceleration
        acc_x = linear_acc.x
        acc_y = linear_acc.y
        acc_z = linear_acc.z

        # Compute total acceleration magnitude
        acc_magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

        self.get_logger().info(
            f'Orientation: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}° | '
            f'Acc: {acc_magnitude:.2f} m/s²',
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## GPS Sensor Simulation

```xml
<gazebo>
  <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
    <update_rate>10</update_rate>
    <topic>gps/fix</topic>
    <frame_name>base_link</frame_name>
    <drift>0.001 0.001 0.001</drift>
    <gaussian_noise>0.01 0.01 0.01</gaussian_noise>
  </plugin>
</gazebo>
```

## Multi-Sensor Fusion Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import numpy as np

class MultiSensorController(Node):
    def __init__(self):
        super().__init__('multi_sensor_controller')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # State variables
        self.obstacle_detected = False
        self.is_tilted = False

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        """Check for obstacles"""
        front_ranges = msg.ranges[-30:] + msg.ranges[:30]
        valid = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        self.obstacle_detected = min(valid) < 0.5 if valid else False

    def imu_callback(self, msg):
        """Check for excessive tilt"""
        from scipy.spatial.transform import Rotation as R
        rot = R.from_quat([msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w])
        roll, pitch, _ = rot.as_euler('xyz', degrees=True)
        self.is_tilted = abs(roll) > 20 or abs(pitch) > 20

    def control_loop(self):
        """Fused sensor control"""
        twist = Twist()

        if self.is_tilted:
            # Emergency stop if tilted
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('Robot tilted! Emergency stop.')
        elif self.obstacle_detected:
            # Avoid obstacle
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        else:
            # Move forward
            twist.linear.x = 0.3
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Takeaways

1. **LIDAR sensors** provide 2D/3D range measurements for obstacle detection and mapping
2. **Cameras** (RGB, depth, RGB-D) enable vision-based perception and object recognition
3. **IMU sensors** measure orientation, angular velocity, and linear acceleration
4. **Sensor noise models** in Gazebo simulate real-world imperfections
5. **Multi-sensor fusion** combines complementary sensors for robust perception

## Next Steps

In **Module 3 (Week 8)**, we'll transition to **NVIDIA Isaac Sim** for GPU-accelerated, photorealistic simulation with advanced rendering and synthetic data generation.

---

**References:**
- [Gazebo Sensor API](https://gazebosim.org/api/sensors/7/classgz_1_1sensors_1_1Sensor.html)
- [ROS 2 Sensor Messages](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [OpenCV Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
