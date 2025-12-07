---
title: Week 9 - Isaac ROS and Navigation
description: Implement Visual SLAM (VSLAM), Nav2 navigation, and real-time perception with Isaac ROS packages
sidebar_position: 2
hardware_tier: advanced
learning_objectives:
  - Deploy Isaac ROS packages for GPU-accelerated perception
  - Implement Visual SLAM with Isaac ROS VSLAM
  - Configure Nav2 navigation stack with Isaac ROS integration
  - Optimize perception pipelines for real-time performance
estimated_time_minutes: 180
---

# Week 9: Isaac ROS and Navigation

## Introduction to Isaac ROS

**Isaac ROS** is a collection of hardware-accelerated ROS 2 packages optimized for NVIDIA GPUs:
- **Isaac ROS VSLAM**: Visual SLAM using NVIDIA cuVSLAM
- **Isaac ROS Nvblox**: Real-time 3D reconstruction and mapping
- **Isaac ROS Image Pipeline**: GPU-accelerated image processing
- **Isaac ROS DNN Inference**: AI model deployment with TensorRT
- **Isaac ROS AprilTag**: Fiducial marker detection

### Performance Benefits

| Task | CPU (ROS 2) | Isaac ROS (GPU) | Speedup |
|------|-------------|-----------------|---------|
| Visual SLAM | 15 FPS | 60+ FPS | 4x |
| Semantic Segmentation | 5 FPS | 30 FPS | 6x |
| Depth Estimation | 10 FPS | 60 FPS | 6x |
| 3D Reconstruction | 2 FPS | 30 FPS | 15x |

## Installation and Setup

### Docker Container Approach (Recommended)

```bash
# Clone Isaac ROS Common (contains Docker utilities)
cd ~/ros2_isaac_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build Docker image
cd isaac_ros_common
./scripts/run_dev.sh

# Inside container, install Isaac ROS packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

### Native Installation (Advanced)

```bash
# Install CUDA and TensorRT
sudo apt install cuda-toolkit-11-8 tensorrt

# Install Isaac ROS packages
cd ~/ros2_isaac_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
cd ~/ros2_isaac_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Visual SLAM with Isaac ROS VSLAM

### VSLAM Architecture

**cuVSLAM** (CUDA Visual SLAM) uses:
- **ORB feature extraction** on GPU
- **Stereo or RGB-D cameras** for depth estimation
- **Loop closure detection** for drift correction
- **Real-time pose estimation** at 60+ Hz

### Launching Isaac ROS VSLAM

```bash
# Launch VSLAM with RealSense camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

# Launch VSLAM with Isaac Sim
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
```

### Custom VSLAM Launch File

```python
# launch/vslam_custom.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Visual SLAM node
    visual_slam_node = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam',
        parameters=[{
            'enable_rectified_pose': True,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'input_base_frame': 'camera_link',
            'input_left_camera_frame': 'camera_infra1_frame',
            'input_right_camera_frame': 'camera_infra2_frame',
            'min_num_images': 2,
            'override_publishing_stamp': False
        }],
        remappings=[
            ('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
            ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
            ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
            ('stereo_camera/right/camera_info', '/camera/infra2/camera_info')
        ]
    )

    # Container for composable nodes (reduces latency)
    visual_slam_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='visual_slam_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([visual_slam_container])
```

### Integrating VSLAM with Isaac Sim

```python
# Isaac Sim script to publish stereo camera data for VSLAM
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.graph.core as og

# Create world
world = World(stage_units_in_meters=1.0)

# Add robot
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)

# Create ROS 2 stereo camera publisher graph
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ROS2_VSLAM_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("LeftCamera", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("RightCamera", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("PublishLeftImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
            ("PublishRightImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
            ("PublishLeftInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),
            ("PublishRightInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishLeftImage.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishRightImage.inputs:execIn"),
            ("LeftCamera.outputs:imageData", "PublishLeftImage.inputs:data"),
            ("RightCamera.outputs:imageData", "PublishRightImage.inputs:data"),
        ],
        keys.SET_VALUES: [
            ("PublishLeftImage.inputs:topicName", "camera/infra1/image_rect_raw"),
            ("PublishRightImage.inputs:topicName", "camera/infra2/image_rect_raw"),
            ("PublishLeftInfo.inputs:topicName", "camera/infra1/camera_info"),
            ("PublishRightInfo.inputs:topicName", "camera/infra2/camera_info"),
        ]
    }
)

# Run simulation
world.reset()
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## Navigation with Nav2 and Isaac ROS

### Nav2 Stack Overview

**Nav2** (Navigation 2) provides:
- **Global planner**: Path planning (A*, Dijkstra, Smac)
- **Local planner**: Dynamic obstacle avoidance (DWB, TEB, MPPI)
- **Recovery behaviors**: Spin, backup, wait
- **Costmap layers**: Static map, inflation, obstacle layer

### Installing Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Nav2 Configuration with Isaac ROS

```yaml
# config/nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["obstacle_layer", "inflation_layer"]

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

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

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
```

### Nav2 Launch File with Isaac ROS VSLAM

```python
# launch/nav2_isaac.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    isaac_vslam_dir = get_package_share_directory('isaac_ros_visual_slam')

    # Isaac ROS VSLAM
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(isaac_vslam_dir, 'launch', 'isaac_ros_visual_slam.launch.py')
        )
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(
                get_package_share_directory('my_robot_nav'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )

    return LaunchDescription([
        vslam_launch,
        nav2_launch
    ])
```

### Sending Navigation Goals

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()

    # Send goal: x=2.0m, y=1.0m, theta=0
    client.send_goal(2.0, 1.0, 0.0)

    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3D Reconstruction with Isaac ROS Nvblox

**Nvblox** creates real-time 3D maps using TSDF (Truncated Signed Distance Fields):

```bash
# Launch Nvblox with RealSense
ros2 launch nvblox_examples_bringup realsense_example.launch.py

# Visualize in RViz
rviz2 -d $(ros2 pkg prefix nvblox_examples_bringup)/share/nvblox_examples_bringup/config/nvblox.rviz
```

## Key Takeaways

1. **Isaac ROS packages** leverage GPU acceleration for 10-100x performance improvements
2. **Isaac ROS VSLAM** provides real-time visual SLAM at 60+ Hz
3. **Nav2 integration** enables autonomous navigation with dynamic obstacle avoidance
4. **Composable nodes** reduce latency by sharing memory between ROS 2 nodes
5. **Nvblox** creates real-time 3D reconstructions for advanced path planning

## Next Steps

In **Week 10**, we'll explore **sim-to-real transfer** techniques to deploy trained models from Isaac Sim to physical robots with minimal performance degradation.

---

**References:**
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Isaac ROS VSLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
