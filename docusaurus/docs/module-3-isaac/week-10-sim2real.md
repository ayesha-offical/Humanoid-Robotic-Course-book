---
title: Week 10 - Sim-to-Real Transfer Techniques
description: Bridge the simulation-reality gap with domain randomization, system identification, and deployment strategies
sidebar_position: 3
hardware_tier: advanced
learning_objectives:
  - Understand the sim-to-real gap and mitigation strategies
  - Implement domain randomization for robust policy learning
  - Apply system identification to align simulation with reality
  - Deploy trained models from Isaac Sim to physical robots
estimated_time_minutes: 180
---

# Week 10: Sim-to-Real Transfer Techniques

## The Sim-to-Real Gap

The **sim-to-real gap** refers to performance degradation when transferring policies from simulation to physical robots, caused by:

### Sources of the Gap

1. **Modeling errors**: Simplified physics, friction, actuator dynamics
2. **Sensor noise**: Perfect sim sensors vs noisy real sensors
3. **Latency**: Zero sim latency vs 10-50ms real latency
4. **Unmodeled dynamics**: Cable drag, wear, temperature effects
5. **Visual appearance**: Synthetic textures vs real-world lighting

### Mitigation Strategies

| Strategy | Description | Difficulty | Effectiveness |
|----------|-------------|------------|---------------|
| Domain Randomization | Randomize sim parameters | Medium | High |
| System Identification | Measure real system, tune sim | High | Very High |
| Adversarial Training | Add worst-case perturbations | Medium | High |
| Sim-to-Real RL | Train partially in sim, fine-tune in real | Very High | Excellent |
| Reality Gap Reward | Penalize unrealistic behaviors | Medium | Medium |

## Domain Randomization in Isaac Sim

**Domain randomization** exposes the policy to diverse conditions during training, improving generalization.

### Randomization Categories

1. **Visual randomization**: Lighting, textures, colors, camera parameters
2. **Dynamics randomization**: Mass, friction, damping, joint properties
3. **Sensor randomization**: Noise, dropout, calibration errors
4. **Task randomization**: Object poses, sizes, goals

### Implementing Visual Randomization

```python
# Domain randomization script for Isaac Sim
import omni.replicator.core as rep
import numpy as np

def create_visual_randomization():
    """Randomize lighting, materials, and camera parameters"""

    # 1. Randomize lighting
    lights = rep.get.light()
    with lights:
        rep.modify.attribute(
            "intensity",
            rep.distribution.uniform(500, 5000)
        )
        rep.modify.attribute(
            "color",
            rep.distribution.uniform((0.7, 0.7, 0.7), (1.0, 1.0, 1.0))
        )
        rep.modify.attribute(
            "temperature",
            rep.distribution.uniform(3000, 6500)
        )

    # 2. Randomize materials
    objects = rep.get.prims(semantics=[("class", "object")])
    with objects:
        rep.randomizer.materials(
            materials=rep.get.material(
                semantics=[("class", "material")]
            )
        )
        rep.modify.attribute(
            "primvars:displayColor",
            rep.distribution.uniform((0, 0, 0), (1, 1, 1))
        )

    # 3. Randomize camera parameters
    camera = rep.get.camera()
    with camera:
        rep.modify.attribute(
            "focalLength",
            rep.distribution.uniform(18, 35)  # mm
        )
        rep.modify.attribute(
            "fStop",
            rep.distribution.uniform(1.8, 5.6)
        )

    # 4. Randomize background
    dome_light = rep.get.light(semantics=[("class", "dome_light")])
    with dome_light:
        rep.modify.attribute(
            "texture:file",
            rep.distribution.choice([
                "omniverse://localhost/NVIDIA/Assets/Skies/Indoor/Studio_1.hdr",
                "omniverse://localhost/NVIDIA/Assets/Skies/Outdoor/Cloudy.hdr",
                "omniverse://localhost/NVIDIA/Assets/Skies/Indoor/Workshop.hdr"
            ])
        )

# Register randomization
rep.randomizer.register(create_visual_randomization)

# Trigger every frame
with rep.trigger.on_frame():
    rep.randomizer.create_visual_randomization()
```

### Physics Randomization

```python
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.prims import RigidPrimView
from pxr import UsdPhysics
import numpy as np

class PhysicsRandomizer:
    def __init__(self, robot_prim_path):
        self.robot_prim = prims_utils.get_prim_at_path(robot_prim_path)
        self.links = self._get_all_links()

    def _get_all_links(self):
        """Get all rigid body links in the robot"""
        links = []
        for prim in self.robot_prim.GetChildren():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                links.append(prim)
        return links

    def randomize_dynamics(self):
        """Randomize mass, friction, and damping"""
        for link in self.links:
            # Randomize mass (±20%)
            mass_api = UsdPhysics.MassAPI(link)
            original_mass = mass_api.GetMassAttr().Get()
            randomized_mass = original_mass * np.random.uniform(0.8, 1.2)
            mass_api.GetMassAttr().Set(randomized_mass)

            # Randomize friction
            material_api = UsdPhysics.MaterialAPI(link)
            material_api.CreateStaticFrictionAttr().Set(
                np.random.uniform(0.3, 1.0)
            )
            material_api.CreateDynamicFrictionAttr().Set(
                np.random.uniform(0.2, 0.8)
            )

    def randomize_actuators(self):
        """Randomize joint properties"""
        for link in self.links:
            joint = link.GetParent()
            if joint.HasAPI(UsdPhysics.DriveAPI):
                drive_api = UsdPhysics.DriveAPI(joint, "angular")

                # Randomize stiffness (±30%)
                stiffness = drive_api.GetStiffnessAttr().Get()
                drive_api.GetStiffnessAttr().Set(
                    stiffness * np.random.uniform(0.7, 1.3)
                )

                # Randomize damping (±30%)
                damping = drive_api.GetDampingAttr().Get()
                drive_api.GetDampingAttr().Set(
                    damping * np.random.uniform(0.7, 1.3)
                )

# Usage in simulation loop
randomizer = PhysicsRandomizer("/World/Robot")
for episode in range(num_episodes):
    randomizer.randomize_dynamics()
    randomizer.randomize_actuators()
    # Run episode...
```

### Sensor Noise Injection

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class SensorNoiseInjector(Node):
    def __init__(self):
        super().__init__('sensor_noise_injector')

        # Subscribe to clean sim sensors
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_clean', self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu_clean', self.imu_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/image_clean', self.image_callback, 10
        )

        # Publish noisy sensors
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

    def scan_callback(self, msg):
        """Add noise to LIDAR"""
        noisy_ranges = []
        for r in msg.ranges:
            if np.isfinite(r):
                # Add Gaussian noise (1cm std dev)
                noisy_r = r + np.random.normal(0, 0.01)
                # Add occasional dropouts (2% probability)
                if np.random.random() < 0.02:
                    noisy_r = float('inf')
                noisy_ranges.append(max(msg.range_min, min(msg.range_max, noisy_r)))
            else:
                noisy_ranges.append(r)

        msg.ranges = noisy_ranges
        self.scan_pub.publish(msg)

    def imu_callback(self, msg):
        """Add noise to IMU"""
        # Accelerometer noise (0.02 m/s² std dev)
        msg.linear_acceleration.x += np.random.normal(0, 0.02)
        msg.linear_acceleration.y += np.random.normal(0, 0.02)
        msg.linear_acceleration.z += np.random.normal(0, 0.02)

        # Gyroscope noise (0.01 rad/s std dev)
        msg.angular_velocity.x += np.random.normal(0, 0.01)
        msg.angular_velocity.y += np.random.normal(0, 0.01)
        msg.angular_velocity.z += np.random.normal(0, 0.01)

        self.imu_pub.publish(msg)

    def image_callback(self, msg):
        """Add noise to camera"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Add Gaussian noise
        noise = np.random.normal(0, 5, cv_image.shape).astype(np.uint8)
        noisy_image = cv2.add(cv_image, noise)

        # Add motion blur (simulate real camera)
        if np.random.random() < 0.1:  # 10% of frames
            kernel_size = np.random.randint(3, 8)
            kernel = np.zeros((kernel_size, kernel_size))
            kernel[int((kernel_size-1)/2), :] = np.ones(kernel_size)
            kernel /= kernel_size
            noisy_image = cv2.filter2D(noisy_image, -1, kernel)

        noisy_msg = self.bridge.cv2_to_imgmsg(noisy_image, encoding='bgr8')
        noisy_msg.header = msg.header
        self.image_pub.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNoiseInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Identification

**System identification** measures real robot parameters to improve simulation accuracy.

### Measuring Real Robot Dynamics

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

class SystemIdentifier(Node):
    def __init__(self):
        super().__init__('system_identifier')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.velocities = []
        self.times = []
        self.start_time = self.get_clock().now()

    def odom_callback(self, msg):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        v_linear = msg.twist.twist.linear.x
        v_angular = msg.twist.twist.angular.z
        self.velocities.append((v_linear, v_angular))
        self.times.append(t)

    def run_step_response_test(self):
        """Apply step input and measure response"""
        # Send zero velocity
        twist = Twist()
        for _ in range(20):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Send step input (0.5 m/s linear)
        self.start_time = self.get_clock().now()
        self.velocities = []
        self.times = []

        twist.linear.x = 0.5
        for _ in range(100):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Analyze response
        self.analyze_step_response()

    def analyze_step_response(self):
        """Fit first-order system model"""
        times = np.array(self.times)
        velocities = np.array([v[0] for v in self.velocities])

        # Fit: v(t) = v_ss * (1 - exp(-t/tau))
        v_ss = velocities[-1]  # Steady-state velocity
        tau_est = times[np.argmax(velocities >= 0.632 * v_ss)]  # Time constant

        self.get_logger().info(f'Steady-state velocity: {v_ss:.3f} m/s')
        self.get_logger().info(f'Time constant: {tau_est:.3f} s')

        # Plot
        plt.plot(times, velocities, label='Measured')
        plt.plot(times, v_ss * (1 - np.exp(-times/tau_est)), '--', label='Fitted model')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid()
        plt.savefig('step_response.png')
        self.get_logger().info('Saved step response plot to step_response.png')

def main(args=None):
    rclpy.init(args=args)
    node = SystemIdentifier()
    node.run_step_response_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Updating Simulation Parameters

After system identification, update Isaac Sim parameters:

```python
from pxr import UsdPhysics

def update_simulation_parameters(robot_prim_path, identified_params):
    """Update sim parameters based on system ID"""
    robot_prim = prims_utils.get_prim_at_path(robot_prim_path)

    # Update joint damping based on measured time constant
    for joint in robot_prim.GetChildren():
        if joint.HasAPI(UsdPhysics.DriveAPI):
            drive_api = UsdPhysics.DriveAPI(joint, "angular")
            # tau = I / b, where I is inertia, b is damping
            measured_tau = identified_params['time_constant']
            inertia = identified_params['inertia']
            damping = inertia / measured_tau
            drive_api.GetDampingAttr().Set(damping)

    # Update friction based on steady-state error
    for link in robot_prim.GetChildren():
        if link.HasAPI(UsdPhysics.RigidBodyAPI):
            material_api = UsdPhysics.MaterialAPI(link)
            material_api.CreateDynamicFrictionAttr().Set(
                identified_params['friction_coefficient']
            )
```

## Deployment Pipeline

### 1. Train in Isaac Sim with Domain Randomization

```python
# Train RL policy with Isaac Sim Gym
from omni.isaac.gym.vec_env import VecEnvBase
import torch

class NavigationEnv(VecEnvBase):
    def __init__(self, config):
        super().__init__(config)
        # Setup environment with domain randomization

    def step(self, actions):
        # Apply domain randomization each step
        self.randomize_scene()
        # Execute actions, return obs, rewards, dones

# Train with PPO
from stable_baselines3 import PPO

env = NavigationEnv(config)
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1000000)
model.save("navigation_policy")
```

### 2. Convert Model for Deployment

```python
# Convert PyTorch model to ONNX for TensorRT
import torch.onnx

model = torch.load("navigation_policy.pth")
dummy_input = torch.randn(1, observation_size)

torch.onnx.export(
    model,
    dummy_input,
    "navigation_policy.onnx",
    export_params=True,
    opset_version=11,
    do_constant_folding=True,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
)
```

### 3. Deploy with Isaac ROS DNN Inference

```python
# ROS 2 node for model inference
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tensorrt as trt
import pycuda.driver as cuda
import numpy as np

class PolicyDeploymentNode(Node):
    def __init__(self):
        super().__init__('policy_deployment')

        # Load TensorRT engine
        self.engine = self.load_engine("navigation_policy.trt")
        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()

        # ROS 2 interface
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def load_engine(self, engine_path):
        with open(engine_path, 'rb') as f, trt.Runtime(trt.Logger(trt.Logger.WARNING)) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def scan_callback(self, msg):
        # Preprocess observation
        obs = np.array(msg.ranges, dtype=np.float32)

        # Run inference
        action = self.infer(obs)

        # Publish action
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self.cmd_vel_pub.publish(twist)

    def infer(self, observation):
        # Copy input to device
        np.copyto(self.inputs[0].host, observation.ravel())
        [cuda.memcpy_htod_async(inp.device, inp.host, self.stream) for inp in self.inputs]

        # Run inference
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)

        # Copy output to host
        [cuda.memcpy_dtoh_async(out.host, out.device, self.stream) for out in self.outputs]
        self.stream.synchronize()

        return self.outputs[0].host

def main(args=None):
    rclpy.init(args=args)
    node = PolicyDeploymentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Sim-to-Real

1. **Start conservative**: Test simple behaviors before complex policies
2. **Use safety limits**: Velocity/acceleration caps during initial tests
3. **Iterative refinement**: Sim → Real → Update Sim → Repeat
4. **Monitor sim vs real**: Log both for offline analysis
5. **Hardware-in-the-loop**: Run real perception + sim control before full deployment

## Key Takeaways

1. **Domain randomization** improves policy robustness by exposing it to diverse conditions
2. **System identification** aligns simulation with real robot dynamics
3. **Sensor noise injection** bridges the gap between perfect sim and noisy real sensors
4. **Deployment pipeline** involves training, model conversion, and TensorRT optimization
5. **Iterative refinement** is essential for closing the sim-to-real gap

## Next Steps

In **Module 4 (Week 11)**, we'll begin humanoid robotics with bipedal locomotion, kinematics, and balance control for full-body robotic systems.

---

**References:**
- [Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey](https://arxiv.org/abs/2009.13303)
- [NVIDIA Isaac Sim Domain Randomization](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_domain_randomization.html)
