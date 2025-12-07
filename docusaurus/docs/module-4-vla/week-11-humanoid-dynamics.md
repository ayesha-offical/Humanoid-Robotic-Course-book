---
title: Week 11 - Humanoid Kinematics and Bipedal Locomotion
description: Master humanoid robot kinematics, bipedal locomotion patterns, and balance control for full-body robotic systems
sidebar_position: 1
hardware_tier: advanced
learning_objectives:
  - Understand humanoid kinematics and forward/inverse kinematics
  - Implement bipedal locomotion using Zero Moment Point (ZMP) control
  - Design balance controllers for dynamic stability
  - Simulate humanoid robots in Isaac Sim with full-body dynamics
estimated_time_minutes: 180
---

# Week 11: Humanoid Kinematics and Bipedal Locomotion

## Introduction to Humanoid Robotics

**Humanoid robots** are anthropomorphic systems designed to mimic human morphology and capabilities:

### Key Challenges
- **High degrees of freedom (DOF)**: 20-50+ joints (vs 2-6 for mobile robots)
- **Dynamic balance**: Maintaining stability during locomotion
- **Complex kinematics**: Redundant solutions for end-effector positioning
- **Energy efficiency**: Mimicking human walking efficiency
- **Real-time control**: 100-1000 Hz control loops for stability

### Representative Humanoid Platforms

| Robot | Manufacturer | Height | DOF | Notable Features |
|-------|--------------|--------|-----|------------------|
| Atlas | Boston Dynamics | 1.5m | 28 | Parkour, backflips |
| Digit | Agility Robotics | 1.6m | 20 | Warehouse automation |
| Optimus | Tesla | 1.73m | ~40 | General-purpose tasks |
| Figure 01 | Figure AI | 1.7m | ~40 | Commercial humanoid |
| Unitree H1 | Unitree | 1.8m | 23 | High-speed locomotion |

## Humanoid Kinematics

### Forward Kinematics

**Forward kinematics** computes end-effector positions from joint angles using the **Denavit-Hartenberg (DH) convention**:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class HumanoidLeg:
    """7-DOF humanoid leg kinematics"""

    def __init__(self):
        # Link lengths (meters)
        self.hip_width = 0.1
        self.thigh_length = 0.4
        self.shank_length = 0.4
        self.foot_length = 0.1

    def forward_kinematics(self, joint_angles):
        """
        Compute foot position from joint angles
        joint_angles: [hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]
        Returns: 4x4 transformation matrix
        """
        q = joint_angles

        # Hip yaw
        T_hip_yaw = self.dh_transform(0, 0, 0, q[0])

        # Hip roll
        T_hip_roll = self.dh_transform(0, -np.pi/2, 0, q[1])

        # Hip pitch
        T_hip_pitch = self.dh_transform(0, np.pi/2, self.hip_width, q[2])

        # Knee pitch
        T_knee = self.dh_transform(self.thigh_length, 0, 0, q[3])

        # Ankle pitch
        T_ankle_pitch = self.dh_transform(self.shank_length, 0, 0, q[4])

        # Ankle roll
        T_ankle_roll = self.dh_transform(0, -np.pi/2, 0, q[5])

        # Foot
        T_foot = self.dh_transform(self.foot_length, 0, 0, 0)

        # Chain transformations
        T_total = T_hip_yaw @ T_hip_roll @ T_hip_pitch @ T_knee @ T_ankle_pitch @ T_ankle_roll @ T_foot

        return T_total

    @staticmethod
    def dh_transform(a, alpha, d, theta):
        """Denavit-Hartenberg transformation matrix"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

# Example usage
leg = HumanoidLeg()
joint_angles = np.array([0, 0, -np.pi/4, np.pi/2, -np.pi/4, 0])  # Standing pose
T_foot = leg.forward_kinematics(joint_angles)
print(f"Foot position: {T_foot[:3, 3]}")
```

### Inverse Kinematics

**Inverse kinematics (IK)** computes joint angles for a desired end-effector pose:

```python
from scipy.optimize import minimize

class HumanoidIK:
    def __init__(self, leg):
        self.leg = leg

    def inverse_kinematics(self, target_position, target_orientation=None):
        """
        Compute joint angles for desired foot pose
        target_position: [x, y, z]
        target_orientation: (optional) rotation matrix or None
        """
        def cost_function(q):
            T = self.leg.forward_kinematics(q)
            current_pos = T[:3, 3]

            # Position error
            pos_error = np.linalg.norm(current_pos - target_position)

            # Orientation error (if specified)
            if target_orientation is not None:
                current_rot = T[:3, :3]
                rot_error = np.linalg.norm(current_rot - target_orientation)
            else:
                rot_error = 0

            # Joint limits penalty
            q_min = np.array([-0.5, -0.3, -1.5, 0, -0.8, -0.3])
            q_max = np.array([0.5, 0.3, 0.8, 2.5, 0.8, 0.3])
            limit_penalty = np.sum(np.maximum(0, q_min - q) + np.maximum(0, q - q_max))

            return pos_error + 0.1 * rot_error + 10 * limit_penalty

        # Initial guess (standing pose)
        q0 = np.array([0, 0, -0.3, 0.6, -0.3, 0])

        # Optimize
        result = minimize(cost_function, q0, method='SLSQP',
                          options={'maxiter': 1000, 'ftol': 1e-6})

        return result.x if result.success else None

# Example usage
ik = HumanoidIK(leg)
target_pos = np.array([0.0, 0.1, -0.7])  # 70cm below hip
joint_solution = ik.inverse_kinematics(target_pos)
print(f"Joint angles: {np.degrees(joint_solution)}")
```

## Bipedal Locomotion

### Zero Moment Point (ZMP) Control

The **Zero Moment Point** is the point on the ground where the resultant of gravity and inertial forces equals zero. For stable walking, ZMP must stay within the support polygon.

```python
import numpy as np
import matplotlib.pyplot as plt

class ZMPWalkingPattern:
    """Generate walking patterns using ZMP control"""

    def __init__(self, step_length=0.2, step_height=0.05, step_duration=0.8):
        self.step_length = step_length
        self.step_height = step_height
        self.step_duration = step_duration
        self.com_height = 0.8  # Center of mass height

    def generate_trajectory(self, num_steps=4):
        """Generate foot and COM trajectories for walking"""
        dt = 0.01  # 100 Hz control
        total_time = num_steps * self.step_duration
        t = np.arange(0, total_time, dt)

        # Initialize trajectories
        left_foot = np.zeros((len(t), 3))
        right_foot = np.zeros((len(t), 3))
        com = np.zeros((len(t), 3))
        zmp = np.zeros((len(t), 2))

        # Initial positions
        left_foot[:, :] = [-0.1, 0.1, 0]   # Left hip offset
        right_foot[:, :] = [-0.1, -0.1, 0]  # Right hip offset
        com[:, 2] = self.com_height

        for step in range(num_steps):
            t_start = step * self.step_duration
            t_end = (step + 1) * self.step_duration
            idx_start = int(t_start / dt)
            idx_end = int(t_end / dt)

            # Determine swing leg
            is_left_swing = (step % 2 == 0)

            # Generate swing trajectory (cubic polynomial)
            t_step = np.linspace(0, 1, idx_end - idx_start)

            if is_left_swing:
                # Left leg swings
                left_foot[idx_start:idx_end, 0] = self.cubic_trajectory(
                    left_foot[idx_start, 0],
                    left_foot[idx_start, 0] + self.step_length,
                    t_step
                )
                left_foot[idx_start:idx_end, 2] = self.swing_height_trajectory(t_step)

                # ZMP stays under right foot
                zmp[idx_start:idx_end, 1] = -0.1
            else:
                # Right leg swings
                right_foot[idx_start:idx_end, 0] = self.cubic_trajectory(
                    right_foot[idx_start, 0],
                    right_foot[idx_start, 0] + self.step_length,
                    t_step
                )
                right_foot[idx_start:idx_end, 2] = self.swing_height_trajectory(t_step)

                # ZMP stays under left foot
                zmp[idx_start:idx_end, 1] = 0.1

            # COM follows ZMP with preview control
            zmp[idx_start:idx_end, 0] = 0.5 * (left_foot[idx_start:idx_end, 0] +
                                               right_foot[idx_start:idx_end, 0])
            com[idx_start:idx_end, 0] = zmp[idx_start:idx_end, 0]
            com[idx_start:idx_end, 1] = zmp[idx_start:idx_end, 1]

        return t, left_foot, right_foot, com, zmp

    @staticmethod
    def cubic_trajectory(start, end, t):
        """Cubic polynomial trajectory"""
        return start + (end - start) * (3*t**2 - 2*t**3)

    def swing_height_trajectory(self, t):
        """Swing leg height (parabolic)"""
        return self.step_height * 4 * t * (1 - t)

    def visualize(self, t, left_foot, right_foot, com, zmp):
        """Visualize walking pattern"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        # Foot trajectories (top view)
        axes[0, 0].plot(left_foot[:, 0], left_foot[:, 1], 'b-', label='Left foot')
        axes[0, 0].plot(right_foot[:, 0], right_foot[:, 1], 'r-', label='Right foot')
        axes[0, 0].plot(com[:, 0], com[:, 1], 'g--', label='COM')
        axes[0, 0].plot(zmp[:, 0], zmp[:, 1], 'k.', label='ZMP', markersize=1)
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        axes[0, 0].set_title('Top View')
        axes[0, 0].legend()
        axes[0, 0].grid()

        # Foot height over time
        axes[0, 1].plot(t, left_foot[:, 2], 'b-', label='Left foot')
        axes[0, 1].plot(t, right_foot[:, 2], 'r-', label='Right foot')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Height (m)')
        axes[0, 1].set_title('Foot Height')
        axes[0, 1].legend()
        axes[0, 1].grid()

        # COM trajectory
        axes[1, 0].plot(t, com[:, 0], 'g-', label='COM X')
        axes[1, 0].plot(t, zmp[:, 0], 'k--', label='ZMP X')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('X Position (m)')
        axes[1, 0].set_title('COM vs ZMP (Sagittal)')
        axes[1, 0].legend()
        axes[1, 0].grid()

        axes[1, 1].plot(t, com[:, 1], 'g-', label='COM Y')
        axes[1, 1].plot(t, zmp[:, 1], 'k--', label='ZMP Y')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Y Position (m)')
        axes[1, 1].set_title('COM vs ZMP (Lateral)')
        axes[1, 1].legend()
        axes[1, 1].grid()

        plt.tight_layout()
        plt.savefig('walking_pattern.png')
        print("Saved walking pattern to walking_pattern.png")

# Generate and visualize
walker = ZMPWalkingPattern(step_length=0.3, step_height=0.08, step_duration=1.0)
t, left, right, com, zmp = walker.generate_trajectory(num_steps=4)
walker.visualize(t, left, right, com, zmp)
```

## Balance Control

### Linear Inverted Pendulum Model (LIPM)

```python
import numpy as np
from scipy.integrate import odeint

class LinearInvertedPendulum:
    """Simplified model for humanoid balance control"""

    def __init__(self, com_height=0.8, gravity=9.81):
        self.h = com_height
        self.g = gravity
        self.omega = np.sqrt(gravity / com_height)  # Natural frequency

    def dynamics(self, state, t, zmp_ref):
        """
        State: [x, x_dot] (COM position and velocity)
        zmp_ref: desired ZMP position
        """
        x, x_dot = state
        x_ddot = self.omega**2 * (x - zmp_ref)
        return [x_dot, x_ddot]

    def simulate(self, x0, v0, zmp_ref, t_span):
        """Simulate COM dynamics"""
        state0 = [x0, v0]
        t = np.linspace(0, t_span, 1000)
        solution = odeint(self.dynamics, state0, t, args=(zmp_ref,))
        return t, solution[:, 0], solution[:, 1]

# Example: Balance recovery
lipm = LinearInvertedPendulum()

# Push robot forward (initial velocity)
t, x, x_dot = lipm.simulate(x0=0.0, v0=0.5, zmp_ref=0.1, t_span=2.0)

plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(t, x, label='COM position')
plt.axhline(0.1, color='r', linestyle='--', label='ZMP reference')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()

plt.subplot(1, 2, 2)
plt.plot(t, x_dot, label='COM velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.savefig('balance_recovery.png')
```

## Humanoid Simulation in Isaac Sim

### Loading a Humanoid Robot

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World(stage_units_in_meters=1.0)

# Add humanoid robot (example: Unitree H1)
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Robots/Unitree/H1/h1.usd",
    prim_path="/World/Humanoid"
)

# Get robot articulation
humanoid = world.scene.add(Robot(prim_path="/World/Humanoid", name="h1_robot"))

# Reset to standing pose
world.reset()

# Control loop
while simulation_app.is_running():
    world.step(render=True)

    # Apply simple balance control
    # (In practice, use ZMP controller or whole-body control)

simulation_app.close()
```

## Key Takeaways

1. **Forward kinematics** computes end-effector positions from joint angles using DH parameters
2. **Inverse kinematics** solves for joint angles to achieve desired poses (optimization-based)
3. **Zero Moment Point (ZMP)** must remain within support polygon for dynamic stability
4. **Linear Inverted Pendulum Model** simplifies humanoid balance control
5. **Bipedal locomotion** requires coordinated swing/stance phases with COM trajectory planning

## Next Steps

In **Week 12**, we'll explore **humanoid manipulation** including grasping, dexterous hand control, and whole-body manipulation strategies.

---

**References:**
- [Introduction to Humanoid Robotics](https://link.springer.com/book/10.1007/978-3-642-54536-8)
- [ZMP-based Walking Control](https://www.sciencedirect.com/science/article/pii/S0921889004001648)
- [Kajita et al. - Biped Walking Pattern Generation](https://ieeexplore.ieee.org/document/1241826)
