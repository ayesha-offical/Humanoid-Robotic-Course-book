---
title: Week 12 - Humanoid Manipulation and Grasping
description: Master dexterous manipulation, grasping strategies, and multi-fingered hand control for humanoid robots
sidebar_position: 2
hardware_tier: advanced
learning_objectives:
  - Implement grasping strategies for multi-fingered robotic hands
  - Design whole-body manipulation controllers
  - Apply contact dynamics for stable object manipulation
  - Simulate dexterous manipulation tasks in Isaac Sim
estimated_time_minutes: 180
---

# Week 12: Humanoid Manipulation and Grasping

## Introduction to Humanoid Manipulation

Humanoid manipulation combines:
- **Dexterous hands**: 12-20 DOF multi-fingered hands
- **Arm kinematics**: 7 DOF arms for redundancy
- **Whole-body coordination**: Legs + torso + arms working together
- **Contact-rich interactions**: Stable grasping, pushing, pulling

### Manipulation Challenges

| Challenge | Description | Solutions |
|-----------|-------------|-----------|
| High dimensionality | 40+ DOF for full-body manipulation | Hierarchical control, motion primitives |
| Contact dynamics | Non-smooth, discontinuous physics | Compliant control, force feedback |
| Perception | Object pose estimation, occlusion | RGB-D cameras, tactile sensors |
| Generalization | Diverse objects and environments | Learning-based approaches, sim-to-real |

## Grasp Planning

### Grasp Types

1. **Power grasps**: Full hand enclosure (e.g., cylindrical, spherical)
2. **Precision grasps**: Fingertip control (e.g., pinch, tripod)
3. **Hook grasps**: Fingers hook around objects
4. **Platform grasps**: Object rests on palm

### Grasp Quality Metrics

```python
import numpy as np
from scipy.spatial import ConvexHull

class GraspAnalyzer:
    """Analyze grasp quality using force closure and wrench space"""

    def __init__(self, friction_coefficient=0.5):
        self.mu = friction_coefficient

    def force_closure(self, contact_points, contact_normals):
        """
        Check if grasp achieves force closure
        contact_points: Nx3 array of contact positions
        contact_normals: Nx3 array of contact normals (pointing inward)
        Returns: True if force closure is achieved
        """
        n_contacts = len(contact_points)

        # Build wrench matrix (force + torque at each contact)
        G = []
        for i in range(n_contacts):
            p = contact_points[i]
            n = contact_normals[i]

            # Friction cone: 4 basis vectors per contact
            tangent1 = self._get_perpendicular(n)
            tangent2 = np.cross(n, tangent1)

            for v in [n, self.mu*tangent1, self.mu*tangent2, -self.mu*tangent1, -self.mu*tangent2]:
                # Force component
                force = v
                # Torque component (r x f)
                torque = np.cross(p, v)
                # Stack into 6D wrench
                wrench = np.concatenate([force, torque])
                G.append(wrench)

        G = np.array(G).T  # 6 x (5*n_contacts)

        # Force closure if origin is in interior of convex hull of wrenches
        try:
            hull = ConvexHull(G.T)
            return self._point_in_hull(np.zeros(6), hull)
        except:
            return False

    def grasp_quality(self, contact_points, contact_normals):
        """
        Compute grasp quality metric (largest perturbation wrench resisted)
        Returns: minimum distance from origin to wrench space boundary
        """
        G = self._build_wrench_matrix(contact_points, contact_normals)

        try:
            hull = ConvexHull(G.T)
            # Minimum distance from origin to hull facets
            min_dist = np.min([np.abs(eq[-1]) / np.linalg.norm(eq[:-1])
                               for eq in hull.equations])
            return min_dist
        except:
            return 0.0

    @staticmethod
    def _get_perpendicular(v):
        """Get a vector perpendicular to v"""
        if abs(v[0]) > 0.1:
            return np.array([v[1], -v[0], 0])
        else:
            return np.array([0, v[2], -v[1]])

    @staticmethod
    def _point_in_hull(point, hull):
        """Check if point is inside convex hull"""
        return all(np.dot(eq[:-1], point) + eq[-1] <= 0 for eq in hull.equations)

    def _build_wrench_matrix(self, contact_points, contact_normals):
        """Build wrench matrix for grasp"""
        G = []
        for p, n in zip(contact_points, contact_normals):
            tangent1 = self._get_perpendicular(n)
            tangent2 = np.cross(n, tangent1)

            for v in [n, self.mu*tangent1, -self.mu*tangent1]:
                force = v
                torque = np.cross(p, v)
                wrench = np.concatenate([force, torque])
                G.append(wrench)

        return np.array(G).T

# Example: Evaluate a power grasp
analyzer = GraspAnalyzer(friction_coefficient=0.5)

# Three-finger pinch grasp on a sphere
contact_points = np.array([
    [0.05, 0, 0],
    [-0.025, 0.043, 0],
    [-0.025, -0.043, 0]
])

contact_normals = np.array([
    [-1, 0, 0],
    [0.5, -0.866, 0],
    [0.5, 0.866, 0]
])

is_force_closure = analyzer.force_closure(contact_points, contact_normals)
quality = analyzer.grasp_quality(contact_points, contact_normals)

print(f"Force closure: {is_force_closure}")
print(f"Grasp quality: {quality:.3f}")
```

## Dexterous Hand Control

### Shadow Hand Kinematics

```python
import numpy as np

class ShadowHandController:
    """Controller for Shadow Dexterous Hand (24 DOF)"""

    def __init__(self):
        # Joint limits (radians)
        self.joint_limits = {
            'thumb': {
                'THJ5': (-1.047, 1.047),    # Abduction
                'THJ4': (0, 1.222),          # Flexion MCP
                'THJ3': (-0.209, 0.209),     # Abduction
                'THJ2': (-0.524, 0.524),     # Flexion PIP
                'THJ1': (0, 1.571)           # Flexion DIP
            },
            'index': {
                'FFJ4': (-0.349, 0.349),     # Abduction
                'FFJ3': (0, 1.571),          # Flexion MCP
                'FFJ2': (0, 1.571),          # Flexion PIP
                'FFJ1': (0, 1.571)           # Flexion DIP
            },
            # Similar for middle, ring, little fingers
        }

    def grasp_pose(self, grasp_type='power'):
        """Generate joint angles for different grasp types"""
        if grasp_type == 'power':
            # Full finger curl
            return {
                'thumb': [0.5, 1.0, 0, 0.3, 1.2],
                'index': [0, 1.3, 1.3, 1.0],
                'middle': [0, 1.3, 1.3, 1.0],
                'ring': [0, 1.3, 1.3, 1.0],
                'little': [0, 1.3, 1.3, 1.0]
            }
        elif grasp_type == 'pinch':
            # Thumb-index pinch
            return {
                'thumb': [0.8, 0.5, 0, 0, 0.5],
                'index': [0, 0.5, 0.5, 0.3],
                'middle': [0, 0, 0, 0],
                'ring': [0, 0, 0, 0],
                'little': [0, 0, 0, 0]
            }
        elif grasp_type == 'tripod':
            # Three-finger precision
            return {
                'thumb': [0.6, 0.6, 0, 0, 0.6],
                'index': [0, 0.6, 0.6, 0.4],
                'middle': [0, 0.6, 0.6, 0.4],
                'ring': [0, 0, 0, 0],
                'little': [0, 0, 0, 0]
            }

    def interpolate_trajectory(self, start_pose, end_pose, duration=2.0, hz=100):
        """Generate smooth trajectory between poses"""
        n_steps = int(duration * hz)
        t = np.linspace(0, 1, n_steps)

        trajectory = []
        for finger in start_pose:
            start = np.array(start_pose[finger])
            end = np.array(end_pose[finger])

            # Cubic interpolation for smooth motion
            finger_traj = start[:, None] + (end[:, None] - start[:, None]) * (3*t**2 - 2*t**3)
            trajectory.append(finger_traj)

        return trajectory

# Example: Generate power grasp trajectory
controller = ShadowHandController()
open_pose = controller.grasp_pose('open')  # All fingers extended
power_pose = controller.grasp_pose('power')
trajectory = controller.interpolate_trajectory(open_pose, power_pose, duration=1.5)
```

## Whole-Body Manipulation

### Task-Space Control with Null-Space Projection

```python
import numpy as np
from scipy.linalg import pinv

class WholeBodyController:
    """Prioritized task-space controller for humanoid manipulation"""

    def __init__(self, robot):
        self.robot = robot
        self.n_joints = 40  # Example: full humanoid

    def compute_joint_velocities(self, tasks, joint_state):
        """
        Compute joint velocities using task prioritization
        tasks: List of (Jacobian, desired_velocity, priority) tuples
        """
        q_dot = np.zeros(self.n_joints)
        N = np.eye(self.n_joints)  # Null-space projector

        # Sort tasks by priority (lower number = higher priority)
        tasks = sorted(tasks, key=lambda x: x[2])

        for J, v_des, _ in tasks:
            # Compute task in null space of higher-priority tasks
            J_N = J @ N

            # Pseudo-inverse
            J_N_pinv = pinv(J_N, rcond=1e-4)

            # Task velocity
            q_dot_task = J_N_pinv @ (v_des - J @ q_dot)
            q_dot += q_dot_task

            # Update null-space projector
            N = N @ (np.eye(self.n_joints) - J_N_pinv @ J_N)

        return q_dot

    def manipulate_object(self, hand_position, hand_velocity, com_position):
        """
        Control hand position while maintaining balance
        Priority 1: Hand end-effector
        Priority 2: Center of mass (balance)
        Priority 3: Joint limits avoidance
        """
        # Task 1: Hand position (6 DOF)
        J_hand = self.robot.compute_hand_jacobian()
        v_hand = hand_velocity

        # Task 2: COM position (3 DOF)
        J_com = self.robot.compute_com_jacobian()
        v_com = np.array([0, 0, 0])  # Keep COM fixed

        # Task 3: Joint limits (null-space task)
        J_limits = np.eye(self.n_joints)
        q = self.robot.get_joint_positions()
        q_mid = (self.robot.q_min + self.robot.q_max) / 2
        v_limits = -0.1 * (q - q_mid)  # Push toward mid-range

        tasks = [
            (J_hand, v_hand, 1),
            (J_com, v_com, 2),
            (J_limits, v_limits, 3)
        ]

        q_dot = self.compute_joint_velocities(tasks, q)
        return q_dot

# Example usage
# controller = WholeBodyController(humanoid_robot)
# joint_velocities = controller.manipulate_object(
#     hand_position=np.array([0.5, 0.3, 1.0]),
#     hand_velocity=np.array([0.1, 0, 0, 0, 0, 0]),
#     com_position=np.array([0, 0, 0.8])
# )
```

## Manipulation in Isaac Sim

### Simulating Dexterous Grasping

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add humanoid with dexterous hand
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Robots/Manipulation/shadow_hand.usd",
    prim_path="/World/ShadowHand"
)

# Add object to grasp
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="target_object",
        position=np.array([0.3, 0, 0.5]),
        size=0.05,
        color=np.array([1, 0, 0])
    )
)

# Get hand controller
hand = world.scene.get_object("ShadowHand")

# Reset simulation
world.reset()

# Grasp sequence
grasp_sequence = [
    # Phase 1: Open hand
    {'duration': 1.0, 'closure': 0.0},
    # Phase 2: Move to pre-grasp
    {'duration': 2.0, 'closure': 0.0},
    # Phase 3: Close fingers
    {'duration': 1.5, 'closure': 1.0},
    # Phase 4: Lift object
    {'duration': 2.0, 'closure': 1.0}
]

phase = 0
phase_start_time = 0

while simulation_app.is_running():
    world.step(render=True)

    current_time = world.current_time

    if phase < len(grasp_sequence):
        phase_data = grasp_sequence[phase]

        # Compute closure amount (0=open, 1=closed)
        closure = phase_data['closure']

        # Set finger joint positions
        # (Actual implementation depends on robot URDF/USD structure)
        # hand.set_joint_positions(compute_finger_positions(closure))

        # Check phase transition
        if current_time - phase_start_time > phase_data['duration']:
            phase += 1
            phase_start_time = current_time

simulation_app.close()
```

### Contact Force Control

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np

class ContactForceController(Node):
    """Control manipulation with force feedback"""

    def __init__(self):
        super().__init__('contact_force_controller')

        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/hand/force_torque',
            self.force_callback,
            10
        )

        self.desired_force = 5.0  # Newtons
        self.kp = 0.01  # Position gain
        self.measured_force = 0.0

    def force_callback(self, msg):
        """Process force sensor data"""
        force = msg.wrench.force
        self.measured_force = np.linalg.norm([force.x, force.y, force.z])

    def compute_position_correction(self):
        """Adjust position based on force error"""
        force_error = self.desired_force - self.measured_force

        # Proportional control (simplified)
        position_delta = self.kp * force_error

        return position_delta

# Example: Maintain constant grasp force
def main(args=None):
    rclpy.init(args=args)
    controller = ContactForceController()

    while rclpy.ok():
        rclpy.spin_once(controller)

        # Adjust grasp closure based on force feedback
        delta = controller.compute_position_correction()
        # Apply delta to hand controller...

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Takeaways

1. **Grasp planning** requires analyzing force closure and wrench space for stability
2. **Dexterous hands** enable precision grasps with 12-20 DOF multi-fingered designs
3. **Whole-body manipulation** uses prioritized task-space control for coordinated motion
4. **Contact force control** enables compliant interaction with objects
5. **Isaac Sim** provides high-fidelity physics for simulating contact-rich manipulation

## Next Steps

In **Week 13**, we'll integrate **conversational AI** with humanoid systems using GPT models and Vision-Language-Action (VLA) models for embodied intelligence.

---

**References:**
- [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [Grasp Quality Metrics](https://journals.sagepub.com/doi/10.1177/02783649922066267)
- [Shadow Dexterous Hand](https://www.shadowrobot.com/dexterous-hand-series/)
