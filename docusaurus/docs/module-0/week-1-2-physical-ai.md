---
id: week-1-2-physical-ai
"title": Weeks 1-2: Foundations of Physical AI
"sidebar_label": Weeks 1-2: Foundations
"sidebar_position": 1
"description": Introduction to Embodied Intelligence and the sensor systems that power humanoid robots.
---

# 🏗️ Foundations of Physical AI & Embodied Intelligence

> **"The future of AI extends beyond digital spaces into the physical world."**

Welcome to the start of your journey into **Physical AI**. Unlike traditional AI that lives on servers (like ChatGPT), Physical AI must operate in the real world, obeying the laws of physics, dealing with gravity, friction, and unpredictable human environments.

---

## 🎯 Learning Objectives

By the end of this module, you will be able to:
1.  Define **Embodied Intelligence** and how it differs from Digital AI.
2.  Analyze the current **Humanoid Robotics Landscape**.
3.  Understand the core **Sensor Systems** (LIDAR, Cameras, IMUs) that give robots senses.

---

## 1. Digital Brains vs. Physical Bodies

### From Digital AI to Physical Laws
Traditional "Digital AI" (LLMs, Image Generators) processes information in a vacuum. If an LLM makes a mistake, it hallucinates text. If a Physical AI makes a mistake, **it crashes a robot or breaks an object**.

**Physical AI** refers to AI systems that:
* **Function in Reality:** They interact with physical objects.
* **Comprehend Physics:** They understand mass, velocity, and inertia.
* **Experience Consequences:** Their actions have real-world impacts.

### What is Embodied Intelligence?
Embodied intelligence is the concept that intelligence is not just a brain algorithm, but requires a body to interact with the environment. A humanoid robot learns by *doing*—walking, grasping, and sensing.

---

## 2. The Humanoid Robotics Landscape

Humanoid robots are designed to excel in our human-centered world because they share our form factor.

| Category | Description | Examples |
| :--- | :--- | :--- |
| **Bipedal Dynamics** | Robots that actively balance and walk like humans. | Boston Dynamics Atlas, Unitree G1 |
| **Social Humanoids** | Designed for interaction, utilizing gestures and speech. | Ameca, Pepper |
| **General Purpose** | Robots built to perform diverse manual labor tasks. | Tesla Optimus, Figure 01 |

:::info Industry Insight
The goal of this course is to bridge the gap between high-level AI reasoning and low-level motor control, enabling robots to perform complex tasks autonomously.
:::

---

## 3. Sensor Systems: The Robot's Senses

Just as humans need eyes and ears, robots need sensors to perceive the world.

### 👁️ Cameras (Vision)
The primary sensor for **Visual SLAM** (Simultaneous Localization and Mapping) and object recognition.
* **RGB Cameras:** Provide color data for object identification (like a webcam).
* **Depth Cameras (RGB-D):** Measure distance to every pixel. Crucial for understanding 3D space.
    * *Hardware Example:* **Intel RealSense D435i** (Included in Course Kit).

### 📡 LIDAR (Light Detection and Ranging)
Uses laser pulses to create precise 3D maps of the environment.
* **Role:** Essential for navigation and obstacle avoidance.
* **Advantage:** Works in low light and provides extremely accurate distance measurements.

### ⚖️ IMU (Inertial Measurement Unit)
The "Inner Ear" of the robot. It measures balance and orientation.
* **Components:** Accelerometer (speed) + Gyroscope (rotation).
* **Function:** Tells the robot which way is "down" and how fast it is falling or turning.
    * *Hardware Example:* **BNO055** or built-in RealSense IMU.

### 💪 Force/Torque Sensors
These sensors give the robot a sense of "touch" and "effort."
* **Location:** Often in the wrists or ankles.
* **Use Case:** Allows a robot to hold an egg without crushing it, or walk on uneven ground by feeling the resistance.

---

## 🛠️ Practical Exercise: Sensor Data Visualization

In the upcoming weeks, we will visualize data from these sensors using **ROS 2**. Here is a preview of how we define a simple camera sensor in a robot description file (URDF).

```xml title="robot_camera.urdf"
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="black"/>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>