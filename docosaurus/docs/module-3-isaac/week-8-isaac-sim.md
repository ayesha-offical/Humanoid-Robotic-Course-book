---
title: Week 8 - NVIDIA Isaac Sim Introduction
description: Master GPU-accelerated photorealistic simulation with Isaac Sim for synthetic data generation and advanced robotics
sidebar_position: 1
hardware_tier: advanced
learning_objectives:
  - Set up NVIDIA Isaac Sim with RTX rendering for photorealistic simulation
  - Create robots and environments using USD (Universal Scene Description)
  - Generate synthetic datasets with domain randomization
  - Integrate Isaac Sim with ROS 2 for seamless workflow
estimated_time_minutes: 180
---

# Week 8: NVIDIA Isaac Sim Introduction

## Introduction to Isaac Sim

**NVIDIA Isaac Sim** is a GPU-accelerated robotics simulation platform built on NVIDIA Omniverse, offering:
- **Photorealistic rendering**: RTX ray tracing and path tracing
- **Physics accuracy**: PhysX 5 for high-fidelity dynamics
- **Synthetic data generation**: Pixel-perfect annotations for ML training
- **Scalability**: Multi-GPU support for large-scale sim-to-real workflows
- **ROS 2 integration**: Native Isaac ROS packages

### Isaac Sim vs Gazebo

| Feature | Gazebo Classic/Sim | Isaac Sim |
|---------|-------------------|-----------|
| Rendering | OGRE (basic) | RTX ray tracing (photorealistic) |
| Physics | ODE/Bullet/DART | PhysX 5 (GPU-accelerated) |
| Synthetic data | Limited | Full semantic/instance segmentation |
| Performance | CPU-bound | GPU-accelerated (10-100x faster) |
| ML integration | Manual export | Built-in dataset generation |
| Format | URDF/SDF | USD (Universal Scene Description) |

## System Requirements

### Minimum (for lightweight experiments)
- **GPU**: NVIDIA RTX 3060 (12GB VRAM)
- **CPU**: Intel Core i7 or AMD Ryzen 7
- **RAM**: 16GB
- **Storage**: 50GB SSD

### Recommended (for full features)
- **GPU**: NVIDIA RTX 4080 or A6000 (24GB+ VRAM)
- **CPU**: Intel Core i9 or AMD Ryzen 9
- **RAM**: 32GB+
- **Storage**: 100GB NVMe SSD

## Installation

### Step 1: Install Isaac Sim

```bash
# Download Omniverse Launcher from NVIDIA
# https://www.nvidia.com/en-us/omniverse/download/

# Launch Omniverse and install:
# 1. Nucleus (asset management)
# 2. Isaac Sim 2023.1.1 or later

# Verify installation
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh --version
```

### Step 2: Install ROS 2 Workspace for Isaac

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_isaac_ws/src
cd ~/ros2_isaac_ws/src

# Clone Isaac ROS packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# Install dependencies
cd ~/ros2_isaac_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### Step 3: Configure Isaac Sim with ROS 2

```bash
# Set environment variables
export ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1"
export ROS_DISTRO=humble

# Enable ROS 2 bridge in Isaac Sim
cd $ISAAC_SIM_PATH
./python.sh -m pip install --upgrade pip
./python.sh -m pip install rospy rclpy
```

## Universal Scene Description (USD)

Isaac Sim uses **USD** (Pixar's Universal Scene Description) for scene composition:

### Basic USD Structure

```python
# Create a simple USD scene programmatically
from pxr import Usd, UsdGeom, Gf

# Create stage
stage = Usd.Stage.CreateNew("simple_scene.usd")

# Add ground plane
xform = UsdGeom.Xform.Define(stage, "/World/GroundPlane")
ground_plane = UsdGeom.Mesh.Define(stage, "/World/GroundPlane/Mesh")
ground_plane.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
ground_plane.CreateFaceVertexCountsAttr([4])
ground_plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])

# Add cube
cube_xform = UsdGeom.Xform.Define(stage, "/World/Cube")
cube = UsdGeom.Cube.Define(stage, "/World/Cube/Geometry")
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
cube.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))

# Save stage
stage.Save()
```

## Creating Robots in Isaac Sim

### Importing URDF to USD

```python
# Python script to run in Isaac Sim
import omni
from omni.isaac.urdf import _urdf

# Import URDF
urdf_path = "/path/to/your/robot.urdf"
usd_path = "/path/to/output/robot.usd"

# Convert URDF to USD
result, prim_path = omni.kit.commands.execute(
    "URDFCreateImportConfig",
    urdf_path=urdf_path,
    import_config=_urdf.ImportConfig(
        merge_fixed_joints=False,
        convex_decompose=True,
        fix_base=False,
        make_default_prim=True,
        self_collision=False,
        create_physics_scene=True
    )
)

# Import robot
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=result
)
```

### Adding Sensors to USD Robot

```python
# Add camera sensor in Isaac Sim
import omni.replicator.core as rep

# Create camera
camera = rep.create.camera(
    position=(2, 0, 1),
    rotation=(0, 0, 0),
    focus_distance=400,
    f_stop=1.8,
    focal_length=24.0
)

# Attach to robot
omni.kit.commands.execute(
    "AttachCameraToPath",
    camera_path=camera.get_path(),
    target_path="/World/Robot/camera_link"
)
```

## Photorealistic Rendering with RTX

### Configuring RTX Settings

```python
# Enable RTX rendering in Isaac Sim
import carb

settings = carb.settings.get_settings()

# Enable ray tracing
settings.set("/rtx/rendermode", "RaytracedLighting")
settings.set("/rtx/pathtracing/enabled", True)
settings.set("/rtx/pathtracing/spp", 32)  # Samples per pixel
settings.set("/rtx/post/dlss/enabled", True)  # DLSS for performance

# HDR environment
from omni.isaac.core.utils.stage import add_reference_to_stage
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCGcom_ExhibitionHall_Interior1.hdr",
    prim_path="/World/Environment"
)
```

## Synthetic Data Generation

### Replicator API for Dataset Creation

```python
import omni.replicator.core as rep
import omni.isaac.core.utils.prims as prims_utils

# Define randomization
def randomize_scene():
    # Randomize lighting
    light = rep.get.light()
    with light:
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 3000))
        rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1)))

    # Randomize object poses
    objects = prims_utils.get_prim_at_path("/World/Objects")
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

# Register randomization
rep.randomizer.register(randomize_scene)

# Create writer for RGB + segmentation
rgb_writer = rep.WriterRegistry.get("BasicWriter")
rgb_writer.initialize(
    output_dir="/tmp/isaac_dataset",
    rgb=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    bounding_box_2d_tight=True
)

# Attach writer to render product
camera = rep.create.camera()
rp = rep.create.render_product(camera, (1280, 720))
rgb_writer.attach([rp])

# Run simulation and capture
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.randomize_scene()
```

### Example: Collecting Training Data

```python
#!/usr/bin/env python3
# Standalone script to generate synthetic dataset

import omni
from omni.isaac.kit import SimulationApp

# Start Isaac Sim headless
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World(stage_units_in_meters=1.0)

# Add warehouse environment
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Environments/Warehouse/warehouse.usd",
    prim_path="/World/Warehouse"
)

# Add robot
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)

# Setup camera
camera = rep.create.camera(
    position=(0, 0, 1.5),
    look_at="/World/Robot"
)

# Configure writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/robot_dataset",
    rgb=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    bounding_box_2d_tight=True,
    colorize_semantic_segmentation=True,
    colorize_instance_segmentation=True
)

# Attach writer
rp = rep.create.render_product(camera, (1920, 1080))
writer.attach([rp])

# Run simulation
for i in range(1000):
    world.step(render=True)
    if i % 10 == 0:
        print(f"Captured frame {i}/1000")

# Cleanup
simulation_app.close()
print("Dataset generation complete!")
```

## ROS 2 Integration

### Publishing Isaac Sim Data to ROS 2

```python
# In Isaac Sim Python console or script
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.graph.core as og

# Create ROS 2 publisher graph
keys = og.Controller.Keys

(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ROS2_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("PublishImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
            ("PublishCameraInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishImage.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishCameraInfo.inputs:execIn"),
            ("CameraHelper.outputs:imageData", "PublishImage.inputs:data"),
            ("CameraHelper.outputs:cameraInfo", "PublishCameraInfo.inputs:cameraInfo"),
        ],
        keys.SET_VALUES: [
            ("PublishImage.inputs:topicName", "camera/image_raw"),
            ("PublishCameraInfo.inputs:topicName", "camera/camera_info"),
            ("CameraHelper.inputs:viewport", "Viewport"),
        ]
    }
)

# Set camera prim path
set_target_prims(
    primPath="/World/ROS2_Graph/CameraHelper",
    targetPrimPaths=["/World/Robot/camera_link/Camera"]
)
```

## Key Takeaways

1. **Isaac Sim** leverages GPU-accelerated RTX rendering for photorealistic simulation
2. **USD format** provides composable, scalable scene descriptions
3. **Replicator API** enables large-scale synthetic dataset generation with domain randomization
4. **ROS 2 integration** allows seamless connection to existing robotics workflows
5. **PhysX 5** delivers high-fidelity, GPU-accelerated physics simulation

## Next Steps

In **Week 9**, we'll explore **Isaac ROS** packages for VSLAM, navigation (Nav2), and real-time perception pipelines optimized for NVIDIA hardware.

---

**References:**
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- [Omniverse Replicator](https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_replicator.html)
- [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
