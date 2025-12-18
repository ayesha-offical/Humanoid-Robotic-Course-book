"""
Simple script to initialize Qdrant collection and ingest sample curriculum data.
"""

import os
from dotenv import load_dotenv
from embeding_helpers import create_collection, save_chunk_to_qdrant, chunk_text, embed
from qdrant_client import QdrantClient

load_dotenv()

qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
collection_name = os.getenv("COLLECTION_NAME")

# Connect to Qdrant
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    check_compatibility=False
)

# Sample curriculum content
SAMPLE_DATA = {
    "ROS 2 Installation": """
    ROS 2 is a middleware for robotics development. To install ROS 2:

    1. Download from https://docs.ros.org/en/humble/Installation.html
    2. Choose your operating system (Ubuntu, Windows, macOS)
    3. Follow the installation guide specific to your OS
    4. Verify installation with: ros2 --version
    5. Set up your environment with: source /opt/ros/humble/setup.bash

    ROS 2 requires Python 3.9 or higher and uses colcon as build tool.
    """,

    "ROS 2 Core Concepts": """
    ROS 2 architecture consists of:

    - Nodes: Independent processes that communicate with each other
    - Topics: Named buses for asynchronous communication
    - Services: Request-reply communication pattern
    - Actions: Long-running tasks with feedback
    - Messages: Strongly typed data structures

    The middleware uses DDS (Data Distribution Service) for communication.
    """,

    "Physical AI and Humanoid Robotics": """
    Physical AI combines machine learning with robotics for embodied intelligence.

    Key areas:
    - Perception: Using sensors (cameras, lidar) to understand the environment
    - Planning: Motion planning and task planning algorithms
    - Control: Low-level control and feedback loops
    - Learning: Reinforcement learning and imitation learning

    Humanoid robots replicate human form factors and movements,
    enabling robots to work in human-designed environments.
    """,

    "Gazebo Simulation": """
    Gazebo is a powerful robot simulator used with ROS 2.

    Features:
    - 3D physics simulation with ODE, Bullet, or DART engines
    - Sensor simulation (camera, lidar, IMU)
    - Plugin system for custom behaviors
    - Integration with ROS 2 topics and services

    To launch Gazebo with ROS 2: ros2 launch gazebo_ros gazebo.launch.py
    """,

    "Sim-to-Real Transfer": """
    Sim-to-Real Transfer bridges the gap between simulation and physical robots.

    Challenges:
    - Physics inaccuracies in simulation
    - Sensor noise and delays
    - Actuator differences

    Solutions:
    - Domain randomization in simulation
    - Careful system identification
    - Robust control policies
    - Incremental real-world testing
    """
}


def main():
    """Initialize collection and ingest sample data."""

    print("=" * 60)
    print("Qdrant Collection Initialization & Data Ingestion")
    print("=" * 60)

    # Step 1: Create collection
    print("\n[1/3] Creating Qdrant collection...")
    try:
        create_collection()
        print("✓ Collection created or already exists")
    except Exception as e:
        print(f"✗ Error creating collection: {e}")
        return

    # Step 2: Ingest sample data
    print("\n[2/3] Ingesting sample curriculum data...")
    chunk_id = 1

    for title, content in SAMPLE_DATA.items():
        print(f"\n  Processing: {title}")

        try:
            # Chunk the text
            chunks = chunk_text(content, max_chars=500)
            print(f"  Generated {len(chunks)} chunks")

            # Save each chunk
            for i, chunk in enumerate(chunks):
                save_chunk_to_qdrant(
                    chunk=chunk,
                    chunk_id=chunk_id,
                    url=f"/docs/{title.lower().replace(' ', '-')}"
                )
                chunk_id += 1

            print(f"  ✓ Saved {len(chunks)} chunks")

        except Exception as e:
            print(f"  ✗ Error processing {title}: {e}")

    # Step 3: Verify data
    print("\n[3/3] Verifying ingested data...")
    try:
        collection_info = qdrant.get_collection(collection_name)
        point_count = collection_info.points_count
        print(f"✓ Collection '{collection_name}' now contains {point_count} points")
    except Exception as e:
        print(f"✗ Error verifying collection: {e}")
        return

    print("\n" + "=" * 60)
    print("✓ Ingestion Complete!")
    print("=" * 60)
    print("\nYou can now test the API with queries like:")
    print("  - 'How do I install ROS 2?'")
    print("  - 'What is Physical AI?'")
    print("  - 'Explain sim-to-real transfer'")
    print("  - 'Tell me about Gazebo simulation'")
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
