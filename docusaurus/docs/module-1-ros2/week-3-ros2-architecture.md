---
title: Week 3 - ROS 2 Architecture and Core Concepts
description: Master ROS 2 Nodes, Topics, Services, and Actions for distributed robotic systems
sidebar_position: 1
hardware_tier: beginner
learning_objectives:
  - Understand the ROS 2 computational graph and communication paradigms
  - Create and manage ROS 2 Nodes for modular robot control
  - Implement publish-subscribe patterns with Topics
  - Use Services for synchronous request-response and Actions for long-running tasks
estimated_time_minutes: 120
---

# Week 3: ROS 2 Architecture and Core Concepts

## Introduction to ROS 2

**ROS 2** (Robot Operating System 2) is a middleware framework designed for distributed robotic systems. Unlike its predecessor (ROS 1), ROS 2 is built on **DDS** (Data Distribution Service) for real-time, reliable communication suitable for production robotics.

### Key Improvements in ROS 2
- **Real-time capable**: Deterministic communication with QoS policies
- **Multi-robot systems**: Native support for distributed networks
- **Security**: Built-in authentication and encryption (SROS2)
- **Cross-platform**: Linux, Windows, macOS support
- **Better tooling**: Improved CLI, launch system, and debugging

## ROS 2 Computational Graph

The ROS 2 system is modeled as a **computational graph** where:
- **Nodes** are processes that perform computation
- **Topics** enable asynchronous publish-subscribe messaging
- **Services** provide synchronous request-response calls
- **Actions** handle long-running, preemptible tasks with feedback

### ROS 2 Node Architecture

A **Node** is the fundamental building block in ROS 2. Each node:
- Runs as an independent process
- Can publish/subscribe to Topics
- Can provide/call Services and Actions
- Has a unique name in the ROS 2 graph

```python
# Example: Creating a simple ROS 2 Node
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal Node has been started!')

        # Create a timer that calls a callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer callback executed {self.counter} times')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)  # Keep node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run this node:**
```bash
# Save as minimal_node.py
python3 minimal_node.py

# In another terminal, inspect the node
ros2 node list
ros2 node info /minimal_node
```

## Topics: Publish-Subscribe Communication

**Topics** enable many-to-many asynchronous communication. Nodes can:
- **Publish** messages to a topic (data producers)
- **Subscribe** to a topic (data consumers)

### Creating a Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create a publisher on the 'robot_status' topic
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # Publish a message every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_status)
        self.counter = 0

    def publish_status(self):
        msg = String()
        msg.data = f'Robot operational: iteration {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Subscribe to the 'robot_status' topic
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test the pub-sub system:**
```bash
# Terminal 1: Run the publisher
python3 publisher_node.py

# Terminal 2: Run the subscriber
python3 subscriber_node.py

# Terminal 3: Inspect the topic
ros2 topic list
ros2 topic info /robot_status
ros2 topic echo /robot_status
```

## Services: Request-Response Communication

**Services** provide synchronous communication for request-response patterns (e.g., querying robot state, triggering actions).

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('AddTwoInts service is ready')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Call the service from CLI:**
```bash
# Terminal 1: Run the service server
python3 service_server.py

# Terminal 2: Call the service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

## Actions: Long-Running Tasks with Feedback

**Actions** are for tasks that:
- Take time to complete (e.g., navigation, grasping)
- Provide periodic feedback
- Can be canceled mid-execution

Actions consist of three communication channels:
1. **Goal**: Client sends a goal to the action server
2. **Feedback**: Server sends progress updates
3. **Result**: Server sends final result when complete

### Example: Fibonacci Action Server

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci Action Server started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Generate Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            # Compute next Fibonacci number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return final result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test the action:**
```bash
# Terminal 1: Run the action server
python3 fibonacci_action_server.py

# Terminal 2: Send a goal
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}" --feedback
```

## Quality of Service (QoS) Policies

ROS 2 uses **QoS policies** to configure communication reliability and performance:

- **Reliability**: `RELIABLE` (guaranteed delivery) vs `BEST_EFFORT` (lossy, low latency)
- **Durability**: `TRANSIENT_LOCAL` (late joiners receive history) vs `VOLATILE` (no history)
- **History**: `KEEP_LAST` (buffer size) vs `KEEP_ALL`

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create a QoS profile for sensor data (best effort, low latency)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Create a publisher with custom QoS
self.publisher_ = self.create_publisher(
    LaserScan,
    'scan',
    qos_profile=sensor_qos
)
```

## Key Takeaways

1. **Nodes** are the fundamental units of computation in ROS 2
2. **Topics** enable asynchronous, many-to-many communication
3. **Services** provide synchronous request-response patterns
4. **Actions** handle long-running, preemptible tasks with feedback
5. **QoS policies** allow fine-tuning of communication behavior for different use cases

## Next Steps

In **Week 4**, we'll build complete ROS 2 packages with custom message types, launch files, and parameter management for real-world robotic systems.

---

**References:**
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Principles](https://design.ros2.org/)
