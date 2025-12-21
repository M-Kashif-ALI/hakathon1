---
sidebar_position: 3
title: 'Publishing and Subscribing Patterns'
---

# Publishing and Subscribing Patterns

This section covers the implementation of publish-subscribe communication patterns in Python using rclpy. These patterns are fundamental to ROS 2 and enable effective communication between different parts of a humanoid robot system.

## Publisher Implementation

A publisher sends messages to a topic that any number of subscribers can receive. Here's the basic pattern:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')

        # Create publisher with topic name and queue size
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Optional: Create a timer to publish at regular intervals
        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

## Subscriber Implementation

A subscriber receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')

        # Create subscriber with topic name, callback function, and queue size
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10
        )
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Practical Example: Sensor Data Publisher

Here's a more realistic example for a humanoid robot sensor:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Simulate joint names for a humanoid robot
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
        ]

        # Create timer for publishing at 50Hz (20ms interval)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        self.i = 0

    def publish_joint_states(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Generate simulated joint positions (sine wave pattern)
        positions = []
        for i, _ in enumerate(self.joint_names):
            # Create different oscillation patterns for each joint
            pos = math.sin(self.i * 0.1 + i * 0.5) * 0.5
            positions.append(pos)

        msg.position = positions

        # Simulate velocities and efforts
        velocities = [0.0] * len(positions)
        efforts = [0.0] * len(positions)
        msg.velocity = velocities
        msg.effort = efforts

        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Example: Robot Command Subscriber

Here's an example of a subscriber that receives commands:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class RobotCommandSubscriber(Node):
    def __init__(self):
        super().__init__('robot_command_subscriber')

        # Subscribe to joint commands
        self.command_subscriber = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10
        )

        # Subscribe to current joint states for feedback
        self.state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10
        )

        # Store current state
        self.current_positions = []

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # In a real robot, this would send commands to actuators
        self.execute_command(msg.data)

    def state_callback(self, msg):
        self.current_positions = msg.position

    def execute_command(self, target_positions):
        # This is where you would implement the actual control logic
        # For simulation, we just log the command
        self.get_logger().info(f'Executing command for {len(target_positions)} joints')

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

For humanoid robots, QoS settings are crucial for reliable communication:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For critical safety messages - reliable delivery
safety_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For sensor data - best effort with higher depth
sensor_qos = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Create publisher with specific QoS
safety_publisher = self.create_publisher(String, '/safety_status', safety_qos)
sensor_publisher = self.create_publisher(JointState, '/joint_states', sensor_qos)
```

## Advanced Patterns

### Publisher with Multiple Topics

A single node can publish to multiple topics:

```python
class MultiTopicPublisher(Node):
    def __init__(self):
        super().__init__('multi_topic_publisher')

        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        self.debug_publisher = self.create_publisher(String, '/debug_info', 10)
        self.error_publisher = self.create_publisher(String, '/error_status', 10)

        self.timer = self.create_timer(1.0, self.publish_all)

    def publish_all(self):
        # Publish different information to different topics
        status_msg = String()
        status_msg.data = 'Operational'
        self.status_publisher.publish(status_msg)

        debug_msg = String()
        debug_msg.data = f'Debug info at {self.get_clock().now()}'
        self.debug_publisher.publish(debug_msg)
```

### Conditional Publishing

Only publish when conditions are met:

```python
def conditional_publish(self, new_data):
    if self.should_publish(new_data):
        self.publisher.publish(new_data)

def should_publish(self, data):
    # Only publish if data has changed significantly
    if not hasattr(self, 'last_data'):
        self.last_data = data
        return True

    # Check if change is significant enough
    change_threshold = 0.01
    if abs(data.data - self.last_data.data) > change_threshold:
        self.last_data = data
        return True

    return False
```

## Best Practices

### 1. Queue Size
Choose appropriate queue sizes based on your application:
- Small queues (1-10): For real-time data where old messages are irrelevant
- Large queues (50+): For data that must not be lost

### 2. Naming Conventions
Use consistent naming:
- `/robot_name/sensor_type` (e.g., `/left_arm/joint_states`)
- `/robot_name/command_type` (e.g., `/right_arm/commands`)

### 3. Message Validation
Validate messages before publishing:
```python
def safe_publish(self, msg):
    if self.validate_message(msg):
        self.publisher.publish(msg)
    else:
        self.get_logger().warn('Invalid message, not publishing')
```

### 4. Resource Management
Clean up publishers and subscribers properly:
- Publishers and subscribers are automatically cleaned up when the node is destroyed
- Use try-finally blocks for critical operations

## Key Takeaways

- Publishers send messages to topics, subscribers receive from topics
- QoS settings control message delivery characteristics
- Multiple publishers/subscribers can exist in one node
- Always validate data before publishing
- Use appropriate queue sizes for your application
- Follow naming conventions for consistency
- Implement proper error handling and logging