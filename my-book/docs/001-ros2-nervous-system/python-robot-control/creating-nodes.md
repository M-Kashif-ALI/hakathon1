---
sidebar_position: 2
title: 'Creating ROS 2 Nodes with rclpy'
---

# Creating ROS 2 Nodes with rclpy

This section covers the fundamentals of creating ROS 2 nodes using Python's rclpy library. You'll learn how to set up a basic node structure, initialize the ROS 2 communication system, and implement the essential components of a robot control node.

## Setting Up Your Environment

Before creating nodes, ensure you have the required dependencies:

```bash
pip install rclpy  # Usually included with ROS 2 installation
```

When using ROS 2 from source or in a development environment, make sure to source your ROS 2 installation:

```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution
```

## Basic Node Structure

Every ROS 2 node in Python follows this basic structure:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize node components here
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Robot Control Node

Here's a practical example of a robot control node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Create subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create a timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot controller initialized')

    def joint_state_callback(self, msg):
        # Process joint state information
        self.get_logger().info(f'Received joint states: {len(msg.position)} joints')

    def control_loop(self):
        # Implement your control logic here
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0, 0.0, 0.0]  # Example joint positions
        self.joint_cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Lifecycle Management

ROS 2 nodes have a well-defined lifecycle that includes:

1. **Initialization**: Setting up publishers, subscribers, services, and timers
2. **Activation**: Node becomes active and starts processing messages
3. **Execution**: Processing callbacks and performing work
4. **Shutdown**: Cleanup and resource release

## Best Practices for Node Creation

### 1. Descriptive Node Names
Use clear, descriptive names that indicate the node's function:
- `left_arm_controller` instead of `arm`
- `lidar_processor` instead of `proc`

### 2. Parameter Configuration
Use parameters to make nodes configurable:
```python
self.declare_parameter('control_rate', 10.0)
control_rate = self.get_parameter('control_rate').value
```

### 3. Error Handling
Implement proper error handling for robust operation:
```python
def safe_publish(self, msg):
    try:
        self.publisher.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Failed to publish: {e}')
```

### 4. Logging
Use appropriate logging levels:
- `info`: Normal operation messages
- `warn`: Potential issues
- `error`: Actual problems
- `debug`: Detailed debugging information

## Advanced Node Features

### Timers
Timers allow you to execute code at regular intervals:
```python
timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
```

### Services
Nodes can provide services for synchronous requests:
```python
self.service = self.create_service(
    MyServiceType,
    'service_name',
    self.service_callback
)
```

### Actions
For long-running tasks with feedback:
```python
self.action_server = ActionServer(
    self,
    MyActionType,
    'action_name',
    self.execute_callback
)
```

## Key Takeaways

- Every ROS 2 node inherits from `rclpy.node.Node`
- Always initialize rclpy before creating nodes
- Use `rclpy.spin()` to process callbacks
- Implement proper cleanup in the node's destructor
- Follow naming conventions and best practices for maintainability
- Use parameters for configuration flexibility