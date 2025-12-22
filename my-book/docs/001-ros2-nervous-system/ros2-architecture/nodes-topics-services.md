---
sidebar_position: 3
title: 'Nodes, Topics, Services, and Message Flow'
---

# Nodes, Topics, Services, and Message Flow

This section covers the fundamental communication mechanisms in ROS 2: nodes, topics, and services. Understanding these concepts is crucial for implementing effective communication patterns in humanoid robot systems.

## Nodes: The Building Blocks

A **node** is a process that performs computation in a ROS 2 system. In humanoid robotics, nodes typically represent:

- Individual sensors (camera, LIDAR, IMU)
- Actuator controllers (motor drivers, servo controllers)
- Processing units (perception, planning, control)
- AI agents (behavior, learning, decision-making)

### Node Characteristics

- Each node runs as a separate process
- Nodes communicate with each other through topics and services
- Nodes can be written in different programming languages
- Nodes can run on different machines in a distributed system

## Topics: Publish-Subscribe Communication

**Topics** enable asynchronous, one-way communication through a publish-subscribe pattern:

```
Publisher Node → Topic → Subscriber Node
```

### Key Features

- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Broadcast**: One publisher can send to multiple subscribers
- **Asynchronous**: Messages are sent without waiting for responses
- **Data-driven**: Ideal for sensor data, control commands, and state updates

### Example in Humanoid Robotics

```python
# Publisher: Sensor node publishing joint states
publisher = node.create_publisher(JointState, '/joint_states', 10)

# Subscriber: Controller node receiving joint states
subscriber = node.create_subscription(JointState, '/joint_states', callback, 10)
```

## Services: Request-Response Communication

**Services** enable synchronous, two-way communication through a request-response pattern:

```
Client Node → Request → Service Server Node
Client Node ← Response ← Service Server Node
```

### Key Features

- **Synchronous**: Client waits for response from server
- **Request-Response**: Each request gets exactly one response
- **Action-oriented**: Ideal for tasks that require confirmation or results
- **Reliable**: Request-response pattern ensures communication completion

### Example in Humanoid Robotics

```python
# Service Server: Calibration service
service = node.create_service(Calibrate, '/calibrate_arm', calibrate_callback)

# Service Client: Requesting calibration
client = node.create_client(Calibrate, '/calibrate_arm')
```

## Actions: Advanced Request-Response

**Actions** are an extension of services that support long-running tasks with feedback:

- Goal: Initial request
- Feedback: Periodic updates during execution
- Result: Final outcome when task completes

Actions are ideal for complex robot behaviors like navigation or manipulation tasks.

## Message Flow in Humanoid Robots

### Sensor Data Flow

```
Sensors → Nodes → Topics → Processing Nodes → Topics → Controllers
```

Example flow:
1. IMU sensor node publishes orientation data to `/imu/data`
2. State estimation node subscribes to `/imu/data` and computes robot state
3. Controller node subscribes to estimated state and computes control commands
4. Controller publishes commands to actuator topics like `/left_arm/commands`

### Control Command Flow

```
AI Decision → Topics → Controllers → Actuators → Physical Movement
```

### Coordination Flow

```
Coordinator → Topics → Multiple Subsystems → Synchronized Behavior
```

## Quality of Service (QoS) Settings

ROS 2 provides QoS settings to control communication behavior:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (for late-joining nodes)
- **History**: Keep last N messages vs. keep all messages
- **Rate**: Communication frequency settings

These settings are crucial for humanoid robots where some data (like safety information) requires reliable delivery while other data (like video feeds) can tolerate some loss.

## Best Practices for Humanoid Robotics

1. **Modularity**: Design nodes to have single, well-defined responsibilities
2. **Naming**: Use consistent, descriptive names for topics and services
3. **QoS**: Choose appropriate QoS settings based on data criticality
4. **Rate Control**: Manage message rates to avoid overwhelming the system
5. **Error Handling**: Implement robust error handling for communication failures

## Key Takeaways

- Nodes are the fundamental computational units in ROS 2
- Topics enable asynchronous publish-subscribe communication
- Services provide synchronous request-response communication
- Actions support long-running tasks with feedback
- Proper message flow design is essential for effective humanoid robot control
- QoS settings allow fine-tuning of communication behavior