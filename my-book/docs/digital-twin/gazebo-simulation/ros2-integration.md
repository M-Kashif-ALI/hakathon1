---
sidebar_position: 3
title: 'ROS 2 Integration with Simulated Robots'
---

# ROS 2 Integration with Simulated Robots

This section covers how to integrate simulated robots in Gazebo with ROS 2 systems for control and monitoring, enabling realistic testing of control algorithms.

## Overview

ROS 2 integration with Gazebo enables seamless communication between simulation and the ROS 2 ecosystem. This integration allows students to test control algorithms, perception systems, and navigation stacks in a physics-accurate environment before deploying to real hardware.

## ROS 2-Gazebo Bridge

The ROS 2-Gazebo bridge is a critical component that enables communication between ROS 2 nodes and the Gazebo simulation environment.

### Required Packages

The primary package for ROS 2-Gazebo integration is `ros_gz` (ROS Garden Bridge), which provides:

- Message bridging between ROS 2 and Gazebo
- TF tree publishing for coordinate transforms
- Sensor data publishing from simulated sensors
- Actuator control command handling

### Installation

```bash
# Install ROS 2 Humble packages for Gazebo integration
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-gazebo-ros-pkgs
```

## Robot Model Integration

### URDF to SDF Conversion

Gazebo uses SDF (Simulation Description Format) while ROS 2 typically uses URDF (Unified Robot Description Format). The integration process involves:

1. **URDF Definition**: Define the robot in URDF with proper joint and link properties
2. **SDF Generation**: Gazebo can automatically convert URDF to SDF
3. **Simulation-Specific Extensions**: Add Gazebo-specific tags to the URDF

Example URDF with Gazebo extensions:

```xml
<robot name="humanoid_robot">
  <!-- Standard URDF links and joints -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- ROS 2 control plugin -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

## Control Interface

### Joint State Publisher

The joint state publisher provides real-time information about joint positions, velocities, and efforts:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']  # Actual joint names
        msg.position = [0.0, 0.5, -0.3]  # Actual positions
        msg.velocity = [0.0, 0.1, -0.05]  # Actual velocities
        msg.effort = [0.0, 5.0, -2.0]  # Actual efforts

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.publisher.publish(msg)
```

### Joint Trajectory Controller

For precise control of robot movements, use joint trajectory controllers:

```yaml
# Controller configuration file
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_trajectory_controller:
      ros__parameters:
        joints:
          - joint1
          - joint2
          - joint3
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity
```

## Sensor Integration

### Simulated Sensors

Gazebo can simulate various sensor types that publish ROS 2 messages:

#### LiDAR Simulation

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

#### Camera Simulation

```xml
<gazebo reference="camera_link">
  <sensor name="camera_sensor" type="camera">
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

#### IMU Simulation

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Launch Files

### Basic Simulation Launch

Create launch files to start both Gazebo and ROS 2 nodes:

```python
# launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'my_robot.urdf.xacro'
            ])])
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Control Strategies

### Position Control

For simple position-based control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

    def move_to_position(self, joint_positions):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']  # Actual joint names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 1  # Move to position in 1 second

        msg.points.append(point)
        self.pub.publish(msg)
```

### Velocity and Effort Control

For more advanced control strategies:

```python
# Velocity control
from std_msgs.msg import Float64MultiArray

def publish_velocity_commands(self, velocities):
    msg = Float64MultiArray()
    msg.data = velocities
    self.velocity_pub.publish(msg)

# Effort control
def publish_effort_commands(self, efforts):
    msg = Float64MultiArray()
    msg.data = efforts
    self.effort_pub.publish(msg)
```

## Best Practices

### Simulation Accuracy

1. **Realistic Robot Models**: Use accurate URDF models with proper mass and inertia properties
2. **Sensor Noise**: Add realistic noise models to simulated sensors
3. **Physics Parameters**: Tune damping and friction parameters to match real-world behavior

### Performance Optimization

1. **Update Rates**: Balance sensor update rates with computational performance
2. **Physics Step Size**: Adjust simulation step size based on required accuracy
3. **Model Simplification**: Use simplified collision meshes where appropriate

### Debugging and Validation

1. **TF Tree**: Verify proper TF tree publication between robot frames
2. **Topic Monitoring**: Monitor sensor topics and control command topics
3. **Visualization**: Use RViz2 to visualize robot state and sensor data

## Key Takeaways

- ROS 2 integration enables realistic testing of robot algorithms in simulation
- Proper configuration of robot models and controllers is essential for accurate simulation
- Sensor simulation allows testing of perception and navigation algorithms
- Launch files provide organized startup of simulation and ROS 2 nodes
- Balance accuracy with performance for efficient simulation workflows