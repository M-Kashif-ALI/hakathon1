---
sidebar_position: 4
title: 'Simulation and Control Usage'
---

# Simulation and Control Usage

This section covers how URDF models are used in both simulation and real robot control contexts. Understanding this dual usage is crucial for developing humanoid robots that can be tested in simulation before deployment.

## URDF in Simulation

### Gazebo/IGNITION Integration

URDF models can be used directly in physics simulators like Gazebo or Ignition. The simulator uses the URDF to:

- Create visual representations of the robot
- Set up collision detection
- Calculate physics interactions
- Apply joint dynamics

Example Gazebo-specific tags in URDF:

```xml
<link name="link_name">
  <!-- Visual and collision tags as before -->
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.5"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.5"/>
    </geometry>
  </collision>
</link>

<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <turnGravityOff>false</turnGravityOff>
</gazebo>

<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  <axis xyz="0 1 0"/>
</joint>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

### Physics Properties

Simulation requires additional physics properties:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
</inertial>
```

## URDF in Real Robot Control

### Robot State Publisher

The robot_state_publisher node uses URDF to publish the TF tree based on joint states:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        # Process joint states and broadcast transforms
        # based on URDF kinematic structure
        pass
```

### Controller Integration

Controllers use URDF information to understand the robot's structure:

```xml
<!-- Controller configuration that references URDF joint names -->
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotHardware</plugin>
  </hardware>
  <joint>
    <name>left_shoulder_pitch</name>
    <command_interface>
      <name>position</name>
    </command_interface>
    <state_interface>
      <name>position</name>
    </state_interface>
  </joint>
</ros2_control>
```

## URDF Variants for Different Uses

### Simplified URDF for Control

For real-time control, you might use a simplified URDF:

```xml
<!-- Simplified collision geometry for control -->
<link name="simplified_link">
  <collision>
    <geometry>
      <sphere radius="0.1"/>  <!-- Simple sphere instead of complex mesh -->
    </geometry>
  </collision>
</link>
```

### Detailed URDF for Simulation

For simulation, you might use more detailed geometry:

```xml
<!-- Detailed geometry for simulation -->
<link name="detailed_link">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/detailed_model.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://my_robot/meshes/collision_model.stl"/>
    </geometry>
  </collision>
</link>
```

## Launch Files for Simulation vs Control

### Simulation Launch

```xml
<!-- launch_sim.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Load URDF for simulation
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', FindFile('my_robot', 'urdf/sim_robot.urdf.xacro')])
            }]
        ),
        # Launch Gazebo with robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-topic', 'robot_description']
        )
    ])
```

### Real Robot Launch

```xml
<!-- launch_real.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Load URDF for real robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', FindFile('my_robot', 'urdf/real_robot.urdf.xacro')])
            }]
        ),
        # Launch real robot hardware interface
        Node(
            package='my_robot_hardware',
            executable='my_robot_hardware_node'
        )
    ])
```

## Xacro for URDF Parameterization

Xacro (XML Macros) allows parameterization of URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Parameters -->
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.087" />

  <!-- Macro for a wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.2 0.1 0" rpy="0 0 0"/>

</robot>
```

## Simulation-Specific Considerations

### Sensor Integration

```xml
<!-- Simulation sensors -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30.0</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Plugin Configuration

```xml
<!-- Control plugin -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_humanoid</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

## Control-Specific Considerations

### Hardware Interface

```xml
<!-- ros2_control configuration -->
<ros2_control name="HumanoidRobot" type="system">
  <hardware>
    <plugin>my_humanoid_hardware/MyHumanoidHardware</plugin>
    <param name="example_param">example_value</param>
  </hardware>

  <!-- Joints for the left leg -->
  <joint name="left_hip_pitch">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="left_hip_roll">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>

  <!-- Add more joints... -->
</ros2_control>
```

## Best Practices for Simulation-Reality Gap

### 1. Realistic Physics Parameters
- Use accurate mass and inertia values
- Set appropriate friction and damping coefficients
- Model actuator dynamics realistically

### 2. Sensor Noise Modeling
- Add realistic noise models in simulation
- Include latency and bandwidth limitations
- Model sensor failures and limitations

### 3. Control Frequency Considerations
- Match simulation update rates to real hardware capabilities
- Consider computational delays in real systems
- Account for communication delays

### 4. Validation Process
- Test control algorithms in simulation first
- Gradually transfer to real robot with safety measures
- Compare simulation and real robot behavior

## Tools for URDF Validation

### Check URDF
```bash
# Validate URDF syntax and structure
check_urdf my_robot.urdf
```

### Visualize URDF
```bash
# Visualize in RViz
ros2 run rviz2 rviz2
```

### Test Joint Limits
```bash
# Use MoveIt! to test reachability and collisions
ros2 launch moveit_resources demo.launch.py
```

## Key Takeaways

- URDF serves as the bridge between simulation and real robot control
- Simulation uses detailed physics models, control uses simplified models for real-time performance
- Xacro enables parameterization and reusability of URDF models
- Physics properties in URDF affect both simulation and control
- Proper validation is essential to minimize simulation-reality gap
- Use appropriate level of detail for each application
- Consider sensor and actuator limitations in both contexts