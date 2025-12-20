---
sidebar_position: 2
title: 'URDF Links and Joints'
---

# URDF Links and Joints

This section covers the fundamental building blocks of URDF: links and joints. Understanding these concepts is essential for defining the structure of humanoid robots in ROS.

## URDF Overview

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and their relationships.

## Links: The Rigid Bodies

A **link** represents a rigid body part of the robot. Each link can have:

- Visual properties (for display)
- Collision properties (for physics simulation)
- Inertial properties (for dynamics)

### Basic Link Structure

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="1 2 3"/>
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="1 2 3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Link Properties

#### Visual Properties
- **geometry**: Shape of the link (box, cylinder, sphere, mesh)
- **material**: Color and texture information
- **origin**: Position and orientation relative to joint

#### Collision Properties
- **geometry**: Collision shape (often simplified from visual)
- **origin**: Position and orientation relative to joint

#### Inertial Properties
- **mass**: Mass of the link
- **inertia**: Inertial tensor values for dynamics simulation

## Joints: The Connections

A **joint** connects two links and defines their relative motion. Joints have types that determine the allowed degrees of freedom.

### Joint Types

1. **revolute**: Rotational joint with limited range
2. **continuous**: Rotational joint without limits
3. **prismatic**: Linear sliding joint with limits
4. **fixed**: No movement (rigid connection)
5. **floating**: 6 DOF (rarely used)
6. **planar**: Motion on a plane

### Basic Joint Structure

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Joint Properties

#### Connection Properties
- **parent**: Name of the parent link
- **child**: Name of the child link
- **origin**: Position and orientation of the joint relative to parent

#### Motion Properties
- **axis**: Axis of motion (for revolute/prismatic joints)
- **limit**: Range of motion and physical limits
- **dynamics**: Damping and friction coefficients

## Example: Simple Humanoid Limb

Here's an example of a simple humanoid arm with two joints:

```xml
<!-- Shoulder Link -->
<link name="shoulder">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
  </inertial>
</link>

<!-- Upper Arm Link -->
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0.2 0.2 0.8 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
  </inertial>
</link>

<!-- Shoulder Joint -->
<joint name="shoulder_pitch" type="revolute">
  <parent link="shoulder"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
</joint>
```

## Humanoid-Specific Considerations

### Head Structure
- Use a spherical or box-shaped head link
- Include sensor links (cameras, IMU) as fixed joints

### Torso Structure
- Typically a single large link or multiple connected links
- Should include attachment points for limbs

### Limb Structure
- Arms: shoulder → upper arm → lower arm → hand
- Legs: hip → thigh → shin → foot
- Use appropriate joint limits for human-like movement

### Common Joint Limits for Humanoid Robots

#### Arms
- Shoulder: -1.57 to 1.57 rad (±90°) for pitch, -0.78 to 1.57 rad for roll
- Elbow: 0 to 2.35 rad (0 to 135°)
- Wrist: -1.57 to 1.57 rad for pitch and roll

#### Legs
- Hip: -0.78 to 0.78 rad for pitch, -0.52 to 0.52 rad for roll and yaw
- Knee: 0 to 2.35 rad (0 to 135°)
- Ankle: -0.52 to 0.52 rad for pitch and roll

## URDF Best Practices

### 1. Naming Conventions
- Use descriptive, consistent names
- Follow a hierarchy (e.g., `left_arm_shoulder`, `right_leg_hip`)

### 2. Mass and Inertia
- Accurately estimate mass properties for simulation
- Use simplified shapes for collision detection
- Consider the weight of actuators and electronics

### 3. Joint Limits
- Set realistic limits based on physical constraints
- Include safety margins to prevent damage
- Consider the range of motion needed for tasks

### 4. Structure Organization
- Create a clear kinematic chain from base to end effectors
- Use fixed joints to attach sensors and accessories
- Group related links logically

## Validation and Testing

Always validate your URDF files:

```bash
# Check URDF syntax
check_urdf your_robot.urdf

# Visualize in RViz
ros2 run rviz2 rviz2

# Use robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..."
```

## Key Takeaways

- Links represent rigid body parts of the robot
- Joints connect links and define their relative motion
- URDF uses XML format to describe robot structure
- Proper mass and inertia properties are crucial for simulation
- Joint limits should reflect physical constraints
- Use appropriate geometry for both visual and collision models
- Follow consistent naming conventions for maintainability