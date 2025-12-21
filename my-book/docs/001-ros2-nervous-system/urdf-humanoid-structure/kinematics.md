---
sidebar_position: 3
title: 'Kinematics in URDF'
---

# Kinematics in URDF

This section covers how URDF defines the kinematic structure of humanoid robots and how this information is used for motion planning and control.

## Kinematic Chains Overview

A **kinematic chain** is an assembly of rigid bodies (links) connected by joints that transmit motion. In humanoid robots, kinematic chains define how different body parts move relative to each other.

### Forward Kinematics

Forward kinematics calculates the position and orientation of the end effector (like a hand or foot) based on the joint angles. URDF provides the structure needed for these calculations.

### Inverse Kinematics

Inverse kinematics calculates the required joint angles to achieve a desired end effector position and orientation. URDF structure is essential for solving these problems.

## URDF and Kinematics

URDF provides the kinematic structure but not the kinematic solvers themselves. The kinematic information in URDF includes:

- Link lengths and dimensions
- Joint types and axes
- Joint limits
- Transformation matrices between links

## Kinematic Models in ROS

### Robot State Publisher

The robot_state_publisher node uses URDF to publish transformations between links:

```xml
<!-- In your launch file -->
<node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
  <param name="robot_description" value="$(var robot_description)"/>
</node>
```

### TF (Transform) Tree

URDF defines the structure of the TF tree, which represents all coordinate frames and their relationships in the robot.

## Example: Kinematic Chain for Humanoid Arm

Here's how a simple arm kinematic chain is represented in URDF:

```xml
<!-- Base of the arm -->
<link name="torso">
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.5 0.3 0.8"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 0.8"/>
    </geometry>
  </collision>
</link>

<!-- Shoulder joint -->
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.2 0.15 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>

<link name="left_upper_arm">
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
</link>

<joint name="left_elbow" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.35" effort="80" velocity="2"/>
</joint>

<link name="left_lower_arm">
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </collision>
</link>

<joint name="left_wrist_pitch" type="revolute">
  <parent link="left_lower_arm"/>
  <child link="left_hand"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>

<link name="left_hand">
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.1 0.08 0.15"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.08 0.15"/>
    </geometry>
  </collision>
</link>
```

## Kinematic Solvers Integration

While URDF defines the structure, kinematic solvers handle the calculations:

### KDL (Kinematics and Dynamics Library)
- Provides forward and inverse kinematics solvers
- Works with URDF structure to perform calculations

### MoveIt!
- High-level motion planning framework
- Uses URDF to understand robot structure
- Provides inverse kinematics solvers

## Kinematic Constraints in Humanoid Robots

### Closed Kinematic Chains

Humanoid robots often have closed kinematic chains when both feet are on the ground or when both hands grasp an object. This requires special handling:

```xml
<!-- For manipulation tasks, you might have multiple end effectors -->
<joint name="left_gripper_joint" type="fixed">
  <parent link="left_hand"/>
  <child link="left_gripper"/>
</joint>

<link name="left_gripper">
  <visual>
    <geometry>
      <box size="0.05 0.02 0.05"/>
    </geometry>
  </visual>
</link>
```

### Center of Mass Considerations

For stable humanoid locomotion, the center of mass trajectory is crucial:

```xml
<!-- The URDF helps calculate center of mass for each link -->
<!-- Which is essential for balance control algorithms -->
```

## Practical Kinematic Applications

### Walking Pattern Generation
- URDF structure is used to calculate footstep positions
- Center of mass trajectory planning
- Balance maintenance during locomotion

### Manipulation Tasks
- Inverse kinematics for reaching tasks
- Trajectory planning for smooth motion
- Collision avoidance using URDF geometry

### Whole-Body Control
- Coordination of multiple kinematic chains
- Balance control using all available DOFs
- Task-space control with multiple constraints

## URDF for Kinematic Simulation

### Gazebo Integration
```xml
<!-- Additional tags for physics simulation -->
<gazebo reference="left_elbow">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

### Joint Limits and Safety
```xml
<!-- Proper joint limits prevent kinematic singularities -->
<limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
```

## Debugging Kinematic Issues

### Common Problems
1. **Kinematic Singularities**: Occur when joints align in problematic configurations
2. **Joint Limit Violations**: When IK solvers ignore physical limits
3. **Poor Convergence**: IK solvers fail to find solutions

### Visualization Tools
- RViz for TF tree visualization
- MoveIt! Rviz plugins for motion planning
- URDF model display to verify structure

## Key Takeaways

- URDF defines the kinematic structure but not the solvers
- Kinematic chains connect base to end effectors
- Joint limits in URDF prevent physical damage
- Forward kinematics: joint angles → end effector position
- Inverse kinematics: end effector position → joint angles
- Proper mass properties are essential for dynamics
- Closed kinematic chains require special handling
- URDF is fundamental for motion planning and control