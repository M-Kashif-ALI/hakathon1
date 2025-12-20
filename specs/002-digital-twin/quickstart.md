# Quickstart: Digital Twin for Humanoid Robotics (Gazebo & Unity)

## Overview
This quickstart guide will help you set up and begin working with the digital twin module for humanoid robotics, covering Gazebo physics simulation, Unity rendering, and ROS 2 sensor integration.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Gazebo Garden or compatible version
- Unity Hub with Unity 2022.3 LTS (for Unity integration)
- Node.js 18+ and npm/yarn
- Basic understanding of ROS 2 concepts

## Setup Process

### 1. Environment Setup
```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Install Docusaurus dependencies
cd my-book
npm install
```

### 2. Gazebo Simulation Setup
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install Gazebo dependencies
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify Gazebo installation
gazebo --version
```

### 3. Unity Integration Setup (Optional)
```bash
# Install rosbridge_suite for Unity-ROS communication
sudo apt install ros-humble-rosbridge-suite

# Unity ROS-TCP-Connector can be installed via Unity Package Manager
# Follow the documentation in the Unity chapter
```

## Running the Examples

### 1. Start the Documentation Server
```bash
cd my-book
npm start
```
This will start a local development server at http://localhost:3000

### 2. Navigate to Digital Twin Module
- Open your browser to http://localhost:3000
- Go to "Module 2: Digital Twin" in the documentation
- Follow the chapters in order:
  1. Physics Simulation with Gazebo
  2. Environment & Interaction in Unity
  3. Sensor Simulation

### 3. Basic Gazebo Simulation Example
```bash
# In a new terminal, source ROS 2
source /opt/ros/humble/setup.bash

# Launch a simple humanoid robot simulation
ros2 launch gazebo_ros empty_world.launch.py
```

### 4. Sensor Simulation Example
```bash
# Launch a robot with simulated sensors
ros2 launch your_robot_gazebo robot_with_sensors.launch.py

# View sensor data
ros2 topic list | grep /sensor
ros2 topic echo /sensor/lidar_scan sensor_msgs/msg/LaserScan
```

## Key Concepts to Understand

### Digital Twin Environment
- A virtual representation that mirrors real-world properties
- Combines physics accuracy (Gazebo) with visual fidelity (Unity)
- Allows testing before real-world deployment

### Physics Simulation
- Configurable parameters: gravity, collisions, dynamics
- Integration with ROS 2 through standard interfaces
- Realistic behavior for humanoid robots

### Sensor Simulation
- LiDAR, depth cameras, and IMUs
- Realistic data output compatible with ROS 2
- Essential for perception algorithm testing

## Troubleshooting

### Common Issues
1. **Gazebo not launching**: Ensure proper ROS 2 sourcing and dependencies installed
2. **Documentation not building**: Check Node.js version and run `npm install` again
3. **Sensor data not publishing**: Verify sensor configuration in URDF

### Getting Help
- Check the detailed chapters in the documentation
- Review the ROS 2 and Gazebo official documentation
- Examine the example configurations in the static/examples directory

## Next Steps
1. Complete the Physics Simulation with Gazebo chapter
2. Explore Environment & Interaction in Unity
3. Implement Sensor Simulation techniques
4. Integrate all components in a complete digital twin scenario