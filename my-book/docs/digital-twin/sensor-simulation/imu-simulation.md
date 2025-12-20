---
sidebar_position: 4
title: 'IMU Simulation'
---

# IMU Simulation

This section covers the simulation of Inertial Measurement Units (IMUs) in robotics environments, including orientation, angular velocity, and linear acceleration data generation, noise modeling, and integration with ROS 2 for humanoid robotics applications.

## Overview

IMUs are critical sensors for robotics applications, providing information about a robot's orientation, angular velocity, and linear acceleration. IMU simulation enables testing of state estimation algorithms, balance control, navigation, and other applications that require motion sensing in a controlled environment.

## IMU Fundamentals

### How IMUs Work

An IMU typically combines three types of sensors:

- **Accelerometer**: Measures linear acceleration along three axes
- **Gyroscope**: Measures angular velocity around three axes
- **Magnetometer**: Measures magnetic field (provides absolute orientation reference)

### Key Parameters

- **Sample Rate**: How frequently the IMU updates (typically 100Hz-1000Hz)
- **Measurement Range**: Maximum measurable values for each sensor type
- **Noise Density**: Noise characteristics of each sensor
- **Bias Stability**: Long-term stability of sensor readings
- **Scale Factor Error**: Deviation from ideal sensor response
- **Cross-Axis Sensitivity**: Crosstalk between different axes

## Simulation Principles

### Physics-Based Simulation

IMU simulation can be based on:

1. **Pure kinematic simulation**: Calculate measurements from known motion
2. **Physics-based simulation**: Include realistic sensor behavior and noise
3. **Hybrid approach**: Combine kinematic and physics models

### IMU Data Generation

```xml
<!-- Example Gazebo IMU sensor configuration -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev> <!-- 0.1 deg/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev> <!-- 0.017 m/s^2 -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

## Noise Modeling

### Realistic Noise Simulation

Real IMUs have various sources of noise that must be simulated:

- **White Noise**: Random noise with constant power spectral density
- **Bias Drift**: Slow-changing offset in sensor readings
- **Quantization Noise**: Discrete steps due to digital conversion
- **Temperature Effects**: Changes in sensor characteristics with temperature

### Noise Parameters

```xml
<sensor name="imu_sensor" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev> <!-- White noise: 0.1 deg/s -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00017</bias_stddev> <!-- Bias drift -->
        </noise>
      </x>
      <!-- Similar for y and z axes -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev> <!-- White noise: 0.017 m/s^2 -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0017</bias_stddev> <!-- Bias drift -->
        </noise>
      </x>
      <!-- Similar for y and z axes -->
    </linear_acceleration>
  </imu>
</sensor>
```

## ROS 2 Integration

### Message Types

IMU data is typically published as `sensor_msgs/Imu`:

```python
# Example IMU message in Python
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import math

def create_imu_message():
    imu_msg = Imu()
    imu_msg.header = Header()
    imu_msg.header.stamp = self.get_clock().now().to_msg()
    imu_msg.header.frame_id = 'imu_link'

    # Orientation (in quaternion format)
    # For simulation, this might come from the robot's physics state
    imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Orientation covariance (set to 0 if not available)
    imu_msg.orientation_covariance = [0.0] * 9  # Unknown covariance

    # Angular velocity (rad/s)
    imu_msg.angular_velocity = Vector3(x=0.1, y=0.05, z=0.02)

    # Angular velocity covariance
    imu_msg.angular_velocity_covariance = [0.0] * 9  # Unknown covariance

    # Linear acceleration (m/s^2)
    imu_msg.linear_acceleration = Vector3(x=0.5, y=0.1, z=9.81)  # Include gravity

    # Linear acceleration covariance
    imu_msg.linear_acceleration_covariance = [0.0] * 9  # Unknown covariance

    return imu_msg
```

### Covariance Matrices

Covariance matrices represent uncertainty in measurements:

```python
def create_imu_with_covariance():
    imu_msg = Imu()
    imu_msg.header.stamp = self.get_clock().now().to_msg()
    imu_msg.header.frame_id = 'imu_link'

    # Set orientation (example: small roll and pitch)
    roll = 0.05  # 0.05 radians = ~2.86 degrees
    pitch = 0.02  # 0.02 radians = ~1.15 degrees
    yaw = 0.0

    # Convert Euler angles to quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

    # Orientation covariance (diagonal: roll, pitch, yaw variances)
    imu_msg.orientation_covariance = [
        0.01, 0.0,  0.0,   # Row 1: var_roll, cov_ry, cov_rz
        0.0,  0.01, 0.0,   # Row 2: cov_pr, var_pitch, cov_pz
        0.0,  0.0,  0.01   # Row 3: cov_yr, cov_yp, var_yaw
    ]

    # Angular velocity with covariance
    imu_msg.angular_velocity = Vector3(x=0.01, y=0.005, z=0.002)
    imu_msg.angular_velocity_covariance = [
        0.0001, 0.0,    0.0,      # Row 1: var_x, cov_xy, cov_xz
        0.0,    0.0001, 0.0,      # Row 2: cov_yx, var_y, cov_yz
        0.0,    0.0,    0.0001    # Row 3: cov_zx, cov_zy, var_z
    ]

    # Linear acceleration with covariance
    imu_msg.linear_acceleration = Vector3(x=0.2, y=0.1, z=9.81)
    imu_msg.linear_acceleration_covariance = [
        0.01, 0.0,  0.0,    # Row 1: var_x, cov_xy, cov_xz
        0.0,  0.01, 0.0,    # Row 2: cov_yx, var_y, cov_yz
        0.0,  0.0,  0.01    # Row 3: cov_zx, cov_zy, var_z
    ]

    return imu_msg
```

## Common IMU Models

### Low-Cost MEMS IMU (e.g., MPU-6050)

```xml
<sensor name="mpu6050_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001745</stddev> <!-- 0.1 deg/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001745</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001745</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev> <!-- 0.017 m/s^2 -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

### High-Performance IMU (e.g., ADIS16470)

```xml
<sensor name="high_perf_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>500</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0001745</stddev> <!-- 0.01 deg/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0001745</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0001745</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev> <!-- 0.0017 m/s^2 -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Integration with Robot Dynamics

### Physics Engine Integration

IMU readings should be consistent with the robot's physical motion:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def simulate_imu_from_physics(robot_state):
    """
    Generate IMU measurements based on robot's physical state
    """
    # Get robot's linear acceleration (from physics simulation)
    linear_accel = robot_state.linear_acceleration

    # Add gravity to acceleration (IMU measures proper acceleration)
    gravity = np.array([0, 0, -9.81])
    imu_linear_accel = linear_accel - gravity

    # Get angular velocity from physics state
    angular_vel = robot_state.angular_velocity

    # Apply sensor noise and bias
    noise_std = 0.0017  # 0.1 deg/s for gyroscope
    bias_drift = 0.0001  # Slow bias drift

    # Add noise to measurements
    noisy_angular_vel = angular_vel + np.random.normal(0, noise_std, 3) + bias_drift
    noisy_linear_accel = imu_linear_accel + np.random.normal(0, 0.017, 3)  # 0.017 m/s^2

    # Get orientation from robot state
    orientation_quat = robot_state.orientation

    return {
        'orientation': orientation_quat,
        'angular_velocity': noisy_angular_vel,
        'linear_acceleration': noisy_linear_accel
    }
```

## Sensor Fusion Considerations

### Complementary Filtering

Combine IMU data with other sensors for better state estimation:

```python
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.orientation = np.array([0, 0, 0, 1])  # w, x, y, z quaternion
        self.bias = np.zeros(3)  # Gyroscope bias

    def update(self, gyro, accel, dt):
        # Normalize accelerometer
        accel_norm = accel / np.linalg.norm(accel)

        # Convert accelerometer to roll/pitch (assuming no external acceleration)
        roll_acc = np.arctan2(accel_norm[1], accel_norm[2])
        pitch_acc = np.arctan2(-accel_norm[0], np.sqrt(accel_norm[1]**2 + accel_norm[2]**2))

        # Integrate gyroscope data
        gyro_corrected = gyro - self.bias
        dt_quat = self.quat_multiply([0, *gyro_corrected], self.orientation) * 0.5 * dt
        orientation_from_gyro = self.orientation + dt_quat

        # Convert accelerometer data to quaternion
        orientation_from_accel = self.euler_to_quat([roll_acc, pitch_acc, 0])

        # Complementary filter
        self.orientation = self.normalize_quat(
            self.alpha * (self.orientation + dt_quat) +
            (1 - self.alpha) * orientation_from_accel
        )

        return self.orientation

    def quat_multiply(self, q1, q2):
        # Quaternion multiplication
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return np.array([w, x, y, z])

    def normalize_quat(self, q):
        return q / np.linalg.norm(q)

    def euler_to_quat(self, euler):
        roll, pitch, yaw = euler
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return np.array([w, x, y, z])
```

## Performance Considerations

### Computational Complexity

IMU simulation is generally lightweight but can have cumulative effects:

- **Update rate**: Higher rates require more computation
- **Noise generation**: Random number generation can be expensive
- **Integration**: Sensor fusion algorithms add computational load
- **Communication**: High-frequency ROS message publishing

### Optimization Strategies

- **Efficient noise generation**: Use fast random number generators
- **Batch processing**: Process multiple samples at once when possible
- **Approximation**: Use simplified models when full accuracy isn't needed

## Quality Validation

### Accuracy Metrics

Validate simulated IMU data against real sensors:

- **Static accuracy**: Error when sensor is stationary
- **Dynamic accuracy**: Error during motion
- **Bias stability**: Long-term drift characteristics
- **Scale factor**: Accuracy of measurement scaling

### Calibration Validation

- **Intrinsic calibration**: Sensor-specific parameters
- **Extrinsic calibration**: Sensor placement and orientation
- **Temperature effects**: Performance across operating range

## Best Practices

### Realistic Simulation

- Use noise models that match your real sensor specifications
- Validate simulation results against real sensor data when possible
- Consider environmental factors that affect IMU performance
- Match update rates to real sensor capabilities

### Integration with Control

- Ensure IMU data is synchronized with control loops
- Account for sensor latency in control algorithms
- Implement proper sensor fusion for state estimation

## Key Takeaways

- IMU simulation requires accurate modeling of noise and bias characteristics
- Proper integration with physics simulation ensures realistic measurements
- ROS 2 integration enables use of standard state estimation algorithms
- Sensor fusion techniques can improve overall system performance
- Validation against real sensors ensures simulation quality